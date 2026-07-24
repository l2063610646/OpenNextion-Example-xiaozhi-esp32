#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "mcp_server.h"
#include "uart_snapshot_service.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#define TAG "OpenNextion_ONX3248G035"

class PCF8574 {
protected:
    i2c_master_dev_handle_t i2c_device_;

public:
    PCF8574(i2c_master_bus_handle_t i2c_bus, uint8_t addr)  {
         i2c_device_config_t i2c_device_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 50 * 1000,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = 0,
            },
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &i2c_device_cfg, &i2c_device_));
        assert(i2c_device_ != NULL);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        ESP_ERROR_CHECK(!(bit < 8));
        uint8_t raw;
        ESP_ERROR_CHECK(i2c_master_receive(i2c_device_, &raw, 1, 100));
        raw = (raw & ~(1 << bit)) | (level << bit);
        ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, &raw, 1, 100));
    }

    uint8_t GetInputState(uint8_t bit) {
        ESP_ERROR_CHECK(!(bit < 8));
        uint8_t raw;
        ESP_ERROR_CHECK(i2c_master_receive(i2c_device_, &raw, 1, 100));
        return (raw >> bit) & 0x01;
    }

    void WriteReg(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, buffer, 2, 100));
    }

    uint8_t ReadReg(uint8_t reg) {
        uint8_t buffer[1];
        ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, 1, 100));
        return buffer[0];
    }

    void ReadRegs(uint8_t reg, uint8_t* buffer, size_t length) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, length, 100));
    }
};

class CST826 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    enum TouchEvent {
        TOUCH_NONE,
        TOUCH_PRESS,
        TOUCH_RELEASE,
        TOUCH_HOLD
    };

    CST826(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[6];
        was_touched_ = false;
        press_count_ = 0;
    }

    ~CST826()
    {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint()
    {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t &GetTouchPoint()
    {
        return tp_;
    }

    TouchEvent CheckTouchEvent()
    {
        bool is_touched = (tp_.num > 0);
        TouchEvent event = TOUCH_NONE;

        if (is_touched && !was_touched_) {
            // Press event (transition from not touched to touched)
            press_count_++;
            event = TOUCH_PRESS;
            ESP_LOGI(TAG, "TOUCH PRESS - count: %d, x: %d, y: %d", press_count_, tp_.x, tp_.y);
        } else if (!is_touched && was_touched_) {
            // Release event (transition from touched to not touched)
            event = TOUCH_RELEASE;
            ESP_LOGI(TAG, "TOUCH RELEASE - total presses: %d", press_count_);
        } else if (is_touched && was_touched_) {
            // Continuous touch (hold)
            event = TOUCH_HOLD;
            ESP_LOGD(TAG, "TOUCH HOLD - x: %d, y: %d", tp_.x, tp_.y);
        }

        // Update previous state
        was_touched_ = is_touched;
        return event;
    }

    int GetPressCount() const
    {
        return press_count_;
    }

    void ResetPressCount()
    {
        press_count_ = 0;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;

    // Touch state tracking
    bool was_touched_;
    int press_count_;
};

//Single Speaker: Audio Power Amplifier NS4168
//Dual Microphones: PDM MICs
class ONX_NoAudioCodecDuplex : public NoAudioCodec {
public:
    PCF8574* pcf8574_;  // PCF8574 I/O Expander

    ONX_NoAudioCodecDuplex(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, gpio_num_t mic_sck, gpio_num_t mic_din, PCF8574* pcf8574) {
        duplex_ = true;
        input_reference_ = true;    // Whether to use reference input to enable echo cancellation.
        input_channels_ = 2;        // Number of Input Channels
        input_sample_rate_ = input_sample_rate;
        output_sample_rate_ = output_sample_rate;
        pcf8574_ = pcf8574;

        // Set NS4168 audio power amplifier shutdown pin, initial state is shutdown to prevent background noise and power-on pop sound
        pcf8574_->SetOutputState(PCF8574_I2S_CTRL_BIT,0);
    
        i2s_chan_config_t chan_cfg = {
            .id = I2S_NUM_1,
            .role = I2S_ROLE_MASTER,
            .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
            .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
            .auto_clear_after_cb = true,
            .auto_clear_before_cb = false,
            .intr_priority = 0,
        };
        ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, NULL));
    
        i2s_std_config_t std_cfg = {
            .clk_cfg = {
                .sample_rate_hz = (uint32_t)output_sample_rate_,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
                #ifdef   I2S_HW_VERSION_2
                    .ext_clk_freq_hz = 0,
                #endif
            },
            .slot_cfg = {
                .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode = I2S_SLOT_MODE_STEREO,
                .slot_mask = I2S_STD_SLOT_BOTH,
                .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
                .ws_pol = false,
                .bit_shift = true,
                #ifdef   I2S_HW_VERSION_2
                    .left_align = true,
                    .big_endian = false,
                    .bit_order_lsb = false
                #endif
            },
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = spk_bclk,
                .ws = spk_ws,
                .dout = spk_dout,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false
                }
            }
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));

        // Create a new channel for MIC in PDM mode
        i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)0, I2S_ROLE_MASTER);
        rx_chan_cfg.auto_clear = true;
        ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle_));
        i2s_pdm_rx_config_t pdm_rx_cfg = {
            .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG((uint32_t)input_sample_rate_),
            /* The data bit-width of PDM mode is fixed to 16 */
            .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .clk = mic_sck,
                .din = mic_din,

                .invert_flags = {
                    .clk_inv = false,
                },
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle_, &pdm_rx_cfg));

        ESP_LOGI(TAG, "Duplex channels created");
    }

    int Read(int16_t* dest, int samples) {
        size_t bytes_read;

        // PDM demodulated data bit width is 16 bits, directly read into the target buffer
        if (i2s_channel_read(rx_handle_, dest, samples * sizeof(int16_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
            ESP_LOGE(TAG, "Read Failed!");
            return 0;
        }

        samples = bytes_read / sizeof(int16_t);
        if (input_gain_ > 0) {
            int gain_factor = (int)input_gain_;
            for (int i = 0; i < samples; i++) {
                int32_t amplified = dest[i] * gain_factor;
                dest[i] = (amplified > INT16_MAX) ? INT16_MAX : (amplified < -INT16_MAX) ? -INT16_MAX : (int16_t)amplified;
            }
        }
        return samples;
    }

    void EnableOutput(bool enable) {
        if (enable == output_enabled_) {
            return;
        }

        // Set NS4168 audio power amplifier shutdown pin, initial state is shutdown to prevent background noise and power-on pop sound
        if (enable) {
            pcf8574_->SetOutputState(PCF8574_I2S_CTRL_BIT,1);
        } else {
            pcf8574_->SetOutputState(PCF8574_I2S_CTRL_BIT,0);
        }
        NoAudioCodec::EnableOutput(enable);
    }
};

class OpenNextion_ONX3248G035 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;
    PCF8574* pcf8574_;
    CST826* cst826_;
    UartSnapshotService* uart_snapshot_service_ = nullptr;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = AUDIO_I2C_SDA_PIN,
            .scl_io_num = AUDIO_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
        ESP_LOGI(TAG, "i2c_new_master_bus success");

        // Initialize PCF8574 I/O expander
        pcf8574_ = new PCF8574(i2c_bus_, PCF8574_ADDR);
    }

    // Initialize spi peripheral
    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        ESP_LOGD(TAG, "Install panel IO");
        // LCD panel IO initialization
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io);

        // Initialize LCD driver chip ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel);
        
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY); 
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    // MCP Tools Initialization
    void InitializeTools() {
        // Refer to MCP documentation
    }

    static void touch_event_task(void* arg)
    {
        CST826* touchpad = static_cast<CST826*>(arg);
        if (touchpad == nullptr) {
            ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
            vTaskDelete(NULL);
            return;
        }

        while (true) {
            touchpad->UpdateTouchPoint();
            auto touch_event = touchpad->CheckTouchEvent();

            if (touch_event == CST826::TOUCH_RELEASE) {
                auto &app = Application::GetInstance();
                auto &board = (OpenNextion_ONX3248G035 &)Board::GetInstance();

                if (app.GetDeviceState() == kDeviceStateStarting) {
                    // board.EnterWifiConfigMode();
                } else if(app.GetDeviceState() != kDeviceStateWifiConfiguring) {
                    app.ToggleChatState();
                    // board.EnterWifiConfigMode();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50ms
        }
    }

    void InitializeCST826TouchPad()
    {
        cst826_ = new CST826(i2c_bus_, 0x15);
        xTaskCreate(touch_event_task, "touch_task", 2 * 1024, cst826_, 5, NULL);
    }

public:
    OpenNextion_ONX3248G035() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        uart_snapshot_service_ = new UartSnapshotService(display_, SNAPSHOT_UART_RX_PIN, SNAPSHOT_UART_TX_PIN);
        InitializeButtons();
        InitializeTools();
        GetBacklight()->SetBrightness(100);
        InitializeCST826TouchPad();
    }
    
    virtual AudioCodec* GetAudioCodec() override {
        static ONX_NoAudioCodecDuplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_WS,
            AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_DIN,
            pcf8574_);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    // Get Backlight Control
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }
};

DECLARE_BOARD(OpenNextion_ONX3248G035);
