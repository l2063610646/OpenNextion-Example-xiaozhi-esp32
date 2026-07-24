#include "uart_snapshot_service.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>
#include <string>
#include <vector>

#include "display/lvgl_display/lvgl_display.h"

namespace {
constexpr uart_port_t kUart = UART_NUM_1;
constexpr int kBaudRate = 460800;
constexpr char kTag[] = "UartSnapshot";

struct __attribute__((packed)) SnapshotHeader {
    char magic[4];              // "ONXS"
    uint8_t version;            // 1
    uint8_t pixel_format;       // 2 = RGB565 big-endian
    uint16_t header_size;
    uint16_t width;
    uint16_t height;
    uint32_t data_size;
    uint32_t crc32;
};
static_assert(sizeof(SnapshotHeader) == 20, "Unexpected UART snapshot header size");

bool IsSaveCommand(const std::string& value) {
    size_t first = value.find_first_not_of(" \t\r\n");
    return first != std::string::npos && value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1) == "save";
}
}  // namespace

UartSnapshotService::UartSnapshotService(LvglDisplay* display, gpio_num_t rx_pin, gpio_num_t tx_pin)
    : display_(display), rx_pin_(rx_pin), tx_pin_(tx_pin) {
    uart_config_t config = {
        .baud_rate = kBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(kUart, 256, 4096, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(kUart, &config));
    ESP_ERROR_CHECK(uart_set_pin(kUart, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(TaskEntry, "uart_snapshot", 6144, this, 4, nullptr);
    ESP_LOGI(kTag, "UART1 snapshot: RX=GPIO%d, TX=GPIO%d, %d baud", rx_pin_, tx_pin_, kBaudRate);
}

void UartSnapshotService::TaskEntry(void* arg) {
    static_cast<UartSnapshotService*>(arg)->Run();
}

void UartSnapshotService::Run() {
    std::string command;
    uint8_t byte;
    while (true) {
        if (uart_read_bytes(kUart, &byte, 1, portMAX_DELAY) != 1) continue;
        if (byte == '\n') {
            if (IsSaveCommand(command)) SendSnapshot();
            command.clear();
        } else if (command.size() < 64) {
            command.push_back(static_cast<char>(byte));
        } else {
            command.clear();
        }
    }
}

void UartSnapshotService::SendSnapshot() {
    std::vector<uint8_t> pixels;
    uint16_t width = 0;
    uint16_t height = 0;
    if (display_ == nullptr || !display_->SnapshotToRgb565(pixels, width, height)) {
        ESP_LOGE(kTag, "Snapshot capture failed");
        return;
    }
    SnapshotHeader header{};
    std::memcpy(header.magic, "ONXS", sizeof(header.magic));
    header.version = 1;
    header.pixel_format = 2;
    header.header_size = sizeof(SnapshotHeader);
    header.width = width;
    header.height = height;
    header.data_size = static_cast<uint32_t>(pixels.size());
    header.crc32 = CalculateCrc32(pixels.data(), pixels.size());
    if (!WriteAll(&header, sizeof(header)) || !WriteAll(pixels.data(), pixels.size())) {
        ESP_LOGE(kTag, "Failed to send snapshot");
        return;
    }
    if (uart_wait_tx_done(kUart, pdMS_TO_TICKS(10000)) != ESP_OK) ESP_LOGW(kTag, "Snapshot transmission timed out");
    ESP_LOGI(kTag, "Sent %ux%u snapshot (%u bytes)", width, height, static_cast<unsigned>(pixels.size()));
}

bool UartSnapshotService::WriteAll(const void* data, size_t size) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t offset = 0; offset < size;) {
        size_t chunk = size - offset > 1024 ? 1024 : size - offset;
        int written = uart_write_bytes(kUart, reinterpret_cast<const char*>(bytes + offset), chunk);
        if (written <= 0) return false;
        offset += static_cast<size_t>(written);
    }
    return true;
}

uint32_t UartSnapshotService::CalculateCrc32(const uint8_t* data, size_t size) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0U);
    }
    return ~crc;
}
