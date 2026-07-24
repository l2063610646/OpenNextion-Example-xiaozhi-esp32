#ifndef UART_SNAPSHOT_SERVICE_H
#define UART_SNAPSHOT_SERVICE_H

#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>

class LvglDisplay;

class UartSnapshotService {
public:
    UartSnapshotService(LvglDisplay* display, gpio_num_t rx_pin, gpio_num_t tx_pin);

private:
    static void TaskEntry(void* arg);
    void Run();
    void SendSnapshot();
    bool WriteAll(const void* data, size_t size);
    static uint32_t CalculateCrc32(const uint8_t* data, size_t size);

    LvglDisplay* display_;
    gpio_num_t rx_pin_;
    gpio_num_t tx_pin_;
};

#endif
