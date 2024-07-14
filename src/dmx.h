/*
 * This file is part of the ESP32-DMX distribution (https://github.com/luksal/ESP32-DMX).
 * Copyright (c) 2021 Lukas Salomon.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#ifdef DISABLE_DEBUG_TO_SERIAL
#define DMX_SERIAL_INPUT_PIN GPIO_NUM_3  // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN GPIO_NUM_1 // pin for dmx tx
#define DMX_SERIAL_IO_PIN GPIO_NUM_23    // pin for dmx rx/tx change

#define DMX_UART_NUM UART_NUM_0 // dmx uart
#else
#define DMX_SERIAL_INPUT_PIN GPIO_NUM_9   // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN GPIO_NUM_10 // pin for dmx tx
#define DMX_SERIAL_IO_PIN GPIO_NUM_23     // pin for dmx rx/tx change

#define DMX_UART_NUM UART_NUM_1 // dmx uart
#endif
#define HEALTHY_TIME 500                // timeout in ms
#define BUF_SIZE 1024                   //  buffer size for rx events
#define DMX_CORE 1                      // select the core the rx/tx thread should run on
#define DMXSERIAL_IGNORE_THREADSAFETY 0 // set to 1 to disable all threadsafe mechanisms

#ifndef DMX_h
#define DMX_h

#define DEBUG_LED 2

namespace dmxserial
{

    enum DMXDirection
    {
        input,
        output
    };
    enum DMXState
    {
        DMX_IDLE,
        DMX_BREAK,
        DMX_DATA,
        DMX_OUTPUT
    };

    class DMX
    {
    public:
        static void Initialize(); // initialize library

        static uint8_t Read(uint16_t channel); // returns the dmx value for the givven address (values from 1 to 512)

        static void ReadAll(uint8_t *data, uint16_t start, size_t size); // copies the defined channels from the read buffer

        static void Write(uint16_t channel, uint8_t value); // writes the dmx value to the buffer

        static void WriteAll(uint8_t *data, uint16_t start, size_t size); // copies the defined channels into the write buffer

        static uint8_t IsHealthy(); // returns true, when a valid DMX signal was received within the last 500ms

        static bool getNewData();

        static unsigned long getLastUpdateTime();
        static unsigned long getFramecount();
        static void resetFramecount();
        void setComdir(DMXDirection direction);

        static void enableOutput(bool enable);

    private:
        DMX(); // hide constructor

        static bool newdataflag;

        static bool enableoutput;

        static QueueHandle_t dmx_rx_queue; // queue for uart rx events

        static SemaphoreHandle_t sync_serial_dmx; // semaphore for syncronising access to dmx array

        static DMXState dmx_state; // status, in which recevied state we are

        static uint16_t current_rx_addr; // last received dmx channel

        static long last_dmx_packet; // timestamp for the last received packet
        static long last_dmx_packet_time;
        static unsigned long _dmxserialFramecount;

        static uint8_t dmx_data[513]; // stores the received dmx data

        static void uart_event_task(void *pvParameters); // event task
        static void uart_send_task(void *pvParameters);  // transmit task

        static TaskHandle_t uart_send_task_h;
        static TaskHandle_t uart_event_task_h;
    };
}

#endif