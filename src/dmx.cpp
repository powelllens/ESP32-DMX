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

#include <dmx.h>

#define DEBUG_PIN_6 19
#define DEBUG_PIN_7 18

namespace dmxserial
{

    TaskHandle_t DMX::uart_send_task_h;
    TaskHandle_t DMX::uart_event_task_h;
    QueueHandle_t DMX::dmx_rx_queue;
    SemaphoreHandle_t DMX::sync_serial_dmx;
    DMXState DMX::dmx_state = DMX_IDLE;
    uint16_t DMX::current_rx_addr = 0;
    long DMX::last_dmx_packet = 0;
    long DMX::last_dmx_packet_time = 0;
    uint8_t DMX::dmx_data[513];
    unsigned long DMX::_dmxserialFramecount = 0;
    bool DMX::enableoutput = true;
    bool DMX::newdataflag = false;

    DMX::DMX() {}

    void DMX::Initialize()
    {
        pinMode(DEBUG_LED, OUTPUT);
        digitalWrite(DEBUG_LED, 0);
        // configure UART for DMX
        uart_config_t uart_config =
            {
                .baud_rate = 250000,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_2,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

        uart_param_config(DMX_UART_NUM, &uart_config);

        // Set pins for UART
        uart_set_pin(DMX_UART_NUM, DMX_SERIAL_OUTPUT_PIN, DMX_SERIAL_INPUT_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        // install queue
        uart_driver_install(DMX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &dmx_rx_queue, 0);

        // create mutex for syncronisation
        sync_serial_dmx = xSemaphoreCreateMutex();

        // set gpio for direction
        gpio_pad_select_gpio(DMX_SERIAL_IO_PIN);
        gpio_set_direction(DMX_SERIAL_IO_PIN, GPIO_MODE_OUTPUT);
    }

    void DMX::setComdir(DMXDirection direction)
    {
        // depending on parameter set gpio for direction change and start rx or tx thread
        if (uart_send_task_h != NULL)
        {
            vTaskDelete(uart_send_task_h);
            delay(500);
        }

        if (uart_event_task_h != NULL)
        {
            vTaskDelete(uart_event_task_h);
            delay(500);
            enableoutput = true;
        }

        if (direction == output)
        {
            gpio_set_level(DMX_SERIAL_IO_PIN, 1);
            dmx_state = DMX_OUTPUT;

            // create send task
            xTaskCreatePinnedToCore(uart_send_task, "uart_send_task", 1024, NULL, 2, &uart_send_task_h, DMX_CORE);
        }
        else
        {
            gpio_set_level(DMX_SERIAL_IO_PIN, 0);
            dmx_state = DMX_IDLE;

            // create receive task
            xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 2048, NULL, 1, &uart_event_task_h, DMX_CORE);
        }
    }

    uint8_t DMX::Read(uint16_t channel)
    {
        // restrict acces to dmx array to valid values
        if (channel < 1 || channel > 512)
        {
            return 0;
        }

        // take data threadsafe from array and return
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        uint8_t tmp_dmx = dmx_data[channel];
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
        return tmp_dmx;
    }

    void DMX::ReadAll(uint8_t *data, uint16_t start, size_t size)
    {
        // restrict acces to dmx array to valid values
        if (start < 1 || start > 512 || start + size > 513)
        {
            return;
        }
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        memcpy(data, (uint8_t *)dmx_data + start, size);
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
    }

    void DMX::Write(uint16_t channel, uint8_t value)
    {
        // restrict acces to dmx array to valid values
        if (channel < 1 || channel > 512)
        {
            return;
        }

#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        dmx_data[channel] = value;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
    }

    void DMX::WriteAll(uint8_t *data, uint16_t start, size_t size)
    {
        // restrict acces to dmx array to valid values
        if (start < 1 || start > 512 || start + size > 513)
        {
            return;
        }
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        memcpy((uint8_t *)dmx_data + start, data, size);
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
    }

    uint8_t DMX::IsHealthy()
    {
        // get timestamp of last received packet
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        long dmx_timeout = last_dmx_packet;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
        // check if elapsed time < defined timeout
        if (xTaskGetTickCount() - dmx_timeout < HEALTHY_TIME)
        {
            return 1;
        }
        return 0;
    }

    unsigned long DMX::getLastUpdateTime()
    {
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        unsigned long dmx_time = last_dmx_packet_time;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
        return dmx_time;
    }

    unsigned long DMX::getFramecount()
    {
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        unsigned long framecount = _dmxserialFramecount;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
        return framecount;
    }

    void DMX::enableOutput(bool enable)
    {
        /*
        if (uart_send_task_h == NULL)
        {
            return;
        }
        eTaskState state = eTaskGetState(uart_send_task_h);
        */
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        enableoutput = enable;
        /*
        if (enableoutput && (state == eSuspended))
        {
            vTaskResume(uart_send_task_h);
        }
        if(!enableoutput && (state != eSuspended))
        {
            vTaskSuspend(uart_send_task_h);
        }
        */
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
    }

    void DMX::resetFramecount()
    {
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        _dmxserialFramecount = 0;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
    };

    bool DMX::getNewData()
    {
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
        bool tmpdataflag = newdataflag;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_serial_dmx);
#endif
        if (tmpdataflag)
        {
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
            xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
            newdataflag = false;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
            xSemaphoreGive(sync_serial_dmx);
#endif
            return true;
        }
        return false;
    }

    void DMX::uart_send_task(void *pvParameters)
    {
        enableOutput(false);
        uint8_t start_code = 0x00;
        bool running = false;
        for (;;)
        {
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
            xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
            // running = true;
            running = enableoutput;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
            xSemaphoreGive(sync_serial_dmx);
#endif
            if (running) // enableoutput)
            {
                // digitalWrite(DEBUG_PIN_6,!digitalRead(DEBUG_PIN_6));
                //  wait till uart is ready
                uart_wait_tx_done(DMX_UART_NUM, 1000);
                // set line to inverse, creates break signal
                uart_set_line_inverse(DMX_UART_NUM, UART_SIGNAL_TXD_INV);
                // wait break time
                ets_delay_us(184);
                // disable break signal
                uart_set_line_inverse(DMX_UART_NUM, 0);
                // wait mark after break
                ets_delay_us(24);
                // write start code
                uart_write_bytes(DMX_UART_NUM, (const char *)&start_code, 1);
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
                xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
                // transmit the dmx data
                uart_write_bytes(DMX_UART_NUM, (const char *)dmx_data + 1, 512);
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
                xSemaphoreGive(sync_serial_dmx);
#endif
            }
            else
            {
                // digitalWrite(DEBUG_LED, 1);
                delay(1);
            }
            digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
        }
    }

    void DMX::uart_event_task(void *pvParameters)
    {
        uint8_t dmx_data_old[513]; // stores the received dmx data
        uart_event_t event;
        uint8_t *dtmp = (uint8_t *)malloc(BUF_SIZE);
        for (;;)
        {
            //  wait for data in the dmx_queue
            if (xQueueReceive(dmx_rx_queue, (void *)&event, (portTickType)portMAX_DELAY))
            {
                bzero(dtmp, BUF_SIZE);
                switch (event.type)
                {
                case UART_DATA:
                    // read the received data
                    uart_read_bytes(DMX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    // check if break detected
                    if (dmx_state == DMX_BREAK)
                    {
                        digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
                        // if not 0, then RDM or custom protocol
                        if (dtmp[0] == 0)
                        {
                            dmx_state = DMX_DATA;
                            // reset dmx adress to 0
                            current_rx_addr = 0;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
                            xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
                            last_dmx_packet_time = xTaskGetTickCount() - last_dmx_packet;
                            // store received timestamp
                            last_dmx_packet = xTaskGetTickCount();
                            ++_dmxserialFramecount;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
                            xSemaphoreGive(sync_serial_dmx);
#endif
                        }
                    }
                    // check if in data receive mode
                    if (dmx_state == DMX_DATA)
                    {

#ifndef DMXSERIAL_IGNORE_THREADSAFETY
                        xSemaphoreTake(sync_serial_dmx, portMAX_DELAY);
#endif
                        // copy received bytes to dmx data array
                        for (int i = 0; i < event.size; i++)
                        {
                            if (current_rx_addr < 513)
                            {
                                if (dtmp[i] == 0)
                                {
                                    if (dmx_data_old[current_rx_addr] == dtmp[i])
                                    {
                                        dmx_data[current_rx_addr] = dtmp[i];
                                    }
                                }
                                else
                                {
                                    dmx_data[current_rx_addr] = dtmp[i];
                                }
                                dmx_data_old[current_rx_addr++] = dtmp[i];
                            }
                        }
                        newdataflag = true;
#ifndef DMXSERIAL_IGNORE_THREADSAFETY
                        xSemaphoreGive(sync_serial_dmx);
#endif
                    }
                    break;
                case UART_BREAK:
                    // break detected
                    // clear queue und flush received bytes
                    uart_flush_input(DMX_UART_NUM);
                    xQueueReset(dmx_rx_queue);
                    dmx_state = DMX_BREAK;
                    break;
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                case UART_BUFFER_FULL:
                case UART_FIFO_OVF:
                default:
                    // error recevied, going to idle mode
                    uart_flush_input(DMX_UART_NUM);
                    xQueueReset(dmx_rx_queue);
                    dmx_state = DMX_IDLE;
                    break;
                }
            }
        }
    }
}