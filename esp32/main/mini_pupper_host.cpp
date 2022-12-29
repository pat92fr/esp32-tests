/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

static const char* TAG = "HOST";

#include "mini_pupper_protocol.h"
#include "mini_pupper_host.h"
#include "mini_pupper_servos.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define HOST_SERVER_TXD 17
#define HOST_SERVER_RXD 18
#define HOST_SERVER_RTS (UART_PIN_NO_CHANGE)
#define HOST_SERVER_CTS (UART_PIN_NO_CHANGE)

void HOST_TASK(void * parameters);

HOST host;

HOST::HOST() : 
_uart_port_num(2),
_task_handle(NULL)
{
    // set UART port
    uart_config_t uart_config;
    uart_config.baud_rate = 3000000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(_uart_port_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(_uart_port_num, HOST_SERVER_TXD, HOST_SERVER_RXD, HOST_SERVER_RTS, HOST_SERVER_CTS));
    ESP_ERROR_CHECK(uart_driver_install(_uart_port_num, 1024, 1024, 0, NULL, 0));
}

void HOST::start()
{
    xTaskCreate(
        HOST_TASK,                 /* Function that implements the task. */
        "HOST BUS SERVICE",        /* Text name for the task. */
        10000,                      /* Stack size in words, not bytes. */
        (void*)this,                /* Parameter passed into the task. */
        1,           /* Priority at which the task is created. */
        &_task_handle                /* Used to pass out the created task's handle. */
    );
}

void HOST_TASK(void * parameters)
{
    HOST * host = reinterpret_cast<HOST*>(parameters);
    for(;;)
    {
        // delay 1ms
        // - about 1KHz refresh frequency for sync write servo setpoints
        // - about 80Hz refresh frequency for read/ack servo feedbacks
        vTaskDelay(1 / portTICK_PERIOD_MS);


        // wait for a frame from host
        size_t available_length {0};
        uart_get_buffered_data_len(host->_uart_port_num,&available_length);
        if(available_length<4)
            continue;

        // read a frame header from host
        size_t const rx_buffer_size {128};
        u8 rx_buffer[rx_buffer_size] {0};
        // copy RX fifo into local buffer (4 bytes : Header + ID + Length)
        int read_length {uart_read_bytes(host->_uart_port_num,rx_buffer,4,0)}; // timeout = 0

        // waiting for a header...
        if(read_length != 4) 
        {
            // log
            ESP_LOGI(TAG, "RX frame error : truncated header [expected:%d, received:%d]!",4,read_length);
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        // waiting for a valid frame header with my ID...
        bool const rx_header_check { 
                    (rx_buffer[0]==0xFF) 
                &&  (rx_buffer[1]==0xFF) 
                &&  (rx_buffer[2]==0x01) // my ID
                &&  (rx_buffer[3]<=(rx_buffer_size-4)) // keep the header in the rx buffer
        };  
        if(!rx_header_check) 
        {
            // log
            ESP_LOGI(TAG, "RX frame error : header invalid!");
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        // read paylaod length from frame header
        size_t const rx_payload_length {(size_t)rx_buffer[3]};

        // wait for a frame payload from host
        uart_get_buffered_data_len(host->_uart_port_num,&available_length);
        if(available_length<rx_payload_length)
        {
            // wait one ms
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        // copy RX fifo into local buffer (L bytes : Payload + Checksum)
        read_length = uart_read_bytes(host->_uart_port_num,rx_buffer+4,rx_payload_length,0);

        // waiting for a (full) payload...
        if(read_length != rx_payload_length) 
        {
            // log
            ESP_LOGI(TAG, "RX frame error : truncated payload [expected:%d, received:%d]!",rx_payload_length,read_length);
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        // checksum
        u8 expected_checksum {0};
        bool const rx_checksum {checksum(rx_buffer,expected_checksum)};

        // waiting for a valid instruction and checksum...
        bool const rx_payload_checksum_check { 
                    (rx_buffer[4]==INST_CONTROL) 
                &&  rx_checksum
        };  
        if(!rx_payload_checksum_check) 
        {
            // log
            ESP_LOGI(TAG, "RX frame error : bad instruction [%d] or checksum [received:%d,expected:%d]!",rx_buffer[4],rx_buffer[rx_payload_length+4-1],expected_checksum);
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        // decode parameters
        parameters_control_instruction_format parameters {0};
        memcpy(&parameters,rx_buffer+5,sizeof(parameters_control_instruction_format));

        // log
        ESP_LOGD(TAG, "Goal Position: %d %d %d %d %d %d %d %d %d %d %d %d",
            parameters.goal_position[0],parameters.goal_position[1],parameters.goal_position[2],
            parameters.goal_position[3],parameters.goal_position[4],parameters.goal_position[5],
            parameters.goal_position[6],parameters.goal_position[7],parameters.goal_position[8],
            parameters.goal_position[9],parameters.goal_position[10],parameters.goal_position[11]
        );

        // control servo
        servo.enable();
        servo.setPosition12Async(parameters.goal_position);

        // flush RX FIFO
        uart_flush(host->_uart_port_num);  

        // servo feedback
        parameters_control_acknowledge_format feedback_parameters;
        servo.getPosition12Async(feedback_parameters.present_position);
        servo.getLoad12Async(feedback_parameters.present_load);

        // build acknowledge frame
        static size_t const tx_payload_length {1+sizeof(parameters_control_acknowledge_format)+1};            
        static size_t const tx_buffer_size {4+tx_payload_length};            
        u8 tx_buffer[tx_buffer_size] {
            0xFF,                                       // Start of Frame
            0xFF,                                       // Start of Frame
            0x01,                                       // ID
            tx_payload_length,                          // Length
            0x00,                                       // Status
        };
        memcpy(tx_buffer+5,&feedback_parameters,sizeof(parameters_control_acknowledge_format));

        // compute checksum
        tx_buffer[tx_buffer_size-1] = compute_checksum(tx_buffer);

        // send frame to host
        uart_write_bytes(host->_uart_port_num,tx_buffer,tx_buffer_size);

        // Wait for packet to be sent
        //ESP_ERROR_CHECK(uart_wait_tx_done(host->_uart_port_num, 10)); // wait timeout is 10 RTOS ticks (TickType_t)

    }    
}