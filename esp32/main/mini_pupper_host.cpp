/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

static const char* TAG = "HOST";

#include "mini_pupper_protocol.h"
#include "mini_pupper_host.h"
#include "mini_pupper_servos.h"
#include "mini_pupper_imu.h"
#include "mini_pupper_power.h"
#include "mini_pupper_taskes.h"

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
_is_service_enabled(false),
_task_handle(NULL),
_uart_queue(NULL)
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
    ESP_ERROR_CHECK(uart_driver_install(_uart_port_num, 1024, 1024, 20, &_uart_queue, 0));
}

static size_t const stack_size = 10000;
static StackType_t stack[stack_size] {0};
static StaticTask_t task_buffer;

void HOST::start()
{
    _task_handle = xTaskCreateStaticPinnedToCore(
        HOST_TASK,   
        "HOST INTERFACE SERVICE",
        stack_size,         
        (void*)this,        
        HOST_PRIORITY,     
        stack,
        &task_buffer,
        HOST_CORE
    );
}

void HOST::enable_service(bool enable)
{
    _is_service_enabled = enable;
}

void HOST_TASK(void * parameters)
{
    HOST * host = reinterpret_cast<HOST*>(parameters);
    uart_event_t event;
    u8 rx_buffer[1024] {0};
    protocol_interpreter_handler protocole_handler;
    for(;;)
    {
        // Waiting for UART event.
        if(xQueueReceive(host->_uart_queue,(void*)&event,(TickType_t)portMAX_DELAY))
        {

            // Waiting for a DATA event
            if(event.type==UART_DATA)
            {
                // log
                ESP_LOGD(TAG, "RX uart event size: %d", event.size);

                // read a frame from host
                int const read_length {uart_read_bytes(host->_uart_port_num,rx_buffer,event.size,portMAX_DELAY)};

                // decode received data
                bool have_to_reply {false};
                for(size_t index=0; index<read_length; ++index)
                {
                    bool const payload = protocol_interpreter(rx_buffer[index],protocole_handler);
                    if(payload)
                    {
                        // waitinf for a INST_CONTROL frame
                        if(protocole_handler.payload_buffer[0]==INST_CONTROL)
                        {
                            // decode parameters
                            parameters_control_instruction_format parameters {0};
                            memcpy(&parameters,&protocole_handler.payload_buffer[1],sizeof(parameters_control_instruction_format));

                            // log
                            ESP_LOGD(TAG, "Goal Position: %d %d %d %d %d %d %d %d %d %d %d %d",
                                parameters.goal_position[0],parameters.goal_position[1],parameters.goal_position[2],
                                parameters.goal_position[3],parameters.goal_position[4],parameters.goal_position[5],
                                parameters.goal_position[6],parameters.goal_position[7],parameters.goal_position[8],
                                parameters.goal_position[9],parameters.goal_position[10],parameters.goal_position[11]
                            );

                            // update servo setpoint only if service is enabled
                            if(host->_is_service_enabled)
                            {
                                //servo.setTorque12Async(parameters.torque_enable);
                                servo.setPosition12Async(parameters.goal_position);
                            }

                            // send have_to_reply
                            have_to_reply = true;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "RX unexpected frame. Instr:%d. Length:%d",protocole_handler.payload_buffer[0],protocole_handler.payload_length);        
                        }
                    }
                }

                // have to reply ?
                if(have_to_reply)
                {
                    // servo feedback
                    parameters_control_acknowledge_format feedback_parameters;
                    servo.getPosition12Async(feedback_parameters.present_position);
                    servo.getLoad12Async(feedback_parameters.present_load);
                    // imu feedback
                    feedback_parameters.roll = imu.get_roll();
                    feedback_parameters.pitch = imu.get_pitch();
                    feedback_parameters.yaw = imu.get_yaw();
                    // power supply feedback
                    feedback_parameters.voltage_V = POWER::get_voltage_V();
                    feedback_parameters.current_A = POWER::get_current_A();

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
            else // Uart event but not a DATA event
            {
                // log
                ESP_LOGI(TAG, "RX uart event type: %d", event.type);
                // next
                continue;
            } 

    /*
            // waiting for a header...
            if(read_length < 4) 
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

            // waiting for a (full) payload...
            if(read_length != (rx_payload_length+4))
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

            // update servo setpoint only if service is enabled
            if(host->_is_service_enabled)
            {
                //servo.setTorque12Async(parameters.torque_enable);
                servo.setPosition12Async(parameters.goal_position);
            }

            // flush RX FIFO
            uart_flush(host->_uart_port_num);  
    */
            

        } // xQueue

    } // for

}
