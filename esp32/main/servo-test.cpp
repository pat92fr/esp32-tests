#include "servo_cmd.h"
#include "imu_cmd.h"
#include "uart_server.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "esp_system.h"
#include "cmd_system.h"
#include "cmd_wifi.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/i2c.h"

// Pat92fr
#include "esp_timer.h"
#include "driver/uart.h"
#include "mini_pupper_servos.h"
extern SERVO servo;
// Pat92fr

static const char* TAG = "servo_tests";

#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_SDA_IO 41
#define I2C_MASTER_SCL_IO 42


#if CONFIG_CONSOLE_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .format_if_mount_failed = true,
            .max_files = 4,
	    .allocation_unit_size = 1024,
	    .disk_status_check_enable = false
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_STORE_HISTORY

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

extern "C" void app_main(void)
{

    /* start i2c bus */
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    i2c_param_config(I2C_NUM_0, &conf);

    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "muni_pupper>";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    initialize_nvs();

#if CONFIG_CONSOLE_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif

    /* Register commands */
    esp_console_register_help_command();
#if CONFIG_RASPI_CONTROLLED
    /* start UART server for Raspberry Pi communication */
    UARTServer uart_server;
#else
    register_servo_cmds();
    register_imu_cmds();
#endif
    register_system();
    register_wifi();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif


    //ESP_ERROR_CHECK(esp_console_start_repl(repl));



    // Pat92fr
    // Pat92fr
    // Pat92fr

    servo.enable();
    servo.enableTorque();

    // PERFTEST #1 : one servo timing
    // Results setPosition    : Time: 844us < 880us < 1075us
    // Results setPositionFast: Time: 716us < 751us < 941us
    if(false)
    {
        u16 position = 600;
        uint64_t min_timing = 1000000;
        uint64_t max_timing = 0;
        float mean_timing = 0.0f;

        for(;;)
        {
            //static char const * test_str = "This is a test string.\n";
            //uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, (const char*)test_str, strlen(test_str));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            uint64_t start_time = esp_timer_get_time();
            servo.setPosition(2,position);
            //servo.setPositionFast(2,position);
            
            uint64_t end_time = esp_timer_get_time();
            uint64_t  timing = end_time-start_time;
            min_timing = timing < min_timing ? timing : min_timing;
            max_timing = timing > max_timing ? timing : max_timing;
            mean_timing = 0.1f * timing + 0.9f * mean_timing;
            //ESP_LOGI(TAG, "Time: %llu microseconds", end_time-start_time);       
            static char log_str[80];
            sprintf(log_str, "Time: %lluus < %.0fus < %lluus\r\n",min_timing,mean_timing,max_timing);       
            uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, (const char*)log_str, strlen(log_str));
        }
    }

    // PERFTEST #1 : one servo timing
    // Results setPosition12    : Time: 844us < 880us < 1075us
    if(true)
    {
        uint64_t min_timing = 1000000;
        uint64_t max_timing = 0;
        float mean_timing = 0.0f;
        u8 servoIDs[] {1,2,3,4,5,6,7,8,9,10,11,12};
        u16 position = 600;
        u16 servoPositions[] {position,position,position,position,position,position,position,position,position,position,position,position};

        for(;;)
        {
            //static char const * test_str = "This is a test string.\n";
            //uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, (const char*)test_str, strlen(test_str));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            uint64_t start_time = esp_timer_get_time();
            servo.setPosition12(servoIDs,servoPositions);
            
            uint64_t end_time = esp_timer_get_time();
            uint64_t  timing = end_time-start_time;
            min_timing = timing < min_timing ? timing : min_timing;
            max_timing = timing > max_timing ? timing : max_timing;
            mean_timing = 0.1f * timing + 0.9f * mean_timing;
            //ESP_LOGI(TAG, "Time: %llu microseconds", end_time-start_time);       
            static char log_str[80];
            sprintf(log_str, "Time: %lluus < %.0fus < %lluus\r\n",min_timing,mean_timing,max_timing);       
            uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, (const char*)log_str, strlen(log_str));
        }
    }








    // TODO : test send pos+vel timing at 500kbps, with and w/o ring buffer
    // TODO : test send pos+vel and rcv feedback timing at 500kbps, with and w/o ring buffer
    // TODO : test send pos+vel timing at 1Mbps.
    // TODO : test send pos+vel and rcv feedback timing at 1Mbps.
    // TODO : communication about performances

    // TODO desactiver return ACK

    // TODO : timing of a sequence of 12 set pos+vel
    // TODO : timing of a sequence of 12 set pos+vel and feedback
    // TODO : communication about frequency (>200Hz)

    // TODO : rendre configurable la priode de récupération du feedback


    // Pat92fr
    // Pat92fr
    // Pat92fr


}
