#include "pwm_task.h"
#include <Arduino.h>
#include <Adafruit_LSM9DS0.h>

#include "driver/rmt.h"
#include "esp_log.h"

// RMT Configuration
#define RMT_TX_CHANNEL    RMT_CHANNEL_0
#define RMT_TX_GPIO_NUM   GPIO_NUM_3
#define RMT_CLK_DIV       80    // 80MHz / 80 = 1MHz (1μs resolution)
#define RMT_TICK_1_US     1     // 1 tick = 1μs

// Servo Configuration
#define SERVO_MIN_PULSE_US    1000
#define SERVO_MAX_PULSE_US    2000
#define SERVO_PERIOD_US       20000

// RMT data structure for servo signal
static rmt_item32_t servo_items[2];

void example_rmt_init(void)
{
    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO_NUM, RMT_TX_CHANNEL);
    rmt_tx_config.clk_div = RMT_CLK_DIV;
    rmt_tx_config.mem_block_num = 1;
    rmt_tx_config.tx_config.loop_en = true;
    rmt_tx_config.tx_config.carrier_en = false;
    rmt_tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_tx_config.tx_config.idle_output_en = true;
    
    ESP_ERROR_CHECK(rmt_config(&rmt_tx_config));
    ESP_ERROR_CHECK(rmt_driver_install(RMT_TX_CHANNEL, 0, 0));
}

void rmt_send_servo_pulse(uint32_t pulse_width_us)
{
    // Clamp pulse width to valid servo range
    if (pulse_width_us < SERVO_MIN_PULSE_US) pulse_width_us = SERVO_MIN_PULSE_US;
    if (pulse_width_us > SERVO_MAX_PULSE_US) pulse_width_us = SERVO_MAX_PULSE_US;
    
    // Calculate low period (rest of 20ms period)
    uint32_t low_period_us = SERVO_PERIOD_US - pulse_width_us;
    
    // Item 0: High pulse (servo control signal)
    servo_items[0].level0 = 1;
    servo_items[0].duration0 = pulse_width_us * RMT_TICK_1_US;
    servo_items[0].level1 = 0;
    servo_items[0].duration1 = low_period_us * RMT_TICK_1_US;
    
    // Item 1: End marker
    servo_items[1].level0 = 0;
    servo_items[1].duration0 = 0;
    servo_items[1].level1 = 0;
    servo_items[1].duration1 = 0;
    
    // Send the RMT signal
    ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL, servo_items, 2, false));
}

uint32_t servo_percent_to_pulse(uint8_t percent)
{
    if (percent > 100) percent = 100;
    return SERVO_MIN_PULSE_US + 
           ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * percent) / 100;
}

uint32_t pulse_us_clamp_1_to_1(uint32_t pulse_width_us)
{
    if (pulse_width_us < SERVO_MIN_PULSE_US) pulse_width_us = SERVO_MIN_PULSE_US;
    if (pulse_width_us > SERVO_MAX_PULSE_US) pulse_width_us = SERVO_MAX_PULSE_US;
    return pulse_width_us;
}

int scale_signal_by_half(int input_us)
{
    // Clamp input to valid range
    if (input_us < 1000) input_us = 1000;
    if (input_us > 2000) input_us = 2000;
    
    // Calculate the excess above 1000us
    int excess = input_us - 1000;
    
    // Scale the excess by quarter (was half in original, keeping same logic)
    int output = 1000 + (excess / 6);
    
    // ESP_LOGE("WritePWM", "Scaled signal: input_us=%d, output=%d", input_us, output);
    return output;
}

uint32_t pulse_us_scaled_down(uint32_t pulse_width_us)
{
    pulse_width_us = scale_signal_by_half(pulse_width_us);
    if (pulse_width_us < SERVO_MIN_PULSE_US) pulse_width_us = SERVO_MIN_PULSE_US;
    if (pulse_width_us > SERVO_MAX_PULSE_US) pulse_width_us = SERVO_MAX_PULSE_US;
    return pulse_width_us;
}

// Convenience functions that send the pulse directly
void servo_write_percent(uint8_t percent)
{
    uint32_t pulse_us = servo_percent_to_pulse(percent);
    rmt_send_servo_pulse(pulse_us);
}

void servo_write_pulse_us(uint32_t pulse_us)
{
    pulse_us = pulse_us_clamp_1_to_1(pulse_us);
    rmt_send_servo_pulse(pulse_us);
}

void servo_write_pulse_us_scaled(uint32_t pulse_us)
{
    pulse_us = pulse_us_scaled_down(pulse_us);
    rmt_send_servo_pulse(pulse_us);
}



extern int controlOutput; // Control output from the control task

void pwmTask(void *pvParameters) {

    // Configure the LEDC (PWM) channel
    example_rmt_init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    Serial.println("RMT Servo control initialized");

    rmt_send_servo_pulse(1000);

    vTaskDelay(pdMS_TO_TICKS(5000));

    while (true) {
        int localControlOutput = 0;
        
        // Protect controlOutput access with mutex
        if (xSemaphoreTake(controlOutputMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localControlOutput = controlOutput;
            xSemaphoreGive(controlOutputMutex);
        } else {
            Serial.println("PWM task: Failed to acquire controlOutput mutex");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }


        uint32_t pulse_us = servo_percent_to_pulse(localControlOutput);
        // Send the PWM signal using RMT
        servo_write_pulse_us_scaled(pulse_us);
        

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms (100 Hz)
    }
}

