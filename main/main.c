#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_random.h"

#include "driver/gpio.h"
#include "esp_random.h"
#include "esp_log.h"

#define TAG "RAND_GPIO"

#define NUM_OUTPUTS 4

#define TAG "PWM_RANDOM"

// Change these GPIOs as needed
#define PWM_GPIO_0    4
#define PWM_GPIO_1    5
#define PWM_GPIO_2    6
#define PWM_GPIO_3    7

#define PWM_MIN_FREQ  500
#define PWM_MAX_FREQ  3800

#define NUM_PWM       4

typedef enum {
    MACHINE_RUNNING,
    MACHINE_FAULT
} machine_state_t;

typedef struct {
    machine_state_t state;
    uint32_t pwm_freq[NUM_PWM];
    uint8_t led_state[NUM_OUTPUTS];
    uint8_t led_blink_mask;
    int failed_roller;
    TickType_t state_start;
} machine_t;

static machine_t machine;

static void machine_start_cycle(void)
{
    machine.state = MACHINE_RUNNING;
    machine.state_start = xTaskGetTickCount();

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "Starting new production cycle");

    for (int i = 0; i < NUM_PWM; i++){
        machine.pwm_freq[i] =  PWM_MIN_FREQ + esp_random() % PWM_MAX_FREQ;
        ESP_LOGI(TAG,
                 "Roller %d -> %lu Hz",
                 i,
                 (unsigned long)machine.pwm_freq[i]);
    }
    uint8_t leds = esp_random() & 0x0F;

    ESP_LOGI(TAG,
             "LED Pattern: %d %d %d %d",
             (leds >> 0) & 1,
             (leds >> 1) & 1,
             (leds >> 2) & 1,
             (leds >> 3) & 1);

    for (int i = 0; i < NUM_OUTPUTS; i++){
        machine.led_state[i] = (leds >> i) & 1;
    }
    machine.led_blink_mask = 0;
    ESP_LOGI(TAG, "Machine RUNNING");
    ESP_LOGI(TAG, "======================================");
}
static void machine_fault(void)
{
    machine.state = MACHINE_FAULT;
    machine.state_start = xTaskGetTickCount();
    machine.failed_roller = esp_random() % NUM_PWM;
    machine.pwm_freq[machine.failed_roller] = 0;

    switch (machine.failed_roller)
    {
        case 0:
            machine.led_blink_mask = BIT0;
            break;

        case 1:
            machine.led_blink_mask = BIT1;
            break;

        case 2:
            machine.led_blink_mask = BIT2;
            break;

        case 3:
            machine.led_blink_mask = BIT3;
            break;
    }
    ESP_LOGW(TAG, "######################################");
    ESP_LOGW(TAG, "FAULT DETECTED");
    ESP_LOGW(TAG, "Stopped Roller : %d", machine.failed_roller);
    ESP_LOGW(TAG, "Blink Mask     : 0x%02X", machine.led_blink_mask);
    ESP_LOGW(TAG, "Machine entered FAULT state");
    ESP_LOGW(TAG, "######################################");
}

static const int pwm_gpios[NUM_PWM] = {
    PWM_GPIO_0,
    PWM_GPIO_1,
    PWM_GPIO_2,
    PWM_GPIO_3
};

static const ledc_channel_t pwm_channels[NUM_PWM] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};


/*
 * Select the highest possible duty resolution for a given frequency.
 *
 * Timer clock = 80 MHz
 *
 * freq = clk / (2^resolution * divider)
 *
 * divider must be >= 1.
 */
static ledc_timer_bit_t choose_resolution(uint32_t freq)
{
    if (freq <= 4882)
        return LEDC_TIMER_14_BIT;
    else if (freq <= 9765)
        return LEDC_TIMER_13_BIT;
    else if (freq <= 19531)
        return LEDC_TIMER_12_BIT;
    else if (freq <= 39062)
        return LEDC_TIMER_11_BIT;
    else if (freq <= 78125)
        return LEDC_TIMER_10_BIT;
    else
        return LEDC_TIMER_9_BIT;
}

// Change to your desired pins
static const gpio_num_t output_pins[NUM_OUTPUTS] = {
    GPIO_NUM_18,
    GPIO_NUM_17,
    GPIO_NUM_16,
    GPIO_NUM_15,
};


void apply_pwm(void)
{
    for (int i = 0; i < NUM_PWM; i++)
    {
        ledc_timer_bit_t resolution =
            choose_resolution(machine.pwm_freq[i] == 0 ?
                              500 :
                              machine.pwm_freq[i]);

        ledc_timer_config_t timer_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = resolution,
            .timer_num = (ledc_timer_t)i,
            .freq_hz = machine.pwm_freq[i] == 0 ?
                       500 :
                       machine.pwm_freq[i],
            .clk_cfg = LEDC_AUTO_CLK,
        };

        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        uint32_t max_duty = (1 << resolution) - 1;

        if(machine.pwm_freq[i] == 0)
            ledc_set_duty(LEDC_LOW_SPEED_MODE,
                          pwm_channels[i],
                          0);
        else
            ledc_set_duty(LEDC_LOW_SPEED_MODE,
                          pwm_channels[i],
                          max_duty/2);

        ledc_update_duty(LEDC_LOW_SPEED_MODE,
                         pwm_channels[i]);
    }
}

void apply_leds(void)
{
    bool blink = ((xTaskGetTickCount() / pdMS_TO_TICKS(50)) & 1);

    for(int i=0;i<NUM_OUTPUTS;i++){
        bool level = machine.led_state[i];

        if(machine.led_blink_mask & BIT(i))
            level = blink;

        gpio_set_level(output_pins[i], level);
    }
}

void machine_update(void)
{
    TickType_t elapsed =
        xTaskGetTickCount() - machine.state_start;

    switch(machine.state)
    {
        case MACHINE_RUNNING:
            if(elapsed > pdMS_TO_TICKS(15000)){
                ESP_LOGI(TAG, "Production cycle finished.");
                machine_fault();
            }
            break;

        case MACHINE_FAULT:
            if(elapsed > pdMS_TO_TICKS(15000)){
                ESP_LOGI(TAG, "Fault cleared.");
                machine_start_cycle();
            }
            break;
    }
}


void random_gpio_outputs_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (int i = 0; i < NUM_OUTPUTS; i++) {
        io_conf.pin_bit_mask |= (1ULL << output_pins[i]);
    }

    gpio_config(&io_conf);

    // Initialize all outputs LOW
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        gpio_set_level(output_pins[i], 0);
    }
}

static TickType_t last_status = 0;

void machine_status_log(void)
{
    TickType_t now = xTaskGetTickCount();

    if(now - last_status < pdMS_TO_TICKS(1000))
        return;

    last_status = now;

    ESP_LOGI(TAG,
        "[%s] PWM = [%4lu %4lu %4lu %4lu]  LEDs = [%d %d %d %d] BlinkMask=0x%X",
        machine.state == MACHINE_RUNNING ? "RUN" : "FLT",

        (unsigned long)machine.pwm_freq[0],
        (unsigned long)machine.pwm_freq[1],
        (unsigned long)machine.pwm_freq[2],
        (unsigned long)machine.pwm_freq[3],

        machine.led_state[0],
        machine.led_state[1],
        machine.led_state[2],
        machine.led_state[3],

        machine.led_blink_mask);
}


void app_main(void)
{
    // Seed standard rand() if desired
    srand((unsigned int)time(NULL));
    random_gpio_outputs_init();

    for (int i = 0; i < NUM_PWM; i++) {

        uint32_t freq = PWM_MIN_FREQ;
        ledc_timer_bit_t resolution = choose_resolution(freq);

        ledc_timer_config_t timer_cfg = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = resolution,
            .timer_num        = (ledc_timer_t)i,
            .freq_hz          = freq,
            .clk_cfg          = LEDC_AUTO_CLK,
        };

        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        uint32_t max_duty = (1U << resolution) - 1;

        ledc_channel_config_t ch_cfg = {
            .gpio_num   = pwm_gpios[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = pwm_channels[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = (ledc_timer_t)i,
            .duty       = max_duty / 2, // 50%
            .hpoint     = 0,
        };

        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

        ESP_LOGI(TAG,
                 "PWM%d -> GPIO %d : %lu Hz",
                 i,
                 pwm_gpios[i],
                 (unsigned long)freq);
    }

    machine_start_cycle();
    apply_pwm();
    apply_leds();

    while (1)
    {
        machine_update();

        apply_pwm();

        apply_leds();

        machine_status_log();

        vTaskDelay(pdMS_TO_TICKS(50));
    }

}