#include "system_tile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_flash.h"
#include "esp_psram.h"
#include "driver/temperature_sensor.h"
#include "esp_private/esp_clk.h"
#include "esp_heap_caps.h"

#include "esp_pcf85063_port.h"
#include "esp_sdcard_port.h"
#include "esp_es8311_port.h"
#include "esp_3inch5_lcd_port.h"

#include "esp_codec_dev.h"

SemaphoreHandle_t es8311_test_semaphore;
temperature_sensor_handle_t temp_sensor = NULL;

// extern void brightness_set_level(uint8_t level);
// extern void es8311_test_init(SemaphoreHandle_t xBinarySemaphore);

lv_obj_t *label_brightness;
lv_obj_t *label_time;
lv_obj_t *label_date;
lv_obj_t *label_flash;
lv_obj_t *label_psram;
lv_obj_t *label_chip_temp;
lv_obj_t *label_chip_freq;
lv_obj_t *label_sd;
lv_obj_t *label_es8311_test;  // For updating button text
lv_obj_t *btn_es8311_test;    // For updating button color


static void slider_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_t *slider = lv_event_get_target(e);
        int value = lv_slider_get_value(slider);
        // printf("Slider value: %d\n", value);

        lv_label_set_text_fmt(label_brightness, "%d %%", value);
        // brightness_set_level(value);
        esp_28_brightness_port_set(value);
        // bsp_display_handle_t display = bsp_display_get_handle();
        // display->set_brightness(display, value);
        lv_event_stop_bubbling(e);
    }
}

static void system_time_cb(lv_timer_t *timer)
{
    char str[20];
    float tsens_out;
    RTC_DateTime datetime = rtc.getDateTime();

    lv_label_set_text_fmt(label_date, "%04d-%02d-%02d", datetime.year, datetime.month, datetime.day);
    lv_label_set_text_fmt(label_time, "%02d:%02d:%02d", datetime.hour, datetime.minute, datetime.second);

    temperature_sensor_get_celsius(temp_sensor, &tsens_out);
    sprintf(str, "%.1f'C", tsens_out);
    if(label_chip_temp != NULL)
    lv_label_set_text(label_chip_temp, str);
}

// Callback function for updating button text in LVGL thread
static void update_button_text_cb(void *arg)
{
    const char *text = (const char *)arg;
    if (label_es8311_test != NULL)
    {
        lv_label_set_text(label_es8311_test, text);
    }
}

// Callback function for updating button color in LVGL thread
static void update_button_color_cb(void *arg)
{
    uint32_t color = (uint32_t)(uintptr_t)arg;
    if (btn_es8311_test != NULL)
    {
        lv_obj_set_style_bg_color(btn_es8311_test, lv_color_hex(color), LV_PART_MAIN);
    }
}

static void lvgl_es8311_test_task(void *arg)
{
    const int RECORD_SECONDS = 5;  // Recording duration in seconds
    const int SAMPLE_RATE = 48000;
    const int CHANNELS = 1;
    const int BITS_PER_SAMPLE = 16;
    const int CHUNK_DURATION_MS = 200;  // Chunk duration in milliseconds for updating countdown
    
    while (1)
    {
        if (xSemaphoreTake(es8311_test_semaphore, portMAX_DELAY) == pdTRUE)
        {
            printf("Audio test started\r\n");
            
            int err = 0;
            // 5 seconds recording
            const int total_size = RECORD_SECONDS * SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE >> 3);
            const int chunk_size = (CHUNK_DURATION_MS * SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE >> 3)) / 1000;
            const int num_chunks = (total_size + chunk_size - 1) / chunk_size;
            
            uint8_t *data = (uint8_t *)heap_caps_malloc(total_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (data == NULL)
            {
                printf("Memory allocation failed\r\n");
                lv_async_call(update_button_text_cb, (void *)"Audio Test");
                lv_async_call(update_button_color_cb, (void *)(uintptr_t)0x2196F3); // Restore default blue color
                continue;
            }
            
            // Get codec device handles
            esp_codec_dev_handle_t input_dev = esp_es8311_get_input_dev();
            esp_codec_dev_handle_t output_dev = esp_es8311_get_output_dev();
            
            // Phase 1: Recording (with countdown)
            printf("Recording...\r\n");
            lv_async_call(update_button_color_cb, (void *)(uintptr_t)0xFF5722); // Recording phase: red color
            
            esp_codec_dev_set_in_gain(input_dev, 100.0); // Set maximum gain
            
            int bytes_read = 0;
            for (int i = 0; i < num_chunks; i++)
            {
                int current_chunk_size = (i == num_chunks - 1) ? (total_size - bytes_read) : chunk_size;
                
                // Update countdown
                int remaining_seconds = RECORD_SECONDS - (i * CHUNK_DURATION_MS / 1000);
                char countdown_text[32];
                snprintf(countdown_text, sizeof(countdown_text), "Recording %ds", remaining_seconds);
                lv_async_call(update_button_text_cb, (void *)countdown_text);
                
                err = esp_codec_dev_read(input_dev, data + bytes_read, current_chunk_size);
                if (err != ESP_CODEC_DEV_OK)
                {
                    printf("Recording error %d at chunk %d\n", err, i);
                    break;
                }
                bytes_read += current_chunk_size;
            }
            
            esp_codec_dev_set_in_gain(input_dev, 0.0);
            printf("Recorded %d bytes\n", bytes_read);
            
            // Phase 2: Playback
            lv_async_call(update_button_text_cb, (void *)"Replaying");
            lv_async_call(update_button_color_cb, (void *)(uintptr_t)0x4CAF50); // Playback phase: green color
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for UI update
            
            printf("Replaying...\r\n");
            esp_codec_dev_set_out_vol(output_dev, 100.0); // Set maximum volume
            err = esp_codec_dev_write(output_dev, data, bytes_read);
            esp_codec_dev_set_out_vol(output_dev, 0.0);
            
            if (err == ESP_CODEC_DEV_OK)
                printf("Replayed %d bytes\n", bytes_read);
            else
                printf("Replay error %d\n", err);
            
            heap_caps_free(data);
            
            // Phase 3: Restore initial state
            lv_async_call(update_button_text_cb, (void *)"Audio Test");
            lv_async_call(update_button_color_cb, (void *)(uintptr_t)0x2196F3); // Restore default blue color
            
            printf("Audio test completed\r\n");
        }        
    }
}

void system_init(void)
{
    uint32_t flash_size;
    uint32_t psram_size;
    uint64_t sdcard_size;
    uint32_t cpu_freq;

    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    temperature_sensor_enable(temp_sensor);

    esp_flash_get_size(NULL, &flash_size);
    if(label_flash != NULL)
    lv_label_set_text_fmt(label_flash, "%d MB", (int)(flash_size / 1024 / 1024));

    psram_size = (uint32_t)esp_psram_get_size();
    if(label_psram != NULL)
    lv_label_set_text_fmt(label_psram, "%d MB", (int)(psram_size / 1024 / 1024));

    cpu_freq = esp_clk_cpu_freq();
    if(label_chip_freq != NULL)
    lv_label_set_text_fmt(label_chip_freq, "%d MHz", (int)(cpu_freq / 1000 / 1000));

    sdcard_size = esp_sdcard_port_get_size();
    if(label_sd != NULL){
        if(sdcard_size == 0){
            lv_label_set_text(label_sd, "No SDCard");
            lv_obj_set_style_text_color(label_sd, lv_color_hex(0xFF0000), LV_PART_MAIN);
        } else {
            lv_label_set_text_fmt(label_sd, "%d MB", (int)(sdcard_size / 1024 / 1024));
        }
    }


    es8311_test_semaphore = xSemaphoreCreateBinary();
    assert(es8311_test_semaphore != NULL);
    xTaskCreatePinnedToCore(lvgl_es8311_test_task, "lvgl_es8311_test_task", 4096, NULL, 1, NULL, 1);

    // es8311_test_init(xBinarySemaphore);
}

static void btn_es8311_test_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED && es8311_test_semaphore != NULL)
    {
        xSemaphoreGive(es8311_test_semaphore);
    }
}

void system_tile_init(lv_obj_t *parent)
{
    /*Create a list*/
    lv_obj_t *list = lv_list_create(parent);
    lv_obj_t *lable = lv_label_create(parent);
    lv_obj_set_style_text_font(lable, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_label_set_text(lable, "System");
    lv_obj_align(lable, LV_ALIGN_TOP_MID, 0, 3);

    lv_obj_set_size(list, lv_pct(95), LV_SIZE_CONTENT);
    lv_obj_set_style_max_height(list, lv_pct(80), LV_PART_MAIN);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 30);

    btn_es8311_test = lv_btn_create(parent);
    label_es8311_test = lv_label_create(btn_es8311_test);
    lv_obj_set_size(btn_es8311_test, 110, 30); // Set fixed size to accommodate countdown text
    lv_label_set_text(label_es8311_test, "Audio Test");
    lv_obj_center(label_es8311_test);
    lv_obj_align(btn_es8311_test, LV_ALIGN_BOTTOM_LEFT, 2, -20);
    lv_obj_set_style_bg_color(btn_es8311_test, lv_color_hex(0x2196F3), LV_PART_MAIN); // Default blue color
    lv_obj_add_event_cb(btn_es8311_test, btn_es8311_test_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *slider = lv_slider_create(parent);
    lv_slider_set_range(slider, 1, 100);
    lv_slider_set_value(slider, 80, LV_ANIM_OFF);
    lv_obj_set_size(slider, lv_pct(50), 20); // Height set to 20 pixels for easier clicking
    lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 55, -25);
    // Increase slider knob size for easier touch
    lv_obj_set_style_pad_all(slider, 5, LV_PART_KNOB);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *list_item;
    // list_item = lv_list_add_btn(list, NULL, "Chip");
    // lv_obj_t *label_chip = lv_label_create(list_item);
    // lv_label_set_text(label_chip, "ESP32-S3");

    // list_item = lv_list_add_btn(list, NULL, "ChipTemp");
    // label_chip_temp = lv_label_create(list_item);
    // lv_label_set_text(label_chip_temp, "--- C");

    // list_item = lv_list_add_btn(list, NULL, "ChipFreq");
    // label_chip_freq = lv_label_create(list_item);
    // lv_label_set_text(label_chip_freq, "--- MHz");

    list_item = lv_list_add_btn(list, NULL, "Brightness");
    label_brightness = lv_label_create(list_item);
    lv_label_set_text(label_brightness, "80 %");

    // list_item = lv_list_add_btn(list, NULL, "SRAM");
    // lv_obj_t *label_ram = lv_label_create(list_item);
    // lv_label_set_text(label_ram, "512 KB");

    // list_item = lv_list_add_btn(list, NULL, "PSRAM");
    // label_psram = lv_label_create(list_item);
    // lv_label_set_text(label_psram, "--- MB");

    // list_item = lv_list_add_btn(list, NULL, "Flash");
    // label_flash = lv_label_create(list_item);
    // lv_label_set_text(label_flash, "--- MB");

    list_item = lv_list_add_btn(list, NULL, "SDCard");
    label_sd = lv_label_create(list_item);
    lv_label_set_text(label_sd, "--- MB");

    list_item = lv_list_add_btn(list, NULL, "Date");
    label_date = lv_label_create(list_item);
    lv_label_set_text(label_date, "2025-01-01");

    list_item = lv_list_add_btn(list, NULL, "Time");
    label_time = lv_label_create(list_item);
    lv_label_set_text(label_time, "12:00:00");
    system_init();
    lv_timer_create(system_time_cb, 1000, NULL);
}