
#include "camera_tile.h"
#include "esp_camera.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_lvgl_port.h"
#include "esp_log.h"

static const char *TAG = "camera_tile";

lv_obj_t *cam_ing;

#define EXPECTED_FRAME_SIZE (320 * 480 * 2)
#define MAX_CONSECUTIVE_ERRORS 10

void camera_task(void *arg)
{
    camera_fb_t *pic;
    lv_img_dsc_t img_dsc;
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = 320;
    img_dsc.header.h = 480;
    img_dsc.data_size = EXPECTED_FRAME_SIZE;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data = NULL;
    
    uint32_t error_count = 0;
    uint32_t frame_count = 0;
    
    while (1)
    {
        pic = esp_camera_fb_get();

        if (pic != NULL)
        {
            // Validate frame size before using it
            if (pic->len == EXPECTED_FRAME_SIZE)
            {
                img_dsc.data = pic->buf;
                
                // Try to acquire LVGL lock with timeout
                if (lvgl_port_lock(10))  // 10ms timeout instead of 0
                {
                    lv_img_set_src(cam_ing, &img_dsc);
                    lvgl_port_unlock();
                    error_count = 0;  // Reset error counter on success
                }
                else
                {
                    ESP_LOGW(TAG, "Failed to acquire LVGL lock");
                }
            }
            else
            {
                error_count++;
                ESP_LOGW(TAG, "Invalid frame size: %zu (expected %d), errors: %lu", 
                         pic->len, EXPECTED_FRAME_SIZE, error_count);
                
                // If too many consecutive errors, try to reset camera
                if (error_count >= MAX_CONSECUTIVE_ERRORS)
                {
                    ESP_LOGE(TAG, "Too many consecutive errors, camera may need reset");
                    error_count = 0;  // Reset counter to avoid log spam
                }
            }
            
            // Always return the frame buffer
            esp_camera_fb_return(pic);
            frame_count++;
        }
        else
        {
            ESP_LOGW(TAG, "Failed to get camera frame");
            error_count++;
        }

        // Longer delay to reduce CPU load and give other tasks time
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms instead of 1ms
    }
}

void camera_tile_init(lv_obj_t *parent)
{
    cam_ing = lv_img_create(parent);
    
    // Set image rotation angle: 90 degrees counterclockwise = 2700 (LVGL angle unit is 0.1 degree)
    lv_img_set_angle(cam_ing, 2700);
    // Set rotation pivot point to image center
    lv_img_set_pivot(cam_ing, 160, 240);
    
    // Center display
    lv_obj_center(cam_ing);
    
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL)
    {
        // Increase stack size from 10KB to 16KB for safety margin
        // This helps prevent stack overflow during error conditions
        xTaskCreatePinnedToCore(camera_task, "camera_task", 1024 * 16, NULL, 1, NULL, 1);
    }
}