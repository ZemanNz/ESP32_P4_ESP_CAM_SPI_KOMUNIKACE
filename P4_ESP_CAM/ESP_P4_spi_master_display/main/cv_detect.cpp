/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cv_detect.hpp"
#include "bear_detection.hpp"

static const char *TAG = "cv_detect";

void CvDetect::draw_rectangle_rgb(uint8_t *buffer, int width, int height,
                                  int x1, int y1, int x2, int y2,
                                  uint8_t r, uint8_t g, uint8_t b,
                                  int thickness)
{
    if (x1 < 0) {
        x1 = 0;
    }
    if (y1 < 0) {
        y1 = 0;
    }
    if (x2 >= width) {
        x2 = width - 1;
    }
    if (y2 >= height) {
        y2 = height - 1;
    }

    for (int t = 0; t < thickness; ++t) {
        for (int x = x1; x <= x2; ++x) {
            if (y1 + t >= 0 && y1 + t < height) {
                int index = ((y1 + t) * width + x) * 3;
                buffer[index] = b;
                buffer[index + 1] = g;
                buffer[index + 2] = r;
            }
            if (y2 - t >= 0 && y2 - t < height) {
                int index = ((y2 - t) * width + x) * 3;
                buffer[index] = b;
                buffer[index + 1] = g;
                buffer[index + 2] = r;
            }
        }

        for (int y = y1; y <= y2; ++y) {
            if (x1 + t >= 0 && x1 + t < width) {
                int index = ((y) * width + (x1 + t)) * 3;
                buffer[index] = b;
                buffer[index + 1] = g;
                buffer[index + 2] = r;
            }
            if (x2 - t >= 0 && x2 - t < width) {
                int index = ((y) * width + (x2 - t)) * 3;
                buffer[index] = b;
                buffer[index + 1] = g;
                buffer[index + 2] = r;
            }
        }
    }
}

void CvDetect::detect(cam_fb_t *fb)
{
    long long start_time = esp_timer_get_time();
   
}

void CvDetect::lcd_refresh_task(void *args)
{
    CvDetect *self = (CvDetect *)args;
    while (1) {
        self->m_cam->cam_fb_get();
        cam_fb_t *fb = self->m_cam->cam_fb_peek(false);

        std::vector<result_t> results;
        xSemaphoreTake(self->m_mutex, portMAX_DELAY);
        results = self->m_results;
        xSemaphoreGive(self->m_mutex);

        do_bear_detection(fb);

        //self->draw_rectangle_rgb((uint8_t*) fb->buf, fb->width, fb->height, 100, 100, 150, 150, 0, 255, 0, 5);

        //esp_lcd_panel_draw_bitmap(self->m_lcd_panel, 0, 0, fb->width, fb->height, fb->buf);
        self->m_cam->cam_fb_return();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void CvDetect::cv_detect_task(void *args)
{
    CvDetect *self = (CvDetect *)args;
    while (1) {
        self->m_cam->cam_fb_get();
        cam_fb_t *fb = self->m_cam->cam_fb_peek(false);
        self->detect(fb);
        self->m_cam->cam_fb_return();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

CvDetect::CvDetect(esp_lcd_panel_handle_t lcd_panel,
                   Camera *cam) :
    m_lcd_panel(lcd_panel),
    m_cam(cam)
{
    m_mutex = xSemaphoreCreateMutex();
    m_is_running = false;
}

CvDetect::~CvDetect()
{
    vSemaphoreDelete(m_mutex);
    m_mutex = nullptr;
    m_is_running = false;
}

void CvDetect::run()
{
    if (m_is_running) {
        return;
    }

    if (xTaskCreatePinnedToCore(lcd_refresh_task, "lcd_refresh_task", 4 * 1024, this, 10, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create lcd refresh task");
        return;
    }

    if (xTaskCreatePinnedToCore(cv_detect_task, "cv_detect_task", 10 * 1024, this, 10, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create cv detect task");
        return;
    }

    m_is_running = true;
}
