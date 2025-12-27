#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

// === PINY (Tvoje funkční zapojení) ===
#define GPIO_HANDSHAKE 2   // Handshake
#define GPIO_MOSI      13
#define GPIO_MISO      12
#define GPIO_SCLK      14
#define GPIO_CS        15


#define RCV_HOST       SPI2_HOST

// === NASTAVENÍ OBRAZU (QQVGA 160x120) ===
#define SCREEN_WIDTH   160
#define CHUNK_HEIGHT   20
#define BYTES_PER_PIXEL 2
// Velikost jednoho balíku: 160 * 20 * 2 = 6 400 Bajtů
#define CHUNK_SIZE     (SCREEN_WIDTH * CHUNK_HEIGHT * BYTES_PER_PIXEL)
#define TOTAL_CHUNKS   6     

static const char *TAG = "CAM_SYNC_SENDER";

// Konfigurace kamery
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN, .pin_reset = CAM_PIN_RESET, .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD, .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6, .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4, .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 10000000,         
    .frame_size = FRAMESIZE_QQVGA,    
    .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565, 
    .jpeg_quality = 12, .fb_count = 1, 
    .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

void my_post_setup_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 1);
}
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 0);
}

// === HSV KONVERZE ===
typedef struct {
    int h; // 0-360
    int s; // 0-100
    int v; // 0-100
} hsv_color;

hsv_color rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b) {
    hsv_color hsv;
    unsigned char min_val, max_val, delta;

    min_val = (g < b) ? g : b;
    min_val = (r < min_val) ? r : min_val;

    max_val = (g > b) ? g : b;
    max_val = (r > max_val) ? r : max_val;

    hsv.v = (int)(((long)max_val * 100) / 255);
    delta = max_val - min_val;

    if (delta == 0) { // Achromatické (šedá)
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    if (max_val > 0) {
        hsv.s = (int)(((long)delta * 100) / max_val);
    } else { // Nemělo by nastat, pokud delta > 0
        hsv.s = 0;
        hsv.h = 0;
        return hsv;
    }
    
    long hue;
    if (r >= max_val)
        hue = (long)(g - b) * 60 / delta;
    else if (g >= max_val)
        hue = 120 + ((long)(b - r) * 60 / delta);
    else
        hue = 240 + ((long)(r - g) * 60 / delta);

    if (hue < 0)
        hue += 360;

    hsv.h = (int)hue;
    return hsv;
}
// ====================

void app_main(void)
{
    if (esp_camera_init(&camera_config) != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    // Init SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI, 
        .miso_io_num = GPIO_MISO, 
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1,
        .max_transfer_sz = 8192, 
    };
    spi_slave_interface_config_t slvcfg = {
        .mode = 0, .spics_io_num = GPIO_CS, .queue_size = 3, .flags = 0,
        .post_setup_cb = my_post_setup_cb, .post_trans_cb = my_post_trans_cb
    };
    
    gpio_config_t io_conf = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = (1ULL << GPIO_HANDSHAKE) };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_HANDSHAKE, 0);

    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

    // Buffery
    uint8_t *sendbuf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
    uint8_t *recvbuf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
    
    // --- OPRAVA ZDE ---
    spi_slave_transaction_t t; // Správný typ pro SLAVE
    memset(&t, 0, sizeof(t));

    ESP_LOGI(TAG, "Streaming QQVGA (Sync Header Active)...");

    while (1) {
        camera_fb_t *pic = esp_camera_fb_get();
        if (!pic) {
            vTaskDelay(100);
            continue;
        }

        // --- Levý modrý a pravý zelený pruh ---
        int vertical_stripe_width = 20;
        for (int y = 0; y < pic->height; y++) {
            // Levý modrý pruh
            for (int x = 0; x < vertical_stripe_width; x++) {
                int pixel_index = (y * pic->width + x) * BYTES_PER_PIXEL;
                pic->buf[pixel_index] = 0x00; // Modrá (High)
                pic->buf[pixel_index + 1] = 0x1F; // Modrá (Low)
            }
            // Pravý zelený pruh
            for (int x = pic->width - vertical_stripe_width; x < pic->width; x++) {
                int pixel_index = (y * pic->width + x) * BYTES_PER_PIXEL;
                pic->buf[pixel_index] = 0x07; // Zelená (High)
                pic->buf[pixel_index + 1] = 0xE0; // Zelená (Low)
            }
        }

        // === DETEKCE BAREV (HSV) UPROSTŘED ===
        int box_size = 80;
        int box_start_x = (pic->width / 2) - (box_size / 2);
        int box_start_y = (pic->height / 2) - (box_size / 2);
        int box_end_x = box_start_x + box_size;
        int box_end_y = box_start_y + box_size;

        // --- Prahové hodnoty pro ČERVENOU ---
        #define RED_HUE_MIN1 0
        #define RED_HUE_MAX1 20
        #define RED_HUE_MIN2 340
        #define RED_HUE_MAX2 360
        #define RED_SAT_MIN 35
        #define RED_VAL_MIN 35

        // --- Prahové hodnoty pro ZELENOU ---
        #define GREEN_HUE_MIN 100
        #define GREEN_HUE_MAX 140
        #define GREEN_SAT_MIN 35
        #define GREEN_VAL_MIN 35

        // --- Prahové hodnoty pro MODROU ---
        #define BLUE_HUE_MIN 220
        #define BLUE_HUE_MAX 260
        #define BLUE_SAT_MIN 35
        #define BLUE_VAL_MIN 35

        for (int y = box_start_y; y < box_end_y; y++) {
            for (int x = box_start_x; x < box_end_x; x++) {
                int pixel_index = (y * pic->width + x) * BYTES_PER_PIXEL;

                // Vykreslit ohraničení bílou barvou
                if (y == box_start_y || y == box_end_y - 1 || x == box_start_x || x == box_end_x - 1) {
                    pic->buf[pixel_index] = 0xFF; // Bílá (High)
                    pic->buf[pixel_index + 1] = 0xFF; // Bílá (Low)
                    continue;
                }

                // 1. Získat RGB565 pixel
                uint16_t pixel_value = (pic->buf[pixel_index] << 8) | pic->buf[pixel_index + 1];

                // 2. Převést RGB565 na RGB888
                uint8_t r5 = (pixel_value >> 11) & 0x1F;
                uint8_t g6 = (pixel_value >> 5) & 0x3F;
                uint8_t b5 = pixel_value & 0x1F;

                uint8_t r8 = (r5 * 255) / 31;
                uint8_t g8 = (g6 * 255) / 63;
                uint8_t b8 = (b5 * 255) / 31;

                // 3. Převést RGB888 na HSV
                hsv_color hsv = rgb_to_hsv(r8, g8, b8);

                // 4. Detekce barvy podle Hue, Saturation a Value
                if (((hsv.h >= RED_HUE_MIN1 && hsv.h <= RED_HUE_MAX1) || (hsv.h >= RED_HUE_MIN2 && hsv.h <= RED_HUE_MAX2)) && hsv.s > RED_SAT_MIN && hsv.v > RED_VAL_MIN) {
                    // Červená
                    pic->buf[pixel_index] = 0xF8;
                    pic->buf[pixel_index + 1] = 0x00;
                } else if ((hsv.h >= GREEN_HUE_MIN && hsv.h <= GREEN_HUE_MAX) && hsv.s > GREEN_SAT_MIN && hsv.v > GREEN_VAL_MIN) {
                    // Zelená
                    pic->buf[pixel_index] = 0x07;
                    pic->buf[pixel_index + 1] = 0xE0;
                } else if ((hsv.h >= BLUE_HUE_MIN && hsv.h <= BLUE_HUE_MAX) && hsv.s > BLUE_SAT_MIN && hsv.v > BLUE_VAL_MIN) {
                    // Modrá
                    pic->buf[pixel_index] = 0x00;
                    pic->buf[pixel_index + 1] = 0x1F;
                }
                // Jinak ponechat původní barvu
            }
        }
        // =====================================

        size_t offset = 0;
        
        for (int i = 0; i < TOTAL_CHUNKS; i++) {
            if (offset + CHUNK_SIZE <= pic->len) {
                memcpy(sendbuf, pic->buf + offset, CHUNK_SIZE);
            } else {
                memset(sendbuf, 0, CHUNK_SIZE);
            }

            // === VLOŽENÍ HESLA (SYNC HEADER) ===
            // Do prvního chunku vložíme AA CC, aby se P4 chytila
            if (i == 0) {
                sendbuf[0] = 0xAA;
                sendbuf[1] = 0xCC;
            }
            // ===================================

            offset += CHUNK_SIZE;

            t.length = CHUNK_SIZE * 8;
            t.tx_buffer = sendbuf;
            t.rx_buffer = recvbuf;

            spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
        }
        
        esp_camera_fb_return(pic);
    }
}