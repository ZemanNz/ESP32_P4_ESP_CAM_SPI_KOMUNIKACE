// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "nvs.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "esp_check.h"
// #include "esp_memory_utils.h"
// #include "lvgl.h"
// #include "bsp/esp-bsp.h"
// #include "bsp/display.h"
// #include "lv_demos.h"
// #include "driver/uart.h"
// #include "string.h"
// #include "driver/gpio.h"
// #include <stdint.h>
// #include <stdbool.h>
// #include <math.h>
// /////////////////
// #include <stdio.h>
// #include "lcd.hpp"
// #include "camera.hpp"
// #include "cv_detect.hpp"
// #include "bear_detection.hpp"
// //#include "LogoSokolska.c"
// //#include "Logo_s.c"
// #include "sokolska_logo.c" // Předpokládám, že Logo_s je v tomto souboru
// // ...nyní můžeš používat vidime_medveda...
// ///////////////////////

// static esp_lcd_panel_handle_t display_panel;


// #define UART_PORT      UART_NUM_1
// #define TXD_PIN (GPIO_NUM_4)
// #define RXD_PIN (GPIO_NUM_5)
// static const int RX_BUF_SIZE = 1024;
// static const int TX_BUF_SIZE = 1024;
// #define BAUD_RATE      115200

// #define SYNC0 0xAA
// #define SYNC1 0x55

// bool vidime_medveda = false; // Přidat tuto proměnnou

// static lv_obj_t *xy_label = NULL;
// static lv_obj_t *uart_log_label = NULL; // Přidat tento řádek

// // ...existing code...
// typedef struct __attribute__((packed)) {
//     uint16_t x;
//     uint16_t y;
//     bool camera;
//     bool on;
//     int16_t angle;      // Úhel ve stupních ×10
//     uint16_t distance;  // Délka v mm
//     uint16_t max_distance; // Maximální možná délka v mm
// } Message;
// // ...existing code...
// static Message msg = {0, 0, false, false, 0, 0, 0}; // Inicializace struktury

// static void send_struct(const Message *m)
// {
//     // Magic bytes (hlavička)
//     uint8_t hdr[2] = { SYNC0, SYNC1 };
//     uart_write_bytes(UART_PORT, (const char*)hdr, 2);

//     // Payload (struktura)
//     uart_write_bytes(UART_PORT, (const char*)m, sizeof(*m));

//     // Jednoduchý checksum (součet všech bajtů payloadu)
//     uint8_t checksum = 0;
//     const uint8_t *payload = (const uint8_t *)m;
//     for (size_t i = 0; i < sizeof(*m); ++i) {
//         checksum += payload[i];
//     }
//     uart_write_bytes(UART_PORT, (const char*)&checksum, 1);
//     char buf[128];
//     snprintf(buf, sizeof(buf), "UART: x=%u y=%u camera=%u on=%u angle=%d dist=%umm max=%umm chksum=0x%02X",
//          m->x, m->y, m->camera, m->on, m->angle, m->distance, m->max_distance, checksum);
//     ESP_LOGI("UART", "Sent struct: x=%u y=%u camera=%u on=%u angle=%d dist=%u max=%u checksum=0x%02X",
//          m->x, m->y, m->camera, m->on, m->angle, m->distance, m->max_distance, checksum);
//     if(uart_log_label) lv_label_set_text(uart_log_label, buf);
// }


// typedef struct __attribute__((packed)) {
//     bool esp_ready;
// } EspReadyMsg;

// bool wait_for_esp_ready()
// {
//     uint8_t rx_buf[4]; // 2x sync + 1x bool + 1x checksum
//     while (1) {
//         int len = uart_read_bytes(UART_PORT, rx_buf, sizeof(rx_buf), 100 / portTICK_PERIOD_MS);
//         if (len == sizeof(rx_buf)) {
//             if (rx_buf[0] == SYNC0 && rx_buf[1] == SYNC1) {
//                 uint8_t checksum = rx_buf[2]; // pouze jeden byte v payloadu
//                 if ((uint8_t)rx_buf[2] == rx_buf[3]) { // jednoduchý checksum = hodnota bool
//                     EspReadyMsg *msg = (EspReadyMsg*)&rx_buf[2];
//                     if (msg->esp_ready) {
//                         // ESP32 je připraveno
//                         printf("ESP je ready, spoustím kameru!\n");
//                         return true;
//                     }
//                 }
//             }
//         }
//         // můžeš přidat timeout nebo další logiku
//     }
//     return false;
// }

// // // ...když je potřeba poslat potvrzení...
// // uint8_t tx_buf[4];
// // tx_buf[0] = SYNC0;
// // tx_buf[1] = SYNC1;
// // tx_buf[2] = 1; // true
// // tx_buf[3] = tx_buf[2]; // jednoduchý checksum
// // uart_write_bytes(UART_PORT, (const char*)tx_buf, sizeof(tx_buf));

// static void init(void)
// {
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };
//     uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 0, NULL, 0);
//     uart_param_config(UART_NUM_1, &uart_config);
//     uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

// #define GRID_COUNT 11

// static lv_point_precise_t grid_h_points[GRID_COUNT][2];
// static lv_point_precise_t grid_v_points[GRID_COUNT][2];

// static void create_grid(lv_obj_t * parent)
// {
//      for (int idx = 0; idx < GRID_COUNT; ++idx) {
//         int i = idx * 60;

//         // Vodorovná čára
//         grid_h_points[idx][0].x = 0;
//         grid_h_points[idx][0].y = i;
//         grid_h_points[idx][1].x = 600;
//         grid_h_points[idx][1].y = i;

//         lv_obj_t * line_h = lv_line_create(parent);
//         lv_line_set_points(line_h, grid_h_points[idx], 2);
//         lv_obj_set_style_line_color(line_h, lv_color_hex(0xFAFAFA), 0); // šedá
//         lv_obj_set_style_line_width(line_h, 2, 0);
//         lv_obj_set_style_line_opa(line_h, LV_OPA_COVER, 0);

//         // Svislá čára
//         grid_v_points[idx][0].x = i;
//         grid_v_points[idx][0].y = 0;
//         grid_v_points[idx][1].x = i;
//         grid_v_points[idx][1].y = 600;

//         lv_obj_t * line_v = lv_line_create(parent);
//         lv_line_set_points(line_v, grid_v_points[idx], 2);
//         lv_obj_set_style_line_color(line_v, lv_color_hex(0xFAFAFA), 0); // šedá
//         lv_obj_set_style_line_width(line_v, 2, 0);
//         lv_obj_set_style_line_opa(line_v, LV_OPA_COVER, 0);
//     }
// }

// static void grid_area_event_cb(lv_event_t *e)
// {
//     lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
//     lv_point_t point;
//     lv_indev_t *indev = lv_indev_get_act();
//     if (indev) {
//         lv_indev_get_point(indev, &point);
//         lv_area_t coords;
//         lv_obj_get_coords(obj, &coords);
//         int x = coords.x2 - point.x;
//         int y = 600 - (point.y - coords.y1);
//         if (x >= 0 && x <= 600 && y >= 0 && y <= 600) {
//             msg.x = x;
//             msg.y = y;
//             float dx = (float)x;
//             float dy = (float)y;

//             // Výpočet úhlu od osy Y (0° = nahoru)
//             float angle_rad = atan2f(dx, dy);
//             float angle_deg = angle_rad * 180.0f / M_PI;
//             if (angle_deg < 0) angle_deg += 360.0f;
//             msg.angle = (int16_t)(angle_deg); // Celé stupně

//             // Délka přepony v mm
//             float hypotenuse_px = sqrtf(dx*dx + dy*dy);
//             float scale = 1300.0f / 600.0f; // 600 px = 1300 mm
//             float hypotenuse_mm = hypotenuse_px * scale;
//             msg.distance = (uint16_t)(hypotenuse_mm + 0.5f);

//             // Výpočet max vzdálenosti v daném směru (od [0,0] po okraj)
//             float max_x = 600.0f;
//             float max_y = 600.0f;
//             float dir_x = cosf(angle_rad);
//             float dir_y = sinf(angle_rad);
//             float t_x = (dir_x > 0) ? (max_x / dir_x) : ((dir_x < 0) ? (0.0f / dir_x) : INFINITY);
//             float t_y = (dir_y > 0) ? (max_y / dir_y) : ((dir_y < 0) ? (0.0f / dir_y) : INFINITY);
//             float t = fminf(fabsf(t_x), fabsf(t_y));
//             if (t < 0) t = 0;
//             float max_distance_mm = t * scale;
//             msg.max_distance = (uint16_t)(max_distance_mm + 0.5f);

//             char buf[96];
//             snprintf(buf, sizeof(buf), "X: %d\nY: %d\nA: %d°\nD: %d mm\nMAX: %d mm",
//                 x, y, msg.angle, msg.distance, msg.max_distance);
//             lv_label_set_text(xy_label, buf);
//             send_struct(&msg);
//         }
//     }
// }


// // Callback pro tlačítko "Camera"
// static void btn_camera_event_cb(lv_event_t *e)
// {
//     lv_label_set_text(xy_label, "jedu na kameru.");
//     msg.camera = true; // Nastavíme, že kamera je zapnutá
//     send_struct(&msg); // Odeslat strukturu s informacemi
//     msg.camera = false; // Resetovat flag pro další použití
//     // Spustit detekci kamerou (to co bylo v extern "C" void app_main)
//     bool is_rbcx_ready = wait_for_esp_ready();
//     if (is_rbcx_ready) {
//         ESP_LOGI("RBCX", "dojelo na pozici.");
//     }
//     //ESP_ERROR_CHECK(lcd_init(&display_panel));
//     bsp_display_brightness_set(0);
//     auto cam = new Camera(VIDEO_PIX_FMT_RGB888, 4, V4L2_MEMORY_MMAP, true);
//     auto cv_detect = new CvDetect(display_panel, cam);
//     cv_detect->run();
// }

// extern "C" void app_main(void)
// {
//     bsp_display_cfg_t cfg = {
//         .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
//         .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
//         .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
//         .flags = {
//             .buff_dma = true,
//             .buff_spiram = false,
//             .sw_rotate = false,
//         }
//     };
//     bsp_display_start_with_config(&cfg);
//     bsp_display_brightness_set(50);
//     bsp_display_lock(0);

//     // Klikací plocha bez barvy
//     lv_obj_t * grid_area = lv_obj_create(lv_scr_act());
//     lv_obj_set_size(grid_area, 600, 600);
//     lv_obj_align(grid_area, LV_ALIGN_LEFT_MID, 0, 0);
//     lv_obj_set_style_bg_opa(grid_area, LV_OPA_TRANSP, 0); // průhledné pozadí
//     lv_obj_set_style_bg_color(grid_area, lv_color_make(0,0,0), 0); // nebo LV_COLOR_TRANSP
//     lv_obj_set_style_pad_all(grid_area, 0, 0);           // <--- přidat toto
//     lv_obj_set_style_border_width(grid_area, 0, 0);      // <--- přidat toto
//     create_grid(grid_area);
//     lv_obj_add_event_cb(grid_area, grid_area_event_cb, LV_EVENT_CLICKED, NULL);

//     // X/Y label - barevný text
//     xy_label = lv_label_create(lv_scr_act());
//     lv_obj_align(xy_label, LV_ALIGN_TOP_RIGHT, -200, 40);
//     lv_label_set_text(xy_label, "X: \nY: ");
//     lv_obj_set_style_text_color(xy_label, lv_palette_main(LV_PALETTE_BLUE), 0);

//     // Tlačítko "Camera" - vysoké, skoro průhledné
//     lv_obj_t * btn_camera = lv_btn_create(lv_scr_act());
//     lv_obj_set_size(btn_camera, 300, 580); // výška téměř celá polovina (při výšce 600 px)
//     lv_obj_align(btn_camera, LV_ALIGN_TOP_RIGHT, -90, 10); // posunout nahoru
//     lv_obj_set_style_bg_color(btn_camera, lv_palette_main(LV_PALETTE_RED), 0);
//     lv_obj_set_style_bg_opa(btn_camera, LV_OPA_20, 0); // téměř průhledné (0-255, 20 je slabé)
//     lv_obj_set_style_border_width(btn_camera, 0, 0); // bez rámečku
//     lv_obj_add_event_cb(btn_camera, btn_camera_event_cb, LV_EVENT_CLICKED, NULL);
//     lv_obj_t * label_camera = lv_label_create(btn_camera);
//     lv_label_set_text(label_camera, "Camera");

//     // UART log label - posouvající se text
//     uart_log_label = lv_label_create(lv_scr_act());
//     lv_obj_align(uart_log_label, LV_ALIGN_TOP_RIGHT, -90, 320);
//     lv_label_set_long_mode(uart_log_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
//     lv_obj_set_width(uart_log_label, 300);
//     lv_label_set_text(uart_log_label, "UART: ---");

//     // ...existing code...
//     lv_obj_t * logo_img = lv_img_create(lv_scr_act());
//     lv_img_set_src(logo_img, &Logo_s); // název proměnné z .c souboru
//     lv_obj_align(logo_img, LV_ALIGN_CENTER, -215, 0);
//     // ...existing code...

//     init();
//     bsp_display_unlock();
// }
