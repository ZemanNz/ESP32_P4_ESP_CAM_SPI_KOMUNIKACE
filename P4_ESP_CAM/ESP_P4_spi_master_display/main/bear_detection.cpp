#include "bear_detection.hpp"

void set_pixel(cam_fb_t* fb, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    int index = (y * fb->width + x) * 3;
    uint8_t* buf = (uint8_t*) fb->buf;
    buf[index] = r;
    buf[index + 1] = g;
    buf[index + 2] = b;
}

void get_pixel(cam_fb_t* fb, int x, int y, uint8_t* r, uint8_t* g, uint8_t* b) {
    int index = (y * fb->width + x) * 3;
    uint8_t* buf = (uint8_t*) fb->buf;
    if(r != NULL) {
        *r = buf[index];
    }
    if(g != NULL) {
        *g = buf[index + 1];
    }
    if(b != NULL) {
        *b = buf[index + 2];
    }
    
}

void fill_rect(cam_fb_t* fb, int x, int y, int width, int height) {
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            set_pixel(fb, xx, yy, 0, 255, 0);
        }
    }
}

void grayscale(cam_fb_t* fb, int x, int y, int width, int height) {
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            get_pixel(fb, xx, yy, &r, &g, &b);
            unsigned int avg = (r + g + b) / 3;
            set_pixel(fb, xx, yy, avg, avg, avg);
        }
    }
}

void threshold(cam_fb_t* fb, int x, int y, int width, int height) {
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            get_pixel(fb, xx, yy, &r, &g, &b);
            unsigned int avg = (r + g + b) / 3;
            if(avg < 100) {
                avg = 0;
            }
            set_pixel(fb, xx, yy, avg, avg, avg);
        }
    }
}
int calculate_white_pix(int x, int y, int width, int height, cam_fb_t* fb) {
    int vysledek = 0;
    uint8_t pixel = 0;
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            get_pixel(fb, xx, yy, &pixel, &pixel, &pixel); //grayscale
            if(pixel < 255) {
                continue;
            }
            vysledek++;
        }
    }
    return vysledek;

}
int calculate_color_pix(int x, int y, int width, int height, cam_fb_t* fb, uint8_t r = 100, uint8_t g= 40, uint8_t b= 20) {
    int vysledek = 0;
    uint8_t pixel_r = 0;
    uint8_t pixel_g = 0;
    uint8_t pixel_b = 0;
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            get_pixel(fb, xx, yy, &pixel_r, &pixel_g, &pixel_b);
            if(pixel_r > r && pixel_g > g &&  pixel_b < b) {
                set_pixel(fb, xx, yy, 150, 50, 0); // clear pixel if not matching
                vysledek++;
            }

            if(xx == 500) {
                ESP_LOGI("bear", "Pixel at (500, y): R=%d, G=%d, B=%d", pixel_r, pixel_g, pixel_b);
            }
        }
    }
    return vysledek;
}

void calculate_derivation(cam_fb_t* fb, int x, int y, int width, int height) {
    
    int* pixels = new int[width * height];
    
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            if(xx < 0 || xx >= fb->width || yy < 0 || yy >= fb->height) {
                continue;
            }
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            get_pixel(fb, xx, yy, &r, &g, &b);
            int avg = (r + g + b) / 3;
            pixels[width * yy + xx] = avg;
        }
    }
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            int lx = xx - 1;
            int rx = xx + 1;
            int result = 0;
            if(lx > 0 && rx < width) {
                result = (pixels[width * yy + rx] - pixels[width * yy + lx]);
                if(result < 0) {
                    result *= -1;
                }
                if(result > 30) {
                    result = 255;
                }
            }
            uint8_t display_val = (uint8_t) (result);
            set_pixel(fb, xx, yy, display_val, display_val, display_val);
        }
    }

    unsigned int sum = 0;
    unsigned int count = 0;

    int last_xx = 0;
    int last_yy = 0;
    for(int yy = y; yy < y + height; yy++) {
        for(int xx = x; xx < x + width; xx++) {
            uint8_t pixel = 0;
            get_pixel(fb, xx, yy, &pixel, &pixel, &pixel); //grayscale
            if(pixel < 255) {
                continue;
            }
            if(last_xx > xx - 25){
                //continue; // skip if too close to last detected pixel
                set_pixel(fb, xx, yy, 0, 0, 0);
                continue; // skip if too close to last detected pixel
            }
            if (last_yy > yy - 25) {
                //continue; // skip if too close to last detected pixel
                set_pixel(fb, xx, yy, 0, 0, 0);
                continue; // skip if too close to last detected pixel
            }
            last_xx = xx;
            sum += xx;
            count++;
        }
        last_xx = 0; // reset last_xx for next row
    } 

    unsigned int avg_x = sum / count;

    //rozdelim to na 5 sektoru, tam kde je nejvice pixelu, tam bude medved, plus musi nevo navic
    fill_rect(fb, 200, 400, 3, 100);
    fill_rect(fb, 400, 400, 3, 100);
    fill_rect(fb, 600, 400, 3, 100);
    fill_rect(fb, 800, 400, 3, 100);

    ////////////// casty vyktesleny 
    ////////////////////// pocitan casti:
    int part1 =  calculate_white_pix(0, 400, 200, 100, fb);
    int part2 =  calculate_white_pix(200, 400, 200, 100, fb);
    int part3 =  calculate_white_pix(400, 400, 200, 100, fb);
    int part4 =  calculate_white_pix(600, 400, 200, 100, fb);
    int part5 =  calculate_white_pix(800, 400, 200, 100, fb);
    ESP_LOGI("bear", "Part 1: %d, Part 2: %d, Part 3: %d, Part 4: %d, Part 5: %d", part1, part2, part3, part4, part5);
    if(part3 > part1 && part3 > part2 && part3 > part4 && part3 > part5 && part3 > 60) {
        avg_x = 400 + 100; // middle of part 3
        ESP_LOGI("bear", "Detected bear in the middle");
    }

    static int64_t last_time = 0;
    int64_t t = esp_timer_get_time();
    if(t - last_time > 10000) {
        ESP_LOGI("bear", "Pint count: %d", count);
        last_time = t;
    }

    fill_rect(fb, avg_x, y, 10, height);

    fill_rect(fb, avg_x - 50, y + (height / 2), 100, 10);

    delete[] pixels;
}

void do_bear_detection(cam_fb_t* fb) {
    fill_rect(fb, 0, 400, fb->width - 1, 5);
    fill_rect(fb, 0, 500, fb->width - 1, 5);
    int a = calculate_color_pix(0, 400, fb->width - 1, 100, fb);

    // Pokud detekuješ medvěda, nastav na true:
    if (a > 100) { // příklad podmínky
        vidime_medveda = true;
    } else {
        vidime_medveda = false;
    }
    return;
}
