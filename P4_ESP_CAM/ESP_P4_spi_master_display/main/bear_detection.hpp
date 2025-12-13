#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cv_detect.hpp"
#include <stdbool.h>

extern bool vidime_medveda;
void do_bear_detection(cam_fb_t* fb);
