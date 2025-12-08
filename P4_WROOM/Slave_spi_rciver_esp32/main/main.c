#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

// === KONFIGURACE WROOM (Slave) ===
#define GPIO_HANDSHAKE 2   // Modrá LED
#define GPIO_SCLK      18  
#define GPIO_MOSI      23  
#define GPIO_MISO      22  // Zelený drát
#define GPIO_CS        5   

#define RCV_HOST       SPI3_HOST 

// Callbacky
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

void my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 0);
}

void app_main(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    // Config Handshake
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_HANDSHAKE, 0);

    // Pull-upy
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    esp_err_t ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    // Buffery
    char *sendbuf = heap_caps_malloc(129, MALLOC_CAP_DMA);
    char *recvbuf = heap_caps_malloc(129, MALLOC_CAP_DMA);
    
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    printf("--- SLAVE WROOM STARTED ---\n");

    int n = 0;
    while (1) {
        // Čištění bufferů (důležité pro odstranění smetí)
        memset(recvbuf, 0, 129);
        memset(sendbuf, 0, 129);

        // Zpráva pro Mastera
        snprintf(sendbuf, 128, "Hello from Slave #%d", n);

        t.length = 128 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;

        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        printf("Received from P4: %s\n", recvbuf);
        n++;
    }
}