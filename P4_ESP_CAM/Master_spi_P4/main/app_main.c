#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// ==========================================
// KONFIGURACE P4 (MASTER) - QQVGA VERZE
// ==========================================

// Piny (Tvoje funkční zapojení)
#define GPIO_HANDSHAKE      20
#define GPIO_SCLK           21
#define GPIO_MOSI           22
#define GPIO_MISO           24   // Zelený drát
#define GPIO_CS             25

#define SENDER_HOST         SPI2_HOST

// --- NASTAVENÍ PRO QQVGA (160x120) ---
// Šířka 160 * Výška chunku 20 * 2 bajty = 6400 Bajtů
#define CHUNK_SIZE          6400  
#define TOTAL_CHUNKS        6     // 120 řádků / 20 = 6 kusů

static QueueHandle_t rdySem;

// Přerušení od Handshake pinu (Signál od kamery)
static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    static uint32_t lasthandshaketime_us;
    uint32_t currtime_us = esp_timer_get_time();
    uint32_t diff = currtime_us - lasthandshaketime_us;
    if (diff < 1000) return; // Debounce 1ms
    lasthandshaketime_us = currtime_us;

    BaseType_t mustYield = false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t handle;

    // 1. Konfigurace sběrnice
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // DŮLEŽITÉ: Musí být větší než CHUNK_SIZE (6400)
        .max_transfer_sz = 8192, 
    };

    // 2. Konfigurace zařízení (Slave)
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 1000000,  // 1 MHz (Stabilní rychlost pro dráty)
        .duty_cycle_pos = 128,      
        
        // Mode 0 + Softwarový fix fungoval nejlépe
        .mode = 0,
        
        .spics_io_num = GPIO_CS,
        // Časování pro stabilitu signálu
        .cs_ena_pretrans = 16,
        .cs_ena_posttrans = 4,
        .input_delay_ns = 0,
        .queue_size = 3
    };

    // 3. Handshake GPIO (Input)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
    };

    rdySem = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    // Init SPI
    ret = spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    // Zapneme Pull-up na MISO pro jistotu
    gpio_set_pull_mode(GPIO_MISO, GPIO_PULLUP_ONLY);

    ret = spi_bus_add_device(SENDER_HOST, &devcfg, &handle);
    ESP_ERROR_CHECK(ret);

    // Počáteční uvolnění
    xSemaphoreGive(rdySem);

    // 4. Alokace velkých bufferů (DMA)
    uint8_t *sendbuf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
    uint8_t *recvbuf = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
    
    if (!sendbuf || !recvbuf) {
        printf("FAILED to allocate DMA memory!\n");
        return;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    int chunk_index = 0;

    printf("--- MASTER P4 STARTED (QQVGA 160x120 Mode) ---\n");

    while (1) {
        memset(recvbuf, 0, CHUNK_SIZE);
        
        // Povel pro Slave (obsah je ignorován, ale pro debug dobrý)
        snprintf((char*)sendbuf, 32, "Get QQVGA Chunk %d", chunk_index);
        
        t.length = CHUNK_SIZE * 8; // Bitů
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;

        // Čekáme, až kamera řekne "Mám data" (Handshake HIGH)
        xSemaphoreTake(rdySem, portMAX_DELAY);

        // Spustíme přenos
        ret = spi_device_transmit(handle, &t);
        
        if (ret == ESP_OK) {
            
            // 1. APLIKACE FIXU (To tam nech)
            for(int i=0; i<CHUNK_SIZE; i++) {
                recvbuf[i] = (recvbuf[i] >> 1);
            }

            // 2. VÝPIS HEX (Pro kontrolu, klidně zakomentuj, ať to nespamuje)
            // printf("Chunk %d/5 Recv: ...\n");

            // 3. ASCII ART VIZUALIZACE (Novinka!)
            // Uděláme to jen pro prostřední chunk (např. index 2 nebo 3), ať toho není moc
            if (chunk_index == 3) {
                printf("\n--- ASCII PREVIEW (Chunk 3) ---\n");
                // Projdeme data (RGB565 = 2 bajty na pixel)
                // Bereme každý 2. pixel, ať se to vejde na řádek
                for (int i = 0; i < CHUNK_SIZE; i += 4) { 
                    uint8_t high_byte = recvbuf[i]; // Horní bajt nese informaci o jasu
                    
                    // Jednoduchý převod jasu na znak
                    char c = ' ';
                    if (high_byte > 0xE0) c = '#'; // Bílá/Jasná
                    else if (high_byte > 0xA0) c = 'O';
                    else if (high_byte > 0x60) c = ':';
                    else if (high_byte > 0x30) c = '.';
                    else c = ' '; // Tma

                    printf("%c", c);
                    
                    // Zalomení řádku (QQVGA má šířku 160, my bereme každý 2. = 80 znaků)
                    if ((i / 2) % 160 == 159) printf("\n");
                }
                printf("\n-----------------------------\n");
            }
        }

        chunk_index++;
        if (chunk_index >= TOTAL_CHUNKS) {
            chunk_index = 0;
            printf("--- FRAME COMPLETE (QQVGA) ---\n");
            // Krátká pauza mezi snímky
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}