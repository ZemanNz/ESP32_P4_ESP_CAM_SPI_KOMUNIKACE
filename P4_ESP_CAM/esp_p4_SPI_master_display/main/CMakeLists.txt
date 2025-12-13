idf_component_register(SRCS "mipi_isp_dsi_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_mm dsi_init esp_timer
                       )
