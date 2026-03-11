#pragma once

#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_codec_dev.h"

void esp_es8311_port_init(i2c_master_bus_handle_t bus_handle);
void esp_es8311_test(void);
esp_codec_dev_handle_t esp_es8311_get_output_dev(void);
esp_codec_dev_handle_t esp_es8311_get_input_dev(void);


