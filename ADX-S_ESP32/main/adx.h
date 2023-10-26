/*
 * adx.h - library for adx transceiver (ESP32)
 *  
 * Copyright (C) 2023 Hitoshi Kawaji (JE1RAV) <je1rav@gmail.com> 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ADX_H_
#define ADX_H_

#include <stdio.h>
#include "btstack_config.h"
#include "btstack_audio.h"

//--------------[ GPIO PINS]
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"

#define UP 17    // UP Switch (INPUT)
#define DOWN 0  // DOWN Switch (INPUT)
#define TXSW 4  // TX Switch (INPUT)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<UP) | (1ULL<<DOWN) | (1ULL<<TXSW))

#define TX 18 //TX LED (OUTPUT)
#define WSPR  25 //WSPR LED (OUTPUT)
//#define WSPR  14 //WSPR LED (OUTPUT)
#define JS8  15 //JS8 LED (OUTPUT)
#define FT4  23 //FT4 LED (OUTPUT)
#define FT8 19 //FT8 LED (OUTPUT)

#define RX  26 // RX SWITCH (OUTPUT)
//#define RX  16 // RX SWITCH (OUTPUT)
#define ATT 5 // RX Attenuator Switch (OUTPUT)
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<TX) | (1ULL<<WSPR) |(1ULL<<JS8) | (1ULL<<FT4) | (1ULL<<FT8) | (1ULL<<RX) | (1ULL<<ATT))

#define HIGH 1 
#define LOW 0

#define ESP_INTR_FLAG_DEFAULT 0

//---------------------------------------------------------------
//-------------------I2C from here--------------------------
//---------------------------------------------------------------
#include "driver/i2c.h"
#include "si5351.h"
static const char *TAG = "EXAMPLE";

#define I2C_MASTER_SCL_IO           22      //GPIO number used for I2C master clock 
#define I2C_MASTER_SDA_IO           21      //GPIO number used for I2C master data 
#define I2C_MASTER_NUM                0        //I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip
#define I2C_MASTER_FREQ_HZ          400000                     //< I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0                        //< I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0                        //< I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//------------------------------------------------------------
//-------------------ADC (I2S)from here---------------
//------------------------------------------------------------

// how often should we check for new samples from adc
#define DRIVER_POLL_INTERVAL_MS 15
// DMA settings
#define DMA_BUFFER_COUNT 10
// keep this fairly low to for low latency
//#define DMA_BUFFER_SAMPLES 100
#define DMA_BUFFER_SAMPLES 240
//#define DMA_BUFFER_SAMPLES 320
// ADC pin for RX
#define ADC_INPUT (ADC1_CHANNEL_6)     //pin34

//#define I2S_SAMPLE_RATE (16000) // Sampling frequency  
//#define I2S_DMA_BUF_LEN (480)   //every 7.5 mS at 64 kHz sampling

int16_t adc_offset_value;

//--------------[ FUNCTIONS ]
// void CAT_control();
unsigned int crossbandcheck();
int freqcheck(unsigned long frequency) ;
void band2led(int band_slot_number);
void blink3times(int band_slot_num);
void ledalloff();
int keypressed(int key_number);
void Mode_assign();
void Freq_assign();
void Band_assign();
void ManualTX();
void Band_Select();
void Calibration();

void digitalWrite(int output_pin, int on_off);
int digitalRead(int input_pin);
void delay(uint32_t ms);

static void gpio_botton_task(void* arg);
void adx_setup();

void transmit(uint32_t audio_frequency);
void receive();
void adx_force_receive();

void EEPROM_get(int addr);
void EEPROM_put(int addr);
void mode2led(int mode_number);                  //
void att2led(int on_off);

const btstack_audio_source_t *btstack_audio_esp32_source_get_instance_adx(void);
void adx_process_read_audio_data(int16_t *values, uint32_t size, int16_t audio_sampling_rate);

int16_t adc_offset();

#endif /* ADX_H_ */
