#pragma once
#define BMP_SETTING16 16
#define BMP_SETTING8 8
#define BMP_SETTING4 4
#define BMP_SETTING2 2
#define BMP_SETTING1 1
#define BMP_SETTING0 0
#define CONFIG 0xf5
#define CTRL_MEAS 0xf4
#include "main.h"

typedef struct Bmp280_struct
{
	uint8_t config;//control filter etc.
	uint8_t ctrl_meas;//control resolution
	GPIO_TypeDef * CS_port;
	uint32_t CS_pin;
	SPI_HandleTypeDef *spi_handle;
	int16_t pressure_cali[10];
	int16_t temp_cali[4];
}Bmp280;

void pressure_resolution_set(uint8_t resolution, Bmp280* sensor);
void temperature_resolution_set(uint8_t resolution, Bmp280* sensor);
void filter_set(uint8_t filter_setting, Bmp280* sensor);
void pressure_and_temperature_read(double* pressure_and_temperature, Bmp280* sensor);
void Bmp280_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef * port, uint32_t pin, uint8_t pressure_resolution, uint8_t temperature_resolution, uint8_t filter_setting, Bmp280* sensor);

