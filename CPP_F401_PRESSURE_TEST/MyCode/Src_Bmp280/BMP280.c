#include "main.h"
#include "BMP280.h"
void pressure_resolution_set(uint8_t resolution, Bmp280* sensor)
{
	uint8_t ctrl_meas=sensor->ctrl_meas;
	uint8_t command[2]={CTRL_MEAS & 0x7f, ctrl_meas};
	uint8_t receive[2];
	switch(resolution)
	{
	case 0: ctrl_meas=(ctrl_meas&0B11100011)|(0x00<<2);
	break;
	case 1: ctrl_meas=(ctrl_meas&0B11100011)|(0x01<<2);
	break;
	case 2: ctrl_meas=(ctrl_meas&0B11100011)|(0x02<<2);
	break;
	case 4: ctrl_meas=(ctrl_meas&0B11100011)|(0x03<<2);
	break;
	case 8: ctrl_meas=(ctrl_meas&0B11100011)|(0x04<<2);
	break;
	case 16: ctrl_meas=(ctrl_meas&0B11100011)|(0x05<<2);
	break;
	default:
		break;
	}
	sensor->ctrl_meas=ctrl_meas;
	command[1]=ctrl_meas;
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(sensor->spi_handle, command, receive, 2, 100);
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void temperature_resolution_set(uint8_t resolution, Bmp280* sensor)
{
	uint8_t ctrl_meas=sensor->ctrl_meas;
	uint8_t command[2]={CTRL_MEAS & 0x7f, ctrl_meas};
	uint8_t receive[2];
	switch(resolution)
	{
	case 0: ctrl_meas=(ctrl_meas&0B00011111)|(0x00<<5);
	break;
	case 1: ctrl_meas=(ctrl_meas&0B00011111)|(0x01<<5);
	break;
	case 2: ctrl_meas=(ctrl_meas&0B00011111)|(0x02<<5);
	break;
	case 4: ctrl_meas=(ctrl_meas&0B00011111)|(0x03<<5);
	break;
	case 8: ctrl_meas=(ctrl_meas&0B00011111)|(0x04<<5);
	break;
	case 16: ctrl_meas=(ctrl_meas&0B00011111)|(0x05<<5);
	break;
	default:
		break;
	}
	sensor->ctrl_meas=ctrl_meas;
	command[1]=ctrl_meas;
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(sensor->spi_handle, command, receive, 2, 100);
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void filter_set(uint8_t filter_setting, Bmp280* sensor)
{
	uint8_t config=sensor->config;
	uint8_t command[2]={CONFIG & 0x7f, config};
	uint8_t receive[2];
	switch(filter_setting)
	{
	case 0: config=(config&0x00)|(0x00<<2);
	break;
	case 1: config=(config&0x00)|(0x01<<2);
	break;
	case 2: config=(config&0x00)|(0x02<<2);
	break;
	case 4: config=(config&0x00)|(0x03<<2);
	break;
	case 8: config=(config&0x00)|(0x04<<2);
	break;
	case 16: config=(config&0x00)|(0x05<<2);
	break;
	default:
		break;
	}
	sensor->config=config;
	command[1]=config;
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(sensor->spi_handle, command, receive, 2, 100);
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void pressure_and_temperature_read(double* pressure_and_temperature, Bmp280* sensor)
{
	int16_t* temp_cali=sensor->temp_cali;
	int16_t* pressure_cali=sensor->pressure_cali;
	uint8_t read_pressure[7]={0xf7, 0x00, 0x00, 0X00, 0x00, 0x00, 0X00};
	uint8_t receive[7];
	uint32_t pressure_raw, temp_raw;
	double var1, var2, p, tfine, T;
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(sensor->spi_handle, read_pressure, receive, 7, 100);
	HAL_GPIO_WritePin(sensor->CS_port,sensor-> CS_pin, GPIO_PIN_SET);
	HAL_Delay(10);
	pressure_raw=(((uint32_t)receive[1]<<12)|((uint32_t)receive[2]<<4)|((uint32_t)receive[3]>>4));
	temp_raw=((uint32_t)receive[4]*256*256+(uint32_t)receive[5]*256+(uint32_t)receive[6])/16;
	var1=(((double)temp_raw)/16384.0-((double)((uint16_t)temp_cali[1]))/1024.0)    *(double)temp_cali[2];
	var2=((((double)temp_raw)/131072.0-((double)((uint16_t)temp_cali[1]))/8192.0)*(((double)temp_raw)/131072.0-((double)((uint16_t)temp_cali[1]))/8192.0))*((double)temp_cali[3]);
	tfine=var1+var2;
	T=tfine/5120.0;
	var1=tfine/2-64000;
	var2=var1*var1*((double)pressure_cali[6])/32768.0;
	var2=var2+var1*((double)pressure_cali[5])*2;
	var2=(var2/4)+(((double)pressure_cali[4])*65536.0);
	var1=(((double)pressure_cali[3])*var1*var1/524288.0+((double)pressure_cali[2])*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)((uint16_t)pressure_cali[1]));
	p=1048576.0-(double)pressure_raw;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)pressure_cali[9])*p*p/2147483648.0;
	var2=p*((double)pressure_cali[8])/32768.0;
	p=p+(var1+var2+((double)pressure_cali[7]))/16.0;
	pressure_and_temperature[0]=p;
	pressure_and_temperature[1]=T;
}

void Bmp280_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef * port, uint32_t pin, uint8_t pressure_resolution, uint8_t temperature_resolution, uint8_t filter_setting, Bmp280* sensor)
{
	uint8_t send[25];
	uint8_t receive[25];
	sensor->spi_handle=hspi;
	uint8_t command[4];
	sensor->config=0B00010100;
	sensor->ctrl_meas=0B01010111;
	sensor->CS_port=port;
	sensor->CS_pin=pin;
	pressure_resolution_set(pressure_resolution, sensor);
	temperature_resolution_set(temperature_resolution, sensor);
	filter_set(filter_setting, sensor);
	//read calibration data
	send[0]=0x88;
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(sensor->spi_handle, send, receive, 25, 1000);
	HAL_GPIO_WritePin(sensor->CS_port, sensor->CS_pin, GPIO_PIN_SET);
	HAL_Delay(10);
	for(uint8_t i=0; i<10; i++)
	{
		(sensor->pressure_cali)[i]=(uint16_t)receive[2*i+5]+256*(uint16_t)receive[2*i+6];
	}
	for(uint8_t i=1; i<4; i++)
	{
		(sensor->temp_cali)[i]=(uint16_t)receive[2*i-1]+256*(uint16_t)receive[2*i];
	}
}



