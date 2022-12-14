/*
 * Esp8266_Demo.c
 *
 * Created: 5/2/2022 2:09:33 AM
 * Author : Ghaith
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */
#include <stdbool.h>			/* Include standard boolean library */
#include <string.h>			/* Include string library */
#include <stdio.h>			/* Include standard IO library */
#include <stdlib.h>			/* Include standard library */
#include <avr/interrupt.h>		/* Include avr interrupt header file */
#include "esp8266.h"
#include "convertToHexFunctions.h"
#include "DHT.h"

#ifndef Variables_Region
char temp[10] = "";
char hum[10]  = "";

unsigned char temperature_sensor = 0;
unsigned char humidity_sensor    = 0;
char String[10];
char String2[10];
long curr_adc_value;
float curr_v = 0.0;
long v_mv = 0;
long current_measure = 0;
float average = 0;

float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5.0;
long voltage_measure = 0;
int adc_value = 0.0;
float adc_voltage = 0.0;
float in_voltage = 0.0;

int rece_len = 0;
unsigned long prev_millis;
unsigned long current_millis;
unsigned long timeout = 10000;
int ind = 0;
char _buffer[128];
char server_response[64];
uint8_t Connect_Status;
#ifdef SEND_DEMO
uint8_t Sample = 0;
#endif

int resp_len = 0;
char response_length[6] = "";
int index = 0;
int resp_ind = 0;
int start_index = 0;
int resp_len_on = 0;

int r = 0;
int k=0;
int i=0;
uint8_t m = 0;

uint16_t checkSum_index = 0;
uint16_t check_sum = 0;


char IP_ADDRESS[22] = "192.168.43.3";
char DEFAULT_PORT[6] = "27015";
char ssid[64] = "abo adam";
char password[64] = "12345678";
uint16_t current = -1;
uint16_t voltage = -1;
uint16_t humidity = -1;
uint16_t temperature = -1;
uint16_t min_current = -1;
uint16_t max_current = -1;
uint16_t min_voltage = -1;
uint16_t max_voltage = -1;
uint16_t min_humidity = -1;
uint16_t max_humidity = -1;
uint16_t min_temperature = -1;
uint16_t max_temperature = -1;
uint16_t numalarm = -1;
uint8_t Talarm[8];

uint8_t imei_num[15] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36};

int year = 0;
int month = 0;
int day = 0;
int hour = 0;
int minute = 0;
int second = 0;

bool login_done = false;
bool message_check = false;
bool alarm_location = false;
uint8_t cntr = 0;
#endif

#ifndef DHT11_Variables_Functions

#define DHT11_PIN 6
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;

void Request()				/* Microcontroller send start pulse/request */
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);	/* set to low pin */
	_delay_ms(20);			/* wait for 20ms */
	PORTD |= (1<<DHT11_PIN);	/* set to high pin */
}

void Response()				/* receive response from DHT11 */
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}

uint8_t Receive_data()			/* receive data */
{
	for (int q=0; q<8; q++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);  /* check received bit 0 or 1 */
		_delay_us(30);
		if(PIND & (1<<DHT11_PIN))/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);	/* then its logic HIGH */
		else			/* otherwise its logic LOW */
		c = (c<<1);
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

//Convert temperature data to double temperature.
unsigned char ExtractTemperature(uint8_t Data2, uint8_t Data3)
{
	unsigned char temp = 0;
	temp = Data2;
	return temp;
}

unsigned char ExtractHumidity(uint8_t Data0, uint8_t Data1)
{
	unsigned char hum = 0;
	hum = Data0;
	return hum;
}
//---------------------------------------------//

void readDHT11Values()
{
	Request();		/* send start pulse */
	Response();		/* receive response */
	I_RH=Receive_data();	/* store first eight bit in I_RH */
	D_RH=Receive_data();	/* store next eight bit in D_RH */
	I_Temp=Receive_data();	/* store next eight bit in I_Temp */
	D_Temp=Receive_data();	/* store next eight bit in D_Temp */
	CheckSum=Receive_data();/* store next eight bit in CheckSum */
	
	if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
	{
	}
	else
	{		
		temperature_sensor = ExtractTemperature(I_Temp, D_Temp);
		humidity_sensor    = ExtractHumidity(I_RH, D_RH);
	}
}

#endif

#ifndef ADC_READ_Functions

//void adc_init(void){
	//ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));   // 16Mhz/128 = 125Khz the ADC reference clock
	//ADMUX |= (1<<REFS0);                            // Voltage reference from Avcc (5v)
	//ADCSRA |= (1<<ADEN);                            // Turn on ADC
	//ADCSRA |= (1<<ADSC);                            // Do an initial conversion because this one is the slowest and to ensure that everything is up and running
//}
//
//uint16_t Read_ADC(uint8_t ADCchannel)
//{
	////select ADC channel with safety mask
	//ADMUX=ADMUX|(ADCchannel & 0x0f);
	////single conversion mode
	//ADCSRA |= (1<<ADSC);
	//// wait until ADC conversion is complete
	//while( ADCSRA & (1<<ADSC) );
	//return ADC;
//}

void adc_init()
{
	DDRA=0x00;			/* Make ADC port as input */
	ADCSRA = 0x87;			/* Enable ADC, fr/128  */
	ADMUX = 0x40;			/* Vref: Avcc, ADC channel: 0 */
	
}

int Read_ADC(char channel)
{
	int Ain,AinLow;
	
	ADMUX=ADMUX|(channel & 0x0f);	/* Set input channel to read */

	ADCSRA |= (1<<ADSC);		/* Start conversion */
	while((ADCSRA&(1<<ADIF))==0);	/* Monitor end of conversion interrupt */
	
	_delay_us(10);
	AinLow = (int)ADCL;		/* Read lower byte*/
	Ain = (int)ADCH*256;		/* Read higher 2 bits and 
					Multiply with weight */
	Ain = Ain + AinLow;				
	return(Ain);			/* Return digital value*/
}

void readCurrentMeasure()
{
	adc_init();
	_delay_ms(10);
	average = 0;
	for(i = 0; i<200; i++)
	{
		average += Read_ADC(0);
		_delay_ms(1);
	}
	average = average/200;
	curr_adc_value = (long) average;
	v_mv = (curr_adc_value * 5000) / 1024.0;
	//current_measure = ((v_mv - 2500) * 1000) / (185);
	curr_v = ((curr_adc_value*(5.0/1024.0)) - 2.5)/(0.185 * 2);
	current_measure = (long) (curr_v * 1000);
	if(current_measure < 0)
	{
		current_measure = -1 * current_measure;
	}
	if(current_measure < 35)
	{
		current_measure = 0;
	}
}

void readVoltageMeasure()
{
	_delay_ms(10);
	adc_init();
	_delay_ms(10);
	adc_value = Read_ADC(0x02);
	adc_voltage  = (adc_value * ref_voltage) / 1024.0;
	in_voltage = adc_voltage / (R2/(R1+R2)) ;
	voltage_measure = (long)(in_voltage * 1000);
}
#endif

#ifndef parser_region

void get_actual_response()
{
	resp_len = 0;
	memset(response_length, 0, sizeof(response_length));
	index = 0;
	resp_ind = 0;
	start_index = 0;
	resp_len_on = 0;
	memset(server_response,0,sizeof(server_response));
	for(ind = 0; ind < sizeof(_buffer); ind++)
	{
		if(_buffer[ind] == ',')
		{
			resp_len_on = 1;
			continue;
		}
		if(_buffer[ind] == ':')
		{
			resp_len_on = 0;
			start_index = ind + 1;
			break;
		}
		if(resp_len_on == 1)
		{
			response_length[index] = _buffer[ind]; index++;
		}
	}
	
	resp_len = atoi(response_length);
	for(index = 0; index < resp_len; index++)
	{
		server_response[index] = _buffer[start_index + index];
	}
}


#endif

#ifndef gen_messages_region

void genLoginMessage(uint8_t imei_num[])
{
	uint16_t dataLength = 26;
	uint8_t direction = 1;
	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x01);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	for(k = 0; k < 15; k++)
	{
		to_String_X2(imei_num[k]);
		strcat(_buffer, value_2); // 1 (IMEI[k])
	}
	to_String_X4(0x00);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(_buffer, checkSum_index);
	
	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x01);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	for(k = 0; k < 15; k++)
	{
		to_String_X2(imei_num[k]);
		strcat(_buffer, value_2); // 1 (IMEI[k])
	}
	to_String_X4(check_sum);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	
	// Convert LOGIN pakage to bytes array depending on the length = 17 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
}

void genheartbeatMessage()
{
	uint16_t dataLength = 12;
	uint8_t direction = 1;
	uint8_t content = 1;
	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x02);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	to_String_X2(content);
	strcat(_buffer, value_2); // 1 (content)
	to_String_X4(0x00);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(_buffer, checkSum_index);
	
	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x02);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	to_String_X2(content);
	strcat(_buffer, value_2); // 1 (content)
	to_String_X4(check_sum);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	
	// Convert LOGIN pakage to bytes array depending on the length = 17 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
}

void genMeasurmentMessage(uint16_t CurrVal, uint16_t VoltVal, uint16_t TempVal, uint16_t HumVal)
{
	uint16_t dataLength = 19;
	uint8_t direction = 1;
	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x03);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	to_String_X4(CurrVal);
	strcat(_buffer, value_4); // 2 (CurrVal)
	to_String_X4(VoltVal);
	strcat(_buffer, value_4); // 2 (VoltVal)
	to_String_X4(TempVal);
	strcat(_buffer, value_4); // 2 (TempVal)
	to_String_X4(HumVal);
	strcat(_buffer, value_4); // 2 (HumVal)
	to_String_X4(0x00);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(_buffer, checkSum_index);
	
	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x03);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	to_String_X4(CurrVal);
	strcat(_buffer, value_4); // 2 (CurrVal)
	to_String_X4(VoltVal);
	strcat(_buffer, value_4); // 2 (VoltVal)
	to_String_X4(TempVal);
	strcat(_buffer, value_4); // 2 (TempVal)
	to_String_X4(HumVal);
	strcat(_buffer, value_4); // 2 (HumVal)
	to_String_X4(check_sum);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	
	// Convert LOGIN pakage to bytes array depending on the length = 17 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
}

void genalarmMessage(uint8_t numAlarm, uint16_t CurrVal, uint16_t VoltVal, uint16_t TempVal, uint16_t HumVal, uint8_t Talarm[])
{
	uint16_t dataLength = 20 + numAlarm;;
	uint8_t direction = 1;
	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x04);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	to_String_X2(numAlarm);
	strcat(_buffer, value_2); // 1 (numAlarm)
	for(k = 0; k < numAlarm; k++)
	{
		to_String_X2(Talarm[k]);
		strcat(_buffer, value_2); // 1 (Talarm[k])
	}
	to_String_X4(CurrVal);
	strcat(_buffer, value_4); // 2 (CurrVal)
	to_String_X4(VoltVal);
	strcat(_buffer, value_4); // 2 (VoltVal)
	to_String_X4(TempVal);
	strcat(_buffer, value_4); // 2 (TempVal)
	to_String_X4(HumVal);
	strcat(_buffer, value_4); // 2 (HumVal)
	to_String_X4(0x00);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(_buffer, checkSum_index);

	memset(_buffer, 0, sizeof(_buffer));
	to_String_X2(0xff);
	strcat(_buffer, value_2); // 1 (header)
	to_String_X2(dataLength);
	strcat(_buffer, value_2); // 1 (data length)
	to_String_X2(direction);
	strcat(_buffer, value_2); // 1 (direction)
	to_String_X2(0x04);
	strcat(_buffer, value_2); // 1 (type)
	to_String_X2(year);
	strcat(_buffer, value_2); // 1 (year)
	to_String_X2(month);
	strcat(_buffer, value_2); // 1 (month)
	to_String_X2(day);
	strcat(_buffer, value_2); // 1 (day)
	to_String_X2(hour);
	strcat(_buffer, value_2); // 1 (hour)
	to_String_X2(minute);
	strcat(_buffer, value_2); // 1 (minute)
	to_String_X2(second);
	strcat(_buffer, value_2); // 1 (second)
	to_String_X2(numAlarm);
	strcat(_buffer, value_2); // 1 (numAlarm)
	for(k = 0; k < numAlarm; k++)
	{
		to_String_X2(Talarm[k]);
		strcat(_buffer, value_2); // 1 (Talarm[k])
	}
	to_String_X4(CurrVal);
	strcat(_buffer, value_4); // 2 (CurrVal)
	to_String_X4(VoltVal);
	strcat(_buffer, value_4); // 2 (VoltVal)
	to_String_X4(TempVal);
	strcat(_buffer, value_4); // 2 (TempVal)
	to_String_X4(HumVal);
	strcat(_buffer, value_4); // 2 (HumVal)
	to_String_X4(check_sum);
	strcat(_buffer, value_4); // 4 (CheckSum)
	to_String_X2(0xcd);
	strcat(_buffer, value_2); // 1 (footer)
	
	// Convert LOGIN pakage to bytes array depending on the length = 17 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(dataLength + 2);i+=2,r++){
		_buffer[r] = (char)(convertCharToHex(_buffer[i])<<4)|convertCharToHex(_buffer[i+1]);
		checkSum_index++;
	}
}
#endif

#ifndef parse_messages_region

int package_length = 0;
uint8_t type = 0;
uint16_t error_check = 0;

void parse_received_message(uint8_t data_mat[])
{
	type = (unsigned char) data_mat[3];
	switch(type)
	{
		case 0x01:								// Login Response
		{
			package_length		   = (unsigned char) data_mat[1];
			error_check            = ((unsigned char) data_mat[25]) * 256 + (unsigned char) data_mat[26];
		}break;
		case 0x02:								// HeartBeat Response
		{
			package_length		   = (unsigned char) data_mat[1];
			min_current = ((unsigned char)data_mat[10]) * 256 + (unsigned char)data_mat[11];
			max_current = ((unsigned char)data_mat[12]) * 256 + (unsigned char)data_mat[13];
			min_voltage = ((unsigned char)data_mat[14]) * 256 + (unsigned char)data_mat[15];
			max_voltage = ((unsigned char)data_mat[16]) * 256 + (unsigned char)data_mat[17];
			min_temperature = ((unsigned char)data_mat[18]) * 256 + (unsigned char)data_mat[19];
			max_temperature = ((unsigned char)data_mat[20]) * 256 + (unsigned char)data_mat[21];
			min_humidity = ((unsigned char)data_mat[22]) * 256 + (unsigned char)data_mat[23];
			max_humidity = ((unsigned char)data_mat[24]) * 256 + (unsigned char)data_mat[25];
			error_check            = ((unsigned char) data_mat[26]) * 256 + (unsigned char) data_mat[27];
			
			//USART_TxChar(min_current & 0xff);
			//USART_TxChar((min_current >> 8) & 0xff);
			//USART_TxChar(max_current & 0xff);
			//USART_TxChar((max_current >> 8) & 0xff);
			//USART_TxChar(min_voltage & 0xff);
			//USART_TxChar((min_voltage >> 8) & 0xff);
			//USART_TxChar(max_voltage & 0xff);
			//USART_TxChar((max_voltage >> 8) & 0xff);
			
			
		}break;
		case 0x03:								// HeartBeat Response
		{
			package_length		   = (unsigned char) data_mat[1];
			error_check            = ((unsigned char) data_mat[18]) * 256 + (unsigned char) data_mat[19];
		}break;
		case 0x04:								// HeartBeat Response
		{
			package_length		   = (unsigned char) data_mat[2];
			numalarm               = (unsigned char) data_mat[10];
			error_check            = ((unsigned char) data_mat[19 + numalarm]) * 256 + (unsigned char) data_mat[20 + numalarm];
		}break;
		default:
		{
			
		}break;
	}
}

bool check_message_correctness(int len_of_data)
{
	uint16_t ch_sum = GetCrc16(server_response, len_of_data);
	//char ch_sum_char[10] = "";
	//sprintf(ch_sum_char, "%d", ch_sum);
	//USART_SendString("checksum\n");
	//USART_SendString(ch_sum_char);
	//sprintf(ch_sum_char, "%d", error_check);
	//USART_SendString("checksum_readed\n");
	//USART_SendString(ch_sum_char);
	if(ch_sum == error_check)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int compare_for_alarm(int CurrVal, int VoltVal, int TempVal, int HumVal, int Talarm[])
{
	int res = 0;
	if (CurrVal >= max_current)
	{
		res++;
		Talarm[res - 1] = 0x02;
	}
	if (CurrVal <= min_current)
	{
		res++;
		Talarm[res - 1] = 0x01;
	}
	if (VoltVal >= max_voltage)
	{
		res++;
		Talarm[res - 1] = 0x04;
	}
	if (VoltVal <= min_voltage)
	{
		res++;
		Talarm[res - 1] = 0x03;
	}
	if (TempVal >= max_temperature)
	{
		res++;
		Talarm[res - 1] = 0x06;
	}
	if (TempVal <= min_temperature)
	{
		res++;
		Talarm[res - 1] = 0x05;
	}
	if (HumVal >= max_humidity)
	{
		res++;
		Talarm[res - 1] = 0x08;
	}
	if (HumVal <= min_humidity)
	{
		res++;
		Talarm[res - 1] = 0x07;
	}
	return res;
}


#endif

int main(void)
{
	adc_init();
	DHT_Setup();
	USART_Init(115200);			/* Initiate USART with 115200 baud rate */
	init_millis(16000000UL); //frequency the atmega328p is running at
	sei();					/* Start global interrupt */
	
	/*while(1)
	{
		readDHT11Values();
		readCurrentMeasure();
		readVoltageMeasure();
		
		USART_TxChar(temperature_sensor);
		USART_TxChar(humidity_sensor);
		USART_TxChar((current_measure & 0xff));
		USART_TxChar(((current_measure >> 8) & 0xff));
		USART_TxChar((voltage_measure & 0xff));
		USART_TxChar(((voltage_measure >> 8) & 0xff));
		_delay_ms(5000);
	}*/

	while(!ESP8266_Begin());
	ESP8266_WIFIMode(BOTH_STATION_AND_ACCESPOINT);/* 3 = Both (AP and STA) */
	ESP8266_ConnectionMode(SINGLE);		/* 0 = Single; 1 = Multi */
	ESP8266_ApplicationMode(NORMAL);	/* 0 = Normal Mode; 1 = Transperant Mode */
	if(ESP8266_connected() == ESP8266_NOT_CONNECTED_TO_AP)
	ESP8266_JoinAccessPoint(ssid, password);
	//ESP8266_Start(0, IP_ADDRESS, DEFAULT_PORT);
	
	login_done = false;
	message_check = false;
	alarm_location = true;
	
	
	
	while(1)
	{
		while(login_done == false)
		{
			ESP8266_Start(0, IP_ADDRESS, DEFAULT_PORT);
			Connect_Status = ESP8266_connected();
			if(Connect_Status == ESP8266_NOT_CONNECTED_TO_AP)
			ESP8266_JoinAccessPoint(ssid, password);
			if(Connect_Status == ESP8266_TRANSMISSION_DISCONNECTED)
			ESP8266_Start(0, IP_ADDRESS, DEFAULT_PORT);
			
			memset(_buffer, 0, sizeof(_buffer));
			ESP8266_Clear();
			//sprintf(_buffer, "GET /channels/%s/feeds/last.txt", CHANNEL_ID);
			genLoginMessage(imei_num);
			ESP8266_Send_with_len(_buffer, 28);
			
			memset(_buffer, 0, sizeof(_buffer));
			prev_millis = millis();
			while(millis() - prev_millis < timeout)
			{
				rece_len = Read_Data(_buffer);
				if(rece_len > 0)
				{
					break;
				}
			}
			
			if(rece_len > 0)
			{
				get_actual_response();
				//USART_SendString("response:\n");
				//USART_SendString_len(server_response, 28);
				//USART_SendString("\r\n");
				parse_received_message(server_response);
				message_check = check_message_correctness(28);
				
				
				if(message_check == true)
				{
					//USART_SendString("login response is correct\n");
					login_done = true;
					ESP8266_Clear();
					_delay_ms(1000);
				}
				else
				{
					//USART_SendString("login response is false\n");
					login_done = false;
					ESP8266_Close();
				}
			}
			else
			{
				//USART_SendString("Timeout On Login\n");
				login_done = false;
				ESP8266_Close();
				_delay_ms(3000);
			}
		}
		
		if(login_done == true)
		{
			if(cntr == 0)
			{
				//ESP8266_Start(0, IP_ADDRESS, PORT);
				memset(_buffer, 0, sizeof(_buffer));
				//sprintf(_buffer, "GET /channels/%s/feeds/last.txt", CHANNEL_ID);
				genheartbeatMessage();
				ESP8266_Clear();
				ESP8266_Send_with_len(_buffer, 14);
				memset(_buffer, 0, sizeof(_buffer));
				prev_millis = millis();
				while(millis() - prev_millis < timeout)
				{
					rece_len = Read_Data(_buffer);
					if(rece_len > 0)
					{
						break;
					}
				}
				
				if(rece_len > 0)
				{
					get_actual_response();
					//USART_SendString("response:\n");
					//USART_SendString_len(server_response, 29);
					//USART_SendString("\r\n");
					parse_received_message(server_response);
					message_check = check_message_correctness(29);
					
					if(message_check == true)
					{
						//USART_SendString("heartbeat response is correct\n");
						ESP8266_Clear();
						cntr = 1;
					}
					else
					{
						//USART_SendString("heartbeat response is false\n");
						login_done = false;
						ESP8266_Close();
					}
				}
				else
				{
					//USART_SendString("timeout on heartbeat\n");
					login_done = false;
					ESP8266_Close();
					_delay_ms(3000);
				}
			}
			else
			{
				alarm_location = true;
				readDHT11Values();
				readCurrentMeasure();
				readVoltageMeasure();
				numalarm = compare_for_alarm(current_measure, voltage_measure, temperature_sensor * 100, humidity_sensor * 100, Talarm);
				//ESP8266_Start(0, IP_ADDRESS, PORT);
				if (numalarm > 0)
				{
					alarm_location = false;
				}
				if(alarm_location == true)
				{
					memset(_buffer, 0, sizeof(_buffer));
					//sprintf(_buffer, "GET /channels/%s/feeds/last.txt", CHANNEL_ID);
					genMeasurmentMessage(current_measure,voltage_measure,temperature_sensor * 100, humidity_sensor * 100);
					ESP8266_Clear();
					ESP8266_Send_with_len(_buffer, 21);
					memset(_buffer, 0, sizeof(_buffer));
					prev_millis = millis();
					while(millis() - prev_millis < timeout)
					{
						rece_len = Read_Data(_buffer);
						if(rece_len > 0)
						{
							break;
						}
					}
					
					if(rece_len > 0)
					{
						get_actual_response();
						//USART_SendString("response:\n");
						//USART_SendString_len(server_response, 21);
						//USART_SendString("\r\n");
						parse_received_message(server_response);
						message_check = check_message_correctness(21);
						
						if(message_check == true)
						{
							//USART_SendString("measurment response is correct\n");
							ESP8266_Clear();
							cntr = 0;
						}
						else
						{
							//USART_SendString("measurment response is false\n");
							login_done = false;
							ESP8266_Close();
						}
					}
					else
					{
						//USART_SendString("timeout on measurment\n");
						login_done = false;
						ESP8266_Close();
						_delay_ms(3000);
					}
				}
				else
				{
					memset(_buffer, 0, sizeof(_buffer));
					//sprintf(_buffer, "GET /channels/%s/feeds/last.txt", CHANNEL_ID);
					genalarmMessage(numalarm, current_measure, voltage_measure, temperature_sensor * 100, humidity_sensor * 100, Talarm);
					ESP8266_Clear();
					ESP8266_Send_with_len(_buffer, 22 + numalarm);
					memset(_buffer, 0, sizeof(_buffer));
					prev_millis = millis();
					while(millis() - prev_millis < timeout)
					{
						rece_len = Read_Data(_buffer);
						if(rece_len > 0)
						{
							break;
						}
					}
					
					if(rece_len > 0)
					{
						get_actual_response();
						//USART_SendString("response:\n");
						//USART_SendString_len(server_response, 22 + numalarm);
						//USART_SendString("\r\n");
						parse_received_message(server_response);
						message_check = check_message_correctness(22 + numalarm);
						
						if(message_check == true)
						{
							//USART_SendString("alarm response is correct\n");
							ESP8266_Clear();
							cntr = 0;
						}
						else
						{
							//USART_SendString("alarm response is false\n");
							login_done = false;
							ESP8266_Close();
						}
					}
					else
					{
						//USART_SendString("timeout on alarm\n");
						login_done = false;
						ESP8266_Close();
						_delay_ms(3000);
					}
				}
			}
			_delay_ms(4000);
		}
		
		//USART_SendString("Hello");
		//_delay_ms(1000);
	}
}