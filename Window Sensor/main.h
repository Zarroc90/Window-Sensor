/*
 * main.h
 *
 *  Created on: 14.06.2016
 *      Author: Marvin
 */
//

#ifndef MAIN_H_
#define MAIN_H_

#include <msp430.h>
#include <MPU9250.h>
#include <LSM9DS1.h>
#include <BMX055.h>
#include <stdio.h>
#include <math.h>
#include <BMI160.h>

#define rxIDLE 0x00
#define rxSOF 0x01
#define rxCMD 0x02
#define rxLEN 0x03
#define rxDAT 0x04
#define rxCRC 0x05
#define rxPARSE 0x06
#define ACK 0x07
#define NAK 0x08
#define TXDATA 0x0A
#define RXDATA 0x0B


int test_0,test_1,test_2,test_3;
unsigned char rx_REC = 0, rx_State = rxIDLE, rx_CRC = 0, rx_CMD = 0, rx_LEN = 0, rec_LEN = 0, rxPackArray[255],rx_ACK=0,uart_rx_received=0;
volatile char received_ch;

enum sensor_type{
	MPU9250,
	LSM9DS1,
	BMX055,
	BMI160,
};

#define	M_PI		3.14159265358979323846	/* pi */

#define CS_0 	BIT0
#define CS_1	BIT1
#define CS_2	BIT2


int sensor,read;
int accelorameter_raw[3];
int gyroscope_raw[3];
int magnetometer_raw[3];
int whoami;
int MagID;
float aRes;
float gRes;
float mRes;
float ax,ay,az,gx,gy,gz,mx,my,mz;
float pitch,roll,pax,pay,paz;

float temperature;

void String_number_rightify(float number, char *str);
void Init();
void Init_MPU9250();
void Init_LSM9DS1();
void Init_BMX055();
void Init_BMI160();
int Read_Temp();
void Read_Accelorameter(int * destination);
void Read_Gyroscope(int * destination);
void Read_Magnetometer(int * destination);
int Read_Magnetometer_Id();
void Setup_Wake_on_Motion_Interrupt();
int Twos_Complement_To_Decimal(int number_of_bits,int number);
char SPI_Transceive(char cs_signal,char reg,char data);
void SPI_Write (char cs_signal,char reg, char data);
char SPI_Read (char cs_signal,char reg);
void Uart_putchar(char c);
void Uart_TransmitTxPack(char cmd,unsigned char* data, unsigned char length);
char PackCRC(unsigned char *s, unsigned char length);


#endif /* MAIN_H_ */
