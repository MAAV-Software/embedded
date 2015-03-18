/*
 * imu_uart.h
 *
 *  Created on: Feb 21, 2015
 *      Authors: Carl Chan, Zhengjie Cui
 */

#ifndef IMU_UART_H_
#define IMU_UART_H_

#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

typedef struct _IMU_data {
	float AccX;
	float AccY;
	float AccZ;
	float AngRateX;
	float AngRateY;
	float AngRateZ;
	float MagX;
	float MagY;
	float MagZ;
	float M11;
	float M12;
	float M13;
	float M21;
	float M22;
	float M23;
	float M31;
	float M32;
	float M33;
	uint32_t Timer;
} IMU_data;

// Global data to send and receive for testing
extern volatile unsigned char g_IMU_Data_Received[79];
// 0xC2:Acc&AngRates	0xCE:EulerAng
extern volatile const unsigned char g_IMU_Address_Sent[];
extern volatile uint8_t g_IMU_Index;
extern volatile uint8_t g_IMU_DataLength;
extern volatile uint32_t g_one_sec;
// g_Status = 0(sleep) g_Status = 1(awake)
extern volatile uint8_t g_IMU_Status;

// Configure Sys Clock.
void imu_uart_config_sys_clock(void);

// Configure LED GPIO.
void imu_uart_config_LED(void);

void imu_uart_toggle_LED(uint32_t, uint32_t);

// Configure SW GPIO SW1 = PF4 SW2 = PF0.
void imu_uart_config_SW(void);

void imu_uart_SW_int_handler(void);

// Configure UART.
void imu_uart_config_uart(void);

void imu_uart_int_handler(void);

void imu_uart_send(uint32_t, const uint8_t *, uint32_t);

void imu_uart_init_dataReceived(char *);

void imu_uart_parseData(unsigned char *);

uint32_t Bytes2Int(unsigned char *raw, unsigned int i);

float Bytes2Float(unsigned char *raw, unsigned int i);

// Return data
float imu_uart_getAccX();
float imu_uart_getAccY();
float imu_uart_getAccZ();
float imu_uart_getRoll();
float imu_uart_getPitch();
float imu_uart_getYaw();
float imu_uart_getAngRateX();
float imu_uart_getAngRateY();
float imu_uart_getAngRateZ();
uint32_t imu_uart_getTimer();

#endif /* IMU_UART_H_ */
