/*
 * main.c
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "MPU6050.h"

#define FILTER

#ifdef FILTER
double i16_Accel_Raw[3];
double i16_Gyro_Raw[3];
double i16_Kalman_Filter[3];
double Raw;
#else
static int16_t i16_Accel_Raw[3];
static int16_t i16_Gyro_Raw[3];
#endif
static uint8_t u8_Buf[100];

void WriteSerial(uint8_t* c_Buff, uint16_t ui16_len)
{
    uint16_t ui16_idx;
    for(ui16_idx = 0; ui16_idx < ui16_len; ui16_idx++)
    {
        UARTCharPut(UART0_BASE, (char)c_Buff[ui16_idx]);
    }
}

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //enable pin for LED PF2

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0); //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
    IntMasterEnable();

    MPU6050_Init();

    while(1){
#ifdef FILTER
        MPU6050_Kalman_Angle(i16_Accel_Raw);
        MPU6050_Complimentary_Angle(i16_Gyro_Raw);
        sprintf((char*)u8_Buf,"%f \t %f \t %f \t %f\n\r",i16_Accel_Raw[0], i16_Accel_Raw[1], i16_Gyro_Raw[0], i16_Gyro_Raw[1]);
//        sprintf((char*)u8_Buf,"%f \t %f \t n\r", i16_Accel_Raw[0],roll);
#else
        MPU6050_Get_Accel_Raw(i16_Accel_Raw);
        MPU6050_Get_Gyro_Raw(i16_Gyro_Raw);
        sprintf((char*)u8_Buf,"%d \t %d \t %d \t %d\n\r",i16_Accel_Raw[0], i16_Accel_Raw[1], i16_Gyro_Raw[0], i16_Gyro_Raw[1]);
#endif

        WriteSerial(u8_Buf,strlen((char*)u8_Buf));
        SysCtlDelay(SysCtlClockGet()/30);
    }
	return 0;
}
