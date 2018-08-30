// HAPTIC SONAR PROJECT
//------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include <driverlib/pin_map.h>
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "inc/tm4c123gh6pm.h"

//------------------------------------------------------------------------------

// Global Variables
unsigned int start0=0, stop0=0, start1=0, stop1=0, start2=0, stop2=0, start3=0, stop3=0;
unsigned int temp=0, d0=0, d1=0, d2=0, d3=0, pw0=0, pw1=0, pw2=0, pw3=0;

//LOOKUP TABLE
//------------------------------------------------------------------------------
//Try implementing LUT for edge time calculation. This LUT only takes up 2kB in RAM
void PWM_Map(uint8_t i)
{
//1-D LUT with linear distrubution and offset for motor characteristics
static const uint16_t lookup[100] = {
0x0007,
0x001A,
0x003B,
0x0069,
0x00A4,
0x00EC,
0x0141,
0x01A3,
0x0213,
0x028F,
0x0319,
0x03B0,
0x0454,
0x0504,
0x05C3,
0x068E,
0x0766,
0x084B,
0x093E,
0x0A3D,
0x0B4A,
0x0C64,
0x0D8B,
0x0EBF,
0x1000,
0x114E,
0x12AA,
0x1412,
0x1587,
0x170A,
0x189A,
0x1A37,
0x1BE1,
0x1D98,
0x1F5C,
0x212D,
0x230C,
0x24F7,
0x26F0,
0x28F6,
0x2B08,
0x2D28,
0x2F55,
0x3190,
0x33D7,
0x362B,
0x388D,
0x3AFB,
0x3D77,
0x4000,
0x4296,
0x4539,
0x47E9,
0x4AA6,
0x4D70,
0x5048,
0x532C,
0x561E,
0x591D,
0x5C29,
0x5F42,
0x6268,
0x659B,
0x68DB,
0x6C29,
0x6F83,
0x72EB,
0x765F,
0x79E1,
0x7D70,
0x810C,
0x84B5,
0x886C,
0x8C2F,
0x8FFF,
0x93DD,
0x97C8,
0x9BBF,
0x9FC4,
0xA3D6,
0xA7F6,
0xAC22,
0xB05B,
0xB4A1,
0xB8F5,
0xBD56,
0xC1C3,
0xC63E,
0xCAC6,
0xCF5B,
0xD3FE,
0xD8AD,
0xDD69,
0xE233,
0xE709,
0xEBED,
0xF0DE,
0xF5DC,
0xFAE7,
0xFFFF};
if(i>70)
{
	temp = 0xFFF0;
}
else
{
	temp = lookup[i];
}
}

// TIMER INTERRUPT HANDLERS
//------------------------------------------------------------------------------

void Timer0A_Handler(void)
{
	TIMER0_ICR_R |= 0x00000001; 
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
	SysCtlDelay(54);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0x0);
	start0 = 0;
	start1 = 0;
	start2 = 0;
	start3 = 0;
	//temp = 0;
//stop0 = 0;
}

void WTimer0A_Handler(void)
{
	TimerIntClear(WTIMER0_BASE,TIMER_A);
	start0 = WTIMER0_TAR_R;
	UARTCharPut(UART0_BASE, '1');
}
void WTimer0B_Handler(void)
{
	TimerIntClear(WTIMER0_BASE,TIMER_B);
	stop0 = WTIMER0_TBR_R;
	d0 = start0 - stop0;
	d0 = round(d0/3750);

}

void WTimer1A_Handler(void)
{
	TimerIntClear(WTIMER1_BASE,TIMER_A);
	start1 = WTIMER1_TAR_R;		
}
void WTimer1B_Handler(void)
{
	TimerIntClear(WTIMER1_BASE,TIMER_B);
	stop1 = WTIMER1_TBR_R;
	d1 = start1 - stop1;
	d1 = round(d1/3750);
}

void WTimer2A_Handler(void)
{
	TimerIntClear(WTIMER2_BASE,TIMER_A);
	start2 = WTIMER2_TAR_R;		
}
void WTimer2B_Handler(void)
{
	TimerIntClear(WTIMER2_BASE,TIMER_B);
	stop2 = WTIMER2_TBR_R;
	d2 = start2 - stop2;
	d2 = round(d2/3750);
}

void WTimer3A_Handler(void)
{
	TimerIntClear(WTIMER3_BASE,TIMER_A);
	start3 = WTIMER3_TAR_R;		
}
void WTimer3B_Handler(void)
{
	TimerIntClear(WTIMER3_BASE,TIMER_B);
	stop3 = WTIMER3_TBR_R;
	d3 = start3 - stop3;
	d3 = round(d3/3750);
}
//------------------------------------------------------------------------------

// TIMER INITIALIZATION
//------------------------------------------------------------------------------
void Timer0A_Init(void)
{   
	// Change to peripheral lib API?
	volatile uint32_t ui32Loop; 
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0; // activate timer0
	ui32Loop = SYSCTL_RCGC1_R;				// Do a dummy read to insert a few cycles after enabling the peripheral.
  TIMER0_CTL_R &= ~0x00000001;     // disable timer0A during setup
  TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_PIOSC); //NEW: try setting source to make the timer values make sense
	TIMER0_CFG_R = 0x00000000;       // configure for 32-bit timer mode
  TIMER0_TAMR_R = 0x00000002;      // configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = 0x00186A00;       //NEW: Reload value in hex
	NVIC_PRI4_R &= ~0xE0000000; 	 // configure Timer0A interrupt priority as 0
  NVIC_EN0_R |= 0x00080000;     // enable interrupt 19 in NVIC (Timer0A)
	TIMER0_IMR_R |= 0x00000001;      // arm timeout interrupt
  TIMER0_CTL_R |= 0x00000001;      // enable timer0A
}

void WTimer0_Init(void)
{
	// Wide Timer Init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	TimerDisable(WTIMER0_BASE,TIMER_BOTH);
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
	TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_PIOSC);
	TimerControlEvent(WTIMER0_BASE,TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER0_BASE,TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	WTIMER0_TAILR_R = 0xFFFFFFFF; //NEW: Try loading in hex
	WTIMER0_TBILR_R = 0xFFFFFFFF;// Old Value : 0x00F42400;
	NVIC_PRI23_R &= ~0xE0E00000; //Set Int Priority to 0 for WT0 A/B (Int# 94, 95)
	TimerEnable(WTIMER0_BASE,TIMER_BOTH);
	TimerIntRegister(WTIMER0_BASE,TIMER_A, WTimer0A_Handler);
	TimerIntRegister(WTIMER0_BASE,TIMER_B, WTimer0B_Handler);
	TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT);
	TimerIntEnable(WTIMER0_BASE, TIMER_CAPB_EVENT);
	
	// CCP Config
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  GPIOPinConfigure(GPIO_PC4_WT0CCP0);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PC5_WT0CCP1);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5);
}

void WTimer1_Init(void)
{
	// Wide Timer Init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	TimerDisable(WTIMER1_BASE,TIMER_BOTH);
	TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
	TimerClockSourceSet(WTIMER1_BASE, TIMER_CLOCK_PIOSC);
	TimerControlEvent(WTIMER1_BASE,TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER1_BASE,TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	WTIMER1_TAILR_R = 0xFFFFFFFF; //NEW: Try loading in hex
	WTIMER1_TBILR_R = 0xFFFFFFFF;// Old Value : 0x00F42400;
	NVIC_PRI24_R &= ~0x0000E0E0; //Set Int Priority to 0 for WT1 A/B (Int# 96, 97)
	TimerEnable(WTIMER1_BASE,TIMER_BOTH);
	TimerIntRegister(WTIMER1_BASE,TIMER_A, WTimer1A_Handler);
	TimerIntRegister(WTIMER1_BASE,TIMER_B, WTimer1B_Handler);
	TimerIntEnable(WTIMER1_BASE, TIMER_CAPA_EVENT);
	TimerIntEnable(WTIMER1_BASE, TIMER_CAPB_EVENT);
	
	// CCP Config
  GPIOPinConfigure(GPIO_PC6_WT1CCP0);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PC7_WT1CCP1);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_7);
}

void WTimer2_Init(void)
{
	// Wide Timer Init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
	TimerDisable(WTIMER2_BASE,TIMER_BOTH);
	TimerConfigure(WTIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
	TimerClockSourceSet(WTIMER2_BASE, TIMER_CLOCK_PIOSC);
	TimerControlEvent(WTIMER2_BASE,TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER2_BASE,TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	WTIMER2_TAILR_R = 0xFFFFFFFF;
	WTIMER2_TBILR_R = 0xFFFFFFFF;
	NVIC_PRI24_R &= ~0xE0E00000; //Set Int Priority to 0 for WT2 A/B (Int# 98, 99)
	TimerEnable(WTIMER2_BASE,TIMER_BOTH);
	TimerIntRegister(WTIMER2_BASE,TIMER_A, WTimer2A_Handler);
	TimerIntRegister(WTIMER2_BASE,TIMER_B, WTimer2B_Handler);
	TimerIntEnable(WTIMER2_BASE, TIMER_CAPA_EVENT);
	TimerIntEnable(WTIMER2_BASE, TIMER_CAPB_EVENT);
	
	// CCP Config
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  GPIOPinConfigure(GPIO_PD0_WT2CCP0);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD1_WT2CCP1);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_1);
}

void WTimer3_Init(void)
{
	// Wide Timer Init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
	TimerDisable(WTIMER3_BASE,TIMER_BOTH);
	TimerConfigure(WTIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
	TimerClockSourceSet(WTIMER3_BASE, TIMER_CLOCK_PIOSC);
	TimerControlEvent(WTIMER3_BASE,TIMER_A, TIMER_EVENT_POS_EDGE);
	TimerControlEvent(WTIMER3_BASE,TIMER_B, TIMER_EVENT_NEG_EDGE);
	
	WTIMER3_TAILR_R = 0xFFFFFFFF;
	WTIMER3_TBILR_R = 0xFFFFFFFF;
	NVIC_PRI25_R &= ~0x0000E0E0; //Set Int Priority to 0 for WT3 A/B (Int# 100, 101)
	TimerEnable(WTIMER3_BASE,TIMER_BOTH);
	TimerIntRegister(WTIMER3_BASE,TIMER_A, WTimer3A_Handler);
	TimerIntRegister(WTIMER3_BASE,TIMER_B, WTimer3B_Handler);
	TimerIntEnable(WTIMER3_BASE, TIMER_CAPA_EVENT);
	TimerIntEnable(WTIMER3_BASE, TIMER_CAPB_EVENT);
	
	// CCP Config
  GPIOPinConfigure(GPIO_PD2_WT3CCP0);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_2);
	GPIOPinConfigure(GPIO_PD3_WT3CCP1);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_3);
}
//------------------------------------------------------------------------------

// PWM INITIALIZATION
//------------------------------------------------------------------------------

void PWM0_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	PWMClockSet(PWM1_BASE, PWM_SYSCLK_DIV_1);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN);
	PWMGenPeriodSet(PWM1_BASE,PWM_GEN_1, 0xFFFF);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2,0xFFFF);
	PWMGenEnable(PWM1_BASE,PWM_GEN_1);
	PWMOutputState(PWM1_BASE,PWM_OUT_2_BIT, true);
	PWMOutputInvert(PWM1_BASE,PWM_OUT_2_BIT,true);
	PWMOutputUpdateMode(PWM1_BASE, PWM_OUT_2_BIT,PWM_OUTPUT_MODE_NO_SYNC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  GPIOPinConfigure(GPIO_PE4_M1PWM2);
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
}

void PWM1_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	PWMClockSet(PWM1_BASE, PWM_SYSCLK_DIV_1);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN);
	PWMGenPeriodSet(PWM1_BASE,PWM_GEN_1, 0xFFFF);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,0xFFFF);
	PWMGenEnable(PWM1_BASE,PWM_GEN_1);
	PWMOutputState(PWM1_BASE,PWM_OUT_3_BIT, true);
	PWMOutputUpdateMode(PWM1_BASE, PWM_OUT_3_BIT,PWM_OUTPUT_MODE_NO_SYNC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  GPIOPinConfigure(GPIO_PE5_M1PWM3);
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
}

void PWM2_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	PWMClockSet(PWM1_BASE, PWM_SYSCLK_DIV_1);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN);
	PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2, 0xFFFF);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4,0xFFFF);
	PWMGenEnable(PWM1_BASE,PWM_GEN_2);
	PWMOutputState(PWM1_BASE,PWM_OUT_4_BIT, true);
	PWMOutputUpdateMode(PWM1_BASE, PWM_OUT_4_BIT,PWM_OUTPUT_MODE_NO_SYNC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  GPIOPinConfigure(GPIO_PF0_M1PWM4);
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
}

void PWM3_Init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	PWMClockSet(PWM1_BASE, PWM_SYSCLK_DIV_1);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN);
	PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2, 0xFFFF);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,0xFFFF);
	PWMGenEnable(PWM1_BASE,PWM_GEN_2);
	PWMOutputState(PWM1_BASE,PWM_OUT_5_BIT, true);
	PWMOutputUpdateMode(PWM1_BASE, PWM_OUT_5_BIT,PWM_OUTPUT_MODE_NO_SYNC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  GPIOPinConfigure(GPIO_PF1_M1PWM5);
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
}
//------------------------------------------------------------------------------

// GENERAL INITIALIZATION
//------------------------------------------------------------------------------
void UART0_Init(void)
{
	// UART for debugging and demo purposes.
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
	UARTFIFOEnable(UART0_BASE);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);	
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void PortFuncionInit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// MAIN FUNCTION
//------------------------------------------------------------------------------
int main(void)
{
	// Local Variables
	char str[4];
	
	// HW Initialization
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	PortFuncionInit();

	Timer0A_Init();
	WTimer0_Init();
	WTimer1_Init();
	WTimer2_Init();
	WTimer3_Init();
	PWM0_Init();
	PWM1_Init();
	PWM2_Init();
	PWM3_Init();
	UART0_Init();

	IntMasterEnable();
			

		while (1)
	{
		PWM_Map(d0);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2,temp);
		sprintf(str, "%3i", d0);
		UARTCharPut(UART0_BASE, 'd');
			UARTCharPut(UART0_BASE, '0');
		UARTCharPut(UART0_BASE, ':');
			UARTCharPut(UART0_BASE, ' ');
		UARTCharPut(UART0_BASE, str[0]); 
		UARTCharPut(UART0_BASE, str[1]); 
		UARTCharPut(UART0_BASE, str[2]);
		UARTCharPut(UART0_BASE, str[3]);
			UARTCharPut(UART0_BASE, '\n');
			UARTCharPut(UART0_BASE, '\r');
		PWM_Map(d1);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,temp);
		PWM_Map(d2);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4,temp);
		PWM_Map(d3);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,temp);
				sprintf(str, "%3i", d3);
		UARTCharPut(UART0_BASE, 'd');
			UARTCharPut(UART0_BASE, '3');
		UARTCharPut(UART0_BASE, ':');
			UARTCharPut(UART0_BASE, ' ');
		UARTCharPut(UART0_BASE, str[0]); 
		UARTCharPut(UART0_BASE, str[1]); 
		UARTCharPut(UART0_BASE, str[2]);
		UARTCharPut(UART0_BASE, str[3]);
			UARTCharPut(UART0_BASE, '\n');
			UARTCharPut(UART0_BASE, '\r');
	}	
}
