//*****************************************************************************
// no se que
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/cpu_usage.h"
#include "utils/uartstdio.h"

int k=0;
int q=0;

float fuerzard=0;
float fuerzari=0;
float fuerzaad=0;
float fuerzaai=0;
float fuerzad;
float fuerzai;
int ruedaderecha;
int ruedaizquierda;
char c;

void calculadireccion(){


    if(fuerzaad==0 || fuerzaai==0){
        if(fuerzaad==0 && fuerzaai==0){
            //Parado
            fuerzai=0;
            fuerzad=0;
        }
        else if(fuerzaad==0){
            //Parado
            fuerzai=200*fuerzaai; //Girar parado
            fuerzad=200*fuerzaad;
        }
        else if(fuerzaai==0){
            fuerzad=200*fuerzaad; //Girar parado
            fuerzai=200*fuerzaai;
        }
    }else{
        fuerzad=200*(fuerzaad-fuerzard);
        fuerzai=200*(fuerzaai-fuerzari);
    }
    ruedaderecha=1500+fuerzad;
    ruedaizquierda=1500-fuerzai;
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ruedaderecha); //
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ruedaizquierda); //

}


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        //ROM_UARTCharPutNonBlocking(UART1_BASE,
                                   //ROM_UARTCharGetNonBlocking(UART1_BASE));
        c=ROM_UARTCharGetNonBlocking(UART1_BASE);
        //
        // Blink the LED to show a character transfer is occuring.
        //
        if(c=='5'){
            fuerzaad=0;
            fuerzaai=0;
        }
        if(c=='1'){
            fuerzaad=1;
            fuerzaai=2;
        }
        if(c=='2'){
            fuerzaad=2;
            fuerzaai=2;
        }
        if(c=='3'){
            fuerzaad=2;
            fuerzaai=1;
        }
        if(c=='4'){
            fuerzaad=1;
            fuerzaai=-1;
        }
        if(c=='6'){
            fuerzaad=-1;
            fuerzaai=1;
        }
        if(c=='7'){
            fuerzaad=-2;
            fuerzaai=-1;
        }
        if(c=='8'){
            fuerzaad=-2;
            fuerzaai=-2;
        }
        if(c=='9'){
            fuerzaad=-1;
            fuerzaai=-2;
        }
        if(c=='0'){
            fuerzaad++;
            fuerzaai++;
        }
        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //

        //SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        if(c=='a'){
            ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);           // Habilitamos los generadores pwm
            ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);           // Habilitamos los generadores pwm
        }
        if(c=='b'){
            ROM_PWMGenDisable(PWM0_BASE, PWM_GEN_0);           // Habilitamos los generadores pwm
            ROM_PWMGenDisable(PWM0_BASE, PWM_GEN_1);           // Habilitamos los generadores pwm
        }

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************

void configADC_IniciaADC(void)
{
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
     //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

     //HABILITAMOS EL GPIOE
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
     //ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
     // Enable pin PE3 for ADC AIN0|AIN1|AIN2|AIN3
     ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
     ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);


     //CONFIGURAR SECUENCIADOR 1
     ROM_ADCSequenceDisable(ADC0_BASE,1);
     ROM_ADCSequenceDisable(ADC1_BASE,1);

     //Configuramos la velocidad de conversion al maximo (1MS/s)
     ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);
     ROM_TimerControlTrigger(TIMER2_BASE,TIMER_A, true);
     ROM_ADCSequenceConfigure(ADC0_BASE,1, ADC_TRIGGER_TIMER,0); //Disparo software (processor trigger)
     ROM_ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH3|ADC_CTL_IE |ADC_CTL_END);
     ROM_ADCSequenceEnable(ADC0_BASE,1); //ACTIVO LA SECUENCIA

     ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_RATE_FULL, 1);
     ROM_TimerControlTrigger(TIMER3_BASE,TIMER_A, true);
     ROM_ADCSequenceConfigure(ADC1_BASE,1, ADC_TRIGGER_TIMER,0); //Disparo software (processor trigger)
     ROM_ADCSequenceStepConfigure(ADC1_BASE,1,0,ADC_CTL_CH9|ADC_CTL_IE |ADC_CTL_END);
     ROM_ADCSequenceEnable(ADC1_BASE,1); //ACTIVO LA SECUENCIA

     //Habilita las interrupciones
     ROM_ADCIntClear(ADC0_BASE,1);
     ROM_ADCIntEnable(ADC0_BASE,1);

     ROM_ADCIntClear(ADC1_BASE,1);
     ROM_ADCIntEnable(ADC1_BASE,1);

     //ROM_IntPrioritySet(INT_ADC0SS1,3);
     ROM_IntEnable(INT_ADC0SS1);
     ROM_TimerEnable(TIMER2_BASE, TIMER_A);

     ROM_IntEnable(INT_ADC1SS1);
     ROM_TimerEnable(TIMER3_BASE, TIMER_A);

}

void configADC_ISRTimer2(void)
{
    uint32_t ui32Status;
    uint32_t muestraleida[20];
    uint16_t muestra[4];
    int i,dist;


    ui32Status = ROM_ADCIntStatus(ADC0_BASE,1, true);
    ROM_ADCIntClear(ADC0_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    //ADCIntClear(ADC0_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ROM_ADCSequenceDataGet(ADC0_BASE,1,&muestraleida);//COGEMOS LOS DATOS GUARDADOS

    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)
    //Guardamos en la cola

        dist=(muestraleida[0]+muestraleida[1])/2;
        if(dist<950){
            fuerzari=0;
        }else if(dist>950 && dist<1350){
            fuerzari=1;
        }else if(dist>1350 ){
            fuerzari=2;
        }

    //portEND_SWITCHING_ISR(higherPriorityTaskWoken);

}

void configADC_ISRTimer3(void)
{
    uint32_t ui32Status;
    uint32_t muestraleida[20];
    uint16_t muestra[1];
    int i,dist;


    ui32Status = ROM_ADCIntStatus(ADC1_BASE,1, true);
    ROM_ADCIntClear(ADC1_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    //ADCIntClear(ADC0_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ROM_ADCSequenceDataGet(ADC1_BASE,1,&muestraleida);//COGEMOS LOS DATOS GUARDADOS

    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)
    //Guardamos en la cola



        dist=(muestraleida[0]+muestraleida[1])/2;
        if(dist<1350){
            fuerzard=0;
        }else if(dist>1350 && dist<2100){
            fuerzard=1;
        }else if(dist>2100 ){
            fuerzard=2;
        }


    //portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}


int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    //ROM_FPUEnable();
    //ROM_FPULazyStackingEnable();
    uint32_t ui32Period;
    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40MHz
    //
    // Enable the GPIO port that is used for the on-board LED.
    //

    ROM_SysCtlPeripheralClockGating(true);
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    //ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    //ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    //ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); // Habilitamos UART para bluetooth
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Habilitamos puerto A para la UART
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Habilitamos puerto B para los motores
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  // Habilitamos PWM0 para un motor
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // Habilitamos PWM1 para el otro motor



    ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);           // Configuramos el pin como salida PWM
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6); // Pin motor 1
    ROM_GPIOPinConfigure(GPIO_PB7_M0PWM1);           // Configuramos el pin como salida PWM
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7); // Pin motor 2

    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32); //PWM a 1.25MHz

    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 25000); // 25000*1/1.25MHz = 20ms periodo señal PWM
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 25000); // 25000*1/1.25MHz = 20ms periodo señal PWM

    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1500); //
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1500); //

    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true); // Habilitamos salida pwm
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Habilitamos salida pwm

    //ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);           // Habilitamos los generadores pwm
    //ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);           // Habilitamos los generadores pwm

    //Inicializa timer 2
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerControlStall(TIMER2_BASE,TIMER_A,true);
    ui32Period = 2000000;
    ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period-1);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerControlStall(TIMER3_BASE,TIMER_A,true);
    ui32Period = 2000000;
    ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period-1);
    //ROM_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    //ROM_IntEnable(INT_TIMER4A);
    //ROM_TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);



    configADC_IniciaADC();

    //
    // Enable processor interrupts.
    //


    //
    // Set GPIO A0 and A1 as UART pins.
    //
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX); // Pin TXD de bluetooth
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX); // Pin RXD de bluetooth
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);


    ROM_IntMasterEnable();
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER3);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1); // Habilitamos UART para bluetooth
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA); // Habilitamos puerto A para la UART
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB); // Habilitamos puerto B para los motores
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM0);  // Habilitamos PWM0 para un motor
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);  // Habilitamos PWM1 para el otro motor

    //SysCtlDeepSleepPowerSet(SYSCTL_FLASH_LOW_POWER||SYSCTL_SRAM_LOW_POWER);
    ROM_SysCtlSleep();

    //
    // Prompt for text to be entered.
    //
    //UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
        calculadireccion();
    }
}

void Timer2IntHandler(void)
{
    // Borra la interrupción de Timer

    ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);



}

void Timer3IntHandler(void)
{
    // Borra la interrupción de Timer

    ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);



}
