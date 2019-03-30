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
#include "math.h"

int k=0;
int q=0;

float fuerzard=0;
float fuerzari=0;
float fuerzaad=0;
float fuerzaai=0;
float fuerzad=0;
float fuerzai=0;
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
            fuerzai=300*fuerzaai; //Girar parado
            fuerzad=300*fuerzaad;
        }
        else if(fuerzaai==0){
            fuerzad=300*fuerzaad; //Girar parado
            fuerzai=300*fuerzaai;
        }
    }else{
        fuerzad=(300*fuerzaad);//-fuerzard;
        fuerzai=(300*fuerzaai)-fuerzari;
    }
    ruedaderecha=1900+fuerzad;//+90
    ruedaizquierda=1900-fuerzai;//-150
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
            fuerzaad=2;
            fuerzaai=1;
        }
        if(c=='2'){
            fuerzaad=2;
            fuerzaai=2;
        }
        if(c=='3'){
            fuerzaad=1;
            fuerzaai=2;
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
            fuerzaad+=10;
            fuerzaai+=10;
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
     //HABILITAMOS ADCS
     ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);
     ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC1);
     //HABILITAMOS EL GPIOE
     ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
     ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
     ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
     //CONFIGURAR SECUENCIADOR 1
     ROM_ADCSequenceDisable(ADC0_BASE,1);
     ROM_ADCSequenceDisable(ADC1_BASE,1);
     //
     ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 2);
     ROM_TimerControlTrigger(TIMER2_BASE,TIMER_A, true);
     ROM_ADCSequenceConfigure(ADC0_BASE,1, ADC_TRIGGER_TIMER,0); //Disparo software (processor trigger)
     ROM_ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH3|ADC_CTL_IE |ADC_CTL_END);
     ROM_ADCSequenceEnable(ADC0_BASE,1); //ACTIVO LA SECUENCIA
     //
     ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_RATE_FULL, 2);
     ROM_TimerControlTrigger(TIMER3_BASE,TIMER_A, true);
     ROM_ADCSequenceConfigure(ADC1_BASE,1, ADC_TRIGGER_TIMER,0); //Disparo software (processor trigger)
     ROM_ADCSequenceStepConfigure(ADC1_BASE,1,0,ADC_CTL_CH9|ADC_CTL_IE |ADC_CTL_END);
     ROM_ADCSequenceEnable(ADC1_BASE,1); //ACTIVO LA SECUENCIA
     //Habilita las interrupciones
     ROM_ADCIntClear(ADC0_BASE,1);
     ROM_ADCIntEnable(ADC0_BASE,1);
     //
     ROM_ADCIntClear(ADC1_BASE,1);
     ROM_ADCIntEnable(ADC1_BASE,1);
     //
     ROM_IntEnable(INT_ADC0SS1);
     ROM_TimerEnable(TIMER2_BASE, TIMER_A);
     //
     ROM_IntEnable(INT_ADC1SS1);
     ROM_TimerEnable(TIMER3_BASE, TIMER_A);

}

void configADC_ISRTimer2(void)
{
    uint32_t ui32Status;
    uint32_t muestraleida[4];
    uint16_t muestra[4];
    int i,dist;

    ADCIntClear(ADC0_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ROM_ADCSequenceDataGet(ADC0_BASE,1,&muestraleida);//COGEMOS LOS DATOS GUARDADOS

    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)

    for(i=0;i<4;i++){
        muestra[i]=muestraleida[i];
    }
    dist=(muestra[0]+muestra[1]+muestra[2]+muestra[3])/2;
    if(dist<1350){
            fuerzard=0;
    }else if(dist>1350 && dist<2150){
            fuerzard=(dist-1350);
    }else if(dist>2150 ){
            fuerzard=800;
    }

}

void configADC_ISRTimer3(void)
{
    uint32_t ui32Status;
    uint32_t muestraleida[4];
    uint16_t muestra[4];
    int i,dist;

    ADCIntClear(ADC1_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ROM_ADCSequenceDataGet(ADC1_BASE,1,&muestraleida);//COGEMOS LOS DATOS GUARDADOS

    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)

    for(i=0;i<4;i++){
        muestra[i]=muestraleida[i];
    }
    dist=(muestra[0]+muestra[1]+muestra[2]+muestra[3])/2;
    if(dist<1350){
            fuerzard=0;
    }else if(dist>1350 && dist<2150){
            fuerzard=(dist-1350);
    }else if(dist>2150 ){
            fuerzard=800;

    }
}


int
main(void)
{

    uint32_t ui32Period;
    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40MHz
    //
    ROM_SysCtlPeripheralClockGating(true);
    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); // Habilitamos UART para bluetooth
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Habilitamos puerto A para la UART
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Habilitamos puerto B para los motores
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  // Habilitamos PWM0 para un motor
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // Habilitamos PWM1 para el otro motor
    //
    //
    //
    ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);           // Configuramos el pin como salida PWM
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6); // Pin motor 1
    ROM_GPIOPinConfigure(GPIO_PB7_M0PWM1);           // Configuramos el pin como salida PWM
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7); // Pin motor 2
    //
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_32); //PWM a 1.25MHz
    //
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 25000); // 25000*1/1.25MHz = 20ms periodo señal PWM
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 25000); // 25000*1/1.25MHz = 20ms periodo señal PWM
    //
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1900); //
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1900); //
    //
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true); // Habilitamos salida pwm
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Habilitamos salida pwm
    //
    //Inicializa timer 2
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerControlStall(TIMER2_BASE,TIMER_A,true);
    ui32Period = 20000000;
    ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period-1);
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerControlStall(TIMER3_BASE,TIMER_A,true);
    ui32Period = 20000000;
    ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period-1);
    //
    configADC_IniciaADC();
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
    //
    ROM_IntMasterEnable();
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER3);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1); // Habilitamos UART para bluetooth
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA); // Habilitamos puerto A para la UART
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB); // Habilitamos puerto B para los motores
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM0);  // Habilitamos PWM0 para un motor
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);  // Habilitamos PWM1 para el otro motor
    //
    ROM_SysCtlSleep();
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
