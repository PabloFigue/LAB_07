/* 
 * File:   Prelab_07.c
 * Author: Pablo Figueroa
 *
 * Created on 4 de abril de 2023, 10:44 a.m.
 */

// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF        // Code Protection bit (Program memory code protection is enabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


/**********LIBRERIAS**********/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pic16f887.h>

/**********DEFINIR CONSTANTES**********/

#define _tmr0_value 61
#define _XTAL_FREQ 250000 
#define valPR2 194

/**********VARIABLES GLOBALES**********/

/**********PROTOTIPOS**********/

void setup(void);
void pwm_init(void);
void pwm_dc (uint16_t DutyCycle);
/*uint16_t map(uint16_t value, uint16_t fS, uint16_t fE, uint16_t tS, uint16_t tE){
    return ((long)(value-fS)*(long)(tE-tS)/((fE-fS)+tS));
}*/

/**********INTERRUPCIONES**********/

/**********CÓDIGO PRINCIPAL**********/

void main(void) 
{
    setup();
    pwm_init();
    while(1)   //loop principal
    {
        pwm_dc(50);
    }
}


/**********FUNCIONES**********/

void setup(void)
{
    OSCCON = 0x25;
    ANSEL = 0x00;
    ANSELH = 0x00;
    TRISC =0;
    PORTC = 0;
    return;
}

void pwm_init(void){

    /***************************************************
                    Setup PWM secc 11.5.7
     **************************************************
    11.5.7 SETUP FOR PWM OPERATION
    The following steps should be taken when configuring
    the CCP module for PWM operation:
    1. Disable the PWM pin (CCPx) output drivers as
    an input by setting the associated TRIS bit.
    2. Set the PWM period by loading the PR2 register.
    3. Configure the CCP module for the PWM mode
    by loading the CCPxCON register with the
    appropriate values.
    4. Set the PWM duty cycle by loading the CCPRxL
    register and DCxB<1:0> bits of the CCPxCON
    register.
    5. Configure and start Timer2:
    ? Clear the TMR2IF interrupt flag bit of the
    PIR1 register.
    ? Set the Timer2 prescale value by loading the
    T2CKPS bits of the T2CON register.
    ? Enable Timer2 by setting the TMR2ON bit of
    the T2CON register.
    6. Enable PWM output after a new PWM cycle has
    started:
    ? Wait until Timer2 overflows (TMR2IF bit of
    the PIR1 register is set).
    ? Enable the CCPx pin output driver by clearing
    the associated TRIS bit
 */    
    
    TRISC =0b100;            //paso 1
    PR2 = valPR2;           //paso 2
    CCP1CON = 0x0C;         //paso 3 y 4 configuracion PWM, CCP1CON=xx0011xx
    CCPR1L = 0x00;          //Configurar el Ciclo de Trabajo
    TMR2IF = 0;             //Paso 5, limpiar la bandera TMR2
    T2CON = 0x07;           //Post 1:1, T2_on, Pre 16, T2CON = 00000011
    while (!TMR2IF){;}    //Paso 6
    TMR2IF = 0;             //Esperar un ciclo del TMR2
    TRISC = 0b000;           //Salida PWM
    return;
    }

void pwm_dc (uint16_t DutyCycle){
    CCPR1L = DutyCycle>>2;
    CCP1CON = CCP1CON&0xCF;
    CCP1CON = CCP1CON|(0x30&(DutyCycle<<4));
    return;
}