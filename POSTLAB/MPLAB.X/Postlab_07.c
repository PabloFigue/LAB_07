/* 
 * File:   Postlab_07.c
 * Author: pablo
 *
 * Created on 6 de abril de 2023, 8:58 p.m.
 */

// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF          // Code Protection bit (Program memory code protection is enabled)
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
#include "PWM_Library.h"

/**********DEFINIR CONSTANTES**********/

#define tmr0_value 50    //
#define _XTAL_FREQ 500000
//#define valPR2 155

/**********VARIABLES GLOBALES**********/
int adc0, adc1, adc2;
uint8_t ondaPWM;
int valor_mapeado0, valor_mapeado1, valor_mapeado2;

/**********PROTOTIPOS**********/
void setup(void);
uint16_t map(uint16_t valor, uint16_t rango_min, uint16_t rango_max, uint16_t nuevo_min, uint16_t nuevo_max);

/**********INTERRUPCIONES**********/
void __interrupt () isr(void)
{
    if(T0IF)    //tmr0
    { 
        ondaPWM ++;         //Registro de 8 bits.
        TMR0 = tmr0_value;
        INTCONbits.T0IF = 0;
    }
    if (ADIF)   //Interrupcion del ADC
    { 
        if (ADCON0bits.CHS == 0b0000) {         // Canal 0
            adc0 = ADRESH;
            ADCON0bits.CHS = 0b0001;            // Cambia a canal 1
        } else if (ADCON0bits.CHS == 0b0001){   // Canal 1
            adc1 = ADRESH;
            ADCON0bits.CHS = 0b0010;            // Cambia a canal 2
        } else if (ADCON0bits.CHS == 0b0010){   // Canal 2
            adc2 = ADRESH;
            ADCON0bits.CHS = 0b0000;           // Cambia a canal 0
        }   
        PIR1bits.ADIF = 0;
    }
    return;
}

/**********CÓDIGO PRINCIPAL**********/

void main(void) 
{
    setup();
    PWM_config(1, 20);
    PWM_config(2, 20);
    while(1)   //loop principal
    {
        ADCON0bits.GO = 1;      //Iniciar a convertir
        while (ADIF == 0); 
        __delay_us(2);
        valor_mapeado0 = map(adc0,0,255,30,65);     //Para varial el DutyCycle entre 1ms y 2ms el mapeo debe de ser de 30 a 65
        valor_mapeado1 = map(adc1,0,255,30,65);
        valor_mapeado2 = map(adc2,4,255,0,255);
        PWM_duty(2,valor_mapeado0);
        PWM_duty(1,valor_mapeado1);
        PWM_manual(ondaPWM, valor_mapeado2, 0b00000001);
    }
}

/**********FUNCIONES**********/
uint16_t map(uint16_t valor, uint16_t rango_min, uint16_t rango_max, uint16_t nuevo_min, uint16_t nuevo_max){
    uint16_t nuevo_valor = nuevo_min + (valor - rango_min)*(nuevo_max - nuevo_min)/(rango_max - rango_min);
    return nuevo_valor;
}

void setup(void)
{
    //configuracion de entradas y salidas
    ANSELH = 0;     //Analog Select High Register PORTB<5:0> (1=Analog, 0=Digital)
    ANSEL = 0b111;  //Analog Select Register PORTA<4:0> PORTE<7:5> (1=Analog, 0=Digital)
   
    TRISA = 0b111;  //I/O Ports (1=Input, 0= Output)
    TRISB = 0;      //I/O Ports (1=Input, 0= Output)
    TRISC = 0;
    TRISD = 0;   
    
    PORTA = 0;      //Set 0
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
        
    //Configuración Oscilador Interno
    OSCCONbits.IRCF = 0b011;    //500kHz
    OSCCONbits.SCS = 1;         //Internal oscillator is used for system clock
    
    //configuración tmr0
    OPTION_REGbits.T0CS = 0;    //Selector del CLOCK del TMR0
    OPTION_REGbits.PSA = 0;     //Selector del Prescaler para TMR0
    OPTION_REGbits.PS = 0b011;  //Prescaler 1:16
    TMR0 = tmr0_value;         //Inicialización del Conteo TMR0
    
    //Configuración de las interrupciones
    INTCONbits.T0IF = 0; // Limpiar la bandera de interrupcion del TMR0
    INTCONbits.T0IE = 1; // Habilitacion de la interrupcion del TMR0
    //INTCONbits.PEIE = 1; // habilitar interrupciones perifericas
    PIR1bits.ADIF = 0;   // limpiar la bandera de interrupcion del ADC
    PIE1bits.ADIE = 1;   // habilitar interrupciones de ADC
    INTCONbits.GIE = 1;  // Usualmente la global se enciende de [ultimo]
    
    //Configuracion ADC
    ADCON0bits.ADCS = 0b10;     //Conversion Clock (Fosc/32)
    ADCON0bits.CHS = 0b0000;    //Canal 0
    __delay_us(50);
    ADCON1bits.ADFM = 0;        //justificado a la izquierda
    ADCON1bits.VCFG0 = 0;       //Vdd
    ADCON1bits.VCFG1 = 0;       //Vss
    ADCON0bits.ADON = 1;        //ADC enable
    ADIF =  0; 
    return;
}