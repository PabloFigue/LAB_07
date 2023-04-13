/* 
 * File:   LAB_07.c
 * Author: pablo
 *
 * Created on 4 de abril de 2023, 2:03 p.m.
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

/**********DEFINIR CONSTANTES**********/

#define _tmr0_value 61
#define _XTAL_FREQ 500000
#define valPR2 155

/**********VARIABLES GLOBALES**********/
int adc0;
int adc1;

/**********PROTOTIPOS**********/
void setup(void);
void pwm_init(void);
void pwm1_dc (uint16_t DutyCycle1);
void pwm2_dc (uint16_t DutyCycle2);
uint16_t map(uint16_t valor, uint16_t rango_min, uint16_t rango_max, uint16_t nuevo_min, uint16_t nuevo_max);

/**********INTERRUPCIONES**********/
void __interrupt () isr(void)
{
    if(T0IF)    //tmr0
    { 
        PORTB ++;
        TMR0 = _tmr0_value;
        INTCONbits.T0IF = 0;
    }
    if (ADIF)   //Interrupcion del ADC
    { 
        if (ADCON0bits.CHS == 0b0000) {         // Canal 0
            adc0 = ADRESH;
            ADCON0bits.CHS = 0b0001;            // cambia a canal 1
        } else if (ADCON0bits.CHS == 0b0001){   // Canal 1
            adc1 = ADRESH;
            ADCON0bits.CHS = 0b0000;            // Canal 0
        }
        PIR1bits.ADIF = 0;
    }
    return;
}

/**********CÓDIGO PRINCIPAL**********/

void main(void) 
{
    setup();
    pwm_init();
    while(1)   //loop principal
    {
        ADCON0bits.GO = 1;      //Iniciar a convertir
        while (ADIF == 0); 
        __delay_ms(10);
        int valor_mapeado1 = map(adc0,0,255,30,65);     //Para varial el DutyCycle entre 1ms y 2ms el mapeo debe de ser de 30 a 65
        int valor_mapeado2 = map(adc1,0,255,30,65);
        pwm1_dc(valor_mapeado1);
        pwm2_dc(valor_mapeado2);
    }
}

/**********FUNCIONES**********/
uint16_t map(uint16_t valor, uint16_t rango_min, uint16_t rango_max, uint16_t nuevo_min, uint16_t nuevo_max){
    uint16_t nuevo_valor = nuevo_min + (valor - rango_min)*(nuevo_max - nuevo_min)/(rango_max - rango_min);
    return nuevo_valor;
}

void pwm2_dc (uint16_t DutyCycle2){
    CCPR1L = DutyCycle2>>2;                     //Colocar los 8 bits mas significativos del DutyCycle
    CCP1CON = CCP1CON&0xCF;                     //Preparar para colocar los 2 bits menos significativos
    CCP1CON = CCP1CON|(0x30&(DutyCycle2<<4));   //Colocar los 2 bits menos significativos del DutyCycle sin afectar nada mas
    return;
}

void pwm1_dc (uint16_t DutyCycle1){
    CCPR2L = DutyCycle1>>2;
    CCP2CON = CCP2CON&0xCF;
    CCP2CON = CCP2CON|(0x30&(DutyCycle1<<4));
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
    
    TRISC = 0b110;          //paso 1
    PR2 = valPR2;           //paso 2
    CCP1CON = 0x0C;         //paso 3 y 4 configuracion Modo PWM, CCP1CON=xx0011xx (PORTB bit 2)
    CCP2CON = 0x0C;         //PWM mode (PORTB bit 1)
    CCPR1L = 0x00;          //Configurar el Ciclo de Trabajo (Inicial de 0)
    CCPR2L = 0x00;          //Bits más significativos del Ciclo de Trabajo
    TMR2IF = 0;             //Paso 5, limpiar la bandera TMR2
    T2CON = 0x07;           //Post 1:1, TMR2: ON, Prescales: 16, T2CON = 00000111
    while (!TMR2IF){;}    //Paso 6
    TMR2IF = 0;             //Esperar un ciclo del TMR2
    TRISC = 0b000;           //Salida PWM
    return;
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
    OSCCONbits.IRCF = 0b011;    //8MHz
    OSCCONbits.SCS = 1;         //Internal oscillator is used for system clock
    
    //configuración tmr0
    OPTION_REGbits.T0CS = 0;    //Selector del CLOCK del TMR0
    OPTION_REGbits.PSA = 0;     //Selector del Prescaler para TMR0
    OPTION_REGbits.PS = 0b111;  //Prescaler 1:256
    TMR0 = _tmr0_value;         //Inicialización del Conteo TMR0
    
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
    