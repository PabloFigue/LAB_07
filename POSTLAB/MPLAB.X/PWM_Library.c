
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pic16f887.h>

//  CCP1 = 1 (Bit 2 del PORTC);
//  CCP2 = 2 (Bit 1 del PORTC);

/*
 Características de la librería:
 *Prescaler del TMR2: 16
 *Postcaler del TMR2: 1
 *Frecuencia del Oscilador Interno 5Khz
 */


void PWM_config(int canal, float periodo_ms){
    int valPR2 = (int)((periodo_ms/1000)*500000/(16*4))-1; //Formula para calcular el valor de PR2 a partir del periodo de PWM.
    if (canal == 1){
        TRISC = 0b100;          //paso 1
        PR2 = (char)valPR2;           //paso 2
        CCP1CON = 0x0C;         //paso 3 y 4 configuracion Modo PWM, CCP1CON=xx0011xx (PORTB bit 2)
        CCPR1L = 0x00;          //Configurar el Ciclo de Trabajo (Inicial de 0)
    }
    else if (canal == 2){
        TRISC = 0b010;          //paso 1
        PR2 = (char)valPR2;           //paso 2
        CCP2CON = 0x0C;         //PWM mode (PORTB bit 1)
        CCPR2L = 0x00;          //Bits más significativos del Ciclo de Trabajo
    }
    TMR2IF = 0;             //Paso 5, limpiar la bandera TMR2
    T2CON = 0x07;           //Post 1:1, TMR2: ON, Prescales: 16, T2CON = 00000111
    while (!TMR2IF){;}    //Paso 6
    TMR2IF = 0;             //Esperar un ciclo del TMR2
    TRISC = 0b000;           //Salida PWM
    return;
}

void PWM_duty (int canal, int DutyCycle){
    if (canal == 1){
    CCPR1L = DutyCycle>>2;                     //Colocar los 8 bits mas significativos del DutyCycle
    CCP1CON = CCP1CON&0xCF;                     //Preparar para colocar los 2 bits menos significativos
    CCP1CON = CCP1CON|(0x30&(DutyCycle<<4));   //Colocar los 2 bits menos significativos del DutyCycle sin afectar nada mas
    }
    else if (canal == 2){
    CCPR2L = DutyCycle>>2;
    CCP2CON = CCP2CON&0xCF;
    CCP2CON = CCP2CON|(0x30&(DutyCycle<<4));
    }
    return;
}

void PWM_manual (int ondaPWM, int valor_comparador, int pin){
    if (ondaPWM <valor_comparador){
        PORTD = pin;      
    }
    else{
        PORTD = 0b00000000;
    }
    return;
}
