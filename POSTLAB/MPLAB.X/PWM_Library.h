/* 
 * File:   PWM_Library.h
 * Author: pablo
 *
 * Created on 12 de abril de 2023, 1:06 p.m.
 */

#ifndef PWM_LIBRARY_H
#define	PWM_LIBRARY_H

#ifdef	__cplusplus
extern "C" {
#endif

void PWM_config(int canal, float periodo_ms);
void PWM_duty (int canal, int DutyCycle);
void PWM_manual (int ondaPWM, int valor_comparador, int pin);


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_LIBRARY_H */

