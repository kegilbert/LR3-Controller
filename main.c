#include <stdint.h>
#include <xc.h>
#include "custom_pin.h"
#include "TM1637Display.h"
#include "mcc_generated_files/mcc.h"

#define MOTORDIR_PIN (tPin){_PORTC_RC0_MASK, _PORTC_RC0_POSN, &PORTC}
#define MOTORSPEED_CONTROLLER_A &PWM3_LoadDutyValue
#define MOTORSPEED_CONTROLLER_B &PWM4_LoadDutyValue
#define CW  0
#define CCW 1
#define MOTOR_RAMP_RATE_MS 50

#define ADC_TO_DEC(raw_adc) (uint16_t)((((float)raw_adc * 5)/1023) * 100)

uint16_t duty_cycle(uint16_t percent);
void ramp_motor(void (*speed_controller_A)(), void (*speed_controller_B)(), uint8_t dir, uint16_t start_speed, uint16_t end_speed, uint16_t ms);

uint16_t duty_cycle(uint16_t percent) { return (uint16_t)((1023 * percent) / 100); }

/**
 * 
 * @param start_speed
 * @param end_speed
 * @param ms
 * 
 * Linearly ramp motor from start speed to end speed
 */
void ramp_motor(void (*speed_controller_A)(), void (*speed_controller_B)(), uint8_t dir, uint16_t start_speed, uint16_t end_speed, uint16_t ms) {
    int16_t step_sz = (duty_cycle(end_speed) - duty_cycle(start_speed)) / (ms / MOTOR_RAMP_RATE_MS);
    int16_t curr_speed = duty_cycle(start_speed);
    
    void (*speed_controller)() = dir ? speed_controller_A : speed_controller_B;
    void (*gnd_controller)() = dir ? speed_controller_B : speed_controller_A;
    
    gnd_controller(duty_cycle(0));
    speed_controller(duty_cycle(curr_speed));
    
    for (uint16_t i = 0; i < ms; i += MOTOR_RAMP_RATE_MS) {
        __delay_ms(MOTOR_RAMP_RATE_MS);
        curr_speed += step_sz;
        speed_controller(curr_speed);
    }
} 


void main(void) {
    // initialize the device
    SYSTEM_Initialize();

    PWM3_LoadDutyValue(duty_cycle(0));    // 0 - 1023
    PWM4_LoadDutyValue(duty_cycle(0));    // 0 - 1023
    IO_RC5_SetLow();
    IO_RC0_SetLow();
    IO_RC1_SetLow();
    IO_RA2_SetLow();
    IO_RA5_SetLow();
    
    tm1637Pins_t *display = tm1637Init(
        (tPin){_PORTA_RA2_MASK, _PORTA_RA2_POSN, &PORTA},
        (tPin){_PORTA_RA5_MASK, _PORTA_RA5_POSN, &PORTA}
    );
    
    uint16_t adc_conv = ADC_GetConversion(channel_AN0);
    
    tm1637SetBrightness(display, 3);
    tm1637DisplayDecimal(display, 0000, 1);
    
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    while (1) {
        __delay_ms(100);
        adc_conv = ADC_GetConversion(channel_AN0);
        uint16_t adc_v = ADC_TO_DEC(adc_conv);
        uint8_t dir = (adc_v > 300) ? CW : CCW;
        //IO_RC5_PORT = IO_RB6_GetValue();
        
        //tm1637DisplayDecimal(display, ADC_TO_DEC(adc_conv), 1);
        /**
         * Leave a few mV of between states to avoid fluttering between animations
         */
        if (adc_v > 390 && !display->animationRunning) {
            tm1637CycleAnimationStart(display, 1);
        } else if (adc_v < 110 && !display->animationRunning) {
            tm1637CycleAnimationStart(display, 0);
        } else if (display->animationRunning && (adc_v > 100 && adc_v < 380)){
            tm1637DisplayDecimal(display, 0000, 1);
            tm1637CycleAnimationStop(display);
        }
        if (!IO_RC4_GetValue()) {
//            ramp_motor(
//                MOTORDIR_PIN,
//                CCW, //dir,
//                MOTORSPEED_CONTROLLER,
//                0,
//                50,
//                5000
//            );
            ramp_motor(
                MOTORSPEED_CONTROLLER_A,
                MOTORSPEED_CONTROLLER_B,
                CW, //dir,
                0,
                70,
                2000
            );
            tm1637CycleAnimationStart(display, dir);
        }
    }
}
