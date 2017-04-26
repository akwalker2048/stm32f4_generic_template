#ifndef HARDWARE_TB6612_H
#define HARDWARE_TB6612_H

typedef enum {TB6612_CW, TB6612_CCW, TB6612_SHORT_BRAKE, TB6612_STOP} TB6612_driver_states;
typedef enum {TB6612_MOTOR_A, TB6612_MOTOR_B} TB6612_motor_select;


/* Current configuration:
 *  1)  Both inputs and outputs tied together.
 *  2)  PWM pin will be ON/OFF
 *  3)  IN1 and IN2 will be complimentary PWM.
 */
#define TB6612_PWM_PIN    GPIO_Pin_1
#define TB6612_ENABLE_PIN GPIO_Pin_2

void TB6612_initialize(void);
void TB6612_init_gpio(void);
void TB6612_set_duty(float duty);
void TB6612_brake(void);
void TB6612_enable(void);
void TB6612_disable(void);
void TB6612_init_complimentary_pwm(void);

#endif
