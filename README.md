# FOC

sensorless and fixed point foc use smo and pll

## doc
define in gobal:
```c
MotorParams motor;
```

in main function:
```c
// you may modify this function for different motor
InitMotorParams1(&motor);
SetMotorDir(&motor,0);// 1 or 0,default 0
```

in pwm timer interrupt function(freq:12K)
```c
uint16_t adc_u,adc_v;
...
// read adc,format to Q12(-4096~4096)
...
motor.Iu = adc_u;
motor.Iv = adc_v;
SetMotorVq(&motor,8800);// 0~32768,default 8800
MotorStep(&motor);
// ARR = 3500
TIM1->CCR1 = motor.PWMOut[0];
TIM1->CCR2 = motor.PWMOut[1];
TIM1->CCR3 = motor.PWMOut[2];
```
