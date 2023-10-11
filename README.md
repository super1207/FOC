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
int32_t adc_u,adc_v;
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
## ..
![image](https://github.com/super1207/FOC/assets/30586004/b9abf442-6cf1-443c-b901-45823610e5f8)
![image](https://github.com/super1207/FOC/assets/30586004/47f3a4e8-000a-4547-8959-06532046b325)

