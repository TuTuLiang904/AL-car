# AL-car
接线：
tb6612:   stm32
3v3 -------3v3
stby-------3v3
pwma-----pa0
pwmb-----pa1
an1--------pa4
an2--------pa5
bn1--------pa6-----pa2
bn2--------pa7-----pa3
e1a--------pb6
e1b--------pb7
e2a--------pb12
e2b--------pb13
gnd--------gnd
          pb0---- 舵机1
          pb1----舵机2
          pa6----舵机3
 RDK X5 Pin 8 (TX) -> 接 STM32 PA10 (RX)

RDK X5 Pin 10 (RX) -> 接 STM32 PA9 (TX)

RDK X5 Pin 6 (GND) -> 接 STM32 GND (共地非常重要！)
