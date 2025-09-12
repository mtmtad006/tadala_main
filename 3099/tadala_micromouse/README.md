# UCT Micro-mouse main repository

## First Time Hardware setup
To program the micro-mouse see the README.md file in Jesse Arendse's 'MicroMouseTemplate' repo
[https://github.com/JesseJabezArendse/MicroMouseTemplate](https://github.com/JesseJabezArendse/MicroMouseTemplate).


# FAQ

## Sensors not working
All lights on (only 0 values are being read from the ToF sensors -> Check your soldering as your I2C lines are not operating.

Some lights flickering -> This could be normal sensor noise. Are you in a well lit environment? Light sensors typically dont like sunlight.

One sensor light always on -> Check the soldering on that connection of the uSensor to the Sensor PCB.
