# Meteo station
This part of the project is dedicated to the development of the meteo station on raspberry pi. It is mainly coded in C++.
You need to buy some equipment for this part. You can use all the different models of Raspberry pi and some equivalent that have the same GPIO and that interpret C and C++ as Raspi. Originally the code is done for Arduino and hasn't the same connection function as this one. The code for arduino is available on the following repo [A2_BLOC1_Project](https://github.com/noiia/A2_BLOC1_Project).
## Materials
For this part of the project i used :
- Raspberry Pi 3 B+
- Alimentation for Raspberry Pi
- SD Card of 8 Gb
- BME280
- Grove Led v2.0
- Grove light sensor
- Module Grove Base Hat Zero

I2C : BME280,     
Analog : Grove light sensor,    
Digital : Grove Led v2.0, 

I also built a box to protect it, I printed it with my 3D printer, I advice you to design yours in function of your board and the place where you store it.
## Installation
First of all, you need to have the noobs package installed on your Raspi or a C++ compatible OS.
