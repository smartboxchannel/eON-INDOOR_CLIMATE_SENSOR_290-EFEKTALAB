# EFEKTA EINK2.9 Inch TEMP-HUM-PRES WEATHER MINI STATION ON NRF52

Temperature, humidity, pressure and light sensor with 2.9 e-ink display.

#### Don't donate to me, it doesn't work in this world: https://paypal.me/efektalab , just buy:

Sale: https://www.tindie.com/products/diyberk/wireless-mini-weather-station-29-inch-e-paper/

Video: https://youtu.be/0yZfeFMDKKY

More info at http://efektalab.com/eink290

---

![FEKTA EINK2.9 Inch TEMP-HUM-PRES WEATHER MINI STATION ON NRF52](https://github.com/smartboxchannel/EFEKTA-EINK290-TEMP-HUM-PRES-WEATHER-MINI-STATION-NRF52/blob/main/Images/0002.jpg) 


---


#### Pay special attention:

#### Unscrupulous buyers, scammers:

##### Ion Gheorghe
##### Mississauga, ON
##### Canada




---


1. Install the latest arduino-nRF5 library (https://github.com/sandeepmistry/arduino-nRF5)

2. Install the latest version of the MySensors library (https://github.com/mysensors/MySensors)

3. Download the archive of this project to your computer

4. Add support for devices of this project to the arduino-nRF5 library, the description is in the README.md file (https://github.com/smartboxchannel/EFEKTA-EINK290-TEMP-HUM-PRES-WEATHER-MINI-STATION-NRF52/blob /main/for_sandeepmistry_nRF5/README.md)

5. Add support for interrupts via gpiote, for this go to the ... packages \ sandeepmistry \ hardware \ nRF5 \ 0.7.0 \ cores \ nRF5 folder, and in the WInterrupts.c file, before the void GPIOTE_IRQHandler () function, add the line __attribute__ ((weak ))
