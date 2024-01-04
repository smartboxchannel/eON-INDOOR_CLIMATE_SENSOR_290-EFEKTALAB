# EFEKTA EINK2.9 Inch TEMP-HUM-PRES WEATHER MINI STATION ON NRF52

Телеграм чат DIY Devices - https://t.me/diy_devices

Продажа DIY Устройств - https://t.me/diydevmart

Temperature, humidity, pressure and light sensor with 2.9 e-ink display.

### You can buy a ready-made device by writing to the mail hello@efektalab.com

### Delivery is carried out worldwide.

### You can make your own pcb here - https://www.pcbway.com/setinvite.aspx?inviteid=550959

https://hackaday.io/project/177911-mini-weather-station-efekta-with-e-paper-display

Video: https://youtu.be/0yZfeFMDKKY

More info at http://efektalab.com/eink290

---

![FEKTA EINK2.9 Inch TEMP-HUM-PRES WEATHER MINI STATION ON NRF52](https://github.com/smartboxchannel/EFEKTA-EINK290-TEMP-HUM-PRES-WEATHER-MINI-STATION-NRF52/blob/main/Images/0002.jpg) 


---


---

### Instruction

#### First of all, I recommend installing the Arduino IDE portable (optional, but desirable)

https://www.arduino.cc/en/Guide/PortableIDE

#### 1. Install the latest arduino-nRF5 library (https://github.com/sandeepmistry/arduino-nRF5)

#### 2. Install the latest version of the MySensors library (https://github.com/mysensors/MySensors)

#### 3. Download the archive of this project to your computer

#### 4. Add support for devices of this project to the arduino-nRF5 library, the description is in the README.md file (https://github.com/smartboxchannel/EFEKTA-EINK290-TEMP-HUM-PRES-WEATHER-MINI-STATION-NRF52/blob/main/for_sandeepmistry_nRF5/README.md)

#### 5. Add support for interrupts via gpiote, for this go to the ... packages \ sandeepmistry \ hardware \ nRF5 \ 0.7.0 \ cores \ nRF5 folder, and in the WInterrupts.c file, before the void GPIOTE_IRQHandler () function, add the line: \_\_attribute\_\_ ((weak ))

#### 6. Add the libraries in the archive () https://github.com/smartboxchannel/EFEKTA-EINK290-TEMP-HUM-PRES-WEATHER-MINI-STATION-NRF52/tree/main/CODE/Arduino/libraries  of this project to the libraries folder on your computer ( path: ...\Documents\Arduino\libraries )

#### 7. Create an EINK290_1 folder on your computer under the Arduino directory (Documents \ Arduino). Add the files of this project located in the Arduino section (https://github.com/smartboxchannel/EFEKTA-EINK290-TEMP-HUM-PRES-WEATHER-MINI-STATION-NRF52/tree/main/CODE/Arduino) to the created EINK290_1 folder

#### 8. Open the EINK290_1.ino file in the Arduino IDE program, go to the MyConfig.h tab and configure according to your board version and settings of your MySensors network.

#### 9. In the main menu of the Arduino IDE go to Tools-> Boards-> Nordic Semiconductors nRF5 Boards, in the list that opens, select the EFEKTA EINK290 nRF52832 board, EFEKTA EINK290 nRF52840, EFEKTA EINK290 nRF52840 PRO (depending on your device type). In the menu of the selected board, select the type of clock crystal (internal, external), also select the Reset: Enable item.

#### 10. Click on the icon - check and then download



