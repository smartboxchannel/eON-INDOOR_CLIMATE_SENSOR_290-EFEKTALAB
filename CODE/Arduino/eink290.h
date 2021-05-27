// ###################           Mini wither station with electronic ink display 2.9 Inch | nRF52            ############### //
//                                                                                                                           //
//        @filename   :   EFEKTA_THPEINK290_1.ino                                                                         //
//        @brief en   :   Wireless, battery-operated temperature,humidity and pressure sensor(SHT20, SI7020, HTU21D, BME280) //
//                        with electronic ink display(Good Display GDEH029A1). The extended version adds the MAX44009 light  //
//                        sensor, an active bizzer Works on nRF52.                                                           //
//        @brief ru   :   Беcпроводной, батарейный датчик температуры, влажности и давления(SHT20, SI7020, HTU21D, BME280)   //
//                        с дисплеем на электронных чернилах(Good Display GDEH029A1). В расширенной версии добавлен          //
//                        датчик света MAX44009, активный биззер. Работает на nRF52832, nRF52840.                            //
//        @author     :   Andrew Lamchenko aka Berk                                                                          //
//                                                                                                                           //
//        Copyright (C) EFEKTALAB 2020                                                                                       //
//        Copyright (c) 2014-2015 Arduino LLC.  All right reserved.                                                          //
//        Copyright (c) 2016 Arduino Srl.  All right reserved.                                                               //
//        Copyright (c) 2017 Sensnology AB. All right reserved.                                                              //
//        Copyright (C) Waveshare     August 10 2017//                                                                       //
//                                                                                                                           //
// ######################################################################################################################### //


#ifndef EINK290_H
#define EINK290_H

#include "eink.h"
#include "MyConfig.h"

// Display resolution
#define EPD_WIDTH       128
#define EPD_HEIGHT      296

// EPD2IN9 commands
#define DRIVER_OUTPUT_CONTROL                       0x01
#define BOOSTER_SOFT_START_CONTROL                  0x0C
#define GATE_SCAN_START_POSITION                    0x0F
#define DEEP_SLEEP_MODE                             0x10
#define DATA_ENTRY_MODE_SETTING                     0x11
#define SW_RESET                                    0x12
#define TEMPERATURE_SENSOR_CONTROL                  0x1A
#define BUILTINTEMPERATURE_SENSOR_CONTROL           0x18
#define MASTER_ACTIVATION                           0x20
#define DISPLAY_UPDATE_CONTROL_1                    0x21
#define DISPLAY_UPDATE_CONTROL_2                    0x22
#define WRITE_RAM                                   0x24
#define WRITE_RAM2                                  0x26
#define WRITE_VCOM_REGISTER                         0x2C
#define WRITE_LUT_REGISTER                          0x32
#define SET_DUMMY_LINE_PERIOD                       0x3A
#define SET_GATE_TIME                               0x3B
#define BORDER_WAVEFORM_CONTROL                     0x3C
#define SET_RAM_X_ADDRESS_START_END_POSITION        0x44
#define SET_RAM_Y_ADDRESS_START_END_POSITION        0x45
#define SET_RAM_X_ADDRESS_COUNTER                   0x4E
#define SET_RAM_Y_ADDRESS_COUNTER                   0x4F
#define TERMINATE_FRAME_READ_WRITE                  0xFF

extern const unsigned char lut_full_update[];
extern const unsigned char lut_partial_update[];

class Epd : EpdIf {
  public:
    int width;
    int height;

    Epd();
    ~Epd();
    int  InitV1(const unsigned char* lut);
    int  InitV2();
    void SendCommand(unsigned char command);
    void SendData(unsigned char data);
    void WaitUntilIdle(void);
    void Reset(void);
    void SetFrameMemory(
      const unsigned char* image_buffer,
      int x,
      int y,
      int image_width,
      int image_height
    );
    void SetFrameMemoryFull(
      const unsigned char* image_buffer,
      int x,
      int y,
      int image_width,
      int image_height
    );
    void SetFrameMemory(const unsigned char* image_buffer);
    void SetFrameMemory_Base(const unsigned char* image_buffer);
    void ClearFrameMemory(unsigned char color);
    void DisplayFrame(void);
    void DisplayFrameFull(void);
    void Sleep(void);

  private:
    unsigned int reset_pin;
    unsigned int dc_pin;
    unsigned int cs_pin;
    unsigned int busy_pin;
    const unsigned char* lut;

    void SetLut2(void);
    void SetLut(const unsigned char* lut);
    void SetMemoryArea(int x_start, int y_start, int x_end, int y_end);
    void SetMemoryPointer(int x, int y);
};

#endif /* EPD2IN9_H */

/* END OF FILE */
