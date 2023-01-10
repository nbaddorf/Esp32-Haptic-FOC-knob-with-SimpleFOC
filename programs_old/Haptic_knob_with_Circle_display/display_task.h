#pragma once

#include <Arduino.h>
//#include <TFT_eSPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include <analogWrite.h>

//#include "logger.h"
//#include "proto_gen/smartknob.pb.h"
#include "task.h"

class DisplayTask : public Task<DisplayTask> {
    friend class Task<DisplayTask>; // Allow base Task to invoke protected run()

    public:
        DisplayTask(const uint8_t task_core);
        ~DisplayTask();

        QueueHandle_t getKnobStateQueue();

        void setBrightness(uint16_t brightness);
        //void setLogger(Logger* logger);

    protected:
        void run();

    private:
        //TFT_eSPI tft_ = TFT_eSPI();
        int TFT_DC = 15;
      int TFT_CS = 19;
  int BL = 2;
  int MOSI = 23;
  int SCLK = 18;
  int RST = 4;
        Adafruit_GC9A01A tft(19, 15, 23, 18, 4, -1);

        /** Full-size sprite used as a framebuffer */
        //TFT_eSprite spr_ = TFT_eSprite(&tft_);

        QueueHandle_t knob_state_queue_;

       // PB_SmartKnobState state_;
        SemaphoreHandle_t mutex_;
        uint16_t brightness_;
        //Logger* logger_;
        //void log(const char* msg);
};
