//#include <analogWrite.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"

#include <SimpleFOC.h>
#include <PID_v1.h>

#define configCHECK_FOR_STACK_OVERFLOW 1
#define configASSERT

struct knobState {
  int rot;
  int num_of_detent;
};

QueueHandle_t structQueue;

TaskHandle_t motorTask;
TaskHandle_t displayTask;


#define TFT_DC 15
#define TFT_CS 19
#define BL 2
#define MOSI 23
#define SCLK 18
#define RST 4

// Hardware SPI on Feather or other boards
Adafruit_GC9A01A tft(TFT_CS, TFT_DC, MOSI, SCLK, RST, -1);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 12, 14, 27);

// encoder instance
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

int num = 0;
float start_rot = 0;
float start_angle = 0.0f;
//int pos_setting = 50;
float start_pos_rot = 0.0f;
float pos_rot = 1.57f;
float old_rot = 0;
int pos_max_rotation = 3.14 * 1; //3.14

bool last_haptic_button_state = false;

double P_Input, P_Output;
double P_Setpoint = 0;

void setup() {

  structQueue = xQueueCreate(1, // Queue length
                              sizeof(struct knobState));
  if(structQueue != NULL){

    xTaskCreatePinnedToCore(motorTaskCode,"motorTask",100000,NULL,1,&motorTask,0); 
    xTaskCreatePinnedToCore(displayTaskCode,"DisplayTask",1500,NULL,2,&displayTask,1);  
  }
  
  
                              
  
  //Serial.println("GC9A01A Test!");
  

  
//  start_angle = attract_angle - start_rot;
}
/*
PIDController P_rubber{.P=10,.I=0,.D=0,.output_ramp=100000,.limit=4};

// attractor angle variable
float attract_angle = 0;
// distance between attraction points
int deg = 20;
float attractor_distance = deg*_PI/180.0; // dimp each 45 degrees

float findAttractor(float current_angle){
  return round(current_angle/attractor_distance)*attractor_distance;
}
*/

// attractor angle variable
float attract_angle = 0;
// distance between attraction points
int deg = 20;
float attractor_distance = deg*_PI/180.0; // dimp each 45 degrees

float findAttractor(float current_angle){
  return round(current_angle/attractor_distance)*attractor_distance;
}

void loop(void) {

 
  
  //tft.fillScreen(GC9A01A_BLACK);
/*
  if (motor.shaft_angle != old_rot) {
    tft.setCursor(1, 3);
    tft.setTextColor(GC9A01A_BLACK);  tft.setTextSize(3);
    tft.print(old_rot);
    tft.setTextColor(GC9A01A_GREEN);  tft.setTextSize(3);
    tft.print(motor.shaft_angle);
  }
  old_rot = motor.shaft_angle;
  */
  
  
}

void hapticButton(bool state) {
  // Play a hardcoded haptic "click"
  if (state != last_haptic_button_state) {
    float strength = state ? 5 : 1.5; // 5 1.5
    motor.move(strength);
    for (uint8_t i = 0; i < 3; i++) {
        motor.loopFOC();
        delay(1);
    }
    motor.move(-strength);
    for (uint8_t i = 0; i < 3; i++) {
        motor.loopFOC();
        delay(1);
    }
    motor.move(0);
    motor.loopFOC();
  }
  last_haptic_button_state = state;

}

template <typename T> T CLAMP(const T& value, const T& low, const T& high) {
  return value < low ? low : (value > high ? high : value); 
}

void setDetent(double detent_strength, double position_width_deg) {

  attractor_distance = position_width_deg*_PI/180.0;
  
  const float position_width_radians = position_width_deg * _PI / 180;
  const float derivative_lower_strength = detent_strength * 0.08;
  const float derivative_upper_strength = detent_strength * 0.02;
  const float derivative_position_width_lower = radians(3);
  const float derivative_position_width_upper = radians(8);
  const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength)/(derivative_position_width_upper - derivative_position_width_lower)*(position_width_radians - derivative_position_width_lower);
  motor.PID_velocity.D = CLAMP(
      raw,
      min(derivative_lower_strength, derivative_upper_strength),
      max(derivative_lower_strength, derivative_upper_strength)
  );
  motor.PID_velocity.P = detent_strength * 4;
}

void motorTaskCode(void *pvParameters) {
  (void) pvParameters;

  Serial.begin(115200);

  while (!Serial) {
    vTaskDelay(1);
  }

  PIDController P_rubber{.P=10,.I=0,.D=0,.output_ramp=100000,.limit=4};




  // initialize encoder sensor hardware
  encoder.init();

  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  motor.voltage_limit = 6;

    // velocity PI controller parameters
  motor.PID_velocity.P = 4;
  motor.PID_velocity.I = 0; //20
  motor.PID_velocity.D = 0.04;
  motor.PID_velocity.output_ramp = 10000;
  motor.velocity_limit = 10;
   //motor.LPF_velocity.Tf = 0.01f;

  // use monitoring with serial 
  //Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println("Motor ready.");
  
  vTaskDelay(1000);

  start_rot = motor.shaft_angle;
  start_pos_rot = pos_rot;

  for (;;) {
     // main FOC algorithm function
    motor.loopFOC();
    float shaftAngle = motor.shaft_angle;

    setDetent(0.5, 15);
    motor.move(motor.PID_velocity(attract_angle - shaftAngle));

    attract_angle = findAttractor(shaftAngle);
    
    struct knobState currentState;
    currentState.rot = shaftAngle * 100;
    currentState.num_of_detent = 15;

    xQueueOverwrite(structQueue, &currentState);
    //bool output = xQueueSend(structQueue, &currentState, 1);
    //Serial.println(output);
    //xQueueSendToFront(structQueue, &currentState, portMAX_DELAY);
    
    //vTaskDelay(2);
    
  }
  vTaskDelete(motorTask);
}

  void displayTaskCode(void *pvParameters) {
  (void) pvParameters;

  while (!Serial) {
    vTaskDelay(1);
  }

  //pinMode(BL, OUTPUT);
  //analogWrite(BL, 125);
  int LEDC_CHANNEL_LCD_BACKLIGHT = 0;
  ledcSetup(LEDC_CHANNEL_LCD_BACKLIGHT, 5000, 16);
  ledcAttachPin(BL, LEDC_CHANNEL_LCD_BACKLIGHT);
  ledcWrite(LEDC_CHANNEL_LCD_BACKLIGHT, UINT16_MAX / 2);
  tft.begin();

  tft.setRotation(0);

  tft.fillScreen(GC9A01A_BLACK);
  
  tft.setCursor(20, 100);
  tft.setTextColor(GC9A01A_GREEN);  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setCursor(80, 130);
  tft.setTextColor(GC9A01A_BLUE); tft.setTextSize(5);
  tft.println(180);

  int oldNum = 0;
  tft.fillScreen(GC9A01A_BLACK);

  for(;;) {
    struct knobState currentState;

    if (xQueueReceive(structQueue, &currentState, portMAX_DELAY) == pdPASS) {
      //Serial.println(currentState.rot);
     // vTaskDelay(100);
      //delay(100);
      tft.fillScreen(GC9A01A_BLACK);
      vTaskDelay(1000);
      tft.fillScreen(GC9A01A_BLUE);
      vTaskDelay(1000);
      /*
      if (oldNum != currentState.rot) {
        tft.setTextColor(GC9A01A_BLACK); tft.setTextSize(5);
        tft.setCursor(20, 100);
        tft.println(oldNum);
        tft.setTextColor(GC9A01A_BLUE); tft.setTextSize(5);
        tft.setCursor(20, 100);
        tft.println(currentState.rot / 100);
        Serial.println(uxTaskGetStackHighWaterMark(motorTask));
        vTaskDelay(100);
      }
      oldNum = currentState.rot / 100;
      */
      Serial.println(uxTaskGetStackHighWaterMark(motorTask));
        vTaskDelay(100);
    }
  }
  vTaskDelete(displayTask);
}
