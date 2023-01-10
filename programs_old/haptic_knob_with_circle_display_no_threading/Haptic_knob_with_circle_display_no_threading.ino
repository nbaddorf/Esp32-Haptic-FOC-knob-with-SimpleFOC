#include <analogWrite.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"

#include <SimpleFOC.h>
#include <PID_v1.h>


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
  Serial.begin(115200);
  Serial.println("GC9A01A Test!");
  pinMode(BL, OUTPUT);
  analogWrite(BL, 125);

  tft.begin();

  tft.setRotation(0);

  tft.fillScreen(GC9A01A_BLACK);
  
  tft.setCursor(20, 100);
  tft.setTextColor(GC9A01A_GREEN);  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setCursor(80, 130);
  tft.setTextColor(GC9A01A_BLUE); tft.setTextSize(5);
  tft.println(180);

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

  //P_haptic.SetMode(AUTOMATIC);
  //P_haptic.SetOutputLimits(-10,10);
  
  _delay(1000);

  start_rot = motor.shaft_angle;
  start_pos_rot = pos_rot;
//  start_angle = attract_angle - start_rot;
}

PIDController P_rubber{.P=10,.I=0,.D=0,.output_ramp=100000,.limit=4};

// attractor angle variable
float attract_angle = 0;
// distance between attraction points
int deg = 20;
float attractor_distance = deg*_PI/180.0; // dimp each 45 degrees

float findAttractor(float current_angle){
  return round(current_angle/attractor_distance)*attractor_distance;
}


void loop(void) {

  // main FOC algorithm function
  motor.loopFOC();

  setDetent(0.5, 15);
  motor.move(motor.PID_velocity(attract_angle - motor.shaft_angle));

  attract_angle = findAttractor(motor.shaft_angle);
  
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
