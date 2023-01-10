#include "SPI.h"
#include <littlefs_api.h>
#include <esp_littlefs.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>  // Widget library
// Include the jpeg decoder library
#include <TJpg_Decoder.h>
#include <SimpleFOC.h>
#include <PID_v1.h>

#include "NotoSansBold36.h"
#define AA_FONT_LARGE NotoSansBold36
// Jpeg image array attached to this sketch
#include "myDial.h"

uint16_t* tft_buffer;
bool      buffer_loaded = false;
uint16_t  spr_width = 0;
uint16_t  bg_color = 0;

#define LIGHT_BLUE_BG 0x64BF
#define DARK_GREEN 0x03D5
#define DARK_BLUE 0x022F
#define DARK_SIAN 0x034F

#define NEEDLE_LENGTH 35  // Visible length
#define NEEDLE_WIDTH   6  // Width of needle - make it an odd number
#define NEEDLE_RADIUS 90  // Radius at tip
#define NEEDLE_COLOR1 TFT_MAROON  // Needle periphery colour
#define NEEDLE_COLOR2 TFT_RED     // Needle centre colour
#define DIAL_CENTRE_X 120
#define DIAL_CENTRE_Y 120

#define configCHECK_FOR_STACK_OVERFLOW 1
#define configASSERT

struct knobState {
  float rot;
  int num_of_detent;
  //float detent_rad_width;
  //float max_rot;
  //float start_num;
  //float end_num;
};

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite ark    = TFT_eSprite(&tft); 
TFT_eSprite clr    = TFT_eSprite(&tft);
TFT_eSprite spr    = TFT_eSprite(&tft); 



QueueHandle_t knobPositionQueue;

TaskHandle_t motorTask;
TaskHandle_t displayTask;

#define TFT_DC 15
#define TFT_CS 19
#define BL 2
#define MOSI 23
#define SCLK 18
#define RST 4

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 12, 14, 27);

// encoder instance
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

float detent_set = 30.00 * (_PI/180.00);
float pos_rot = 0.0f;//0.0f; //1.58 // this is the rotation variable when rubberbanding.
int rubber_temp_value = 5;

float detent = 0.0f;
bool is_detent_calibrated = false;
float start_rot = 0.0f;
float start_pos_rot = 0.0f;
//float old_rot = 0.0f;
//float pos_max_rotation = 4.18879;
float pos_max_rotation = (240.00 * (_PI/180.00)); //3.14
float pos_min_rotation = 0;
float offsetRad = 0;
int current_detent_pos = 0;
bool motor_has_loaded = false;
bool motor_calibration_error = false;
int button_pin = 5;
bool old_button_state = false;

float attract_angle = 0.0f;
float old_attract_angle = 0.0f;
float attractor_distance = 0.0f;

bool last_haptic_button_state = false;

double P_Input, P_Output;
double P_Setpoint = 0;

PIDController P_rubber{.P=10,.I=0,.D=0,.output_ramp=100000,.limit=4};

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // Return 1 to decode next block
  return 1;
}

void setup() {

  

  knobPositionQueue = xQueueCreate(1, // Queue length
                              sizeof(struct knobState));
  if(knobPositionQueue != NULL){

    xTaskCreatePinnedToCore(motorTaskCode,"motorTask",100000,NULL,1,&motorTask,0); 
    xTaskCreatePinnedToCore(displayTaskCode,"DisplayTask",1500,NULL,2,&displayTask,1);  
  }
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



float findAttractor(float current_angle){
  float attractor = round(current_angle/attractor_distance)*attractor_distance;
  //current_detent_pos
  return attractor;
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

int runDetent(double detent_strength, double position_width_rad) {
  detent = position_width_rad;
  attractor_distance = position_width_rad;
  float shaftAngle = motor.shaft_angle + offsetRad;
  attract_angle = findAttractor(shaftAngle);
  if (is_detent_calibrated) {
    current_detent_pos = 0;
    is_detent_calibrated = false; 
  } else {
    if (attract_angle != old_attract_angle) {
      current_detent_pos = (attract_angle - shaftAngle) > (old_attract_angle - shaftAngle)? current_detent_pos + 1 : current_detent_pos - 1;
    }
  }
  //const float position_width_radians = position_width_deg * _PI / 180;
  const float position_width_radians = position_width_rad;
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

  motor.move(motor.PID_velocity(attract_angle - shaftAngle));
  old_attract_angle = attract_angle;
  return current_detent_pos;
}

int runDetentWithEndstops(double detent_strength, double position_width_rad, double input_value) {
  detent = position_width_rad;
  attractor_distance = position_width_rad;
  float shaftAngle = motor.shaft_angle + offsetRad;
  attract_angle = findAttractor(shaftAngle);
  if (is_detent_calibrated) {
    start_rot = shaftAngle;
    current_detent_pos = input_value;
    //start_pos_rot = (position_width_rad * input_value) * (_PI/180.00);
    start_pos_rot = (position_width_rad * input_value);
    is_detent_calibrated = false; 
  } else {
    if (attract_angle != old_attract_angle) {
      current_detent_pos = (attract_angle - shaftAngle) > (old_attract_angle - shaftAngle)? current_detent_pos + 1 : current_detent_pos - 1;
    }
  }
  //const float position_width_radians = position_width_deg * _PI / 180;
  const float position_width_radians = position_width_rad;
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
  if (current_detent_pos <= pos_max_rotation && current_detent_pos >= pos_min_rotation) {
    motor.move(motor.PID_velocity(attract_angle - shaftAngle));
  } //else if (current_detent_pos >= pos_max_rotation || current_detent_pos <= pos_min_rotation) {
    //motor.move(0);
  //}
  else {
    //Serial.println("end stops");
    float input = start_pos_rot - (start_rot - shaftAngle);
    float pos_max_rad = (position_width_rad * pos_max_rotation);
    float pos_min_rad = (position_width_rad * pos_min_rotation);
    float constrained_rot = constrain(input, pos_min_rad, pos_max_rad);
    //Serial.println(pos_rot >= (detent / 2.00) && pos_rot <= (pos_max_rotation - (detent / 2.00)));
    //Serial.println(pos_max_rad);
    //Serial.println(pos_min_rad);
    
    float pid_input = input - constrained_rot;
    if (abs(pid_input) <= 0.05) {
      pid_input = 0;
    }
    
    motor.move(P_rubber(pid_input) * -1);
    //motor.move(0);
  }

  
  old_attract_angle = attract_angle;
  return constrain(current_detent_pos, pos_min_rotation, pos_max_rotation);
  
}

void calibrate_detent(double detent_width_rad, double min_rotation, double max_detent_rotation) {
  attractor_distance = detent_width_rad;
  pos_min_rotation = min_rotation;
  float shaftAngle = motor.shaft_angle;
  offsetRad = findAttractor(shaftAngle) - shaftAngle;
  is_detent_calibrated = true;

   //start_rot = shaftAngle + offsetRad;
   //start_pos_rot = pos_rot;
   pos_max_rotation = max_detent_rotation;
   //pos_max_rotation = (detent_width_rad * max_detent_rotation) * (_PI/180.00)); //3.14
  //Serial.println(offsetRad);
}

void motorTaskCode(void *pvParameters) {
  (void) pvParameters;

  Serial.begin(115200);

  while (!Serial) {
    vTaskDelay(1);
  }

  pinMode(button_pin, INPUT_PULLUP);
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
  motor_calibration_error = !motor.initFOC();
 // Serial.println;
  Serial.println("Motor ready.");
  //Serial.println(pos_max_rotation);
  //Serial.println(detent);
  //float number_of_detent = pos_max_rotation / detent;
  //Serial.println(number_of_detent);

  calibrate_detent(detent_set, 0, 3);
  old_button_state = digitalRead(button_pin);;
  vTaskDelay(1000);
  //motor.loopFOC();
  //vTaskDelay(1000);
  
  motor_has_loaded = true;
  for (;;) {
     // main FOC algorithm function
    motor.loopFOC();
    float shaftAngle = motor.shaft_angle;
    //Serial.println(shaftAngle);

    //setDetent(0.7, 15);

    //float deg_rot = pos_rot * (180.00 / PI);
    //float deg_max_rot = pos_max_rotation * (180.00 / PI);
    bool buttonState = digitalRead(button_pin);
    //Serial.println(buttonState);
    if (buttonState != old_button_state && !buttonState) {
      Serial.println("button pressed");
    }
  
   //rubber_temp_value = runDetentWithEndstops(0.9, detent_set, rubber_temp_value);
    rubber_temp_value = runDetent(0.9, detent_set)%int(pos_max_rotation); 
    rubber_temp_value = rubber_temp_value < 0? pos_max_rotation + rubber_temp_value : rubber_temp_value;
   
    old_button_state = buttonState;

      //motor.move(motor.PID_velocity(attract_angle - shaftAngle));
      //} else {
      //motor.move(0);
       // motor.move
      //}
    //} else {
    //  motor.move(P_rubber(pid_input) * -1);
   // }
    

    //attract_angle = findAttractor(shaftAngle);

    struct knobState currentState;
    currentState.rot = rubber_temp_value;
    //currentState.num_of_detent = number_of_detent;
    //currentState.detent_rad_width = detent;
    //currentState.max_rot = pos_max_rotation;
    //currentState.start_num = 0;
    //currentState.end_num = pos_max_rotation;

    xQueueOverwrite(knobPositionQueue, &currentState);
    //bool output = xQueueSend(structQueue, &currentState, 1);
    //Serial.println(output);
    //xQueueSendToFront(structQueue, &currentState, portMAX_DELAY);
    
    vTaskDelay(1);
    
  }
  vTaskDelete(motorTask);
}

  void displayTaskCode(void *pvParameters) {
  (void) pvParameters;

  while (!Serial) {
    vTaskDelay(1);
  }

  tft.init();
  tft.setRotation(0);
  
  bg_color = DARK_SIAN;
  tft.fillScreen(bg_color);

  // Draw the dial
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  TJpgDec.drawJpg(0, 0, myDial, sizeof(myDial));
  //tft.drawCircle(DIAL_CENTRE_X, DIAL_CENTRE_Y, NEEDLE_RADIUS-NEEDLE_LENGTH, TFT_DARKGREY);

  // Load the font and create the Sprite for reporting the value
  //spr.loadFont(AA_FONT_LARGE);
  while (!motor_has_loaded) {
    vTaskDelay(1);
  }
  tft.fillScreen(bg_color);
  spr.setTextSize(30);
  //bg_color = tft.readPixel(120, 120); // Get colour from dial centre
  spr_width = spr.textWidth("777"); // 7 is widest numeral in this font
  spr.createSprite(spr_width, spr.fontHeight());

  float num_of_pixels = (2.00 * _PI * 115.00) * (1.00/ pos_max_rotation); //pos_max_rotation
  ark.createSprite(num_of_pixels,115);
  clr.createSprite(num_of_pixels,115);
  Serial.println(num_of_pixels);
  //ark.fillSprite(TFT_GREEN);
  //ark.fillCircle(num_of_pixels/2,10,10,TFT_BLUE);
  
  spr.fillSprite(bg_color);
  //spr.setTextSize(100);
  spr.setTextColor(TFT_WHITE, bg_color, true);
  spr.setTextDatum(MC_DATUM);
  spr.setTextPadding(spr_width);
  spr.drawNumber(0, spr_width/2, spr.fontHeight()/2);
  spr.pushSprite(DIAL_CENTRE_X - spr_width / 2, DIAL_CENTRE_Y - spr.fontHeight() / 2);
  tft.setPivot(120, 120);
  ark.setPivot(num_of_pixels/2, 115);
  clr.setPivot(num_of_pixels/2, 115);
  // Create the needle Sprite
  clr.fillSprite(bg_color);
  ark.fillSprite(bg_color);
  vTaskDelay(100);
  int old_value = 0;

  for (float i= 270 - (num_of_pixels/4) ; i<270 +(num_of_pixels/4); i+= 0.05) {
          for (int t=115; t>105; t-= 0.1) {       
          int dx = (t * cos((i)* (_PI/180.00)) + num_of_pixels/2);
          int dy = (t * sin((i)* (_PI/180.00)) + 115);
          ark.drawPixel(dx, dy, TFT_GREEN);
          }
      }
  float draw_ark = (2 * _PI) * (0/ pos_max_rotation);
  ark.pushRotated(draw_ark * (180.00/_PI));
  
  for(;;) {
    struct knobState currentState;

    if (xQueueReceive(knobPositionQueue, &currentState, portMAX_DELAY) == pdPASS) {
      //Serial.println(currentState.rot);
     // vTaskDelay(100);
      //delay(100);
      //float value = map(currentState.rot * 100, 0, (24000.00 * (PI/180.00)), 0, 24000);
      
      //value = value / 100;
      //value = constrain(value, 0, 240);
      float value = currentState.rot;
      draw_ark = (2 * _PI) * (value/ pos_max_rotation);
      float incrament = (2 * _PI) * (1/ pos_max_rotation);
      
      //Serial.println(incrament * (180.00/_PI));
      //ark.fillSprite(bg_color);
      //for (float i=0.00; i<incrament; i+= 0.02) {
      //for (float i= 270 - (incrament * (180.00/_PI)); i<270 + (incrament * (180.00/_PI)); i+= 0.5) {
        


      if (old_value != value) {
        //ark.fillSprite(bg_color);
        
        float draw_ark_old = (2 * _PI) * (old_value/ pos_max_rotation);
        clr.pushRotated(draw_ark_old * (180.00/_PI));
        //ark.fillCircle(num_of_pixels / 2,120,120,LIGHT_BLUE_BG);
        //ark.fillCircle(num_of_pixels / 2,120, 110,bg_color);
        //ark.drawCircle(num_of_pixels / 2,120,120, TFT_BLUE);
        //ark.drawCircle(num_of_pixels / 2,119,120, TFT_BLUE);
        //ark.drawCircle(num_of_pixels / 2,118,120, TFT_BLUE);

        
      
        ark.pushRotated(draw_ark * (180.00/_PI));
        
      }
      old_value = value;

      
      //ark.fillCircle(40,40, 40, TFT_BLUE);
      //ark.pushSprite(120, 120);
      
      spr.setTextColor(TFT_WHITE, bg_color, true); //bg_color
      spr.drawNumber(value, spr_width/2, spr.fontHeight()/2);
      spr.pushSprite(120 - spr_width / 2, 120 - spr.fontHeight() / 2);
      //plotNeedle(value, 0, false);
      
      //ark.pushRotated(draw_ark * (180.00/_PI));
      //ark.pushSprite(0, 100);
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
      //Serial.println(uxTaskGetStackHighWaterMark(motorTask));
      //  vTaskDelay(100);
    }
  }
  vTaskDelete(displayTask);
}

/*
void createNeedle(void)
{
  needle.setColorDepth(16);
  needle.createSprite(NEEDLE_WIDTH, NEEDLE_LENGTH);  // create the needle Sprite

  needle.fillSprite(TFT_BLACK); // Fill with black

  // Define needle pivot point relative to top left corner of Sprite
  uint16_t piv_x = NEEDLE_WIDTH / 2; // pivot x in Sprite (middle)
  uint16_t piv_y = NEEDLE_RADIUS;    // pivot y in Sprite
  needle.setPivot(piv_x, piv_y);     // Set pivot point in this Sprite

  // Draw the red needle in the Sprite
  needle.fillRect(0, 0, NEEDLE_WIDTH, NEEDLE_LENGTH, TFT_MAROON);
  needle.fillRect(1, 1, NEEDLE_WIDTH-2, NEEDLE_LENGTH-2, TFT_RED);

  // Bounding box parameters to be populated
  int16_t min_x;
  int16_t min_y;
  int16_t max_x;
  int16_t max_y;

  // Work out the worst case area that must be grabbed from the TFT,
  // this is at a 45 degree rotation
  needle.getRotatedBounds(45, &min_x, &min_y, &max_x, &max_y);

  // Calculate the size and allocate the buffer for the grabbed TFT area
  tft_buffer =  (uint16_t*) malloc( ((max_x - min_x) + 2) * ((max_y - min_y) + 2) * 2 );
}

// =======================================================================================
// Move the needle to a new position
// =======================================================================================
void plotNeedle(int16_t angle, uint16_t ms_delay, bool timed)
{
  static int16_t old_angle = -120; // Starts at -120 degrees
  //TJpgDec.drawJpg(0, 0, dial, sizeof(dial));

  // Bounding box parameters
  static int16_t min_x;
  static int16_t min_y;
  static int16_t max_x;
  static int16_t max_y;

  if (angle < 0) angle = 0; // Limit angle to emulate needle end stops
  if (angle > 240) angle = 240;

  angle -= 120; // Starts at -120 degrees

  // Move the needle until new angle reached
  while (angle != old_angle || !buffer_loaded) {

    if (old_angle < angle) old_angle++;
    else old_angle--;
    if (!timed) {
      old_angle = angle;
    }
    

    // Only plot needle at even values to improve plotting performance
    if ( (old_angle & 1) == 0)
    {
      if (buffer_loaded) {
        // Paste back the original needle free image area
        tft.pushRect(min_x, min_y, 1 + max_x - min_x, 1 + max_y - min_y, tft_buffer);
      }

      if ( needle.getRotatedBounds(old_angle, &min_x, &min_y, &max_x, &max_y) )
      {
        // Grab a copy of the area before needle is drawn
        tft.readRect(min_x, min_y, 1 + max_x - min_x, 1 + max_y - min_y, tft_buffer);
        buffer_loaded = true;
      }

      // Draw the needle in the new position, black in needle image is transparent
      needle.pushRotated(old_angle, TFT_BLACK); //old_angle

      // Wait before next update
      if (timed){
        delay(ms_delay);
      }
      
    }

    // Update the number at the centre of the dial
    spr.setTextColor(TFT_WHITE, bg_color, true);
    spr.drawNumber(old_angle+120, spr_width/2, spr.fontHeight()/2);
    spr.pushSprite(120 - spr_width / 2, 120 - spr.fontHeight() / 2);

    // Slow needle down slightly as it approaches the new position
    if (abs(old_angle - angle) < 10) ms_delay += ms_delay / 5;
  }
}
*/

// =======================================================================================
