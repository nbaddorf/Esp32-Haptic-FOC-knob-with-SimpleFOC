//#include <analogWrite.h>
#include "SPI.h"
//#include "Adafruit_GFX.h"
//#include "Adafruit_GC9A01A.h"
#include <littlefs_api.h>
#include <esp_littlefs.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>  // Widget library

#include "NotoSansBold36.h"
#define AA_FONT_LARGE NotoSansBold36

// Jpeg image array attached to this sketch
#include "dial.h"
#include "myDial.h"

// Include the jpeg decoder library
#include <TJpg_Decoder.h>

uint16_t* tft_buffer;
bool      buffer_loaded = false;
uint16_t  spr_width = 0;
uint16_t  bg_color =0;

#define NEEDLE_LENGTH 35  // Visible length
#define NEEDLE_WIDTH   6  // Width of needle - make it an odd number
#define NEEDLE_RADIUS 90  // Radius at tip
#define NEEDLE_COLOR1 TFT_MAROON  // Needle periphery colour
#define NEEDLE_COLOR2 TFT_RED     // Needle centre colour
#define DIAL_CENTRE_X 120
#define DIAL_CENTRE_Y 120


//#include <lfs_util.h>
//#include <lfs.h>
//#include <LITTLEFS.h>
//#include <vfs_api.h>
//#include <FS.h>
//#include <FSImpl.h>

#include <SimpleFOC.h>
#include <PID_v1.h>

#define configCHECK_FOR_STACK_OVERFLOW 1
#define configASSERT

struct knobState {
  float rot;
  int num_of_detent;
};

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite needle = TFT_eSprite(&tft); // Sprite object for needle
TFT_eSprite spr    = TFT_eSprite(&tft); // Sprite for meter reading

//MeterWidget   numDisplay  = MeterWidget(&tft);

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
//Adafruit_GC9A01A tft(TFT_CS, TFT_DC, MOSI, SCLK, RST, -1);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 12, 14, 27);

// encoder instance
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

int detent = 6;

int num = 0;
float start_rot = 0;
float start_angle = 0.0f;
//int pos_setting = 50;
float start_pos_rot = 0.0f;
float pos_rot = 0.0f; //1.58
float old_rot = 0;
float pos_max_rotation = 4.18879;
//float pos_max_rotation = (240.00 * (PI/180.00)); //3.14

bool last_haptic_button_state = false;

double P_Input, P_Output;
double P_Setpoint = 0;

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

  start_rot = 0; // motor.shaft_angle;
  start_pos_rot = pos_rot; //
  float temp = 0;

  for (;;) {
     // main FOC algorithm function
    motor.loopFOC();
    float shaftAngle = motor.shaft_angle;
    //Serial.println(shaftAngle);

    //setDetent(0.7, 15);
    //motor.move(motor.PID_velocity(attract_angle - shaftAngle));
    float input = start_pos_rot - (start_rot - shaftAngle);
   
    
    pos_rot = constrain(input, 0, pos_max_rotation);
    
    float pid_input = input - pos_rot;
    if (abs(pid_input) <= 0.05) {
      pid_input = 0;
    }

    if (pid_input == 0) {
      setDetent(0.9, detent);
      motor.move(motor.PID_velocity(attract_angle - shaftAngle));
    } else {
      motor.move(P_rubber(pid_input) * -1);
    }

    attract_angle = findAttractor(shaftAngle);

    struct knobState currentState;
    currentState.rot = shaftAngle;
    currentState.num_of_detent = 15;

    xQueueOverwrite(structQueue, &currentState);
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

  // The byte order can be swapped (set true for TFT_eSPI)
  TJpgDec.setSwapBytes(true);

  // The jpeg decoder must be given the exact name of the rendering function above
  TJpgDec.setCallback(tft_output);
  
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // Draw the dial
  TJpgDec.drawJpg(0, 0, dial, sizeof(dial));
  //tft.drawCircle(DIAL_CENTRE_X, DIAL_CENTRE_Y, NEEDLE_RADIUS-NEEDLE_LENGTH, TFT_DARKGREY);

  // Load the font and create the Sprite for reporting the value
  spr.loadFont(AA_FONT_LARGE);
  spr_width = spr.textWidth("777"); // 7 is widest numeral in this font
  spr.createSprite(spr_width, spr.fontHeight());
  bg_color = tft.readPixel(120, 120); // Get colour from dial centre
  spr.fillSprite(bg_color);
  spr.setTextColor(TFT_WHITE, bg_color, true);
  spr.setTextDatum(MC_DATUM);
  spr.setTextPadding(spr_width);
  spr.drawNumber(0, spr_width/2, spr.fontHeight()/2);
  spr.pushSprite(DIAL_CENTRE_X - spr_width / 2, DIAL_CENTRE_Y - spr.fontHeight() / 2);

  // Plot the label text
  tft.setTextColor(TFT_WHITE, bg_color);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("(degrees)", DIAL_CENTRE_X, DIAL_CENTRE_Y + 48, 2);

  // Define where the needle pivot point is on the TFT before
  // creating the needle so boundary calculation is correct
  tft.setPivot(DIAL_CENTRE_X, DIAL_CENTRE_Y);

  // Create the needle Sprite
  createNeedle();

  // Reset needle position to 0
  plotNeedle(0, 0, true);
  plotNeedle(240,1, true);
  plotNeedle(0, 1, true);
  vTaskDelay(500);

  for(;;) {
    struct knobState currentState;

    if (xQueueReceive(structQueue, &currentState, portMAX_DELAY) == pdPASS) {
      //Serial.println(currentState.rot);
     // vTaskDelay(100);
      //delay(100);
      float value = map(currentState.rot * 100, 0, (24000.00 * (PI/180.00)), 0, 24000);
      value = value / 100;
      plotNeedle(value, 0, false);
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

// =======================================================================================
