#include <SimpleFOC.h>
#include <PID_v1.h>

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
int pos_max_rotation = 3.14 * 1; //3.14

bool last_haptic_button_state = false;

double P_Input, P_Output;
double P_Setpoint = 0;

//PID P_haptic(&P_Input, &P_Output, &P_Setpoint, 4.00, 0.00, 0.04, DIRECT);
//PIDController P_haptic{.P=4,.I=0,.D=0.04,.output_ramp=10000,.limit=10};

void setup() { 
  
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
  Serial.begin(115200);
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
}

// haptic attraction controller - only Proportional 
//PIDController P_haptic{.P=4,.I=0,.D=0.04,.output_ramp=10000,.limit=10};
//PIDController P_haptic{.P=10,.I=0,.D=0,.output_ramp=100000,.limit=12};

PIDController P_rubber{.P=10,.I=0,.D=0,.output_ramp=100000,.limit=4};

// attractor angle variable
float attract_angle = 0;
// distance between attraction points
int deg = 20;
float attractor_distance = deg*_PI/180.0; // dimp each 45 degrees

float findAttractor(float current_angle){
  return round(current_angle/attractor_distance)*attractor_distance;
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

   if (Serial.available() > 0) {
 //numtest = Serial1.parseInt();
   num = Serial.parseInt();
 
   if (Serial.read() == '\n') {
    start_rot = motor.shaft_angle;
    start_pos_rot = pos_rot;
    start_angle = attract_angle - start_rot;
    if (num != 5) {
      //motor.controller = MotionControlType::torque;
    } else if (num == 5) {
      //motor.controller = MotionControlType::velocity;
    }
   /*
   if (num == 1) {
    motor.
   } else if (num == 2) {
    analogWrite(m1, 0);
   } 
  */
  }
 }

  // Motion control function
  if (num == 0) {
    //medium detent
    setDetent(1, 45);
    //deg = 45;
    //attractor_distance = deg*_PI/180.0;
    motor.move(motor.PID_velocity(attract_angle - motor.shaft_angle));
   // motor.move(P_haptic(attract_angle - motor.shaft_angle));
  } else if (num == 1) {
    //no resistance scroll
    motor.move(0);
  } else if (num == 2) {
    setDetent(0.5, 15);
    //small detent
    //deg = 20;
    //attractor_distance = deg*_PI/180.0;
    //int dAngle = attract_angle - start_rot;
    //Serial.println(attract_angle - motor.shaft_angle - start_angle);
    motor.move(motor.PID_velocity(attract_angle - motor.shaft_angle));
    //motor.move(P_haptic(attract_angle - (motor.shaft_angle)));
  } else if (num == 3) {
    // large detent
    setDetent(2, 100);
    //deg = 60;
    //attractor_distance = deg*_PI/180.0;
    motor.move(motor.PID_velocity(attract_angle - motor.shaft_angle));
    //motor.move(P_haptic(attract_angle - motor.shaft_angle));
  } else if (num == 4) {
    // rubberband scrolling
    float input = start_pos_rot - (start_rot - motor.shaft_angle);
    //input = start_pos_rot - input;
    pos_rot = constrain(input, 0, pos_max_rotation);

    //Serial.println(pos_rot);

    float pid_input = input - pos_rot;
    if (abs(pid_input) <= 0.02) {
      pid_input = 0;
    }
    //float out = P_rubber(pid_input) * -1;
    motor.move(P_rubber(pid_input) * -1);
    
  } else if (num == 5) {
    /*
    //"infinite" scrolling test
    float vel = encoder.getVelocity();

    //if ((abs(vel) <= 0.3) || (abs(vel) > 60)) {
    if ((abs(vel) <= 0.3) || (abs(vel) > 60)) {
      vel = 0;
    }
    float test = (abs(vel) - 0.01) * (vel / abs(vel));
    Serial.println(test);
    test = abs(test) > 0 ? (test) : 0;
    //Serial.println(test);
    motor.move(1);
    */
    setDetent(0.25, 15);
    motor.move(motor.PID_velocity(attract_angle - motor.shaft_angle));
    
  } else if (num == 6) {
    //simulate button press
    hapticButton(true);
    delay(200);
    hapticButton(false);
    num = 1;
  } else if (num == 7) {
    float input = start_pos_rot - (start_rot - motor.shaft_angle);
    pos_rot = constrain(input, 0, pos_max_rotation);
    
    float pid_input = input - pos_rot;
    if (abs(pid_input) <= 0.05) {
      pid_input = 0;
    }

    if (pid_input == 0) {
      setDetent(0.5, 15);
      motor.move(motor.PID_velocity(attract_angle - motor.shaft_angle));
    } else {
      motor.move(P_rubber(pid_input) * -1);
    }
    
  }
  

  // calculate the attractor
  //attract_angle = findAttractor(start_rot - motor.shaft_angle);
  attract_angle = findAttractor(motor.shaft_angle);
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
