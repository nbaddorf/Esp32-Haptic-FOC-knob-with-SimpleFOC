#include <SimpleFOC.h>
#include <PID_v1.h>
//#include <BleKeyboard.h>


//BleKeyboard bleKeyboard;

int old_range = 0;

//My motor pins (13, 12, 14, 27)
//pp = 7

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Kp=10, Ki=0, Kd=0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double DetentSetpoint, DetentInput, DetentOutput;
double DKp=0.06, DKi=0, DKd=0.1;
PID detentPID(&DetentInput, &DetentOutput, &DetentSetpoint, DKp, DKi, DKd, DIRECT);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 12, 14, 27);

// voltage set point variable
float target_voltage = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // aligning voltage 
  motor.voltage_sensor_align = 5;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
   //motor.controller = MotionControlType::angle;
  // motor.controller = MotionControlType::velocity;

   // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 10; //20
  motor.PID_velocity.D = 0;
  
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;

  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.02f; //0.01f

  // angle P controller
  motor.P_angle.P = 10; //20
  // maximal velocity of the position control
  motor.velocity_limit = 40;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  Setpoint = 0;
  Input = 0;
  Output = 0;

  DetentSetpoint = 0;
  DetentInput = 0;
  DetentOutput = 0;

  detentPID.SetMode(AUTOMATIC);
  detentPID.SetOutputLimits(-2,2);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-2,2);

  //bleKeyboard.begin();
  _delay(1000);
}

void loop() {

  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code

  Input = sensor.getAngle();

  DetentInput = int(sensor.getAngle() * 20) % 50;
  if (DetentInput <= 25) {
    DetentInput = DetentInput - 25;
  }
  if (DetentInput > 25) {
    DetentInput = DetentInput - 25;
  }
  //detentPID.Compute();
  

  if (sensor.getAngle() < 0) {
    myPID.SetMode(AUTOMATIC);
    //Output = 0;
    Setpoint = 0;
    myPID.Compute();
    motor.move(Output);
  } else if (sensor.getAngle() > 2.0) {
    myPID.SetMode(AUTOMATIC);
    //Output = 0;
    Setpoint = 2;
    myPID.Compute();
    motor.move(Output);
  } else {
    myPID.SetMode(MANUAL);
    Output = 0;
    //Setpoint = Input;
    //myPID.Compute();
    motor.move(0);
    //motor.move(DetentOutput);
  }

  
/*
    DetentInput = sensor.getVelocity();
    //detentPID.SetMode(AUTOMATIC);
    //Output = 0;
    DetentSetpoint = 10;
    detentPID.Compute();
    motor.move(DetentOutput);
*/
    /*
  if (sensor.getAngle() < 1.5) {
    
    myPID.SetMode(AUTOMATIC);
    //Output = 0;
    Setpoint = 0;
    myPID.Compute();
    motor.move(DetentOutput);
  } else if (sensor.getAngle() >= 1.5) {
    
    myPID.SetMode(AUTOMATIC);
    //Output = 0;
    Setpoint = 3;
    myPID.Compute();
    motor.move(DetentOutput);
  }
*/
   
  int range = map(Input * 100, 0.00, 200, 0.0, 20.0);
  range = constrain(range, 0, 16);
  //if(bleKeyboard.isConnected()) {
    if ( range != old_range) {
      if (range > old_range) {
        //bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
      } else {
        //bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
      }
    }
  //}

  old_range = range;

  //motor.move(DetentOutput);
  Serial.print(DetentInput);
  Serial.print("\t");
  Serial.print(DetentOutput);
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
  
  // user communication
  //command.run();
  
 // driver.setPwm(0,10,0);
}
