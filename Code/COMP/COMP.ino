//initializations
#include "Configuration.h"
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h> 
#include <Wire.h>
#define SAFE

//-------PID Config----------
#define ROLL_PID_KP  0.250
#define ROLL_PID_KI  0.950
#define ROLL_PID_KD  0.011
#define ROLL_PID_MIN  -200.0
#define ROLL_PID_MAX  200.0

#define PITCH_PID_KP  0.250
#define PITCH_PID_KI  0.950
#define PITCH_PID_KD  0.011
#define PITCH_PID_MIN  -200.0
#define PITCH_PID_MAX  200.0

#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001
#define YAW_PID_MIN  100.0
#define YAW_PID_MAX  100.0

//-------------------------



//-------RX Config----------
#define THROTTLE_RMIN  1000
#define THROTTLE_SAFE_SHUTOFF 1120
#define THROTTLE_RMAX  1900
#define THROTTLE_RMID  1470

#define ROLL_RMIN  THROTTLE_RMIN
#define ROLL_RMAX  THROTTLE_RMAX
#define ROLL_WMIN  -30
#define ROLL_WMAX  30

#define PITCH_RMIN  THROTTLE_RMIN
#define PITCH_RMAX  THROTTLE_RMAX
#define PITCH_WMIN  -30
#define PITCH_WMAX  30

#define YAW_RMIN  THROTTLE_RMIN
#define YAW_RMAX  THROTTLE_RMAX
#define YAW_WMIN  -30
#define YAW_WMAX  30

//-------IMU Config-----------
#define ADDR_SLAVE_I2C 2
#define PACKET_SIZE 12

//-------Debug Config---------
#define DEBUG_OUTPUT
#define DEBUG_ANGLES
#define DEBUG_PID
#define DEBUG_RX
#define DEBUG_MOTORS
#define DEBUG_LOOP_TIME
//----------------------------

//-------Motor PWM Levels
#define MOTOR_ZERO_LEVEL  1000
#define MOTOR_ARM_START  1500
#define MOTOR_MAX_LEVEL  2000


//-------RX PINS-------------
#define RX_PINS_OFFSET 2
#define PIN_RX_ROLL 2
#define PIN_RX_PITCH 3
#define PIN_RX_THROTTLE 4
#define PIN_RX_YAW 5

//-------MOTOR PINS-----------
#define PIN_MOTOR0  6
#define PIN_MOTOR1  9
#define PIN_MOTOR2  10
#define PIN_MOTOR3  11

//-------LED PINS-------------
#define PIN_LED 13

// Angles
float angleX,angleY,angleZ = 0.0;

// RX Signals
int throttle=THROTTLE_RMIN;
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW

// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;
  
// Motors
int m0, m1, m2, m3; // Front, Right, Back, Left

// Track loop time.
unsigned long prev_time = 0;


void setup() 
{  
  #ifdef DEBUG_OUTPUT
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Debug Output ON");
  #endif
  
  motors_initialize();
  leds_initialize();
  rx_initialize();
  pid_initialize();
  motors_arm();
  
  //wait for IMU YAW  to settle before beginning??? ~20s
}

void loop() 
{
  imu_update();
  control_update();

  #ifdef DEBUG_OUTPUT
    debug_process();
  #endif
  prev_time = micros();
}
void control_update(){
  throttle=map(rx_values[2],THROTTLE_RMIN,THROTTLE_RMAX,MOTOR_ZERO_LEVEL,MOTOR_MAX_LEVEL);
  
  setpoint_update();
  pid_update();
  pid_compute();
  
  // yaw control disabled for stabilization testing...
  m0 = throttle + pid_pitch_out ;//+ pid_yaw_out;
  m1 = throttle + pid_roll_out ;//- pid_yaw_out;
  m2 = throttle - pid_pitch_out ;//+ pid_yaw_out;
  m3 = throttle - pid_roll_out ;//- pid_yaw_out;
  
  #ifdef SAFE
    if(throttle < THROTTLE_SAFE_SHUTOFF)
    {
      m0 = m1 = m2 = m3 = MOTOR_ZERO_LEVEL;
    }
  #endif
  
  update_motors(m0, m1, m2, m3);
}

void setpoint_update() {
  // here we allow +- 20 for noise and sensitivity on the RX controls...
  // ROLL rx at mid level?
  if(rx_values[0] > THROTTLE_RMID - 20 && rx_values[0] < THROTTLE_RMID + 20)
    pid_roll_setpoint = 0;
  else
    pid_roll_setpoint = map(rx_values[0],ROLL_RMIN,ROLL_RMAX,ROLL_WMIN,ROLL_WMAX);
  //PITCH rx at mid level?
  if(rx_values[1] > THROTTLE_RMID - 20 && rx_values[1] < THROTTLE_RMID + 20)
    pid_pitch_setpoint = 0;
  else
    pid_pitch_setpoint = map(rx_values[1],PITCH_RMIN,PITCH_RMAX,PITCH_WMIN,PITCH_WMAX);
  //YAW rx at mid level?
  if(rx_values[3] > THROTTLE_RMID - 20 && rx_values[3] < THROTTLE_RMID + 20)
    pid_yaw_setpoint = 0;
  else
    pid_yaw_setpoint = map(rx_values[3],YAW_RMIN,YAW_RMAX,YAW_WMIN,YAW_WMAX);
}
void imu_update() 
{
  Wire.requestFrom(ADDR_SLAVE_I2C, PACKET_SIZE);
  byte data[PACKET_SIZE];
  int i = 0;
  while(Wire.available())
  { 
    data[i] = Wire.read(); 
    i++;
  }
  
  // we use a c union to convert between byte[] and float
  union ROL_tag {byte b[4]; float fval;} ROL_Union; 
  union PIT_tag {byte b[4]; float fval;} PIT_Union; 
  union YAW_tag {byte b[4]; float fval;} YAW_Union;
  
  ROL_Union.b[0] = data[0];
  ROL_Union.b[1] = data[1];
  ROL_Union.b[2] = data[2];
  ROL_Union.b[3] = data[3];
  if(isnan(ROL_Union.fval) != 1)
  {
    angleX = ROL_Union.fval;
  }
  
  PIT_Union.b[0] = data[4];
  PIT_Union.b[1] = data[5];
  PIT_Union.b[2] = data[6];
  PIT_Union.b[3] = data[7];
  if(isnan(PIT_Union.fval) != 1)
  {
    angleY = PIT_Union.fval;
  }
  
  YAW_Union.b[0] = data[8];
  YAW_Union.b[1] = data[9];
  YAW_Union.b[2] = data[10];
  YAW_Union.b[3] = data[11];
  if(isnan(YAW_Union.fval) != 1)
  {
    angleZ = YAW_Union.fval;
  }
}
Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

void motors_initialize(){
  motor0.attach(PIN_MOTOR0);
  motor1.attach(PIN_MOTOR1);
  motor2.attach(PIN_MOTOR2);
  motor3.attach(PIN_MOTOR3);
  motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
}

void motors_arm(){
  motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
}

void update_motors(int m0, int m1, int m2, int m3)
{
  motor0.writeMicroseconds(m0);
  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
}
PID roll_controller(&pid_roll_in,   &pid_roll_out,  &pid_roll_setpoint,  5.0, 0.0, 0.0, REVERSE);
PID pitch_controller(&pid_pitch_in, &pid_pitch_out, &pid_pitch_setpoint, 5.0, 0.0, 0.0, REVERSE);
PID yaw_controller(&pid_yaw_in,     &pid_yaw_out,   &pid_yaw_setpoint,   1.0, 0.0, 0.0, DIRECT); 


void pid_initialize() {
  roll_controller.SetOutputLimits(ROLL_PID_MIN,ROLL_PID_MAX);
  pitch_controller.SetOutputLimits(PITCH_PID_MIN,PITCH_PID_MAX);
  yaw_controller.SetOutputLimits(YAW_PID_MIN,YAW_PID_MAX);
  roll_controller.SetMode(AUTOMATIC);
  pitch_controller.SetMode(AUTOMATIC);
  yaw_controller.SetMode(AUTOMATIC);
  roll_controller.SetSampleTime(10);
  pitch_controller.SetSampleTime(10);
  yaw_controller.SetSampleTime(10);
}

void pid_update(){
  pid_roll_in = angleX;
  pid_pitch_in = angleY;
  pid_yaw_in = angleZ; 
}

void pid_compute() {
   roll_controller.Compute();
   pitch_controller.Compute();
   yaw_controller.Compute();
}

void leds_initialize(){
 pinMode(PIN_LED, OUTPUT);
 digitalWrite(PIN_LED, LOW); 
}

void leds_status(byte stat){
 digitalWrite(PIN_LED, stat);  
}
void debug_process(){
#ifdef DEBUG_OUTPUT  

#ifdef DEBUG_ANGLES
  Serial.print(F("X:"));
  Serial.print((float)(angleX));
  Serial.print('\t');
  Serial.print(F("Y:"));
  Serial.print((float)(angleY));
  Serial.print('\t');
  //Serial.print(F("Z:"));
  //Serial.print((float)(angleZ));
  //Serial.print('\t');
#endif

#ifdef DEBUG_RX
  Serial.print('\t');
  Serial.print(F("RX_Roll:"));
  Serial.print(rx_values[0]);
  Serial.print('\t');
  Serial.print(F("RX_Pitch:"));
  Serial.print(rx_values[1]);
  Serial.print('\t');
  Serial.print(F("RX_Throttle:"));
  Serial.print(rx_values[2]);
  Serial.print('\t');
  //Serial.print(F("RX_Yaw:"));
  //Serial.print(rx_values[3]);
  //Serial.print('\t');   
#endif

#ifdef DEBUG_PID
  Serial.print(F("PID_R:"));
  Serial.print(pid_roll_in);Serial.print(',');
  Serial.print(pid_roll_setpoint);Serial.print(',');
  Serial.print(pid_roll_out);
  Serial.print('\t');
  Serial.print(F("PID_P:"));
  Serial.print(pid_pitch_in);Serial.print(',');
  Serial.print(pid_pitch_setpoint);Serial.print(',');
  Serial.print(pid_pitch_out);
  Serial.print('\t');
  //Serial.print(F("PID_Y:"));
  //Serial.print(pid_yaw_in);Serial.print(',');
  //Serial.print(pid_yaw_setpoint);Serial.print(',');
  //Serial.print(pid_yaw_out);
  //Serial.print('\t');
#endif

#ifdef DEBUG_MOTORS
  Serial.print('\t');
  Serial.print(F("M0:"));
  Serial.print(m0);
  Serial.print('\t');
  Serial.print(F("M1:"));
  Serial.print(m1);
  Serial.print('\t');
  Serial.print(F("M2:"));
  Serial.print(m2);
  Serial.print('\t');
  Serial.print(F("M3:"));
  Serial.print(m3);
  Serial.print('\t'); 
#endif

#ifdef DEBUG_LOOP_TIME
  Serial.print('\t');
  unsigned long elapsed_time = micros() - prev_time;
  Serial.print(F("Time:"));
  Serial.print((float)elapsed_time/1000);
  Serial.print(F("ms"));
  Serial.print('\t');
#endif

  Serial.println();
#endif
}

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeIn = 0xE8E8F0F0E1LL;     //Remember that this code is the same as in the transmitter
RF24 radio(9, 10);  //CSN and CE pins

// The sizeof this struct should not exceed 32 bytes
struct Received_data {
  byte ch1;
};

int ch1_value = 0;
Received_data received_data;


/**************************************************/

void setup()
{
  Serial.begin(9600);
  //We reset the received values
  received_data.ch1 = 127;
 
  //Once again, begin and radio configuration
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,pipeIn);
  
  //We start the radio comunication
  radio.startListening();

}

/**************************************************/

unsigned long last_Time = 0;

//We create the function that will read the data each certain time
void receive_the_data()
{
  while ( radio.available() ) {
  radio.read(&received_data, sizeof(Received_data));
  last_Time = millis(); //Here we receive the data
}
}

/**************************************************/

void loop()
{
  //Receive the radio data
  receive_the_data();

  
  ch1_value = map(received_data.ch1,0,255,1000,2000);
  Serial.println(ch1_value);
  
  
}//Loop end
