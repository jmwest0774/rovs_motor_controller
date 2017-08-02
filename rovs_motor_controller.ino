#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <EnableInterrupt.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// macros to directly read the pins for encoder direction
// these are specific to the Mega
#define READ_PIN_22 bitRead(PINA, 0)
#define READ_PIN_23 bitRead(PINA, 1)
#define READ_PIN_24 bitRead(PINA, 2)
#define READ_PIN_25 bitRead(PINA, 3)
#define READ_PIN_26 bitRead(PINA, 4)
#define READ_PIN_27 bitRead(PINA, 5)
#define READ_PIN_28 bitRead(PINA, 6)
#define READ_PIN_29 bitRead(PINA, 7)

#define READ_PIN_30 bitRead(PINC, 7)
#define READ_PIN_31 bitRead(PINC, 6)
#define READ_PIN_32 bitRead(PINC, 5)
#define READ_PIN_33 bitRead(PINC, 4)
#define READ_PIN_34 bitRead(PINC, 3)
#define READ_PIN_35 bitRead(PINC, 2)
#define READ_PIN_36 bitRead(PINC, 1)
#define READ_PIN_37 bitRead(PINC, 0)

#define READ_PIN_38 bitRead(PIND, 7)
#define READ_PIN_39 bitRead(PING, 2)
#define READ_PIN_40 bitRead(PING, 1)
#define READ_PIN_41 bitRead(PING, 0)
#define READ_PIN_42 bitRead(PINL, 7)
#define READ_PIN_43 bitRead(PINL, 6)
#define READ_PIN_44 bitRead(PINL, 5)
#define READ_PIN_45 bitRead(PINL, 4)
#define READ_PIN_46 bitRead(PINL, 3)
#define READ_PIN_47 bitRead(PINL, 2)
#define READ_PIN_48 bitRead(PINL, 1)
#define READ_PIN_49 bitRead(PINL, 0)

// assign pins to all of the encoders
// pin_a is the interrupt pin.  Verify ISR pin mapping if you change these
const byte lfs_pin_a = 19;
const byte lfs_pin_b = 26;
#define READ_PIN_LFS READ_PIN_26
const byte lrs_pin_a = 18;
const byte lrs_pin_b = 28;
#define READ_PIN_LRS READ_PIN_28
const byte rfs_pin_a = 15;
const byte rfs_pin_b = 30;
#define READ_PIN_RFS READ_PIN_30
const byte rrs_pin_a = 14;
const byte rrs_pin_b = 32;
#define READ_PIN_RRS READ_PIN_32

const byte lfd_pin_a = 2;
const byte lfd_pin_b = 34;
#define READ_PIN_LFD READ_PIN_34
const byte lrd_pin_a = 3;
const byte lrd_pin_b = 36;
#define READ_PIN_LRD READ_PIN_36
const byte rfd_pin_a = 10;
const byte rfd_pin_b = 38;
#define READ_PIN_RFD READ_PIN_38
const byte rrd_pin_a = 11;
const byte rrd_pin_b = 40;
#define READ_PIN_RRD READ_PIN_40

const byte head_pin_a = 12;
const byte head_pin_b = 42;
#define READ_PIN_HEAD READ_PIN_42
const byte spare_pin_a = 13;
const byte spare_pin_b = 44;
#define READ_PIN_SPARE READ_PIN_44

const byte rcd_pin_a = A8;
const byte rcd_pin_b = 46;
#define READ_PIN_RCD READ_PIN_46
const byte lcd_pin_a = A9;
const byte lcd_pin_b = 48;
#define READ_PIN_LCD READ_PIN_48

Adafruit_MotorShield AFMS_1 = Adafruit_MotorShield(0x60);
Adafruit_DCMotor *lfs_motor = AFMS_1.getMotor(1);
Adafruit_DCMotor *lfd_motor = AFMS_1.getMotor(2);
Adafruit_DCMotor *lrs_motor = AFMS_1.getMotor(3);
Adafruit_DCMotor *lrd_motor = AFMS_1.getMotor(4);

Adafruit_MotorShield AFMS_2 = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *rfs_motor = AFMS_2.getMotor(1);
Adafruit_DCMotor *rfd_motor = AFMS_2.getMotor(2);
Adafruit_DCMotor *rrs_motor = AFMS_2.getMotor(3);
Adafruit_DCMotor *rrd_motor = AFMS_2.getMotor(4);

Adafruit_MotorShield AFMS_3 = Adafruit_MotorShield(0x62);
Adafruit_DCMotor *lcd_motor = AFMS_3.getMotor(1);
Adafruit_DCMotor *rcd_motor = AFMS_3.getMotor(2);
Adafruit_DCMotor *head_motor = AFMS_3.getMotor(3);
//Adafruit_DCMotor *spare_motor = AFMS_3.getMotor(4);

class Gains {
  public:
    long int Kp;
    long int Ki;
    long int Kd;
    long int max_step;
    Gains(void) {};
    Gains(long int _Kp, long int _Ki, long int _Kd, long int _max_step) {Kp=_Kp;Ki=_Ki;Kd=_Kd;max_step=_max_step;};
};

class PIDController {
  public:
  long int cmd;
  long int setpoint;
  long int error;
  int pin_a;
  int pin_b;
  long int prev_error;
  long int integration;
  int pulses_per_rev;
  bool speed_control;
  long int motor_drive;
  bool open_loop;
  long int integration_limit;

  // control loop gains
  Gains gains;
  Adafruit_DCMotor *motor;
  PIDController(int _pin_a, int _pin_b, Gains _gains, Adafruit_DCMotor *_motor);
  void softStart();
  volatile long int count;
  long int position;
  long int speed;
  void loop(void);
};
PIDController::PIDController(int _pin_a, int _pin_b, Gains _gains, Adafruit_DCMotor *_motor) {
  pin_a = _pin_a;
  pin_b = _pin_b;
  gains = _gains;
  motor = _motor;
  pinMode(pin_a, INPUT_PULLUP);
  digitalWrite(pin_a, HIGH);
  pinMode(pin_b, INPUT_PULLUP);
  digitalWrite(pin_b, HIGH);
  pulses_per_rev = 18141;
  speed_control = true;
  integration_limit = 255;
  integration = 0;
  open_loop = false;
}
void PIDController::softStart() {
  long int d_setpoint = cmd-setpoint;
  if(d_setpoint > 0) {
     if(d_setpoint > gains.max_step) setpoint += gains.max_step;
     else setpoint = cmd;
  } else if(d_setpoint < 0) {
     if(d_setpoint < gains.max_step) setpoint -= gains.max_step;
     else setpoint = cmd;
  }
}
#define MAX_SPEED 200
void PIDController::loop() {
  // encoder misreads give large strange values so ignore them
  if(!(count>MAX_SPEED || count<-MAX_SPEED)) speed = count;
  count = 0;
  position += speed;
  if(open_loop) {
/*
      motor->setSpeed(255);
      motor->run(FORWARD);
*/
    motor->setSpeed(0);
    return;
  }
  // control loop
  softStart();
  if(speed_control) {
    error = setpoint- speed;
  } else {
    error = setpoint - position;
  }
  long int d_error = error-prev_error;
  prev_error = error;
  integration += error;
  if(integration > integration_limit) integration = integration_limit;
  if(integration < -integration_limit) integration = -integration_limit;
  motor_drive = (error * gains.Kp)/10 + (integration*gains.Ki)/10 + (d_error*gains.Kd)/10;
  if(motor_drive > 255) motor_drive = 255;
  if(motor_drive < -255) motor_drive = -255;
    if(motor_drive>=0) {
      motor->run(FORWARD);
      motor->setSpeed(motor_drive);
    } else {
      motor->run(BACKWARD);
      motor->setSpeed(-motor_drive);
    }
}
Gains positionGains(5,1,0,100);
Gains speedGains(10,8,1,10);

// create a controller for each motor
PIDController lfs(lfs_pin_a, lfs_pin_b, positionGains, lfs_motor);
PIDController lrs(lrs_pin_a, lrs_pin_b, positionGains, lrs_motor);
PIDController rfs(rfs_pin_a, rfs_pin_b, positionGains, rfs_motor);
PIDController rrs(rrs_pin_a, rrs_pin_b, positionGains, rrs_motor);

PIDController head(head_pin_a, head_pin_b, positionGains, head_motor);

PIDController lfd(lfd_pin_a, lfd_pin_b, speedGains, lfd_motor);
PIDController lrd(lrd_pin_a, lrd_pin_b, speedGains, lrd_motor);
PIDController rfd(rfd_pin_a, rfd_pin_b, speedGains, rfd_motor);
PIDController rrd(rrd_pin_a, rrd_pin_b, speedGains, rrd_motor);

PIDController rcd(rcd_pin_a, rcd_pin_b, speedGains, rcd_motor);
PIDController lcd(lcd_pin_a, lcd_pin_b, speedGains, lcd_motor);


// the ISRs have to be hard coded since they cannot be class functions
// use direct port access since digitalRead is too slow for fast encoders
void isr_LFS() {
  if(READ_PIN_LFS) lfs.count--;
  else lfs.count++;
}
void isr_LRS() {
  if(READ_PIN_LRS) lrs.count--;
  else lrs.count++;
}
void isr_RFS() {
  if(READ_PIN_RFS) rfs.count--;
  else rfs.count++;
}
void isr_RRS() {
  if(READ_PIN_RRS) rrs.count--;
  else rrs.count++;
}

void isr_LFD() {
  if(READ_PIN_LFD) lfd.count--;
  else lfd.count++;
}
void isr_LRD() {
  if(READ_PIN_LRD) lrd.count--;
  else lrd.count++;
}
void isr_RFD() {
  if(READ_PIN_RFD) rfd.count--;
  else rfd.count++;
}
void isr_RRD() {
  if(READ_PIN_RRD) rrd.count--;
  else rrd.count++;
}
void isr_RCD() {
  if(READ_PIN_RFD) rcd.count--;
  else rcd.count++;
}
void isr_LCD() {
  if(READ_PIN_LCD) lcd.count--;
  else lcd.count++;
}

void isr_HEAD() {
  if(READ_PIN_HEAD) head.count--;
  else head.count++;
}
#define LOOP_FREQ 10.0
#define STEER_CNT_PER_REV 4535.0
#define DRIVE_CNT_PER_REV 2062.0
/* user provides speed as rad per sec, this converts it to integer */
#define SPEED_TO_COUNTS (DRIVE_CNT_PER_REV/2.0/3.141/LOOP_FREQ)
#define COUNTS_TO_SPEED (1.0/SPEED_TO_COUNTS)
/* user provides angle in radians, this converts it to count */
#define RAD_TO_COUNTS (STEER_CNT_PER_REV/2.0/3.141)
#define COUNTS_TO_RAD (1.0/RAD_TO_COUNTS)

ros::NodeHandle nh;
void speedCb( const geometry_msgs::Twist& msg){
  lfd.cmd = msg.linear.x * SPEED_TO_COUNTS;
  lcd.cmd = msg.linear.y * SPEED_TO_COUNTS;
//  lcd.cmd = msg.linear.y;
  lrd.cmd = msg.linear.z * SPEED_TO_COUNTS;
  rfd.cmd = msg.angular.x * SPEED_TO_COUNTS;
  rcd.cmd = msg.angular.y * SPEED_TO_COUNTS;
  rrd.cmd = msg.angular.z * SPEED_TO_COUNTS;
}
void steerCb( const geometry_msgs::Twist& msg){
  lfs.cmd = msg.linear.x * RAD_TO_COUNTS;
  lrs.cmd = msg.linear.z * RAD_TO_COUNTS;
  rfs.cmd = msg.angular.x * RAD_TO_COUNTS;
  rrs.cmd = msg.angular.z * RAD_TO_COUNTS;
}

void headCb( const std_msgs::Float32& msg){
  msg.data;
}
ros::Subscriber<std_msgs::Float32> headSub("head", &headCb );
ros::Subscriber<geometry_msgs::Twist> steerSub("steer", &steerCb );
ros::Subscriber<geometry_msgs::Twist> speedSub("speed", &speedCb );
geometry_msgs::Twist speedFeedbackMsg;
geometry_msgs::Twist steerFeedbackMsg;
std_msgs::Float32 headFeedbackMsg;

ros::Publisher speedPub("speed_feedback", &speedFeedbackMsg);
ros::Publisher steerPub("steer_feedback", &steerFeedbackMsg);
ros::Publisher headPub("head_feedback", &headFeedbackMsg);
void setup()
{  
  enableInterrupt(lfs_pin_a, isr_LFS, RISING);
  enableInterrupt(lrs_pin_a, isr_LRS, RISING);
  enableInterrupt(rfs_pin_a, isr_RFS, RISING);
  enableInterrupt(rrs_pin_a, isr_RRS, RISING);
  enableInterrupt(lfd_pin_a, isr_LFD, RISING);

  enableInterrupt(lrd_pin_a, isr_LRD, RISING);
  enableInterrupt(rfd_pin_a, isr_RFD, RISING);
  enableInterrupt(rrd_pin_a, isr_RRD, RISING);
  enableInterrupt(lcd_pin_a, isr_LCD, RISING);
  enableInterrupt(rcd_pin_a, isr_RCD, RISING);

  enableInterrupt(head_pin_a, isr_HEAD, RISING);
  AFMS_1.begin();
  AFMS_2.begin();
  AFMS_3.begin();
  nh.initNode();
  nh.subscribe(headSub);
  nh.subscribe(steerSub);
  nh.subscribe(speedSub);
  nh.advertise(headPub);
  nh.advertise(steerPub);
  nh.advertise(speedPub);
}
int loopCount = 0;
int loopLimit = 1000/LOOP_FREQ;
void loop()
{  
  nh.spinOnce();
  delay(1);
  if(loopCount++ >= loopLimit) {
    loopCount = 0;
    lfs.loop();
    lrs.loop();
    rfs.loop();
    rrs.loop();
    lfd.loop();
    lrd.loop();
    rfd.loop();
    rrd.loop();
    lcd.loop();
    rcd.loop();
    head.loop();
    headFeedbackMsg.data = head.position;
    steerFeedbackMsg.linear.x = lfs.position;
    steerFeedbackMsg.linear.y = lfd.cmd;
    steerFeedbackMsg.linear.z = lrs.position;
    steerFeedbackMsg.angular.x = rfs.position;
//    steerFeedbackMsg.angular.x = lcd.position;
    steerFeedbackMsg.angular.y = lcd.motor_drive;
//    steerFeedbackMsg.angular.z = lcd.speed;
    steerFeedbackMsg.angular.z = rrs.position;
    speedFeedbackMsg.linear.x = (float)lfd.speed*COUNTS_TO_SPEED;
    speedFeedbackMsg.linear.y = (float)lcd.speed*COUNTS_TO_SPEED;
    speedFeedbackMsg.linear.z = (float)lrd.speed*COUNTS_TO_SPEED;
    speedFeedbackMsg.angular.x = (float)rfd.speed*COUNTS_TO_SPEED;
    speedFeedbackMsg.angular.y = (float)rcd.speed*COUNTS_TO_SPEED;
    speedFeedbackMsg.angular.z = (float)rrd.speed*COUNTS_TO_SPEED;
    headPub.publish(&headFeedbackMsg);
    steerPub.publish(&steerFeedbackMsg);
    speedPub.publish(&speedFeedbackMsg);
  }
}

