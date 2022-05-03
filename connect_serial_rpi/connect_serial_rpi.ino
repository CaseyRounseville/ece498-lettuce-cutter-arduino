#include <AccelStepper.h>
#include <Wire.h>

/**
 * Commands
 * All commands begin with the command identifier,
 * and then followed by 32 bytes of data
 */
#define CMD_00                  0x00
#define CMD_01                  0x01
#define CMD_02                  0x02

#define CMD_MOTOR_1_PWR         0x10 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_2_PWR         0x11 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_3_PWR         0x12 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_4_PWR         0x13 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_5_PWR         0x14 // data[0]=1 -> ON, data[0]=0 -> OFF

#define CMD_MOTOR_1_DST         0x20 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_2_DST         0x21 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_3_DST         0x22 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_4_DST         0x23 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_5_DST         0x24 // data[0]=1 -> ON, data[0]=0 -> OFF

#define CMD_MOTOR_1_VEL         0x30 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_2_VEL         0x31 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_3_VEL         0x32 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_4_VEL         0x33 // data[0]=1 -> ON, data[0]=0 -> OFF
#define CMD_MOTOR_5_VEL         0x34 // data[0]=1 -> ON, data[0]=0 -> OFF

#define CMD_SOLENOID_1_PWR      0x40 // data[0]=1 -> ON, data[0]=0 -> OFF

#define CMD_DATA_LEN 32
#define CMD_BUF_LEN (CMD_DATA_LEN + 1)

// command buffer
// we need space for the command id and the command data
byte global_cmd_buf[CMD_BUF_LEN];

// pins
#define PIN_CMD_00              10
#define PIN_CMD_01              11
#define PIN_CMD_02              12

#define PIN_MOTOR_1_PUL         5
#define PIN_MOTOR_1_DIR         6

#define PIN_MOTOR_2_PUL         8
#define PIN_MOTOR_2_DIR         9

#define PIN_MOTOR_3_PUL         53
#define PIN_MOTOR_3_DIR         52

#define PIN_MOTOR_4_PUL         51
#define PIN_MOTOR_4_DIR         300

#define PIN_MOTOR_5_PUL         49
#define PIN_MOTOR_5_DIR         48

#define PIN_ENCODER_1_AI0       2
#define PIN_ENCODER_1_AI1       3

// because the triggers use pullup resistors for debouncing,
// the default state is HIGH, and the triggered state is LOW
#define PIN_TRIGGER_1           30
#define PIN_TRIGGER_2           32
#define PIN_TRIGGER_3           34
#define PIN_TRIGGER_4           0 // unused
#define PIN_TRIGGER_5           0 // unused

// high connects power to solenoid
// low disconnects power from solenoid
#define PIN_SOLENOID_1          45

// motor 1
// open loop stepper motor
AccelStepper motor1(AccelStepper::DRIVER, PIN_MOTOR_1_PUL, PIN_MOTOR_1_DIR);
long motor1DstVar = 0;
long motor1VelVar = 0;

// motor 2
// closed loop stepper motor
AccelStepper motor2(AccelStepper::DRIVER, PIN_MOTOR_2_PUL, PIN_MOTOR_2_DIR);
long motor2DstVar = 0;
long motor2VelVar = 0;

// motor 3
// open loop stepper motor
AccelStepper motor3(AccelStepper::DRIVER, PIN_MOTOR_3_PUL, PIN_MOTOR_3_DIR);
long motor3DstVar = 0;
long motor3VelVar = 0;

// motor 4
// open loop stepper motor
AccelStepper motor4(AccelStepper::DRIVER, PIN_MOTOR_4_PUL, PIN_MOTOR_4_DIR);
long motor4DstVar = 0;
long motor4VelVar = 0;

// motor 5
// open loop stepper motor
AccelStepper motor5(AccelStepper::DRIVER, PIN_MOTOR_5_PUL, PIN_MOTOR_5_DIR);
long motor5DstVar = 0;
long motor5VelVar = 0;

// encoder 1
// these variables are used in interrupts, so they must be declared volatile
volatile int encoder1Counter = 0;
volatile int encoder1Temp = 0;

// function prototypes
void sendBytesToRpi(byte *buf, size_t len);
void readBytesFromRpi(byte *buf, size_t len);
void readCmdFromRpi(byte *buf);

// encoder stuff
void processEncoders();
void encoder1_ai0();
void encoder1_ai1();

void processMotorAction(
  AccelStepper *motor,
  struct MotorAction *motorActionList,
  int numMotorActions,
  int *motorActionCounter
);
bool motorAction_move                         (AccelStepper *motor, long numSteps,     long arg2);
bool motorAction_movePtrDst                   (AccelStepper *motor, long ptrDst,       long arg2);
bool motorAction_moveTo                       (AccelStepper *motor, long numSteps,     long arg2);
bool motorAction_moveToPtrDst                 (AccelStepper *motor, long ptrDst,       long arg2);
bool motorAction_setSpeed                     (AccelStepper *motor, long speed,        long arg2);
bool motorAction_setMaxSpeed                  (AccelStepper *motor, long maxSpeed,     long arg2);
bool motorAction_setAcceleration              (AccelStepper *motor, long acceleration, long arg2);
bool motorAction_stop                         (AccelStepper *motor, long arg1,         long arg2);
bool motorAction_waitDestinationReached       (AccelStepper *motor, long arg1,         long arg2);
bool motorAction_waitSpeedDestinationReached  (AccelStepper *motor, long speed,        long arg2);
bool motorAction_runSpeedUntilTriggerRises    (AccelStepper *motor, long triggerPin,   long speed);
bool motorAction_runPtrSpeedUntilTriggerRises (AccelStepper *motor, long triggerPin,   long ptrSpeed);
bool motorAction_runSpeedUntilTriggerFalls    (AccelStepper *motor, long triggerPin,   long speed);
bool motorAction_runPtrSpeedUntilTriggerFalls (AccelStepper *motor, long triggerPin,   long ptrSpeed);
bool motorAction_waitUntilTriggerRises        (AccelStepper *motor, long triggerPin,   long arg2);
bool motorAction_waitUntilTriggerFalls        (AccelStepper *motor, long triggerPin,   long arg2);
bool motorAction_waitForever                  (AccelStepper *motor, long arg1,         long arg2);
bool motorAction_waitMs                       (AccelStepper *motor, long ptrTimerInfo, long ms);
bool motorAction_moveByEncoder                (AccelStepper *motor, long ptrCounter,   long ptrTemp);
bool motorAction_activateSolenoid             (AccelStepper *motor, long solenoidPin,  long arg2);
bool motorAction_deactivateSolenoid           (AccelStepper *motor, long solenoidPin,  long arg2);

void handler_CMD_00                           (byte *data);
void handler_CMD_01                           (byte *data);
void handler_CMD_02                           (byte *data);

void handler_CMD_MOTOR_1_PWR                  (byte *data);
void handler_CMD_MOTOR_2_PWR                  (byte *data);
void handler_CMD_MOTOR_3_PWR                  (byte *data);
void handler_CMD_MOTOR_4_PWR                  (byte *data);
void handler_CMD_MOTOR_5_PWR                  (byte *data);

void handler_CMD_MOTOR_1_DST                  (byte *data);
void handler_CMD_MOTOR_2_DST                  (byte *data);
void handler_CMD_MOTOR_3_DST                  (byte *data);
void handler_CMD_MOTOR_4_DST                  (byte *data);
void handler_CMD_MOTOR_5_DST                  (byte *data);

void handler_CMD_MOTOR_1_VEL                  (byte *data);
void handler_CMD_MOTOR_2_VEL                  (byte *data);
void handler_CMD_MOTOR_3_VEL                  (byte *data);
void handler_CMD_MOTOR_4_VEL                  (byte *data);
void handler_CMD_MOTOR_5_VEL                  (byte *data);

void handler_CMD_SOLENOID_1_PWR               (byte *data);

// machine state
enum MachineState {
  /**
   * no motors are running
   * machine waits for sensor at the loader to be triggered,
   * and then transisitions to the loading state
   */
  MACHINE_STATE_IDLE = 99,

  /**
   * all motors run at constant speed until the sensor near
   * the pusher is triggered, and then transitions to the
   * pusherAlignInitial state
   */
  MACHINE_STATE_LOADING = 199,

  /**
   * motors move a predetermined number of steps, in order to
   * line up the first row of holes with the pusher, then
   * transitions tot he pusherActuate state to actually use
   * the pusher
   */
  MACHINE_STATE_PUSHER_ALIGN_INITIAL = 299,

  /**
   * pusher goes down, hopefully hits in the right spot, and
   * then comes back up
   * 
   * if we still have more actuations to do, then transition
   * to the pusherAlignSubsequent state
   * 
   * if we do not have any more actuations to process, then
   * transition to the exiting state
   */
  MACHINE_STATE_PUSHER_ACTUATE = 399,

  /**
   * motors move a predetermined number of steps, in order to
   * line up the next row of holes with the pusher, then
   * transitions tot he pusherActuate state to actually use
   * the pusher
   */
  MACHINE_STATE_PUSHER_ALIGN_SUBSEQUENT = 499,

  /**
   * motors run at constant speed until the sensor near the pusher
   * is no longer triggered, and then motors move a predetermined
   * number of extra steps, to exit the raft from the machine
   * 
   * then, transitions to the idle state
   */
  MACHINE_STATE_EXITING = 599
};
enum MachineState machineState = MACHINE_STATE_IDLE;

void machineState_idle();
void machineState_loading();
void machineState_pusherAlignInitial();
void machineState_pusherActuate();
void machineState_pusherAlignSubsequent();
void machineState_exiting();

// machine state varialbes: MACHINE_STATE_LOADING
long loading_timer = 0;
bool loading_timerSet = false;

// machine state varialbes: MACHINE_STATE_PUSHER_ALIGN_INITIAL
long pusherAlignInitial_numSteps = 3560+50;

// machine state varialbes: MACHINE_STATE_PUSHER_ACTUATE
long pusherActuate_maxActuations = 6; // number of pairs of rows on the raft
long pusherActuate_numActuations = 0;
long pusherActuate_timerDown = 0;
bool pusherActuate_timerDownSet = false;
long pusherActuate_timerUp = 0;
bool pusherActuate_timerUpSet = false;

// machine state variables: MACHINE_STATE_PUSHER_ALIGN_SUBSEQUENT
long pusherAlignSubsequent_numSteps = 3288+12;

// machine state variables: MACHINE_STATE_EXITING
long exiting_numSteps = 3288+12;

struct MotorTimerInfo {
  long timerMs;
  bool timerMsDirty;
};

struct MotorTimerInfo motor1TimerInfo = { 0, true };
struct MotorTimerInfo motor2TimerInfo = { 0, true };
struct MotorTimerInfo motor3TimerInfo = { 0, true };
struct MotorTimerInfo motor4TimerInfo = { 0, true };
struct MotorTimerInfo motor5TimerInfo = { 0, true };

// a struct to represent an action for a motor to perform
struct MotorAction {
  // return whether the action is complete
  bool (*actionFn)(AccelStepper *motor, long arg1, long arg2);
  long arg1;
  long arg2;
};

// idle the motor
struct MotorAction motorActionList_idle[] = {
  { motorAction_stop },
  { motorAction_waitDestinationReached }
};
int motorActionList_idle_numActions = sizeof(motorActionList_idle) / sizeof(struct MotorAction);

// speed test slow
struct MotorAction motorActionList_speedTestSlow[] = {
  //{ motorAction_setMaxSpeed, 3000 },
  //{ motorAction_setAcceleration, 3000 },
  { motorAction_move, 3000 },
  { motorAction_waitDestinationReached },
  { motorAction_move, -3000 },
  { motorAction_waitDestinationReached }
};
int motorActionList_speedTestSlow_numActions = sizeof(motorActionList_speedTestSlow) / sizeof(struct MotorAction);

// run the motor until trigger 1 rises
struct MotorAction motorActionList_triggerTest1[] = {
  { motorAction_waitUntilTriggerFalls, PIN_TRIGGER_1 },
  { motorAction_runSpeedUntilTriggerRises, PIN_TRIGGER_1, 300  },
  { motorAction_stop },
  { motorAction_waitDestinationReached }
};
int motorActionList_triggerTest1_numActions = sizeof(motorActionList_triggerTest1) / sizeof(struct MotorAction);

// run the motor until trigger 2 rises
struct MotorAction motorActionList_triggerTest2[] = {
  { motorAction_waitUntilTriggerFalls, PIN_TRIGGER_2 },
  { motorAction_runSpeedUntilTriggerRises, PIN_TRIGGER_2, 300 },
  { motorAction_stop },
  { motorAction_waitDestinationReached }
};
int motorActionList_triggerTest2_numActions = sizeof(motorActionList_triggerTest2) / sizeof(struct MotorAction);

// run the motor until trigger 3 rises
struct MotorAction motorActionList_triggerTest3[] = {
  { motorAction_waitUntilTriggerFalls, PIN_TRIGGER_3 },
  { motorAction_runSpeedUntilTriggerRises, PIN_TRIGGER_3, 300 },
  { motorAction_stop },
  { motorAction_waitDestinationReached }
};
int motorActionList_triggerTest3_numActions = sizeof(motorActionList_triggerTest3) / sizeof(struct MotorAction);

// run the motor until trigger 4 rises
struct MotorAction motorActionList_triggerTest4[] = {
  { motorAction_waitUntilTriggerFalls, PIN_TRIGGER_4 },
  { motorAction_runSpeedUntilTriggerRises, PIN_TRIGGER_4, 300 },
  { motorAction_stop },
  { motorAction_waitDestinationReached }
};
int motorActionList_triggerTest4_numActions = sizeof(motorActionList_triggerTest4) / sizeof(struct MotorAction);

// run the motor until trigger 5 rises
struct MotorAction motorActionList_triggerTest5[] = {
  { motorAction_waitUntilTriggerFalls, PIN_TRIGGER_5 },
  { motorAction_runSpeedUntilTriggerRises, PIN_TRIGGER_5, 300 },
  { motorAction_stop },
  { motorAction_waitDestinationReached }
};
int motorActionList_triggerTest5_numActions = sizeof(motorActionList_triggerTest5) / sizeof(struct MotorAction);

// distance test for motor 2
struct MotorAction motorActionList_motor2DstTest[] = {
  { motorAction_movePtrDst, &motor2DstVar },
  { motorAction_waitDestinationReached },
  { motorAction_waitForever }
};
int motorActionList_motor2DstTest_numActions = sizeof(motorActionList_motor2DstTest) / sizeof(struct MotorAction);

// velocity test for motor 3
struct MotorAction motorActionList_motor3VelTest[] = {
  { motorAction_runPtrSpeedUntilTriggerFalls, PIN_TRIGGER_1, &motor3VelVar },
  { motorAction_stop },
  { motorAction_waitDestinationReached },
  { motorAction_waitUntilTriggerRises, PIN_TRIGGER_1 }
};
int motorActionList_motor3VelTest_numActions = sizeof(motorActionList_motor3VelTest) / sizeof(struct MotorAction);

// encoder test
struct MotorAction motorActionList_encoderTest[] = {
  { motorAction_moveByEncoder, &encoder1Counter, &encoder1Temp },
  { motorAction_waitDestinationReached },
};
int motorActionList_encoderTest_numActions = sizeof(motorActionList_encoderTest) / sizeof(struct MotorAction);

// motor 3 main
struct MotorAction motorActionList_motor3Main[] = {
  { motorAction_move, 30000 },
  { motorAction_waitDestinationReached }
};
int motorActionList_motor3Main_numActions = sizeof(motorActionList_motor3Main) / sizeof(struct MotorAction);

// motor 3 main2
struct MotorAction motorActionList_motor3Main2[] = {
  { motorAction_move, 300 },
  { motorAction_waitDestinationReached },
  { motorAction_activateSolenoid, PIN_SOLENOID_1 },
  { motorAction_waitMs, &motor3TimerInfo, 3000 },
  { motorAction_deactivateSolenoid, PIN_SOLENOID_1 },
  { motorAction_waitMs, &motor3TimerInfo, 3000 }
};
int motorActionList_motor3Main2_numActions = sizeof(motorActionList_motor3Main2) / sizeof(struct MotorAction);

// loading the raft
struct MotorAction motorActionList_loading[] = {
  // run until the trigger near the pusher is triggered
  { motorAction_runSpeedUntilTriggerFalls, PIN_TRIGGER_1, 400 },
  { motorAction_stop },
  { motorAction_waitDestinationReached }
};
int motorActionList_loading_numActions = sizeof(motorActionList_loading) / sizeof(struct MotorAction);

// aligning initial for the pusher
struct MotorAction motorActionList_pusherAlignInitial[] = {
  // run a predetermined number of steps
  { motorAction_move, pusherAlignInitial_numSteps },
  { motorAction_waitDestinationReached }
  
};
int motorActionList_pusherAlignInitial_numActions = sizeof(motorActionList_pusherAlignInitial) / sizeof(struct MotorAction);

// aligning subsequent for the pusher
struct MotorAction motorActionList_pusherAlignSubsequent[] = {
  // move a predetermined number of steps
  { motorAction_move, pusherAlignSubsequent_numSteps },
  { motorAction_waitDestinationReached }
};
int motorActionList_pusherAlignSubsequent_numActions = sizeof(motorActionList_pusherAlignSubsequent) / sizeof(struct MotorAction);

// exiting the raft
struct MotorAction motorActionList_exiting[] = {
  // run until the trigger near the pusher is no longer triggered
  { motorAction_runSpeedUntilTriggerRises, PIN_TRIGGER_1, 400 },
  { motorAction_stop },
  { motorAction_waitDestinationReached },
  
  // move the extra steps to exit the raft
  { motorAction_move, exiting_numSteps },
  { motorAction_waitDestinationReached }
  
//  { motorAction_activateSolenoid, PIN_SOLENOID_1 },
//  { motorAction_waitMs, &motor3TimerInfo, 3000 },
//  { motorAction_deactivateSolenoid, PIN_SOLENOID_1 },
//  { motorAction_waitMs, &motor3TimerInfo, 3000 }
};
int motorActionList_exiting_numActions = sizeof(motorActionList_exiting) / sizeof(struct MotorAction);

// motor 1 action list
//struct MotorAction *motor1ActionList = motorActionList_idle;
//int motor1NumActions = motorActionList_idle_numActions;
//int motor1ActionCounter = 0;
struct MotorAction *motor1ActionList = motorActionList_idle;
int motor1NumActions = motorActionList_idle_numActions;
int motor1ActionCounter = 0;
//struct MotorAction *motor1ActionList = motorActionList_encoderTest;
//int motor1NumActions = motorActionList_encoderTest_numActions;
//int motor1ActionCounter = 0;

// motor 2 action list
//struct MotorAction *motor2ActionList = motorActionList_idle;
//int motor2NumActions = motorActionList_idle_numActions;
//int motor2ActionCounter = 0;
struct MotorAction *motor2ActionList = motorActionList_idle;
int motor2NumActions = motorActionList_idle_numActions;
int motor2ActionCounter = 0;

// motor 3 action list
//struct MotorAction *motor3ActionList = motorActionList_encoderTest;
//int motor3NumActions = motorActionList_encoderTest_numActions;
//int motor3ActionCounter = 0;
struct MotorAction *motor3ActionList = motorActionList_idle;
int motor3NumActions = motorActionList_idle_numActions;
int motor3ActionCounter = 0;
//struct MotorAction *motor3ActionList = motorActionList_triggerTest2;
//int motor3NumActions = motorActionList_triggerTest2_numActions;
//int motor3ActionCounter = 0;

// motor 4 action list
struct MotorAction *motor4ActionList = motorActionList_idle;
int motor4NumActions = motorActionList_idle_numActions;
int motor4ActionCounter = 0;

// motor 5 action list
struct MotorAction *motor5ActionList = motorActionList_idle;
int motor5NumActions = motorActionList_idle_numActions;
int motor5ActionCounter = 0;

/**
 * https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/
 */
void setup() {  
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(3000);

  // zero out the command buffer
  memset(global_cmd_buf, 0, CMD_BUF_LEN);

  // initialize pins
  pinMode(PIN_CMD_00,           OUTPUT);
  pinMode(PIN_CMD_01,           OUTPUT);
  pinMode(PIN_CMD_02,           OUTPUT);
  
  pinMode(PIN_MOTOR_1_PUL,      OUTPUT);
  pinMode(PIN_MOTOR_1_DIR,      OUTPUT);
  
  pinMode(PIN_MOTOR_2_PUL,      OUTPUT);
  pinMode(PIN_MOTOR_2_DIR,      OUTPUT);
  
  pinMode(PIN_MOTOR_3_PUL,      OUTPUT);
  pinMode(PIN_MOTOR_3_DIR,      OUTPUT);

  pinMode(PIN_TRIGGER_1,        INPUT_PULLUP);
  pinMode(PIN_TRIGGER_2,        INPUT_PULLUP);
  pinMode(PIN_TRIGGER_3,        INPUT_PULLUP); // mechanical switch needs pullup resistor for debounce
  pinMode(PIN_TRIGGER_4,        INPUT_PULLUP);
  pinMode(PIN_TRIGGER_5,        INPUT_PULLUP);

  pinMode(PIN_SOLENOID_1,       OUTPUT);

  // initialize motor 1
  // limit to 33000 to avoid getting stuck when both motors are on
  motor1.setMaxSpeed(300);
  motor1.setAcceleration(300);

  // initialize motor 2
  motor2.setMaxSpeed(300);
  motor2.setAcceleration(300);

  // initialize motor 3
  motor3.setMaxSpeed(300);
  motor3.setAcceleration(300);

  // initialize encoder 1
  pinMode(PIN_ENCODER_1_AI0, INPUT_PULLUP);
  pinMode(PIN_ENCODER_1_AI1, INPUT_PULLUP);
  attachInterrupt(0, encoder1_ai0, RISING); // interrupt 0 is pin 2
  attachInterrupt(1, encoder1_ai1, RISING); // interrupt 1 is pin 3
}

void loop() {
  //Serial.print("state ");
  Serial.println(machineState);
  switch (machineState) {
    case MACHINE_STATE_IDLE:                    machineState_idle(); break;
    case MACHINE_STATE_LOADING:                 machineState_loading(); break;
    case MACHINE_STATE_PUSHER_ALIGN_INITIAL:    machineState_pusherAlignInitial(); break;
    case MACHINE_STATE_PUSHER_ACTUATE:          machineState_pusherActuate(); break;
    case MACHINE_STATE_PUSHER_ALIGN_SUBSEQUENT: machineState_pusherAlignSubsequent(); break;
    case MACHINE_STATE_EXITING:                 machineState_exiting(); break;
  }
  
  // process motor actions
  processMotorAction(&motor1, motor1ActionList, motor1NumActions, &motor1ActionCounter);
  processMotorAction(&motor2, motor2ActionList, motor2NumActions, &motor2ActionCounter);
  processMotorAction(&motor3, motor3ActionList, motor3NumActions, &motor3ActionCounter);

  // process encoders
  processEncoders();

  if (Serial.available() > 0) {
    // read command from rpi
    readCmdFromRpi(global_cmd_buf);
  
    // handle the command
    byte cmdId = global_cmd_buf[0];
    switch (cmdId) {
      case CMD_00:              handler_CMD_00(global_cmd_buf + 1);           break;
      case CMD_01:              handler_CMD_01(global_cmd_buf + 1);           break;
      case CMD_02:              handler_CMD_02(global_cmd_buf + 1);           break;
      
      case CMD_MOTOR_1_PWR:     handler_CMD_MOTOR_1_PWR(global_cmd_buf + 1);  break;
      case CMD_MOTOR_2_PWR:     handler_CMD_MOTOR_2_PWR(global_cmd_buf + 1);  break;
      case CMD_MOTOR_3_PWR:     handler_CMD_MOTOR_3_PWR(global_cmd_buf + 1);  break;
      case CMD_MOTOR_4_PWR:     handler_CMD_MOTOR_4_PWR(global_cmd_buf + 1);  break;
      case CMD_MOTOR_5_PWR:     handler_CMD_MOTOR_5_PWR(global_cmd_buf + 1);  break;

      case CMD_MOTOR_1_DST:     handler_CMD_MOTOR_1_DST(global_cmd_buf + 1);  break;
      case CMD_MOTOR_2_DST:     handler_CMD_MOTOR_2_DST(global_cmd_buf + 1);  break;
      case CMD_MOTOR_3_DST:     handler_CMD_MOTOR_3_DST(global_cmd_buf + 1);  break;
      case CMD_MOTOR_4_DST:     handler_CMD_MOTOR_4_DST(global_cmd_buf + 1);  break;
      case CMD_MOTOR_5_DST:     handler_CMD_MOTOR_5_DST(global_cmd_buf + 1);  break;

      case CMD_MOTOR_1_VEL:     handler_CMD_MOTOR_1_VEL(global_cmd_buf + 1);  break;
      case CMD_MOTOR_2_VEL:     handler_CMD_MOTOR_2_VEL(global_cmd_buf + 1);  break;
      case CMD_MOTOR_3_VEL:     handler_CMD_MOTOR_3_VEL(global_cmd_buf + 1);  break;
      case CMD_MOTOR_4_VEL:     handler_CMD_MOTOR_4_VEL(global_cmd_buf + 1);  break;
      case CMD_MOTOR_5_VEL:     handler_CMD_MOTOR_5_VEL(global_cmd_buf + 1);  break;
      
      case CMD_SOLENOID_1_PWR:  handler_CMD_SOLENOID_1_PWR(global_cmd_buf + 1);  break;
      default:
        // unknown command, do nothing
        break;
    }
  }
}

void sendBytesToRpi(byte *buf, size_t len) {
  Serial.write(buf, len);
}

/**
 * Blocks until len bytes are read, or the waiting times out.
 */
void readBytesFromRpi(byte *buf, size_t len) {
  Serial.readBytes(buf, len);
}

void readCmdFromRpi(byte *buf) {
  // we must read one byte for the command id,
  // and CMD_DATA_LEN for the data
  readBytesFromRpi(buf, CMD_BUF_LEN);
}

void processEncoders() {
  // encoder 1
  if (encoder1Counter != encoder1Temp) {
    //Serial.println(encoder1Counter);
    encoder1Temp = encoder1Counter;
  }
  //Serial.println(encoder1Counter);
}

void encoder1_ai0() {
  if (digitalRead(PIN_ENCODER_1_AI1) == LOW) {
    encoder1Counter++;
  } else {
    encoder1Counter--;
  }
}

void encoder1_ai1() {
  if (digitalRead(PIN_ENCODER_1_AI0) == LOW) {
    encoder1Counter--;
  } else {
    encoder1Counter++;
  }
}

void processMotorAction(
  AccelStepper *motor,
  struct MotorAction *motorActionList,
  int numMotorActions,
  int *motorActionCounter
) {
  // call the function to execute the action
  struct MotorAction *action = motorActionList + *motorActionCounter;
  bool done = action->actionFn(motor, action->arg1, action->arg2);

  // if the action is done, move on to the next action, or wrap around to the start
  if (done) {
    (*motorActionCounter)++;
    if (*motorActionCounter == numMotorActions) {
      *motorActionCounter = 0;
    }
  }
  return done;
}
/**
 * Tell the motor to move numSteps number of steps.
 * 
 * @param motor the motor to move
 * @param numSteps how many steps to tell the motor to move
 * @param arg2 unused
 * 
 * @return true
 */
bool motorAction_move(AccelStepper *motor, long numSteps, long arg2) {
  motor->move(numSteps);
  motor->run();
  return true;
}

bool motorAction_movePtrDst(AccelStepper *motor, long ptrDst, long arg2) {
  return motorAction_move(motor, *((long *)ptrDst), arg2);
}

/**
 * Tell the motor to move to a specific number of steps away from
 * its "origin" zero position.
 * 
 * @param motor the motor to move
 * @param numSteps how many steps away from zero we want the motor to be
 * @param arg2 unused
 * 
 * @return true
 */
bool motorAction_moveTo(AccelStepper *motor, long numSteps, long arg2) {
  motor->moveTo(numSteps);
  motor->run();
  return true;
}

bool motorAction_moveToPtrDst(AccelStepper *motor, long ptrDst, long arg2) {
  return motorAction_moveTo(motor, *((long *)ptrDst), arg2);
}

/**
 * Set the speed of the motor.
 * 
 * @param motor the motor to set the speed
 * @param speed the speed we want, max is 4000
 * @param arg2 unused
 * 
 * @return true
 */
bool motorAction_setSpeed(AccelStepper *motor, long speed, long arg2) {
  motor->setSpeed(speed);
  motor->run();
  return true;
}

/**
 * Set the max speed of the motor.
 * 
 * @param motor the motor to set the max speed
 * @param maxSpeed the max speed, max is 4000
 * @param arg2 unused
 * 
 * @return true
 */
bool motorAction_setMaxSpeed(AccelStepper *motor, long maxSpeed, long arg2) {
  motor->setMaxSpeed(maxSpeed);
  motor->run();
  return true;
}

/**
 * Set the acceleration of the motor.
 * 
 * @param motor the motor to set the acceleration
 * @param acceleration the acceleration we want
 * @param arg2 unused
 * 
 * @return true
 */
bool motorAction_setAcceleration(AccelStepper *motor, long acceleration, long arg2) {
  motor->setAcceleration(acceleration);
  motor->run();
  return true;
}

/**
 * Tell the motor to stop.
 * 
 * @param motor the motor to tell to stop
 * @param arg1 unused
 * @param arg2 unused
 * 
 * @return true
 */
bool motorAction_stop(AccelStepper *motor, long arg1, long arg2) {
  motor->stop();
  motor->run();
  return true;
}

/**
 * Wait until the motor reaches its destination.
 * 
 * @param motor the motor we want to wait for
 * @param arg1 unused
 * @param arg2 unused
 * 
 * @return true if the motor has reached its destination, or false otherwise
 */
bool motorAction_waitDestinationReached(AccelStepper *motor, long arg1, long arg2) {
  motor->run();
  return motor->distanceToGo() == 0;
}

bool motorAction_waitSpeedDestinationReached  (AccelStepper *motor, long speed, long arg2) {
  motor->setSpeed(speed);
  motor->runSpeed();
  return motor->distanceToGo() == 0;
}

/**
 * Run the motor at constant speed until the specified trigger pin is HIGH,
 * and then tell the motor to stop.
 * 
 * @param motor the motor to run at constant speed
 * @param triggerPin the pin to check if it is HIGH
 * @param speed the speed to run the motor
 * 
 * @return true if the trigger pin is HIGH, or false otherwise
 */
bool motorAction_runSpeedUntilTriggerRises(AccelStepper *motor, long triggerPin, long speed) {
  if (digitalRead(triggerPin) == HIGH) {
    motor->stop();
    motor->run();
    return true;
  } else {
    motor->setSpeed(speed);
    motor->runSpeed();
    return false;
  }
}

bool motorAction_runPtrSpeedUntilTriggerRises (AccelStepper *motor, long triggerPin, long ptrSpeed) {
  return motorAction_runSpeedUntilTriggerRises(motor, triggerPin, *((long *)ptrSpeed));
}

/**
 * Run the motor at constant speed until the specified trigger pin is LOW,
 * and then tell the motor to stop.
 * 
 * @param motor the motor to run at constant speed
 * @param triggerPin the pin to check if it is LOW
 * @param speed the speed to run the motor
 * 
 * @return true if the trigger pin is LOW, or false otherwise
 */
bool motorAction_runSpeedUntilTriggerFalls(AccelStepper *motor, long triggerPin, long speed) {
  if (digitalRead(triggerPin) == LOW) {
    motor->stop();
    motor->run();
    return true;
  } else {
    motor->setSpeed(speed);
    motor->runSpeed();
    return false;
  }
}

bool motorAction_runPtrSpeedUntilTriggerFalls (AccelStepper *motor, long triggerPin, long ptrSpeed) {
  return motorAction_runSpeedUntilTriggerFalls(motor, triggerPin, *((long *)ptrSpeed));
}

/**
 * Wait until the specified trigger in is HIGH, continuing to run the motor
 * in whatever state it is currently in.
 * 
 * @param motor the motor to run
 * @param triggerPin the pin to check if it is HIGH
 * @param arg2 unused
 * 
 * @return true if the trigger pin is HIGH, or false otherwise
 */
bool motorAction_waitUntilTriggerRises(AccelStepper *motor, long triggerPin, long arg2) {
  if (digitalRead(triggerPin) == HIGH) {
    motor->run();
    return true;
  } else {
    motor->run();
    return false;
  }
}

/**
 * Wait until the specified trigger in is LOW, continuing to run the motor
 * in whatever state it is currently in.
 * 
 * @param motor the motor to run
 * @param triggerPin the pin to check if it is LOW
 * @param arg2 unused
 * 
 * @return true if the trigger pin is HIGH, or false otherwise
 */
bool motorAction_waitUntilTriggerFalls(AccelStepper *motor, long triggerPin, long arg2) {
  if (digitalRead(triggerPin) == LOW) {
    motor->run();
    return true;
  } else {
    motor->run();
    return false;
  }
}

bool motorAction_waitForever(AccelStepper *motor, long arg1, long arg2) {
  return false;
}

bool motorAction_waitMs(AccelStepper *motor, long ptrTimerInfo, long ms) {
  struct MotorTimerInfo *ptrCastedTimerInfo = (struct MotorTimerInfo *)ptrTimerInfo;
  long timeStampNowMs = millis();
  if (ptrCastedTimerInfo->timerMsDirty) {
    // reset
    ptrCastedTimerInfo->timerMs = timeStampNowMs;
    ptrCastedTimerInfo->timerMsDirty = false;
    return false;
  } else if (timeStampNowMs - ptrCastedTimerInfo->timerMs >= ms) {
    ptrCastedTimerInfo->timerMsDirty = true;
    return true;
  } else {
    return false;
  }
}

bool motorAction_moveByEncoder(AccelStepper *motor, long ptrCounter, long ptrTemp) {
  volatile int counter = *((volatile int *)ptrCounter);
  volatile int temp = *((volatile int *)ptrTemp);

  // every 5 pulses on the encoder translates to 1 pulse on the motor
  int encoderStep2motorStep = 5;

  int numFullSteps = counter / encoderStep2motorStep;
  if (numFullSteps != 0) { // could be negative, which is just direction
    // tell the motor how much to move
    motor->move(numFullSteps);
    motor->run();

    // reset the counts, accounting for fractional steps
    if (numFullSteps > 0) {
      *((volatile int *)ptrCounter) = *((volatile int *)ptrCounter) % encoderStep2motorStep;
      //*((int *)ptrTemp) = *((int *)ptrTemp) % encoderStep2motorStep;
    } else {
      *((volatile int *)ptrCounter) = -(-*((volatile int *)ptrCounter) % encoderStep2motorStep);
      //*((int *)ptrTemp) = -(-*((int *)ptrTemp) % encoderStep2motorStep);
    }
    return true;
  } else {
    return false;
  }
}

bool motorAction_activateSolenoid(AccelStepper *motor, long solenoidPin,  long arg2) {
  digitalWrite(solenoidPin, HIGH);
  return true;
}

bool motorAction_deactivateSolenoid(AccelStepper *motor, long solenoidPin,  long arg2) {
  digitalWrite(solenoidPin, LOW);
  return true;
}

void handler_CMD_00(byte *data) {
  if (data[0] == 0) {
    digitalWrite(PIN_CMD_00, LOW);
  } else {
    digitalWrite(PIN_CMD_00, HIGH);
  }
}

void handler_CMD_01(byte *data) {
  if (data[0] == 0) {
    digitalWrite(PIN_CMD_01, LOW);
  } else {
    digitalWrite(PIN_CMD_01, HIGH);
  }
}

void handler_CMD_02(byte *data) {
  if (data[0] == 0) {
    digitalWrite(PIN_CMD_02, LOW);
  } else {
    digitalWrite(PIN_CMD_02, HIGH);
  }
}

void handler_CMD_MOTOR_1_PWR(byte *data) {
  if (data[0] == 0) {
    motor1.stop();
    motor1.run();
    motor1ActionList = motorActionList_idle;
    motor1NumActions = motorActionList_idle_numActions;
    motor1ActionCounter = 0;
  } else {
    //motor1.move(0x7FFFFFFF);
    motor1ActionList = motorActionList_triggerTest1;
    motor1NumActions = motorActionList_triggerTest1_numActions;
    motor1ActionCounter = 0;
  }
}

void handler_CMD_MOTOR_2_PWR(byte *data) {
  if (data[0] == 0) {
    motor2.stop();
    motor2.run();
    motor2ActionList = motorActionList_idle;
    motor2NumActions = motorActionList_idle_numActions;
    motor2ActionCounter = 0;
  } else {
    //motor2.move(0x7FFFFFFF);
    motor2ActionList = motorActionList_motor2DstTest;
    motor2NumActions = motorActionList_motor2DstTest_numActions;
    motor2ActionCounter = 0;
  }
}

void handler_CMD_MOTOR_3_PWR(byte *data) {
  if (data[0] == 0) {
    motor3.stop();
    motor3.run();
    motor3ActionList = motorActionList_idle;
    motor3NumActions = motorActionList_idle_numActions;
    motor3ActionCounter = 0;
  } else {
    //motor3.move(0x7FFFFFFF);
    motor3ActionList = motorActionList_motor3VelTest;
    motor3NumActions = motorActionList_motor3VelTest_numActions;
    motor3ActionCounter = 0;
  }
}

void handler_CMD_MOTOR_4_PWR(byte *data) {
  if (data[0] == 0) {
    motor4.stop();
    motor4.run();
    motor4ActionList = motorActionList_idle;
    motor4NumActions = motorActionList_idle_numActions;
    motor4ActionCounter = 0;
  } else {
    //motor4.move(0x7FFFFFFF);
    motor4ActionList = motorActionList_speedTestSlow;
    motor4NumActions = motorActionList_speedTestSlow_numActions;
    motor4ActionCounter = 0;
  }
}

void handler_CMD_MOTOR_5_PWR(byte *data) {
  if (data[0] == 0) {
    motor5.stop();
    motor5.run();
    motor5ActionList = motorActionList_idle;
    motor5NumActions = motorActionList_idle_numActions;
    motor5ActionCounter = 0;
  } else {
    //motor5.move(0x7FFFFFFF);
    motor5ActionList = motorActionList_speedTestSlow;
    motor5NumActions = motorActionList_speedTestSlow_numActions;
    motor5ActionCounter = 0;
  }
}

void handler_CMD_MOTOR_1_DST(byte *data) {
  int dst = (data[0] << 8) | data[1];
  motor1DstVar = dst;
}

void handler_CMD_MOTOR_2_DST(byte *data) {
  int dst = (data[0] << 8) | data[1];
  motor2DstVar = dst;
}

void handler_CMD_MOTOR_3_DST(byte *data) {
  int dst = (data[0] << 8) | data[1];
  motor3DstVar = dst;
}

void handler_CMD_MOTOR_4_DST(byte *data) {
  int dst = (data[0] << 8) | data[1];
  motor4DstVar = dst;
}

void handler_CMD_MOTOR_5_DST(byte *data) {
  int dst = (data[0] << 8) | data[1];
  motor5DstVar = dst;
}

void handler_CMD_MOTOR_1_VEL(byte *data) {
  int vel = (data[0] << 8) | data[1];
  motor1VelVar = vel;
}

void handler_CMD_MOTOR_2_VEL(byte *data) {
  int vel = (data[0] << 8) | data[1];
  motor2VelVar = vel;
}

void handler_CMD_MOTOR_3_VEL(byte *data) {
  int vel = (data[0] << 8) | data[1];
  motor3VelVar = vel;
}

void handler_CMD_MOTOR_4_VEL(byte *data) {
  int vel = (data[0] << 8) | data[1];
  motor4VelVar = vel;
}

void handler_CMD_MOTOR_5_VEL(byte *data) {
  int vel = (data[0] << 8) | data[1];
  motor5VelVar = vel;
}

void handler_CMD_SOLENOID_1_PWR(byte *data) {
  if (data[0] == 0) {
    // turn solenoid off
    digitalWrite(PIN_SOLENOID_1, LOW);
  } else {
    // turn solenoid on
    digitalWrite(PIN_SOLENOID_1, HIGH);
  }
}

void machineState_idle() {
  // only allow pusher in idle state, to avoid breaking the raft
  // pusher stuff
  if (digitalRead(PIN_TRIGGER_3) == LOW) {
    // turn solenoid on
    digitalWrite(PIN_SOLENOID_1, HIGH);
  } else {
    // turn solenoid off
    digitalWrite(PIN_SOLENOID_1, LOW);
  }
  
  if (digitalRead(PIN_TRIGGER_2) == LOW) {
    // motor 1
    motor1ActionList = motorActionList_loading;
    motor1NumActions = motorActionList_loading_numActions;
    motor1ActionCounter = 0;
  
    // motor 2
    motor2ActionList = motorActionList_loading;
    motor2NumActions = motorActionList_loading_numActions;
    motor2ActionCounter = 0;
  
    // motor 3
    motor3ActionList = motorActionList_loading;
    motor3NumActions = motorActionList_loading_numActions;
    motor3ActionCounter = 0;
    
    machineState = MACHINE_STATE_LOADING;
  }
}

void machineState_loading() {
  if (
    digitalRead(PIN_TRIGGER_1) == LOW &
    digitalRead(PIN_TRIGGER_1) == LOW &&
    digitalRead(PIN_TRIGGER_1) == LOW &&
    digitalRead(PIN_TRIGGER_1) == LOW &&
    digitalRead(PIN_TRIGGER_1) == LOW
  ) {
    // transition to initial alignment for pusher
    //while (digitalRead(PIN_TRIGGER_2) == HIGH) { /* do nothing */ }
    // motor 1
    motor1ActionList = motorActionList_pusherAlignInitial;
    motor1NumActions = motorActionList_pusherAlignInitial_numActions;
    motor1ActionCounter = 0;
  
    // motor 2
    motor2ActionList = motorActionList_pusherAlignInitial;
    motor2NumActions = motorActionList_pusherAlignInitial_numActions;
    motor2ActionCounter = 0;
  
    // motor 3
    motor3ActionList = motorActionList_pusherAlignInitial;
    motor3NumActions = motorActionList_pusherAlignInitial_numActions;
    motor3ActionCounter = 0;
    
    machineState = MACHINE_STATE_PUSHER_ALIGN_INITIAL;
  }
}

void machineState_pusherAlignInitial() {
  if (motor1.distanceToGo() == 0) {
    // motor 1
    motor1ActionList = motorActionList_idle;
    motor1NumActions = motorActionList_idle_numActions;
    motor1ActionCounter = 0;
  
    // motor 2
    motor2ActionList = motorActionList_idle;
    motor2NumActions = motorActionList_idle_numActions;
    motor2ActionCounter = 0;
  
    // motor 3
    motor3ActionList = motorActionList_idle;
    motor3NumActions = motorActionList_idle_numActions;
    motor3ActionCounter = 0;
    
    machineState = MACHINE_STATE_PUSHER_ACTUATE;
  }
}

void machineState_pusherActuate() {  
  if (pusherActuate_timerDownSet) {
    // the down timer is set, which means we already sent the pusher down
    if (millis() - pusherActuate_timerDown > 1000) {
      // clear the down timer
      pusherActuate_timerDown = 0;
      pusherActuate_timerDownSet = false;

      // set the up timer, and send the pusher up
      pusherActuate_timerUp = millis();
      pusherActuate_timerUpSet = true;
      //digitalWrite(PIN_SOLENOID_1, LOW);
    }
  } else if (pusherActuate_timerUpSet) {
    // the up timer is set, which means the pusher is coming up
    if (millis() - pusherActuate_timerUp > 1000) {
      // clear the up timer
      pusherActuate_timerUp = 0;
      pusherActuate_timerUpSet = false;

      // increment number of actuations
      pusherActuate_numActuations++;

      // transition to next state
      if (pusherActuate_numActuations == pusherActuate_maxActuations) {
        // reset count
        pusherActuate_numActuations = 0;
        
        // transisition to exiting the raft
      
        // motor 1
        motor1ActionList = motorActionList_exiting;
        motor1NumActions = motorActionList_exiting_numActions;
        motor1ActionCounter = 0;
      
        // motor 2
        motor2ActionList = motorActionList_exiting;
        motor2NumActions = motorActionList_exiting_numActions;
        motor2ActionCounter = 0;

        // motor 3
        motor3ActionList = motorActionList_exiting;
        motor3NumActions = motorActionList_exiting_numActions;
        motor3ActionCounter = 0;
        
        machineState = MACHINE_STATE_EXITING;
        //while (digitalRead(PIN_TRIGGER_2) == HIGH) { /* do nothing */ }
      } else {
        // transition to aligning the subsequent row for the pusher

        // motor 1
        motor1ActionList = motorActionList_pusherAlignSubsequent;
        motor1NumActions = motorActionList_pusherAlignSubsequent_numActions;
        motor1ActionCounter = 0;
      
        // motor 2
        motor2ActionList = motorActionList_pusherAlignSubsequent;
        motor2NumActions = motorActionList_pusherAlignSubsequent_numActions;
        motor2ActionCounter = 0;
      
        // motor 3
        motor3ActionList = motorActionList_pusherAlignSubsequent;
        motor3NumActions = motorActionList_pusherAlignSubsequent_numActions;
        motor3ActionCounter = 0;

        machineState = MACHINE_STATE_PUSHER_ALIGN_SUBSEQUENT;
        //while (digitalRead(PIN_TRIGGER_2) == HIGH) { /* do nothing */ }
      }
    }
  } else {
    // set the down timer and send the pusher down
    pusherActuate_timerDown = millis();
    pusherActuate_timerDownSet = true;
    //digitalWrite(PIN_SOLENOID_1, HIGH);
  }
}

void machineState_pusherAlignSubsequent() {
  if (motor1.distanceToGo() == 0) {
    // motor 1
    motor1ActionList = motorActionList_idle;
    motor1NumActions = motorActionList_idle_numActions;
    motor1ActionCounter = 0;
  
    // motor 2
    motor2ActionList = motorActionList_idle;
    motor2NumActions = motorActionList_idle_numActions;
    motor2ActionCounter = 0;
  
    // motor 3
    motor3ActionList = motorActionList_idle;
    motor3NumActions = motorActionList_idle_numActions;
    motor3ActionCounter = 0;
    
    machineState = MACHINE_STATE_PUSHER_ACTUATE;
  }
}

void machineState_exiting() {
  if (
    digitalRead(PIN_TRIGGER_1 == HIGH) &&
    digitalRead(PIN_TRIGGER_1 == HIGH) &&
    digitalRead(PIN_TRIGGER_1 == HIGH) &&
    digitalRead(PIN_TRIGGER_1 == HIGH) &&
    digitalRead(PIN_TRIGGER_1 == HIGH) &&
    motor1.distanceToGo() == 0
  ) {
    // motor 1
    motor1ActionList = motorActionList_idle;
    motor1NumActions = motorActionList_idle_numActions;
    motor1ActionCounter = 0;
  
    // motor 2
    motor2ActionList = motorActionList_idle;
    motor2NumActions = motorActionList_idle_numActions;
    motor2ActionCounter = 0;
  
    // motor 3
    motor3ActionList = motorActionList_idle;
    motor3NumActions = motorActionList_idle_numActions;
    motor3ActionCounter = 0;
    
    machineState = MACHINE_STATE_IDLE;
  }
}
