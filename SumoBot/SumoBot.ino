#include <Servo.h>
#include <QTRSensors.h>


//Outputs
  //2 motors (PWM)
    constexpr int PIN_MOTOR_LEFT = 23;
    constexpr int PIN_MOTOR_RIGHT = 22;
    constexpr int PIN_LED = 13;
//Inputs 
  //4 interupt pins (digital inputs) edge of field sensors
    constexpr int PIN_LINE_FRONT_LEFT = 14; // Green
    constexpr int PIN_LINE_FRONT_RIGHT = 15; // White
    constexpr int PIN_LINE_BACK_LEFT = 16; // orange
    constexpr int PIN_LINE_BACK_RIGHT = 17; // yellow
    
  //2 ultrasonic sensors (analog) pin 2 on sensor
    constexpr int PIN_SONIC_LEFT = 20;
    constexpr int PIN_SONIC_RIGHT = 21;

constexpr int NEUTRAL_LEFT = 94;
constexpr int NEUTRAL_RIGHT = 96;
constexpr int MAX_COAST_SPEED = 15;
constexpr int SONIC_THRESHOLD = 130;
constexpr float DRIVE_VALUE_F = 0.8;
constexpr float DRIVE_VALUE_B = -0.6;
constexpr uint16_t LINE_SENSOR_THRESHOLD = 400;

constexpr bool TESTING_MODE = false;

enum stateTypes {
    SEEK_STATE      = 0,
    PURSUIT_STATE   = 1,
    ATTACK_STATE    = 2,
    OVERDRIVE_STATE = 3,
    EDGE_STATE      = 4,
    HALT_STATE      = 5,
    DEAD_STATE      = 6    
};

enum lineSensorID {
    FRONTLEFT   = 0, 
    FRONTRIGHT  = 1, 
    BACKLEFT    = 2, 
    BACKRIGHT   = 3
};

enum sonicID {
    LEFT  = 20,
    RIGHT = 21
};
    

Servo motorLeft;
Servo motorRight;


QTRSensors qtr;
uint16_t sensorValues[4];

float rampup = 0;

int activeState = PURSUIT_STATE;

//writes pwm to motors, scales -1 to 1 max speed values range 
void differentialDrive(float left, float right){
  motorLeft.write(NEUTRAL_LEFT - left * MAX_COAST_SPEED);
  motorRight.write(NEUTRAL_RIGHT + right * MAX_COAST_SPEED);
}

bool getLineSensor(lineSensorID ID){
  return !(sensorValues[ID] >= LINE_SENSOR_THRESHOLD);
}

bool getSonic(sonicID sonicID){
  return (analogRead(sonicID) <= SONIC_THRESHOLD);
}

bool lineCheck(){
  return getLineSensor(BACKRIGHT) || getLineSensor(BACKLEFT) || getLineSensor(FRONTRIGHT) || getLineSensor(FRONTLEFT);
}
  
class StateBehavior{
  public:
    virtual void execute();
};

class SeekState : public StateBehavior{
  public:
    void execute(){
      differentialDrive(TURN_VALUE, 0);
      if(getSonic(LEFT) || getSonic(RIGHT)){
        activeState = PURSUIT_STATE;
      }
      if(lineCheck()){
        activeState = EDGE_STATE;
      }
    }
};

class PursuitState : public StateBehavior{
  public:
    void execute(){
      bool leftSonic = getSonic(LEFT);
      bool rightSonic = getSonic(RIGHT);
      //digitalWrite(PIN_LED, LOW);
      if (leftSonic && rightSonic){
        activeState = ATTACK_STATE;
        rampup = 0;
      }else{
        if(leftSonic){
          differentialDrive(-TURN_VALUE, TURN_VALUE);
        }else if(rightSonic){
          differentialDrive(TURN_VALUE, -TURN_VALUE);
        }else{
          //differentialDrive(0, 0);
          activeState = SEEK_STATE;
        }
      }
      if(lineCheck()){
        activeState = EDGE_STATE;
      }
    }
};

//Spiritually DestroyState
class AttackState : public StateBehavior{
  public:
    void execute(){
      delay(10);
      if(rampup < 20){
        rampup++;
      }
      if(!(getSonic(LEFT) && getSonic(RIGHT))){
        activeState = PURSUIT_STATE;
      }
      differentialDrive(DRIVE_VALUE_F * .7 * (.05 * rampup), DRIVE_VALUE_F * .7 * (.05 * rampup));
      if(lineCheck()){
        activeState = EDGE_STATE;
      }
    }
};

class OverDriveState : public StateBehavior{
  public:
    void execute(){
      differentialDrive(1.5, 1.5);
      if(getLineSensor(BACKRIGHT) && getLineSensor(BACKLEFT) && getLineSensor(FRONTRIGHT) && getLineSensor(FRONTLEFT)){
        activeState = HALT_STATE;
      }
    }
};

class EdgeState : public StateBehavior{
  public:
    void execute(){
      bool FL = getLineSensor(FRONTLEFT);
      bool BL = getLineSensor(BACKLEFT);
      bool FR = getLineSensor(FRONTRIGHT);
      bool BR = getLineSensor(BACKRIGHT);
      //differentialDrive(0, 0);
      if(FL && BL && FR && BR){
        activeState = HALT_STATE;
      }else if(BL && BR){
        differentialDrive(DRIVE_VALUE_F, DRIVE_VALUE_F);
        delay(500);
      }else if(FL && FR){
        differentialDrive(DRIVE_VALUE_B, DRIVE_VALUE_B);
        delay(500);
      }else if(FL && BL){
        differentialDrive(TURN_VALUE, 0);
      }else if(FR && BR){ 
        differentialDrive(0, TURN_VALUE);
      }else if(FR && BL){
        differentialDrive(0, TURN_VALUE);
      }else if(BR && FL){
        differentialDrive(TURN_VALUE, 0);
      }else if(FR){
        //differentialDrive(-TURN_VALUE, 0);
        differentialDrive(DRIVE_VALUE_B, DRIVE_VALUE_B);
        delay(500);
      }else if(FL){
        //differentialDrive(0, -TURN_VALUE);
        differentialDrive(DRIVE_VALUE_B, DRIVE_VALUE_B);
        delay(500);
      }else if(BR){
        //differentialDrive(0, TURN_VALUE);
        differentialDrive(DRIVE_VALUE_F, DRIVE_VALUE_F);
        delay(500);
      }else if(BL){
        //differentialDrive(TURN_VALUE, 0);
        differentialDrive(DRIVE_VALUE_F, DRIVE_VALUE_F);
        delay(500);
      }else{
        activeState = SEEK_STATE;
      }
    }
};

class HaltState : public StateBehavior{
  public:
    void execute(){
      motorLeft.detach();
      motorRight.detach();
      activeState = 6;
    }
};

class DeadState : public StateBehavior{
  public:
    void execute(){
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(500);
    }
}; 






StateBehavior *states[7];

void setup() {
  // pin mode init
  pinMode(PIN_MOTOR_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_LINE_FRONT_LEFT, INPUT);
  pinMode(PIN_LINE_FRONT_RIGHT, INPUT);
  pinMode(PIN_LINE_BACK_LEFT, INPUT);
  pinMode(PIN_LINE_BACK_RIGHT, INPUT);

  pinMode(PIN_SONIC_LEFT, INPUT);
  pinMode(PIN_SONIC_RIGHT, INPUT);


  qtr.setTypeRC();
    
  qtr.setSensorPins((const uint8_t[]){PIN_LINE_FRONT_LEFT, PIN_LINE_FRONT_RIGHT, PIN_LINE_BACK_LEFT, PIN_LINE_BACK_RIGHT}, 4);

  // pwm driver init
  motorLeft.attach(PIN_MOTOR_LEFT);
  motorRight.attach(PIN_MOTOR_RIGHT);

  // statemachine init
  states[0] = new SeekState();
  states[1] = new PursuitState();
  states[2] = new AttackState();
  states[3] = new OverDriveState();
  states[4] = new EdgeState();
  states[5] = new HaltState();
  states[6] = new DeadState();

  Serial.begin(9600);
  
  // pregame delay
  if(!TESTING_MODE){
    delay(3000);
  }
  delay(2000);
}

void loop() {
  qtr.read(sensorValues);

  //differentialDrive(0.5, 0.5);

  states[activeState]->execute();

  //Serial.println(activeState);
  
  
  
}
