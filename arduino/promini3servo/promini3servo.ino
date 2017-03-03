
// Author: Bayu Iriantomo
// Created: 29 Feb 2017
// Arduino ProMini moveArmToAngles Ver 1.0
// HBRIDGE:
// - Motor 1: VNH2SP30
// - Motor 2 & Motor 3: BTS 7960
//

#include <ros.h>
#include <tarobaban/set_angle.h>
#include <tarobaban/angle.h>

tarobaban::angle ros_angle;

ros::Publisher pub_ang("ros_angle", &ros_angle);
ros::NodeHandle nh;

// 
#include <EEPROM.h>
// eeprom interesting article: http://playground.arduino.cc/Code/EEPROMWriteAnything

// Macros to support option testing
#define _CAT(a, ...) a ## __VA_ARGS__
#define SWITCH_ENABLED_      1
#define ENABLED(b) _CAT(SWITCH_ENABLED_, b)
#define DISABLED(b) (!_CAT(SWITCH_ENABLED_, b))

// set as comment this if oled not used
#define USE_OLED_DISPLAY
#define SHOWMOREINFO
// Set as comment this in real env
#define USE_SERIAL_MONITOR

#include <avr/eeprom.h>
#include "Delay.h"

#if ENABLED(USE_OLED_DISPLAY)
#include <U8glib.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);  // Fast I2C / TWI 
#endif

#define MIN_ANGLE    0.0
#define MAX_ANGLE    180.0
#define MIN_VALUE    0
#define MAX_VALUE    1023

#define P_FRACTION 0.3         //0.0 - 10.0 (0.3)
#define I_FRACTION 0.3         //0.0 - 10.0 (0.3)
#define D_FRACTION 4.0         //0.0 - 10.0 (4.0)
#define V_WINDOW 25            //10 - 1000 (25)
#define MIN_DUTYCYCLE 25       //0 - 255 (25)
#define MAX_DUTYCYCLE 200      //0 - 255 (255)
#define SOFT_START 0.3         //0.00 - 1.00 (0.30) 1.00 = OFF
#define D_FRACTION_DEMO 0      //1 - consider only D_FRACTION for servo movement (0 - OFF)
#define EMERGENCY_SHUTDOWN 4   //0 - 100 (4), 0 - OFF, Stops motor if blocked
#define SHOOT_THROUGH_PAUSE 10 //Prevent H bridge from shoot through whenever the direction pin is changed

enum rotation {
  STOP = 0,
  BRAKE = 1,
  FORWARD = 2,
  REVERSE = 3
};

typedef struct {
  float base;
  float lower;
  float upper;
  byte counter;
} angles_t;

angles_t angle, last_angle;

typedef struct {
  int sensor;
  int ra;
  int lb;
  int pwm;
} servo_pin_t;

typedef struct {
  const servo_pin_t pins;
  float angle;
  int ADC_SetPoint;
  int ADC_SetPointOld;
  int ADC_ServoPoti;
  int ADC_ServoPotiOld;
  int dutyCycle; // 10 - 255
  int ADCdiff;
  int timeDiff;
  byte dir; // for debug purpose
} servo_t;

servo_t servos[] = {
  { { A2, 7, 8, 9 }, 0.0, 0, 0, 0, 0, 50, 0, 0, 0 },   // base arm
  { { A1, 13, 11, 0 }, 0.0, 0, 0, 0, 0, 50, 0, 0, 0 }, // lower arm
  { { A0, 12, 10, 0 }, 0.0, 0, 0, 0, 0, 50, 0, 0, 0 }  // upper arm
};


void set_angles(const tarobaban::set_angle& set_angle){
    //lcd.setCursor(0,0);
    //lcd.print(set_angle.ba_to);   //base angle
   
    //lcd.setCursor(0,8);
    //lcd.print(set_angle.la_to);   //lower angle
    
    //lcd.setCursor(1,0);
    //lcd.print(set_angle.up_to);

  if(set_angle.ba_to != servos[0].ADC_SetPoint) {
     servos[0].ADC_SetPointOld = servos[0].ADC_SetPoint;
     servos[0].ADC_SetPoint = set_angle.ba_to;
     } 
  
  if(set_angle.la_to != servos[1].ADC_SetPoint) {
     servos[1].ADC_SetPointOld = servos[1].ADC_SetPoint;
     servos[1].ADC_SetPoint = set_angle.la_to;
     } 
  
  if(set_angle.up_to != servos[2].ADC_SetPoint) {
     servos[2].ADC_SetPointOld = servos[2].ADC_SetPoint;
     servos[2].ADC_SetPoint = set_angle.up_to; 
     }

}

ros::Subscriber<tarobaban::set_angle> sub("angle", set_angles);

void ros_status(void){

  ros_angle.ba = servos[0].ADC_ServoPoti;
  ros_angle.la = servos[1].ADC_ServoPoti;
  ros_angle.ua = servos[2].ADC_ServoPoti;

  pub_ang.publish(&ros_angle);
}

void motorSpin(const servo_pin_t &motor, rotation direction, byte dutycycle);

#if ENABLED(USE_SERIAL_MONITOR)

// format data to be parse = <baseAngle, upperArmAngle, lowerArmAngle>
// for example: <12.5, 30.4, 27.7>

const byte numChars = 28; // 32
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
boolean newData = false;

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while(Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if(recvInProgress == true) {
      if(rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if(ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if(rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  char *strtokIndx; // strtok index
  strtokIndx = strtok(tempChars, ",");
  last_angle.base = angle.base;
  angle.base = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  last_angle.lower = angle.lower;
  angle.lower = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  last_angle.upper = angle.upper;
  angle.upper = atof(strtokIndx);
  angle.counter = 3;
}

void showParsedData() {
  Serial.print("baseAngle ");
  Serial.println(angle.base);
  Serial.print("lowerArmAngle ");
  Serial.println(angle.lower);
  Serial.print("upperArmAngle ");
  Serial.println(angle.upper);
}

#endif

void setup() {
  Serial.begin(115200);
  #if ENABLED(USE_OLED_DISPLAY)
  u8g.setColorIndex(1);
  u8g.setRot180();
  #endif

  eeprom_read_block((void*)&angle, (void*)0, sizeof(angle));

  for(int i=0; i<3; i++) {
    pinMode(servos[i].pins.ra, OUTPUT);
    pinMode(servos[i].pins.lb, OUTPUT);
    if(servos[i].pins.pwm) pinMode(servos[i].pins.pwm, OUTPUT);
    pinMode(servos[i].pins.sensor, INPUT);
  }

  nh.initNode();
  nh.advertise(pub_ang);
  nh.subscribe(sub);
}

#if ENABLED(USE_OLED_DISPLAY)
#define LINE_ROLL 26
#ifndef SHOWMOREINFO
void showInfo(servo_t *servos) {
  char *name[] = {"ba", "lo", "up"};

  u8g.setFont(u8g_font_8x13B);
  u8g.drawStr(0, 10, "ARM Angles");
  
  u8g.setFont(u8g_font_profont11);
  u8g.drawStr(0, 21, "    angle  poin  poti");
  for(int i=0; i<3; i++) {
    char ch[50], sa[7];
    servo_t &s = servos[i];
    strcpy(sa, "");
    dtostrf(s.angle, 2, 2, &sa[strlen(sa)]);
    sprintf(ch, "%s %6s %5d %5d", name[i], sa, s.ADC_SetPoint, s.ADC_ServoPoti);
    u8g.drawStr(0, (10*(i+1))+21, ch);
  }

  static NonBlockDelay dcnt;
  static bool maju = true;
  static byte cnt = 0;
  if(dcnt.Timeout()) {
    if(maju) {
      cnt++;
      if(cnt == LINE_ROLL) maju = false;
    }
    else {
      cnt--;
      if(cnt == 0) maju = true;
    }
    dcnt.Delay(100);
  }
  u8g.drawStr(cnt, 63, "T-A-R-O-B-A-B-A-N");
}
#else
void showMoreInfo(servo_t *servos) {
  u8g.setFont(u8g_font_5x8);
  char *item[] = { "ARM", "ANGL", "AMAP", "POTI", "SPIN", "DUTY" };
  char *spin[] = { "STP", "BRK", "FWD", "REV" };
  for(int i=0; i<6; i++) {
    char ch[50];
    switch(i) {
      case 1: {
          char sa[7], sb[7], sc[7];
          sa[0] = '\0'; sb[0] = '\0'; sc[0] = '\0';
          dtostrf(servos[0].angle, 2, 2, &sa[strlen(sa)]);
          dtostrf(servos[1].angle, 2, 2, &sb[strlen(sb)]);
          dtostrf(servos[2].angle, 2, 2, &sc[strlen(sc)]);
          sprintf(ch, "%4s %6s %6s %6s", item[i], sa, sb, sc);
        }
        break;
      case 2:
        sprintf(ch, "%4s %6d %6d %6d", item[i], servos[0].ADC_SetPoint, servos[1].ADC_SetPoint, servos[2].ADC_SetPoint);
        break;
      case 3:
        sprintf(ch, "%4s %6d %6d %6d", item[i], servos[0].ADC_ServoPoti, servos[1].ADC_ServoPoti, servos[2].ADC_ServoPoti);
        break;
      case 4:
        sprintf(ch, "%4s %6s %6s %6s", item[i], spin[servos[0].dir], spin[servos[1].dir], spin[servos[2].dir]);
        break;
      case 5:
        sprintf(ch, "%4s %6d %6d %6d", item[i], servos[0].dutyCycle, servos[1].dutyCycle, servos[2].dutyCycle);
        break;
      default:
        sprintf(ch, "%4s %6s %6s %6s", item[i], "BASE", "LOWER", "UPPER");
        break;
    }
    u8g.drawStr(0, 10*(i+1), ch);
  }
}
#endif
void oledDraw(void) {
  static NonBlockDelay doled;
  if(doled.Timeout()) {
    u8g.firstPage();  
    do {
      #ifndef SHOWMOREINFO
      showInfo(servos);
      #else
      showMoreInfo(servos);
      #endif
    } while(u8g.nextPage());
    doled.Delay(50);
  }
}
#endif

void loop() {
  #if ENABLED(USE_SERIAL_MONITOR)

  recvWithStartEndMarkers();
  if(newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    showParsedData();
    newData = false;
  }

  #else

  while(!Serial.available()) {}
  float incoming_value;
  unsigned char buffer[4];
  if(Serial.readBytes(buffer, sizeof(float)) == sizeof(float)) {
    memcpy(&incoming_value, buffer, sizeof(float));
  }
  else {
    incoming_value = 5.0;
  }
  // Serial.println(incoming_value);

  angle.counter += 1;
  if(angle.counter == 1) {
    last_angle.base = angle.base;
    angle.base = incoming_value;
  }
  else if(angle.counter == 2) {
    last_angle.lower = angle.lower;
    angle.lower = incoming_value;
  }
  else if(angle.counter == 3) {
    last_angle.upper = angle.upper;
    angle.upper = incoming_value;
    angleCounter = 0;
  }

  #endif

  if(angle.counter == 3) {
    servos[0].angle = angle.base;
    servos[1].angle = angle.lower;
    servos[2].angle = angle.upper;
    eeprom_write_block((const void*)&angle, (void*)0, sizeof(angle));
    angle.counter = 0;
  }

  // moveArmToAngles
  static NonBlockDelay nbd[3];
  for(int i=0; i<3; i++) {
    if(nbd[i].Timeout()) {
      runServo(i);
      nbd[i].Delay(15);
    }
  }

  #if ENABLED(USE_OLED_DISPLAY)
  oledDraw();
  #endif

  ros_status();
  nh.spinOnce();
}

void motorSpin(const servo_pin_t &motor, rotation direction, byte dutycycle) {
  if(motor.pwm) {
    analogWrite(motor.pwm, 0);
    delayMicroseconds(SHOOT_THROUGH_PAUSE);
    switch(direction) {
      case STOP:
        digitalWrite(motor.ra, 0);
        digitalWrite(motor.lb, 0);
        break;
      case BRAKE:
        digitalWrite(motor.ra, 1);
        digitalWrite(motor.lb, 1);
        break;
      case FORWARD:
        digitalWrite(motor.ra, 1); // INA
        digitalWrite(motor.lb, 0); // INB
        break;
      case REVERSE:
        digitalWrite(motor.ra, 0); // INA
        digitalWrite(motor.lb, 1); // INB
        break;
    }
    delayMicroseconds(SHOOT_THROUGH_PAUSE);
    analogWrite(motor.pwm, dutycycle); // PWM
  }
  else {
    analogWrite(motor.lb, 0); // L
    delayMicroseconds(SHOOT_THROUGH_PAUSE);
    switch(direction) {
      case STOP:
      case BRAKE:
        digitalWrite(motor.ra, 0); // R
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        break;
      case FORWARD:
        digitalWrite(motor.ra, 0); // R
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(motor.lb, dutycycle); // L
        break;
      case REVERSE:
        digitalWrite(motor.ra, 1); // R
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(motor.lb, 255 - dutycycle); // L
        break;
    }
  }
}

void runServo(int ServoId) {
  /* int setPoint = map(servos[ServoId].angle, MIN_ANGLE, MAX_ANGLE, MIN_VALUE, MAX_VALUE);
  if(setPoint != servos[ServoId].ADC_SetPoint) {
    servos[ServoId].ADC_SetPointOld = servos[ServoId].ADC_SetPoint;
    servos[ServoId].ADC_SetPoint = setPoint;
  } */
  servos[ServoId].ADC_ServoPotiOld = servos[ServoId].ADC_ServoPoti;
  servos[ServoId].ADC_ServoPoti = analogRead(servos[ServoId].pins.sensor);
  servos[ServoId].ADCdiff = servos[ServoId].ADC_SetPoint - servos[ServoId].ADC_ServoPoti;

  servos[ServoId].dutyCycle = abs(servos[ServoId].ADCdiff) * P_FRACTION;
  servos[ServoId].dutyCycle += servos[ServoId].timeDiff * I_FRACTION;
  servos[ServoId].dutyCycle += abs(servos[ServoId].ADC_SetPointOld - servos[ServoId].ADC_SetPoint) * D_FRACTION;

  if(D_FRACTION_DEMO == 1) servos[ServoId].dutyCycle = abs(servos[ServoId].ADC_SetPointOld - servos[ServoId].ADC_SetPoint) * D_FRACTION;

  if(SOFT_START * servos[ServoId].timeDiff < 1) servos[ServoId].dutyCycle = servos[ServoId].dutyCycle * (SOFT_START * servos[ServoId].timeDiff);
  servos[ServoId].timeDiff++;
  if(servos[ServoId].dutyCycle < MIN_DUTYCYCLE && servos[ServoId].dutyCycle > 0) servos[ServoId].dutyCycle = MIN_DUTYCYCLE;
  if(servos[ServoId].dutyCycle > MAX_DUTYCYCLE) servos[ServoId].dutyCycle = MAX_DUTYCYCLE;
  if(servos[ServoId].dutyCycle < 0) servos[ServoId].dutyCycle = 0;

  if(D_FRACTION_DEMO == 1) {
    if(abs(servos[ServoId].ADC_SetPointOld - servos[ServoId].ADC_SetPoint) < 2) {
      motorSpin(servos[ServoId].pins, BRAKE, 0);
      servos[ServoId].dir = BRAKE;
    }
    else {
      if(servos[ServoId].ADC_SetPointOld - servos[ServoId].ADC_SetPoint < 0) {
        motorSpin(servos[ServoId].pins, FORWARD, servos[ServoId].dutyCycle);
        servos[ServoId].dir = FORWARD;
      }
      if(servos[ServoId].ADC_SetPointOld - servos[ServoId].ADC_SetPoint > 0) {
        motorSpin(servos[ServoId].pins, REVERSE, servos[ServoId].dutyCycle);
        servos[ServoId].dir = REVERSE;
      }
    }
  }
  else {
    if(abs(servos[ServoId].ADCdiff) < V_WINDOW) {
      servos[ServoId].dutyCycle = 0;
      servos[ServoId].timeDiff = 0;
    }

    if(abs(servos[ServoId].ADC_ServoPotiOld - servos[ServoId].ADC_ServoPoti) < EMERGENCY_SHUTDOWN && servos[ServoId].dutyCycle == MAX_DUTYCYCLE && servos[ServoId].timeDiff > 50) {
      motorSpin(servos[ServoId].pins, BRAKE, 0);
      servos[ServoId].dir = BRAKE;
      delayMicroseconds(SHOOT_THROUGH_PAUSE);
      // delay(1000);
      servos[ServoId].timeDiff = 0;
    }
    else {
      if(servos[ServoId].ADCdiff > 0) {
        motorSpin(servos[ServoId].pins, FORWARD, servos[ServoId].dutyCycle);
        servos[ServoId].dir = FORWARD;
      }
      if(servos[ServoId].ADCdiff < 0) {
        motorSpin(servos[ServoId].pins, REVERSE, servos[ServoId].dutyCycle);
        servos[ServoId].dir = REVERSE;
      }
    }
  }

  // waits for the servo to get there
  // delay(15); // replace w/ non blocking delay
}

//
// Reference:
// 1. http://forum.arduino.cc/index.php?topic=396450.0
// 2. Source & info: www.HomoFaciens.de/technics-computer-arduino-uno_en_navion.htm
//    Wiper motor software v 1.3:
//    For the first test run, set MAX_DUTYCYCLE to 75 in order to lower the maximum torque in case something goes wrong.
//    When connecting the servo to the microcontroller for the first time, you have to consider the rotational direction of your wiper motor!
//    Adjust the potentiometer defining the setpoint to the center position.
//    If the servo moves to neutral position, too after connecting the circuit to the supply voltage, the polarity of the servo sensor is correct.
//    If the wiper motor starts spinning away from the center position, the polarity of the servo sensor (+ and - connector) has to be swapped.
//

