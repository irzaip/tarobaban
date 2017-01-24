
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);           // select the pins used on the LCD panel

float baseAngle = 0.0;
float lastBaseAngle = 0.0;
int angleCounter = 0;
float upperArmAngle = 0.0;
float lastLowerArmAngle = 0.0;
float lastUpperArmAngle = 0.0;
float lowerArmAngle =0.0;


void setup() {
  //Connect to the serial port. The input argument is the baud rate. IMPORTNAT: Any software communicating to the arduino must use the same baud rate!

Serial.begin(115200);
lcd.begin(16, 2);               // start the library
lcd.setCursor(0,0);             // set the LCD cursor   position 
}  


//any code that needs to run constantly goes here. this function just keeps getting called (not sure how fast),
void loop() {
  // put your main code here, to run repeatedly:

  //serial.available returns number of bytes available to read from serial port. If no data has been sent, that number will be 0.
  //Therefore, the while function (with nothing in it), keeps getting executed until the arduino serial port receives some data.
  while(!Serial.available()){
     
  }
  //once the serial port has received some data, the previous while loop exits and this code executes
  //I'm expecting only floats for my purposes, so I simply read the float.
  //Look here for documentation the Serial.readBytes function: https://www.arduino.cc/en/Serial/ReadBytes
  //the first paramter is where the incoming bytes get stored (the float variable f) and the second parameter is how many bytes to read from the serial.
  //It makes sense that the arduino should read the number of bytes contained in a float varaible (determined by the sizeof function)
  //see also the following two stackexchange posts for how to send and receive floats from python to arduino:
  // http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
  //https://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
 

  float incoming_value;
  unsigned char buffer[4];

  // If we read enough bytes, unpacked it
  if (Serial.readBytes(buffer, sizeof(float)) == sizeof(float)){
    memcpy(&incoming_value, buffer, sizeof(float));
  }
  else{
  // I/O error - no data, not enough bytes, etc.
    incoming_value = 5.0;//I just arbitrarily specified this to be 5.0 so that I know when I'm testing this if I see all 5.0s while reading the serial from my python code, something went wrong here.
  }
  
  Serial.println(incoming_value);
  //keep track of how many angles I have read.
  angleCounter +=1;

  //If I have read 3 angles (angleCounter = 3), move the dobot arm accordingly!!!
  //if less than 3 have been read, populate the angle variables accordingly
  //Angles are in the following order [base angle, upper arm angle, lower arm angle]
  if (angleCounter == 1){
    lastBaseAngle = baseAngle;
    baseAngle = incoming_value;
  }
  else if(angleCounter == 2){
    lastUpperArmAngle = upperArmAngle;
    upperArmAngle = incoming_value;
  }
  else if(angleCounter == 3){
    lastLowerArmAngle = lowerArmAngle;
    lowerArmAngle = incoming_value;
    //reset the angle counter
    angleCounter = 0;

    moveArmToAngles(baseAngle, upperArmAngle, lowerArmAngle);
  }
  
  

/* Old reading a float from serial code. The code I ended up using above looks like it works better.
  
  Serial.readBytes((char*)&f, sizeof(f));
  //print the float value received to the serial monitor. Once running this program in the arduino ide, open the Tools -> Serial Monitor menu item to see output from this function.
  //Ensure that the baud rate is set to match the baud rate you used for the serial!!!!! See bottom right of the serial monitor to change this. My baud rate is 115200, assuming this example hasn't been altered.
  Serial.println(f);
*/
  

}


void moveArmToAngles(float baseAngle, float upperArmAngle, float lowerArmAngle){

  //Serial.println("Base Angle");
  //Serial.println(baseAngle);
  lcd.setCursor(0,0);
  lcd.print(baseAngle);
  
  //need this because of the abs value function, which is needed for proper rounding
  if (baseAngle < 1){
  }

  //Serial.println("Upper Arm Angle");
  //Serial.println(upperArmAngle);
  lcd.setCursor(7,0);
  lcd.print(upperArmAngle);
 
  //need this because of the abs value function, which is needed for proper rounding
  if (upperArmAngle < 1){
  }


  //Serial.println("Lower Arm Angle");
  //Serial.println(lowerArmAngle);
  lcd.setCursor(7,1);
  lcd.print(lowerArmAngle);
  //need this because of the abs value function, which is needed for proper rounding
  if (lowerArmAngle < 1){
  }

//necessary to reverse the direction in which the steppers move, so anngles match my defined angles

  
}


