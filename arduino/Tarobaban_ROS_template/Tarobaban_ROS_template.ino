
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);           // select the pins used on the LCD panel


#include <TimerOne.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <tarobaban/set_angle.h>
#include <tarobaban/angle.h>


tarobaban::set_angle set_angle;
tarobaban::angle angle;

ros::Publisher pub_ang("angle", &angle);
ros::NodeHandle nh;


void set_angle(const tarobaban::set_angle& set_angle){
    lcd.setCursor(0,0);
    lcd.print(set_ang_msg.bs_to);   //base angle
    
    lcd.setCursor(0,8);
    lcd.print(set_ang_msg.la_to);   //lower angle
    
    lcd.setCursor(1,0);
    lcd.print(set_ang.msg.ua_to);
}


ros::Subscriber<tarobaban::angle> sub("set_angle", set_angle);


void setup(){

  lcd.begin(16, 2);               // start the library
  lcd.setCursor(0,0);             // set the LCD cursor   position 

  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  
  
  nh.initNode();
  nh.advertise(pub_ang);
  nh.subscribe(sub);
  
}


void loop() {
  
   angle.ba = 3;
   angle.la = 3;
   angle.ua = 3;
   
   nh.spinOnce();
   delay(1);
}
