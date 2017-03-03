
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);           // select the pins used on the LCD panel


#include <ros.h>
#include <tarobaban/set_angle.h>
#include <tarobaban/angle.h>


tarobaban::angle ros_angle;

ros::Publisher pub_ang("ros_angle", &ros_angle);
ros::NodeHandle nh;


void set_angles(const tarobaban::set_angle& set_angle){
    lcd.setCursor(0,0);
    lcd.print(set_angle.ba_to);   //base angle
    
    lcd.setCursor(0,8);
    lcd.print(set_angle.la_to);   //lower angle
    
    lcd.setCursor(1,0);
    lcd.print(set_angle.up_to);
}


ros::Subscriber<tarobaban::set_angle> sub("angle", set_angles);


void setup(){

  lcd.begin(16, 2);               // start the library
  lcd.setCursor(0,0);             // set the LCD cursor   position 

  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  
  nh.initNode();
  nh.advertise(pub_ang);
  nh.subscribe(sub);
  
}


void loop() {
  
   ros_angle.ba = 3;
   ros_angle.la = 3;
   ros_angle.ua = 3;

   
   nh.spinOnce();
   delay(1);
}
