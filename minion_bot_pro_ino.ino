/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <sensor_msgs/Joy.h> 
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <math.h>

ros::NodeHandle  nh;

int oldx =0.0;

int sigmoid(int x)
{
     float exp_value;
     float return_value;

     /*** Exponential calculation ***/
     exp_value = exp((double) -x);

     /*** Final sigmoid value ***/
     return_value = 1 / (1 + exp_value);

     return (int)return_value;
}

int wx;
int wy;


     



void callback(const geometry_msgs::Point& pt){
  

wy= sigmoid(pt.y)*255;
//going forward 
if(pt.y>90){
     analogWrite(5,90);analogWrite(6,0);
     //digitalWrite(9,HIGH);
     analogWrite(10,90);analogWrite(9,0);
     //analogWrite(1,wy);
     //digitalWrite(13,HIGH);
     delay(10);
     //digitalWrite(13,HIGH);
}
 else if(0<pt.y<90){
     analogWrite(6,0);analogWrite(5,0);
     //analogWrite(1,0);
     analogWrite(10,0);analogWrite(9,0);
     //analogWrite(9,0);
     //digitalWrite(13,HIGH);
     delay(10);
 }
     
//turning
int k=0;
float delx=pt.x-oldx;
float dt = 1/15;
float a=0.01;
float b=0.0;

 k=(int)(pt.x*0.4);
 k= k+(int) (a*(delx/dt));
 if(pt.x<0){
     
     analogWrite(6,-1*k);
     analogWrite(5,0);
     //analogWrite(1,75);
     analogWrite(10,-1*k);
     analogWrite(9,0);
     //analogWrite(9,75);
     //digitalWrite(13,HIGH);
     delay(10);
 }
 else if(pt.x>0){
     analogWrite(6,0);
     analogWrite(5,k);
     //analogWrite(1,75);
     analogWrite(10,0);
     analogWrite(9,k);
     //analogWrite(9,75);
     //digitalWrite(13,HIGH);
     delay(10);

 }
 
oldx = pt.x;
 
}


//ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
ros::Subscriber<geometry_msgs::Point> sub("dhawan" , &callback);

void setup()
{ 
  pinMode(13, OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
 // pinMode(9,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  //pinMode(1,OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

