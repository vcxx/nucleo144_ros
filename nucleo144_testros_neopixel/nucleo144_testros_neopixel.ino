#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <SPI.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

// ROS setting
ros::NodeHandle  nh;
std_msgs::Float64 nucleo_msg;
ros::Publisher pub_nucleo("nucleo_data", &nucleo_msg);

#define PIN D7
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

int buttonState = 0;         // variable for reading the pushbutton status



void setup() {
  Serial.begin(57600);
  delay(1000);
  // ROS node

  nh.initNode();
  nh.advertise(pub_nucleo);  

  pinMode(PB0, OUTPUT);   // User led1
  pinMode(PB7, OUTPUT);   // User led2
  pinMode(PB14, OUTPUT);  // User led3
  pinMode(PC13, INPUT);   // User button

  pinMode(D7, OUTPUT);  // User led3

  nucleo_msg.data = 64;

  strip.begin();
  strip.show();  
}

// the loop function runs over and over again forever
void loop() {
  colorWipe(strip.Color(255, 0, 0), 50); // Red
  colorWipe(strip.Color(0, 255, 0), 50); // Green
  colorWipe(strip.Color(0, 0, 255), 50); // Blue
  
  buttonState = digitalRead(PC13);
  
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH)
  {
    // turn LED on:
    digitalWrite(PB0, HIGH);  
    digitalWrite(PB7, HIGH);  
    digitalWrite(PB14, HIGH); 
    digitalWrite(D7, HIGH);  
    nucleo_msg.data +=1;
  } 
  else 
  {
    // turn LED off:
    digitalWrite(PB0, LOW);
    digitalWrite(PB7, LOW); 
    digitalWrite(PB14, LOW);
    digitalWrite(D7, LOW);  
  }
   
  pub_nucleo.publish(&nucleo_msg);
  nh.spinOnce();
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
