// TODO:
// 1. samemode synchronize (all of the stips)
// 2. stop subscribe SAME message all the time
// 3. test durability 
// 2019.06.14 edit by Vincy
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#include <SPI.h>
  
#define PIN_FRONT_LEFT 31
#define PIN_FRONT_RIGHT 33
#define PIN_TOP_LEFT 35
#define PIN_TOP_RIGHT 37
#define PIN_LIDAR 39
#define PIN_NONE 41
#define PIN_BOTTOM_LEFT 43
#define PIN_BOTTOM_RIGHT 45
#define PIN_BACK_LEFT 32
#define PIN_BACK_RIGHT 34
#define PIN_EYE_LEFT 36
#define PIN_EYE_RIGHT 38

#define PIXEL_FRONT_LEFT 92
#define PIXEL_FRONT_RIGHT 91
#define PIXEL_TOP_LEFT 100
#define PIXEL_TOP_RIGHT 100
#define PIXEL_LIDAR 50
#define PIXEL_NONE 10
#define PIXEL_BOTTOM_LEFT 100
#define PIXEL_BOTTOM_RIGHT 100
#define PIXEL_BACK_LEFT 79
#define PIXEL_BACK_RIGHT 79
#define PIXEL_EYE_LEFT 20
#define PIXEL_EYE_RIGHT 20

#define LED_RATE_ 20   // period in ms
#define SHOW_COLOR_RATE_ 20   // period in ms
#define UPDATE_RGB_RATE_ 15

// ROS setup
ros::NodeHandle nh;

// Functions
void frontCb( const std_msgs::ColorRGBA& rec_msg);
void topCb( const std_msgs::ColorRGBA& rec_msg);
void lidarCb( const std_msgs::ColorRGBA& rec_msg);
void bottomCb( const std_msgs::ColorRGBA& rec_msg);
void strip_5thCb( const std_msgs::ColorRGBA& rec_msg);
void strip_6thCb( const std_msgs::ColorRGBA& rec_msg);
void backCb( const std_msgs::ColorRGBA& rec_msg);
void eyeCb( const std_msgs::ColorRGBA& rec_msg);

void initial_neopixel(void);
void update_rgb(void* context);
void show_color(void* context);
void fillAll(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c);
void colorWipe(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, long &count);
void colorBlink(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, long &count, bool &flag);
void breathAll(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, std_msgs::ColorRGBA &rgba, long &count, bool &flag);
void snakeScroll(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, long &count, bool &flag);
void segmentLight(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, uint16_t start_pixel, uint16_t end_pixel, bool &clear_flag);
void rainbow(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, std_msgs::ColorRGBA &rgba, long &firstPixelHue);
void monoChase(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, std_msgs::ColorRGBA &rgba, long &count, bool &flag);
void leftTurn(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c);
void rightTurn(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c);

// Parameter
bool blink_flag_front = true;
bool blink_flag_top = true;
bool blink_flag_lidar = true;
bool blink_flag_bottom = true;
bool blink_flag_back = true;
bool blink_flag_eye = true;

bool snake_flag_front = true;
bool snake_flag_top = true;
bool snake_flag_lidar = true;
bool snake_flag_bottom = true;
bool snake_flag_back = true;
bool snake_flag_eye = true;

bool breath_flag_front = true;
bool breath_flag_top = true;
bool breath_flag_lidar = true;
bool breath_flag_bottom = true;
bool breath_flag_back = true;
bool breath_flag_eye = true;

long count_front = 0;
long count_top = 0;
long count_lidar = 0;
long count_bottom = 0;
long count_back = 0;
long count_eye = 0;

long pixelhue_front = 0;
long pixelhue_top = 0;
long pixelhue_lidar = 0;
long pixelhue_bottom = 0;
long pixelhue_back = 0;
long pixelhue_eye = 0;


// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip_FRONT_LEFT = Adafruit_NeoPixel(PIXEL_FRONT_LEFT, PIN_FRONT_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_FRONT_RIGHT = Adafruit_NeoPixel(PIXEL_FRONT_RIGHT, PIN_FRONT_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_TOP_LEFT = Adafruit_NeoPixel(PIXEL_TOP_LEFT, PIN_TOP_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_TOP_RIGHT = Adafruit_NeoPixel(PIXEL_TOP_RIGHT, PIN_TOP_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_LIDAR = Adafruit_NeoPixel(PIXEL_LIDAR, PIN_LIDAR, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_NONE = Adafruit_NeoPixel(PIXEL_NONE, PIN_NONE, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_BOTTOM_LEFT = Adafruit_NeoPixel(PIXEL_BOTTOM_LEFT, PIN_BOTTOM_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_BOTTOM_RIGHT = Adafruit_NeoPixel(PIXEL_BOTTOM_RIGHT, PIN_BOTTOM_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_BACK_LEFT = Adafruit_NeoPixel(PIXEL_BACK_LEFT, PIN_BACK_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_BACK_RIGHT = Adafruit_NeoPixel(PIXEL_BACK_RIGHT, PIN_BACK_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_EYE_LEFT = Adafruit_NeoPixel(PIXEL_EYE_LEFT, PIN_EYE_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_EYE_RIGHT = Adafruit_NeoPixel(PIXEL_EYE_RIGHT, PIN_EYE_RIGHT, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

// Timer
Timer t_update_rgb;
Timer t_show_color;


// ROS messages
std_msgs::ColorRGBA front_rgba_, top_rgba_;
std_msgs::ColorRGBA lidar_rgba_, bottom_rgba_;
std_msgs::ColorRGBA back_rgba_, eye_rgba_;
std_msgs::String str_msg;
std_msgs::Float64 nucleo_msg;

// ROS setup
//ros::NodeHandle nh;
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_front("strip_front", &frontCb);
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_top("strip_top", &topCb);
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_lidar("strip_lidar", &lidarCb);
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_bottom("strip_bottom", &bottomCb);
ros::Publisher pub_nucleo("nucleo_data", &nucleo_msg);
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_back("strip_back", &backCb);
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_eye("strip_eye", &eyeCb);

//ros::Publisher chatter("chatter", &str_msg);
//char hello[] = "hello world!";
//char looper[] = "looper!";
//char cstr[] = "";

/// For Prescaler == 64
/// Base rate = 16MHz
/// 1 秒 / (16 000 000 / 64)= 4*10^(-6)= 0.000004 sec / per cycle
/// For 25Hz = 0.04 sec period
/// 0.04 sec / 0.000004 sec = 10000 (actually should be 10000-1 cuz count from 0)
/// For 50Hz = 0.02 sec period
/// 0.02 sec / 0.000004 sec = 5000 (actually should be 5000-1 cuz count from 0)

volatile int ggyy = 1; 
const int myTOP = 10000;
ISR(TIMER1_COMPA_vect)
{
  strip_FRONT_LEFT.show(); 
  strip_FRONT_RIGHT.show(); 
  strip_TOP_LEFT.show(); 
  strip_TOP_RIGHT.show(); 
  strip_LIDAR.show(); 
  strip_NONE.show(); 
  strip_BOTTOM_LEFT.show(); 
  strip_BOTTOM_RIGHT.show();   
  pub_nucleo.publish(&nucleo_msg);  
  ggyy = 1 - ggyy; //  給下次進入  ISR 用
}



void setup() {
  Serial.begin(57600);
  delay(100);
  
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  //Initialize all the led on the breath mode.
  initial_neopixel(); 

  // LED mode choice and color setting update
  t_update_rgb.every(UPDATE_RGB_RATE_, update_rgb, 0);
  t_show_color.every(SHOW_COLOR_RATE_, show_color, 0);

  // * Init the ROS
  nh.initNode();
  nh.subscribe(sub_strip_front);  
  nh.subscribe(sub_strip_top);  
  nh.subscribe(sub_strip_lidar);  
  nh.subscribe(sub_strip_bottom); 
  nh.subscribe(sub_strip_back);  
  nh.subscribe(sub_strip_eye);  
  nh.advertise(pub_nucleo);    
  delay(100); 
  nh.initNode();
  nh.subscribe(sub_strip_front);  
  nh.subscribe(sub_strip_top);  
  nh.subscribe(sub_strip_lidar);  
  nh.subscribe(sub_strip_bottom); 
  nh.subscribe(sub_strip_back);  
  nh.subscribe(sub_strip_eye);      
  nh.advertise(pub_nucleo);    
  delay(100);   
  nucleo_msg.data = 64;  
//  nh.advertise(chatter); 

//  str_msg.data = hello;
//  chatter.publish( &str_msg );

  cli();  // 禁止中斷
  TCCR1A = 0;
  TCCR1B = 0; 
  TCCR1B |= (1<<WGM12);  // CTC mode; Clear Timer on Compare

  TCCR1B |= (1<<CS10) | (1<<CS11);  // Prescaler == 64

  OCR1A = myTOP;  // TOP count for CTC, 與 prescaler 有關
  TCNT1=0;  // counter 歸零 
  TIMSK1 |= (1 << OCIE1A);  // enable CTC for TIMER1_COMPA_vect
  sei();  // 允許中斷  
  
}

void loop() {
  //initial_neopixel(); 
  t_update_rgb.update();
  //t_show_color.update();
  nh.spinOnce();
}

void frontCb( const std_msgs::ColorRGBA& rec_msg){
    // once any ros parameters changed, reset the "count_front "/ "count_top"
    if(rec_msg.a != front_rgba_.a || rec_msg.r != front_rgba_.r || rec_msg.g != front_rgba_.g || rec_msg.b != front_rgba_.b){
        count_front = 0;
    }
    front_rgba_ = rec_msg;
//    
//    str_msg.data = "front_cb"; 
//    chatter.publish( &str_msg );
}

void topCb( const std_msgs::ColorRGBA& rec_msg){
    if(rec_msg.a != top_rgba_.a || rec_msg.r != top_rgba_.r || rec_msg.g != top_rgba_.g || rec_msg.b != top_rgba_.b){
        count_top = 0;
    }  
    top_rgba_ = rec_msg;
//    
//    str_msg.data = "top_cb"; 
//    chatter.publish( &str_msg );
}

void lidarCb( const std_msgs::ColorRGBA& rec_msg){
    // once any ros parameters changed, reset the "count_lidar "/ "count_bottom"
    if(rec_msg.a != lidar_rgba_.a || rec_msg.r != lidar_rgba_.r || rec_msg.g != lidar_rgba_.g || rec_msg.b != lidar_rgba_.b){
        count_lidar = 0;
    }
    lidar_rgba_ = rec_msg;
    
//    str_msg.data = "lidar_cb"; 
//    chatter.publish( &str_msg );
}

void bottomCb( const std_msgs::ColorRGBA& rec_msg){
    if(rec_msg.a != bottom_rgba_.a || rec_msg.r != bottom_rgba_.r || rec_msg.g != bottom_rgba_.g || rec_msg.b != bottom_rgba_.b){
        count_bottom = 0;
    }  
    bottom_rgba_ = rec_msg;
    
//    str_msg.data = "bottom_cb"; 
//    chatter.publish( &str_msg );
}

void backCb( const std_msgs::ColorRGBA& rec_msg){
    // once any ros parameters changed, reset the "count_back "/ "count_eye"
    if(rec_msg.a != back_rgba_.a || rec_msg.r != back_rgba_.r || rec_msg.g != back_rgba_.g || rec_msg.b != back_rgba_.b){
        count_back = 0;
    }
    back_rgba_ = rec_msg;
    
//    str_msg.data = "back_cb"; 
//    chatter.publish( &str_msg );
}

void eyeCb( const std_msgs::ColorRGBA& rec_msg){
    if(rec_msg.a != eye_rgba_.a || rec_msg.r != eye_rgba_.r || rec_msg.g != eye_rgba_.g || rec_msg.b != eye_rgba_.b){
        count_eye = 0;
    }  
    eye_rgba_ = rec_msg;
    
//    str_msg.data = "eye_cb"; 
//    chatter.publish( &str_msg );
}


void initial_neopixel(void){
  front_rgba_.a = top_rgba_.a = 1;
  lidar_rgba_.a = bottom_rgba_.a = 1;

  front_rgba_.r = 0;
  front_rgba_.g = 100;
  front_rgba_.b = 0;
  top_rgba_.r = 10;
  top_rgba_.g = 10;
  top_rgba_.b = 10;
  lidar_rgba_.r = bottom_rgba_.r = 37;
  lidar_rgba_.g = bottom_rgba_.g = 50;
  lidar_rgba_.b = bottom_rgba_.b = 33;  
        
  // Init all the strips
  strip_FRONT_LEFT.begin();
  strip_FRONT_RIGHT.begin();
  strip_TOP_LEFT.begin();
  strip_TOP_RIGHT.begin();
  strip_LIDAR.begin();
  strip_NONE.begin();
  strip_BOTTOM_LEFT.begin();
  strip_BOTTOM_RIGHT.begin();

  fillAll(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b));
  fillAll(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b));
  fillAll(strip_LIDAR, strip_NONE, strip_LIDAR.Color(lidar_rgba_.r, lidar_rgba_.g, lidar_rgba_.b));
  fillAll(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, strip_BOTTOM_LEFT.Color(bottom_rgba_.r, bottom_rgba_.g, bottom_rgba_.b));

  strip_FRONT_LEFT.show(); 
  strip_FRONT_RIGHT.show(); 
  strip_TOP_LEFT.show(); 
  strip_TOP_RIGHT.show(); 
  strip_LIDAR.show(); 
  strip_NONE.show(); 
  strip_BOTTOM_LEFT.show(); 
  strip_BOTTOM_RIGHT.show(); 
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_rgb(void* context){
  
  // Depending on the mode to set the front strips
  switch((int) front_rgba_.a)
  {       
    case 1: 
        fillAll(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b));
        break;
    case 2: 
        colortWipe(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b), count_front);
        break;
    case 3:
        colorBlink(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b), count_front, blink_flag_front);
        break;   
    case 4:
        snakeScroll(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b), count_front, snake_flag_front);
        break;
    case 5:
        breathAll(strip_FRONT_LEFT, strip_FRONT_RIGHT, front_rgba_, count_front, breath_flag_front);
        break;
    case 6:
        rainbow(strip_FRONT_LEFT, strip_FRONT_RIGHT, front_rgba_, count_front);
        break;
    case 7:
        leftTurn(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b));
        break;
    case 8:
        rightTurn(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b));
        break;        
    case 9:
        monoChase(strip_FRONT_LEFT, strip_FRONT_RIGHT, front_rgba_, count_front, snake_flag_front);
        break;
    default: 
        break;
        
  }
  
  // Depending on the mode to set the top strips
  switch((int) top_rgba_.a)
  {       
    case 1: 
        fillAll(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b));
        break;
    case 2: 
        colortWipe(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b), count_top);
        break;
    case 3:
        colorBlink(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b), count_top, blink_flag_top);
        break;   
    case 4:
        snakeScroll(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b), count_top, snake_flag_top);
        break;
    case 5:
        breathAll(strip_TOP_LEFT, strip_TOP_RIGHT, top_rgba_, count_top, breath_flag_top);
        break;
    case 6:
        rainbow(strip_TOP_LEFT, strip_TOP_RIGHT, top_rgba_, count_top);
        break;
    case 7: 
        leftTurn(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b));
        break;
    case 8: 
        rightTurn(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b));
        break;        
    case 9:
        monoChase(strip_TOP_LEFT, strip_TOP_RIGHT, top_rgba_, count_top, snake_flag_top);
        break;
    default: 
        break;     
  }

  // Depending on the mode to set the lidar strips
  switch((int) lidar_rgba_.a)
  {       
    case 1: 
        fillAll(strip_LIDAR, strip_NONE, strip_LIDAR.Color(lidar_rgba_.r, lidar_rgba_.g, lidar_rgba_.b));
        break;
    case 2: 
        colortWipe(strip_LIDAR, strip_NONE, strip_LIDAR.Color(lidar_rgba_.r, lidar_rgba_.g, lidar_rgba_.b), count_lidar);
        break;
    case 3:
        colorBlink(strip_LIDAR, strip_NONE, strip_LIDAR.Color(lidar_rgba_.r, lidar_rgba_.g, lidar_rgba_.b), count_lidar, blink_flag_lidar);
        break;   
    case 4:
        snakeScroll(strip_LIDAR, strip_NONE, strip_LIDAR.Color(lidar_rgba_.r, lidar_rgba_.g, lidar_rgba_.b), count_lidar, snake_flag_lidar);
        break;
    case 5:
        breathAll(strip_LIDAR, strip_NONE, lidar_rgba_, count_lidar, breath_flag_lidar);
        break;
    case 6:
        rainbow(strip_LIDAR, strip_NONE, lidar_rgba_,count_lidar);
        break;
    case 7: 
        leftTurn(strip_LIDAR, strip_NONE, strip_LIDAR.Color(lidar_rgba_.r, lidar_rgba_.g, lidar_rgba_.b));
        break;
    case 8: 
        rightTurn(strip_LIDAR, strip_NONE, strip_LIDAR.Color(lidar_rgba_.r, lidar_rgba_.g, lidar_rgba_.b));
        break;
    case 9:
        monoChase(strip_LIDAR, strip_NONE, lidar_rgba_, count_lidar, snake_flag_lidar);
        break;
    default: 
        break;
        
  }

  // Depending on the mode to set the bottom strips
  switch((int) bottom_rgba_.a)
  {       
    case 1: 
        fillAll(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, strip_BOTTOM_LEFT.Color(bottom_rgba_.r, bottom_rgba_.g, bottom_rgba_.b));
        break;
    case 2: 
        colortWipe(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, strip_BOTTOM_LEFT.Color(bottom_rgba_.r, bottom_rgba_.g, bottom_rgba_.b), count_bottom);
        break;
    case 3:
        colorBlink(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, strip_BOTTOM_LEFT.Color(bottom_rgba_.r, bottom_rgba_.g, bottom_rgba_.b), count_bottom, blink_flag_bottom);
        break;   
    case 4:
        snakeScroll(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, strip_BOTTOM_LEFT.Color(bottom_rgba_.r, bottom_rgba_.g, bottom_rgba_.b), count_bottom, snake_flag_bottom);
        break;
    case 5:
        breathAll(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, bottom_rgba_, count_bottom, breath_flag_bottom);
        break;
    case 6:
        rainbow(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, bottom_rgba_, count_bottom);
        break;
    case 7: 
        leftTurn(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, strip_BOTTOM_LEFT.Color(bottom_rgba_.r, bottom_rgba_.g, bottom_rgba_.b));
        break;
    case 8: 
        rightTurn(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, strip_BOTTOM_LEFT.Color(bottom_rgba_.r, bottom_rgba_.g, bottom_rgba_.b));
        break;
    case 9:
        monoChase(strip_BOTTOM_LEFT, strip_BOTTOM_RIGHT, bottom_rgba_, count_bottom, snake_flag_bottom);
        break;
    default: 
        break;     
  }

  // Depending on the mode to set the back strips
  switch((int) back_rgba_.a)
  {       
    case 1: 
        fillAll(strip_BACK_LEFT, strip_BACK_RIGHT, strip_BACK_LEFT.Color(back_rgba_.r, back_rgba_.g, back_rgba_.b));
        break;
    case 2: 
        colortWipe(strip_BACK_LEFT, strip_BACK_RIGHT, strip_BACK_LEFT.Color(back_rgba_.r, back_rgba_.g, back_rgba_.b), count_back);
        break;
    case 3:
        colorBlink(strip_BACK_LEFT, strip_BACK_RIGHT, strip_BACK_LEFT.Color(back_rgba_.r, back_rgba_.g, back_rgba_.b), count_back, blink_flag_back);
        break;   
    case 4:
        snakeScroll(strip_BACK_LEFT, strip_BACK_RIGHT, strip_BACK_LEFT.Color(back_rgba_.r, back_rgba_.g, back_rgba_.b), count_back, snake_flag_back);
        break;
    case 5:
        breathAll(strip_BACK_LEFT, strip_BACK_RIGHT, back_rgba_, count_back, breath_flag_back);
        break;
    case 6:
        rainbow(strip_BACK_LEFT, strip_BACK_RIGHT, back_rgba_,count_back);
        break;
    case 7:
        leftTurn(strip_BACK_LEFT, strip_BACK_RIGHT, strip_BACK_LEFT.Color(back_rgba_.r, back_rgba_.g, back_rgba_.b));
        break;
    case 8:
        rightTurn(strip_BACK_LEFT, strip_BACK_RIGHT, strip_BACK_LEFT.Color(back_rgba_.r, back_rgba_.g, back_rgba_.b));
        break;
    case 9:
        monoChase(strip_BACK_LEFT, strip_BACK_RIGHT, back_rgba_, count_back, snake_flag_back);
        break;
    default: 
        break;
        
  }
  
  // Depending on the mode to set the eye strips
  switch((int) eye_rgba_.a)
  {       
    case 1: 
        fillAll(strip_EYE_LEFT, strip_EYE_RIGHT, strip_EYE_LEFT.Color(eye_rgba_.r, eye_rgba_.g, eye_rgba_.b));
        break;
    case 2: 
        colortWipe(strip_EYE_LEFT, strip_EYE_RIGHT, strip_EYE_LEFT.Color(eye_rgba_.r, eye_rgba_.g, eye_rgba_.b), count_eye);
        break;
    case 3:
        colorBlink(strip_EYE_LEFT, strip_EYE_RIGHT, strip_EYE_LEFT.Color(eye_rgba_.r, eye_rgba_.g, eye_rgba_.b), count_eye, blink_flag_eye);
        break;   
    case 4:
        snakeScroll(strip_EYE_LEFT, strip_EYE_RIGHT, strip_EYE_LEFT.Color(eye_rgba_.r, eye_rgba_.g, eye_rgba_.b), count_eye, snake_flag_eye);
        break;
    case 5:
        breathAll(strip_EYE_LEFT, strip_EYE_RIGHT, eye_rgba_, count_eye, breath_flag_eye);
        break;
    case 6:
        rainbow(strip_EYE_LEFT, strip_EYE_RIGHT, eye_rgba_,count_eye);
        break;
    case 7: 
        leftTurn(strip_EYE_LEFT, strip_EYE_RIGHT, strip_EYE_LEFT.Color(eye_rgba_.r, eye_rgba_.g, eye_rgba_.b));
        break;
    case 8: 
        rightTurn(strip_EYE_LEFT, strip_EYE_RIGHT, strip_EYE_LEFT.Color(eye_rgba_.r, eye_rgba_.g, eye_rgba_.b));
        break;  
    case 9:
        monoChase(strip_EYE_LEFT, strip_EYE_RIGHT, eye_rgba_, count_eye, snake_flag_eye);
        break;
    default: 
        break;     
  }

}

// Show all the color setting
void show_color(void* context){
  strip_FRONT_LEFT.show(); 
  strip_FRONT_RIGHT.show(); 
  strip_TOP_LEFT.show(); 
  strip_TOP_RIGHT.show(); 
  strip_LIDAR.show(); 
  strip_NONE.show(); 
  strip_BOTTOM_LEFT.show(); 
  strip_BOTTOM_RIGHT.show();   
  strip_BACK_LEFT.show(); 
  strip_BACK_RIGHT.show(); 
  strip_EYE_LEFT.show(); 
  strip_EYE_RIGHT.show();  
  pub_nucleo.publish(&nucleo_msg);    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set all the dots with a color
void fillAll(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c) {
  for(uint16_t i=0; i<strip_l.numPixels(); i++) {
    strip_l.setPixelColor(i, c);
    strip_r.setPixelColor(i, c);
  }
}

// Set left-side with a color
void leftTurn(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c) {
  fillAll(strip_l, strip_r, 0);
  for(uint16_t i=0; i<strip_l.numPixels(); i++) {
    strip_l.setPixelColor(i, c);
  }
}

// Set right-side with a color
void rightTurn(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c) {
  fillAll(strip_l, strip_r, 0);
  for(uint16_t i=0; i<strip_l.numPixels(); i++) {
    strip_r.setPixelColor(i, c);
  }
}
// Fill the dots one after the other with a color
void colortWipe(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, long &count) {
  if(count >= 0){
    if(count > strip_l.numPixels() || count > strip_r.numPixels()){
      count=0;
    }
    strip_l.setPixelColor(count, c);
    strip_r.setPixelColor(count, c);
    count += 1;
  }
}

// Toggle all the dots with a color
void colorBlink(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, long &count, bool &flag) {
  if (flag){
      

      if (count ==0){
        fillAll(strip_l, strip_r, c);
        count += 1;
      }
      else if (count > 0 && count <= 50){
        count += 1;
      }
      else{
        flag = false;
        count = 0; 
      }
      
//      str_msg.data = "set color"; 
//      chatter.publish( &str_msg );
  }
  else {

      if (count ==0){
        fillAll(strip_l, strip_r, 0);
        count += 1;
      }
      else if (count > 0 && count <= 50){
        count += 1;
      }
      else{
        flag = true;
        count = 0; 
      }
      

//      str_msg.data = "clear color!";
//      chatter.publish( &str_msg );

  }

}

// Scroll a specific number of dots 
void snakeScroll(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, long &count, bool &flag) {
  float dots_number = 6;
  bool clear_flag = true;

  if(flag){
    if((count + dots_number) >= strip_l.numPixels() || (count + dots_number) >= strip_r.numPixels()) {
      count = strip_l.numPixels() - dots_number;
      flag = false;
    } 
    else {
      segmentLight(strip_l, strip_r, c, count, count + dots_number, clear_flag);
      count += 1;
    }
  }
  else {
    if(count < 1) {
      flag = true;
      count = 0;
    } 
    else {
      segmentLight(strip_l, strip_r, c, count, count + dots_number, clear_flag);
      count -= 1;
    }
  }
}

void segmentLight(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, uint32_t c, uint16_t start_pixel, uint16_t end_pixel, bool &clear_flag) {
  if (clear_flag){  fillAll(strip_l, strip_r, 0); }

  for(uint16_t i=start_pixel; i<end_pixel; i++) {
    strip_l.setPixelColor(i, c);
    strip_r.setPixelColor(i, c);
  }  
}


// Breathe all the dots
void breathAll(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, std_msgs::ColorRGBA &rgba, long &count, bool &flag) {
  float frequency = 1;
  //static double count = 0; 
  static uint8_t max_brightness = 200;
  static double increase = (float) 10 / 1000.0 * frequency * max_brightness * 2.0;
  static double r_ratio, g_ratio, b_ratio;
  r_ratio = rgba.r / max_brightness;
  g_ratio = rgba.g / max_brightness;
  b_ratio = rgba.b / max_brightness;

  if(flag){
    if(count >= max_brightness) {
      flag= false;
    } 
    else {
      fillAll(strip_l, strip_r, strip_l.Color(count*r_ratio, count*g_ratio, count*b_ratio));
      count += increase;
    }
  }
  else {
    if(count < 1) {
      flag = true;
    } 
    else {
      fillAll(strip_l, strip_r, strip_l.Color(count*r_ratio, count*g_ratio, count*b_ratio));
      count -= increase;
    }
  }
}


// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, std_msgs::ColorRGBA &rgba, long &firstPixelHue) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:

  if(firstPixelHue >= 0 && firstPixelHue < 65536){
    for(int i=0; i<strip_l.numPixels() || i<strip_r.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip_l.numPixels());
      //strip_l.setPixelColor(i, strip_l.gamma32(strip_l.ColorHSV(pixelHue,200,90)));
      //strip_r.setPixelColor(i, strip_r.gamma32(strip_r.ColorHSV(pixelHue,200,90)));
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
    }

    firstPixelHue += 256;
    if(firstPixelHue >= 65536){
        firstPixelHue = 0;
    }
    
  }
}


void monoChase(Adafruit_NeoPixel &strip_l, Adafruit_NeoPixel &strip_r, std_msgs::ColorRGBA &rgba, long &count, bool &flag){


  float dots_number = 6;
  bool clear_flag = false;
  int colorMultiple = 15;
  std_msgs::ColorRGBA ratio;
  ratio.r = rgba.r / 10;
  ratio.g = rgba.g / 10;
  ratio.b = rgba.b / 10;
  float max_ratio = 255 / (max( max(rgba.r, rgba.g), rgba.r));
  
  

  fillAll(strip_l,strip_r,strip_l.Color(rgba.r, rgba.g, rgba.b));

  if(flag){
    if((count + dots_number) >= strip_l.numPixels() || (count + dots_number) >= strip_r.numPixels()) {
      count = strip_l.numPixels() - dots_number;
      flag = false;
    } 
    else {
      //segmentLight(strip_l, strip_r, strip_l.Color(rgba.r+(ratio.r*colorMultiple), rgba.g+(ratio.g*colorMultiple), rgba.b+(ratio.b*colorMultiple)), count, count + dots_number, clear_flag);
      segmentLight(strip_l, strip_r, strip_l.Color(rgba.r*max_ratio, rgba.g*max_ratio, rgba.b*max_ratio), count, count + dots_number, clear_flag);
      count += 1;
    }
  }
  else {
    if(count < 1) {
      flag = true;
      count = 0;
    } 
    else {
      //segmentLight(strip_l, strip_r, strip_l.Color(rgba.r+(ratio.r*colorMultiple), rgba.g+(ratio.g*colorMultiple), rgba.b+(ratio.b*colorMultiple)), count, count + dots_number, clear_flag);
      segmentLight(strip_l, strip_r, strip_l.Color(rgba.r*max_ratio, rgba.g*max_ratio, rgba.b*max_ratio), count, count + dots_number, clear_flag);
//      ros_popout(String(rgba.r*max_ratio));
//      ros_popout(String(rgba.g*max_ratio));
//      ros_popout(String(rgba.b*max_ratio));
      count -= 1;
    }
  } 
}


//void ros_popout(String str){
//    //String str = String(firstPixelHue);
//    str.toCharArray(cstr,16);
//    str_msg.data = cstr;
//    chatter.publish( &str_msg );
//}
