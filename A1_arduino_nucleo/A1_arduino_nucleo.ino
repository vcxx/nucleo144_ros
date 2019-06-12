#include <Timer.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN_FRONT_LEFT D7
#define PIN_FRONT_RIGHT D8
#define PIN_TOP_LEFT D9
#define PIN_TOP_RIGHT D10

#define PIXEL_FRONT_LEFT 92
#define PIXEL_FRONT_RIGHT 91
#define PIXEL_TOP_LEFT 100
#define PIXEL_TOP_RIGHT 100

#define LED_RATE_ 20   // period in ms
#define SHOW_COLOR_RATE_ 20   // period in ms
#define UPDATE_RGB_RATE_ 15


// Functions
void frontCb( const std_msgs::ColorRGBA& rec_msg);
void topCb( const std_msgs::ColorRGBA& rec_msg);

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
bool snake_flag_front = true;
bool snake_flag_top = true;
bool breath_flag_front = true;
bool breath_flag_top = true;
long count_front = 0;
long count_top = 0;
long pixelhue_front = 0;
long pixelhue_top = 0;


// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip_FRONT_LEFT = Adafruit_NeoPixel(PIXEL_FRONT_LEFT, PIN_FRONT_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_FRONT_RIGHT = Adafruit_NeoPixel(PIXEL_FRONT_RIGHT, PIN_FRONT_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_TOP_LEFT = Adafruit_NeoPixel(PIXEL_TOP_LEFT, PIN_TOP_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_TOP_RIGHT = Adafruit_NeoPixel(PIXEL_TOP_RIGHT, PIN_TOP_RIGHT, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

// Timer
Timer t_update_rgb;
Timer t_show_color;


// ROS messages
std_msgs::ColorRGBA front_rgba_, top_rgba_;
std_msgs::String str_msg;

// ROS setup
ros::NodeHandle nh;
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_front("strip_front", &frontCb);
ros::Subscriber<std_msgs::ColorRGBA> sub_strip_top("strip_top", &topCb);
//ros::Publisher chatter("chatter", &str_msg);
//char hello[] = "hello world!";
//char looper[] = "looper!";
//char cstr[] = "";


void setup() {
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
//  nh.advertise(chatter); 

//  str_msg.data = hello;
//  chatter.publish( &str_msg );
 
  
}

void loop() {
  //initial_neopixel(); 
  t_update_rgb.update();
  t_show_color.update();
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



void initial_neopixel(void){
  front_rgba_.a = top_rgba_.a = 1;
  
  front_rgba_.r = 0;
  front_rgba_.g = 100;
  front_rgba_.b = 0;
  
  top_rgba_.r = 10;
  top_rgba_.g = 10;
  top_rgba_.b = 10;
        
  // Init all the strips
  strip_FRONT_LEFT.begin();
  strip_FRONT_RIGHT.begin();
  strip_TOP_LEFT.begin();
  strip_TOP_RIGHT.begin();


  fillAll(strip_FRONT_LEFT, strip_FRONT_RIGHT, strip_FRONT_LEFT.Color(front_rgba_.r, front_rgba_.g, front_rgba_.b));
  fillAll(strip_TOP_LEFT, strip_TOP_RIGHT, strip_TOP_LEFT.Color(top_rgba_.r, top_rgba_.g, top_rgba_.b));


  strip_FRONT_LEFT.show(); 
  strip_FRONT_RIGHT.show(); 
  strip_TOP_LEFT.show(); 
  strip_TOP_RIGHT.show(); 
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
}

// Show all the color setting
void show_color(void* context){
  strip_FRONT_LEFT.show(); 
  strip_FRONT_RIGHT.show(); 
  strip_TOP_LEFT.show(); 
  strip_TOP_RIGHT.show(); 
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
