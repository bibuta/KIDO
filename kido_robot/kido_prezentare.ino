#include <SoftwareSerial.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Fonts/FreeSansBold24pt7b.h>
#include <Servo.h>
Servo servo_fata;
Servo servo_cap;
//functie reset
void(* resetFunc) (void) = 0;
//senzori ultrasonici
#define trigPinS2  2
#define echoPinS2  3
#define trigPinS1  5
#define echoPinS1  4
#define trigPinD2  6
#define echoPinD2  7
#define trigPinD1  8
#define echoPinD1  9
#define trigPinF  A5
#define echoPinF  A4
#define trigPinF1 32
#define echoPinF1 33
#define buzzer    47
//lcd
#define TFT_DC    49
#define TFT_CS    98
#define TFT_MOSI  51
#define TFT_CLK   52
#define TFT_RST   48
#define TFT_MISO  99
//motoare
int vit;
#define E1 11
#define M1 13
#define E2 10
#define M2 12
//distante
#define distmed 25
#define distmic  5
#define distf   12
#define distl   20

//delay
#define interval  1000
int previousMillis =  0;

//senzori ultrasonici
long durationS1;
int  distanceS1;
long durationS2;
int  distanceS2;
long durationD1;
int  distanceD1;
long durationD2;
int  distanceD2;
long durationF;
int  distanceF;
long durationF1;
int  distanceF1;

//string-uri
String state;
String caut = "ploua";

//verificari
int ok =  0;
int oka = 0;
int ok1 = 0;
int sr =  0;
int ret = 0;
int i;
int intoarcere, alarm;
int cr, cg, cb;
int buzzerState = LOW;

//imagini
const unsigned char termo [] PROGMEM = {
  0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x07, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x87, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x87, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x7f, 0xf8, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x3f, 0xf0, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x7f, 0xf8, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x3f, 0xf0, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x7f, 0xf8, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x3f, 0xf0, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xff, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x03, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x03, 0xff, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x03, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x07, 0xfe, 0x01, 0xff, 0x80, 0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x7f, 0xc0, 0x00, 0x00,
  0x00, 0x0f, 0xf0, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x3f, 0xc0, 0x00, 0x00,
  0x00, 0x1f, 0xe0, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x1f, 0xe0, 0x00, 0x00,
  0x00, 0x1f, 0xe0, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x0f, 0xe0, 0x00, 0x00,
  0x00, 0x1f, 0xe0, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x1f, 0xe0, 0x00, 0x00,
  0x00, 0x1f, 0xe0, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x1f, 0xe0, 0x00, 0x00,
  0x00, 0x0f, 0xf0, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x7f, 0xc0, 0x00, 0x00,
  0x00, 0x0f, 0xfc, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x07, 0xfe, 0x01, 0xff, 0x80, 0x00, 0x00,
  0x00, 0x03, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x00
};


const unsigned char grade1 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x03, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xf9, 0xff, 0xff, 0xf0, 0x00, 0x00,
  0x00, 0x07, 0xf0, 0x7f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x7f, 0xff, 0xf8, 0x00, 0x00,
  0x00, 0x0f, 0xe0, 0x7f, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x7f, 0xff, 0xfc, 0x00, 0x00,
  0x00, 0x1f, 0xf0, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x1f, 0xf9, 0xf8, 0x1f, 0xff, 0xf0, 0x00,
  0x00, 0x3f, 0xff, 0xe0, 0x07, 0xff, 0xf8, 0x00, 0x00, 0x3f, 0xff, 0xc7, 0xe3, 0xff, 0xfc, 0x00,
  0x00, 0x3f, 0xff, 0x8f, 0xf1, 0xff, 0xfe, 0x00, 0x00, 0x3f, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0x00,
  0x00, 0x3f, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0x00, 0x00, 0x3f, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0x00,
  0x00, 0x3f, 0xff, 0x7f, 0xfe, 0x7f, 0xff, 0x00, 0x00, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0x80,
  0x03, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xc0, 0x0f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xf0,
  0x1f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc,
  0x3f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfe,
  0x7f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xfe,
  0xff, 0xff, 0xfe, 0x7f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0x8f, 0xf1, 0xff, 0xff, 0xfe,
  0x7f, 0xff, 0xff, 0xc7, 0xe3, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xe0, 0x07, 0xff, 0xff, 0xfe,
  0x3f, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0,
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80,
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char somn [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x60, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xfc, 0x00, 
  0x00, 0x00, 0x00, 0x7f, 0xe0, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x1f, 0xf0, 0x00, 
  0x00, 0x00, 0x7f, 0xff, 0xf0, 0x1f, 0xf0, 0x00, 0x00, 0x00, 0xff, 0xff, 0xf0, 0x3f, 0xe0, 0x00, 
  0x00, 0x00, 0xff, 0xff, 0xf0, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xe0, 0xff, 0x80, 0x00, 
  0x00, 0x00, 0x7f, 0x07, 0xe0, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x78, 0x0f, 0xc0, 0xff, 0xff, 0xff, 
  0x00, 0x00, 0x00, 0x0f, 0xc0, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x1f, 0x80, 0xff, 0xff, 0xff, 
  0x00, 0x00, 0x00, 0x1f, 0x80, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x3f, 0x00, 0xff, 0xff, 0xff, 
  0x00, 0x00, 0x00, 0x3f, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x1f, 0x00, 0x00, 0x00, 
  0x00, 0x18, 0x03, 0xf8, 0xff, 0x80, 0x00, 0x00, 0x00, 0x7c, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 
  0x01, 0xfc, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x07, 0xfc, 0x07, 0xff, 0xff, 0x80, 0x00, 0x00, 
  0x1f, 0xfc, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 
  0xff, 0x3c, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x3c, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 
  0x78, 0x3c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x78, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x79, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

SoftwareSerial BT(A9, A8);

void setup() {
  stop_lcd();
  servo_fata.attach(34);
  servo_fata.write(75);
  delay(800);
  servo_fata.detach();
  BT.begin(9600);
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(trigPinS1, OUTPUT);
  pinMode(echoPinS1,  INPUT);
  pinMode(trigPinS2, OUTPUT);
  pinMode(echoPinS2,  INPUT);
  pinMode(trigPinD1, OUTPUT);
  pinMode(echoPinD1,  INPUT);
  pinMode(trigPinD2, OUTPUT);
  pinMode(echoPinD2,  INPUT);
  pinMode(trigPinF,  OUTPUT);
  pinMode(echoPinF,   INPUT);
  pinMode(trigPinF1, OUTPUT);
  pinMode(echoPinF1,  INPUT);
  pinMode(buzzer,    OUTPUT);

}

void loop() {
  while (BT.available()) {
    char c = BT.read();
    state += c;
  }
  meniu();
  if (ok1 == 1)
  { sortare();
  }
  if (ok1 == 2)
  { urmarire();
  }
  if (ok1 == 3)
  { go(0, 0);
  }
  if (ok1 == 4)
  { localizare1();
  }
  if (ok1 == 5)
  {
    localizare2();
  }
  if (alarm == 1)
  {
    alarma();
  }
}

void senzori(int a, int b, int c, int d, int f, int h)//controlul senzorilor ultrasonici in functie de necesitatea actiunii
{
  if (a == 1) //senzor fata sus
  { digitalWrite(trigPinF, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinF, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinF, LOW);
    durationF = pulseIn(echoPinF, HIGH);
    distanceF = durationF * 0.034 / 2;
  }

  if (b == 1) //senzor fata jos
  {
    digitalWrite(trigPinF1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinF1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinF1, LOW);
    durationF1 = pulseIn(echoPinF1, HIGH);
    distanceF1 = durationF1 * 0.034 / 2;
  }
  if (c == 1) //senzor stanga centru
  {
    digitalWrite(trigPinS1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinS1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinS1, LOW);
    durationS1 = pulseIn(echoPinS1, HIGH);
    distanceS1 = durationS1 * 0.034 / 2;
  }
  if (d == 1) //senzor stanga margine
  {
    digitalWrite(trigPinS2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinS2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinS2, LOW);
    durationS2 = pulseIn(echoPinS2, HIGH);
    distanceS2 = durationS2 * 0.034 / 2;
  }
  if (f == 1) //senzor dreapta centru
  {
    digitalWrite(trigPinD1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinD1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinD1, LOW);
    durationD1 = pulseIn(echoPinD1, HIGH);
    distanceD1 = durationD1 * 0.034 / 2;
  }
  if (h == 1) //senzor dreapta margine
  {
    digitalWrite(trigPinD2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinD2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinD2, LOW);
    durationD2 = pulseIn(echoPinD2, HIGH);
    distanceD2 = durationD2 * 0.034 / 2;
  }
}

void go(int vits, int vitd) { //controlul motoarelor
  if (vitd > 0) {
    analogWrite(M1, HIGH);
    analogWrite(E1, vitd);
  }
  else {
    analogWrite(M1, LOW);
    analogWrite(E1, -vitd);
  }

  if (vits > 0) {
    analogWrite(M2, HIGH);
    analogWrite(E2, vits);
  }
  else {
    analogWrite(M2, LOW);
    analogWrite(E2, -vits);
  }
}
void meniu() //controlul tuturor comenzilor din aplicatie
{
  //----------------------Control Telecomanda---------------------
  long currentMillis = millis();
  if ((state == "fta") && (currentMillis - previousMillis >= 300))
  {
    previousMillis = currentMillis;
    go(150, 150);
    state = "";
  }
  else if ((state == "spt") && (currentMillis - previousMillis >= 300))
  {
    previousMillis = currentMillis;
    go(-150, -150);
    state = "";
  }
  else if ((state == "stg" || state == "g") && (currentMillis - previousMillis >= 300))
  {
    previousMillis = currentMillis;
    go(0, 150);
    state = "";

  }
  else if ((state == "drp") && (currentMillis - previousMillis >= 300))
  {
    previousMillis = currentMillis;
    go(150, 0);
    state = "";
  }
  else if ((state == "st") && (currentMillis - previousMillis >= 300))
  {
    previousMillis = currentMillis;
    go(0, 0);
    state = "";
  }
  else if (state == "deschid")
  {
    servo_fata.attach(34);
    servo_fata.write(75);
    delay(800);
    servo_fata.detach();
    state = "";
  }
  else if (state == "inchid")
  {
    servo_fata.attach(34);
    servo_fata.write(120);
    delay(800);
    servo_fata.detach();
    state = "";
  }
  else if (state == "localizare1")
  {
    ok1 = 4;
    state = "";
  }
  else if (state == "localizare2")
  {
    ok1 = 5;
    state = "";
  }
  //----------------------Control sortare culori-----------------
  if (state == "bcub")
  {
    servo_fata.attach(34);
    servo_fata.write(75);
    delay(800);
    servo_fata.detach();
    sr = cb;
    ok = 1;
    ok1 = 1;
    state = "";

  }
  else if (state == "gcub")
  {
    servo_fata.attach(34);
    servo_fata.write(75);
    delay(800);;
    servo_fata.detach();
    sr = cg;
    ok = 1;
    ok1 = 1;
    state = "";

  }
  else if (state == "rcub")
  {

    sr = cr;
    ok = 1;
    servo_fata.attach(34);
    servo_fata.write(75);
    delay(800);
    servo_fata.detach();
    sr = cr;
    ok = 1;
    ok1 = 1;
    state = "";

  }
  else if (state == "stand")
  {
    cr = 1;
    cb = 2;
    cg = 3;
    state = "";
  }
  else if (state == "verificare")
  {
    if (i < 4)
      state = "";
    go(120, 0);
    delay(850);
    go(0, -120);
    delay(900);
    go(0, 0);
    delay(100);
    i++;
  }
  else if (state == "re")
  {
    state = "";
    cr = i;
  }
  else if (state == "gr")
  {
    state = "";
    cg = i;
  }
  else if (state == "bl")
  {
    state = "";
    cb = i;
  }
  //----------------------Control stari ecran--------------------
  if (state == "fericit")
  {
    emotie(1);
    state = "";
  }
  else if (state == "trist")
  {
    emotie(2);
    state = "";
  }
  else if (state == "surprins")
  {
    emotie(3);
    state = "";
  }
  else if (state == "bravo")
  {
    fericit();
    state = "";

  }
  else if (state == "start")
  {
    start_lcd();
    state = "";
  }
  //----------------------Control urmarire obiecte------------------
  else if (state == "urmarire")
  {
    ok1 = 2;
    state = "";
  }
  else if (state == "stop")
  {
    ok1 = 3;
    state = "";
  }
  //----------------------Functie de resetare a robotului---------------
  else if (state == "reset")
  {
    delay(3000);
    resetFunc();
    state = "";
  }
  //----------------------Control alarmei-------------------------------
  else if (state == "alarma_on")
  {
    intoarcere = 1;
    alarm = 1;
    ret = 0;
    state = "";
  }
  else if (state == "alarma_off")
  {
    alarm = 0;
    oka = 0;
    noTone(buzzer);
    
    state = "";
  }
  //----------------------Control afisare vreme--------------------------
  else if (state.indexOf(caut) >= 0)
  {
    tft.fillScreen(0xAEDB);
    tft.setFont(&FreeSansBold24pt7b);
    tft.fillRoundRect(75, 25, 185, 70, 5, 0xF50C);
    tft.fillRoundRect(0, 92, 320, 50, 5, 0xE36A);
    tft.fillRoundRect(92, 31, 10, 38, 2, 0xE1C8);
    tft.fillRoundRect(85, 67, 20, 20, 2, 0xE1C8);
    tft.drawBitmap(70, 27, termo, 64, 64, 0xF7DD);
    tft.drawBitmap(190, 25, grade1, 64, 64, 0xF7DD);
    Text(state, ILI9341_WHITE, 128, 73, 1);
    delay(8000);
    stop_lcd();
    state = "";
  }


}
//----------------------Functie urmarire obiecte------------------------
void urmarire()
{
  senzori(1, 0, 0, 1, 0, 1);
  if (distanceF < 10)
  {
    go(0, 0);
  }
  else if (distanceS2 > distl && distanceD2 > distl)
  {
    if (distanceF > 30)
    {
      go(120, 120);
    }
    else if (distanceF <= 30)
    {
      vit = (distanceF - 6) * 2.5 + 40;
      go(vit, vit);
    }
  }
  else if (distanceS2 < distanceD2)
  {
    go(40, 150);
  }
  else if (distanceS2 > distanceD2)
  {
    go(150, 40);
  }
}
//-------------Functii localizari obiecte(la sortarea pe culori sau simpla)---------
void sortare()
{ senzori(1, 1, 1, 0, 1, 0);
  if (state == "spate")
  {
    distanceF1 = 1;
    distanceF = 1;
    state = "";
  }
  if (ok == 3)
  {
    go(0, 0);

  }
  if (ok == 1)
  {
    if ((distanceD1 < distmic) || (distanceS1 < distmic))
    {
      go(-150, -150);
      delay(700);
      go(0, 0);
      delay(200);
      if (distanceS1 < distanceD1)
      {
        go(0, 90);
        delay(400);
      }
      else if (distanceS1 > distanceD1)
      {
        go(90, 0);
        delay(400);
      }
      go(0, 0);
      delay(200);
    }
    if (((distanceD1 < distmed) && (distanceD1 > distmic))  || ((distanceS1 < distmed) && (distanceS1 > distmic)) )
    { if (distanceS1 < distanceD1)
      {
        go(0, 60);
        
      }
      else if (distanceS1 > distanceD1)
      {
        go(60, 0);
        
      }

    }
    else
    {
      if (distanceF1 > distmic || distanceF1 < distl)
      {
        vit = (distanceF1 - 6) * 2.307 + 40;

      }
      else if (distanceF1 > distl)
      {
        vit = 55;

      }
      go(vit, vit);
    }

    if (distanceF1 <= distmic)
    {
      go(100, 100);
      delay(650);
      go(0, 0);
      delay(500);
      servo_fata.attach(34);
      servo_fata.write(120);
      delay(800);
      servo_fata.detach();

      if (sr == 1)
      {
        go(-100, -100);
        delay(800);
        go(120, 0);
        delay(850);
        go(0, -120);
        delay(900);
        go(0, 0);
        delay(300);
        ok = 2;
      }
      if (sr == 2)
      { go(-100, -100);
        delay(300);
        go(120, 0);
        delay(850);
        go(0, -120);
        delay(900);
        go(0, 0);
        delay(50);
        go(120, 0);
        delay(850);
        go(0, -120);
        delay(900);
        go(0, 0);
        delay(300);
        ok = 2;
      }
      if (sr == 3)
      {
        go(-100, -100);
        delay(800);
        go(0, 120);
        delay(850);
        go(-120, 0);
        delay(900);
        go(0, 0);
        delay(300);
        ok = 2;
      }
    }

  }
  if (ok == 2)
  {
    if ((distanceD1 < distmic) || (distanceS1 < distmic))
    {
      go(-150, -150);
      delay(600);
      go(0, 0);
      delay(200);
      if (distanceS1 < distanceD1)
      {
        go(0, 90);
        delay(400);
      }
      else if (distanceS1 > distanceD1)
      {
        go(90, 0);
        delay(400);
      }
      go(0, 0);
      delay(200);
    }
    if (((distanceD1 < distmed) && (distanceD1 > distmic))  || ((distanceS1 < distmed) && (distanceS1 > distmic)) )
    { if (distanceS1 < distanceD1)
      {
        go(0, 60);
        
      }
      else if (distanceS1 > distanceD1)
      {
        go(60, 0);
        
      }

    }
    else
    {
      if (distanceF > distmic || distanceF < distl)
      {
        vit = (distanceF - 6) * 1.4 + 40;
      }
      else if (distanceF > 20)
      {
        vit = 40  ;
      }
      go(vit, vit);
    }
    if (distanceF <= distf)
    {
      go(90, 90);
      delay(300);
      go(0, 0);
      delay(500);
      servo_fata.attach(34);
      servo_fata.write(75);
      delay(800);
      servo_fata.detach();
      ok = 1;
      if (sr == 1)
      {
        go(-100, -100);
        delay(1000);
        go(0, 120);
        delay(850);
        go(-120, 0);
        delay(900);
        go(0, 0);
        delay(300);
        ok = 3;
      }
      if (sr == 2)
      { go(-100, -100);
        delay(700);
        go(120, 0);
        delay(850);
        go(0, -120);
        delay(900);
        go(0, 0);
        delay(50);
        go(120, 0);
        delay(850);
        go(0, -120);
        delay(900);
        go(0, 0);
        delay(300);
        ok = 3;
      }
      if (sr == 3)
      {
        go(-100, -100);
        delay(1000);
        go(120, 0);
        delay(850);
        go(0, -120);
        delay(900);
        go(0, 0);
        delay(300);
        ok = 3;
      }
      state = "";
      senzori(1, 1, 1, 1, 1, 1);
      if ((distanceD1 < distmed) || (distanceS1 < distmed))
      {
        if (distanceS1 < distanceD1)
        {
          go(0, 90);
          delay(400);
        }
        else if (distanceS1 > distanceD1)
        {
          go(90, 0);
          delay(400);
        }
      }
      else if ((distanceD2 < distmed) || (distanceS2 < distmed))
      {
        if (distanceS2 < distanceD2)
        {
          go(0, 90);
          delay(700);
        }
        else if (distanceS2 > distanceD2)
        {
          go(90, 0);
          delay(700);
        }
      }
    }

  }


}
void localizare1()
{ state = "";
  senzori(1, 1, 1, 0, 1, 0);
  if (state == "spate")
  { state = "";
    distanceF1 = 1;

  }
  if ((distanceD1 < distmic) || (distanceS1 < distmic))
  {
    go(-150, -150);
    delay(700);
    go(0, 0);
    delay(200);
    if (distanceS1 < distanceD1)
    {
      go(0, 90);
      delay(400);
    }
    else if (distanceS1 > distanceD1)
    {
      go(90, 0);
      delay(400);
    }
    go(0, 0);
    delay(200);
  }
  if (((distanceD1 < distmed) && (distanceD1 > distmic))  || ((distanceS1 < distmed) && (distanceS1 > distmic)) )
  { if (distanceS1 < distanceD1)
    {
      go(0, 60);
    }
    else if (distanceS1 > distanceD1)
    {
      go(60, 0);
    }

  }
  else
    go(55, 55);
  if (distanceF1 <= distmic)
  {
    go(100, 100);
    delay(650);
    go(0, 0);
    delay(500);
    servo_fata.attach(34);
    servo_fata.write(120);
    delay(1200);
    servo_fata.detach();
    ok1 = 0;
  }
}
void localizare2()
{ state = "";
  senzori(1, 1, 1, 0, 1, 0);
  if (state == "spate")
  { state = "";
    distanceF = 1;
  }

  if ((distanceD1 < distmic) || (distanceS1 < distmic))
  {
    go(-150, -150);
    delay(600);
    go(0, 0);
    delay(200);
    if (distanceS1 < distanceD1)
    {
      go(0, 90);
      delay(400);
    }
    else if (distanceS1 > distanceD1)
    {
      go(90, 0);
      delay(400);
    }
    go(0, 0);
    delay(200);
  }
  if (((distanceD1 < distmed) && (distanceD1 > distmic))  || ((distanceS1 < distmed) && (distanceS1 > distmic)) )
  { if (distanceS1 < distanceD1)
    {
      go(0, 60);
    }
    else if (distanceS1 > distanceD1)
    {
      go(60, 0);
    }

  }
  else
    go(55, 55);
  if (distanceF <= distf)
  {
    go(90, 90);
    delay(300);
    go(0, 0);
    delay(500);
    servo_fata.attach(34);
    servo_fata.write(75);
    delay(800);
    servo_fata.detach();
    ok1 = 0;
  }
}
//----------------------Functii lcd----------------------------
void emotie(int a)
{ curatare_fata();
  if (a == 1) //fericit
  {
    tft.fillRoundRect(45, 15, 90, 20, 7, ILI9341_BLACK);
    tft.fillRoundRect(185, 15, 90, 20, 7, ILI9341_BLACK);
    tft.fillRoundRect(110, 190, 100, 20, 8, ILI9341_ORANGE);
    tft.fillCircle(160, 190, 40, ILI9341_BLACK);
    tft.fillCircle(160, 174, 34, ILI9341_ORANGE);
    tft.fillRoundRect(110, 170, 100, 15, 8, ILI9341_ORANGE);
    tft.fillCircle(95, 110, 23, 0xBF2C);
    tft.fillCircle(95, 110, 17, ILI9341_BLACK);
    tft.fillCircle(102, 102, 4, ILI9341_WHITE);
    tft.fillCircle(225, 110, 23, 0xBF2C);
    tft.fillCircle(225, 110, 17, ILI9341_BLACK);
    tft.fillCircle(232, 102, 4, ILI9341_WHITE);

  }
  if (a == 2) //trist
  {
    tft.fillCircle(100, 120, 23, 0x055F);
    tft.fillCircle(100, 120, 17, ILI9341_BLACK);
    tft.fillCircle(107, 112, 4, ILI9341_WHITE);
    tft.fillCircle(220, 120, 23, 0x055F);
    tft.fillCircle(220, 120, 17, ILI9341_BLACK);
    tft.fillCircle(227, 112, 4, ILI9341_WHITE);
    tft.fillRoundRect(45, 15, 90, 20, 7, ILI9341_BLACK);
    tft.fillRoundRect(185, 15, 90, 20, 7, ILI9341_BLACK);
    tft.fillCircle(160, 210, 40, ILI9341_BLACK);
    tft.fillCircle(160, 225, 34, ILI9341_ORANGE);
    tft.fillRoundRect(110, 220, 100, 15, 8, ILI9341_ORANGE);
  }
  if (a == 3) //uimit(dezactivata momentan)
  {
    tft.fillRoundRect(45, 15, 90, 20, 7, ILI9341_BLACK);
    tft.fillRoundRect(185, 15, 90, 20, 7, ILI9341_BLACK);
    tft.fillCircle(160, 174, 34, ILI9341_ORANGE);
    tft.fillRoundRect(110, 170, 100, 15, 8, ILI9341_ORANGE);
    tft.fillCircle(95, 110, 23, 0xBF2C);
    tft.fillCircle(95, 110, 17, ILI9341_BLACK);
    tft.fillCircle(102, 102, 4, ILI9341_WHITE);
    tft.fillCircle(225, 110, 23, 0xBF2C);
    tft.fillCircle(225, 110, 17, ILI9341_BLACK);
    tft.fillCircle(232, 102, 4, ILI9341_WHITE);
    tft.fillCircle(160, 195, 40, ILI9341_BLACK);
    tft.fillCircle(160, 205, 24, ILI9341_ORANGE);
  }
}
void curatare_fata()
{ 
  tft.fillRoundRect(45, 15, 90, 20, 7, ILI9341_ORANGE);
  tft.fillRoundRect(185, 15, 90, 20, 7, ILI9341_ORANGE);
  tft.fillRect(90, 5, 140, 33, ILI9341_ORANGE);
  tft.fillRect(142, 5, 36, 43, ILI9341_ORANGE);
  tft.fillTriangle(131, 37, 145, 37, 145, 48, ILI9341_ORANGE);
  tft.fillTriangle(185, 37, 175, 37, 175, 49, ILI9341_ORANGE);
  tft.fillCircle(169, 45, 7, ILI9341_ORANGE);
  tft.fillCircle(197, 11, 8, ILI9341_ORANGE);
  tft.fillCircle(147, 41, 7, ILI9341_ORANGE);
  tft.fillCircle(113, 10, 8, ILI9341_ORANGE);
  tft.fillRoundRect(110, 190, 100, 20, 8, ILI9341_ORANGE);
  tft.fillCircle(160, 190, 40, ILI9341_ORANGE);
  tft.fillCircle(160, 210, 40, ILI9341_ORANGE);
  tft.fillCircle(160, 195, 40, ILI9341_ORANGE);
  tft.fillRoundRect(50, 50, 80, 100, 8, ILI9341_WHITE);
  tft.fillRoundRect(190, 50, 80, 100, 8, ILI9341_WHITE);
}
void start_lcd()
{ servo_cap.attach(35); 
  servo_cap.write(110);
  delay(200);
  servo_cap.detach();  
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_ORANGE);
  ochelari();
  tft.fillRoundRect(45, 15, 90, 20, 7, ILI9341_BLACK);
  tft.fillRoundRect(185, 15, 90, 20, 7, ILI9341_BLACK);
  tft.fillRoundRect(50, 50, 80, 100, 8, ILI9341_WHITE);
  tft.fillRoundRect(190, 50, 80, 100, 8, ILI9341_WHITE);
  tft.fillCircle(95, 110, 23, 0xBF2C);
  tft.fillCircle(95, 110, 17, ILI9341_BLACK);
  tft.fillCircle(102, 102, 4, ILI9341_WHITE);
  tft.fillCircle(225, 110, 23, 0xBF2C);
  tft.fillCircle(225, 110, 17, ILI9341_BLACK);
  tft.fillCircle(232, 102, 4, ILI9341_WHITE);
  tft.fillRoundRect(110, 190, 100, 20, 8, ILI9341_BLACK);
}
void stop_lcd()
{
  servo_cap.attach(35); 
  servo_cap.write(155);
  delay(200);
  servo_cap.detach();  
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_ORANGE);
  ochelari();
  tft.fillRoundRect(45, 15, 90, 20, 7, ILI9341_BLACK);
  tft.fillRoundRect(185, 15, 90, 20, 7, ILI9341_BLACK);
  tft.fillRoundRect(50, 50, 80, 100, 8, ILI9341_ORANGE);
  tft.fillRoundRect(190, 50, 80, 100, 8, ILI9341_ORANGE);
  tft.drawBitmap(150, 115, somn, 64, 64,0xF7DD);
  tft.fillCircle(140, 195, 24, ILI9341_BLACK);
  tft.fillCircle(143, 190, 14, 0xE1C8);
}
void fericit()
{
  go(150, -150);
  delay(350);
  go(0, 0);
  delay(50);
  go(-150, 150);
  delay(700);
  go(0, 0);
  delay(50);
  go(150, -150);
  delay(350);
  go(0, 0);
}
void ochelari()//afisarea pe LCD a ochelarilor robotului
{
  tft.fillRoundRect(40, 40, 100, 120, 15, 0xE1C8);
  tft.fillRoundRect(180, 40, 100, 120, 15, 0xE1C8);
  tft.fillRoundRect(130, 100, 70, 20, 8, 0xE1C8);
  tft.fillRoundRect(15, 100, 70, 20, 8, 0xE36A);
  tft.fillRoundRect(235, 100, 70, 20, 8, 0xE36A);
}
void Text(String text, uint16_t color, int x, int y, int textSize)
{
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.setTextWrap(true);
  tft.print(text);
}
//----------------------Functie alarma-----------------------
void alarma()
{
  senzori(1, 1, 0, 0, 0, 0);
  if (intoarcere == 1)
  { if (ret == 0)
    { go(120, 0);
      delay(850);
      go(0, -120);
      delay(900);
      go(0, 0);
      delay(50);
      go(120, 0);
      delay(850);
      go(0, -120);
      delay(900);
      go(0, 0);
      ret = 1;
    }
    if (distanceF > 20)
    {
      go(70, 70);
    }
    else
    {
      go(0, 0);
      intoarcere = 0;
      stop_lcd();
    }

  }
  if (intoarcere == 0 && distanceF < distmic)
  {
    if(oka!=1)
    {
    oka=1;
    tone(buzzer,1000);
    start_lcd();
    }
  }
  if (oka == 1)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      if (buzzerState == LOW)
      {
        buzzerState = HIGH;
        tone(buzzer, 1000);
      }
      else
      {
        buzzerState = LOW;
        noTone(buzzer);
      }

    }
  }
}
