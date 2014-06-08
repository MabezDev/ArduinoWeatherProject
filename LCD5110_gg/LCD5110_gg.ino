//
// This program requires a Nokia 5110 LCD module.
//
// It is assumed that the LCD module is connected to
// the following pins using a levelshifter to get the
// correct voltage to the module.
//      SCK  - Pin 8
//      MOSI - Pin 9
//      DC   - Pin 10
//      RST  - Pin 11
//      CS   - Pin 12
//
#include <LCD5110_Graph.h>

#include "DHT.h"



#define DHTPIN 3     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT11   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);

LCD5110 myGLCD(8,9,10,11.,12);


extern uint8_t SmallFont[];
extern uint8_t BigNumbers[];
extern uint8_t TinyFont[];


int y;
boolean button = false;
boolean lastb = false;
int buttonPin = 6;
int index = 0;
int maxPages = 5;
int light = 5;

int centerx = 42-2;//- 2 is half font size
int centery = 29;
int needleLenght = 12;
int bonus = 3;

float LightTime;
void setup()
{
 
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  dht.begin();
  pinMode(buttonPin,INPUT);
  pinMode(A0, INPUT);
  pinMode(light,OUTPUT);
  
}

void loop()
{ 
  pageManager();
  
  myGLCD.update();
}

void pageManager(){
 button = digitalRead(buttonPin);
 
  if (button != lastb) {
   digitalWrite(light,LOW);
   LightTime  = millis();
    
    if (button == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      index++;
      if(index > maxPages){
         index = 0; 
      }
    }else if(button ==LOW){
      
    }
    lastb = button;
  } 
  
  switch(index){
    case 0:
    myGLCD.clrScr();
    //temp();
    tempAndHum();
    break;
    case 1:
    myGLCD.clrScr();
    soilHum();
    break;
    case 2:
    myGLCD.clrScr();
    windDirection();
    break;
    case 3:
    myGLCD.clrScr();
    sleepPage();
    break;
    case 4:
    myGLCD.clrScr();
    windSpeed();
    break;
    case 5:
    myGLCD.clrScr();
    batteryMonitor();
    break;
    
  }
  if((millis() - LightTime)>15000){
   digitalWrite(light,HIGH); 
   index = 3;
  }
}


void tempAndHum(){
  y = 0;
    
    int h = (int)dht.readHumidity();
    int t = (int)dht.readTemperature();
    myGLCD.setFont(TinyFont);
    myGLCD.print("Temperature:",0,24);
    myGLCD.print("Humidity:",0,0);
    myGLCD.setFont(SmallFont);
    myGLCD.print("%",75,10);
    myGLCD.print("C",75,28);
    myGLCD.setFont(BigNumbers);
    myGLCD.printNumI(t,48,24,2);
    myGLCD.printNumI(h,48,0,2);
    delay(10); 
}

void soilHum(){
  int input = analogRead(A0);
  float voltage = (input * 5.0) /1024;
  float percent = 100 - (voltage/5)*100;
  myGLCD.setFont(SmallFont);
  myGLCD.print("Soil Humidity:",0,0);
  myGLCD.print("%",64,36);
  myGLCD.setFont(BigNumbers);
  myGLCD.printNumI((int)percent,32,24,2,'0');
  
}

void windDirection(){//for debuggin remove when input aquired
 myGLCD.setFont(SmallFont);
 myGLCD.print("Wind Direction",0,0);
 myGLCD.print("N",centerx,centery-needleLenght-bonus-3);
 myGLCD.print("NE",centerx+needleLenght+bonus,centery-needleLenght-bonus-3);
 myGLCD.print("E",centerx+needleLenght+bonus+6,centery-3);
 myGLCD.print("SE",centerx+needleLenght+bonus,centery+needleLenght+bonus-3);
 myGLCD.print("S",centerx,centery+needleLenght+bonus-3);
 myGLCD.print("SW",centerx-needleLenght-bonus-5,centery+needleLenght+bonus-3);
 myGLCD.print("W",centerx-needleLenght-bonus-5,centery-3);
 myGLCD.print("NW",centerx-needleLenght-bonus-5,centery-needleLenght-bonus-3);

   
moveNeedleTo(0); 
}

void moveNeedleTo(int pos){
  int posi = pos;
  int needleTempx = centerx +4;
 switch(posi){
  case 0 :
    myGLCD.drawLine(needleTempx, centery,needleTempx,centery-needleLenght);
    break;
  case 1:
    myGLCD.drawLine(needleTempx, centery,needleTempx+needleLenght,centery-needleLenght);
    break;
  case 2:
    myGLCD.drawLine(needleTempx, centery,needleTempx+needleLenght,centery);
    break;
  case 3:
    myGLCD.drawLine(needleTempx, centery,needleTempx+needleLenght,centery+needleLenght);
    break;
  case 4:
    myGLCD.drawLine(needleTempx, centery,needleTempx,centery+needleLenght);
    break;
  case 5:
    myGLCD.drawLine(needleTempx, centery,needleTempx-needleLenght,centery+needleLenght);
    break;
  case 6:
    myGLCD.drawLine(needleTempx, centery,needleTempx-needleLenght,centery);
    break;
  case 7:
    myGLCD.drawLine(needleTempx, centery,needleTempx-needleLenght,centery-needleLenght);
    break;
 } 
  
}

void windSpeed(){
  myGLCD.setFont(SmallFont);
  myGLCD.print("Wind speed:",10,0);
  myGLCD.print("m/s",60,36);
  myGLCD.setFont(BigNumbers);
  myGLCD.printNumF(0.4,2,0,24,'.',2,'0');
}

void batteryMonitor(){
  myGLCD.setFont(SmallFont);
  myGLCD.print("Battery level:",0,0);
  myGLCD.print("%",67,34);
  myGLCD.setFont(BigNumbers);
  myGLCD.printNumI(100,20,24,3,'0');
}

void sleepPage(){
  myGLCD.setFont(SmallFont);
   myGLCD.print("Sleeping!",12,0);
   myGLCD.print("Press X",24,20);
   myGLCD.print("To Wake!",24,35);
}

