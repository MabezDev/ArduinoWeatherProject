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

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);



#define DHTPIN 4     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT11   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);

LCD5110 myGLCD(8,9,10,11,12);


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
long oldPosition  = -999;
long angle;

float LightTime;
void setup()
{
 
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  dht.begin();
  pinMode(buttonPin,INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(light,OUTPUT);
  digitalWrite(light,LOW);//turn lcd on initally
 Serial.begin(9600);
}

void loop()
{ 
  returnDegree();
  
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
    case 2:
    myGLCD.clrScr();
    //temp();
    tempAndHum();
    break;
    case 1:
    myGLCD.clrScr();
    soilHum();
    break;
    case 0:
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
 Serial.println(angle);//WORKING DO NOT CHNAGE THESE NUMBERS
 if((angle < 44 && angle >= 0)  || (angle <=360 && angle>=342)){
   moveNeedleTo(0);
 }
 if(angle < 89 && angle >=45){
   moveNeedleTo(1);
 }
 if(angle < 134 && angle >=90){
   moveNeedleTo(2);
 }
 if(angle < 179 && angle >=135){
   moveNeedleTo(3);
 }
 if(angle < 224 && angle >=180){
   moveNeedleTo(4);
 }
 if(angle < 269 && angle >=225){
   moveNeedleTo(5);
 }
 if(angle < 314 && angle >=270){
   moveNeedleTo(6);
 }
 if(angle <= 340 && angle >=315){
   moveNeedleTo(7);
 }
 
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
}

void returnDegree(){
 long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    long degree = (newPosition/4)*18;
    
    if(degree<0){
      degree = 360;
      oldPosition =  999;
      myEnc.write(80);
    }
    if(degree >360){
       degree = 0; 
       oldPosition =  0;
      myEnc.write(0);
    }
    
    
    angle = degree;
}
//else{
 // return angle;
//}
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
  float battVol = (float)readVcc()/1000;
  
  float percent = (battVol/3.2)*100;
  myGLCD.setFont(SmallFont);
  
  myGLCD.print("Battery level:",0,0);
  myGLCD.print("%",67,34);
  myGLCD.setFont(BigNumbers);
  //myGLCD.printNumI((int)battVol,0,0);
  myGLCD.printNumI((int)percent,20,24,3,'0');
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
void sleepPage(){
  myGLCD.setFont(SmallFont);
   myGLCD.print("Sleeping!",12,0);
   myGLCD.print("Press X",24,20);
   myGLCD.print("To Wake!",24,35);
}

