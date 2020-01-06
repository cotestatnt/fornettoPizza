#include <EEPROM.h>
#include <Wire.h>
#include <max6675.h>
#include <SPI.h>
#include <Ticker.h>
#include <AutoPID.h>
#include <PinChangeInterrupt.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "SeqButton.h"

SeqButton  ShortClick, LongClick, SaveSetClick;
SSD1306AsciiWire oled;

#define WindowSize1  5000
#define WindowSize2  10000

// Output 
#define RelayPin1 7
#define RelayPin2 8
// Input
#define ENC_A 6    
#define ENC_B 5   
#define SWITCH 4  

// SPI communication
#define thermoCLK 13
#define thermoDO  12
#define thermoCS1 10
#define thermoCS2 9
MAX6675 TC1(thermoCLK, thermoCS1, thermoDO);
MAX6675 TC2(thermoCLK, thermoCS2, thermoDO);
 
void setRelay1();
void setRelay2();
void sendPlotData();
void readTemp();
Ticker ticker1(setRelay1, 100);
Ticker ticker2(setRelay2, 100);
Ticker tickerPlot(sendPlotData, 5000);

// ************************************************
// PID Variables and constants
// ************************************************
double Setpoint1 = 300.0F, Setpoint2 = 300.0F, oldSetpoint;
double Input1, Input2;
double Output1, Output2;
double Temp1, Temp2;
double Kp1 = 140.5, Kp2 = 90.0;
double Ki1 = 100.0, Ki2 = 0;
double Kd1 = 1000.0, Kd2 = 10.0;

//Specify the links and initial tuning parameters
AutoPID myPID1(&Input1, &Setpoint1, &Output1, 500, WindowSize1, Kp1, Ki1, Kd1);
AutoPID myPID2(&Input2, &Setpoint2, &Output2, 500, WindowSize2, Kp2, Ki2, Kd2);

unsigned long windowStartTime1, windowStartTime2;
unsigned long plotTime, updateTime;
unsigned int  minStartTime = 500;
volatile long onTime1, onTime2, encoder;
long lastEncoder;
char oled_buffer[50];
uint8_t heaterSel = 0, oled_page = 0, kSel = 0, setpointSel = 0;
bool ProgramMode = false , SaveSetpoint = true, ShortEnabled = true;

void setup(void) {
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(9600);
  Serial.println(F("START"));
  
  digitalWrite(RelayPin1, HIGH);             // make sure it is off to start 
  pinMode(RelayPin1, OUTPUT);                // Output mode to drive relay
  digitalWrite(RelayPin2, HIGH);             // make sure it is off to start 
  pinMode(RelayPin2, OUTPUT);                // Output mode to drive relay  
  pinMode(SWITCH, INPUT_PULLUP);  
  pinMode(ENC_A, INPUT_PULLUP); 
  pinMode(ENC_B, INPUT_PULLUP); 
 
  ShortClick.init(SWITCH, NULL, &f_ShortClick, false, LOW, 60);
  LongClick.init(SWITCH, &f_LongClick, NULL, false, LOW, 2000);
  SaveSetClick.init(SWITCH, &f_SaveSetClick, NULL, false, LOW, 500);
  
  // Initialize the PID and related variables   
  myPID1.setGains(Kp1,Ki1,Kd1);
  myPID1.setTimeStep(250);
  myPID2.setGains(Kp2,Ki2,Kd2);
  myPID2.setTimeStep(250);
  windowStartTime1 = millis();
  windowStartTime2 = millis();
  ticker1.start();
  ticker2.start();
  tickerPlot.start();
  oled.begin(&SH1106_128x64, 0x3C);
  attachPCINT(digitalPinToPCINT(ENC_A), leggi_encoder, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_B), leggi_encoder, CHANGE);
  readEEprom();
}
unsigned long _time2On, time2On;

void loop(void) {   
  switch(heaterSel){ 
    case 0: myPID1.run(); myPID2.run();  break; 
    case 1: myPID1.stop(); myPID2.run(); Output1=0; break; 
    case 2: myPID1.run(); myPID2.stop(); Output2=0; break;
  }
  SaveSetClick.handler();
  ShortClick.handler();
  LongClick.handler();
  ticker1.update();
  ticker2.update();
  update_oled(oled_page);
  if(millis()-updateTime > 1000){
    updateTime = millis();    
    if(Setpoint1 != oldSetpoint){
      oldSetpoint = Setpoint1;
      myPID1.reset();
      myPID2.reset();
      myPID1.setBangBang(100.0F, 5.0F);
      myPID2.setBangBang(50.0F, 5.0F);
      update_oled(oled_page);
    }        
    Temp1 = TC1.readCelsius();
    Temp2 = TC2.readCelsius();
    Temp2 += 0.1*Temp2;  
    Input1 = Temp1;
    Input2 = Temp2;
  }  
  // Time Proportional relay state is updated regularly by ticker
  onTime1 = Output1;    
  onTime2 = Output2;    

  if(encoder != lastEncoder){  
    switch(oled_page){
      case 0: 
        switch(setpointSel){ 
          case 0: 
            Setpoint1 += encoder *2;   
            Setpoint1 = constrain(Setpoint1, 25, 450); 
            Setpoint2 = Setpoint1;  
          break; 
          case 1: 
            Setpoint1 += encoder *2;   
            Setpoint1 = constrain(Setpoint1, 25, 450); 
            break; 
          case 2:
            Setpoint2 += encoder *2;   
            Setpoint2 = constrain(Setpoint2, 25, 450); 
            break;
        } 
        
        break;
      case 2: 
        switch(kSel){ 
          case 0: Kp1 += encoder;  break; 
          case 1: Ki1 += encoder;  break; 
          case 2: Kd1 += encoder;  break;
        }          
        break;
      case 3: 
        switch(kSel){ 
          case 0: Kp2 += encoder;  break; 
          case 1: Ki2 += encoder;  break; 
          case 2: Kd2 += encoder;  break;
        }    
        break;
    } 
    Serial.println(encoder);
    encoder = 0;
    lastEncoder = encoder;
  }
  
  tickerPlot.update();                         
}

// **************************      Plot temperature data     *************************
void sendPlotData(){
  Serial.print(Temp1);
  Serial.print("\t");
  Serial.print(Temp2); 
  Serial.print("\t");
  Serial.print(100); 
  Serial.print("\t");
  Serial.println(200);    
}

// **************************      Ticker  Handler     *************************
// **************************         RELAY 1         *************************
void setRelay1(){
 // Time to shift the Relay Window
  if(millis() - windowStartTime1 > WindowSize1) 
     windowStartTime1 += WindowSize1;  
  if((onTime1 > minStartTime) && (onTime1 > (millis() - windowStartTime1)))
     digitalWrite(RelayPin1, LOW);  
  else  
     digitalWrite(RelayPin1, HIGH);
}

// **************************         RELAY 2          *************************
void setRelay2(){  
  // Time to shift the Relay Window
  if(millis() - windowStartTime2 > WindowSize2) 
     windowStartTime2 += WindowSize2;       
  if((onTime2 > minStartTime) && (onTime2 > (millis() - windowStartTime2)))
    digitalWrite(RelayPin2, LOW);       
  else 
    digitalWrite(RelayPin2, HIGH);
}

void readEEprom(){
  uint16_t adr = 0;
  EEPROM.get(adr, Setpoint1);
  adr += sizeof(double);
  EEPROM.get(adr, Setpoint2);  
  adr += sizeof(double);
  EEPROM.get(adr, Kp1); 
  adr += sizeof(double);
  EEPROM.get(adr, Ki1); 
  adr += sizeof(double);
  EEPROM.get(adr, Kd1); 
  adr += sizeof(double);
  EEPROM.get(adr, Kp2); 
  adr += sizeof(double);
  EEPROM.get(adr, Ki2); 
  adr += sizeof(double);
  EEPROM.get(adr, Kd2); 
}
  


// ISR (Interrupt Service Routine) associata al cambiamento di stato di PINA e PINB
void leggi_encoder(void) {
  const int statiValidi[] PROGMEM = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, +1, 0};  
  static volatile uint8_t statoEncoder = 0;
  static volatile uint8_t indiceArray = 0;
  static volatile bool ticToc = false;            
  // Eseguo uno shift di due posizioni a sinistra per non sovrascrivere la lettura precedente  
  statoEncoder <<= 2;  
  // Imposto i bit relativi allo stato attuale del canale A e del canale B  
  statoEncoder |=  bitRead(PIND, ENC_A) << 1 | bitRead(PIND, ENC_B) ;       
  // Applico la maschera per tenere conto solo dei 4 bit di encoderState realmente usati
  indiceArray = 0b00001111 & statoEncoder; 
  // Aggiorno il contatore solo quando ho una variazione sia di PINA che di PINB (quindi ogni 2 interrupt)   
  if (ticToc) {
    encoder += statiValidi[indiceArray];      
  }
  ticToc = !ticToc;
}


///// ****************** Save setpoint button ****************** /////
void f_SaveSetClick(){  
  if(!ProgramMode)  
    if(SaveSetpoint) {
      Serial.println(F("Save Click"));
      uint16_t adr = 0;
      EEPROM.put(adr, Setpoint1);
      adr += sizeof(double);
      EEPROM.put(adr, Setpoint2);      
      oled.clear(); 
      oled.setFont(System5x7);
      oled.println("Setpoint salvato");
      delay(1000);   
      oled.clear();
      oled_page = 0;
      setpointSel = 0;
      update_oled(oled_page);
      delay(500);
    }  
  else
   SaveSetpoint = true; 
}


///// ****************** Short Switch button ****************** /////
void f_ShortClick(){
  if(ShortEnabled){
    Serial.println(F("Short Click"));
    Serial.print(F("Selezione: ")); 
    if(oled_page == 0){
      setpointSel = (setpointSel+1) % 3;
      Serial.println(heaterSel);
    }
    else if(oled_page == 1){
      heaterSel = (heaterSel+1) % 3;
      Serial.println(heaterSel);
    }
    else if((oled_page == 2)||(oled_page == 3)){ 
      kSel = (kSel+1) % 3;
      Serial.println(kSel);
    }
  }
  else
    ShortEnabled = true;
}

///// ****************** Long Switch button ****************** /////
void f_LongClick(){
  Serial.println(F("Long Click"));
  switch(oled_page){
    case 0:
      ProgramMode = true;
      SaveSetpoint = false;
      ShortEnabled = false;
      Serial.println(F("Program mode ON"));
      oled_page = 1;      
      update_oled(oled_page);
      delay(500);
      break;
    case 1:
      oled_page = (oled_page+1)%5;
      update_oled(oled_page);
      ShortEnabled = false;
      break;
    case 2:
      oled_page = (oled_page+1)%5;
      update_oled(oled_page);
      ShortEnabled = false;
      break;
    case 3:
      oled_page = (oled_page+1)%5;
      update_oled(oled_page);
      ShortEnabled = false;
      break;
    case 4:    
      double Kp1 = 140.5, Kp2 = 90.0;
      double Ki1 = 100.0, Ki2 = 0;
      double Kd1 = 1000.0, Kd2 = 10.0; 
      uint16_t adr = 0;
      EEPROM.put(adr, Setpoint1);
      adr += sizeof(double);
      EEPROM.put(adr, Setpoint2); 
      adr += sizeof(double);
      EEPROM.put(adr, Kp1); 
      adr += sizeof(double);
      EEPROM.put(adr, Ki1); 
      adr += sizeof(double);
      EEPROM.put(adr, Kd1); 
      adr += sizeof(double);
      EEPROM.put(adr, Kp2); 
      adr += sizeof(double);
      EEPROM.put(adr, Ki2); 
      adr += sizeof(double);
      EEPROM.put(adr, Kd2); 
      oled_page = 0;
      oled.clear(); 
      update_oled(oled_page);
      delay(2000);
      SaveSetpoint = true;
      ShortEnabled = true;
      break;      
  }
}


// **************************      OLED DISPLAY  Handler     *************************
void update_oled(uint8_t oled_page){
  #define MAXBAR 25
  uint16_t pct = 0;
  static uint32_t updateOled;
  char s1, s2, s3;
  switch(oled_page){
    case 0:
      switch(setpointSel){ 
        case 0: s1=' '; s2=' '; break; 
        case 1: s1=':'; s2=' '; break; 
        case 2: s1=' '; s2=':'; break; 
      }
      oled.setCursor(0,0);
      oled.setFont(cambria_22_bold_digit);
      sprintf(oled_buffer, "%3d %c", (int)Setpoint1, s1);
      oled.println(oled_buffer);
     
      oled.setFont(progressbar);  
      pct = map(Output1, 0, WindowSize1, 0, MAXBAR);
      for(uint8_t i=0; i<MAXBAR; i++)
        if(pct > i)
          oled.print("!");
        else 
          oled.print(".");
      oled.println("");  
    
      oled.setFont(cambria_22_bold_digit);
      sprintf(oled_buffer, "%3d %c", (int)Setpoint2, s2);
      oled.println(oled_buffer);
      
      oled.setFont(progressbar);  
      pct = map(Output2, 0, WindowSize2, 0, MAXBAR);
      for(uint8_t i=0; i<MAXBAR; i++)
        if(pct > i)
          oled.print("!");
        else 
          oled.print(".");
      oled.println("");
     
      oled.setFont(cambria32_bold_digit);
      oled.setCursor(55,0);
      sprintf(oled_buffer, "%03d@", (int)Temp1);  
      oled.println(oled_buffer);
     
      oled.setCursor(55,35);
      sprintf(oled_buffer, "%03d@", (int)Temp2);
      oled.println(oled_buffer);
      break;
    case 1:
     if(millis() - updateOled > 1000){
        switch(heaterSel){ 
          case 0: s1='*'; s2=' '; s3=' '; break; 
          case 1: s1=' '; s2='*'; s3=' '; break; 
          case 2: s1=' '; s2=' '; s3='*'; break;
        }
        updateOled = millis(); 
        oled.clear();
        oled.setFont(Callibri11);
        oled.println("Selezione resistenze:");
        oled.print("Superiore/Inferiore    ");  oled.println(s1); 
        oled.print("Inferiore              ");  oled.println(s2); 
        oled.print("Superiore              ");  oled.println(s3); 
      }    
      break;
    case 2:
     if(millis() - updateOled > 1000){
        switch(kSel){ 
          case 0: s1='*'; s2=' '; s3=' '; break; 
          case 1: s1=' '; s2='*'; s3=' '; break; 
          case 2: s1=' '; s2=' '; s3='*'; break;
        }
        updateOled = millis(); 
        oled.clear();
        oled.println("SETUP PID 1");
        oled.print("Kp: "); oled.print((int)Kp1); oled.println(s1);
        oled.print("Ki: "); oled.print((int)Ki1); oled.println(s2); 
        oled.print("Kd: "); oled.print((int)Kd1); oled.println(s3);
      }    
      break;
    case 3:
     if(millis() - updateOled > 1000){
        switch(kSel){ 
          case 0: s1='*'; s2=' '; s3=' '; break; 
          case 1: s1=' '; s2='*'; s3=' '; break; 
          case 2: s1=' '; s2=' '; s3='*'; break;
        }
        updateOled = millis(); 
        oled.clear();
        oled.println("SETUP PID 2");
        oled.print("Kp: "); oled.print((int)Kp2); oled.println(s1);
        oled.print("Ki: "); oled.print((int)Ki2); oled.println(s2); 
        oled.print("Kd: "); oled.print((int)Kd2); oled.println(s3);
      }    
      break;
    case 4:
     if(millis() - updateOled > 500){        
        updateOled = millis(); 
        oled.clear();
        oled.println("");
        oled.println("     Tenere premuto 2s");
        oled.println("   per salvare in EEPROM.");        
     }    
     break;
  }
}
