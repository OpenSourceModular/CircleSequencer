
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_MCP4725.h>
//Pin Definitions
// Display Pins:
//  defined in User_Setup.h
//  TFT_MOSI 23
//  TFT_SCLK 18
//  TFT_CS 3(not used)
//  TFT_DC 16
//  TFT_RST 4
//  TFT_BL not used
const int CS_PINS[2] = {19, 17};
//  TFT Chip Select pins 19,17
// DAC Pins:
//  I2C SCL 22 -> DACs
//  I2C SDA 21 -> DACs
// Pulse Out Pins
int pulsePins[6] = {12, 13, 12, 13, 12, 13};
#define P1_OUT_PIN 12
#define P2_OUT_PIN 13
//#define P3_OUT_PIN
//Encoder Pins
#define ENCODER2_A_PIN 32 //36
#define ENCODER2_B_PIN 33 //39
#define ENCODER1_A_PIN 25 //34
#define ENCODER1_B_PIN 26 //35
//Encoder Switch Pins
#define ENCODER2_SW_PIN 27 //32
#define ENCODER1_SW_PIN 14 //33



byte middleFontSize = 6;
byte bottomFontSize = 1;

void loop0(void * parameter);
void loop1(void * parameter);
void isr();
void read_encoder1();
void read_button1();
void read_encoder2();
void read_button2();
bool stepRead(int side, int ring, int aStep);
void stepSet(int side, int ring, int aStep);
void stepClear(int side, int ring, int aStep);
void drawEditCursor(int side, int ring, int segNum);
void sendPulse1();
void sendPulse2();
void drawSegment(int side, int ring, int segNum, uint16_t aColor);
void selectDisplay(int screen);
void deselectDisplay(int screen);
void checkButtons();
void drawNumber(int side, int number);
void checkEncoders();
void generateFakeClocks();
void sendPulse(int destination);
void checkPulseEnds();
uint16_t noteNumber(uint16_t aValue);
void drawNote(int side, int note);


TaskHandle_t Task0;
TaskHandle_t Task1;

TFT_eSPI tft = TFT_eSPI();
Adafruit_MCP4725 dac; // creates an instance of the DAC
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;



static volatile bool encoder1Change = false;
static volatile bool encoder2Change = false;
static volatile int currEditStep = 0;
static volatile int currEditRing = 0;
static volatile int currEditSide = 0;
static volatile int encoder1Counter = 0;
static volatile int encoder2Counter = 0;
static volatile uint16_t stepPosition[2][3] { 0,0,0,
                                              0,0,0};
static volatile uint16_t ringDivisions[2][3] = {
  16,12,6,
  16,12,6
};
static volatile int button1Value = 0;
static volatile int button2Value = 0;

static volatile unsigned long lastButton1Press = 0;
static volatile unsigned long lastButton2Press = 0;
static volatile int buttonDebounce = 250;

static volatile uint32_t ringSteps[2][3] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, // 32 bit int stores steps on/off
                          0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
static volatile uint16_t ringValues[2][3][32] = {
  2,5,6,4,5,6,7,8, 9,10,11,12,13,14,15,16, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0
};                         
static volatile int ringPulseDest[2][3] = {0,1,1,1,1,1};                          
int counter = 0;
int ringRadii[4] = {119,98,77,56};
int ringColors[2][3] = {
  TFT_BLUE, TFT_BLUE, TFT_BLUE,
  TFT_BLUE, TFT_BLUE, TFT_BLUE
  };


unsigned long lastClock1In = 0;
unsigned long lastClock2In = 0;
unsigned long lastClock3In = 0;
unsigned long lastClock4In = 0;
unsigned long lastClock5In = 0;
unsigned long lastClock6In = 0;  
unsigned long lastPulse1Sent = 0;
unsigned long lastPulse2Sent = 0;
unsigned long lastClockIn[2][3] = {0,0,0,0,0,0};
unsigned long lastPulseSent[6] = {0,0,0,0,0,0}; 
int lastCounter1 = 1;
int lastCounter2 = 1;
uint16_t lastNote = 0;
bool clock1In = false;

void setup() {
  // put your setup code here, to run once:
  for (int r = 0; r<16; r++)
  {
    ringValues[0][0][r] = random(0,4096);
  }
  Serial.begin(115200);
  Serial.println("Setup started.");
  dac.begin(0x60);
  dac.setVoltage( 0, false);


  xTaskCreatePinnedToCore(loop0, "Task0", 20000, NULL, 0, &Task0, 0); 
  xTaskCreatePinnedToCore(loop1, "Task1", 20000, NULL, 0, &Task1, 1); 
  
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_SW_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_SW_PIN, INPUT_PULLUP);
  pinMode(2,OUTPUT);
  pinMode(P1_OUT_PIN, OUTPUT);
  pinMode(P2_OUT_PIN, OUTPUT);
  pinMode(19,OUTPUT);
  pinMode(17,OUTPUT);
  Serial.println("Setup complete.");
  vTaskDelete (NULL);
}


void loop0(void * parameter) {
  
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), read_encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B_PIN), read_encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), read_encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B_PIN), read_encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_SW_PIN), read_button1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_SW_PIN), read_button2, FALLING);
  for (;;) 
  {
    //vTaskDelay(1);
    checkEncoders();
    generateFakeClocks();
    checkPulseEnds();
  }
}
void checkPulseEnds()
{
  for (int pulseDest = 0; pulseDest<5; pulseDest++)
  {
    if ((millis() - lastPulseSent[pulseDest])>10)
    { 
      if (pulseDest==0) digitalWrite(2,LOW);
      digitalWrite(pulsePins[pulseDest], HIGH);
    }
  }
}
void generateFakeClocks()
{
  int clockLengths[2][3] = {500,200,300,
                            400,500,600};
                           
  for (int side = 0; side<2; side++)
  {
    for (int ring = 0; ring<3; ring++)
    {
      if ((millis()-lastClockIn[side][ring]) > clockLengths[side][ring])
      {
        if ((side==0)&&(ring==0)) {
          dac.setVoltage(ringValues[0][0][stepPosition[0][0]],false);
          //Serial.println(ringValues[0][0][stepPosition[0][0]]);
        }

        lastClockIn[side][ring]=millis();
        stepPosition[side][ring]++;
        if(stepPosition[side][ring] == ringDivisions[side][ring]) stepPosition[side][ring]=0;
        if(stepRead(side,ring,stepPosition[side][ring])) sendPulse(ringPulseDest[side][ring]);
      }
    }
  }
}
void sendPulse(int destination)
{
  
  lastPulseSent[destination] = millis();
  if (destination == 0) digitalWrite(2,HIGH);
  digitalWrite(pulsePins[destination], LOW);
}
void checkEncoders()
{
  if (encoder1Change){
    portENTER_CRITICAL_ISR(&spinlock); 
    encoder1Change = false;
    portEXIT_CRITICAL_ISR(&spinlock);
    if(encoder1Counter > lastCounter1) {
      currEditStep++;
      if (currEditStep > ringDivisions[currEditSide][currEditRing]-1) currEditStep = 0;
    }
    if(encoder1Counter < lastCounter1) {
      currEditStep--;
      if (currEditStep < 0) currEditStep = ringDivisions[currEditSide][currEditRing]-1;
    }
    lastCounter1 = encoder1Counter;
  }
  if (encoder2Change){
    portENTER_CRITICAL_ISR(&spinlock); 
    encoder2Change = false;
    portEXIT_CRITICAL_ISR(&spinlock);
    if(encoder2Counter < lastCounter2) {
      currEditRing++;
      if (currEditRing > 2) currEditRing = 0;
    }
    if(encoder2Counter > lastCounter2) {
      currEditRing--;
      if (currEditRing < 0) currEditRing = 2;
    }
    currEditStep = 0;
    lastCounter2 = encoder2Counter;
  }

}

void sendPulse1()
{
  digitalWrite(2,HIGH);
  digitalWrite(P1_OUT_PIN, LOW);
  lastPulse1Sent = millis();
}
void sendPulse2()
{
  
  digitalWrite(P2_OUT_PIN, LOW);
  lastPulse2Sent = millis();
}

void loop1(void * parameter) {
  char newstr[20];
  int lastCounter = 15;
  int lastStepPosition[2][3] = {1,1,1,
                                1,1,1};
  int lastCurrEditSide = 0;
  int lastCurrEditRing = 0;
  int lastCurrEditStep = 0;
  Serial.println("Loop1 Begins");
  tft.init();
  tft.setRotation(0);
  selectDisplay(0);
  tft.fillScreen(TFT_BLACK);
  deselectDisplay(0);
  selectDisplay(1);
  tft.fillScreen(TFT_BLACK);
  deselectDisplay(1);
  //drawNumber(currEditSide, currEditStep+1); 
  for (;;)
  { 
    //sprintf(newstr, "Task H, Core %i\r\n", xPortGetCoreID());
    //Serial.print(newstr);
    //Serial.println(counter);
    //vTaskDelay(1);
    checkButtons();
    if ((currEditStep != lastCurrEditStep) || (currEditRing != lastCurrEditRing) || (currEditSide != lastCurrEditSide))
    { 
      if (stepRead(lastCurrEditSide, lastCurrEditRing, lastCurrEditStep)) 
        drawSegment(lastCurrEditSide, lastCurrEditRing, lastCurrEditStep, ringColors[lastCurrEditSide][lastCurrEditRing]);    
      else 
        drawSegment(lastCurrEditSide, lastCurrEditRing, lastCurrEditStep, TFT_BLACK);       
      lastCurrEditStep = currEditStep;
      lastCurrEditSide = currEditSide;
      lastCurrEditRing = currEditRing;
      //drawNumber(currEditSide, currEditStep+1);        
    }
    for (int side=0; side<2; side++){
      for (int ring=0; ring<3; ring++){
        if (stepPosition[side][ring] != lastStepPosition[side][ring])
        { 
          if(stepRead(side,ring,lastStepPosition[side][ring])) drawSegment(side,ring,lastStepPosition[side][ring], ringColors[side][ring]);    
          else drawSegment(side,ring,lastStepPosition[side][ring], TFT_BLACK);    
          lastStepPosition[side][ring] = stepPosition[side][ring];
          //int c = random(0x10000);
          drawSegment(side,ring,stepPosition[side][ring], TFT_WHITE);
          if (noteNumber(ringValues[0][0][stepPosition[0][0]])!=lastNote)
          {
            lastNote = noteNumber(ringValues[0][0][stepPosition[0][0]]);
            drawNote(0,(lastNote%12));
          }    
        }
      }
    }
    drawEditCursor(currEditSide, currEditRing, currEditStep);
 
  }
}
uint16_t noteNumber(uint16_t aValue)
{
  return (aValue/68); 
}
void drawSegment(int side, int ring, int segNum, uint16_t aColor){
  
    
    int spacingInDegrees = 5;
    float divisionAngle = 360.0 / (float)ringDivisions[side][ring];
    float startAngle = ((float)segNum * divisionAngle);
    startAngle = startAngle + 180.0; 
    if (startAngle > 360) startAngle = startAngle-360.0;
    if (startAngle == 360) startAngle = 0.0;
    int endAngle = startAngle + (int)divisionAngle - spacingInDegrees;
    if (endAngle > 360) endAngle = endAngle-360.0;
    if (endAngle == 360) endAngle = 0.0;
    //Serial.println("G");
    //Serial.println(startAngle);
    //Serial.println(endAngle);
    selectDisplay(side);
    tft.drawArc(120,120, ringRadii[ring], ringRadii[ring]-17, round(startAngle), endAngle, aColor, aColor, true);
    //tft.drawArc(120,120, ringRadii[ring], ringRadii[ring]-17, 0, 45, aColor, aColor, true);
    deselectDisplay(side);
    //Serial.println("H");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelete (NULL);
}
void selectDisplay(int screen)
{
  pinMode(CS_PINS[screen], OUTPUT);
  digitalWrite(CS_PINS[screen], LOW);  // select the display
  
}
void deselectDisplay(int screen)
{
  pinMode(CS_PINS[screen], OUTPUT);
  digitalWrite(CS_PINS[screen], HIGH);  // deselect the display
  
}
void IRAM_ATTR isr() {
  Serial.println("BTN pressed");
}
void IRAM_ATTR read_button1() {
  if ((millis()-lastButton1Press)>buttonDebounce)
  {
    
    portENTER_CRITICAL_ISR(&spinlock); 
    lastButton1Press = millis();
    button1Value = 1;
    portEXIT_CRITICAL_ISR(&spinlock);
  }
}
void read_button2() {

  if ((millis()-lastButton2Press)>buttonDebounce)
  {
    
    portENTER_CRITICAL_ISR(&spinlock); 
    button2Value = 1;
    lastButton2Press = millis();  
    portEXIT_CRITICAL_ISR(&spinlock);
  }
}
void IRAM_ATTR read_encoder1() {
  
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENCODER1_A_PIN)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENCODER1_B_PIN)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    portENTER_CRITICAL_ISR(&spinlock); 
    encoder1Counter++;              // Increase counter
    encoder1Change = true;
    portEXIT_CRITICAL_ISR(&spinlock);
    //if (counter>15) counter = 0;
    encval = 0;
  }
  else if( encval < -3 ) {  // Four steps backwards
   portENTER_CRITICAL_ISR(&spinlock); 
   encoder1Counter--;               // Decrease counter
   encoder1Change = true;
   portEXIT_CRITICAL_ISR(&spinlock);
   //if (counter < 0) counter = 15;
   encval = 0;
  }
}
void IRAM_ATTR read_encoder2() {
  
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENCODER2_A_PIN)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENCODER2_B_PIN)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    portENTER_CRITICAL_ISR(&spinlock); 
    encoder2Counter++;              // Increase counter
    encoder2Change = true;
    portEXIT_CRITICAL_ISR(&spinlock);
    //if (counter>15) counter = 0;
    encval = 0;
  }
  else if( encval < -3 ) {  // Four steps backwards
   portENTER_CRITICAL_ISR(&spinlock); 
   encoder2Counter--;               // Decrease counter
   encoder2Change = true;
   portEXIT_CRITICAL_ISR(&spinlock);
   //if (counter < 0) counter = 15;
   encval = 0;
  }
}
bool stepRead(int side, int ring, int aStep){
  if (bitRead(ringSteps[side][ring],aStep)) return true;
  else return false;
}

void stepSet(int side, int ring, int aStep){
  portENTER_CRITICAL_ISR(&spinlock);
  bitSet(ringSteps[side][ring], aStep);
  portEXIT_CRITICAL_ISR(&spinlock);
}

void stepClear(int side, int ring, int aStep){
  portENTER_CRITICAL_ISR(&spinlock);
  bitClear(ringSteps[side][ring], aStep);
  portEXIT_CRITICAL_ISR(&spinlock);
}
void drawEditCursor(int side, int ring, int segNum){
  int spacingInDegrees = 5;
  float divisionAngle = 360.0 / (float)ringDivisions[side][ring];
  float startAngle = ((float)segNum * divisionAngle);
  startAngle = startAngle + 180.0;
  if (startAngle > 360) startAngle = startAngle-360.0;
  if (startAngle == 360) startAngle = 0.0; 
  int endAngle = startAngle + (int)divisionAngle - spacingInDegrees;
  if (endAngle > 360) endAngle = endAngle-360.0;
  if (endAngle == 360) endAngle = 0.0; 
  selectDisplay(side);  
  tft.drawArc(120,120, ringRadii[ring]-4, ringRadii[ring]-13, round(startAngle), endAngle, TFT_RED, TFT_RED, true); 
  deselectDisplay(side);
} 
void drawNumber(int side, int number){
  
  String aString = String(number);
  selectDisplay(side);
  tft.drawArc(120,120, 58, 0, 0, 360, TFT_BLACK, TFT_BLACK, true);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(ML_DATUM);
  int a_width = tft.textWidth(aString, middleFontSize);
  tft.drawString(aString, 120-(a_width/2), 120, 7);
  deselectDisplay(side);
}
void drawNote(int side, int note)
{
  Serial.println(note);
  String aString = "##";
  switch(note) 
  {
    case(0):
      aString = String('C');
      break;
    case(1):
      aString = String('C#');
      break;
    case(2):
      aString = "D";
      break;
    case(3):
      aString = "D#";
      break;
    case(4):
      aString = "E";
      break;
    case(5):
      aString = "F";
      break;
    case(6):
      aString = "F#";
      break;
    case(7):
      aString = "G";
      break;
    case(8):
      aString = "G#";
      break;
    case(9):
      aString = "A";
      break;
    case(10):
      aString = "A#";
      break;
    case(11):
      aString = "B";
      break;
  }
  Serial.println(aString);
  selectDisplay(side);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(ML_DATUM);
  //aString = String(12);
  int a_width = tft.textWidth(aString, middleFontSize);
  tft.drawString(aString, 120-(a_width/2), 120, 7);
  deselectDisplay(side);
} 
void checkButtons(){
  if (button2Value) 
  {
    Serial.println("BUTTON2");
    Serial.println(lastButton1Press);
    currEditSide++;
    if (currEditSide > 1) currEditSide = 0;
    currEditStep = 0;
    portENTER_CRITICAL_ISR(&spinlock);
    button2Value = 0;
    portEXIT_CRITICAL_ISR(&spinlock);
  }
  if (button1Value) 
  {
    Serial.println("BUTTON1");
    if(stepRead(currEditSide,currEditRing,currEditStep)) // if the step is set
    { 
      stepClear(currEditSide,currEditRing,currEditStep);   // clear it
      drawSegment(currEditSide, currEditRing, currEditStep, TFT_BLACK);
    }
    else // the step isn't set, so set it
    {
      stepSet(currEditSide,currEditRing,currEditStep);
      drawSegment(currEditSide, currEditRing, currEditStep, ringColors[currEditSide][currEditRing]);
    }
    portENTER_CRITICAL_ISR(&spinlock);
    button1Value = 0;
    portEXIT_CRITICAL_ISR(&spinlock);
  }   
}