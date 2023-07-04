// A Huge shoutout to https://github.com/thaaraak where I got a lot of the software
// for FIR Filter design https://www.arc.id.au/FilterDesign.html but vol is low!!

//Use the internal ADC and DAC
//https://github.com/pschatzmann/arduino-audio-tools/blob/main/examples/examples-stream/streams-adc-i2s/streams-adc-i2s.ino

// For linux "sudo adduser $USER dialout && sudo adduser $USER tty" to make serial port accessable to user

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "AiEsp32RotaryEncoder.h"
#include "si5351.h"
#include "Wire.h"
#include "AudioTools.h"
#include "FIRConverter.h"
#include "MixerConverter.h"
#include "coeff.h"
#include <EEPROM.h>
#include <EasyButton.h>
#include <TFT_eWidget.h>  

TFT_eSPI tft = TFT_eSPI(); 
#define M_SIZE 0.667


void updateTFT(void);
void plotNeedle(int value, byte ms_delay);
void analogMeter();

int eeprom_address = 0;
#define EEPROM_SIZE 8

//------------------------------- Audio I2S Setup  ------------------------------//
uint16_t sample_rate=44100;
uint16_t channels = 2;
uint16_t bits_per_sample = 16;
I2SStream in;


// Doubled up on these to allow dynamic change between CW and SSB ( A bit hacky, but works )
FilteredStream<int16_t, float> filteredSSB(in, channels);  // Defiles the filter as BaseConverter
FilteredStream<int16_t, float> filteredCW(in, channels);  // Defiles the filter as BaseConverter

StreamCopy copierSSB(in, filteredSSB, 512); 
StreamCopy copierCW(in, filteredCW, 512); 

#define SSB_FILTER 0
#define CW_FILTER  1

int filter_select = SSB_FILTER;
  
//------------------------------- Switches Init ------------------------------//
#define SW_4_PIN 19
#define SW_3_PIN 16
#define SW_2_PIN 4

EasyButton sw4_button(SW_4_PIN); 
EasyButton sw3_button(SW_3_PIN); 
EasyButton sw2_button(SW_2_PIN); 

bool saveFrequency = false;

void onSW4_Pressed() {
  Serial.println("sw4 Button has been pressed!");
  saveFrequency = true;
}

void onSW3_Pressed() {
  Serial.println("sw3 Button has been pressed!");
  if(filter_select == SSB_FILTER)
    filter_select = CW_FILTER;
  else
    filter_select = SSB_FILTER;    
}

uint64_t frequencyChange = 100;

void onSW2_Pressed(){

     
  if(frequencyChange==10)
    frequencyChange=100;

  else if(frequencyChange==100)
    frequencyChange=1000;

  else if(frequencyChange==1000)
    frequencyChange=10000;

  else if(frequencyChange==1000)
    frequencyChange=10000;

  else if(frequencyChange==10000)
    frequencyChange=100000;

  else if(frequencyChange==100000)
    frequencyChange=1000000;
    
  else if(frequencyChange==1000000)
    frequencyChange=10;

  Serial.println("sw2 Button has been pressed! : "+String(frequencyChange));  
  
  updateTFT();
    //tft.drawString(" OSC-1 ",5,30,2);  
}

void setupSwitches(void)
{ 
  sw2_button.begin();
  sw2_button.onPressed(onSW2_Pressed); 
  
  sw3_button.begin();
  sw3_button.onPressed(onSW3_Pressed);  

  sw4_button.begin();
  sw4_button.onPressed(onSW4_Pressed);  
}


//------------------------------- TFT Display Init ------------------------------//
//TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
//MeterWidget   FQ  = MeterWidget(&tft);
void setupDisplay(void)
{
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_WHITE);

//FQ.setZones(0, 100, 25, 75, 0, 0, 40, 60);
  //FQ.analogMeter(0, 128, 10.0, "V", "0", "2.5", "5", "7.5", "10"); // Draw analogue meter at 0, 128
  
//  FQ.analogMeter(0, 256, 100, "R", "0", "", "50", "", "100"); // Draw analogue meter at 0, 128
  //FQ.updateNeedle(100, 0);

  analogMeter();
}

//------------------------------- Encoder Init ------------------------------//
#define ROTARY_ENCODER_A_PIN 34
#define ROTARY_ENCODER_B_PIN 35
#define ROTARY_ENCODER_BUTTON_PIN 32
#define ROTARY_ENCODER_STEPS 4
#define ROTARY_ENCODER_VCC_PIN -1
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
long lastEncoderReading = 0;

void IRAM_ATTR readEncoderISR()
{
	rotaryEncoder.readEncoder_ISR();
}

void setupEncoder(void)
{
	rotaryEncoder.begin();
	rotaryEncoder.setup(readEncoderISR);
	rotaryEncoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
}

//------------------------------- Si5351 Init ------------------------------//
Si5351 si5351(0x60);
uint64_t clk_1_frequency = 7000000; // 7MHz
//uint64_t clk_1_frequency =    198000; // 100MHz

volatile int Even_Divisor = 0;
volatile int oldEven_Divisor = 0;
unsigned long pfreq;
//uint64_t frequencyChange = 100;

void EvenDivisor()
{
    if (clk_1_frequency < 6850000)
  {
    Even_Divisor = 126;
  }
  if ((clk_1_frequency >= 6850000) && (clk_1_frequency < 9500000))
  {
    Even_Divisor = 88;
  }
  if ((clk_1_frequency >= 9500000) && (clk_1_frequency < 13600000))
  {
    Even_Divisor = 64;
  }
  if ((clk_1_frequency>= 13600000) && (clk_1_frequency < 17500000))
  {
    Even_Divisor = 44;
  }
  if ((clk_1_frequency >= 17500000) && (clk_1_frequency < 25000000))
  {
    Even_Divisor = 34;
  }
  if ((clk_1_frequency >= 25000000) && (clk_1_frequency < 36000000))
  {
    Even_Divisor = 24;
  }
  if ((clk_1_frequency >= 36000000) && (clk_1_frequency < 45000000)) {
    Even_Divisor = 18;
  }
  if ((clk_1_frequency >= 45000000) && (clk_1_frequency < 60000000)) {
    Even_Divisor = 14;
  }
  if ((clk_1_frequency >= 60000000) && (clk_1_frequency < 80000000)) {
    Even_Divisor = 10;
  }
  if ((clk_1_frequency >= 80000000) && (clk_1_frequency < 100000000)) {
    Even_Divisor = 8;
  }
  if ((clk_1_frequency >= 100000000) && (clk_1_frequency < 146600000)) {
    Even_Divisor = 6;
  }
  if ((clk_1_frequency >= 150000000) && (clk_1_frequency < 220000000)) {
    Even_Divisor = 4;
  }
}


void SendFrequency()
{
  EvenDivisor();

  si5351.set_freq_manual(clk_1_frequency * SI5351_FREQ_MULT, Even_Divisor * clk_1_frequency * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.set_freq_manual(clk_1_frequency * SI5351_FREQ_MULT, Even_Divisor * clk_1_frequency * SI5351_FREQ_MULT, SI5351_CLK2);
  si5351.set_freq_manual((clk_1_frequency+700) * SI5351_FREQ_MULT, Even_Divisor * (clk_1_frequency+700) * SI5351_FREQ_MULT, SI5351_CLK1);

  
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, 0);  
  si5351.set_phase(SI5351_CLK2, Even_Divisor);
 
if(Even_Divisor != oldEven_Divisor)
  {
    si5351.pll_reset(SI5351_PLLA);
    oldEven_Divisor = Even_Divisor;
  }
  
  // Serial.print("Even Divisor  ");
  // Serial.println(Even_Divisor);
  // Serial.print("New Freq  ");
  // Serial.println(clk_1_frequency);
  // Serial.print("Sending  ");
  pfreq =(clk_1_frequency * SI5351_FREQ_MULT);
  //Serial.println(pfreq);
}

void setupSi5351()
{
// Start serial and initialize the Si5351
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  {
    Serial.println("Device not found on I2C bus!");
  }
  else
    Serial.println("Device found on I2C bus!");  

  SendFrequency();
}

void updateTFT()
{
 // tft.setTextColor(TFT_YELLOW,TFT_BLUE );  
 // String freq = String(clk_1_frequency);
 // tft.drawString(freq,60,90,2);

  String freqChange; // = "f Delta : " + String(frequencyChange);
  //tft.drawString(freqChange,10,50,2);

if(frequencyChange==10)
    freqChange = "_____^___";    
  else if(frequencyChange==100)
    freqChange = "____^____";    
  else if(frequencyChange==1000)
    freqChange = "___^_____";    
  else if(frequencyChange==10000)
    freqChange = "__^______";    
  else if(frequencyChange==100000)
    freqChange = "_^_______";    
  else if(frequencyChange==1000000)
    freqChange = "^________";    
  
 // tft.drawString(freqChange,60,50,2);
    //FQ.updateNeedle(100, 0);
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);
 
  setupSwitches();
 
// Swap coeffs plus45 and  coeffs minus45 to change sidebands

  filteredSSB.setFilter(0, new FIR<float>(SSB_plus45_161_filter));   
  filteredSSB.setFilter(1, new FIR<float>(SSB_minus45_161_filter));

  filteredCW.setFilter(0, new FIR<float>(CW_plus45_261_filter));
  filteredCW.setFilter(1, new FIR<float>(CW_minus45_261_filter));
      
  
  // start I2S in
  auto config = in.defaultConfig(RXTX_MODE);
  config.sample_rate = sample_rate;
  config.bits_per_sample = 16;
  config.i2s_format = I2S_STD_FORMAT;
  config.is_master = true;
  config.port_no = 0;
  config.pin_ws = 33;
  config.pin_bck = 5;
  config.pin_data = 25;
  config.pin_data_rx = 26;
  config.pin_mck = 0;
  config.use_apll = true;
  
  in.begin(config);


  EEPROM.begin(EEPROM_SIZE);
  Serial.println("I2S started...");

  setupDisplay();
  setupEncoder();
  clk_1_frequency = EEPROM.readLong64(0);
  setupSi5351();
  
 // tft.setTextColor(TFT_BLUE,TFT_WHITE );  
 // tft.drawString(" www.RADIOBUILDER.org",5,5,2);
 // tft.drawString(" OSC-1 ",5,30,2);

 // updateTFT();

 long unsigned int value =  ((clk_1_frequency -7000000)*100)/200000;   
    plotNeedle((int)value, 0);
    tft.drawCentreString(String(clk_1_frequency), M_SIZE*120, M_SIZE*75, 2); // Comment out to avoid font 4

  uint64_t read_eeprom = EEPROM.readLong64(0);
  Serial.println("EEProm Read" + String((uint64_t)read_eeprom));
   
}



void loop()
{ 
 	if (rotaryEncoder.encoderChanged())
	{
    long currentReading = rotaryEncoder.readEncoder();
    long change =  currentReading -lastEncoderReading;

    lastEncoderReading = currentReading;

    clk_1_frequency += change*frequencyChange;  

    //updateTFT();    
    SendFrequency();

    long unsigned int value =  ((clk_1_frequency -7000000)*100)/200000;   
    plotNeedle((int)value, 0);
    tft.drawCentreString(String(clk_1_frequency), M_SIZE*120, M_SIZE*75, 2); // Comment out to avoid font 4

    Serial.println((int)value);

  }

  

  //if (!rotaryEncoder.isEncoderButtonClicked())
  //  Serial.println("Rotary encoder presses ");
  

  if(filter_select == SSB_FILTER)
    copierSSB.copy();
  else
    copierCW.copy();    

  sw2_button.read();
  sw3_button.read();
  sw4_button.read();
           
  if(saveFrequency==true){
      size_t write_eeprom = EEPROM.writeULong64(0, clk_1_frequency);
      EEPROM.commit();
      Serial.println("EEprom Write" + String((uint64_t)write_eeprom));
      saveFrequency=false;
  }

  

  
}







#define TFT_GREY 0x5AEB
#define TFT_ORANGE      0xFD20      /* 255, 165,   0 */
#define M_SIZE 0.667

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = M_SIZE*120, osy = M_SIZE*120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update

int old_analog =  -999; // Value last displayed

int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;
// #########################################################################
//  Draw the analogue meter on the screen
// #########################################################################
void analogMeter()
{

  // Meter outline
  tft.fillRect(0, 0, M_SIZE*239, M_SIZE*131, TFT_GREY);
  tft.fillRect(1, M_SIZE*3, M_SIZE*234, M_SIZE*125, TFT_WHITE);

  tft.setTextColor(TFT_BLACK);  // Text colour

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
    uint16_t y0 = sy * (M_SIZE*100 + tl) + M_SIZE*150;
    uint16_t x1 = sx * M_SIZE*100 + M_SIZE*120;
    uint16_t y1 = sy * M_SIZE*100 + M_SIZE*150;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (M_SIZE*100 + tl) + M_SIZE*120;
    int y2 = sy2 * (M_SIZE*100 + tl) + M_SIZE*150;
    int x3 = sx2 * M_SIZE*100 + M_SIZE*120;
    int y3 = sy2 * M_SIZE*100 + M_SIZE*150;

    //Yellow zone limits
    if (i >= -50 && i < -30) {
     tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
     tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    }

    // // Green zone limits
    // if (i >= 0 && i < 25) {
    //   tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
    //   tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
    // }

    // // Orange zone limits
    // if (i >= 25 && i < 50) {
    //   tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
    //   tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
    // }

    // Short scale tick length
    if (i % 25 != 0) tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
    y0 = sy * (M_SIZE*100 + tl) + M_SIZE*150;
    x1 = sx * M_SIZE*100 + M_SIZE*120;
    y1 = sy * M_SIZE*100 + M_SIZE*150;

    // Draw tick
    tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

    // Check if labels should be drawn, with position tweaks
    if (i % 25 == 0) {
      // Calculate label positions
      x0 = sx * (M_SIZE*100 + tl + 10) + M_SIZE*120;
      y0 = sy * (M_SIZE*100 + tl + 10) + M_SIZE*150;
      switch (i / 25) {
        case -2: tft.drawCentreString("7.00", x0+4, y0-4, 1); break;
        //case -1: tft.drawCentreString("25", x0+2, y0, 1); break;
        case 0: tft.drawCentreString("7.10", x0, y0, 1); break;
        //case 1: tft.drawCentreString("75", x0, y0, 1); break;
        case 2: tft.drawCentreString("7.2", x0-2, y0-4, 1); break;
      }
    }

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * M_SIZE*100 + M_SIZE*120;
    y0 = sy * M_SIZE*100 + M_SIZE*150;
    // Draw scale arc, don't draw the last part
    if (i < 50) tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }

 // tft.drawString("%RH", M_SIZE*(3 + 230 - 40), M_SIZE*(119 - 20), 2); // Units at bottom right
  //tft.drawCentreString("%RH", M_SIZE*120, M_SIZE*75, 4); // Comment out to avoid font 4
  tft.drawRect(1, M_SIZE*3, M_SIZE*236, M_SIZE*126, TFT_BLACK); // Draw bezel line

  plotNeedle(0, 0); // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotNeedle(int value, byte ms_delay)
{
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  //char buf[8]; dtostrf(value, 4, 0, buf);
  //tft.drawRightString(buf, 33, M_SIZE*(119 - 20), 2);

  if (value < -10) value = -10; // Limit value to emulate needle end stops
  if (value > 110) value = 110;

  // Move the needle until new value reached
  while (!(value == old_analog)) {
    if (old_analog < value) old_analog++;
    else old_analog--;

    if (ms_delay == 0) old_analog = value; // Update immediately if delay is 0

    float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
    // Calculate tip of needle coords
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((sdeg + 90) * 0.0174532925);

    // Erase old needle image
    tft.drawLine(M_SIZE*(120 + 24 * ltx) - 1, M_SIZE*(150 - 24), osx - 1, osy, TFT_WHITE);
    tft.drawLine(M_SIZE*(120 + 24 * ltx), M_SIZE*(150 - 24), osx, osy, TFT_WHITE);
    tft.drawLine(M_SIZE*(120 + 24 * ltx) + 1, M_SIZE*(150 - 24), osx + 1, osy, TFT_WHITE);

    // Re-plot text under needle
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    //tft.drawCentreString("%RH", M_SIZE*120, M_SIZE*75, 4); // // Comment out to avoid font 4

    // Store new needle end coords for next erase
    ltx = tx;
    osx = M_SIZE*(sx * 98 + 120);
    osy = M_SIZE*(sy * 98 + 150);

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
    tft.drawLine(M_SIZE*(120 + 24 * ltx) - 1, M_SIZE*(150 - 24), osx - 1, osy, TFT_RED);
    tft.drawLine(M_SIZE*(120 + 24 * ltx), M_SIZE*(150 - 24), osx, osy, TFT_MAGENTA);
    tft.drawLine(M_SIZE*(120 + 24 * ltx) + 1, M_SIZE*(150 - 24), osx + 1, osy, TFT_RED);

    // Slow needle down slightly as it approaches new postion
    if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

    // Wait before next update
    delay(ms_delay);
  }
}