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

void onSW2_Pressed(){
  Serial.println("sw2 Button has been pressed!");  
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
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
void setupDisplay(void)
{
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_WHITE);
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
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK2, Even_Divisor);
 
if(Even_Divisor != oldEven_Divisor)
  {
    si5351.pll_reset(SI5351_PLLA);
    oldEven_Divisor = Even_Divisor;
  }
  
  Serial.print("Even Divisor  ");
  Serial.println(Even_Divisor);
  Serial.print("New Freq  ");
  Serial.println(clk_1_frequency);
  Serial.print("Sending  ");
  pfreq =(clk_1_frequency * SI5351_FREQ_MULT);
  Serial.println(pfreq);
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
  tft.setTextColor(TFT_YELLOW,TFT_BLUE );  
  String freq = String(clk_1_frequency);
  tft.drawString(freq,60,30,2);
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
  
  tft.setTextColor(TFT_BLUE,TFT_WHITE );  
  tft.drawString(" www.RADIOBUILDER.org",5,5,2);
  tft.drawString(" OSC-1 ",5,30,2);

  updateTFT();

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

    clk_1_frequency += change*100;  

    updateTFT();    
    SendFrequency();
  }

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

