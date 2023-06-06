// A Huge shoutout to https://github.com/thaaraak where I got a lot of the software
// for FIR Filter design https://www.arc.id.au/FilterDesign.html but vol is low!!



#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "AiEsp32RotaryEncoder.h"
#include "si5351.h"
#include "Wire.h"
#include "AudioTools.h"
#include "FIRConverter.h"
#include "MixerConverter.h"
#include "coeff.h"

//------------------------------- Audio I2S Setup  ------------------------------//
uint16_t sample_rate=44100;
uint16_t channels = 2;
uint16_t bits_per_sample = 16;
I2SStream in;

FilteredStream<int16_t, float> filtered(in, channels);  // Defiles the filter as BaseConverter
StreamCopy copier(in, filtered, 512);   


//------------------------------- Switches Init ------------------------------//
#define SW_4_PIN 19
#define SW_3_PIN 16
#define SW_2_PIN 4

void setupSwitches(void)
{
  pinMode(SW_2_PIN,INPUT);  
  pinMode(SW_3_PIN,INPUT);  
  pinMode(SW_4_PIN,INPUT);  
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
//uint64_t clk_1_frequency = 100000000; // 100MHz

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


  
// Swap coeffs_60plus45 and  coeffs_60minus45 to change sidebands

  // SSB Filter  
  filtered.setFilter(0, new FilterChain<float, 2>({new FIR<float>(coeffs_60plus45),new FIR<float>(coeffs_lpf_2205Hz_60_44100)}));
  filtered.setFilter(1, new FilterChain<float, 2>({new FIR<float>(coeffs_60minus45),new FIR<float>(coeffs_lpf_2205Hz_60_44100)}));

  // CW Filter
  //filtered.setFilter(0, new FilterChain<float, 2>({new FIR<float>(coeffs_60plus45),new FIR<float>(BPF_CW)}));
  //filtered.setFilter(1, new FilterChain<float, 2>({new FIR<float>(coeffs_60minus45),new FIR<float>(BPF_CW)}));
  
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

  Serial.println("I2S started...");

  setupDisplay();
  setupEncoder();
  setupSi5351();
  
  tft.setTextColor(TFT_BLUE,TFT_WHITE );  
  tft.drawString(" www.RADIOBUILDER.org",5,5,2);
  tft.drawString(" OSC-1 ",5,30,2);

  updateTFT();
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

  copier.copy();
}

