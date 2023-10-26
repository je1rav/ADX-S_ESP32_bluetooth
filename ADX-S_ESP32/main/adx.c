//********************* ADX-S - ARDUINO DIGITAL MODES HF TRANSCEIVER FUNCTIONS USING ESP32 **************************
//                       Slightly modified to use ESP32 with bluetooth audio (HFP) by Hitoshi Kawaji (JE1RAV) <je1rav@gmail.com>
//                       The setup() function is changed into adx_setup() and called from app_main().
//                       The loop() function is not used.
//                       NT7S "si5351" Library for avr-gcc ”https://github.com/NT7S/Si5351" is used with some modifications.
//                       Data processing part of ADC to bluetooth HFP was written by modifying "microphone.c" in atomic14's esp32-hsp-hf (https://github.com/atomic14/esp32-hsp-hf).
//                       CAT has been not supported yet.
// based on ADX_S_V1.3 by BD6CR.
//*********************************************************************************************************
//********************* ADX-S - ARDUINO DIGITAL MODES 4 BAND HF TRANSCEIVER ***************************
//********************************* Write up start: 02/01/2022 ********************************************
// FW VERSION: ADX_S_V1.3 - Version release date: 08/08/2023
// Barb(Barbaros ASUROGLU) - WB2CBA - 2022
//  Version History
//  Based on ADX-Quad V1.2 Release:
//  -  Modified Calibration code to protect R/W cycle of EEPROM_
//  - 10m/28Mhz band support added.
//  ADX-S V1.3 - 20230808: Add 12m frequency table and 7 band (40,30,20,17,15,12,10) support
//  ADX-S V1.2RX - 20230503: Support RX modification
//  ADX-S V1.2 - 20230419: Ported CAT code based on ADX firmware. Cross Band Check is added before TX to protect the final.
//  ADX-S V1.1 - 20230319: Special measures are taken to avoid WWV 15MHz interference to 14MHz FT8.
//  ADX-S V1.0 - 20230304: modified by Adam Rong, BD6CR E-mail: rongxh@gmail.com
//    1. FSK code bug fix by JE1RAV https://github.com/je1rav/QP-7C, now lower and higher audio frequency will be better supported
//    2. RX modified from direct conversion to superheterodyne based on JA9TTT's blog https://ja9ttt.blogspot.com/2018/07/short-wave-radio-design-2.html . 
//    The modification will significantly reduce BCI and improve RX performance.
//    Hardware modifications required or contact BD6CR for a new PCB design or a whole kit: 
//      a. Cut two PCB traces between CD2003 pin 4 (AM MIX) and 0.47uF capacitor or 1uF in ADX UNO for audio output, and between CD2003 pin 7 (AM IF) and 5V trace.
//      b. Add a 5.1pF ceramic capacitor (C25) between SI5351 CLK2 and CD2003 pin 4 (AM MIX), and use an extension wire if required.
//      c. Add a 455kHz ceramic filter MURATA PFB455JR or equivalent (FL1) to CD2003 pin 4 (AM MIX), 6 (VCC) and 7 (AM IF), same as CD2003 reference circuit.    
//      d. Add a wire between CD2003 pin 11 (DET OUT) and 0.47uF capacitor or 1uF in ADX UNO for audio output.
//    3. Add 1kHz tone to manual TX (freq + 1000) for monitoring at the carrier frequency.
//    4. Changed the way to switch band by pressing and holding SW1 and power on.
//*********************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//*****************************************************************************************************
//* IMPORTANT NOTE: Use V2.1.3 of NT7S SI5351 Library. This is the only version compatible with ADX!!!*
//*****************************************************************************************************
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM_h" EEPROM Library(built-into arduino ide)
//*************************************[ LICENCE and CREDITS ]*********************************************
//  FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino

// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//*****************[ SI5351 VFO CALIBRATION PROCEDURE ]****************************************
// For SI5351 VFO Calibration Procedure follow these steps:
// 1 - Connect CAL test point and GND test point on ADX PCB to a Frequency meter or Scope that can measure 1 Mhz up to 1Hz accurately.
// 2 - Press SW2 / --->(CAL) pushbutton and hold.
// 4-  Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton. 
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL). Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope. 
//     The waveform is Square wave so freqency calculation can be performed esaily.
// 8 - If you read as accurate as possible 1000000 Hz then calibration is done. 
// 9 - Now we must save this calibration value to EEPROM location. 
//     In order to save calibration value, press TX button briefly. TX LED will flash 3 times which indicates that Calibration value is saved.
// 10- Power off ADX.

//*******************************[ Libralies added for ESP32 (esp-idf) ]*******************************************
#include "adx.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_timer.h"
#include <esp32/rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "btstack_config.h"
#include "btstack_audio.h"
#include "sco_demo_util.h"

//*******************************[ LIBRARIES ]*************************************************
#include "si5351.h"
//#include "Wire.h"     //ESP32
//#include <EEPROM_h>      //ESP32
//*******************************[ VARIABLE DECLERATIONS ]*************************************
uint32_t val;
int temp;
uint32_t val_EE; 
int addr = 0;
int mode;
unsigned long freq; 
unsigned long ifreq;    //IF frquency, default to 464570Hz or 447430Hz for the MURATA PFB455JR ceramic filter
int32_t cal_factor;
int TX_State = 0;
int bfo = 1;

//------------ CAT Variables
int cat_stat = 0;
int CAT_mode = 2;   
//String received;
//String receivedPart1;
//String receivedPart2;    
//String command;
//String command2;  
//String parameter;
//String parameter2; 
//String sent;
//String sent2;

int TxStatus = 0; //0 =  RX 1=TX
int attnow = 0; //0 = no ATT, 1 = ATT

unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
unsigned long F_FT8;
unsigned long F_FT4; 
unsigned long F_JS8;
unsigned long F_WSPR;
//**********************************[ BAND SELECT ]************************************************
// ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4 from supported 8 bands.
// To change bands press and hold SW1 and power on. Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
// that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When desired band bank is selected press TX button briefly to exit band select mode. 
// Now the new selected band bank will flash 3 times and then stored mode LED will be lit. 
// TX won't activate when changing bands so don't worry on pressing TX button when changing bands in band mode.
// Assign your prefered bands to B1,B2,B3 and B4
// Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m, 12, 10m
unsigned int Band_slot = 0;  //0,1,2,3,4,5,6
unsigned int Band[7] = {40,30,20,17,15,12,10};      //B1, 2, 3, 4, 5, 6, 7 = 40,30,20,17,15,12,10

// **********************************[ DEFINE's ]***********************************************
// GPIO pin difinition is written in adx.h.
// Si5351 relating difinition is written in si5351.h.

// ******************************************************************************************
// ******************* button input task***************************************************
static void gpio_botton_task(void* arg)
{
	for(;;) {  
		if (keypressed(UP) && keypressed(DOWN)) {
			attnow = !attnow;
			digitalWrite(ATT,attnow);
			att2led(attnow);            //ESP32; Indicate ATT status change 
		}
		if (keypressed(UP)&&(TX_State == 0)) {
			if (cat_stat == 0) {
				mode = mode+3;
				mode = mode % 4;
				addr = 40;
				//EEPROM.put(addr, mode);            //ESP32
				EEPROM_put(addr);             //ESP32
			}
			else freq = freq - 5000;  //manual tuning for BC
		}

		if (keypressed(DOWN)&&(TX_State == 0)) {
			if (cat_stat == 0) {
				mode++;
				mode = mode % 4;
				addr = 40;
				//EEPROM.put(addr, mode);            //ESP32
				EEPROM_put(addr);             //ESP32
			}
			else freq = freq + 5000;  //manual tuning for BC
		}
	
		if (cat_stat == 0)   Mode_assign();

		if (Band[Band_slot] == 20) {
			si5351_set_freq((freq-ifreq), SI5351_PLL_FIXED, SI5351_CLK1);    //special cure to avoid WWV 15MHz interference, thanks VK3YE for feedback
		}
		else {
			si5351_set_freq((freq+ifreq), SI5351_PLL_FIXED, SI5351_CLK1);    //Set RX LO frequency, added by BD6CR
		}
	
		if (bfo == 1) {
			si5351_set_freq(ifreq, SI5351_PLL_FIXED, SI5351_CLK2);   //RX IF frequency = ifreq, added by BD6CR
		}
		
		if (keypressed(TXSW)) {
			if (cat_stat == 0) {
				ManualTX();
			} 
			else {
				if (bfo == 1) {
					bfo = 0;
					Freq_assign();
					si5351_clock_enable(SI5351_CLK2, 0);
				}
				else {
					bfo = 1;
					Freq_assign();
				}
			}
		}
	
		if (bfo == 1) {
			si5351_clock_enable(SI5351_CLK2, 1);   //RX IF on as well, added by BD6CR
		}
        delay(1);
    }
}

//*************************************[ SETUP FUNCTION ]************************************** 
//void setup()          //ESP32
void adx_setup()          //ESP32
{
	//-------------------GPIO from here--------------------------
	//pinMode(UP, INPUT);
	//pinMode(DOWN, INPUT);
	//pinMode(TXSW, INPUT);
	//pinMode(ATT, OUTPUT);  
	//pinMode(TX, OUTPUT);
	//pinMode(WSPR, OUTPUT);
	//pinMode(JS8, OUTPUT);
	//pinMode(FT4, OUTPUT);
	//pinMode(FT8, OUTPUT);
	//pinMode(RX, OUTPUT);
	//digitalWrite(RX,HIGH);    //enable RX and disable TX first
	//digitalWrite(ATT,attnow);    //default no ATT.

    //Setup output pins
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    //Setup input pins
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    digitalWrite(RX,HIGH);    //enable RX and disable TX first
	digitalWrite(TX,LOW);    //enable RX and disable TX first
	digitalWrite(ATT, attnow);    //default no ATT.
	
    //-------------------CAT from here--------------------------
	//SET UP SERIAL FOR CAT CONTROL          //ESP32
	//Serial.begin(115200);           //ESP32
	//Serial.setTimeout(1);           //ESP32
	
	//-------------------EEPROM from here--------------------------
	addr = 30;
	//EEPROM.get(addr,temp);          //ESP32
	EEPROM_get(addr);          //ESP32
	if (temp != 100) {
		addr = 10; 
		//cal_factor = 100000;           //ESP32
		cal_factor = 1500;           //ESP32
		//EEPROM.put(addr, cal_factor);           //ESP32
		EEPROM_put(addr);           //ESP32

		addr = 40;
		//temp = 4;          //ESP32
		//EEPROM.put(addr,temp);          //ESP32
		mode = 4;          //ESP32
		EEPROM_put(addr);          //ESP32
    
		addr = 30;
		temp = 100;
		//EEPROM.put(addr,temp);          //ESP32
  		EEPROM_put(addr);          //ESP32

  		addr = 50;
  		//temp = 1;
  		//EEPROM.put(addr,temp);          //ESP32
    	Band_slot = 1;
    	EEPROM_put(addr);          //ESP32
	}
	else {
		// reaad INIT VALUES from EEPROM
		addr = 30;
		//EEPROM.get(addr,temp);          //ESP32
		EEPROM_get(addr);          //ESP32
		addr = 10;
		//EEPROM.get(addr,cal_factor);          //ESP32
		EEPROM_get(addr);          //ESP32
		addr = 40;
		//EEPROM.get(addr,mode);          //ESP32
		EEPROM_get(addr);          //ESP32
		mode = mode % 4;
		addr = 50;
		//EEPROM.get(addr,Band_slot);          //ESP32
		EEPROM_get(addr);          //ESP32
		Band_slot = Band_slot % 7;
	}
	
  Band_assign();
  //si5351_clock_enable(SI5351_CLK2, 0); // Turn off Calibration Clock

  esp_err_t err_i2c = i2c_master_init();

  //------------------------------- SET SI5351 VFO -----------------------------------  
  // The crystal load value needs to match in order to have an accurate calibration
  //si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);
  si5351_init();
  si5351_set_correction(cal_factor);
  si5351_set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX 
  si5351_drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for max power for RX IF, added by BD6CR

  if ( digitalRead(DOWN) == LOW ) {  //You must reset the system to exit calibration
    Calibration();
  }

  if (digitalRead(UP) == LOW ) {     //You can go on after band select, if things are no problem.
    Band_Select();
  }
  
  /*
  if (digitalRead(TXSW) == LOW ) {     //You can go on after band select, if things are no problem.
    bfo = 0;
    ifreq = 455000UL;
  }
*/

  //TCCR1A = 0x00;          //ESP32
  //TCCR1B = 0x01; // Timer1 Timer 16 MHz          //ESP32
  //TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller          //ESP32
  //ACSR |= (1<<ACIC);  // Analog Comparator Capture Input          //ESP32
  ////pinMode(FSK_input, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0          //ESP32
  
  digitalWrite(RX,LOW);
  Mode_assign(); 
	
	if (Band[Band_slot] == 20) {
		si5351_set_freq((freq-ifreq), SI5351_PLL_FIXED, SI5351_CLK1);    //special cure to avoid WWV 15MHz interference, thanks VK3YE for feedback
	}
	else {
		si5351_set_freq((freq+ifreq), SI5351_PLL_FIXED, SI5351_CLK1);    //Set RX LO frequency, added by BD6CR
	}
	if (bfo == 1) {
		si5351_set_freq(ifreq, SI5351_PLL_FIXED, SI5351_CLK2);   //RX IF frequency = ifreq, added by BD6CR
	}
	
	//si5351_clock_enable(SI5351_CLK0, 0);   // TX off
	//si5351_clock_enable(SI5351_CLK1, 1);   //RX local osc. on
	//si5351_clock_enable(SI5351_CLK2, 1);   //RX BFO on
	adx_force_receive();   // RX start

    //start gpio botton task
    xTaskCreate(gpio_botton_task, "gpio_button_task", 2048, NULL, 10, NULL);
    
    //determine ADC offset value
    adc_offset_value = adc_offset();
}
// **************************[ END OF SETUP FUNCTION ]************************

/*
// ***************************[ Main LOOP Function ]**************************
void loop()          
{  
// Key input part was moved to "static void gpio_botton_task(void* arg)"

    // CAT CHECK SERIAL FOR NEW DATA FROM WSJT-X AND RESPOND TO QUERIES
 
	if (Serial.available() > 0)
	{
		cat_stat = 1; 
        si5351_set_freq(freq, SI5351_PLL_FIXED, SI5351_CLK1);
        si5351_clock_enable(SI5351_CLK1, 1);   //RX on
		digitalWrite(WSPR, LOW); 
		digitalWrite(JS8, HIGH); 
		digitalWrite(FT4, HIGH); 
		digitalWrite(FT8, LOW); 
                     
		CAT_control();  
	}
	
// ***************digital modulation routine was changed for ESP32 with bluetooth audio*********************

}
// *********************[ END OF MAIN LOOP FUNCTION ]*************************
*/

// ********************************************************************************
// ******************************** [ FUNCTIONS ] *********************************
// ********************************************************************************
/*
void CAT_control(void)
{     
    received = Serial.readString();
    received.toUpperCase();  
    received.replace("\n","");  

           String data = "";
           int bufferIndex = 0;

          for (int i = 0; i < received.length(); ++i)
               {
               char c = received[i];
    
                 if (c != ';')
                    {
                    data += c;
                    }
                 else
                    {
                        if (bufferIndex == 0)
                          {  
                              data += '\0';
                              receivedPart1 = data;
                              bufferIndex++;
                              data = "";
                          }
                         else
                          {  
                              data += '\0';
                              receivedPart2 = data;
                              bufferIndex++;
                              data = "";
                          }
                    }

               }
    
    command = receivedPart1.substring(0,2);
    command2 = receivedPart2.substring(0,2);    
    parameter = receivedPart1.substring(2,receivedPart1.length());
    parameter2 = receivedPart2.substring(2,receivedPart2.length());

    if (command == "FA")  
    {  
        
          if (parameter != "")  
              {  
              freq = parameter.toInt();
              //VfoRx = VfoTx;   
              }
          
          sent = "FA" // Return 11 digit frequency in Hz.  
          + String("00000000000").substring(0,11-(String(freq).length()))   
          + String(freq) + ";";     
    }

    else if (command == "PS")  
        {  
        sent = "PS1;";
        }

    else if (command == "TX")  
        {   
        sent = "TX0;";
        //VfoTx = freq;
        //digitalWrite(13,1);             //ESP32
        transmit(0);             //ESP32
       
        }

    else if (command == "RX")  
        {  
        sent = "RX0;";
        //VfoRx = freq;
        //digitalWrite(13,0);             //ESP32
        receive();             //ESP32
        }

    else  if (command == "ID")  
        {  
        sent = "ID019;";
        }

    else if (command == "AI")  
        {
        sent = "AI0;"; 
        }

    else if (command == "IF")  
        {
          if (TX_State == 1)
            {  
            sent = "IF" // Return 11 digit frequency in Hz.  
            + String("00000000000").substring(0,11-(String(freq).length()))   
            //+ String(freq) + String("     ") + "+" + "0000" + "0" + "0" + "0" + "00" + "0" + String(CAT_mode) + "0" + "0" + "0" + "0" + "00" + String(" ") + ";"; 
            + String(freq) + "00000" + "+" + "0000" + "0" + "0" + "0" + "00" + "1" + String(CAT_mode) + "0" + "0" + "0" + "0" + "000" + ";"; 
            } 
             else
            {  
            sent = "IF" // Return 11 digit frequency in Hz.  
            + String("00000000000").substring(0,11-(String(freq).length()))   
            //+ String(freq) + String("     ") + "+" + "0000" + "0" + "0" + "0" + "00" + "0" + String(CAT_mode) + "0" + "0" + "0" + "0" + "00" + String(" ") + ";"; 
            + String(freq) + "00000" + "+" + "0000" + "0" + "0" + "0" + "00" + "0" + String(CAT_mode) + "0" + "0" + "0" + "0" + "000" + ";"; 
            } 
       }
  
    else if (command == "MD")  
      {  
      sent = "MD2;";
      }

//------------------------------------------------------------------------------      

    if (command2 == "ID")  
        {  
        sent2 = "ID019;";
        }
            
    if (bufferIndex == 2)
        {
        Serial.print(sent2); 
        }
        
    else
    {
        Serial.print(sent);
    }  

   if ((command == "RX") or (command = "TX")) delay(50);
    sent = String("");
    sent2 = String("");  
}
*/

//************************************[ Cross Band Check ]**********************************
unsigned int crossbandcheck()
{
  unsigned int bandnow = 0;
  switch (freq / 1000000) {
    case 3:
      bandnow = 80;
      break;
    case 7:
      bandnow = 40;
      break;
    case 10:
      bandnow = 30;
      break;
    case 14:
      bandnow = 20;
      break;
    case 18:
      bandnow = 17;
      break;
    case 21:
      bandnow = 15;
      break;
    case 24:
      bandnow = 12;
      break;
    case 28:
      bandnow = 10;
      break;
    default:
      bandnow = 0;
      break;
  }
  
  if ((Band[Band_slot] == bandnow) && freqcheck(freq))
    return (1);
  else
    return (0);
} 

//******************************[ Frequency Check ]*****************************
//checking for the prevention of out-of-band transmission
int freqcheck(unsigned long frequency)  // retern 0=out-of-band, 1=in-band
{
  if (frequency < 7000000) {
    return 0;
  }
  else if (frequency > 7200000 && frequency < 10100000) {
    return 0;
  }
  else if (frequency > 10150000 && frequency < 14000000) {
    return 0;
  }
  else if (frequency > 14350000 && frequency < 18068000) {
    return 0;
  }
  else if (frequency > 18168000 && frequency < 21000000) {
    return 0;
  }
  else if (frequency > 21450000 && frequency < 24890000) {
    return 0;
  }
  else if (frequency > 24990000 && frequency < 28000000) {
    return 0;
  }
  else if (frequency > 29700000 && frequency < 50000000) {
    return 0;
  }
  else if (frequency > 54000000) {
    return 0;
  }
  else return 1;
}

//******************************[ Band Slot to LED mapping ]*****************************
void band2led(int band_slot_number) 
{
  switch (band_slot_number) {
    case 0:
    digitalWrite(WSPR, HIGH);
    break;
    case 1:
    digitalWrite(WSPR, HIGH);
    digitalWrite(JS8, HIGH);
    break;    
    case 2:
    digitalWrite(JS8, HIGH);
    break;
    case 3:
    digitalWrite(JS8, HIGH); 
    digitalWrite(FT4, HIGH);
    break;  
    case 4:
    digitalWrite(FT4, HIGH);
    break;
    case 5:
    digitalWrite(FT4, HIGH);
    digitalWrite(FT8, HIGH);
    break;  
    case 6:
    digitalWrite(FT8, HIGH);
    break;
  }
}

//************************************[ Blink 3 Times ]**********************************
void blink3times(int band_slot_num) 
{
  for (int i=0;i<3;i++) {
    band2led(band_slot_num);
    delay(250);
    ledalloff();
    delay(250);
  }
}

//************************************[ 4 LEDs all off ]**********************************
void ledalloff() 
{
  digitalWrite(WSPR, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(FT8, LOW);
}


//*************************[ Check if specific key is pressed ]**************************
int keypressed(int key_number)
{
   if (!digitalRead(key_number)) {
    delay(70); 
    if (!digitalRead(key_number)) {
      return (1);
    }
  return (0);
}
return (0);
}


//************************************[ MODE Assign ]**********************************
void Mode_assign() 
{
  addr = 40;
  //EEPROM.get(addr,mode);          //ESP32
  EEPROM_get(addr);          //ESP32
  mode = mode % 4;

  if ( mode == 0){
    freq = F_WSPR;
  }
  else if ( mode == 1){
    freq = F_JS8;
  }
  else if ( mode == 2){
    freq = F_FT4;
  }
  else if ( mode == 3){
    freq = F_FT8;
  }
  if (cat_stat == 0) {
    ledalloff();
    //digitalWrite(WSPR+mode,HIGH);         //ESP32
    mode2led(mode);         //ESP32
	}
}

//*********************[ Band dependent Frequency Assign Function ]********************
void Freq_assign() 
{
  Band_slot = Band_slot % 7;

//---------- 80m/3.5Mhz 
         if (Band[Band_slot] == 80){

            F_FT8 = 3573000;
            F_FT4 = 3575000;
            F_JS8 = 3578000;
            F_WSPR = 3568600;
            ifreq = 460570UL;
          }

//---------- 40m/7 Mhz 
          if (Band[Band_slot] == 40){

            F_FT8 = 7074000;
            F_FT4 = 7047500;
            F_JS8 = 7041000;       //for JA
            //F_JS8 = 7078000;
            F_WSPR = 7038600;
            ifreq = 460570UL;
          }


//---------- 30m/10 Mhz 
          if (Band[Band_slot] == 30){

            F_FT8 = 10136000;
            F_FT4 = 10140000;
            F_JS8 = 10130000;
            F_WSPR = 10138700;
            ifreq = 460570UL;
          }


//---------- 20m/14 Mhz 
          if (Band[Band_slot] == 20){

            F_FT8 = 14074000;
            F_FT4 = 14080000;
            F_JS8 = 14078000;
            F_WSPR = 14095600;
            ifreq = 449430UL; //special cure for WWV 15MHz interference, thanks VK3YE for feedback
          }


 //---------- 17m/18 Mhz 
          if (Band[Band_slot] == 17){

            F_FT8 = 18100000;
            F_FT4 = 18104000;
            F_JS8 = 18104000;
            F_WSPR = 18104600;
            ifreq = 460570UL;
          } 

//---------- 15m/ 21Mhz 
          if (Band[Band_slot] == 15){

            F_FT8 = 21074000;
            F_FT4 = 21140000;
            F_JS8 = 21078000;
            F_WSPR = 21094600;
            ifreq = 460570UL;
          } 

//---------- 12m/ 24Mhz 
          if (Band[Band_slot] == 12){

            F_FT8 = 24915000;
            F_FT4 = 24919000;
            F_JS8 = 24922000;
            F_WSPR = 24924600;
            ifreq = 460570UL;
          } 
          
//---------- 10m/ 28Mhz 
          if (Band[Band_slot] == 10){

            F_FT8 = 28074000;
            F_FT4 = 28180000;
            F_JS8 = 28078000;
            F_WSPR = 28124600;
            ifreq = 460570UL;
          }
  if (bfo == 0) {
    ifreq = 455000UL;
  }                  
}

//******************************[ Band  Assign Function ]******************************
void Band_assign() 
{
  addr = 50;
  //EEPROM.get(addr,Band_slot);          //ESP32
  EEPROM_get(addr);          //ESP32
  Band_slot = Band_slot % 7;
  ledalloff();
  blink3times(Band_slot);
  delay(500);
  Freq_assign();
  Mode_assign();
}

//*******************************[ Manual TX FUNCTION ]********************************
void ManualTX() 
{
 
 digitalWrite(RX,LOW);
 si5351_clock_enable(SI5351_CLK1, 0);   //RX off
 si5351_clock_enable(SI5351_CLK2, 0);   //RX IF off as well, added by BD6CR
 delay(10);

TXON:  

  digitalWrite(TX,HIGH);
  si5351_set_freq((freq+1000), SI5351_PLL_FIXED, SI5351_CLK0); //1000Hz audio for USB mode
  si5351_clock_enable(SI5351_CLK0, 1);   //TX on
  si5351_clock_enable(SI5351_CLK0, 1);   //TX on
  TX_State = 1;
  if (digitalRead(TXSW)) {
    goto EXIT_TX;
  }
  delay(1);  //ESP32 to avoid watchdog reset
goto TXON;

EXIT_TX:
  digitalWrite(TX,LOW); 
  si5351_set_freq(freq, SI5351_PLL_FIXED, SI5351_CLK0); // set frequency back
  si5351_clock_enable(SI5351_CLK0, 0);   //TX off
  TX_State = 0;
  delay(10);
  digitalWrite(RX,HIGH);
  si5351_clock_enable(SI5351_CLK1, 1);   //RX on, added by BD6CR
   if (bfo == 1) {
  si5351_clock_enable(SI5351_CLK2, 1);   //RX IF on as well, added by BD6CR
   }
}

//******************************[ BAND SELECT Function]********************************
void Band_Select() 
{
  digitalWrite(TX,HIGH);
  addr = 50; 
  //EEPROM.get(addr,Band_slot);           //ESP32
  EEPROM_get(addr);           //ESP32
  ledalloff();
  blink3times(Band_slot);

  Band_cont:
  
    ledalloff();
    band2led(Band_slot);

    if (keypressed(UP)) {
      Band_slot=Band_slot+6;
    }
    if (keypressed(DOWN)) {
      Band_slot++;
    } 
    Band_slot = Band_slot % 7;

    if (keypressed(TXSW)) {
      digitalWrite(TX,LOW);
      goto Band_exit;
    }
    delay(30);

  goto Band_cont;

  Band_exit:

    addr = 50;
    //EEPROM.put(addr, Band_slot);           //ESP32
    EEPROM_put(addr);           //ESP32
    Band_assign();

}

//************************** [SI5351 VFO Calibration Function] ************************
void Calibration() 
{
  int loop = 1;                          //ESP32
  ledalloff();
  digitalWrite(WSPR, HIGH); 
  digitalWrite(FT8, HIGH);

  addr = 10;
  //EEPROM.get(addr, cal_factor);            //ESP32
  EEPROM_get(addr);            //ESP32

  //while (1) //loop forever until reset            //ESP32
  while (loop) //loop until TXSW is pressed            //ESP32
  {
    if (keypressed(UP)) {
      cal_factor = cal_factor - 10;
      printf("cal_factor = %d\n", cal_factor);
    }

    if (keypressed(DOWN)) {
      cal_factor = cal_factor + 10;
            printf("cal_factor = %d\n", cal_factor);
    }
      
    si5351_set_correction(cal_factor);
    // Set CLK2 output
    si5351_set_freq(Cal_freq, SI5351_PLL_FIXED, SI5351_CLK2);
    si5351_drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for Calibration
    si5351_clock_enable(SI5351_CLK2, 1); // Enable clock2 

    if (keypressed(TXSW)) {
      addr = 10;
      //EEPROM_put(addr, cal_factor);             //ESP32
      EEPROM_put(addr);             //ESP32
      digitalWrite(TX,HIGH);
      delay(250);
      digitalWrite(TX,LOW);
      delay(250);
      digitalWrite(TX,HIGH);
      delay(250);
      digitalWrite(TX,LOW);
      delay(250);
      digitalWrite(TX,HIGH);
      delay(250);
      digitalWrite(TX,LOW);
      delay(250);
      loop = 0;
    } 
    delay(1);      //ESP32 to avoid watchdog reset
  }
}

//************************** [Newly Added Functions (ESP32)] ************************

//-------------------Arduino compatibility functions --------------------------
void digitalWrite(int output_pin, int on_off){
    gpio_set_level(output_pin, on_off); 
}

int digitalRead(int input_pin){
    return gpio_get_level(input_pin); 
}

//static void delay(uint32_t interval){
//    ets_delay_us(interval * 1000);
//}
void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

//************************** [ transmit control ] ************************
void transmit(uint32_t audio_freq) {
	if (TX_State == 0 && crossbandcheck()){
		TX_State = 1;
		digitalWrite(RX,LOW);
		digitalWrite(TX,HIGH);
		delay(10);
		si5351_clock_enable(SI5351_CLK1, 0);   //RX off
		si5351_clock_enable(SI5351_CLK2, 0);   //RX IF off as well, added by BD6CR
		si5351_clock_enable(SI5351_CLK0, 1);   // TX on
	}
	si5351_set_freq((freq+audio_freq), SI5351_PLL_FIXED, SI5351_CLK0); 
}

//************************** [ receive control ] ************************
void receive(void) {
	if (TX_State == 1){
		TX_State = 0;
		digitalWrite(RX,HIGH);
		digitalWrite(TX,LOW);
		delay(10);
		si5351_clock_enable(SI5351_CLK0, 0);   // TX off
		si5351_clock_enable(SI5351_CLK1, 1);   //RX on
		si5351_clock_enable(SI5351_CLK2, 1);   //RX IF on as well, added by BD6CR
	}
}

//************************** [ force receive control ] ************************
void adx_force_receive(void) {
		TX_State = 0;
		digitalWrite(RX,HIGH);
		digitalWrite(TX,LOW);
		delay(10);
		si5351_clock_enable(SI5351_CLK0, 0);   // TX off
		si5351_clock_enable(SI5351_CLK1, 1);   //RX on
		si5351_clock_enable(SI5351_CLK2, 1);   //RX IF on as well, added by BD6CR
}

//************************** [ EEPROM get function ] ************************
void EEPROM_get(int address){
	esp_err_t err = nvs_flash_init();
	nvs_handle_t adx_handle;
	err = nvs_open("adx_storage", NVS_READONLY, &adx_handle);   // Open nvs (EEPROM)
	int32_t cal_factor_temp;
	int8_t temp_temp;
	int8_t mode_temp;
	uint8_t Band_slot_temp;
	if (address == 10){
			err = nvs_get_i32(adx_handle, "cal_factor", &cal_factor_temp);
			cal_factor = cal_factor_temp;
	}
	else if (address == 30){
			err = nvs_get_i8(adx_handle, "temp", &temp_temp);
			temp = temp_temp;
	}
	else if (address == 40){
			err = nvs_get_i8(adx_handle, "mode", &mode_temp);
			 mode = mode_temp % 4;
	}
	else if (address == 50){
			err = nvs_get_u8(adx_handle, "Band_slot", &Band_slot_temp);
			Band_slot = Band_slot_temp % 7;
	}
	nvs_close(adx_handle);   // Close nvs (EEPROM)
}

//************************** [ EEPROM put function ] ************************
void EEPROM_put(int address)
{
    esp_err_t err = nvs_flash_init();
	nvs_handle_t adx_handle;
	err = nvs_open("adx_storage", NVS_READWRITE, &adx_handle);   // Open nvs (EEPROM)
	if (address == 10){
			err = nvs_set_i32(adx_handle, "cal_factor", cal_factor);
	}
	else if (address == 30){
			err = nvs_set_i8(adx_handle, "temp", (int8_t)temp);
	}
	else if (address==40){
    		err = nvs_set_i8(adx_handle, "mode", (int8_t)mode);
	}
	else if (address==50){
			err = nvs_set_u8(adx_handle, "Band_slot", (uint8_t)Band_slot);
	}
	err = nvs_commit(adx_handle);
	nvs_close(adx_handle);   // Close nvs (EEPROM)
}

//******************************[ MODE to LED mapping ]*****************************
void mode2led(int mode_number) 
{
	ledalloff();
	switch (mode_number) {
    	case 0:
    		digitalWrite(WSPR, HIGH);
    		break;
    	case 1:
    		digitalWrite(JS8, HIGH);
    		break;    
		case 2:
    		digitalWrite(FT4, HIGH);
    		break;
		case 3:
    		digitalWrite(FT8, HIGH);
    		break;  
    	default:
    		break;
	}
}

//************************************[ Indicate ATT status change]**********************************
//TX led blinks 3 times when att becomes ON, and 2 times when becomes OFF
void att2led(int status) 
{
  for (int i=0; i < (status + 2); i++) {
    digitalWrite(TX, HIGH);
    delay(250);
    digitalWrite(TX, LOW);
    delay(250);
  }
}

// **********************************************************************************************************************
// *******************[ for Bluetooth Audio streaming from ADC to PC  ]*******************************************
//
// modified from "microphone.c" of atomic14's esp32-hsp-hf (https://github.com/atomic14/esp32-hsp-hf)
// 16kHz or 8kHz sampling depending on the host PC

// client
static void (*recording_callback)(const int16_t *buffer, uint16_t num_samples);

// timer to fill output ring buffer
static btstack_timer_source_t driver_timer_source;

// gain
static int source_gain = 1;
// reduce gain 
static int gain_reduce = 1;
// are we currently streaming from the microphone
static bool is_source_streaming;
// samples that are read from the microphone
static int16_t buffer_in[DMA_BUFFER_SAMPLES*8];
// samples that have been converted to PCM 16 bit
static int16_t samples_in[DMA_BUFFER_SAMPLES];

static int8_t adc_sum_16k = 2 ;   //ADC sampling rate = adc_sum_16k * 16kHz 

// determine the ADC offset value
int16_t adc_offset(void)
{
    i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC_INPUT, ADC_ATTEN_DB_11);
	int samplinig_number = 1000;
    uint32_t adc_reading = 0;
    for (int i = 0; i < samplinig_number; i++) {
		adc_reading += adc1_get_raw((adc1_channel_t)ADC_INPUT);
    }
    printf("ADC offset: %d\n", adc_reading /samplinig_number);
    return adc_reading /samplinig_number;
}

// read samples from the microphone and send them off to bluetooth
static void copy_samples(void)
{
	//int16_t adc_offset_value = 2048;
	int8_t  adc_sum = adc_sum_16k * 16000 / bluetooth_sample_rate;
    // read from I2S - we pass portMAX_DELAY so we don't delay
    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, buffer_in, DMA_BUFFER_SAMPLES *adc_sum * sizeof(int16_t), &bytes_read, portMAX_DELAY);
    // how many samples have we read
    int samples_read = bytes_read / sizeof(int16_t);
    //printf("samples_read = %d\n", samples_read);
    if (samples_read > 0)
    {
        // convert the samples to 16 bit
      for (int i = 0; i < samples_read; i+=adc_sum)
        {
            // in theory we should shift to the right by 16 bits, but MEMS microphones have a very
            // high dynamic range, so if we shift all the way we lose a lot of signal
            //samples_in[i] = source_gain * (buffer_in[i] >> 11);
            samples_in[i/adc_sum] = 0;
            for (int j=0; j<adc_sum; j++){ 
           	samples_in[i/adc_sum] += source_gain * ((buffer_in[i+j] & 0xFFF) - adc_offset_value);
           	}
           	samples_in[i/adc_sum] = samples_in[i/adc_sum] / gain_reduce;      //reduce receive gain
        }
       (*recording_callback)((int16_t *)samples_in, samples_read / adc_sum );
    }
}

// callback from the timer
static void driver_timer_handler_source(btstack_timer_source_t *ts)
{
    // if we're streaming from the microphone then copy the samples from the I2S device
    if (recording_callback)
    {
        copy_samples();
    }
    // re-set timer
    btstack_run_loop_set_timer(ts, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

// setup the I2S driver
static int btstack_audio_esp32_source_init(
    uint8_t channels,
    uint32_t samplerate,
    void (*recording)(const int16_t *buffer, uint16_t num_samples))
{
    	recording_callback = recording;
i2s_config_t config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 16000 * adc_sum_16k,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        //.channel_format       = I2S_CHANNEL_FMT_ALL_RIGHT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = DMA_BUFFER_COUNT, // Number of DMA buffers. Max 128.
        .dma_buf_len = DMA_BUFFER_SAMPLES, // Size of each DMA buffer in samples. Max 1024.
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .use_apll  = false,
    	.tx_desc_auto_clear   = false,
    	.fixed_mclk           = 0
    		};

    //i2s_pin_config_t pins = {
    //    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    //    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    //    .data_out_num = -1,
    //    .data_in_num = I2S_MIC_SERIAL_DATA};
    
	i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    //i2s_set_pin(I2S_NUM_0, &pins);
    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC_INPUT, ADC_ATTEN_11db);

    return 0;
}

// get_samplerate (added in btstack_audio_esp32_v4, but not used in this program)
static uint32_t btstack_audio_esp32_source_get_samplerate(void) {
    return 16000;   //dummy
}

// update the gain
static void btstack_audio_esp32_source_gain(uint8_t gain)
{
    source_gain = gain;
}

// start streaming from the microphone
static void btstack_audio_esp32_source_start_stream()
{
	// start i2s
    //i2s_start(I2S_NUM_0);
    i2s_adc_enable(I2S_NUM_0);
    // start timer
    btstack_run_loop_set_timer_handler(&driver_timer_source, &driver_timer_handler_source);
    btstack_run_loop_set_timer(&driver_timer_source, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer_source);

    // state
    is_source_streaming = true;
}

// stop streaming from the microphone
static void btstack_audio_esp32_source_stop_stream(void)
{
    if (!is_source_streaming)
        return;

    // stop timer
    btstack_run_loop_remove_timer(&driver_timer_source);

    // stop i2s
    //i2s_stop(I2S_NUM_0);
    i2s_adc_disable(I2S_NUM_0);

    // state
    is_source_streaming = false;
}

// shutdown the driver
static void btstack_audio_esp32_source_close(void)
{
    if (is_source_streaming)
    {
        btstack_audio_esp32_source_stop_stream();
    }
    // uninstall driver
    i2s_driver_uninstall(I2S_NUM_0);
}

static const btstack_audio_source_t btstack_audio_source_esp32 = {
    /* int (*init)(..);*/ &btstack_audio_esp32_source_init,
    &btstack_audio_esp32_source_get_samplerate,                                                   //added for btstack_audio_esp32_v4.c
    /* void (*set_gain)(uint8_t gain); */ &btstack_audio_esp32_source_gain,
    /* void (*start_stream(void));*/ &btstack_audio_esp32_source_start_stream,
    /* void (*stop_stream)(void)  */ &btstack_audio_esp32_source_stop_stream,
    /* void (*close)(void); */ &btstack_audio_esp32_source_close};

const btstack_audio_source_t *btstack_audio_esp32_source_get_instance_adx(void)
{
    return &btstack_audio_source_esp32;
}

// ************************************************************************************************************************************************
// *******************[ Read Bluetooth Audio data, determine the modulation freqency, and then transmit ]******************************
//
void adx_process_read_audio_data(int16_t *values, uint32_t size, int16_t audio_sampling_rate)
 {
	int16_t mono_prev_p=0; 
	int16_t mono_preprev_p=0;
	float delta_prev_p=0;
	uint32_t sampling_p=0;
	uint16_t audio_frequency_p[50];
	int8_t cycle_p = 0;
	int16_t mono_prev_n=0;
	int16_t mono_preprev_n=0;
	float delta_prev_n=0;
	uint32_t sampling_n=0;
	uint16_t audio_frequency_n[50];
	int8_t cycle_n = 0;
	uint32_t mean_audio_freq_p = 0;
  	uint32_t mean_audio_freq_n = 0;
	uint32_t mean_audio_freq;
	uint32_t dev_audio_freq = 0;
	uint32_t dev_max_audio_freq = 0;
		
    for (int j=0; j<size; j++){
    	int16_t mono = values[j];
    	if ((mono_prev_p < 0) && (mono >= 0)) {
        	if ((mono == 0) && (((float)mono_prev_p * 1.8 - (float)mono_preprev_p < 0.0) || ((float)mono_prev_p * 2.02 - (float)mono_preprev_p > 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
        		break;
    		}
    		int32_t audio_freq_p = 0;
    		float delta_p = 0;
    		if ((mono != 0) || (mono_prev_p != 0)) {
        		int16_t difference = mono - mono_prev_p;
        		delta_p = (float)mono_prev_p / (float)difference;
        		float period = (1.0 + delta_prev_p) + (float)sampling_p - delta_p;
        		audio_freq_p = audio_sampling_rate/period; // in Hz    
    		}
    		if ((audio_freq_p>200) && (audio_freq_p<3000)){
        		audio_frequency_p[cycle_p] = audio_freq_p;   
        		cycle_p++;
    		}
        	delta_prev_p = delta_p;
        	sampling_p=0;
        	mono_preprev_p = mono_prev_p;
        	mono_prev_p = mono;     
    	}
    	else {
			sampling_p++;
			mono_preprev_p = mono_prev_p;
			mono_prev_p = mono;
    	}

    	if ((mono_prev_n > 0) && (mono <= 0)) {
    		if ((mono == 0) && (((float)mono_prev_n * 1.8 - (float)mono_preprev_n > 0.0) || ((float)mono_prev_n * 2.02 - (float)mono_preprev_n < 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
    			break;
    		}
    		int32_t audio_freq_n = 0;
    		float delta_n = 0;
    		if ((mono != 0) || (mono_prev_n != 0)) {
        		int16_t difference = mono - mono_prev_n;
        		delta_n = (float)mono_prev_n / (float)difference;
        		float period = (1.0 + delta_prev_n) + (float)sampling_n - delta_n;
        		audio_freq_n = audio_sampling_rate/period; // in Hz    
    		}
    		if ((audio_freq_n>200) && (audio_freq_n<3000)){
      			audio_frequency_n[cycle_n] = audio_freq_n;   
      			cycle_n++;
    		}
    		delta_prev_n = delta_n;
    		sampling_n=0;
    		mono_preprev_n = mono_prev_n;
    		mono_prev_n = mono;     
  		}
  		else {
    		sampling_n++;
    		mono_preprev_n = mono_prev_n;
    		mono_prev_n = mono;
  		}
	}
	if (cycle_p+cycle_n > 3){
  		mean_audio_freq_p = 0;
  		mean_audio_freq_n = 0;
  		for (int i=1; i < cycle_p; i++){
    		mean_audio_freq_p += audio_frequency_p[i];
  		}
  		for (int i=1; i < cycle_n; i++){
    		mean_audio_freq_n += audio_frequency_n[i];
  		}
		mean_audio_freq =  (mean_audio_freq_p + mean_audio_freq_n) / (cycle_p + cycle_n - 2 );
		dev_audio_freq = 0;
		dev_max_audio_freq = 0;
		for (int i=1; i < cycle_p; i++){
    		dev_audio_freq = (audio_frequency_p[i] - mean_audio_freq) * (audio_frequency_p[i] - mean_audio_freq);
    		if (dev_audio_freq > dev_max_audio_freq) dev_max_audio_freq = dev_audio_freq;
    	}
  		for (int i=1; i < cycle_n; i++){
   			dev_audio_freq = (audio_frequency_n[i] - mean_audio_freq) * (audio_frequency_n[i] - mean_audio_freq);
    		if (dev_audio_freq > dev_max_audio_freq) dev_max_audio_freq = dev_audio_freq;
  		}
		if ((cycle_p+cycle_n >  (mean_audio_freq / 150) ) && (dev_max_audio_freq  <  mean_audio_freq * 10) ) {               // Reduction of erroneous transmissions
			transmit(mean_audio_freq); 
			//printf("audio_freq = %d\n", (mean_audio_freq));
  		}
  	}
	else {
		receive();
	}
}