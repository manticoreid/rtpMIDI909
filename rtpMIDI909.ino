#include <Arduino.h>
#include "ESP8266WiFi.h"
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <i2s.h>
#include <i2s_reg.h>
#include <pgmspace.h>
#include "AppleMidi.h"
#include <Ticker.h>


#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include "lib/AudioOutputI2S.h"

AudioOutputI2S soundOut;

extern "C" {
#include "user_interface.h"
}

#define GPIO_IN ((volatile uint32_t*) 0x60000318)    // register contains gpio pin values of ESP8266 in read mode + some extra values (need to be truncated)

char ssid[] = "RUMAH"; //  your network SSID (name)
char pass[] = "rumah4321";    // your network password (use for WPA, or use as key for WEP)

APPLEMIDI_CREATE_INSTANCE(WiFiUDP, AppleMIDI); // see definition in AppleMidi_Defs.h

// Forward declaration
void OnAppleMidiConnected(uint32_t ssrc, char* name);
void OnAppleMidiDisconnected(uint32_t ssrc);
void OnAppleMidiNoteOn(byte channel, byte note, byte velocity);
void OnAppleMidiNoteOff(byte channel, byte note, byte velocity);
void OnAppleMidiControlChange(byte channel, byte note, byte value);

uint32_t i2sACC;
uint8_t i2sCNT=32;
int16_t DAC=0;
uint16_t err;

uint32_t BD16CNT;
uint32_t CP16CNT;
uint32_t CR16CNT;
uint32_t HH16CNT;
uint32_t HT16CNT;
uint32_t LT16CNT;
uint32_t MT16CNT;
uint32_t CH16CNT;
uint32_t OH16CNT;
uint32_t RD16CNT;
uint32_t RS16CNT;
uint32_t SD16CNT;


uint32_t samplecounter=100;

uint8_t pot_control[6];


#include "drum_sampler.h"
#include "gamelan.h"

int16_t SYNTH909() {
  int32_t DRUMTOTAL=0;

  if (TONGCNT<TONGLEN) DRUMTOTAL+=(pgm_read_word_near(TONG + TONGCNT++)^32768)-32768;
  if (THUNGCNT<THUNGLEN) DRUMTOTAL+=(pgm_read_word_near(THUNG + THUNGCNT++)^32768)-32768;
  if (TAKCNT<TAKLEN) DRUMTOTAL+=(pgm_read_word_near(TAK + TAKCNT++)^32768)-32768;
  if (LUNGCNT<LUNGLEN) DRUMTOTAL+=(pgm_read_word_near(LUNG + LUNGCNT++)^32768)-32768;
  if (DLANGCNT<DLANGLEN) DRUMTOTAL+=(pgm_read_word_near(DLANG + DLANGCNT++)^32768)-32768;


  if (BD16CNT<BD16LEN) DRUMTOTAL+=(pgm_read_word_near(BD16 + BD16CNT++)^32768)-32768;
  if (BD16CNT<BD16LEN) DRUMTOTAL+=(pgm_read_word_near(BD16 + BD16CNT++)^32768)-32768;
  if (CP16CNT<CP16LEN) DRUMTOTAL+=(pgm_read_word_near(CP16 + CP16CNT++)^32768)-32768;
  if (CR16CNT<CR16LEN) DRUMTOTAL+=(pgm_read_word_near(CR16 + CR16CNT++)^32768)-32768;
  if (HH16CNT<HH16LEN) DRUMTOTAL+=(pgm_read_word_near(HH16 + HH16CNT++)^32768)-32768;
  if (HT16CNT<HT16LEN) DRUMTOTAL+=(pgm_read_word_near(HT16 + HT16CNT++)^32768)-32768;
  if (LT16CNT<LT16LEN) DRUMTOTAL+=(pgm_read_word_near(LT16 + LT16CNT++)^32768)-32768;
  if (MT16CNT<MT16LEN) DRUMTOTAL+=(pgm_read_word_near(MT16 + MT16CNT++)^32768)-32768;
  if (OH16CNT<OH16LEN) DRUMTOTAL+=(pgm_read_word_near(OH16 + OH16CNT++)^32768)-32768;
  if (RD16CNT<RD16LEN) DRUMTOTAL+=(pgm_read_word_near(RD16 + RD16CNT++)^32768)-32768;
  if (RS16CNT<RS16LEN) DRUMTOTAL+=(pgm_read_word_near(RS16 + RS16CNT++)^32768)-32768;
  if (SD16CNT<SD16LEN) DRUMTOTAL+=(pgm_read_word_near(SD16 + SD16CNT++)^32768)-32768;
  if  (DRUMTOTAL>32767) DRUMTOTAL=32767;
  if  (DRUMTOTAL<-32767) DRUMTOTAL=-32767;
//  DRUMTOTAL+=32768;
  return DRUMTOTAL;
}

// Non-blocking I2S write for left and right 16-bit PCM
bool ICACHE_FLASH_ATTR i2s_write_lr_nb(int16_t left, int16_t right){
  int sample = right & 0xFFFF;
  sample = sample << 16;
  sample |= left & 0xFFFF;
  return i2s_write_sample_nb(sample);
}
uint32_t t = 0;
uint32_t tc = 0;
uint8_t snd = 0;
int16_t sample[2];

void ICACHE_RAM_ATTR onTimerISR(){

  while (!(i2s_is_full())) { //Don't block the ISR

    DAC=SYNTH909();

    //BIT KRASHER
    // 14 - Thrash, SR:44100
    DAC = (DAC >> pot_control[0]) << pot_control[0];

    sample[0] = DAC;
    sample[1] = 0;

    soundOut.ConsumeSample(sample);


    //----------------- Pulse Density Modulated 16-bit I2S DAC --------------------
//     bool flag=i2s_write_lr_nb(0x8000 + DAC,0);
     //bool flag=i2s_write_lr_nb( (DAC^0x8000),0);
    //-----------------------------------------------------------------------

     /*
    // DAC FOR BEATBYTE
    //    snd = t*(t^t+(t>>pot_control[2]|1)^(t-1280^t)>>10);
    //    snd = (t*5&t>>7)|(t*3&t>>10);
    //    snd = (t*9&t>>4|t*5&t>>7|t*3&t/1024)-1;
    //    snd = (t>>6|t|t>>(t>>pot_control[2]))*10+((t>>pot_control[3])&7);
    tc++;
    t = tc >> 2;
    //----------------- Pulse Density Modulated 16-bit I2S DAC --------------------
     bool flag=i2s_write_lr_nb((((((snd)<<8) ^ 32768))),0);
    //-----------------------------------------------------------------------
    */

//          snd = (t*5&t>>7)|(t*3&t>>10);

//    snd = (t>>6|t|t>>(t>>pot_control[2]))*10+((t>>pot_control[3])&7);
//    tc++;
//    t = tc >> 2;

//        sample[0] = snd^0x8000;
//        sample[1] = 0;

//        soundOut.ConsumeSample(sample);


  }

  timer1_write(2000);//Next in 2mS
}

void setup() {
  //WiFi.forceSleepBegin();
  //delay(1);
  system_update_cpu_freq(160);

  //Serial.begin(9600);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }



  // These are fixed by the synthesis routines
  soundOut.SetRate(44100);
  soundOut.SetBitsPerSample(16);
  soundOut.SetChannels(2);
  soundOut.begin();

  /*
  //DAC USING AudioOut Lib
  soundOut.SetRate(44100);
  soundOut.SetBitsPerSample(16);
  soundOut.SetChannels(2);
  soundOut.begin();
  */

  //Serial.print(F("IP address is "));
  //Serial.println(WiFi.localIP());


  AppleMIDI.begin("ESP909"); // 'ESP909' will show up as the session name

  AppleMIDI.OnReceiveNoteOn(OnAppleMidiNoteOn);
  AppleMIDI.OnReceiveControlChange(OnAppleMidiControlChange);

//  i2s_begin();
//  i2s_set_rate(22050); //THRASH
//  i2s_set_rate(44100); //CLEAN
  timer1_attachInterrupt(onTimerISR); //Attach our sampling ISR
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(2000); //Service at 2mS intervall

  ArduinoOTA.setHostname("﻿ESP_123456");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  pot_control[0] = 0;
  pot_control[1] = 0;
  pot_control[2] = 0;
  pot_control[3] = 0;
  pot_control[4] = 0;
  pot_control[5] = 0;



}

void loop() {
 ArduinoOTA.handle();
 AppleMIDI.run();
}


void OnAppleMidiControlChange(byte channel, byte note, byte value) {
    if (channel==10) {
        if (note < 6)
        {
            pot_control[note] = value;
        }

    }
}

void OnAppleMidiNoteOn(byte channel, byte note, byte velocity) {

/* Triggers
Bass Drum MIDI-35
Bass Drum MIDI-36
Rim Shot MIDI-37
Snare Drum MIDI-38
Hand Clap MIDI-39
Snare Drum MIDI-40
Low Tom MIDI-41
Closed Hat MIDI-42
Low Tom MIDI-43
Closed Hat MIDI-44
Mid Tom MIDI-45
Open Hat MIDI-46
Mid Tom MIDI-47
Hi Tom MIDI-48
Crash Cymbal MIDI-49
Hi Tom MIDI-50
Ride Cymbal MIDI-51
*/


  if (channel==10) {
    if(note==30) TONGCNT=0;
    if(note==31) THUNGCNT=0;
    if(note==32) TAKCNT=0;
    if(note==33) LUNGCNT=0;
    if(note==34) DLANGCNT=0;
    if(note==35) BD16CNT=0;
    if(note==36) BD16CNT=0;
    if(note==37) RS16CNT=0;
    if(note==38) SD16CNT=0;
    if(note==39) CP16CNT=0;
    if(note==40) SD16CNT=0;
    if(note==41) LT16CNT=0;
    if(note==42) HH16CNT=0;
    if(note==43) LT16CNT=0;
    if(note==44) HH16CNT=0;
    if(note==45) MT16CNT=0;
    if(note==46) OH16CNT=0;
    if(note==47) MT16CNT=0;
    if(note==48) HT16CNT=0;
    if(note==49) CR16CNT=0;
    if(note==50) HT16CNT=0;
    if(note==51) RD16CNT=0;
  }
}
