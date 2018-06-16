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

uint32_t i2sACC;
uint8_t i2sCNT=32;
uint16_t DAC=0x8000;
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
uint32_t TRIG0, TRIG1, TRIG2, TRIG3, TRIG4, TRIG5, TRIG6, TRIG7, TRIG8, TRIG9, TRIG10;
uint32_t OLDTRIG0,OLDTRIG1,OLDTRIG2,OLDTRIG3,OLDTRIG4,OLDTRIG5,OLDTRIG6,OLDTRIG7,OLDTRIG8,OLDTRIG9,OLDTRIG10;

#include "drum_sampler.h"

uint16_t SYNTH909() {
  int32_t DRUMTOTAL=0;
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
  DRUMTOTAL+=32768;
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

void ICACHE_RAM_ATTR onTimerISR(){

  while (!(i2s_is_full())) { //Don't block the ISR

    DAC=SYNTH909();
//    snd = t*(t^t+(t>>15|1)^(t-1280^t)>>10);
//    snd = (t*5&t>>7)|(t*3&t>>10);
//    snd = (t*9&t>>4|t*5&t>>7|t*3&t/1024)-1;
    snd = (t>>6|t|t>>(t>>16))*10+((t>>11)&7);
    snd = ((snd) ^ 32768);

    //BIT KRASHER
    DAC = (DAC >> 14) << 14;
    tc++;
    t = tc >> 4;

    //----------------- Pulse Density Modulated 16-bit I2S DAC --------------------
     bool flag=i2s_write_lr_nb(0x8000+DAC,0);
    //-----------------------------------------------------------------------

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

  //Serial.print(F("IP address is "));
  //Serial.println(WiFi.localIP());


  AppleMIDI.begin("ESP909"); // 'ESP909' will show up as the session name

  AppleMIDI.OnReceiveNoteOn(OnAppleMidiNoteOn);

  i2s_begin();
//  i2s_set_rate(22050); //THRASH
  i2s_set_rate(44100); //CLEAN
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



}

void loop() {
 ArduinoOTA.handle();
 AppleMIDI.run();
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
