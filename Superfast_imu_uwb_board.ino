/*
 * Copyright (c) 2021 by Yifeng Cao <ycao361@gatech.edu>
 * Peer-peer protocol
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DWM1000-TWR.ino
 * 
 *  
 */

#include <SPI.h>
#include <math.h>
//#include <DW1000.h>
#include "genericFunctions.h"
#include "RangingContainer.h"
// #include "Adafruit_LSM9DS1.h"
#include <SdFat.h>
#include <time.h>
#include<TimeLib.h>
#include "RTClib.h"
#include<Wire.h>
#include <Adafruit_ISM330DLC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>


// PIN Macro
#define VBATPIN A2
#define LED_PIN 12
#define NOISE_PIN 13
#define GOOD_PIN 6
#define SILENCE_PIN 5
#define DEV_INDICATOR_PIN 13

//Timer for implementing timeouts
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

//Other Macros
#define INIT_RTC_ALWAYS 0
#define USB_CONNECTION 0
#define DEBUG_PRINT 0
#define OUR_UWB_FEATHER 1
#define AUS_UWB_FEATHER 0


#if(OUR_UWB_FEATHER==1)
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin
#endif

#if(AUS_UWB_FEATHER==1)
const uint8_t PIN_RST = 2; // reset pin
const uint8_t PIN_IRQ = 3; // irq pin
const uint8_t PIN_SS = 4; // spi select pin
#endif

Adafruit_ISM330DLC ism330dhcx;
Adafruit_LIS3MDL lis3mdl;

// Global control variables
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile boolean sendComplete = false;
volatile boolean RxTimeout = false;

byte probe_msg[PROBE_LEN] = {0};


typedef enum states{STATE_IDLE, STATE_SEND, STATE_RECV} STATES;
volatile uint8_t current_state = STATE_IDLE;
unsigned long silenced_at =0;
long randNumber;
int currentSlots = 8;






// SD-card related variables
SdFat sd;
SdFile store_distance; //The root file system useful for deleting all files on the SDCard
char filename[14];
int filenum = 0;
int entries_in_file=0;
int SDChipSelect = 10;
int SDEnabled=0;

//IMU
const int candidateImusPerPacket[3] = {10, 8, 5};
const int candidateTimerFrequency[3] = {167, 133, 83};

uint8_t imuBuffer[256]= {0};
int pImuBuffer = 0;
int imuCounter = 0;

int imusPerPacket = candidateImusPerPacket[IMU_MODE];
int imuSampleFrequency = candidateTimerFrequency[IMU_MODE];

//Variables storing information
uint8_t recvFpPower[TRX_NUM+1] = {0};
uint64_t pktSendTime[TRX_NUM+1] = {0};
uint64_t pktRecvTime[TRX_NUM+1] = {0};
uint16_t distVec[TRX_NUM+1] = {0};


double recvTimeout = 0;
double recvElapsedTime = 0;
double recvStartTime = 0;
double recvRemainTime = 0;
double testStartTime = 0;
double testEndTime = 0;


const int recv_id_table[TRX_NUM][TRX_NUM]= {
  {1,1,1,1,1,1},
  {1,1,1,1,1,1},
  {1,1,1,1,1,1}, 
  {1,1,1,1,1,1},
  {1,1,1,1,1,1},
  {1,1,1,1,1,1}
};

byte rx_packet[256];
uint8_t myAcc[1000];
Ranging thisRange;
String message;
int sendDelay;
uint64_t respRxTs[TRX_NUM+1]; //first element is reserved 
int distMat[TRX_NUM+1][TRX_NUM+1] = {0};
uint16_t distTsMat[TRX_NUM+1][TRX_NUM+1] = {0};
int respRxIds[TRX_NUM] = {0,0,0,0,0,0};
double nextPollTime;
double eventStartTime; // in ms
double currentTime;
double elapsedTime;
double pollExpectStartTime;
double pollExpectElapsedTime;
double pollExpectRemainTime;

double respExpectStartTime;
double respExpectElapsedTime;
double finalExpectStartTime;
double finalExpectElapsedTime;
double ackExpectStartTime;
double ackExpectElapsedTime;

double globalTimeMs = 0;
double lastTimeMs = 0;
double currentTimeMs = 0;
boolean timerStart = false;

int pollExpectTuneSlot = 0;
int finalExpectTimeout = 0;

boolean skip_receive = 1;

int currentDeviceIndex = 0;
int respCount = 0;
int ackCount = 0;
int currentInitiator = 0;
uint16_t initiatorSeq = 0; // When I am an initiator, which seq # to put in the packet
bool imuFull = false;

uint16_t rxTimeoutUs = TYPICAL_TIMEOUT;
//Time
RTC_PCF8523 rtc;

double imu_prev_time_ms = 0;

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

typedef struct DeviceRespTs {
  int deviceID;
  uint64_t respRxTime;
};


void receiver(uint16_t rxtoval=0 ) {
  RxTimeout = false;
  received = false;
  DW1000.newReceive();
  DW1000.setDefaults();
  // we cannot don't need to restart the receiver manually
  DW1000.receivePermanently(false);
  if (rxtoval>0) {
    DW1000.setRxTimeout(rxtoval);
  } else {
    //Serial.print("Resetting Timeout to  ");
    //Serial.println(rxtoval);
    DW1000.setRxTimeout(rxtoval);
  }
  DW1000.startReceive();
  //Serial.println("Started Receiver");
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(NOISE_PIN, OUTPUT);
  pinMode(GOOD_PIN, OUTPUT);
  pinMode(DEV_INDICATOR_PIN, OUTPUT);
  pinMode(SILENCE_PIN, INPUT_PULLUP);
  digitalWrite(GOOD_PIN, HIGH);
  analogReadResolution(10);
  // DEBUG monitoring
  Serial.begin(115200);
  while(!Serial)
  {
    delay(10);
    #if(USB_CONNECTION==0)
      break;
    #endif
  }
Serial.print("Waiting...");
delay(5000);
Serial.print("Should see this...");
  //Setting up the RTC Clock
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    //while (1);
  }
  
  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("Setting new time");
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
     rtc.adjust(DateTime(2020, 10, 17, 19, 40, 0));
  }

//In production, INIT_RTC_ALWAYS should be 0.
//Only turn this to 1 when testing
#if (INIT_RTC_ALWAYS == 1)
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#endif
  //rtc.adjust(DateTime(2019, 11, 13, 10, 06, 00));
  //SoftRTC.begin(rtc.now());  // Initialize SoftRTC to the current time

//while(1) {
  Serial.println("Current Time");
  DateTime now = rtc.now();
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  delay(1000);
//}
  Serial.print("Initializing SD card...");
  //delay(1000);
  if (!sd.begin(SDChipSelect, SPI_FULL_SPEED)) {
    Serial.println("SDCard Initialization failed!");
    SDEnabled = 0;
  } else {
    Serial.println("SDCard Initialization done.");
    SDEnabled = 1;
  }

  if (SDEnabled==1) {
    sprintf(filename, "node%d_%d-%d-%d-%d-%d-%d.txt", myid, now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    if (!store_distance.open(filename, O_WRITE|O_CREAT)) {
      Serial.println("Could not create file");
      delay(10000);
    }
    store_distance.println("Hi, SD card is tested successfully");
  }
  randomSeed(analogRead(0));
  Serial.println(F("Peer-peer ranging protocol"));
  Serial.println("Free memory: ");
  Serial.println(freeMemory());
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_UMPIRE_TRACKING);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveTimeoutHandler(handleRxTO);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachSentHandler(handleSent);
  // start reception
  
  current_state = STATE_IDLE;

#if (INITIATOR == 1)
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(5000);
digitalWrite(DEV_INDICATOR_PIN, 0);
//delay(500);
#else
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(1000);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(200);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
#endif
  if (use_imu) {
    if (!ism330dhcx.begin_I2C()) {
      // if (!ism330dhcx.begin_SPI(LSM_CS)) {
      // if (!ism330dhcx.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
      Serial.println("Failed to find ISM330DHCX chip");
    } else {
      Serial.println("ISM330DHCX Found!");
      ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
      ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
      ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);
      ism330dhcx.setGyroDataRate(LSM6DS_RATE_104_HZ);
    }

    if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
      Serial.println("Failed to find LIS3MDL chip");
      while (1) { delay(10); }
    }
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    print_imu_information();
  }
  // startTimer(80);
}


void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}


void handleReceived() {
  // status change on reception success
  
  DW1000.getData(rx_packet, DW1000.getDataLength());
  //Serial.println("Received something...");
  received = true;
  //show_packet(rx_packet, DW1000.getDataLength());
}

void handleError() {
  error = true;
}

void handleRxTO() {
  RxTimeout = true;
  #if (DEBUG_PRINT==1)
  Serial.println("Rx Timeout");
  Serial.println("State: ");
  Serial.println(current_state);
  #endif
}


float gravity_f = 0.0f;



#define YIFENG_TEST 0
int test_flag = 0;

double debugPollStartTime = 0;
double debugPollExpectStartTime = 0;
double debugStartTime = 0;
double debugElapsedTime = 0;

void loop() {
  // Serial.println("I am looping");
  //digitalWrite(GOOD_PIN, HIGH);
  //Serial.println(analogRead(A2));
  //Serial.print("Pin Read: ");
  //Serial.println(digitalRead(SILENCE_PIN));

  // if(SDEnabled) {
  //   if (digitalRead(SILENCE_PIN)==0)
  //   {
  //     store_distance.println("Silenced");
  //     silenced_at = rtc.now().unixtime();
  //   }
  // }

  // Handle IMU
  double imu_current_time_ms = get_time_ms();
  double imu_elapsed_time_ms = get_elapsed_time_ms(imu_prev_time_ms, imu_current_time_ms);
  if (imu_elapsed_time_ms > imu_duration_ms) {
    collect_imu_data();
    imu_elapsed_time_ms = get_time_ms();
  }

  if(!skip_receive){
    receiver(rxTimeoutUs);
    while(!RxTimeout && !received){
    }
  }
  skip_receive = 0;
 
  switch(current_state) {
    case STATE_IDLE: {
        //when every node initially starts, it has to wait a full loop in RECV state otherwise it will interrupt on-going ranging
        current_state = STATE_RECV;
        recvRemainTime = (TRX_NUM - 1) * ONE_NODE_TIMESLOT; 
        recvStartTime = get_time_us();
        skip_receive = 1;
        break;
    }
    case STATE_SEND: {
      //Send POLL here
      // Embed msg
      testStartTime = get_time_ms();
      initiatorSeq++;
      probe_msg[SRC_IDX] = myid;  
      probe_msg[SEQ_IDX] = initiatorSeq & 0xFF;
      probe_msg[SEQ_IDX + 1] = initiatorSeq >> 8;
      int idx = SEQ_IDX + 2 + TS_LEN; // 8
      for(int i = 1; i <= TRX_NUM; i++){
        if(i != myid){
          // put the reception time of last packets from other nodes
          any_msg_set_ts(&probe_msg[idx], pktRecvTime[i]);
          idx += TS_LEN;
        }
      }
      //now idx == 33
      embedDistance(probe_msg, distVec, idx, myid);
      idx += DIST_LEN * (TRX_NUM-1);
      noInterrupts(); // lock imuBuffer
      embedImus(probe_msg, imuBuffer, idx, imusPerPacket * SINGLE_IMU_BUF_LEN);
      pImuBuffer = 0;
      interrupts();

      idx += imusPerPacket * SINGLE_IMU_BUF_LEN;
      int16_t voltageValue = (int16_t) (getVoltage() * 1000);
      // Serial.print("Voltage: ");
      // Serial.println(voltageValue / 1000.0);
      
      probe_msg[idx] = (voltageValue & 0xFF);
      probe_msg[idx+1] = ((voltageValue >> 8) & 0xFF);
      // embedPowers(probe_msg, recvFpPower, idx, myid);
      
      //send
      DW1000.newTransmit();
      DW1000Time txTime;
      DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
      txTime = DW1000.setDelay(deltaTime);
      uint64_t txTime64 = txTime.getTimestamp();
      any_msg_set_ts(&probe_msg[PROBE_TX_TS_IDX], txTime64);
      generic_send(probe_msg, sizeof(probe_msg));
      
#if(DEBUG_FLAG)
      Serial.print("Send  a packet, seq = ");
      Serial.print(initiatorSeq);
      Serial.print("send ts = ");
      print_uint64(txTime64);
      Serial.println("");
#endif
      current_state = STATE_RECV;
      testEndTime = get_time_ms();  
      // Serial.print("Send time until while loop: ");
      Serial.print("Before sending loop: ");
      Serial.println(testEndTime - testStartTime);
      while(!sendComplete){
          //Serial.println("I am in sending loop");
      };
      testEndTime = get_time_ms();

      Serial.print("Prepare sending packets: ");
      Serial.println(testEndTime - testStartTime);

      respCount = 0;
      sendComplete = false;
      recvStartTime = get_time_us();
      recvRemainTime = TRX_NUM * ONE_NODE_TIMESLOT;
      rxTimeoutUs = max(1, min(recvRemainTime, TYPICAL_RX_TIMEOUT));
      pktSendTime[myid] = txTime64;
      break;
    }
    case STATE_RECV:{
      currentTime = get_time_us();
      recvElapsedTime = get_elapsed_time_us(recvStartTime, currentTime);
      // Serial.print("eplased time: ");
      // Serial.print(recvElapsedTime);
      // Serial.print(", remained time: ");
      // Serial.println(recvRemainTime);

      if(recvElapsedTime > recvRemainTime){
        current_state = STATE_SEND;
        skip_receive = 1;
      }else{
        if(received){ // should I move if(received) to outer if loop
          received = false;
          int thisSrc = rx_packet[SRC_IDX];
          uint16_t thisSeq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX+1] << 8);
          int powerLevel = (int)DW1000.getFirstPathPower() + 160;
          recvFpPower[thisSrc] = ((powerLevel)&0xFF);
          
          // get the transmission time of this packet (final_tx_ts)
          uint64_t thisSendTime = 0L;
          any_msg_get_ts(&rx_packet[PROBE_TX_TS_IDX], &thisSendTime);

          // get the reception time of this packet (final_rx_ts)
          DW1000Time rxTS;
          DW1000.getReceiveTimestamp(rxTS);
          uint64_t thisRecvTime = rxTS.getTimestamp();

          // get the reception time of the packet sent by me and recevied by thisSrc (resp_rx_ts)
          int gap = myid > thisSrc ? myid - 1 : myid;
          int idx = PROBE_TX_TS_IDX + TS_LEN + (gap-1) * TS_LEN;
          uint64_t mylastRecvTime = 0l;
          any_msg_get_ts(&rx_packet[idx], &mylastRecvTime);
          // Serial.print("whose response : ");
          // Serial.println(rx_packet[idx-1]);

          // Do two-way-ranging calculation
          thisRange.PollTxTime =  DW1000Time((int64_t) pktSendTime[thisSrc]);
          thisRange.PollRxTime =  DW1000Time((int64_t) pktRecvTime[thisSrc]);
          thisRange.RespTxTime =  DW1000Time((int64_t) pktSendTime[myid]);
          thisRange.RespRxTime =  DW1000Time((int64_t) mylastRecvTime);
          thisRange.FinalTxTime =  DW1000Time((int64_t) thisSendTime);
          thisRange.FinalRxTime =  DW1000Time((int64_t) thisRecvTime);

          //update send/recv timing table
          uint64_t tmpPollTxTs = pktSendTime[thisSrc];
          uint64_t tmpPollRxTs = pktRecvTime[thisSrc];

          pktSendTime[thisSrc] = thisSendTime;
          pktRecvTime[thisSrc] = thisRecvTime;

          int testImu = rx_packet[idx] + ((int)rx_packet[idx+1] << 8);
#if(DEBUG_FLAG)
      Serial.print("Receive a packet from ");
      Serial.print(thisSrc);
      Serial.print(", seq = ");
      Serial.print(thisSeq);
      Serial.print(", all ts = ");
      print_uint64(tmpPollTxTs);
      Serial.print(", ");
      print_uint64(tmpPollRxTs);
      Serial.print(", ");
      print_uint64(pktSendTime[myid]);
      Serial.print(", ");
      print_uint64(mylastRecvTime);
      Serial.print(", ");
      print_uint64(thisSendTime);
      Serial.print(", ");
      print_uint64(thisRecvTime);
      Serial.print(", ");
      Serial.println("");
      
#endif

          //calculate distance and output to the SD card
          if(pktSendTime[thisSrc] > 0 && pktRecvTime[thisSrc] > 0){
            if(!timerStart){
              currentTimeMs = get_time_ms();
              globalTimeMs = currentTimeMs;
              lastTimeMs = currentTimeMs;
              timerStart = true;
            }else{
              currentTimeMs = get_time_ms();
              globalTimeMs += get_elapsed_time_ms(lastTimeMs, currentTimeMs);
              lastTimeMs = currentTimeMs;
            }
            int dist = thisRange.calculateRange();
#if(DEBUG_FLAG)
              Serial.print("Distance: ");
              Serial.println(dist);
#endif
            distMat[myid][rx_packet[SRC_IDX]] = dist;
            if(dist > 0 && dist < 10000){
              distVec[thisSrc] = (uint16_t) dist;
            }else{
              distVec[thisSrc] = 0;
            }
#if(DEBUG_FLAG)
            Serial.print(myid);
            Serial.print(", ");
            Serial.print(rx_packet[SRC_IDX]);
            Serial.print(", ");
            Serial.print(thisSeq);
            Serial.print(", ");
            Serial.print(dist);
            Serial.print(",");
            Serial.println(globalTimeMs);
#endif

          }
#if(DEBUG_FLAG == 1)
           
           Serial.print("Do TWR takes ");
           Serial.println(testEndTime - testStartTime);
           Serial.println("ms");
#endif
          recvElapsedTime = 0;
          if( thisSrc % TRX_NUM + 1 == myid){
            delay(7);
            current_state = STATE_SEND;
            skip_receive = 1;
          }else{
            int nodesAheadofme = (myid + TRX_NUM - thisSrc - 1) % 6;
            recvRemainTime = nodesAheadofme * ONE_NODE_TIMESLOT;
            recvStartTime = get_time_us();
            rxTimeoutUs = max(1, min(recvRemainTime, TYPICAL_RX_TIMEOUT));
          }
        }
        else{ // if not received, reset start-timer, and reduce the waiting time
          recvRemainTime -= rxTimeoutUs;
          recvStartTime = get_time_us();
          rxTimeoutUs = max(1, min(recvRemainTime, TYPICAL_RX_TIMEOUT));
        }
      }
      
      break;
    }
  }
}





void show_packet(byte packet[], int num) {
  #if (DEBUG_PRINT==1)
  for (int i=0;i<num;i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  #endif
  
}

//Timer Functions
//Timer functions.
void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  //Serial.println(TC->COUNT.reg);
  //Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;
  
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  
    NVIC_SetPriority(TC3_IRQn, 3);
    NVIC_EnableIRQ(TC3_IRQn);  
    

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;

  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}


void collect_imu_data() {
  double tmp_imu_data[9] = {0};
  if (use_imu) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    ism330dhcx.getEvent(&accel, &gyro, &temp);

    sensors_event_t event; 
    lis3mdl.getEvent(&event);

    if(pImuBuffer  < SINGLE_IMU_BUF_LEN * imusPerPacket){
      ism330dhcx.getEvent(&accel, &gyro, &temp);
      tmp_imu_data[0] = accel.acceleration.x;
      tmp_imu_data[1] = accel.acceleration.y;
      tmp_imu_data[2] = accel.acceleration.z;
      tmp_imu_data[3] = gyro.gyro.x;
      tmp_imu_data[4] = gyro.gyro.y;
      tmp_imu_data[5] = gyro.gyro.z;

      for(int i = 0; i < 9; i++){
        double tmp = tmp_imu_data[i]; //Yifeng: don't forget this scale
        if(i < 6){
          int16_t val = (int16_t) ((max(min(tmp, 320), -320))*100); //bound it in
          imuBuffer[pImuBuffer] = (val & 0xFF);
          imuBuffer[pImuBuffer+1] = ((val>>8) & 0xFF);
          pImuBuffer += 2;
        }
      }
      if(pImuBuffer >= SINGLE_IMU_BUF_LEN * imusPerPacket){
        imuFull = true;
      }
    }
  }
  
  // else { // not use imu
  //     for(int i = 0; i < 9; i++){
  //       double tmp = 0; //Yifeng: don't forget this scale
  //       if(i < 6){
  //         int16_t val = (int16_t) ((max(min(tmp, 320), -320))*100); //bound it in
  //         imuBuffer[pImuBuffer] = (val & 0xFF);
  //         imuBuffer[pImuBuffer+1] = ((val>>8) & 0xFF);
  //         pImuBuffer += 2;
  //       }else{
  //         int tmpint = (int) max(min(tmp, 255), -256); //mag val should be -256
  //         int magCode = ((int)(tmpint / 2)) + 128;
  //         uint8_t val = magCode; //0 ~ 255
  //         imuBuffer[pImuBuffer] = (val & 0xFF);
  //         pImuBuffer += 1;
  //       }
  //     }
  //     if(pImuBuffer >= SINGLE_IMU_BUF_LEN * imusPerPacket){
  //       imuFull = true;
  //     } 
  // }
}



void TC3_Handler() 
{
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    
     collect_imu_data();

  }
}
/*
void set_timeout(int whichTO, uint32_t delayTime) {
  DW1000Time currectTS;
  uint64_t currentUWBTime;
  DW1000.getSystemTimestamp(currectTS);
  currentUWBTime = currectTS.getTimestamp();
  DW1000Time deltaTime = DW1000Time(delayTime, DW1000Time::MILLISECONDS);
  timeout_time[whichTO] = (currectTS + deltaTime).getTimestamp();
  if (timeout_time[whichTO] > 2^40) {
    timeout_overflow[whichTO] = true;
  } else {
    timeout_overflow[whichTO] = false;
  }
}

*/

//Utility functions
void dateTime(uint16_t* date, uint16_t* time_) {
  DateTime now = rtc.now();
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time_ = FAT_TIME(now.hour(), now.minute(), now.second());
  printDateTime();
}

float getVoltage()
{
  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}



void printDateTime()
{
  DateTime now = rtc.now();
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.println(now.second());
}

void print_imu_information(){
  Serial.print("Accelerometer range set to: ");
  switch (ism330dhcx.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (ism330dhcx.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  // ism330dhcx.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (ism330dhcx.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (ism330dhcx.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__



int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
