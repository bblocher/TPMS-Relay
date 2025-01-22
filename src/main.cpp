#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <rtl_433_ESP.h>

#include <AM_ESP32Ble.h>

#include "schraderQueue.h"

/******* Start Global Variable for Widgets *******/
char deviceName[VALUELEN + 1] = "TPMS_RELAY";

#ifndef RF_MODULE_FREQUENCY
#  define RF_MODULE_FREQUENCY 433.92
#endif

#ifndef BIT_RATE 
# define BIT_RATE 8.65f
#endif

#define JSON_MSG_BUFFER 512
#define RAW_BUFFER_SIZE 15

#define RETRANSMISSION_DELAY 30000
#define RETRANSMISSION_COUNT 2 * 30 // 2 transmissions per minute for 30 minutes

#define RADIO_MODE_DELAY 100

#define RADIOLIB_STATE(STATEVAR, FUNCTION)                            \
{                                                                     \
  if ((STATEVAR) != RADIOLIB_ERR_NONE) {                              \
    logprintfLn(LOG_ERR, STR_MODULE " " FUNCTION " failed, code: %d", \
                STATEVAR);                                            \
    while (true)                                                      \
      ;                                                               \
  }                                                                   \
}

void doWork();
void doSync();
void processIncomingMessages(char *variable, char *value);
void processOutgoingMessages();
void processAlarms(char *alarm);
void deviceConnected();

AMController amController(&doWork, &doSync, &processIncomingMessages, &processOutgoingMessages, &processAlarms, &deviceConnected, NULL);

uint8_t receiveDataBuffer[RAW_BUFFER_SIZE];
uint8_t transmitDataBuffer[RAW_BUFFER_SIZE];
SchraderQueue schraderQueue(RETRANSMISSION_COUNT, RETRANSMISSION_DELAY);

char messageBuffer[JSON_MSG_BUFFER];

int lastRetransmission = millis();
int lastTransmission = millis();
int lastReceived = millis();

volatile bool transmitting = false;
volatile bool dataChanged = false;
volatile int transmitCount = 0;

rtl_433_ESP rf;

void setupRx();
void setupTx();

void transmitHandler() {
  int state = rf.getRadio().finishTransmit();
  RADIOLIB_STATE(state, "finishTransmit");
  
  transmitting = false;
  lastRetransmission = millis();
}

void relay(uint8_t* data, int dataSize) {  
  transmitting = true;
  
  Log.verbose(F("Sending data (%d bytes): " CR), dataSize);
  for (int i = 0; i < dataSize; i++) {
    Log.verbose("%X ", data[i]);
  }
  Log.verbose(F(CR));
  
  int state = rf.getRadio().startTransmit(data, dataSize);
  RADIOLIB_STATE(state, "startTransmit");
}

void logJson(JsonObject& jsondata) {
#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  char JSONmessageBuffer[jsondata.measureLength() + 1];
#else
  char JSONmessageBuffer[JSON_MSG_BUFFER];
#endif

  jsondata.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Log.notice(F("Received message : %s" CR), JSONmessageBuffer);
}

void rtl_433_Callback(char* message, uint8_t* data, int dataSize) {
  #if LOG_LEVEL >= LOG_LEVEL_NOTICE
  DynamicJsonBuffer jsonBuffer2(JSON_MSG_BUFFER);
  JsonObject& RFrtl_433_ESPdata = jsonBuffer2.parseObject(message);
  logJson(RFrtl_433_ESPdata);
  #endif

  Log.notice(F("Received data (%d bytes): " CR), dataSize);
  
  schraderQueue.addOrUpdateEntry(data);
}

void setupTx() {
  CC1101 radio = rf.getRadio();
  int state;

  state = radio.setOOK(true);
  RADIOLIB_STATE(state, "setOOK");
  
  state = radio.setEncoding(RADIOLIB_ENCODING_MANCHESTER);
  RADIOLIB_STATE(state, "setEncoding");
 
  state = radio.setOutputPower(10);
  RADIOLIB_STATE(state, "setOutputPower");
  
  state = radio.setBitRate(BIT_RATE);
  RADIOLIB_STATE(state, "setBitRate");
  
  radio.setGdo2Action(transmitHandler, FALLING);
}

void setupRx() {
  rf.initReceiver(RF_MODULE_RECEIVER_GPIO, RF_MODULE_FREQUENCY);
  rf.setCallback(rtl_433_Callback, messageBuffer, JSON_MSG_BUFFER, receiveDataBuffer, RAW_BUFFER_SIZE);
  rf.enableReceiver();
}

void setup() {
  #ifndef LOG_LEVEL
    #define LOG_LEVEL LOG_LEVEL_SILENT
  #endif
  
  Serial.begin(9600);
  delay(1000);

  Log.begin(LOG_LEVEL, &Serial);
  Log.notice(F(" " CR));
  Log.notice(F("****** setup ******" CR));
    
  int state = rf.getRadio().begin();
  RADIOLIB_STATE(state, "begin");
  
  setupRx();

  amController.begin(deviceName);  
  Log.notice(F("****** setup complete ******" CR));
}

void setModeTx() {
  // Stop the receiver
  rf.disableReceiver();
  rf.getRadio().packetMode();
  delay(RADIO_MODE_DELAY);

  // Enable the transmitter  
  setupTx();
  delay(RADIO_MODE_DELAY);
}

void setModeRx() {
  // Disable the transmitter
  rf.getRadio().clearGdo2Action();
  delay(RADIO_MODE_DELAY);

  // Reinitialize the receiver
  rf.setRXSettings();
  rf.getRadio().receiveDirectAsync();
  rf.enableReceiver();
  delay(RADIO_MODE_DELAY);
}

void transmitLoop() {
  // Check if we should transmit
  if (schraderQueue.getNextEntryToRetransmit(transmitDataBuffer, (time_t) millis())) {
    // If first packet
    if (transmitCount == 0)
      setModeTx();

    // Transmit the data
    relay(transmitDataBuffer, RAW_BUFFER_SIZE);
    
    transmitCount++;
    dataChanged = true;
  }
  // Check if we have transmitted all packets
  else {
    if (transmitCount > 0) {
      transmitCount = 0;
      Log.notice(F("All packets transmitted. Reinitalizing receiver." CR));
            
      setModeRx();
      dataChanged = true;
    }
  }
}

void sendDataToManager() {
  // Send data to Arduino Manager
  amController.writeMessage("queueSize", schraderQueue.getQueueSize());

  // Iterate through the queue and send the data
  for (int i = 0; i < schraderQueue.getQueueSize(); i++) {
    std::string identifier = "tire" + std::to_string(i + 1);
    amController.writeTxtMessage(identifier.c_str(), schraderQueue.formatEntry(i).c_str());
  }

  // Clear out any old data
  for (int i = schraderQueue.getQueueSize(); i < 12; i++) {
    std::string identifier = "tire" + std::to_string(i + 1);
    amController.writeTxtMessage(identifier.c_str(), "N/A");
  }
}

void loop() {
  transmitLoop();  
  rf.loop();
  amController.loop();

  if (dataChanged) {
    sendDataToManager();
    dataChanged = false;
  }

  delay(1);
}

void doWork() {
}

void doSync() {
  Log.notice(F("Synchronizing" CR));

  sendDataToManager();
}

void processIncomingMessages(char *variable, char *value) {
  Log.notice(F("Process Incoming" CR));
}

void processOutgoingMessages() {
  // amController.writeMessage("KA", data++);
  // // Write a console log to Arduino Manager
  // amController.log
  // amController.logLn(data);
}

void processAlarms(char *alarm) {
}

void deviceConnected() {
  Log.notice(F("Device connected" CR));
}