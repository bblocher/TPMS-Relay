#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <rtl_433_ESP.h>

#ifndef RF_MODULE_FREQUENCY
#  define RF_MODULE_FREQUENCY 433.92
#endif

#ifndef BIT_RATE 
# define BIT_RATE 8.65f
#endif

#define JSON_MSG_BUFFER 512
#define RAW_BUFFER_SIZE 15
#define MAX_QUEUE_SIZE 50

#define TRANSMISSION_DELAY 10000
#define RETRANSMISSION_DELAY 1000
#define RETRANSMISSION_COUNT 5

#define RADIO_MODE_DELAY 1000

#define RADIOLIB_STATE(STATEVAR, FUNCTION)                            \
{                                                                     \
  if ((STATEVAR) != RADIOLIB_ERR_NONE) {                              \
    logprintfLn(LOG_ERR, STR_MODULE " " FUNCTION " failed, code: %d", \
                STATEVAR);                                            \
    while (true)                                                      \
      ;                                                               \
  }                                                                   \
}

uint8_t syncWord[] = {0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t receiveDataBuffer[RAW_BUFFER_SIZE];
uint8_t receiveQueue[MAX_QUEUE_SIZE][RAW_BUFFER_SIZE];

char messageBuffer[JSON_MSG_BUFFER];

int lastRetransmission = millis();
int lastTransmission = millis();
int lastReceived = millis();

volatile bool transmitting = false;
volatile int retransmissionCount = 0;
volatile int receiveQueueIndex = 0;
volatile int transmitQueueIndex = 0;

rtl_433_ESP rf;

void setupRx();
void setupTx();

void transmitHandler() {
  int state = rf.getRadio().finishTransmit();
  RADIOLIB_STATE(state, "finishTransmit");
  
  //Log.verbose(F("Transmission complete" CR));
  transmitting = false;
  lastRetransmission = millis();
}

void relay(uint8_t* data, int dataSize) {  
  transmitting = true;
  
  Log.verbose(F("Sending data (%d bytes): " CR), dataSize);
  for (int i = 0; i < dataSize; i++) {
    Log.verbose(F("%d "), data[i]);
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

  // Abort if queue is full
  if (receiveQueueIndex == MAX_QUEUE_SIZE) {
    Log.warning(F("Queue is full, aborting" CR));
    return;
  }
  
  // Copy the sync word to the transmit buffer
  memccpy(receiveQueue[receiveQueueIndex], syncWord, sizeof(syncWord), RAW_BUFFER_SIZE);

  // Copy the data to the transmit buffer
  memccpy(receiveQueue[receiveQueueIndex++] + sizeof(syncWord), data, dataSize, RAW_BUFFER_SIZE);
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
  if (receiveQueueIndex == MAX_QUEUE_SIZE || millis() - lastTransmission > TRANSMISSION_DELAY) {
    if (!transmitting)
    {
      if (millis() - lastRetransmission < RETRANSMISSION_DELAY)
        return;
      
      Log.verbose(F("Checking for data to transmit" CR));
      
      if (transmitQueueIndex < receiveQueueIndex) {
        // If first packet
        if (transmitQueueIndex == 0)
          setModeTx();

        // Transmit the data
        Log.notice(F("Retransmitting %d of %d packets. Retry %d of %d" CR), transmitQueueIndex + 1, receiveQueueIndex, retransmissionCount + 1, RETRANSMISSION_COUNT);
        relay(receiveQueue[transmitQueueIndex], RAW_BUFFER_SIZE);
        
        // Track total retransmissions
        retransmissionCount++;
        if(retransmissionCount == RETRANSMISSION_COUNT) {
          retransmissionCount = 0;
          transmitQueueIndex++;
        } 
      }
      // Check if we have transmitted all packets
      else if (transmitQueueIndex == receiveQueueIndex && receiveQueueIndex > 0) {
        transmitQueueIndex = 0;
        receiveQueueIndex = 0;

        Log.notice(F("All packets transmitted. Reinitalizing receiver." CR));
                
        setModeRx();
        lastTransmission = millis();
      }
      else {
        Log.verbose(F("No data to transmit" CR));
        lastTransmission = millis();
      }
    }
  }
}

void loop() {
  transmitLoop();  
  rf.loop();
  delay(1);
}