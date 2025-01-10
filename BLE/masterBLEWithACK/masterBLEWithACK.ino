#include <Wire.h>
#include <Arduino.h>
#include <bluefruit.h>
#include "nrf.h"
#include "nrf_gpio.h"

/** I2C address for local STM Master side */
#define I2C_SLAVE_ADDR  0x09

/** BLE manufacturer ID */
#define MANUFACTURER_ID 0x1234

/** Pin used to wake from system-off (falling edge on D2) */
#define WAKEUP_PIN 2

static uint32_t ackStartTime = 0;    // When we start timing (receive data from slave)
static uint32_t ackSendTime = 0;     // When we send ACK (after STM processes)

// Holds unappended data from SLAVE BLE
static char unappendedBuffer[60] = "";

// Holds appended data from local STM
static char appendedBuffer[60]   = "";

// Add these at top with other static variables
static char lastReceivedMessage[60] = "";

// Flags for logic flow
static bool newFromSlaveBLE      = false;  // Unappended data arrived from SLAVE BLE
static bool appendedFromLocalSTM = false;  // STM wrote appended data

static void enterDeepSleep() {
  Serial.println("[BLE(0x09)] Going to deep sleep...");
  Serial.flush();

  // Configure D2 with pullup, sense low for wake
  nrf_gpio_cfg_sense_input(digitalPinToPinName(WAKEUP_PIN),
                           NRF_GPIO_PIN_PULLUP,
                           NRF_GPIO_PIN_SENSE_LOW);

  // Enter system-off mode
  NRF_POWER->SYSTEMOFF = 1;

  while (1) {
    // Should never reach here
  }
}


void onReceiveEvent(int howMany) {
    char buf[64];
    int i = 0;
    while (Wire.available() && i < (int)sizeof(buf) - 1) {
        buf[i++] = Wire.read();
    }
    buf[i] = '\0';

    if (strcmp(buf, "ACK") == 0) {
        // Calculate elapsed time since we received data from slave
        ackSendTime = millis();
        uint32_t elapsed = ackSendTime - ackStartTime;
        
        // Format ACK message with elapsed time
        snprintf(appendedBuffer, sizeof(appendedBuffer), "ACK_%lu", elapsed);
        appendedFromLocalSTM = true;
        
        Serial.print("[BLE(0x09)] STM sent ACK, elapsed time: ");
        Serial.println(elapsed);
    }
}

void onRequestEvent(void) {
    // Provide unappendedBuffer to the STM
    size_t length = strlen(unappendedBuffer);
    Wire.write((const uint8_t*)unappendedBuffer, length);
}


static void advertiseAppendedOnce() {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    
    uint8_t msd[2 + 29];
    msd[0] = (uint8_t)(MANUFACTURER_ID & 0xFF);
    msd[1] = (uint8_t)((MANUFACTURER_ID >> 8) & 0xFF);
    
    size_t msgLen = strlen(appendedBuffer);
    if (msgLen > 29) msgLen = 29;
    memcpy(&msd[2], appendedBuffer, msgLen);
    
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                                msd, msgLen + 2);
    
    Bluefruit.Advertising.setInterval(32, 32);
    Bluefruit.Advertising.start(0.09);
}


void scan_callback(ble_gap_evt_adv_report_t* report) {
    uint8_t buffer[31];
    int len = Bluefruit.Scanner.parseReportByType(report,
                BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                buffer, sizeof(buffer));
                
    if (len >= 2) {
        uint16_t cid = buffer[0] | (buffer[1] << 8);
        if (cid == MANUFACTURER_ID) {
            int msgLen = len - 2;
            if (msgLen > 0) {
                if (msgLen > sizeof(unappendedBuffer)-1) msgLen = sizeof(unappendedBuffer)-1;
                
                char tempBuf[60];
                memcpy(tempBuf, &buffer[2], msgLen);
                tempBuf[msgLen] = '\0';
                
                if (strlen(tempBuf) > 0 && strcmp(tempBuf, lastReceivedMessage) != 0) {
                    // Start timing when we receive new data from slave
                    ackStartTime = millis();
                    
                    // Store message
                    strncpy(unappendedBuffer, tempBuf, sizeof(unappendedBuffer)-1);
                    unappendedBuffer[sizeof(unappendedBuffer)-1] = '\0';
                    
                    strncpy(lastReceivedMessage, tempBuf, sizeof(lastReceivedMessage)-1);
                    lastReceivedMessage[sizeof(lastReceivedMessage)-1] = '\0';
                    
                    newFromSlaveBLE = true;
                    Serial.print("[BLE(0x09)] Saw from Slave at time ");
                    Serial.print(ackStartTime);
                    Serial.print(" => ");
                    Serial.println(unappendedBuffer);
                }
            }
        }
    }
    Bluefruit.Scanner.resume();
}


void setup()
{
  Serial.begin(115200);
    uint32_t serialStartTime = millis();
    while(!Serial) {
        if(millis() - serialStartTime > 10) {
            break;  // Break if Serial isn't ready after 10ms
        }
    }

  pinMode(WAKEUP_PIN, INPUT_PULLUP);

  // I2C: we are slave @0x09
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onReceiveEvent);  // called if STM writes appended data
  Wire.onRequest(onRequestEvent);  // called if STM reads unappended data

  Serial.println("[BLE(0x09)] Master BLE started @I2C=0x09");

  // BLE init
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("MasterRoverBLE");

  // short scanning
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.setInterval(32, 32);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0.07);  // ~70ms
}


void loop() {
    static bool waitingForACK = false;
    
    // If new data from Slave => STM must read & append
    if (newFromSlaveBLE) {
        newFromSlaveBLE = false;
        Serial.println("[BLE(0x09)] Telling local STM => read & append => write back...");
        waitingForACK = true;
    }

    // Only process STM's ACK if we were waiting for it
    if (waitingForACK && appendedFromLocalSTM) {
        appendedFromLocalSTM = false;
        waitingForACK = false;

        // Advertise ACK
        advertiseAppendedOnce();
        delay(100);
        enterDeepSleep();
    }

    delay(10);
}