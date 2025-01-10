#include <Wire.h>
#include <Arduino.h>
#include <bluefruit.h>
#include "nrf.h"
#include "nrf_gpio.h"

#define I2C_SLAVE_ADDR   0x08
#define MANUFACTURER_ID  0x1234
#define WAKEUP_PIN      2  // D2 for wakeup
#define SLEEP_PIN       3  // D3 for sleep trigger

static char currentMessage[60] = "";
static char lastSentMessage[60] = "";
static uint32_t ackReceiveTime = 0;   // When we received ACK
static uint32_t masterTimer = 0;
static bool gotNewI2CData = false;
static bool gotACK = false;

void sleep_pin_callback(void) {
    // Go to deep sleep on falling edge of D3
    Serial.println("[SLAVE BLE] Timeout");
    delay(10);
    enterDeepSleep();
}

// Function to disable I2C pull-up resistors
void disableI2CPullups() {
    nrf_gpio_cfg(
        digitalPinToPinName(PIN_WIRE_SDA),
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE
    );
    
    nrf_gpio_cfg(
        digitalPinToPinName(PIN_WIRE_SCL),
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE
    );
    
    Serial.println("[SLAVE BLE] I2C pull-up resistors disabled");
}

// Deep sleep entry
static void enterDeepSleep() {
    Serial.println("[SLAVE BLE] Entering deep sleep...");
    Serial.flush();
    
    // Configure D2 (wakeup pin) for wakeup on falling edge
    nrf_gpio_cfg_sense_input(digitalPinToPinName(WAKEUP_PIN),
                           NRF_GPIO_PIN_PULLUP,
                           NRF_GPIO_PIN_SENSE_LOW);
    
    delay(10);  // Small delay to ensure configuration is set
    NRF_POWER->SYSTEMOFF = 1;
    while(1);
}



// I2C receive handler - gets sensor data from STM
void receiveEvent(int howMany) {
    char buffer[64];
    int i = 0;
    while (Wire.available() && i < (int)sizeof(buffer) - 1) {
        buffer[i++] = Wire.read();
    }
    buffer[i] = '\0';
    
    // Only process if new data
    if (strcmp(buffer, lastSentMessage) != 0) {
        strncpy(currentMessage, buffer, sizeof(currentMessage) - 1);
        currentMessage[sizeof(currentMessage) - 1] = '\0';
        
        strncpy(lastSentMessage, buffer, sizeof(lastSentMessage) - 1);
        lastSentMessage[sizeof(lastSentMessage) - 1] = '\0';
        
        gotNewI2CData = true;
        Serial.print("[SLAVE BLE] Received sensor data: '");
        Serial.print(currentMessage);
        Serial.println("'");
    }
}

// I2C request handler - STM wants to read status
void requestEvent(void) {
    if (gotACK) {
        uint32_t sendTime = millis();
        uint32_t elapsed = sendTime - ackReceiveTime;  // Time between ACK and sending
        
        char timerMsg[64];
        snprintf(timerMsg, sizeof(timerMsg), "ACK_%lu_%lu", masterTimer, elapsed);
        strncpy(currentMessage, timerMsg, sizeof(currentMessage)-1);
        
        Wire.write(currentMessage, strlen(currentMessage));
        Serial.print("[SLAVE BLE] Sent ACK and timers. Time between ACK and send: ");
        Serial.println(elapsed);
        
        gotACK = false;
    } else {
        Wire.write("NO", 2);
    }
}

// Advertise sensor data
static void advertiseData() {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    
    uint8_t msd[2 + 29];
    msd[0] = (uint8_t)(MANUFACTURER_ID & 0xFF);
    msd[1] = (uint8_t)((MANUFACTURER_ID >> 8) & 0xFF);
    
    size_t msgLen = strlen(currentMessage);
    if (msgLen > 29) msgLen = 29;
    memcpy(&msd[2], currentMessage, msgLen);
    
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                                msd, msgLen + 2);
    
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.setInterval(72,72);
    Bluefruit.Advertising.start(0.135);  // 60ms advertisement
    
    Serial.println("[SLAVE BLE] Advertising data");
}

// Scan callback - looking for ACK
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
                char msg[64];
                if (msgLen > sizeof(msg)-1) msgLen = sizeof(msg)-1;
                memcpy(msg, &buffer[2], msgLen);
                msg[msgLen] = '\0';
                
                // Check if this is an ACK message with timer
                if (strncmp(msg, "ACK_", 4) == 0) {
                    if (sscanf(msg, "ACK_%lu", &masterTimer) == 1) {
                        ackReceiveTime = millis();  // Record when we got ACK
                        gotACK = true;
                        
                        Serial.print("[SLAVE BLE] Got ACK with master timer: ");
                        Serial.println(masterTimer);
                    }
                }
            }
        }
    }
    Bluefruit.Scanner.resume();
}

void setup() {
    Serial.begin(115200);
    uint32_t serialStartTime = millis();
    while(!Serial) {
        if(millis() - serialStartTime > 10) {
            break;  // Break if Serial isn't ready after 10ms
        }
    }
    
    // Configure wake pin (D2) with pull-up
    pinMode(WAKEUP_PIN, INPUT_PULLUP);
    
    // Configure sleep pin (D3) with pull-up and interrupt on falling edge
    pinMode(SLEEP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SLEEP_PIN), sleep_pin_callback, FALLING);
    // Disable I2C pull-ups before initializing I2C
    disableI2CPullups();
    
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("SlaveRoverBLE");
    
    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.setInterval(64, 64);
    Bluefruit.Scanner.useActiveScan(true);
    
    Serial.println("[SLAVE BLE] Started");
}

void loop() {
    // If new data from STM, advertise it and start scanning for ACK
    if (gotNewI2CData) {
        gotNewI2CData = false;
        
        // First advertise our sensor data
        advertiseData();
        delay(10);  // Give time for advertisement to complete
        
        // Start indefinite scanning for ACK
        Bluefruit.Scanner.start(0);
    }
    
    // Check for ACK separately
    if (gotACK) {
        // Stop scanning before sleep
        Bluefruit.Scanner.stop();
        
        Serial.println("[SLAVE BLE] Got ACK, going to sleep");
        delay(100);  // Give STM time to read ACK status
        enterDeepSleep();
    }
    
    delay(10);
}