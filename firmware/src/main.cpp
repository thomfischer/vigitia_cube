// Based on the BLE_notify example sketch for the ESP32

#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "BLEDevice.h"
#include <string>
#include <stdio.h>

#define SERVICE_UUID                        "441c5e53-1f8b-401d-9222-0e32f400fd15"
#define CHARACTERISTIC_UUID         "441c5e53-1f8b-401d-9222-0e32f400fd15"

// Only GPIOs which are have RTC functionality can be used: 0,2,4,12-15,25-27,32-39.
#define WAKEUP_PIN GPIO_NUM_26
#define WAKEUP_STATE LOW

// set vertical axis in systemInUse()
float accIdleThreshold = 0.30;
int accIdleCounter = 0;
int deepSleepThreshold = 1000; // # of recorded Idle states, before entering deep sleep

MPU9250 IMU;
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;
BLEAdvertising *pAdvertising;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer)
    {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer)
    {
        deviceConnected = false;
    }
};

bool systemInUse()
{
    // assuming the vertical Axis is Z
    if (IMU.az > 1.00 + accIdleThreshold
        || IMU.az < 1.00 - accIdleThreshold
        || IMU.ax > accIdleThreshold
        || IMU.ax < -accIdleThreshold
        || IMU.ay > accIdleThreshold
        || IMU.ay < -accIdleThreshold)
    { accIdleCounter = 0; }
    else accIdleCounter++;

    if (accIdleCounter > deepSleepThreshold) return false;
    else return true;
}

void updateAccel()
{
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];
}

void updateCharacteristic()
{
    String serbuf = "";
    serbuf += IMU.ax;
    serbuf += "|";
    serbuf += IMU.ay;
    serbuf += "|";
    serbuf += IMU.az;
    serbuf += "|";
    serbuf += M5.Power.getBatteryLevel();
    serbuf += ";";

    pCharacteristic->setValue(serbuf.c_str());
}

void setup()
{
    delay(3000);
    Serial.begin(115200);
    Serial.println();

    // set M5Stack and Accelerometer
    M5.begin();
    Wire.begin();
    M5.Lcd.writecommand(ILI9341_DISPOFF);
    // M5.Lcd.clear(0xffffff);
    M5.Lcd.setBrightness(0);
    IMU.initMPU9250();
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

    // set wake pin to pullup, so a connection to GND will trigger wakeup
    pinMode(WAKEUP_PIN, INPUT_PULLUP);
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, WAKEUP_STATE);

    // set BLE options
    BLEDevice::init("VIGITIA Cube");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                    );

    pService->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x12);    // functions that help with iPhone connections issue
    BLEDevice::startAdvertising();
}

void loop()
{
    updateAccel();
    Serial.println(accIdleCounter);
    
    // enter deep sleep if the cube hasn't been moved in a while
    if (!systemInUse())
    {
        Serial.println("shutting down...");
        esp_deep_sleep_start();
    }

    // notify changed value
    if (deviceConnected)
    {
        Serial.println("Connected.");

        updateCharacteristic();
        pCharacteristic->notify();
        delay(5); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        Serial.println("Disconnected.");
        // delay(50); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        // do stuff here on connecting
        Serial.println("Connecting...");
        oldDeviceConnected = deviceConnected;
    }
}
