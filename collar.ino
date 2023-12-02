// libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// defines 
#define BATTERY_SAMPLERATE_PERIOD_MS 60000 
#define BNO055_SAMPLERATE_PERIOD_MS 40 

// UUID for LEFT collar 
#define SERVICE_UUID "ab0828b1-198e-4351-b779-901fa0e0371e"  
#define SENSOR_MESSAGE_UUID "4ac8a682-9736-4e5d-932b-e9b314050490"   
#define BATTERY_MESSAGE_UUID "1a70ee55-f504-40fe-b249-9609224a9310" 

#define DEVINFO_UUID (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29
#define DEVINFO_NAME_UUID (uint16_t)0x2a24
#define DEVINFO_SERIAL_UUID (uint16_t)0x2a25

#define DEVICE_MANUFACTURER "Foobar"
#define DEVICE_NAME "FLEX"

#define I2C_SDA 18
#define I2C_SCL 19 
#define BT_BUTTON 23 
#define RED_LED 25
#define BLUE_LED 26
#define GREEN_LED 27
#define BATTERY 34 


// variables 
SemaphoreHandle_t reset_semaphore; 
SemaphoreHandle_t battery_semaphore;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

BLEServer *server; 
BLECharacteristic *characteristicSensorMessage;
BLECharacteristic *characteristicBatteryMessage; 

float fSensor = 0; 
float fBattery = 100; 
bool deviceConnected = false;
bool inSession = false; 

unsigned long button_time = 0; 
unsigned long last_button_time = 0; 

// function declarations
void TaskReset(void *pvParameters); 
void TaskSensor(void *pvParameters);
void TaskBattery(void *pvParameters);
void initSensor(void);
void initBluetooth(void);
void advertiseBluetooth(void); 

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer *server) {
      Serial.println("Connected"); 
      deviceConnected = true;
      xSemaphoreTake(battery_semaphore, portMAX_DELAY); 
      if(fBattery <= 20) { 
        digitalWrite(BLUE_LED, LOW); 
        digitalWrite(GREEN_LED, LOW); 
        digitalWrite(RED_LED, HIGH); 
      }
      else { 
        digitalWrite(RED_LED, LOW); 
        digitalWrite(BLUE_LED, LOW); 
        digitalWrite(GREEN_LED, HIGH); 
      }
      xSemaphoreGive(battery_semaphore); 
    }
    void onDisconnect(BLEServer* server) {
      Serial.println("Disconnected"); 
      deviceConnected = false;
      inSession = false; 
      xSemaphoreTake(battery_semaphore, portMAX_DELAY); 
      if(fBattery <= 20) { 
        digitalWrite(BLUE_LED, LOW); 
        digitalWrite(GREEN_LED, LOW); 
        digitalWrite(RED_LED, HIGH); 
      }
      else { 
        digitalWrite(GREEN_LED, LOW); 
        digitalWrite(RED_LED, LOW); 
        digitalWrite(BLUE_LED, HIGH); 
      }
      xSemaphoreGive(battery_semaphore); 
    }
};

class MessageCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristicReceiveMessage) {
        std::string data = characteristicReceiveMessage->getValue();
        if(data == "Start") {
          inSession = true; 
          Serial.println("Started exercise"); 
        }
        else if(data == "Stop") {
          inSession = false; 
          Serial.println("Stopped exercise"); 
        }
        else {
          Serial.println("What?"); 
        }
    }
};

void initSensor(void) { 
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void initBluetooth() {
  BLEDevice::init(DEVICE_NAME);
  server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());
  
  BLEService *service = server->createService(SERVICE_UUID);
  characteristicSensorMessage = service->createCharacteristic(SENSOR_MESSAGE_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  characteristicSensorMessage->setCallbacks(new MessageCallbacks());
  characteristicSensorMessage->addDescriptor(new BLE2902());
  
  characteristicBatteryMessage = service->createCharacteristic(BATTERY_MESSAGE_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  characteristicBatteryMessage->setCallbacks(new MessageCallbacks());
  characteristicBatteryMessage->addDescriptor(new BLE2902());
  service->start(); 

  service = server->createService(DEVINFO_UUID);
  BLECharacteristic *characteristicReceiveMessage = service->createCharacteristic(DEVINFO_MANUFACTURER_UUID, BLECharacteristic::PROPERTY_READ);
  characteristicReceiveMessage->setValue(DEVICE_MANUFACTURER);
  characteristicReceiveMessage = service->createCharacteristic(DEVINFO_NAME_UUID, BLECharacteristic::PROPERTY_READ);
  characteristicReceiveMessage->setValue(DEVICE_NAME);
  characteristicReceiveMessage = service->createCharacteristic(DEVINFO_SERIAL_UUID, BLECharacteristic::PROPERTY_READ);
  String chipId = String((uint32_t)(ESP.getEfuseMac() >> 24), HEX);
  characteristicReceiveMessage->setValue(chipId.c_str());
  service->start();
}

void advertiseBluetooth() { 
  BLEAdvertising *advertisement = server->getAdvertising();
  BLEAdvertisementData adv;
  adv.setName(DEVICE_NAME);
  adv.setCompleteServices(BLEUUID(SERVICE_UUID));
  advertisement->setAdvertisementData(adv);
  advertisement->start(); 
  Serial.println("Trying to connect to the app..."); 
}

void IRAM_ATTR bt_button_isr() { 
  button_time = millis(); 
  if(button_time - last_button_time > 250) { 
    last_button_time = button_time; 
    xSemaphoreGiveFromISR(reset_semaphore, NULL); 
  } 
}

void TaskReset(void *pvParameters) { 
  while(1) { 
    xSemaphoreTake(reset_semaphore, portMAX_DELAY); 
    if(!deviceConnected) {
      advertiseBluetooth(); 
    } 
  }
}

void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  portTickType xFrequency = BNO055_SAMPLERATE_PERIOD_MS/portTICK_PERIOD_MS; 
  while(1) { 
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    if(deviceConnected && inSession) { 
      imu::Quaternion quat = bno.getQuat(); 
      quat.x() = -quat.x(); 
      quat.y() = -quat.y(); 
      quat.z() = -quat.z(); 
  
      imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
      imu::Vector<3> earthaccel; 
  
      //earthaccel[0] = (1-2*(quat.y()*quat.y() + quat.z()*quat.z()))*linearaccel[0] +   (2*(quat.x()*quat.y() + quat.w()*quat.z()))*linearaccel[1] +   (2*(quat.x()*quat.z() - quat.w()*quat.y()))*linearaccel[2]; 
      //earthaccel[1] =   (2*(quat.x()*quat.y() - quat.w()*quat.z()))*linearaccel[0] + (1-2*(quat.x()*quat.x() + quat.z()*quat.z()))*linearaccel[1] +   (2*(quat.y()*quat.z() + quat.w()*quat.x()))*linearaccel[2];
      earthaccel[2] =   (2*(quat.x()*quat.z() + quat.w()*quat.y()))*linearaccel[0] +   (2*(quat.y()*quat.z() - quat.w()*quat.x()))*linearaccel[1] + (1-2*(quat.x()*quat.x() + quat.y()*quat.y()))*linearaccel[2];
      fSensor = earthaccel[2];
      if(fSensor > 15.0) { 
        fSensor = 15.0; 
      }
      else if(fSensor < -15.0) { 
        fSensor = -15.0; 
      } 
      Serial.print("send sensor = "); 
      Serial.println(fSensor); 
      characteristicSensorMessage->setValue(fSensor);  
      characteristicSensorMessage->notify(); 
    } 
  } 
}

void TaskBattery(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  portTickType xFrequency = BATTERY_SAMPLERATE_PERIOD_MS/portTICK_PERIOD_MS; 
  while(1) { 
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    xSemaphoreTake(battery_semaphore, portMAX_DELAY); 
    fBattery = float(100*(((2*analogRead(BATTERY))-3.3)/0.4)); 
    if(fBattery <= 20) { 
      Serial.println("low battery = ");
      Serial.print(fBattery);  
      digitalWrite(BLUE_LED, LOW); 
      digitalWrite(GREEN_LED, LOW); 
      digitalWrite(RED_LED, HIGH); 
    }
    if(deviceConnected) { 
      Serial.print("battery = "); 
      Serial.println(fBattery); 
      characteristicBatteryMessage->setValue(fBattery);  
      characteristicBatteryMessage->notify(); 
    } 
    xSemaphoreGive(battery_semaphore); 
  } 
}

void setup() { 
  Serial.begin(115200); 
  Wire.begin(I2C_SDA, I2C_SCL); 

  analogReadResolution(12); 
  pinMode(BT_BUTTON, INPUT_PULLUP); 
  pinMode(RED_LED, OUTPUT); 
  pinMode(GREEN_LED, OUTPUT); 
  pinMode(BLUE_LED, OUTPUT); 
  digitalWrite(RED_LED, LOW); 
  digitalWrite(GREEN_LED, LOW); 
  digitalWrite(BLUE_LED, HIGH); 

  initSensor(); 
  initBluetooth(); 

  // create tasks  
  reset_semaphore = xSemaphoreCreateBinary(); 
  battery_semaphore = xSemaphoreCreateMutex(); 
  attachInterrupt(BT_BUTTON, bt_button_isr, FALLING); 
  xTaskCreate(TaskReset, "TaskReset", 4096, NULL, 3, NULL); 
  xTaskCreate(TaskBattery, "TaskBattery", 2048, NULL, 1, NULL); 
  xTaskCreate(TaskSensor, "TaskSensor", 4096, NULL, 2, NULL); 
}

void loop() { 
}
