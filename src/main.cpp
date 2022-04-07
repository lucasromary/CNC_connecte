#include <Arduino.h>
#include <M5Tough.h>
#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>

#define WIFI_SSID "crunchlab"
#define WIFI_PASSWORD "crunchlab"

#define MQTT_HOST IPAddress(172, 23, 193, 33)
#define MQTT_PORT 1883

#define MQTT_TOPIC_COUNTER "CNC/codeur"
#define MQTT_TOPIC_DISTANCE "CNC/distance"
#define MQTT_TOPIC_CONSO "CNC/consommation"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
int status = WL_IDLE_STATUS;


int val_count = 0;
int last_val_count = 0;
int counter = 0;
long long last_timer_counter = 0;

uint16_t DistCm = 0;      // distance
uint16_t Amp = 0;         // signal strength
uint16_t Temp = 0;        // temperature
uint16_t TStamp = 0;      // timeStamp
uint16_t Erreur = 0;      // Erreur
const byte TfLuna = 0x10; // Adresse I2C du capteur

float amplitude_current; // amplitude current
float effective_value;   // effective current
int sensor_max = 0;
int pin_electricity_sensor = 35;


int getMaxValue()
{
  int sensorValue; // value read from the sensor
  int sensorMax = 0;
  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) // sample for 1000ms
  {
    sensorValue = analogRead(pin_electricity_sensor);
    if (sensorValue > sensorMax)
    {
      /*record the maximum sensor value*/
      sensorMax = sensorValue;
    }
  }
  return sensorMax;
}

void getConso()
{
  sensor_max = getMaxValue();
  // Serial.print("sensor_max = ");
  // Serial.println(sensor_max);
  //  the VCC on the Grove interface of the sensor is 5v
  amplitude_current = (float)sensor_max / 1024 * 5 / 800 * 2000000;
  effective_value = amplitude_current / 1.414; // minimum_current=1/1024*5/800*2000000/1.414=8.6(mA)
                                               // Only for sinusoidal alternating current
}


void connectToWifi()
{
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
}

void onMqttPublish(uint16_t packetId)
{

  // Serial.println("Publish acknowledged.");
}


boolean MesureDistLuna(uint8_t TfLunaAddress)
{
  // Lire les 5 premieères informations (10 octets) sur le capteur Tf-Luna

  Wire.beginTransmission(TfLunaAddress);
  Wire.write(0x00); // Adresse début lecture

  if (Wire.endTransmission(false) != 0)
  {
    return (false); // Sensor did not ACK
  }
  Wire.requestFrom(TfLunaAddress, (uint8_t)10); // lire 10 octets

  if (Wire.available())
  {
    uint8_t Buffer = Wire.read();
    DistCm = Buffer; // octet faible Distance
    Buffer = Wire.read();
    DistCm |= Buffer << 8; // octet fort distance
    Buffer = Wire.read();
    Amp = Buffer; // Amp
    Buffer = Wire.read();
    Amp |= Buffer << 8; // Amp
    Buffer = Wire.read();
    Temp = Buffer; // Temperature
    Buffer = Wire.read();
    Temp |= Buffer << 8; // Temperature
    Buffer = Wire.read();
    TStamp = Buffer; // Time Stamp
    Buffer = Wire.read();
    TStamp |= Buffer << 8; // Time Stamp
    Buffer = Wire.read();
    Erreur = Buffer; // Erreur
    Buffer = Wire.read();
    Erreur |= Buffer << 8; // Erreur
  }
  else
  {
    Serial.println("Pas de donnée à lire");
    return (false);
  }
  return (true);
}

void task1(void * pvParameters) { //Define the tasks to be executed in thread 1.  定义线程1内要执行的任务
  while(1){ //Keep the thread running.  使线程一直运行
    val_count = digitalRead(26);
    //Serial.print("val_pin = ");
    //Serial.println(val_count);
    if(val_count == 1 && last_val_count == 0 && abs(millis() - last_timer_counter) > 50){
      counter++;
      Serial.print("compteur = ");
      Serial.println(counter);
      last_timer_counter = millis();
    }
    last_val_count = val_count;
    delay(10);
  }
}

void task2(void * pvParameters) {
  while(1){
    delay(1000);
    if(MesureDistLuna(TfLuna)){
        //getConso();
        Serial.print(" Distance : ");
        Serial.print(DistCm);
        /*
        Serial.print(" Current = ");
        Serial.print(amplitude_current, 1); // Only one number after the decimal point
        Serial.print(" effective Current = ");
        Serial.println(effective_value, 1);
        */
    }
  }
}

void task3(void * pvParameters) {
  while(1){
    Serial.print("task3 Uptime (ms): ");
    Serial.println(millis());
    mqttClient.publish((MQTT_TOPIC_COUNTER), 2, false, String(counter).c_str());
    mqttClient.publish((MQTT_TOPIC_DISTANCE), 2, false, String(DistCm).c_str());
    delay(2000);
  }
}

void setup() { //task 1 -> encoder , task 2 -> distance, task3 -> mqtt
  M5.begin(true,true,true,true); //Init M5Tough.  初始化M5Tough
  Serial.begin(115200);
  Wire.begin();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();

  pinMode(26,INPUT);
  pinMode(pin_electricity_sensor,INPUT);
  // Creat Task1.  创建线程1
  xTaskCreatePinnedToCore(
                  task1,     //Function to implement the task.  线程对应函数名称(不能有返回值)
                  "task1",   //线程名称
                  4096,      // The size of the task stack specified as the number of * bytes.任务堆栈的大小(字节)
                  NULL,      // Pointer that will be used as the parameter for the task * being created.  创建作为任务输入参数的指针
                  1,         // Priority of the task.  任务的优先级
                  NULL,      // Task handler.  任务句柄
                  0);        // Core where the task should run.  将任务挂载到指定内核

  // Task 2
  xTaskCreatePinnedToCore(
                  task2,
                  "task2",
                  4096,
                  NULL,
                  2,
                  NULL,
                  0);

  // Task 3
  xTaskCreatePinnedToCore(
                  task3,
                  "task3",
                  4096,
                  NULL,
                  3,
                  NULL,
                  0);

  last_timer_counter = millis();
}

void loop() {

}