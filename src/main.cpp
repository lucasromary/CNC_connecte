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

#define WIFI_SSID "Factorybox"
#define WIFI_PASSWORD "F4c70ry80X"

#define MQTT_HOST IPAddress(10, 3, 141, 1)
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

// valeur du seuil de courant mesure au dela duquel la CNC est
// consideree comme en etat de fonctionnement. 
const float seuil_courant = 10; // to be define
// booleen pour suivre si la CNC est en fonctionnement ou pas
bool bCNC_isRuning = false;
enum cncStatusChange_t { CNC_NO_STATUS_CHANGE = 0, CNC_JUST_START , CNC_JUST_STOP};

uint16_t DistCm = 0;      // distance
uint16_t Amp = 0;         // signal strength
uint16_t Temp = 0;        // temperature
uint16_t TStamp = 0;      // timeStamp
uint16_t Erreur = 0;      // Erreur
const byte TfLuna = 0x10; // Adresse I2C du capteur

float amplitude_current; // amplitude current
float effective_value;   // effective current
int sensor_max = 0;
int pin_electricity_sensor = 14;


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

cncStatusChange_t getCNC_isWorking(){
  int sensorValue = analogRead(pin_electricity_sensor);
  getConso();
  Serial.println(amplitude_current);
  Serial.print(" courant = ");
  Serial.println(sensorValue);
  if(sensorValue > seuil_courant && !bCNC_isRuning){
    bCNC_isRuning = true;
    return CNC_JUST_START;
  } else if (sensorValue <=  seuil_courant && bCNC_isRuning){
    bCNC_isRuning = false;
  return CNC_JUST_STOP;
  } else {
    return CNC_NO_STATUS_CHANGE;
  }
}

void connectToWifi()
{
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int counter_wifi = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
    counter_wifi++;
    if(counter_wifi > 120){
      ESP.restart();
    }
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
        Serial.print(" Distance : ");
        Serial.println(DistCm);
        /*
        Serial.print(" Current = ");
        Serial.print(amplitude_current, 1); // Only one number after the decimal point
        Serial.print(" effective Current = ");
        Serial.println(effective_value, 1);
        */
    }
  }
}
// MQTT publish
void task3(void * pvParameters) {
  while(1){
    Serial.print("task3 Uptime (ms): ");
    Serial.println(millis());
    mqttClient.publish((MQTT_TOPIC_COUNTER), 2, false, String(counter).c_str());
    mqttClient.publish((MQTT_TOPIC_DISTANCE), 2, false, String(DistCm).c_str());
    delay(2000);
  }
}
/*
// Power managment
void task4(void * pvParameters) {
  while(1){
    //Serial.print("task4 ");
    //Serial.println(millis());
    switch (getCNC_isWorking())
    {
    case CNC_JUST_START:
    Serial.println("ON");
      mqttClient.publish((MQTT_TOPIC_CONSO), 2, false, "ON");
      break;
    case CNC_JUST_STOP:
    Serial.println("OFF");
      mqttClient.publish((MQTT_TOPIC_CONSO), 2, false, "OFF");
      break;
    default:
    // do nothing
      break;
    }
    delay(500);
  }
}

*/
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
  //pinMode(pin_electricity_sensor,ANALOG);
  // Creat Task1.  
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

  // Task 3, send data over MQTT
  xTaskCreatePinnedToCore(
                  task3,
                  "task3",
                  4096,
                  NULL,
                  3,
                  NULL,
                  0);
/*
  // Task 4, Power managment
  xTaskCreatePinnedToCore(
                  task4,
                  "task4",
                  4096,
                  NULL,
                  3,
                  NULL,
                  0);

  last_timer_counter = millis();
  */
}

void loop() {

}