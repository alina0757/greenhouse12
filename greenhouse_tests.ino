#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP32_Servo.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include <FastLED.h>                   // конфигурация матрицы
#include <FastLED_GFX.h>
#include <FastLEDMatrix.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme280; // Датчик температуры, влажности и атмосферного давления
#include <BH1750FVI.h>
BH1750FVI bh1750; // Датчик освещенности
#include <VEML6075.h>         // добавляем библиотеку датчика ультрафиолета        
VEML6075 veml6075;            // VEML6075

#define NUM_LEDS 64                    // количество светодиодов в матрице 
CRGB leds[NUM_LEDS];                   // определяем матрицу (FastLED библиотека) 
#define LED_PIN             18         // пин к которому подключена матрица 
#define COLOR_ORDER         GRB        // порядок цветов матрицы
#define CHIPSET             WS2812     // тип светодиодов 
#define  pump   17                     // пин насоса 
#define  wind   16                     // пин вентилятора    
// параметры сети
#define WIFI_SSID "ХХХХХХХХХ"
#define WIFI_PASSWORD "ХХХХХХХХХ"

Servo myservo;
int pos = 1;            // начальная позиция сервомотора 
int prevangle = 1;      // предыдущий угол сервомотора

WiFiClientSecure secured_client;


Adafruit_ADS1015 ads(0x48);
const float air_value    = 83900.0;
const float water_value  = 45000.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;  // настройка АЦП на плате расширения I2C MGB-D10

const size_t JSON_BUFFER_SIZE = JSON_OBJECT_SIZE(9);
StaticJsonDocument<JSON_BUFFER_SIZE> jsonDoc;

void setup()
{
  Serial.begin(115200);
  delay(512);
  Serial.println();
  Serial.print("Connecting to Wifi SSID ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.print(" ");
  Serial.println(WiFi.localIP());
  myservo.attach(19);
  Wire.begin();
  pinMode( pump, OUTPUT );
  pinMode( wind, OUTPUT );       // настройка пинов насоса и вентилятора на выход
  digitalWrite(pump, LOW);       // устанавливаем насос и вентилятор изначально выключенными
  digitalWrite(wind, LOW);
  bh1750.begin();
  bh1750.setMode(Continuously_High_Resolution_Mode); // датчик освещенности
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS); // настройки матрицы
  bool bme_status = bme280.begin(); // датчик Т, В и Д
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  if (!veml6075.begin())
    Serial.println("VEML6075 not found!");   // проверка работы датчика ультрафиолета
  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();    // включем АЦП
  
// функция обработки новых сообщений
void handleNewMessages(int numNewMessages)
{
  if (server.hasArg("command")) {
    String command = server.arg("command");
    
  // Выполняйте действия в зависимости от полученной команды
  if ((command == "sensors") || (command == "/sensors")) {
    jsonDoc["uva"] = veml6075.getUVA();
    jsonDoc["uvb"] = veml6075.getUVB();
    jsonDoc["uvIndex"] = veml6075.getUVIndex();
    jsonDoc["light"] = bh1750.getAmbientLight();
    jsonDoc["temperature"] = bme280.readTemperature();
    jsonDoc["humidity"] = bme280.readHumidity();
    jsonDoc["pressure"] = bme280.readPressure() / 100.0F;
    jsonDoc["soilTemperature"] = (float)ads.readADC_SingleEnded(1) * 6.144 * 16 / 1000;
    jsonDoc["soilHumidity"] = map(ads.readADC_SingleEnded(0), air_value, water_value, moisture_0, moisture_100);
    jsonDoc["status"] = "success";
    jsonDoc["command"] = command;

    // Сериализуем JSON в строку
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    String response = "Sensors data";
    server.send(200, "application/json", jsonString);
  
    Serial.println(numNewMessages);
    // выполняем действия в зависимости от пришедшей команды

    if ((command == "/pumpon") || (command == "pumpon"))
    {
      digitalWrite(pump, HIGH);
      delay(400);
      digitalWrite(pump, LOW);
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
      
    }
    if ((command == "/pumpoff") || (command == "pumpoff"))
    {
      digitalWrite(pump, LOW);
      fill_solid( leds, NUM_LEDS, CRGB(0,0,0));
      FastLED.show();
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
    }
    if ((command == "/windon") || (command == "windon"))
    {
      digitalWrite(wind, HIGH);
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
  
    }
    if ((command == "/windoff") || (command == "windoff"))
    {
      digitalWrite(wind, LOW);
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
    }
    if ((command == "/light") || (command == "light"))
    {
      int red = server.arg("red").toInt();
      int green = server.arg("green").toInt();
      int blue = server.arg("blue").toInt();

      fill_solid(leds, NUM_LEDS, CRGB(red, green, blue));
      FastLED.show();
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
    }
    if ((command == "/off") || (command == "off"))
    {
      fill_solid( leds, NUM_LEDS, CRGB(0, 0, 0));
      FastLED.show();
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
    }

    if (command == "/open")
    {
      myservo.write(100);
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
    }
    if (command == "/close")
    {
      myservo.write(0);
      jsonDoc["status"] = "success";
      jsonDoc["command"] = command;      
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      server.send(200, "application/json", jsonString);
    }
  }
}

void loop() // вызываем функцию обработки сообщений через определенный период
{
  if (millis() - bot_lasttime > BOT_MTBS)
  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages)
    {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    bot_lasttime = millis();
  }
}