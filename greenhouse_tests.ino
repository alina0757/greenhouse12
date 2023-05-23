#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP32_Servo.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include <FastLED.h>                   // конфигурация матрицы // LED matrix configuration   
#include <FastLED_GFX.h>
#include <FastLEDMatrix.h>
#include <AsyncTCP.h>
#define NUM_LEDS 64                    // количество светодиодов в матрице // number of LEDs 
CRGB leds[NUM_LEDS];                   // определяем матрицу (FastLED библиотека) // defining the matrix (fastLED library)
#define LED_PIN             18         // пин к которому подключена матрица // matrix pin
#define COLOR_ORDER         GRB        // порядок цветов матрицы // color order 
#define CHIPSET             WS2812     // тип светодиодов // LED type            

// параметры сети
#define WIFI_SSID "ХХХХХХХХХ"
#define WIFI_PASSWORD "ХХХХХХХХХ"
AsyncWebServer server;

#define  pump   17                     // пин насоса // pump pin             
#define  wind   16                     // пин вентилятора // cooler pin          

Servo myservo;
int pos = 1;            // начальная позиция сервомотора // servo start position
int prevangle = 1;      // предыдущий угол сервомотора // previous angle of servo

// Выберите плату расширения вашей сборки (ненужные занесите в комментарии)
#define MGB_D1015 1
//#define MGB_P8 1

WiFiClientSecure secured_client;
WiFiClient client;

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme280; // Датчик температуры, влажности и атмосферного давления

#include <BH1750.h>
BH1750 lightMeter; // Датчик освещенности

#include "MCP3221.h"
#include "SparkFun_SGP30_Arduino_Library.h"
#include <VEML6075.h>         // добавляем библиотеку датчика ультрафиолета // adding Ultraviolet sensor library 

// Выберите датчик вашей сборки (ненужные занесите в комментарии)
//#define MGS_GUVA 1
#define MGS_CO30 1
//#define MGS_UV60 1

#ifdef MGS_CO30
SGP30 mySensor;
#endif
#ifdef MGS_GUVA
const byte DEV_ADDR = 0x4F;  // 0x5С , 0x4D (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
MCP3221 mcp3221(DEV_ADDR);
#endif
#ifdef MGS_UV60
VEML6075 veml6075;
#endif

#ifdef MGB_D1015
Adafruit_ADS1015 ads(0x48);
const float air_value    = 83900.0;
const float water_value  = 45000.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;  // настройка АЦП на плате расширения I2C MGB-D10
#endif
#ifdef MGB_P8
#define SOIL_MOISTURE    34 // A6
#define SOIL_TEMPERATURE 35 // A7
const float air_value    = 1587.0;
const float water_value  = 800.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;
#endif
const size_t JSON_BUFFER_SIZE = JSON_OBJECT_SIZE(9);
StaticJsonDocument<JSON_BUFFER_SIZE> jsonDoc;


// функция обработки новых сообщений
void handleNewMessages()
{
   if (server.hasArg("command")) {
    String command = server.arg("command");
   
    // выполняем действия в зависимости от пришедшей команды
    if ((command  == "/sensors") || (command  == "sensors")) // измеряем данные
    {
#ifdef MGS_UV60
      veml6075.poll();
      float uva = veml6075.getUVA();
      float uvb = veml6075.getUVB();
      float uv_index = veml6075.getUVIndex();
#endif
#ifdef MGS_GUVA
      float sensorVoltage;
      float sensorValue;
      float UV_index;
      sensorValue = mcp3221.getVoltage();
      sensorVoltage = 1000 * (sensorValue / 4096 * 5.0); // напряжение на АЦП
      UV_index = 370 * sensorVoltage / 200000; // Индекс УФ (эмпирическое измерение)
#endif
#ifdef MGS_CO30
      mySensor.measureAirQuality();
#endif

      float light = lightMeter.readLightLevel();

      float t = bme280.readTemperature();
      float h = bme280.readHumidity();
      float p = bme280.readPressure() / 100.0F;

#ifdef MGB_D1015
      float adc0 = (float)ads.readADC_SingleEnded(0) * 6.144 * 16;
      float adc1 = (float)ads.readADC_SingleEnded(1) * 6.144 * 16;
      float t1 = (adc1 / 1000); //1023.0 * 5.0) - 0.5) * 100.0;
      float h1 = map(adc0, air_value, water_value, moisture_0, moisture_100);
#endif
#ifdef MGB_P8
      float adc0 = analogRead(SOIL_MOISTURE);
      float adc1 = analogRead(SOIL_TEMPERATURE);
      float t1 = ((adc1 / 4095.0 * 5.0) - 0.3) * 100.0; // АЦП разрядность (12) = 4095
      float h1 = map(adc0, air_value, water_value, moisture_0, moisture_100);
#endif
      String welcome = "Показания датчиков:\n";
      jsonDoc["temperature"] = "Temp: " + String(t, 1) + " C\n";
      jsonDoc["humidity"] = String(h, 0) + " %\n";
      jsonDoc["press"] = String(p, 0) + " hPa\n";
      jsonDoc["light"] = String(light) + " Lx\n";
      jsonDoc["soilTemperature"] = String(t1, 0) + " C\n";
      jsonDoc["soilHumidity"] = String(h1, 0) + " %\n";
#ifdef MGS_UV60
      jsonDoc["uva"] = String(uva, 0) + " mkWt/cm2\n";
      jsonDoc["uvb"] = String(uvb, 0) + " mkWt/cm2\n";
      jsonDoc["uvIndex"] = String(uv_index, 1) + " \n";
#endif
#ifdef MGS_GUVA
      jsonDoc["Sensor voltage"] = String(sensorVoltage, 1) + " mV\n";
      jsonDoc["UV Index"] = "UV Index: " + String(UV_index, 1) + " \n";
#endif
#ifdef MGS_CO30
      jsonDoc["CO2"] = String(mySensor.CO2) + " ppm\n";
      jsonDoc["TVOC"] = String(mySensor.TVOC) + " ppb\n";
#endif
		
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    String response = "Sensors data";
    server.send(200, "application/json", jsonString);

    }

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
  Serial.println(WiFi.localIP());
  myservo.attach(19);
  Wire.begin();

  pinMode( pump, OUTPUT );
  pinMode( wind, OUTPUT );       // настройка пинов насоса и вентилятора на выход // pump and cooler pins configured on output mode
  digitalWrite(pump, LOW);       // устанавливаем насос и вентилятор изначально выключенными // turn cooler and pump off
  digitalWrite(wind, LOW);

  lightMeter.begin(); // датчик освещенности

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS); // настройки матрицы

  bool bme_status = bme280.begin(); // датчик Т, В и Д
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");

#ifdef MGS_UV60
  if (!veml6075.begin())
    Serial.println("VEML6075 not found!");
#endif
#ifdef MGS_GUVA
  mcp3221.setVinput(VOLTAGE_INPUT_5V);
#endif
#ifdef MGS_CO30
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();
#endif

#ifdef MGB_D1015
  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();    // включем АЦП
#endif
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  } else {
    reconnect();
  }
  handleNewMessages();
  delay(100);
}
