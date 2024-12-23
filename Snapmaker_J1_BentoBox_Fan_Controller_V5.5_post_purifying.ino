/*
 * License:
 * This project is licensed under the Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0).
 * 
 * You are free to:
 * - Share: Copy and redistribute the material in any medium or format.
 * - Adapt: Remix, transform, and build upon the material.
 * 
 * Under the following terms:
 * - Attribution: You must give appropriate credit, provide a link to the license, and indicate if changes were made. 
 *   You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
 * - NonCommercial: You may NOT use the material for commercial purposes.
 * 
 * Full license text: https://creativecommons.org/licenses/by-nc/4.0/
 * 
 * (c) 2024 Vincent Kratzer
 */

 /*
  * PURPOSE: Fan controller code, to control one or two fans that support PWM speed control and monitor fan speed, display individual fan speed on a 128x32 OLED display (mounted upright), add 
  * OTA capability and custom fan speed setting (saved to internal memory of microcontroller)
  * specifically, this code was written to be used for "BentoBox" 3D printer air purifier fan control, with the intent that the fans automatically start and stop using in Infrared sensor that detects 
  * print bed position.
  * The pin definitions provided in this coide are suitable for Wemos ESP32-S2 mini V1.0.0 (typically purple boards in form factor of the well-known Wemos D1 mini (ESP8266), but with two pin rows per side), utilizing its 
  * onboard LED and onboard push button to use the Menu that is displayed on the OLED display.
  * The code will learn your maximum fan speed and adapt the displayed bargraphs according to the percentage of maximum fan speed of your individual fans. For that, of course, 
  * you need to run the fans once at full speed, otherwise the code uses the fallback value MAX_RPM_FALLBACKVALUE as reference. 
  * For use with other microcontrollers you may need to adapt the pin definitions accordingly.
  * 
  * More specifically, this code is intended to be used with my Snapmaker J1 BentoBox Fan Base, but, of course, may be used in other ways as well.
  * 
  * Comments on Fan Tacho signal: to get a clean signal, there are many suggestions to be found online, but I suggest to simply use a ultry cheap level converter board e.g. from AliExpress, 
  * instead of designing your own RC signal filter as often suggested.
  * The fans typically output a 5V/0V signal on the Tacho pin, and ESP32 has a maximum input voltage of 3.3V on the GPIO pins. Besides solving that problem with the level converter, 
  * the level converter also prevents problems like false readings and jumping fan speed detection when just touching the cables, making the signal clean enough to be perfectly usable.
  */

  /* 
   * Parts that I used for this project:
   *      Wemos ESP32-S2 mini V1.0.0 Microcontroller with WiFi, USB-C (1x)
   *      3,19€ | 1-10pcs ESP32 S2 Mini V1.0.0 WIFI IOT Board based ESP32-S2FN4R2 ESP32-S2 4MB FLASH 2MB PSRAM MicroPython Arduino Compatible
          https://a.aliexpress.com/_EGbtFZW
          
          4-Pin high pressure 4028 fan (2x)
          4,99€ | TFA0412CN Cooling Fan for Delta 4028 DC12V 0.81A 8200RPM 4-Wire PWM Temperature Control 4CM Switch
          https://a.aliexpress.com/_EuLFu9E
          
          OLED Display 128x32 Pixel (1x)
          2,75€ | 0.91 Inch OLED Module 0.91" White/Blue 128X32 OLED LCD LED Display Module IIC Communicate for Ardunio
          https://a.aliexpress.com/_EviiCRI
          
          HEPA Filter set (1x Filter needed, rest are for replacement)
          4,59€ | iLife Hepa Filter Replacement for Robot Vacuum Cleaner Accessories Compatible with V5 V5s V3 V3S V5S V50 pro V55 X5 Models
          https://a.aliexpress.com/_EIh6ckg
          
          4x2mm Magnets, 4 pieces per stackable Module needed (5 Modules used = 20 Magnets)
          2,05€ | NdFeB Rare Earth Magnets 50 to 1000 Pcs 4 X 2 Mm Small round Permanent Neodymium Strong Magnet Disc
          https://a.aliexpress.com/_EuvTaeG
          
          BME280 I2C Temperature and Humidity sensor (1x)
          1,15€ | GY BMP280 Temperature Air Pressure Sensor GY-BMP280-3.3V Barometric Digital Sensors Module for Arduino
          https://a.aliexpress.com/_EyzAkxA
          
          Logic lever converter (1x)
          2,89€ | 1 / 5 / 10 PCS 2 Way / 4 Channel Modul IIC I2C Logic Level Converter Board Bi-Directional Module 5V to 3.3V For Arduino
          https://a.aliexpress.com/_Exy8z2u

          Screws for Fans (1 set needed, 4 screws per fan used)
          2,45€ | 25pcs Phillips Flat Head Mount Screws 6#-32x30mm Long for PC Case Video Card Cooler Radiator Water Cooling Fan DIY Computer Kit
          https://a.aliexpress.com/_EzKxKCu

          
          ---------------- ## OPTIONAL ## --------------------
          DHT22 Temperature and Humidity sensor (1x, optional)
          1,49€ | DHT22 digital temperature and humidity sensor temperature and humidity module AM2302
          https://a.aliexpress.com/_EzdcKKU

          Rubber mounts for Fans (optional, one set (you would need 8 pieces)
          6,86€ | 20pcs 41mm Screw Pin Rivet Rubber PC Fan Noise Absorbtion Fans Anti Vibration Mount Silicone Screws Noise Reduction Equipment
          https://a.aliexpress.com/_EJrwYpE
   */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <WiFiManager.h> //include the WiFiManager library
#include <ArduinoOTA.h>  //include the OTA library
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <DHT_U.h>

Adafruit_BME280 bme; //use I2C interface, create BME280 object
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

unsigned long rpmExceedStartTime = 0; //tracks how long RPM exceeds MAX_RPM

#include <EEPROM.h>


//-------------------------------------------------USER MAY NEED or want TO ADAPT THOSE VALUES ---------------------------------------------------------------------------------------------------------------------------------------------

//fAN speed monitoring - you may need to adapt the values if you use different fans. In this case I did use 4028 fans that were reported to be comparable quiet to others. Fan is the version with 4pins (12V, 0.81A, 9300RPM, 4 Pin connector(+12V, GND, PWM fan speed control pin and fan tacho signal pin), type TFA0412CN, manufacturer: delta electronics,inc., 4.86€ per fan on Aliexpress)
int MAX_RPM = 0;          //max RPM for the fan, will be overwritten by value saved in eeprom
#define MAX_RPM_FALLBACKVALUE 3000;          //"Max RPM" fallback value for the fan, if not saved otherwise in eeprom (eeprom value will be overwritten with new max_rpm value es soon as fan speed exceeds the fallback value)
#define TACHO_PULSES_PER_REV 4 //pulses per revolution for fan
#define UPDATE_INTERVAL 200   //interval for RPM calculation in ms

//oLED display
#define I2C_SDA 13
#define I2C_SCL 12
#define OLED_VCC 10
#define OLED_GND 11
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SSD1306_I2C_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//other pin definitions
#define BUTTON_PIN 0
#define FAN_PWM_PIN_1 16
#define FAN_PWM_PIN_2 17
#define FAN_TACHO_PIN_1 33 //use a level converter for both fan speed input pins to change voltage level from fan to EPS32 pins down to 3.3V
#define FAN_TACHO_PIN_2 34 //use a level converter for both fan speed input pins to change voltage level from fan to EPS32 pins down to 3.3V
#define LED_PIN 15 //onbiard LED of Wemos ESP32-S2 mini is on GPIO 15 
#define DHTPIN 39 //define the pin your DHT sensor data pin is connected to. Consider adding a pullup resistor of e.g. 10kOhm between the DHT sensor data pin and VCC.
#define DHTTYPE DHT22 //define DHT sensor type, use DHT11 for DHT11 sensors and DHT22 for DHT22 sensors.

#define TRIGGER_PIN 40 //input pin for bed level sensing, preferably using optical IR sensor switch

#define PWM_FREQUENCY 25000
#define PWM_RESOLUTION 8


//arrays for runtime and purification mapping: printTimes is an array of printing times that you want to set corresponting post-purifying times. see description below.
const unsigned long printTimes[] = {300, 1800, 3600, 18000, 36000}; //in seconds
//purifyTimes is an array of corresponding purifying times. So after 300s printing (5min) there will be 60s post-purifying.
const unsigned long purifyTimes[] = {60, 360, 600, 900, 3600};       //in seconds

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


DHT dht(DHTPIN, DHTTYPE); //use proprietary single wire data transmission and create DHT object for DHT22 (change DHTTYPE to DHT11 if you use DHT11)
bool dht_found = false;

WiFiManager wifiManager;  //declare it globally, so it can be used in both setup() and loop() - necessary if we want to setup a nonblocking wifimanager instance as that needs peridic calling in loop

enum Mode { AUTO, SET, OFF };
Mode currentMode = AUTO;
int targetFanSpeed = 30;
int currentFanSpeed1 = 0, currentFanSpeed2 = 0;
unsigned long currentRPM1 = 0;
unsigned long currentRPM2 = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastButtonPress = 0;
unsigned long lastInteractionTime = 0;
unsigned long lastBlinkTime = 0;
bool buttonHeld = false;
bool settingModeActive = false;
bool blinkState = false;
int lastSavedMode;


unsigned long fanStartTime = 0;
unsigned long purifyTime = 0;
bool fanWasRunning = false;

//arrays for runtime and purification mapping: printTimes is an array of printing times that you want to set corresponting post-purifying times. see description below.
const unsigned long printTimes[] = {300, 1800, 3600, 18000, 36000}; //in seconds
//purifyTimes is an array of corresponding purifying times. So after 300s printing (5min) there will be 60s post-purifying.
const unsigned long purifyTimes[] = {60, 360, 600, 900, 3600};       //in seconds

/* Look at the arrays above. Upper line and lower line entries go together to define dynamic post-print purifying times. I implemented this to prevent too short
 * fan times after long prints but also too long fan times after very short prints. Adjust it to your needs.
 * {300, 60} (5 min print → 1 min post-print purifying)
 * {1800, 360} (30 min print → 6 min post-print purifying)
 * {36000, 3600} (10 hr print → 1 hr post-print purifying)
 */
const size_t numEntries = sizeof(printTimes) / sizeof(printTimes[0]);
unsigned long postPrintPurifying_EndTime = 0;
const float postPrint_purifying_fanSpeed_factor = 0.78; //set to your needs. 0.78 means that if your speed setting during printing was 50%, the fans will run at 50%*0.78 = 39% speed in post-print purifying.
//you could also use the factor as overdrive, i.e. make it larger than 1 - e.g. set fans to 25% while printing and make post-purifying with factor 3.6 --> 25% * 3,6 = 90% fan speed in post purifying.
//the code will automatically prevent a setting or calculation output above 100% or below 0%.
//just be aware ot he math: if you do 100% during printing and set the factor to 2.0 it will not run the fans faster, it stays at 100% --> no change.
//on the other hand side, if you print at 10% and set the factor to 0.1 you will come out at 1% - most fans will not even run with this setting.
//setting it to 0.0 will stop the fans immediately after the print finished.
unsigned long currentTime = 0;


volatile unsigned long pulseCount1 = 0, pulseCount2 = 0;

bool triggerActive = false;
bool fanon = false;

bool bme280_found = false;

//interrupt-based fan rpm signal monitoring for both fans to ensure that no signal is missed, even if each fan is sending more than 20.000 tacho signals per minute, while ensuring smooth code operation:
void IRAM_ATTR onTachoPulse1() { 
  pulseCount1++;
}
void IRAM_ATTR onTachoPulse2() {
  pulseCount2++;
}

//wifi manager:
void configModeCallback(WiFiManager *myWiFiManager) {
  display.clearDisplay();

  //render "WiFi CONF" with simulated inversion (box with black text on white background)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(0, 9, 128, 17, SSD1306_WHITE); //draw a filled white rectangle (adjust size as needed)
  display.setTextColor(SSD1306_BLACK); //set text color to black for contrast
  display.setCursor(3, 10);
  display.println("WiFi");
  display.setCursor(3, 18);
  display.println("CONF");
  display.display();

  //render "WiFi name" normally (non-inverted)
  display.setTextColor(SSD1306_WHITE); //set text color back to white
  display.setCursor(0, 40); //move cursor down to separate from previous text
  display.println("WiFi");
  display.println("name:");
  display.println();
  display.println(myWiFiManager->getConfigPortalSSID());
  display.display();
}

void saveConfigCallback() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  //display wifi connected message and also display the devices IP address (that it gor from your router via DHCP)
  display.setCursor(0, 0);
  display.println("WiFi");
  display.println("OK...");
  display.println("IP");
  display.println("Addr:");
  display.println();
  display.println();

  //get the IP address and display it
  String ip = WiFi.localIP().toString();
  display.println(ip);
  display.display();
  delay(3000);
}

void display_hum_temp() {
  //placeholder for sensor values
  float dhtTemp = NAN, dhtHum = NAN;
  float bmeTemp = NAN, bmeHum = NAN;

  //read from DHT22 if available
  if (dht_found) {
    dhtTemp = dht.readTemperature();
    dhtHum = dht.readHumidity();
    //recheck sensor status if values are not valid
    if (isnan(dhtTemp) || isnan(dhtHum)) {
      dht_found = false; //mark as unavailable
    }
  }

  //read from BME280 if available
  if (bme280_found) {
    bmeTemp = bme.readTemperature();
    bmeHum = bme.readHumidity();
    //recheck sensor status if values are not valid
    if (isnan(bmeTemp) || isnan(bmeHum)) {
      bme280_found = false; //mark as unavailable
    }
  }

  //determine which values to display
  float tempToDisplay = NAN, humToDisplay = NAN;
  //the following effectively displays the temperature of either sensor (DHT and BME280) or the maximum humidity of both sensors if both are connected.
  if (!isnan(dhtTemp) && !isnan(bmeTemp)) { //if dhtTemp (from DHT-sensor) and bmeTemp (value from BME280 I2c sensor) "are NOT NOT-a-number" --> if both are a valid number, then set the temperature that should be displayed to the higher one of both values.
    tempToDisplay = max(dhtTemp, bmeTemp);
  } else if (!isnan(dhtTemp)) { //if dhtTemp (from DHT-sensor) "is NOT NOT-a-number" --> if it is a valid number, then set the temperature that should be displayed to that value.
    tempToDisplay = dhtTemp;
  } else if (!isnan(bmeTemp)) { //if bmeTemp (value from BME280 I2c sensor) "is NOT NOT-a-number" --> if it is a valid number, then set the temperature that should be displayed to that value.
    tempToDisplay = bmeTemp;
  }

  if (!isnan(dhtHum) && !isnan(bmeHum)) { //same concept for humidity, effectively displays the humidity of either sensor or the maximum humidity of both if both sensors are connected.
    humToDisplay = max(dhtHum, bmeHum);
  } else if (!isnan(dhtHum)) {
    humToDisplay = dhtHum;
  } else if (!isnan(bmeHum)) {
    humToDisplay = bmeHum;
  }

  //draw separation lines
  int barWidth = 32;
  int targetHeight = 108; //y-coordinate of the position to draw the target height
  display.drawLine(0, targetHeight, 2 + barWidth - 1, targetHeight, SSD1306_WHITE);
  targetHeight = 109; //y-coordinate of the position to draw the target
  display.drawLine(0, targetHeight, 2 + barWidth - 1, targetHeight, SSD1306_WHITE);

  //add text for Temperature and Humidity
  display.setCursor(0, 112);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print("T:");
  if (!isnan(tempToDisplay)) {
    display.print(tempToDisplay, 0); //display value with zero digits after the decimal point
    display.print("C");
  } else {
    display.print("---");
  }

  display.setCursor(0, 121);
  display.print("H:");
  if (!isnan(humToDisplay)) {
    display.print(humToDisplay, 0); //display value with zero digits after the decimal point
    display.print("%");
  } else {
    display.print("---");
  }
}

void dht_init() {
  //initialize DHT sensor
  dht.begin();


  //display BME280 I2C sensor state
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 32, 12, SSD1306_WHITE); 
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
  display.setCursor(3, 3);
  display.println("TEMP");
  display.setTextColor(SSD1306_WHITE);  //reset text color to white
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("DHT");
  display.print(DHTTYPE);

  //divider line
  display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
  display.println();
  display.println();

  delay(2000);
  dht_found = !isnan(dht.readTemperature());

  //check DHT22 sensor state and display status
  display.setCursor(0, 40);  //adjusted cursor position for DHTxx status display
  if (dht_found) {
    Serial.println(F("Valid DHT22 sensor found"));
    display.print("DHT");
    display.println(DHTTYPE);
    //display.println("OK");
    display.setCursor(0, 55);
    display.setTextSize(2);
    display.println("OK");
    display.setTextSize(1);
  } else {
    Serial.println(F("Could not find a valid DHT22 sensor, check wiring!"));
    display.print("DHT");
    display.println(DHTTYPE);
    display.println();
    display.println("NOT");
    display.println("FOUND");
  }

  display_hum_temp();  //display temperature and humidity readings (combined BME280/DHT22 logic)
  display.display();
  delay(2500); //delay to allow reading of the displayed text

}

void bme280_init() {
  //display BME280 I2C sensor state
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "I2C"
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
  display.setCursor(6, 3);
  display.println("I2C");
  display.setTextColor(SSD1306_WHITE);  //reset text color to white
  display.setTextSize(1);  
  display.setCursor(0, 14);  //position text below the "OTA" box
  display.print("BM280");

  //divider line
  display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
  display.println();
  display.println();
  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    display.println("BME-");
    display.println("280");
    display.println();
    display.println("NOT");
    display.println("FOUND");
  }
  else {
    Serial.println(F("valid BME280 sensor found"));
    display.println("BME-");
    display.println("280");
    //display.println("OK");
    display.setCursor(0, 55);
    display.setTextSize(2);
    display.println("OK");
    display.setTextSize(1);
    bme280_found = true;
    display_hum_temp();
  }


  display.display();
  delay(2500);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(FAN_PWM_PIN_1, OUTPUT);
  pinMode(FAN_PWM_PIN_2, OUTPUT);
  pinMode(FAN_TACHO_PIN_1, INPUT_PULLUP);
  pinMode(FAN_TACHO_PIN_2, INPUT_PULLUP);
  pinMode(DHTPIN, INPUT_PULLUP);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }

  analogWriteResolution(FAN_PWM_PIN_1, PWM_RESOLUTION);
  analogWriteResolution(FAN_PWM_PIN_2, PWM_RESOLUTION);
  analogWriteFrequency(FAN_PWM_PIN_1, PWM_FREQUENCY);
  analogWriteFrequency(FAN_PWM_PIN_2, PWM_FREQUENCY);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) {
    Serial.println(F("OLED initialization failed"));
  } else {
    display.setRotation(1);
    display.clearDisplay();

    //draw Label with Inverted Box
    display.setTextSize(1);
    display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "J1"
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
    display.setCursor(10, 3);
    display.println("J1");
    display.setTextColor(SSD1306_WHITE);  //reset text color to white
    display.setTextSize(1);  
    display.setCursor(7, 14);
    display.print("FAN");

    //divider line
    display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
    display.println();
    display.setTextSize(1);
    display.println();
    display.println("SNAP-");
    display.println("MAKER");
    display.setCursor(4, 48);
    display.setTextSize(2);
    display.println("J1");
    display.setTextSize(1);
    display.setCursor(0, 68);
    display.println("HEPA");
    display.println(" + ");
    display.println("CHAR");
    display.println("COAL");

    display_hum_temp();  //bME280 and/or DHTxx sensors not yet initialized, will display "---" as values

    display.display();
    delay(3200);
  }

  display.clearDisplay();

  //draw Label with Inverted Box
  display.setTextSize(1);
  display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "EPROM"
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
  display.setCursor(2, 3);
  display.println("EPROM");
  display.setTextColor(SSD1306_WHITE);  //reset text color to white
  display.setTextSize(1);  
  display.setCursor(0, 14);
  display.print("READ");

  //divider line
  display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
  display.println();
  display.println();

  display.println("MODE");
  display.println();
  display.println("FAN-");
  display.println("RPM");
  display.println();
  display.println("FAN-");
  display.println("SPEED");

  display_hum_temp();  //bME280 not yet initialized, will display "---" as values

  display.display();

  EEPROM.begin(512);
  lastSavedMode = EEPROM.read(0);
  currentMode = (lastSavedMode == OFF || lastSavedMode == AUTO) ? static_cast<Mode>(lastSavedMode) : AUTO;
  targetFanSpeed = EEPROM.read(1);
  if (targetFanSpeed < 10 || targetFanSpeed > 100) targetFanSpeed = 70;

  //retrieve two bytes for MAX_RPM from EEPROM during boot (if stored value is valid)
  byte lowByte = EEPROM.read(2);
  byte highByte = EEPROM.read(3);

  //combine the two bytes to form the original MAX_RPM
  MAX_RPM = (highByte << 8) | lowByte;  //combine high and low byte into 16-bit integer


  //if the EEPROM value is invalid or zero (if your MAX_RPM is greater than 0 by default)
  if (MAX_RPM <= 0 || MAX_RPM > 32767) {  //ensure a valid RPM range (and being within eeprom range for two bytes)
    MAX_RPM = MAX_RPM_FALLBACKVALUE; //default value if EEPROM is corrupted or empty
  }

  delay(2500);

  display.clearDisplay();

  //draw Label with Inverted Box
  display.setTextSize(1);
  display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "EPROM"
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
  display.setCursor(2, 3);
  display.println("EPROM");
  display.setTextColor(SSD1306_WHITE);  //reset text color to white
  display.setTextSize(1);  
  display.setCursor(0, 14);
  display.print("READ");

  //divider line
  display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
  
  display.println();
  display.println();
  display.println("FAN-");
  display.println("RPM");
  display.println("max:");
  display.println();
  display.println(MAX_RPM);
  display.println("1/min"); //display the maximum fan RPM that have been saved. An intial run at 100% will be necessary to learn the fan RPM. This can be done whenever you want, but only after doing so the bargraphs will match the percentage of maximum speed.
  display_hum_temp();  //bME280 and/or DHT sensor not yet initialized, will display "---" as values
  display.display();
  delay(2500);

  bme280_init(); //initialize I2C humidity and temperature sensor BME280
  dht_init(); //initialize DHT22 humidity and temperature sensor

  attachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN_1), onTachoPulse1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN_2), onTachoPulse2, FALLING);

  //wiFiManager wifiManager; //that is local declare, then it is only available in setup, therefore we need to declare it outside of setup to make it available also in loop, to be able to use it as non-blocking wifimanager
  //wifiManager.setAPCallback(configModeCallback);  //set the custom callback when wifimanager enters config mode
  //wifiManager.setSaveConfigCallback(saveConfigCallback);
  ////wifiManager.autoConnect("J1FanBentoBox"); //start WiFiManager and enter config mode if needed - blocking version
  //wifiManager.startConfigPortal("J1FanBentoBox"); //non blocking version, additionally needs wifiManager.process(); in void loop to work
  //initialize WiFiManager
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //try connecting to Wi-Fi or start the config portal if needed
  if (!wifiManager.autoConnect("J1FanBentoBox")) {  //non-blocking
    Serial.println("Failed to connect, starting Wi-Fi configuration...");
  }
  ArduinoOTA.setHostname("J1FanBentoBox"); //set your desired OTA hostname (you will see that name in Arduino IDE to identify the device) - remeber to rename if you have more than one of these devices, so that you know which one is which
  ArduinoOTA.onStart([]() {
    display.clearDisplay();

    //draw OTA Label with Inverted Box
    display.setTextSize(1);
    display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "OTA"
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
    display.setCursor(7, 3);
    display.println("OTA");
    display.setTextColor(SSD1306_WHITE);  //reset text color to white
    display.setTextSize(1);  
    display.setCursor(0, 14);
    display.print("START");

    //divider line
    display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
    display.println();
    display.println();
    display.println();
    display.println("FLASH");
    display.println("FIRM");
    display.println("WARE");
    display_hum_temp(); 

    display.display();
    delay(2200);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {  //will be triggered if OTA update is started
    display.clearDisplay();

    //draw OTA Label with Inverted Box
    display.setTextSize(1);
    display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "OTA"
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
    display.setCursor(7, 3);
    display.println("OTA");
    display.setTextColor(SSD1306_WHITE);  //reset text color to white

    //calculate and display the percentage text below "OTA"
    int percentage = (progress * 100) / total;
    display.setTextSize(1);  
    display.setCursor(4, 14);  //position for "percentage xy" below the "OTA" box
    display.print(percentage);
    display.print("%");

    //divider line
    display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position

    //further adjust progress bar dimensions
    int maxBarHeight = SCREEN_WIDTH - 54; 
    int barX = 2;  //x position of the bar (leave some margin from display edge on the left side for better looks)
    int barWidth = 28;  //width of the bar (leave margin to the display edge on the right side for better looks)
    int barY = 29;  //y position remains the same to maintain alignment
    int barHeight = map(progress, 0, total, 0, maxBarHeight);  //map progress to new height

    //draw progress bar frame and fill
    display.drawRoundRect(barX, barY, barWidth, maxBarHeight, 3, SSD1306_WHITE);  //outline with reduced height
    display.fillRoundRect(barX, barY + (maxBarHeight - barHeight), barWidth, barHeight, 3, SSD1306_WHITE);  //filled portion with reduced height

    display_hum_temp();

    display.display();
  });

  if (WiFi.status() != WL_CONNECTED) {
    display.clearDisplay();
    display.setTextSize(1);
    display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "WiFi"
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
    display.setCursor(3, 3);
    display.println("WiFi");
    display.setTextColor(SSD1306_WHITE);  //reset text color to white
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.print("ERROR");
    display.println();
    display.setTextSize(2);
    display.setCursor(15, 30);
    display.println("!");
    display.setTextSize(1);

    //divider line
    display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
    display_hum_temp();
    display.display();
    delay(3200);
  }
  else {
    //display message if successfully connected
    display.clearDisplay();
    display.setTextSize(1);
    display.fillRect(0, 0, 32, 12, SSD1306_WHITE);  //white box for "WiFi"
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  //black text on white box
    display.setCursor(3, 3);
    display.println("WiFi");
    display.setTextColor(SSD1306_WHITE);  //reset text color to white
    display.setTextSize(1);  
    display.setCursor(10, 14);
    display.println("OK");
    display.println();
    //display.println("IP");
    //display.println("Addr:");
    //display.println();
    //display.println();
    ////get the IP address and display it
    //string ip = WiFi.localIP().toString();
    //display.println(ip);
    display.println("IP");
    display.println("Addr:");
    display.println();
    display.println();

    //get the IP address and split it into blocks for a newline after each dot:
    String ip = WiFi.localIP().toString();
    int start = 0;
    int end = ip.indexOf('.');

    //loop to print each block of the IP address with a dot
    while (end != -1) {
      display.print(ip.substring(start, end)); //print block
      display.println('.'); //print dot and new line
      start = end + 1; //move start to next block
      end = ip.indexOf('.', start); //find next dot
    }

    //print the last block
    display.println(ip.substring(start));


    //divider line
    display.drawLine(0, 23, SCREEN_WIDTH, 23, SSD1306_WHITE);  //divider line position
    display_hum_temp(); 
    display.display();
    delay(2200);
  }

  ArduinoOTA.begin(); //initialize OTA
}

void loop() {
  wifiManager.process(); //non-blocking WiFiManager process
  ArduinoOTA.handle();
  handleButtonPress();
  handleModeLogic();
  calculateFanSpeed();
  updateDisplay();
}

void handleButtonPress() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonHeld) {
      lastButtonPress = millis();
      buttonHeld = true;
    } else if (millis() - lastButtonPress > 1000 && (currentMode == SET || currentMode == AUTO) && !settingModeActive) {
      settingModeActive = true;
      currentMode = SET;
      lastInteractionTime = millis();
    }
  } else if (buttonHeld) {
    buttonHeld = false;
    if (millis() - lastButtonPress < 1000) {
      if (settingModeActive) {
        adjustTargetFanSpeed();
        lastInteractionTime = millis();
      } else {
        currentMode = (currentMode == AUTO) ? SET : (currentMode == SET) ? OFF : AUTO;
        settingModeActive = false;
        Serial.print("Mode: ");
        Serial.println(currentMode == AUTO ? "AUTO" : currentMode == SET ? "SET" : "OFF");
      }
    }
  }

  if (settingModeActive && millis() - lastInteractionTime > 5000) {
    saveSpeedAndExitToAuto();
  }
  if (currentMode == OFF) {
    EEPROM.write(0, OFF);
    EEPROM.commit();
  }
  else if (currentMode == AUTO) {
    EEPROM.write(0, AUTO);
    EEPROM.commit();
  }
}

//void handleModeLogic() {
//triggerActive = digitalRead(TRIGGER_PIN) == LOW;
//
//if (currentMode == AUTO) {
// if (triggerActive) {
//   //stop the fan if trigger is active (LOW)
//   setFanSpeed(0);
// } else {
//   //run the fan at target speed if trigger is not active (HIGH)
//    setFanSpeed(targetFanSpeed);
//  }
//} else if (currentMode == SET) {
//  setFanSpeed(targetFanSpeed);
//} else if (currentMode == OFF) {
//  setFanSpeed(0);
//}
//}

unsigned long calculatePurifyTime(unsigned long runtime) {
  for (size_t i = 0; i < numEntries; i++) {
    if (runtime <= printTimes[i]) {
      return purifyTimes[i];
    }
  }
  return purifyTimes[numEntries - 1]; //default to the longest purify time
}



void handleModeLogic() { //added dynamic print-time dependent purifying time after print has finishied
  triggerActive = digitalRead(TRIGGER_PIN) == LOW; //trigger is the IR sensor that detects bed position or 
  //unsigned long currentTime = millis() / 1000; //convert to seconds
  currentTime = millis() / 1000; //convert to seconds

  if (currentMode == AUTO) {
    if (triggerActive) { //print not started or finished
      if (fanWasRunning) {
        //calculate post-print purifying time
        unsigned long runtime = currentTime - fanStartTime;
        //unsigned long purifyTime = calculatePurifyTime(runtime);
        purifyTime = calculatePurifyTime(runtime);
        postPrintPurifying_EndTime = currentTime + purifyTime;

        //reset the flag
        fanWasRunning = false;
      }

      //stop the fans when print bed is in home position
      setFanSpeed(0);
    } else { //print is running
      //if a new print starts while post-purifying is active, reset post-print purifying
      if (currentTime < postPrintPurifying_EndTime) {
        postPrintPurifying_EndTime = 0; //cancel any ongoing post-print purifying
      }

      //start tracking runtime
      if (!fanWasRunning) {
        fanStartTime = currentTime;
        fanWasRunning = true;
      }

      //run the fan at target speed
      setFanSpeed(targetFanSpeed);
    }
  } else if (currentMode == SET) {
    setFanSpeed(targetFanSpeed);
    postPrintPurifying_EndTime = 0; //reset post-print purifying when in SET mode
  } else if (currentMode == OFF) {
    setFanSpeed(0);
    postPrintPurifying_EndTime = 0; //reset post-print purifying when in OFF mode
  }

  //handle post-print purifying phase
  if (currentTime >= postPrintPurifying_EndTime && postPrintPurifying_EndTime > 0) {
    postPrintPurifying_EndTime = 0; //ensure this only runs once
    setFanSpeed(0);          //stop the fan after the post-print purifying phase
  } else if (currentTime < postPrintPurifying_EndTime) { //post-print purifying
    setFanSpeed(targetFanSpeed * postPrint_purifying_fanSpeed_factor); //run e.g at 0.5 * speed or at 0.78 * normal speed setting for post-print purifying. not an percentage of absolute max speed but resembles a percentage of the actual setting.
    //display_post-print_purifying(); //needs to be added
  }
}

//void setFanSpeed(int speed) {
// if (speed == 0) {
//   fanon = false;
// }
// else {
//   fanon = true;
// }
// int pwmValue = map(speed, 0, 100, 0, 255); //inverted logic for PWM
// analogWrite(FAN_PWM_PIN_1, pwmValue);
// analogWrite(FAN_PWM_PIN_2, pwmValue);
//}

void setFanSpeed(float speed) { //add support for floating point speed settings
  if (speed <= 0.0f) { //ensure speed <= 0 turns the fans off
    fanon = false;
    speed = 0.0f; //clamp to 0
  } else {
    fanon = true;
    if (speed > 100.0f) speed = 100.0f; //if larger, clamp to 100%
  }

  int pwmValue = map((int)speed, 0, 100, 0, 255); //map 0-100% to 0-255 PWM value
  analogWrite(FAN_PWM_PIN_1, pwmValue);
  analogWrite(FAN_PWM_PIN_2, pwmValue);
}

void calculateFanSpeed() {
  if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = millis();
    noInterrupts();
    unsigned long count1 = pulseCount1;
    unsigned long count2 = pulseCount2;
    pulseCount1 = 0;
    pulseCount2 = 0;
    interrupts();

    //this is RPM calculation
    float currentRPM1 = (float)count1 * 60.0 / TACHO_PULSES_PER_REV / ((float)UPDATE_INTERVAL / 1000.0);
    float currentRPM2 = (float)count2 * 60.0 / TACHO_PULSES_PER_REV / ((float)UPDATE_INTERVAL / 1000.0);

    //check if conditions are met to possibly update MAX_RPM
    if (targetFanSpeed == 100) {
      if (currentRPM1 > MAX_RPM || currentRPM2 > MAX_RPM) {
        if (rpmExceedStartTime == 0) {
          //start tracking the time
          rpmExceedStartTime = millis();
        } else if (millis() - rpmExceedStartTime >= 3000) {
          //only update MAX_RPM if it has been exceeded for more than 3 seconds
          if (currentRPM1 > MAX_RPM) {
            MAX_RPM = currentRPM1;
            saveMaxRPMToEEPROM();  //save updated MAX_RPM to EEPROM
          }
          if (currentRPM2 > MAX_RPM) {
            MAX_RPM = currentRPM2;
            saveMaxRPMToEEPROM();  //save updated MAX_RPM to EEPROM
          }
          //reset the tracking timer after updating
          rpmExceedStartTime = 0;
        }
      } else {
        //reset tracking time if RPM falls below MAX_RPM
        rpmExceedStartTime = 0;
      }
    } else {
      //reset tracking time if targetFanSpeed is not 100
      rpmExceedStartTime = 0;
    }

    //map fan speeds as percentage of MAX_RPM using floating-point arithmetic
    currentFanSpeed1 = (int)((currentRPM1 / MAX_RPM) * 100.0);
    currentFanSpeed2 = (int)((currentRPM2 / MAX_RPM) * 100.0);

    //clamp values to 100%
    if (currentFanSpeed1 > 100) {
      currentFanSpeed1 = 100;
    }
    if (currentFanSpeed2 > 100) {
      currentFanSpeed2 = 100;
    }
  }
}

void saveMaxRPMToEEPROM() {
  if (MAX_RPM <= 32767) {
    EEPROM.write(2, (byte)(MAX_RPM & 0xFF));        //low byte (0-255)
    EEPROM.write(3, (byte)((MAX_RPM >> 8) & 0xFF)); //high byte (0-255)
    EEPROM.commit();
  }
}


void adjustTargetFanSpeed() {
  targetFanSpeed -= 10;
  if (targetFanSpeed < 10) targetFanSpeed = 100;
  Serial.print("Target Fan Speed: ");
  Serial.print(targetFanSpeed);
  Serial.println("%");
}

void saveSpeedAndExitToAuto() {
  EEPROM.write(1, targetFanSpeed);
  EEPROM.write(0, AUTO);
  EEPROM.commit();
  currentMode = AUTO;
  settingModeActive = false;
  Serial.println("Target speed saved and exited to AUTO mode.");
}


//void updateDisplay() {
// display.clearDisplay();
//
// //draw Mode Indicator with Inverted Rectangle
// display.setTextSize(1);
// int textX = (currentMode == AUTO) ? 4 : 8;
// display.setCursor(textX, 3);
//
// //invert the box for the current mode by default
// display.fillRect(0, 0, 32, 12, SSD1306_WHITE);
// if (currentMode == SET && settingModeActive) {
//   if (millis() - lastBlinkTime > 500) {
//     blinkState = !blinkState;
//     lastBlinkTime = millis();
//   }
//   display.setTextColor(blinkState ? SSD1306_BLACK : SSD1306_WHITE, SSD1306_WHITE);
// } else {
//   display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
// }
// String screenMSG;
// if (currentMode == AUTO && currentTime < postPrintPurifying_EndTime && postPrintPurifying_EndTime > 0) { //currentTime will be updated in handleModeLogic function. you could also update it again before this line
//   //if in post-print purifying mode, display the remaining time
//   float remaining_secs = postPrintPurifying_EndTime - currentTime; //display remaining minutes or seconds when being in AUTO setting and in post-print purifying mode
//   if (remaining_secs < 60) {
//     screenMSG = String(remaining_secs, 0) + " s"; //no decimals for seconds
//   } else {
//     screenMSG = String(remaining_secs / 60, 0) + " m"; //1 decimal for minutes
//   }
// } else {
//   //default mode name display
//   screenMSG = currentMode == AUTO ? "AUTO" : currentMode == SET ? "SET" : "OFF";
// }
//
// //display the message
// display.print(screenMSG);
// display.setTextColor(SSD1306_WHITE);
//
// //center and display target speed RPM
// //string speedText = String(targetFanSpeed*MAX_RPM/100);
// String speedText = "---";
// int currentRPM = 0;
// if (currentFanSpeed1 > currentFanSpeed2) {
//   currentRPM = currentFanSpeed1 * MAX_RPM / 100;
// }
// else {
//   currentRPM = currentFanSpeed2 * MAX_RPM / 100;
// }
//
//
// if (currentRPM > 0 && fanon) {
//   speedText = String(currentRPM);
//   if (currentRPM < 1000) {
//     display.setCursor(5, 26);
//   }
//   else if (currentRPM < 10000) {
//     display.setCursor(3, 26);
//   }
//   else if (currentRPM >= 10000) {
//     display.setCursor(5, 26);
//     speedText = String(currentRPM / 1000) + "K";
//   }
//   else {
//     display.setCursor(0, 26);
//   }
//   display.println(speedText);
//   display.print("1/min");
// }
//
//
//
// int centeredX = 0;;
// String statusText = ((currentMode == AUTO && fanon) || (currentMode == SET && fanon)) ? String(targetFanSpeed) + "%" : "-OFF-";
//
//
// if (((currentMode == AUTO && fanon) || (currentMode == SET && fanon))) {
//   //centeredX=4;
//   if (targetFanSpeed < 100) {
//     centeredX = 7;
//   }
//   else {
//     centeredX = 3;
//   }
// }
// else {
//   centeredX = 0;
// }
// if (currentMode != OFF) {
//   int barWidth = 32;  //width of the bar
//   int targetHeight = 22;  //y-coordinate of the position to draw the target
//   display.drawLine(0, targetHeight + 1, 2 + barWidth - 1, targetHeight + 1, SSD1306_WHITE);
//
//   display.setCursor(centeredX, 14);
// }
// else {
//   display.setCursor(0, 16);
//   statusText = triggerActive ? "sensebed: DOWN" : "sensebed: LIFT";
// }
// display.print(statusText);
//
// drawBarGraph(currentFanSpeed1, currentFanSpeed2, targetFanSpeed);
// display.display();
//}

//void updateDisplay() {
// display.clearDisplay();
//
// //draw Mode Indicator with Inverted Rectangle
// display.setTextSize(1);
// int textX = (currentMode == AUTO) ? 4 : 8;
// display.setCursor(textX, 3);
//
// //invert the box for the current mode by default
// display.fillRect(0, 0, 32, 12, SSD1306_WHITE);
// if (currentMode == SET && settingModeActive) {
//   if (millis() - lastBlinkTime > 500) {
//     blinkState = !blinkState;
//     lastBlinkTime = millis();
//   }
//   display.setTextColor(blinkState ? SSD1306_BLACK : SSD1306_WHITE, SSD1306_WHITE);
// } else {
//   display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
// }
//
// String modeText = "";
// String fanDisplay = "";
//
// if (currentMode == AUTO && currentTime < postPrintPurifying_EndTime && postPrintPurifying_EndTime > 0) {
//   //if in post-print purifying mode
//   display.setCursor(1, 3); //move further outwards as "CLEAN" is longer than "AUTO"
//   modeText = "CLEAN";
//   unsigned long remaining_secs = postPrintPurifying_EndTime - currentTime;
//
//   //calculate minutes and seconds
//   unsigned long minutes = remaining_secs / 60;
//   unsigned long seconds = remaining_secs % 60;
//
//   //format time as "-mm:ss"
//   if (minutes < 10) fanDisplay += "0";  //add another 0 for single-digit minutes
//   fanDisplay += String(minutes) + ":";
//   if (seconds < 10) fanDisplay += "0";  //add another 0 for single-digit seconds
//   fanDisplay += String(seconds);
//   //fanDisplay = "-" + fanDisplay;  //add leading minus to clarify its remaining time
// } else {
//   //default mode and fan speed display
//   modeText = currentMode == AUTO ? "AUTO" : currentMode == SET ? "SET" : "OFF";
//
//   //center and display target fan speed or -OFF-
//   if (fanon) {
//     fanDisplay = String(targetFanSpeed) + "%";
//   } else {
//     fanDisplay = "-OFF-";
//   }
// }
//
// //display mode text
// display.print(modeText);
// display.setTextColor(SSD1306_WHITE);
//
// //display fan display text below mode text
// display.setCursor(0, 14);
// display.print(fanDisplay);
//
// //draw fan speed bar graph if not OFF
// if (currentMode != OFF) {
//   int barWidth = 32;
//   int targetHeight = 22;
//   display.drawLine(0, targetHeight + 1, 2 + barWidth - 1, targetHeight + 1, SSD1306_WHITE);
// }
//
// drawBarGraph(currentFanSpeed1, currentFanSpeed2, targetFanSpeed);
// display.display();
//}

//void updateDisplay() {
// display.clearDisplay();
//
// //draw Mode Indicator with Inverted Rectangle
// display.setTextSize(1);
// int textX = (currentMode == AUTO) ? 4 : 8;
// display.setCursor(textX, 3);
//
// //invert the box for the current mode by default
// display.fillRect(0, 0, 32, 12, SSD1306_WHITE);
// if (currentMode == SET && settingModeActive) {
//   if (millis() - lastBlinkTime > 500) {
//     blinkState = !blinkState;
//     lastBlinkTime = millis();
//   }
//   display.setTextColor(blinkState ? SSD1306_BLACK : SSD1306_WHITE, SSD1306_WHITE);
// } else {
//   display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
// }
//
// String modeText = "";
// String fanDisplay = "";
//
// if (currentMode == AUTO && currentTime < postPrintPurifying_EndTime && postPrintPurifying_EndTime > 0) {
//   //post-print purifying mode
//   display.setCursor(1, 3); //adjust cursor position for "CLEAN" as its different to "AUTO"
//   modeText = "CLEAN";
//   unsigned long remaining_secs = postPrintPurifying_EndTime - currentTime;
//
//   //calculate minutes and seconds
//   unsigned long minutes = remaining_secs / 60;
//   unsigned long seconds = remaining_secs % 60;
//
//   //format the remaining time as "mm:ss"
//   fanDisplay = "";
//   if (minutes < 10) fanDisplay += "0"; //pad single-digit minutes
//   fanDisplay += String(minutes) + ":";
//   if (seconds < 10) fanDisplay += "0"; //pad single-digit seconds
//   fanDisplay += String(seconds);
//
// } else if (currentMode == OFF) {
//   //senseBed message in OFF mode
//   modeText = "OFF";
//   //fanDisplay = triggerActive ? "senseBed: DOWN" : "senseBed: LIFT";
//   fanDisplay = triggerActive ? "senseBed:: DOWN" : "senseBed: LIFT";
// } else {
//   //normal mode text
//   modeText = currentMode == AUTO ? "AUTO" : currentMode == SET ? "SET" : "OFF";
//
//   //display target fan speed or -OFF- depending on mode
//   if (fanon) {
//     fanDisplay = String(targetFanSpeed) + "%";
//   } else {
//     fanDisplay = "-OFF-";
//   }
// }
//
// //display mode text
// display.print(modeText);
// display.setTextColor(SSD1306_WHITE);
//
// //display fan RPM below the mode text
// String speedText = "---";
// int currentRPM = 0;
//
// if (currentFanSpeed1 > currentFanSpeed2) {
//   currentRPM = currentFanSpeed1 * MAX_RPM / 100;
// } else {
//   currentRPM = currentFanSpeed2 * MAX_RPM / 100;
// }
//
// display.setCursor(0, 26); //adjust placement for fan RPM
// if (currentRPM > 0 && fanon) {
//   speedText = String(currentRPM);
//   if (currentRPM >= 10000) {
//     speedText = String(currentRPM / 1000) + "K"; //use K notation for high values
//   }
//   display.print(speedText);
//   display.print(" 1/min"); //display RPM unit
// } else {
//   display.print("");
// }
//
// //draw fan speed bar graph if not OFF
// if (currentMode != OFF) {
//   int barWidth = 32;  //width of the bar
//   int targetHeight = 22;  //y-coordinate of the position to draw the target
//   display.drawLine(0, targetHeight + 1, 2 + barWidth - 1, targetHeight + 1, SSD1306_WHITE);
// }
//
// //display fanDisplay text below the mode text
// display.setCursor(0, 14);
// display.print(fanDisplay);
//
// drawBarGraph(currentFanSpeed1, currentFanSpeed2, targetFanSpeed);
// display.display();
//}


void updateDisplay() {
  display.clearDisplay();

  //draw Mode Indicator with Inverted Rectangle
  display.setTextSize(1);
  int textX = (currentMode == AUTO) ? 4 : 8;
  display.setCursor(textX, 3);

  //invert the box for the current mode by default
  display.fillRect(0, 0, 32, 12, SSD1306_WHITE);
  if (currentMode == SET && settingModeActive) {
    if (millis() - lastBlinkTime > 500) {
      blinkState = !blinkState;
      lastBlinkTime = millis();
    }
    display.setTextColor(blinkState ? SSD1306_BLACK : SSD1306_WHITE, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  }

  String modeText = "";
  String fanDisplay = "";

  if (currentMode == AUTO && currentTime < postPrintPurifying_EndTime && postPrintPurifying_EndTime > 0) {
    //post-print purifying mode
    display.setCursor(1, 3); //adjust cursor position for "CLEAN"
    modeText = "CLEAN";
    unsigned long remaining_secs = postPrintPurifying_EndTime - currentTime;

    //calculate minutes and seconds
    unsigned long minutes = remaining_secs / 60;
    unsigned long seconds = remaining_secs % 60;

    //format the remaining time as "mm:ss"
    fanDisplay = "";
    if (minutes < 10) fanDisplay += "0"; //pad single-digit minutes
    fanDisplay += String(minutes) + ":";
    if (seconds < 10) fanDisplay += "0"; //pad single-digit seconds
    fanDisplay += String(seconds);

  } else if (currentMode == OFF) {
    //senseBed message in OFF mode
    modeText = "OFF";
    fanDisplay = triggerActive ? "senseBed: DOWN" : "senseBed: LIFT";
  } else {
    //normal mode text
    modeText = currentMode == AUTO ? "AUTO" : currentMode == SET ? "SET" : "OFF";

    //display target fan speed or -OFF-
    if (fanon) {
      fanDisplay = String(targetFanSpeed) + "%";
    } else {
      fanDisplay = "-OFF-";
    }
  }

  //display mode text
  display.print(modeText);
  display.setTextColor(SSD1306_WHITE);

  //adjust the cursor position dynamically for fanDisplay
  int fanDisplayX = 0;
  if (currentMode != OFF) {
    if (fanDisplay.length() < 5) { //shorter text like percentages < 100%
      fanDisplayX = (fanDisplay.length() == 4) ? 4 : 8; //adjust for lengths like "80%" or "9%"
    }
  }
  display.setCursor(fanDisplayX, 14);
  display.print(fanDisplay);

  //display fan RPM below the mode text
  String speedText = "---";
  int currentRPM = 0;

  if (currentFanSpeed1 > currentFanSpeed2) {
    currentRPM = currentFanSpeed1 * MAX_RPM / 100;
  } else {
    currentRPM = currentFanSpeed2 * MAX_RPM / 100;
  }

  display.setCursor(0, 26); //adjust placement for fan RPM
  if (currentRPM > 0 && fanon) {
    speedText = String(currentRPM);
    if (currentRPM >= 10000) {
      speedText = String(currentRPM / 1000) + "K"; //use K notation for high values
    }
    display.print(speedText);
    display.print(" 1/min"); //display RPM unit
  } else {
    display.print("");
  }

  //draw fan speed bar graph if not OFF
  if (currentMode != OFF) {
    int barWidth = 32;  //width of the bar
    int targetHeight = 22;  //y-coordinate of the position to draw the target
    display.drawLine(0, targetHeight + 1, 2 + barWidth - 1, targetHeight + 1, SSD1306_WHITE);
  }

  drawBarGraph(currentFanSpeed1, currentFanSpeed2, targetFanSpeed);
  display.display();
}




void drawBarGraph(int speed1, int speed2, int target) {
  //map speeds and target to vertical bar heights
  int barHeight1 = map(speed1, 0, 100, 106, 45);  //fan1 speed to bar height, now ending at 106 px
  int barHeight2 = map(speed2, 0, 100, 106, 45);  //fan2 speed to bar height, now ending at 106 px
  int targetHeight = map(target, 0, 100, 106, 45); //target speed height, now ending at 106 px

  int barWidth = 12;  //bar width
  int barSpacing = 18; //spacing between bars

  //draw and fill bar for Fan1 with rounded corners
  display.fillRoundRect(2, barHeight1, barWidth, 106 - barHeight1, 3, SSD1306_WHITE);
  display.drawRoundRect(2, 45, barWidth, 106 - 45, 3, SSD1306_WHITE);

  //draw and fill bar for Fan2 with rounded corners
  display.fillRoundRect(2 + barSpacing, barHeight2, barWidth, 106 - barHeight2, 3, SSD1306_WHITE);
  display.drawRoundRect(2 + barSpacing, 45, barWidth, 106 - 45, 3, SSD1306_WHITE);

  //conditionally draw the target line for Fan1
  if ((currentMode == AUTO && !fanon) || (currentMode == SET) || (currentMode == OFF) || (currentFanSpeed1 < 0.8 * targetFanSpeed)) {
    if (targetHeight >= 47) {
      display.drawLine(2, targetHeight, 2 + barWidth - 1, targetHeight, targetHeight >= barHeight1 ? SSD1306_BLACK : SSD1306_WHITE);
    }
  }

  //conditionally draw the target line for Fan2
  if ((currentMode == AUTO && !fanon) || (currentMode == SET) || (currentMode == OFF) || (currentFanSpeed2 < 0.8 * targetFanSpeed)) {
    if (targetHeight >= 47) {
      display.drawLine(2 + barSpacing, targetHeight, 2 + barSpacing + barWidth - 1, targetHeight, targetHeight >= barHeight2 ? SSD1306_BLACK : SSD1306_WHITE);
    }
  }

  display_hum_temp();

  display.display();
}
