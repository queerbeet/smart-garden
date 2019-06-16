/* YourDuino Multiple DS18B20 Temperature Sensors on 1 wire
  Connections:
  DS18B20 Pinout (Left to Right, pins down, flat side toward you)
  - Left   = Ground
  - Center = Signal (Pin 2):  (with 3.3K to 4.7K resistor to +5 or 3.3 )
  - Right  = +5 or +3.3 V

   Questions: terry@yourduino.com
   V1.01  01/17/2013 ...based on examples from Rik Kretzinger

  /*-----( Import needed libraries )-----*/
// Get 1-wire Library here: http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <OneWire.h>

//Get DallasTemperature Library here:  http://milesburton.com/Main_Page?title=Dallas_Temperature_Control_Library
#include <DallasTemperature.h>

// WIFI
#include "esp_wpa2.h"
#include <WiFi.h>

// LED Library
#include <NeoPixelBus.h> //Library for LED-strip. See https://github.com/adafruit/Adafruit_NeoPixel for more information. 
#define colorSaturation 128 //maximum brightness for LEDs on LED-strip. (lower brigtness means lower power consumption, and less load on the DC/DC converter used for converting 12V from the solar charger controler to required 5V for esp32, LED-strip and sensors. 

//predifined Colors
RgbColor red(colorSaturation, 0, 0); //the color red
RgbColor green(0, colorSaturation, 0); //green
RgbColor blue(0, 0, colorSaturation); //blue
RgbColor white(colorSaturation); //white
RgbColor black(0); //black

//login data for wifi network. replace with your login data.
const char* ssid = "eduroam"; //wifi network name
#define EAP_USERNAME "dnu4122@students.fhv.at" //user name
#define EAP_ID EAP_USERNAME
#define EAP_PASSWORD "VQKSUPR3YfJ9" //wifi password: remember to not make the password public when sharing the code.

const char *host = "api.thingspeak.com"; //host of thinkspeak services
String apiKey = "9M0MUEHWKBW641NQ"; //replace with your channel's thingspeak API key


/*-----( Declare Constants and Pin Numbers )-----*/
#define ONE_WIRE_BUS_PIN 4 //pin for temperature bus

//pins for ultrasonic distance sensor
#define TRIG_PIN 26 // Pinbelegung
#define ECHO_PIN 19 // Pinbelegung
#define MAX_DIST 400 // maximum distance detectable. 
long distanceTrigger=100; // Abstand der unterschritte werden muss, um die Anzeige auszulösen

/* Setup LED Strip */
const uint8_t PixelPin = 2;  // Pin for pixel bus, the data line to the LED-strip. make sure to set this to the correct pin, ignored for Esp8266
const uint16_t pixelNumTotal = 79; //Total number of LEDs on LED-strip.
const uint16_t tPixelNum = 39; //number of LEDs for temperature output. we broke one LED. be careful with the soldering connections on the LED-strip. the copper foil on the LED-strip is not very attached.
const uint16_t hPixelNum = 40; //number of LEDs for humidity output
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(pixelNumTotal, PixelPin); //Initialize LED-strip. Use lower baudrate because of relatively long and unshielded bus wired to led-strip. shielded wires are recommendet. To long wired lead to problems.
//NeoPixelBus<NeoRgbwFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

//timing variables
bool blinkState=false; //variable that gets changed every time blinkTimeSec has passed. if variable is high, blik color1, else blink color two.
float blinkTimeSec = 0.2; //blink time. is half of the blink period
float displayTimeSec=10.; //time LED-strip shows the humidity and temperature values
float showTimeSec=12*60.; //time interval at which garden status gets showed automatically.
bool displayGardenStatus=false; //if true, display is on.

//minimum and maximum values for displaying status
float minTemp = -10.; //cutoff temperature at this temperature all temp-leds ar off
float maxTemp = 38.; //max temp. at this temp all temp-leds are on
float minHumidity = 0.; //same for humidity
float maxHumidity = 100.; //same
float dangerHumidity=20.;
float dangerTemp=5.;

//array for storing show-events in last 24HoursoEdvka, one element for every pixel
int eventsDay[tPixelNum];
int eventsWeek[hPixelNum];
//float timeIntervalDay=

/*-----( Declare objects )-----*/
// Setup a oneWire instance to communicate with any OneWire devices. Temerature is read with OneWire.
OneWire oneWire(ONE_WIRE_BUS_PIN);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Assign the addresses of your 1-Wire temp sensors.
// See the tutorial on how to obtain these addresses: http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html
DeviceAddress Probe01 = { 0x28, 0x8C, 0x2F, 0x27, 0x00, 0x00, 0x80, 0x64 };
DeviceAddress Probe02 = { 0x28, 0xB5, 0x5B, 0x1F, 0x00, 0x00, 0x80, 0x39 };
DeviceAddress Probe03 = { 0x28, 0x84, 0x87, 0x26, 0x00, 0x00, 0x80, 0x47 };
DeviceAddress Probe04 = { 0x28, 0xFF, 0x78, 0x29, 0x84, 0x16, 0x03, 0xA0 };

void setup()   /****** SETUP: RUNS ONCE *************************************************************************************/
{

  // start serial port to show results
  Serial.begin(9600); //initialize serial port with baudrate. this is only for debugging. for real use, no serial port prints are reqired, and will only use unnessecaraly power.
  delay(10); //wait 10ms
  Serial.print("connecting to ");


  // this resets all the neopixels to an off state
  strip.Begin(); //initialize LED-strip
  strip.Show(); //turn all LEDs off/make all black
  /*
      WiFi.mode(WIFI_STA); //set wifi mode
      Serial.println(ssid); //print wifi network name/id
      // WPA2 enterprise magic starts here
      WiFi.disconnect(true);
      esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ID, strlen(EAP_ID));
      esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
      esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
      esp_wifi_sta_wpa2_ent_set_disable_time_check(true);
      esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();
      esp_wifi_sta_wpa2_ent_enable(&config);

      // WPA2 enterprise magic ends here

      WiFi.begin(ssid); connect to wifi network

      while (WiFi.status() != WL_CONNECTED) { //check if wifi connection has succeedet, then wait .5 seconds and print a point, then ceck again unitil connection is obtained
          delay(500);
          Serial.print(".");
      }

      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());


    // initialize serial communication at 9600 bits per Second:
    // Serial.begin(9600);

    Serial.print("Initializing Temperature Control Library Version ");
    Serial.println(DALLASTEMPLIBVERSION);

  */

  // Initialize the Temperature measurement library
  sensors.begin();

  // set the resolution to 10 bit (Can be 9 to 12 bits .. lower is faster)
  sensors.setResolution(Probe01, 10);
  sensors.setResolution(Probe02, 10);
  sensors.setResolution(Probe03, 10);
  sensors.setResolution(Probe04, 10);

}//--(end setup )---

void loop()   /****** LOOP: RUNS CONSTANTLY ******************************************************************************************/
{
  //Read the Humidity Values from Analog Input Pins.
  int sensorValue0 = analogRead(34); //read the input on analog pin 0
  int sensorValue1 = analogRead(35); //read the input on analog pin 1
  int sensorValue2 = analogRead(32); //read the input on analog pin 2
  int sensorValue3 = analogRead(33); //read the input on analog pin 3
  //map and invert the raw humidity values read from Analog pins to a range between 0 ant 100%.
  sensorValue0 = map(sensorValue0, 4095, 1200, 0, 100);
  sensorValue1 = map(sensorValue1, 4095, 1200, 0, 100);
  sensorValue2 = map(sensorValue2, 4095, 1200, 0, 100);
  sensorValue3 = map(sensorValue3, 4095, 1200, 0, 100);

  //get number of Temperature sensors on Bus. Temperature sensors are read not with analog input pins, but are on the same digital bus wire, managed by the DallasTemperature library
  int numTempSensorsOnBus = sensors.getDeviceCount();
  //serial print for debugging
  //delay(1000);
  Serial.println();
  Serial.print("Number of Devices found on bus = ");
  Serial.println(numTempSensorsOnBus);
  Serial.print("Getting temperatures ... ");
  Serial.println();

  // Command all devices on bus to read temperature
  sensors.requestTemperatures();

  // Gibt einen Wert zurück und gibt den Wert gleichzeitig aus
  Serial.print("Probe 01 temperature is:   ");
  float temp1 = printTemperature(Probe01);
  Serial.println();

  Serial.print("Probe 02 temperature is:   ");
  float temp2 = printTemperature(Probe02);
  Serial.println();

  Serial.print("Probe 03 temperature is:   ");
  float temp3 = printTemperature(Probe03);
  Serial.println();

  Serial.print("Probe 04 temperature is:   ");
  float temp4 = printTemperature(Probe04);
  Serial.println();

  Serial.println("--------------------------");

  Serial.print("Probe 01 humidity is:         ");
  Serial.println(sensorValue0);  //print out the value0 you read
  Serial.print("Probe 02 humidity is:         ");
  Serial.println(sensorValue1);  //print out the value1 you read
  Serial.print("Probe 03 humidity is:         ");
  Serial.println(sensorValue2);  //print out the value2 you read
  Serial.print("Probe 04 humidity is:         ");
  Serial.println(sensorValue3);  //print out the value3 you read

  // Gibt den Feuchtigkeitswert an das LED-Band weiter
  int averageHumidity = (sensorValue0 + sensorValue1 + sensorValue2 + sensorValue3) / 4.;
  Serial.print("Durchschnittsfeuchtigkeit ist:   ");
  Serial.println(averageHumidity);

  //Gibt den Temperaturwert an das LED-Band weiter
  float averageTemp = (temp1 + temp2 + temp3 + temp4) / 4.;
  Serial.print("Durchschnittstemperatur ist:   ");
  Serial.println(averageTemp);

  timing(); //update timing variables
  checkDistance();
  Serial.println(blinkState);
  Serial.println(displayGardenStatus); 
  if (displayGardenStatus){ //if dispayGardenStatus is true, display show temp and humidity values.
    showValuesBlinkH(0,hPixelNum,averageHumidity,minHumidity,maxHumidity,dangerHumidity);
    showValuesBlinkT(hPixelNum,tPixelNum,averageTemp,minTemp,maxTemp,dangerTemp);
  }
  else{
    clearStrip();
  }
    strip.Show();
  String path = "/update?api_key=" + apiKey + "&field1=" + (String)(printTemperature(Probe01)) + "&field2=" + (String)(printTemperature(Probe02)) +
                "&field3=" + (String)(printTemperature(Probe03)) + "&field4=" + (String)(printTemperature(Probe04)) +
                "&field5=" + (String)(sensorValue0) + "&field6=" + (String)(sensorValue1) +
                "&field7=" + (String)(sensorValue2) + "&field8=" + (String)(sensorValue3);
  //espert.wifi.getHTTP(host, path.c_str());

  WiFiClient client;
  if (!client.connect(host, 80)) {
    Serial.println("connection to host failed");
    return;

  }

  // This will send the request to the server
  client.print(String("GET ") + path + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  delay(1);
}//--(end main loop )--

//timing variables, used to store the time difference since last time the event occured. 
long timeLastBlink=millis();
long timeLastShowAction=millis();
long timeDisplayStart=millis();

void beginShowStatus(){ //call this function to start display garden status. The counters are set to zero, and the show status variable is set to true for the desired period
  timeLastShowAction=millis();
  timeDisplayStart=millis();
  displayGardenStatus=true;
}

void timing(){ //time control. blink State gets permanently switched on and off again. display garden status gets true if a 
  if (millis()-timeLastBlink>=long(blinkTimeSec*1000.)){ //sets blink state variable. gets constantly switched on and off. and is therefore always ready to use.
    timeLastBlink=millis();
    blinkState=!blinkState;
  }
  if (millis()-timeLastShowAction>=long(showTimeSec*1000.)){ //sets display garden status true every time showTimeSec has passed, also the timing variable timeDisplay start is set to current time.
    beginShowStatus();
  }
  if (millis()-timeDisplayStart>=long(displayTimeSec*1000.)){ //if the desired time (displayTimeSec) has passed, the display gets switched off again. 
    displayGardenStatus=false;
  }
}
void clearStrip(){
  for (int i=0; i<pixelNumTotal; i++){
    strip.SetPixelColor(i,black);
  }
}
/*-----( Declare User-written Functions )-----*/
float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  float returnTemp = -1.0;
  if (tempC == -127.00)
  {
    Serial.print("Error getting temperature  ");
  }
  else
  {
    Serial.print("C: ");
    Serial.print(tempC);
    //Serial.print(" F: ");
    //Serial.print(DallasTemperature::toFahrenheit(tempC));
    returnTemp = tempC;
  }
  return returnTemp;
}// End printTemperature



// nicht-blinkende werteanzeige. Malt einen Balken zwischen pixelStart und pixelCount+pixelStart, der den Wert val ziwschen valmin und valmax anzeigt.
void showValues(int pixelStart, int pixelCount, float val, float minval, float maxval) {
  int ledOnIndex = map(val, minval, maxval, pixelStart, pixelCount+pixelStart);
  int colval = map(val, minval, maxval, 0, colorSaturation);
  RgbColor col(colval, 0, colorSaturation - colval);
  for (int i = pixelStart; i < pixelCount + pixelStart; i++) {
    if (i < ledOnIndex) {
      strip.SetPixelColor(i, col);
    }
    else {
      strip.SetPixelColor(i, black);
    }
  }

}

// blink-anzeige. Malt einen Balken zwischen pixelStart und pixelCount+pixelStart, der den Wert val ziwschen valmin und valmax anzeigt. Wenn der Wert val k
void showValuesBlinkH(int pixelStart, int pixelCount, float val, float minval, float maxval, float tolow) {
  int ledOnIndex = map(val, minval, maxval, pixelStart, pixelCount + pixelStart);
  int dangerIndex = map(tolow, minval, maxval, pixelStart, pixelCount + pixelStart);
  int colval = map(val, minval, maxval, 0, colorSaturation);
  RgbColor col(colorSaturation - colval, colval, colval / 4);
  RgbColor colblink(colorSaturation, 0, colorSaturation);
  for (int i = pixelStart; i < tPixelNum + pixelStart; i++) {
    if (i < ledOnIndex) {
      if (ledOnIndex < dangerIndex) {
        if (blinkState) {
          strip.SetPixelColor(i, col);
        }
        else {
          strip.SetPixelColor(i, black);
        }
      }
      else {
        strip.SetPixelColor(i, col);
      }
    }
    else {
      strip.SetPixelColor(i, black);
    }
  }
}
void showValuesBlinkT(int pixelStart, int pixelCount, float val, float minval, float maxval, float tolow) {
  int ledOnIndex = map(val, minval, maxval, pixelStart, pixelCount + pixelStart);
  int dangerIndex = map(tolow, minval, maxval, pixelStart, pixelCount + pixelStart);
  int colval = map(val, minval, maxval, 0, colorSaturation);
  RgbColor col(colval,0,colorSaturation - colval);
  RgbColor colblink(colorSaturation, colorSaturation, 0);
  for (int i = pixelStart; i < tPixelNum + pixelStart; i++) {
    if (i < ledOnIndex) {
      if (ledOnIndex < dangerIndex) {
        if (blinkState) {
          strip.SetPixelColor(i, col);
        }
        else {
          strip.SetPixelColor(i, colblink);
        }
      }
      else {
        strip.SetPixelColor(i, col);
      }
    }
    else {
      strip.SetPixelColor(i, black);
    }
  }
}

void checkDistance(){
  long distance=getDistance();
  Serial.print("distance:   ");
  Serial.println(distance);
  if (distance<distanceTrigger){
    beginShowStatus();
  }
}


long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delay(5);
  digitalWrite(TRIG_PIN, HIGH);
  delay(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = (duration/2) * 0.03432;
  delay(50);
  return distance;
}
//*********( THE END )***********
