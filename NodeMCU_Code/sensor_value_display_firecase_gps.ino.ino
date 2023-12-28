#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;  // The TinyGPS++ object

SoftwareSerial ss(2, 0); // The serial connection to the GPS device, (Rx,Tx)

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "chandharlabs_4G"
#define WIFI_PASSWORD "9159808783"

// Insert Firebase project API Key
#define API_KEY "AIzaSyDP8FcGMl98KwlclD4s-b9JPXtUIaLYmKo"


// Insert RTDB URLefine the RTDB URL */

#define DATABASE_URL "https://livestock-monitoring-device-default-rtdb.asia-southeast1.firebasedatabase.app/"

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
String xPath = "/x";
String yPath = "/y";
String zPath = "/z";
String timePath = "/timestamp";
String latPath = "/lat";
String lonPath = "/lon";

// Parent Node (to be updated in every loop)
String parentPath;

FirebaseJson json;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variable to save current epoch time
int timestamp;

// BME280 sensor
//Adafruit_BME280 bme; // I2C
float latitude;
float longitude;
String lat_str;
String lon_str;


unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 1000;

int count = 0;
bool signupOK = false;


// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}
Adafruit_ADXL345_Unified accel;
void setup(){
  ss.begin(9600);
  Serial.begin(115200);
  // Initialize accelerometer
  if (!accel.begin()) {
    Serial.println("Accelerometer not found.");
    while (1);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  timeClient.begin();

  /* Assign the api key (required) */
  config.api_key =  API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

    // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  
  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";

}

void loop(){
//  Serial.println(ss.available());
  if (ss.available() > 0)
  {
 //   Serial.println(gps.encode(ss.read()));
    if (gps.encode(ss.read()))
    {
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        lat_str = String(latitude , 6);
        longitude = gps.location.lng();
        lon_str = String(longitude , 6);
//        Serial.print("Lat:");
//        Serial.println(lat_str);

      }
    }
  }
  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

    //Get current timestamp
    timestamp = getTime();
    Serial.print ("time: ");
    Serial.println (timestamp);

    parentPath= databasePath + "/" + String(timestamp);

    // Read accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);
  
  // Extract x, y, and z values
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  // Create a JSON object with the data
  json.set(xPath.c_str(), String(x));
  json.set(yPath.c_str(), String(y));
  json.set(zPath.c_str(), String(z));
  json.set(latPath.c_str(), lat_str);
  json.set(lonPath.c_str(), lon_str);
  json.set(timePath, String(timestamp));

  Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
  }
}
