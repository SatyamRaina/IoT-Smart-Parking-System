#include <SimpleDHT.h>                   // Data ---> D3 VCC ---> 3V3 GND ---> GND
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WiFi parameters
#define WLAN_SSID       "SATYAMRaina"
#define WLAN_PASS       "SatyamR31"

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "SatyamRaina"
#define AIO_KEY         "aio_MxLN33Gg1HO6a2U2CQ4gBlTyx5Da"

WiFiClient client;
//Details for establishing a connection
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish Ultrasonic = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/UltrasonicSens");
Adafruit_MQTT_Publish Humidity1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity1");

#define echoPin1 2 // Echo Pin for sensor 1
#define trigPin1 3 // Trigger Pin for sensor 1
#define echoPin2 4 // Echo Pin for sensor 2 
#define trigPin2 5 // Trigger Pin for sensor 2
#define echoPin3 6 // Echo Pin for sensor 3
#define trigPin3 7 // Trigger Pin for sensor 3
#define echoPin4 9 // Echo Pin for sensor 4
#define trigPin4 8 // Trigger Pin for sensor 4

long duration1, distance1; // Duration used to calculate distance
long duration2, distance2;
long duration3, distance3;
long duration4, distance4;

int count = 0;
int freeSlot = 0;

void setup() {
  Serial.begin(9600);  //Starting Serial terminal
  pinMode(trigPin1, OUTPUT); // trigger pin as output
  pinMode(echoPin1, INPUT);  // echo pin as input
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  Serial.println(F("Adafruit IO Example"));
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

  // connect to adafruit io
  connect();

}

// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if (ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void loop() {
  // ping adafruit io a few times to make sure we remain connected
  if (! mqtt.ping(3)) {
    // reconnect to adafruit io
    if (! mqtt.connected())
      connect();
  }
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);

  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH);
  dht11.read(&temp, &hum, NULL);

  //distance = (high level timevelocity of sound (340M/S) / 2,
  //in centimeter = uS/58
  distance1 = duration1 / 58.2;
  if (distance1 < 10)
    distance1 = 1;
  else distance1 = 0;

  distance2 = duration2 / 58.2;
  if (distance2 < 10)
    distance2 = 1;
  else distance2 = 0;

  distance3 = duration3 / 58.2;
  if (distance3 < 10)
    distance3 = 1;
  else distance3 = 0;

  distance4 = duration4 / 58.2;
  if (distance4 < 10)
    distance4 = 1;
  else distance4 = 0;

  // add the result from all sensor to count total car
  count = distance1 + distance2 + distance3 + distance4;;

  // free slot = total slot - total car
  freeSlot = 4 - count;
  // number of total slot is sent to raspberry pi using usb
  Serial.println(freeSlot);
  // the status is updated every 30 seconds.
  delay(30000);
  freeSlot = 0;
  distance1 = 0;
  distance2 = 0;
  distance3 = 0;
  distance4 = 0;
}

}
