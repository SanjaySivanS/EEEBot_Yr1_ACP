#include <WiFi.h>

#include <PubSubClient.h>

#include <Wire.h>

#include <NewPing.h>


#ifdef __cplusplus
  extern "C" {
 #endif

  uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();

 

#define TRIGGER_PIN 12

#define ECHO_PIN 14

#define MAX_DISTANCE 400

 

// replace the next variables with your SSID/Password combination

const char* ssid = "LB12Rifi";

const char* password = "SSCL@LB12";                

 

// add your MQTT Broker IP address, example:

//const char* mqtt_server = "192.168.1.144";

const char* mqtt_server = "192.168.137.46";

 

WiFiClient espClient;

PubSubClient client(espClient);

long lastMsg = 0;

char msg[50];

int value = 0;

 

int ledPin = 2;

 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int counterValue;

void setup() {

  Serial.begin(115200);

 

  setup_wifi();

  client.setServer(mqtt_server, 1883);

  client.setCallback(callback);

 

  pinMode(ledPin, OUTPUT);

  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent);

}

 

void setup_wifi() {

  delay(10);

  // we start by connecting to a WiFi network

  Serial.println();

  Serial.print("Connecting to ");

  Serial.println(ssid);

 

  WiFi.begin(ssid, password);

 

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.print(".");

  }

 

  Serial.println("");

  Serial.println("WiFi connected");

  Serial.println("IP address: ");

  Serial.println(WiFi.localIP());

}

 

void callback(char* topic, byte* message, unsigned int length) {

  Serial.print("Message arrived on topic: ");

  Serial.print(topic);

  Serial.print(". Message: ");

  String messageTemp;

 

  for (int i = 0; i < length; i++) {

    Serial.print((char)message[i]);

    messageTemp += (char)message[i];

  }

  Serial.println();

 

  // feel free to add more if statements to control more GPIOs with MQTT

 

  // if a message is received on the topic esp32/output, you check if the message is either "on" or "off".

  // changes the output state according to the message

  if (String(topic) == "esp32/output") {

    Serial.print("Changing output to ");

    if(messageTemp == "on"){

      Serial.println("on");

      digitalWrite(ledPin, HIGH);

    }

    else if(messageTemp == "off"){

      Serial.println("off");

      digitalWrite(ledPin, LOW);

    }

  }

}

 

void reconnect() {

  // loop until we're reconnected

  while (!client.connected()) {

    Serial.print("Attempting MQTT connection...");

    // attempt to connect

    if (client.connect("ESP8266Client")) {

      Serial.println("connected");

      // subscribe

      client.subscribe("esp32/output");

    } else {

      Serial.print("failed, rc=");

      Serial.print(client.state());

      Serial.println(" try again in 5 seconds");

      // wait 5 seconds before retrying

      delay(5000);

    }

  }

}

void receiveEvent(int howMany)
{
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  counterValue = Wire.read();    // receive byte as an integer
  Serial.println(counterValue);         // print the integer
}

void loop() {

  delay(50);

  if (!client.connected()) {

    reconnect();

  }

  client.loop();

 

  long now = millis();

  if (now - lastMsg > 5000) {

    lastMsg = now;

   

    float distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)

 

    Serial.print("Distance: ");

    Serial.print(distance);

    Serial.println("cm");

 

    char distString[8];

    dtostrf(distance, 1, 2, distString);

    Serial.print("Distance: ");

    Serial.println(distString);

    client.publish("esp32/distanceCE", distString);

    Serial.print("Temperature: ");
  
    // Convert raw temperature in F to Celsius degrees
    float temperature = (temprature_sens_read() - 32) / 1.8;

    char celsiusString[8];

    dtostrf(temperature, 1, 2, celsiusString);

    Serial.print("Temperature: ");

    Serial.println(celsiusString);

    client.publish("esp32/temperatureCE", celsiusString);

    Serial.print("Temperature: ");


    char rotorString[8];

    dtostrf(counterValue, 1, 2, rotorString);

    Serial.print("Rotor-Count: ");

    Serial.println(rotorString);

    client.publish("esp32/rotorCE", rotorString);

    Serial.print("Rotor-Count: ");
 

  }

}
