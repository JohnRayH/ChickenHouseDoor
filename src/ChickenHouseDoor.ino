#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 1

const char* WIFI_SSID = "your wifi name";
const char* WIFI_PASSWORD = "wifipassword";

const char* mqttServer = "192.168.2.103";  // The IP of your MQTT broker
const int mqttPort = 1883;
const char* mqttUser = "username"; // Add a user to HA.
const char* mqttPassword = "password";
const char* mqttName = "chickenhouse_door";

const char* state_topic = "home/chickenhouse/door";
const char* command_topic = "home/chickenhouse/door/set";
const char* availability_topic = "home/chickenhouse/door/available";

const char* payload_on = "ON";
const char* payload_off = "OFF";

long lastMsg = 0;

unsigned long previousMillis = 0;  // Time open or close sequence started.
const long moveTime = 4000; // how long it takes to move the actuator.
bool doorState = false;
bool isMoving = false;

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, HIGH);
  digitalWrite(output27, HIGH);

   // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  Serial.begin(115200);
  setup_wifi();
  setup_mqtt();
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_mqtt() {
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
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

  // Changes the output state according to the message
  if (String(topic) == command_topic) {
      if (messageTemp == "ON"){
        openDoor();
      }      
      if (messageTemp == "OFF"){
        closeDoor();
      }    
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttName, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(availability_topic);
      client.subscribe(state_topic);
      client.subscribe(command_topic);
      client.publish(availability_topic, "online");
      
      // read the last door state from flash memory
      doorState = EEPROM.read(0);      
    
      if(doorState == true){
        client.publish(state_topic, "ON");
      }
      else{
        client.publish(state_topic, "OFF");
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// This is supposed to turn off the relays after the actuator is done moving.
void checkRelays() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= moveTime && isMoving) {
    isMoving = false;
    Serial.println("Turning off the relays.");
    stop();
  }
}

void startMoving() {
  previousMillis = millis();
  isMoving = true;
}

void stop() {
  Serial.println("Stopping");
  digitalWrite(output26, HIGH);
  digitalWrite(output27, HIGH);
  isMoving = false;
}

void openDoor() {
  if (isMoving) {
    stop();
    Serial.println("Was moving. Waiting 100 ms");
      delay(100);
  }
  Serial.println("Opening Door");
  startMoving();
  doorState = true;
  digitalWrite(output26, HIGH);
  digitalWrite(output27, LOW);
}

void closeDoor() {
  if (isMoving) {
    stop();
    Serial.println("Was moving. Waiting 100 ms");    
      delay(100);
  }
  Serial.println("Closing Door");
  startMoving();
  doorState = false;
  digitalWrite(output26, LOW);
  digitalWrite(output27, HIGH);
}

void writeToEprom(bool doorState){
    EEPROM.write(0, doorState);
    EEPROM.commit();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  checkRelays();
}



