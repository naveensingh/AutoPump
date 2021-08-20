#include <Arduino.h>

#include <NewPing.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>
#include <Arduino_JSON.h>

#include "AutoPump.h"

// UDP parameters
WiFiUDP Udp;
unsigned int udpPort = 2390;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

// IPAddress broadcast
IPAddress broadcast = IPAddress(224, 0, 1, 3);

// Setup Ultrasonic sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Update the following depending on your water tank size
int TANK_HEIGHT = 114;
int MAX_DEPTH = 40;
int MIN_DEPTH = 18;

// When user override has led to overloaded tank.
int MANUAL_OVERRIDE_OVERLOADED_DEPTH = 18;
int isPumpRunning = false;

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 10;

unsigned long readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
unsigned long total = 0;                  // the running total
unsigned long smoothedDepth = 0;         // the smoothedDepth

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void setup() {
    Serial.begin(115200);

    // Connect IO pins
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(OVERRIDE_MANUAL_OVERRIDE_RELAY_PIN, OUTPUT);

    // Turn off motor just in case it was on.
    digitalWrite(RELAY_PIN, HIGH);
    // Initialise Relay 2 normally closed.
    digitalWrite(OVERRIDE_MANUAL_OVERRIDE_RELAY_PIN, HIGH);

    // Initialize all the readings to 0
    for (unsigned long &reading : readings) {
        reading = 0;
    }

    // Init network
    // Setup hotspot
    delay(500);
    WiFi.mode(WIFI_AP_STA);     // Changing ESP9266 wifi mode to AP + STATION
    // Starting AP on given credential
    WiFi.softAP(REPEATER, PASSWORD, AP_CHANNEL, false, AP_MAX_CONNECTION);
    Serial.print("Repeater AP IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("");
    delay(1500);

    // Connect network
    WiFi.begin(SSID, PASSWORD);
    WiFi.setAutoReconnect(true);
    WiFi.setAutoConnect(true);
    WiFi.setOutputPower(20.5);

    // Retry five times. Do not wait indefinitely!
    for (int i = 0; i < 5; i++) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Connecting...");
        }
        delay(1000);
    }

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi failed.");
    }
    Serial.println("Connected!");

    MDNS.begin(HOSTNAME);
    Serial.println("mDNS responder started");

    delay(10);
    Udp.begin(udpPort);
    Serial.println("Ultrasonic Sensor HC-SR04 setup complete.");

    // Setup Wifi Updater
    httpUpdater.setup(&httpServer);
    httpServer.begin();

    MDNS.addService("http", "tcp", 80);
    Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", HOSTNAME);
}

/**
 * Smooths sensor readings over time.
 * @return Smoothed sensor reading.
 */
unsigned long getSmoothedDepth() {
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = sonar.ping_cm();
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
    }

    return total / numReadings;
}

void setPumpState(int pumpOn) {
    // Delay to avoid switching too frequently
    delay(1000);
    if (pumpOn == true) {
        isPumpRunning = true;
        digitalWrite(RELAY_PIN, LOW);
    } else if (pumpOn == false) {
        isPumpRunning = false;
        digitalWrite(RELAY_PIN, HIGH);
    }
}

/**
 * Turn ON/OFF motor depending on water level depth.
 *
 * @param depth
 */
void maintainWaterLevel(unsigned long depth) {
    // Start pump if water level is below a certain level.
    if (depth > MAX_DEPTH) {
        Serial.println("Tank seems empty. Starting water pump.");
        setPumpState(true);
    }
    // Stop pump if water level is above a certain level.
    if (depth < MIN_DEPTH) {
        Serial.println("Tank seems full. Stopping water pump.");
        setPumpState(false);
    }
    // Stop pump if user override has led to overloaded tank.
    if (depth < MANUAL_OVERRIDE_OVERLOADED_DEPTH) {
        Serial.println("Tank is overloaded. Overriding manual override.");
        // Override-user-relay is connected in NC mode. Thus, in a dual channel relay closing the NO opens the NC.
        digitalWrite(OVERRIDE_MANUAL_OVERRIDE_RELAY_PIN, LOW);
    }
    if (depth > MANUAL_OVERRIDE_OVERLOADED_DEPTH) {
        Serial.println("Tank is no longer overloaded. Allow manual override.");
        // Override-user-relay is connected in NC mode. Thus, in a dual channel relay opening the NO closes the NC.
        digitalWrite(OVERRIDE_MANUAL_OVERRIDE_RELAY_PIN, HIGH);
    }
}

void sendUDPMulticast(const String &string) {
    Serial.print("sendUDPMulticast: ");
    Serial.println(string);

    // convert string to char array
    char msg[255];
    string.toCharArray(msg, 255);

    Udp.beginPacketMulticast(broadcast, udpPort, WiFi.localIP());
    Udp.write(msg);
    Udp.endPacket();
}

void loop() {
    // Begin Water tank section
    delay(100);

    // Displays the distance on the Serial Monitor
    Serial.print("Ping: ");
    Serial.print(smoothedDepth);
    Serial.println("cm");

    // Turn ON/OFF water pump.
    smoothedDepth = getSmoothedDepth();
    maintainWaterLevel(smoothedDepth);

    // If network is connected, check if there's an update available.
    if (WiFi.status() == WL_CONNECTED) {
        // Begin Wifi COM section
        unsigned long waterLevelPercentage = (TANK_HEIGHT - smoothedDepth);

        int packetSize = Udp.parsePacket();
        if (packetSize) {
            Serial.print("Received packet of size ");
            Serial.println(packetSize);
            Serial.print("From ");
            IPAddress remoteIp = Udp.remoteIP();
            Serial.print(remoteIp);
            Serial.print(", port ");
            Serial.println(Udp.remotePort());

            // read the packet into packetBuffer
            int len = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
            if (len > 0) {
                packetBuffer[len] = 0;
            }

            JSONVar jsonObject = JSON.parse(packetBuffer);
            if (jsonObject.hasOwnProperty("min")) {
                MIN_DEPTH = jsonObject["min"];
                Serial.println("Minimum smoothedDepth updated: ");
                Serial.println(MIN_DEPTH);
            }

            if (jsonObject.hasOwnProperty("max")) {
                MIN_DEPTH = jsonObject["max"];
                Serial.println("Maximum smoothedDepth updated: ");
                Serial.println(MAX_DEPTH);
            }

            if (jsonObject.hasOwnProperty("state")) {
                int state = jsonObject["state"];
                Serial.print("Manual state override: ");
                Serial.println(state);
                setPumpState(state == true);
            }

            if (jsonObject.hasOwnProperty("reboot")) {
                Serial.print("Reboot reason: ");
                Serial.println(jsonObject["reboot"]);
                ESP.restart();
            }

            // send a reply, to the IP address and port that sent us the packet we received
            Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
            String message = "{\"state\": ";
            message += isPumpRunning;
            message += ", \"level\": ";
            message += waterLevelPercentage;
            message += "}";
            Udp.write(message.c_str());
            Udp.endPacket();
            sendUDPMulticast(message);
        }

        // Handle server requests
        httpServer.handleClient();
        MDNS.update();
    }
}
