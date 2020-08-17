// Caravan twin motor mover control - wireless version
// ST 2020

#include <string>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <WebSocketsServer.h>

#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
int16_t adc1;
int16_t adc2;

int ledPin = 10;
int switchPin = 16;
int leftPin = 12;
int rightPin = 14;
int leftDirPinFwd = 13;
int leftDirPinBkwd = 15;
int rightDirPinFwd = 0;
int rightDirPinBkwd = 2;
int leftOnPin = 1;
int rightOnPin = 3;

int xPosition = 0;
int yPosition = 0;
bool isOn = false;

int r_max = 17450;
int r_min = 34;
int x_mid = 0;
int y_mid = 0;
int deadOffset = 2;
int offset_x = 0;
int offset_y = 0;
int motorValues[2] = { 0,0 };
bool rainbow = false;

ESP8266WiFiMulti wifiMulti;       // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
ESP8266WebServer server(80);       // Create a webserver object that listens for HTTP request on port 80
WebSocketsServer webSocket(81);    // create a websocket server on port 81
File fsUploadFile;                 // a File variable to temporarily store the received file

const char* ssid = "Conqueror"; // The name of the Wi-Fi network that will be created
const char* password = "thereisnospoon";   // The password required to connect to it, leave blank for an open network

const char* OTAName = "ESP8266";           // A name and a password for the OTA service
const char* OTAPassword = "esp8266";
const char* mdnsName = "esp8266"; // Domain name for the mDNS responder

void setup() {
    pinMode(ledPin, OUTPUT);
    pinMode(switchPin, INPUT);
    pinMode(leftDirPinFwd, OUTPUT);
    pinMode(leftDirPinBkwd, OUTPUT);
    pinMode(rightDirPinFwd, OUTPUT);
    pinMode(rightDirPinBkwd, OUTPUT);
    pinMode(leftOnPin, OUTPUT);
    pinMode(rightOnPin, OUTPUT);

    Serial.begin(115200);
    delay(10);
    Serial.println("\r\n");
    Serial.println("Starting...");
    digitalWrite(ledPin, HIGH); // TURN OFF
    ads.begin();
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);

    x_mid = adc2;
    y_mid = adc1;

    //  startWiFi();
    //  startOTA();
    //  startSPIFFS();
    //  startWebSocket();
    //  startMDNS();
    //  startServer();
}

void switchOn() {
    digitalWrite(ledPin, LOW);
    //Serial.println("ON - HIGH");
    isOn = true;
}

void switchOff() {
    digitalWrite(ledPin, HIGH);
    //    Serial.println("OFF - LOW");
    isOn = false;
}

void loop() {

    //  webSocket.loop();
    //  server.handleClient();
    //  ArduinoOTA.handle();
    //  
    if (digitalRead(switchPin) == LOW) {
        if (isOn) {
            switchOff();
        }
        else {
            switchOn();
            Serial.print("Offset X:"); Serial.print(offset_x); Serial.print(", Offset Y:"); Serial.println(offset_y);
        }
        delay(500); // de-bounce
    }

    if (isOn) {
        adc1 = ads.readADC_SingleEnded(1);
        adc2 = ads.readADC_SingleEnded(2);
        //Serial.print(adc2); Serial.print(" : "); Serial.print(adc1); Serial.print(" | ");   

        xPosition = reduceRange(adc2, x_mid);
        yPosition = reduceRange(adc1, y_mid);
        //    Serial.print(xPosition); Serial.print(" : "); Serial.print(yPosition); Serial.print(" | ");   

        convertAndSetMotors(xPosition, yPosition);
    }
}

void convertAndSetMotors(int x, int y) {
    int left = 0;
    int right = 0;

    Serial.print("IN: X="); Serial.print(x); Serial.print(", Y="); Serial.print(y);
    setNewMotorValues(x, y);
    Serial.print(" CONVERTED: L="); Serial.print(motorValues[0]); Serial.print(", R="); Serial.print(motorValues[1]);

    bool leftFwd = false;
    bool rightFwd = false;

    if (x > deadOffset || x < deadOffset * -1
        || y > deadOffset || y < deadOffset * -1) {

        leftFwd = motorValues[0] > 0;
        rightFwd = motorValues[1] > 0;

        direction(leftFwd, rightFwd);

        if (leftFwd) {
            left = motorValues[0];
        }
        else {
            left = motorValues[0] * -1;
        }
        if (rightFwd) {
            right = motorValues[1];
        }
        else {
            right = motorValues[1] * -1;
        }

        if (y < 0) {
            int h = left;
            left = right;
            right = h;
        }
    }
    else {
        left = 0;
        right = 0;
    }

    Serial.print(" ,LDir="); Serial.print(leftFwd ? "F" : "B"); Serial.print(", RDir="); Serial.print(rightFwd ? "F" : "B");
    Serial.print(" : OUTPUT: L="); Serial.print(left); Serial.print(", R="); Serial.print(right);

    // Scale up to (0 - 1023) range
    left = scaleToPwm(left);
    right = scaleToPwm(right);

    Serial.print(", SCALED: L="); Serial.print(left); Serial.print(" : R="); Serial.println(right);

    analogWrite(leftPin, left);
    analogWrite(rightPin, right);
}

void direction(bool left, bool right) {
    if (left) {
        digitalWrite(leftDirPinBkwd, LOW);
        digitalWrite(leftDirPinFwd, HIGH);
    }
    else {
        digitalWrite(leftDirPinFwd, LOW);
        digitalWrite(leftDirPinBkwd, HIGH);
    }

    if (right) {
        digitalWrite(rightDirPinBkwd, LOW);
        digitalWrite(rightDirPinFwd, HIGH);
    }
    else {
        digitalWrite(rightDirPinFwd, LOW);
        digitalWrite(rightDirPinBkwd, HIGH);
    }
}

int scaleFromWeb(int pos) {
    // Input is from -100 to 100
    // Scale to -128 to +127
      //Result = ((Input - InputLow) / (InputHigh - InputLow)) * (OutputHigh - OutputLow) + OutputLow;
    return ((float)(pos - -100) / (100 - -100)) * (127 - -128) + -128;
}

int scaleToPwm(int pos) {
    //Result = ((Input - InputLow) / (InputHigh - InputLow)) * (OutputHigh - OutputLow) + OutputLow;
    return ((float)(pos - 0) / (127 - 0)) * (1023 - 0) + 0;
}

int reduceRange(int pos, int r_mid) {
    // Input is between r_min and r_max
    // Needs scaling to between -128 and +127
    // BUT...
    // The centre isn't halfway between, so scale differently each side of the centre
    float r;
    if (pos > r_mid) {
        //Result = ((Input - InputLow) / (InputHigh - InputLow)) * (OutputHigh - OutputLow) + OutputLow;
        r = ((float)(pos - r_mid) / (r_max - r_mid)) * (127 - 0) + 0;
    }
    else {
        r = ((float)(pos - r_min) / (r_mid - r_min)) * (0 - -128) + -128;
    }

    //Result = ((Input - InputLow) / (InputHigh - InputLow)) * (OutputHigh - OutputLow) + OutputLow;

    return round(r);
}


#define DDRIVE_MIN -127 //The minimum value x or y can be.
#define DDRIVE_MAX 127  //The maximum value x or y can be.
#define MOTOR_MIN_PWM -127 //The minimum value the motor output can be.
#define MOTOR_MAX_PWM 127 //The maximum value the motor output can be.


void setNewMotorValues(float x, float y)
{
    float rawLeft;
    float rawRight;
    float RawLeft;
    float RawRight;


    // first Compute the angle in deg
    // First hypotenuse
    float z = sqrt(x * x + y * y);

    // angle in radians
    float rad = acos(abs(x) / z);

    // Cataer for NaN values
    if (isnan(rad) == true) {
        rad = 0;
    }

    // and in degrees
    float angle = rad * 180 / PI;

    // Now angle indicates the measure of turn
     // Along a straight line, with an angle o, the turn co-efficient is same
     // this applies for angles between 0-90, with angle 0 the co-eff is -1
     // with angle 45, the co-efficient is 0 and with angle 90, it is 1

    float tcoeff = -1 + (angle / 90) * 2;
    float turn = tcoeff * abs(abs(y) - abs(x));
    turn = round(turn * 100) / 100;

    // And max of y or x is the movement
    float mov = max(abs(y), abs(x));

    // First and third quadrant
    if ((x >= 0 && y >= 0) || (x < 0 && y < 0))
    {
        rawLeft = mov; rawRight = turn;
    }
    else
    {
        rawRight = mov; rawLeft = turn;
    }

    // Reverse polarity
    if (y < 0) {
        rawLeft = 0 - rawLeft;
        rawRight = 0 - rawRight;
    }

    // Update the values
    RawLeft = rawLeft;
    RawRight = rawRight;

    // Map the values onto the defined rang
    motorValues[0] = map(rawLeft, DDRIVE_MIN, DDRIVE_MAX, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    motorValues[1] = map(rawRight, DDRIVE_MIN, DDRIVE_MAX, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
}

void setMotorValues(int nJoyX, int nJoyY) {
    // Differential Steering Joystick Algorithm
  // ========================================
  //   by Calvin Hass
  //   https://www.impulseadventure.com/elec/
  //
  // Converts a single dual-axis joystick into a differential
  // drive motor control, with support for both drive, turn
  // and pivot operations.
  //

  // INPUTS
  //int     nJoyX;              // Joystick X input                     (-128..+127)
  //int     nJoyY;              // Joystick Y input                     (-128..+127)

  // OUTPUTS
    int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
    int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

    // CONFIG
    // - fPivYLimt  : The threshold at which the pivot action starts
    //                This threshold is measured in units on the Y-axis
    //                away from the X-axis (Y=0). A greater value will assign
    //                more of the joystick's range to pivot actions.
    //                Allowable range: (0..+127)
    float fPivYLimit = 32.0;

    // TEMP VARIABLES
    float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
    float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
    int     nPivSpeed;      // Pivot Speed                          (-128..+127)
    float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )


    // Calculate Drive Turn output due to Joystick X input
    if (nJoyY >= 0) {
        // Forward
        nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
        nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
    }
    else {
        // Reverse
        nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
        nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
    }

    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * nJoyY / 128.0;
    nMotPremixR = nMotPremixR * nJoyY / 128.0;

    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    nPivSpeed = nJoyX;
    fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

    // Calculate final mix of Drive and Pivot
    nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
    nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

    motorValues[0] = nMotMixL;
    motorValues[1] = nMotMixR;
}

void startWiFi() { // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
    WiFi.softAP(ssid, password);             // Start the access point
    Serial.print("Access Point \"");
    Serial.print(ssid);
    Serial.println("\" started\r\n");

    //wifiMulti.addAP("Skynet", "MollyCat2015");   // add Wi-Fi networks you want to connect to


    Serial.println("Connecting");
    while (wifiMulti.run() != WL_CONNECTED && WiFi.softAPgetStationNum() < 1) {  // Wait for the Wi-Fi to connect
        delay(250);
        Serial.print('.');
    }
    Serial.println("\r\n");
    if (WiFi.softAPgetStationNum() == 0) {      // If the ESP is connected to an AP
        Serial.print("Connected to ");
        Serial.println(WiFi.SSID());             // Tell us what network we're connected to
        Serial.print("IP address:\t");
        Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
    }
    else {                                   // If a station is connected to the ESP SoftAP
        Serial.print("Station connected to ESP8266 AP");
    }
    Serial.println("\r\n");
}

void startOTA() { // Start the OTA service
    ArduinoOTA.setHostname(OTAName);
    ArduinoOTA.setPassword(OTAPassword);

    ArduinoOTA.onStart([]() {
        Serial.println("Start");
        });
    ArduinoOTA.onEnd([]() {
        Serial.println("\r\nEnd");
        });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
    ArduinoOTA.begin();
    Serial.println("OTA ready\r\n");
}

void startSPIFFS() { // Start the SPIFFS and list all contents
    SPIFFS.begin();                             // Start the SPI Flash File System (SPIFFS)
    Serial.println("SPIFFS started. Contents:");
    {
        Dir dir = SPIFFS.openDir("/");
        while (dir.next()) {                      // List the file system contents
            String fileName = dir.fileName();
            size_t fileSize = dir.fileSize();
            Serial.printf("\tFS File: %s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
        }
        Serial.printf("\n");
    }
}

void startWebSocket() { // Start a WebSocket server
    webSocket.begin();                          // start the websocket server
    webSocket.onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'
    Serial.println("WebSocket server started.");
}

void startMDNS() { // Start the mDNS responder
    MDNS.begin(mdnsName);                        // start the multicast domain name server
    Serial.print("mDNS responder started: http://");
    Serial.print(mdnsName);
    Serial.println(".local");
}

void startServer() { // Start a HTTP server with a file read handler and an upload handler
    server.on("/edit.html", HTTP_POST, []() {  // If a POST request is sent to the /edit.html address,
        server.send(200, "text/plain", "");
        }, handleFileUpload);                       // go to 'handleFileUpload'

    server.onNotFound(handleNotFound);          // if someone requests any other file or page, go to function 'handleNotFound'
                                                // and check if the file exists

    server.begin();                             // start the HTTP server
    Serial.println("HTTP server started.");
}

void handleNotFound() { // if the requested file or page doesn't exist, return a 404 not found error
    if (!handleFileRead(server.uri())) {          // check if the file exists in the flash memory (SPIFFS), if so, send it
        server.send(404, "text/plain", "404: File Not Found");
    }
}

bool handleFileRead(String path) { // send the right file to the client (if it exists)
    Serial.println("handleFileRead: " + path);
    if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
    String contentType = getContentType(path);             // Get the MIME type
    String pathWithGz = path + ".gz";
    if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
        if (SPIFFS.exists(pathWithGz))                         // If there's a compressed version available
            path += ".gz";                                         // Use the compressed verion
        File file = SPIFFS.open(path, "r");                    // Open the file
        size_t sent = server.streamFile(file, contentType);    // Send it to the client
        file.close();                                          // Close the file again
        Serial.println(String("\tSent file: ") + path);
        return true;
    }
    Serial.println(String("\tFile Not Found: ") + path);   // If the file doesn't exist, return false
    return false;
}

void handleFileUpload() { // upload a new file to the SPIFFS
    HTTPUpload& upload = server.upload();
    String path;
    if (upload.status == UPLOAD_FILE_START) {
        path = upload.filename;
        if (!path.startsWith("/")) path = "/" + path;
        if (!path.endsWith(".gz")) {                          // The file server always prefers a compressed version of a file 
            String pathWithGz = path + ".gz";                    // So if an uploaded file is not compressed, the existing compressed
            if (SPIFFS.exists(pathWithGz))                      // version of that file must be deleted (if it exists)
                SPIFFS.remove(pathWithGz);
        }
        Serial.print("handleFileUpload Name: "); Serial.println(path);
        fsUploadFile = SPIFFS.open(path, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
        path = String();
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
        if (fsUploadFile)
            fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
    }
    else if (upload.status == UPLOAD_FILE_END) {
        if (fsUploadFile) {                                    // If the file was successfully created
            fsUploadFile.close();                               // Close the file again
            Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
            server.sendHeader("Location", "/success.html");      // Redirect the client to the success page
            server.send(303);
        }
        else {
            server.send(500, "text/plain", "500: couldn't create file");
        }
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) { // When a WebSocket message is received
    switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
        Serial.printf("[%u] Disconnected!\n", num);
        break;
    case WStype_CONNECTED: {              // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        switchOff();
    }
                         break;
    case WStype_TEXT:                     // if new text data is received
    //  Serial.printf("[%u] Data: %s\n", num, payload);
        if (payload[0] == '0') {
            switchOff();
        }
        else if (payload[0] == '1') {                      // the browser sends an R when the rainbow effect is enabled
            switchOn();
            // Do move
            // Split input
            String _payload = String((char*)&payload[0]);
            String xx = (_payload.substring(1, _payload.indexOf(",")));
            //      Serial.print(xx); Serial.print(" - ");
            String yy = (_payload.substring(_payload.indexOf(",") + 1, _payload.length()));
            //      Serial.println(yy);
            int x = xx.toInt();
            int y = yy.toInt();

            int sx = scaleFromWeb(x);
            int sy = scaleFromWeb(y);

            //       Serial.print("XANDY: "); Serial.print(sx); Serial.print(","); Serial.println(sy);
            convertAndSetMotors(sx, sy);
        }
        break;
    }
}

String formatBytes(size_t bytes) { // convert sizes in bytes to KB and MB
    if (bytes < 1024) {
        return String(bytes) + "B";
    }
    else if (bytes < (1024 * 1024)) {
        return String(bytes / 1024.0) + "KB";
    }
    else if (bytes < (1024 * 1024 * 1024)) {
        return String(bytes / 1024.0 / 1024.0) + "MB";
    }
}

String getContentType(String filename) { // determine the filetype of a given filename, based on the extension
    if (filename.endsWith(".html")) return "text/html";
    else if (filename.endsWith(".css")) return "text/css";
    else if (filename.endsWith(".js")) return "application/javascript";
    else if (filename.endsWith(".ico")) return "image/x-icon";
    else if (filename.endsWith(".gz")) return "application/x-gzip";
    return "text/plain";
}
