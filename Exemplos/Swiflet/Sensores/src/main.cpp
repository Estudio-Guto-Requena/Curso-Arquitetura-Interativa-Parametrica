#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Ticker.h>
#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <Adafruit_VL53L0X.h>

#include "credential.h"

#define M_PI 3.14159265358979323846
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define MPU_ADDRESS 0x68

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu(MPU_ADDRESS);
MAX30105 particleSensor;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
AsyncWebServer server(80);
String strReceive = "";

const byte RATE_SIZE = 10; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg = 60;

int distance = 0;

Ticker TickerMPU;
Ticker TickerBPM;
Ticker TickerDIST;

void startDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;) {
        }
    }
    display.clearDisplay();
    display.display();
}

void startDistance() {
    if (!lox.begin()) {
        Serial.println("Failed to boot VL53L0X");
        while(1);
    }
    Serial.println("VL53L0X Started!");
    lox.startRangeContinuous();
}

void startBPMSensor() {
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        while (1);
    }
    else Serial.println("MAX30105 started!");
    
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);

    particleSensor.setup(); //Configure sensor with these settings


}

void startWiFi() {
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println(".");
    }
    Serial.println("Connected");
    Serial.println(WiFi.localIP());
    if (!MDNS.begin("sensor")) Serial.println("Error setting up MDNS responder!");
    Serial.println("mDNS responder started");
    MDNS.addService("esp", "tcp", 80);  // Announce esp tcp service on port 8080
}

void startAccelerometer(MPU6050 &MpuObject) {
    uint8_t devStatus;

    do {
        MpuObject.initialize();
        Serial.println(MpuObject.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        devStatus = MpuObject.dmpInitialize();
        MpuObject.setFullScaleGyroRange(3);
        MpuObject.setFullScaleAccelRange(3);
        MpuObject.setDLPFMode(3);
        // Sensor readings with offsets:   7       7       16377   -1      1       1
        // Your offsets:   -4498   807     735     101     -71     -11
        MpuObject.setXAccelOffset(-4498);
        MpuObject.setYAccelOffset(807);
        MpuObject.setZAccelOffset(735);
        MpuObject.setXGyroOffset(101);
        MpuObject.setYGyroOffset(-71);
        MpuObject.setZGyroOffset(-11);

        if (devStatus == 0) {
            MpuObject.setDMPEnabled(true);
        } else
            Serial.println("\n\n\n\n\nFault\n\n\n\n\n\n");
    } while (devStatus != 0);
    Serial.println("MPU6050 started");
}

void setText(AsyncWebServerRequest *request, JsonVariant &json) {
    String str = json["text"];
    strReceive = str;
    Serial.println(strReceive);
    request->send(200);
}

void drawText(String txt, int line) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, line);
    display.println(txt);
}

void restartBPMCount() {
  beatsPerMinute = 60;
  beatAvg = 60;
  for (int i = 0; i < RATE_SIZE; i++) rates[i] = 60;
}

void bpmCalculate () {
    long irValue = particleSensor.getIR();
    if (irValue >= 50000) {
        if (checkForBeat(irValue) == true) {
            long delta = millis() - lastBeat;
            lastBeat = millis();
            beatsPerMinute = 60 / (delta / 1000.0);
            if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
                rateSpot %= RATE_SIZE; //Wrap variable
                beatAvg = 0;
                for (byte x = 0 ; x < RATE_SIZE ; x++)
                beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
            }
        }
    }
    else restartBPMCount();
}

void distCalculate() {
    if (lox.isRangeComplete()) distance = lox.readRange();
}

void getAccData(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument jsonData(256);
    JsonObject acc = jsonData.createNestedObject("acceleration");
    acc["x"] = aa.x / 1024.;
    acc["y"] = aa.y / 1024.;
    acc["z"] = aa.z / 1024.;
    JsonObject gyro = jsonData.createNestedObject("angle");
    gyro["x"] = ypr[0];
    gyro["y"] = ypr[1];
    gyro["z"] = ypr[2];

    serializeJson(jsonData, *response);
    request->send(response);
}

void getBpmData(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument jsonData(256);
    JsonObject bpm = jsonData.createNestedObject("heartrate");
    bpm["bpm"] = beatAvg;
    bpm["raw"] = particleSensor.getIR();

    serializeJson(jsonData, *response);
    request->send(response);
}

void getDistData(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument jsonData(256);
    JsonObject dist = jsonData.createNestedObject("distance");
    dist["mm"] = distance;

    serializeJson(jsonData, *response);
    request->send(response);
}

void startServerAPI() {
    server.on("/getAcc", HTTP_GET, getAccData);
    server.on("/getBpm", HTTP_GET, getBpmData);
    server.on("/getDist", HTTP_GET, getDistData);
    AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/setText", setText);
    server.addHandler(handler);
    server.begin();
}

void mpuGetValues(MPU6050 &MpuObject) {
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        ypr[0] = ypr[0] * 180 / M_PI;
        ypr[1] = ypr[1] * 180 / M_PI;
        ypr[2] = ypr[2] * 180 / M_PI;
    }
}

void accelerometer() {
    mpuGetValues(mpu);
}


void displayInfos() {
    display.clearDisplay();
    drawText("angle:" + String(int(ypr[0])) + " " + String(int(ypr[1])) + " " + String(int(ypr[2])), 0);
    drawText("bpm:" + String(beatAvg) + " Dist " + String(distance) + " ", 10);
    drawText(strReceive, 20);
    display.display();
}

void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    Wire.begin(D2, D1);
    startDisplay();
    startAccelerometer(mpu);
    startBPMSensor();
    startDistance();
    startWiFi();
    startServerAPI();
    TickerMPU.attach_ms(10, accelerometer);
    TickerBPM.attach_ms(20, bpmCalculate);
    TickerDIST.attach_ms(20, distCalculate);
}

void loop() {
    displayInfos();
    MDNS.update();
    yield();
}
