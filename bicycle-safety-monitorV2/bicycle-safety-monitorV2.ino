//Coded by Jairo Ivo
#include <Adafruit_MPU6050.h>
#include "Adafruit_VL53L0X.h"

#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <ESP32Time.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)

//#define LED 2

// ESP32Time rtc;
ESP32Time rtc(-10800);  // offset in seconds GMT-3

#define GPS_BAUDRATE 9600  // the default baudrate of NEO-6M is 9600

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 14
#define SHT_LOX2 27

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
Adafruit_MPU6050 mpu;


TinyGPSPlus gps;  // the TinyGPS++ object

String dataMessage;  // save data to sdcard

File dataFile;

unsigned long previousMillis = 0;  // Variável para armazenar o tempo da última execução
const unsigned long interval = 100;  // Intervalo desejado em milissegundos (100ms)

// Definição dos pinos dos LEDs
const int botao = 32;
const int ledVerdePin = 2;
const int ledVermelhoPin = 12;
const int ledAmareloPin = 15;
// Definição dos pinos RX e TX para o GPS

// Internal RTC Variables
int rtcdia, rtcmes, rtcano, rtchora, rtcminuto, rtcsegundo{ 0 };
long rtcmillis;
// GPS Variables
int gpsdia, gpsmes, gpsano, gpshora, gpsminuto, gpssegundo{ 0 };
float gpslat, gpslong{ 0 };
char latitudeStr[15];
char longitudeStr[15];
double gpsalt = 0;
float gpsvel = 0;
// Sensor reading variables
// BME
float accx, accy, accz, rotx, roty, rotz, acctemp{ 0 };
int dist1, dist2, botaoStatus{ 0 };

int gpsUpdate = 0; // Informs if the gps was updated on the last data


// Função para ligar o LED Verde
void ligarLedVerde() {
  digitalWrite(ledVerdePin, HIGH);
  digitalWrite(ledVermelhoPin, LOW);
  digitalWrite(ledAmareloPin, LOW);
}

// Função para ligar o LED Vermelho
void ligarLedVermelho() {
  digitalWrite(ledVerdePin, LOW);
  digitalWrite(ledVermelhoPin, HIGH);
  digitalWrite(ledAmareloPin, LOW);
}

void ligarLedAmarelo() {
  digitalWrite(ledVerdePin, LOW);
  digitalWrite(ledVermelhoPin, LOW);
  digitalWrite(ledAmareloPin, HIGH);
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    ligarLedVermelho();
    delay(500);
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    ligarLedVermelho();
    delay(500);
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void acele(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
    dist1 = measure1.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    dist2 = measure2.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
}



void setup() {
  Serial.begin(9600);
  Serial2.begin(GPS_BAUDRATE);

  Serial.println(F("AirTemp View - Coded by Jairo Ivo"));

  //pinMode(LED, OUTPUT);

  initSDCard();
  checkSDFile();  // Check the data.csv file on the memory card or create it if it does not exist

  Serial.println(F("Initiating synchronization of the internal RTC with the gps!"));
  delay(500);
  while (rtc.getYear() < 2001) {
    Serial.println(F("."));
    rtcSyncWithGps();
    //delay(50);
  }


    // Try to initialize!
  if (!mpu.begin()) {
    ligarLedVermelho();
    delay(500);
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();

}

void loop() {
  unsigned long currentMillis = millis();  // Obtém o tempo atual em milissegundos
  //delay(500);
  if (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      //digitalWrite(LED, LOW);
      if (gps.location.isValid()) {
        dtostrf(gps.location.lat(), 12, 8, latitudeStr);
        dtostrf(gps.location.lng(), 12, 8, longitudeStr);
        gpsUpdate = 1;
        if (gps.altitude.isValid()) {
          gpsalt = (gps.altitude.meters());
        } else {
          Serial.println(F("- alt: INVALID"));
          //delay(150);
        }
      } else {
        Serial.println(F("- location: INVALID"));
        //delay(150);
      }
      if (gps.speed.isValid()) {
        gpsvel = (gps.speed.kmph());
      } else {
        Serial.println(F("- speed: INVALID"));
        //delay(150);
      }
      if (gps.date.isValid() && gps.time.isValid()) {
        gpsano = (gps.date.year());
        gpsmes = (gps.date.month());
        gpsdia = (gps.date.day());
        gpshora = (gps.time.hour());
        gpsminuto = (gps.time.minute());
        gpssegundo = (gps.time.second());
      } else {
        Serial.println(F("- gpsDateTime: INVALID"));
        //delay(150);
      }
      //Serial.println();
    }
  }
  if (currentMillis - previousMillis >= interval) {
    //digitalWrite(LED, HIGH);
    //Serial.print(F("- RTC date&time: ")); Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));  // (String) returns time with specified format
    rtcmes = rtc.getMonth(); rtcdia = rtc.getDay(); rtcano = rtc.getYear(); rtchora = rtc.getHour(true); rtcminuto = rtc.getMinute(); rtcsegundo = rtc.getSecond();

    //Serial.print(F("- GPS date&time: ")); Serial.print(gpsano); Serial.print(F("-")); Serial.print(gpsmes); Serial.print(F("-")); Serial.print(gpsdia); Serial.print(F(" "));
    //Serial.print(gpshora); Serial.print(F(":")); Serial.print(gpsminuto); Serial.print(F(":")); Serial.println(gpssegundo);
    //Serial.print(F("- latitude: ")); Serial.println(latitudeStr);
    //Serial.print(F("- longitude: ")); Serial.println(longitudeStr);
    //Serial.print(F("- altitude: ")); Serial.println(gpsalt);
    //Serial.print(F("- speed: ")); Serial.print(gpsvel); Serial.println(F(" km/h"));

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    accx = (a.acceleration.x);
    accy = (a.acceleration.y);
    accz = (a.acceleration.z);
    
    rotx = (g.gyro.x);
    roty = (g.gyro.y);
    rotz = (g.gyro.z);
    
    acctemp = (temp.temperature);

    read_dual_sensors();

    if(digitalRead(botao) == HIGH){
        botaoStatus = 1;
        digitalWrite(ledAmareloPin, HIGH);
      } else{
        botaoStatus = 0;
      }
      // Ligar o LED Verde
      //ligarLedVerde();
      //delay(500);
      ligarLedVerde();
    

    saveData();
    gpsUpdate = 0;
    Serial.println();
    previousMillis = currentMillis;
  }
}

void rtcSyncWithGps() {
  if (Serial2.available() > 0) {
    //delay(150);
    if (gps.encode(Serial2.read())) {
      delay(150);
      Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        ligarLedVerde();
        Serial.print(gps.date.year()); Serial.print(F("-")); Serial.print(gps.date.month()); Serial.print(F("-")); Serial.print(gps.date.day()); Serial.print(F(" ")); 
        Serial.print(gps.time.hour()); Serial.print(F(":")); Serial.print(gps.time.minute()); Serial.print(F(":")); Serial.println(gps.time.second());
        rtc.setTime((gps.time.second()), (gps.time.minute()), (gps.time.hour()), (gps.date.day()), (gps.date.month()), (gps.date.year()));  // 17th Jan 2021 15:24:30
        //rtc.setTime(1609459200);  // 1st Jan 2021 00:00:00
        //rtc.offset = 7200; // change offset value
        Serial.print(F("- RTC date&time: ")); Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));  // (String) returns time with specified format
        // formating options  http://www.cplusplus.com/reference/ctime/strftime/
      } else {
        Serial.println(F("No valid date and time data!"));
      }
      Serial.println();
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No valid gps data: check connection"));
    ligarLedVermelho();
}