#include <MPU6050_light.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>             // Include Wire library for I2C
#include <TinyGPS++.h>        // Include TinyGPS++ library for GPS functionality
#include <ESP32Time.h>        // Include ESP32Time library for time functionality
#include <SD.h>               // Include SD library for SD card functionality
#include <SPI.h>              // Include SPI library for SPI communication
#include <ESP.h> // Necessário para acesso à função ESP.restart()


MPU6050 mpu(Wire);

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

long timer = 0;

// Sampling control definitions
const int amostrasPorSegundo = 10;                        // Samples per second
unsigned long tempoUltimaAmostra = 0;                     // Time of the last sample
const long intervaloAmostras = 1000 / amostrasPorSegundo; // Interval between samples in ms
int indiceAmostra = 0;                                    // Sample index initialization

// Definição dos pinos dos LEDs
const int botao = 32;
const int ledVerdePin = 2;
const int ledVermelhoPin = 12;
const int ledAmareloPin = 15;
// Definição dos pinos RX e TX para o GPS

// Initialize ESP32Time object with an offset for GMT-3
ESP32Time rtc(-10800);

#define GPS_BAUDRATE 9600  // Define the default baudrate for NEO-6M GPS module

TinyGPSPlus gps;   // Create a TinyGPS++ object for handling GPS data

String dataMessage;  // String to save data for SD card writing

File dataFile;  // File object for SD card operations

// Internal RTC variables
int rtcdia, rtcmes, rtcano, rtchora, rtcminuto, rtcsegundo{ 0 };
// GPS variables
int gpsdia, gpsmes, gpsano, gpshora, gpsminuto, gpssegundo{ 0 };
float gpslat, gpslong{ 0 };  // GPS latitude and longitude
char latitudeStr[15];        // String to store latitude
char longitudeStr[15];       // String to store longitude
double gpsalt = 0;           // GPS altitude
float gpsvel = 0;            // GPS speed

// Sensor reading variables
float temperatura;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float accAngleX, accAngleY;
float angleX, angleY, angleZ;
int dist1, dist2, botaoStatus{ 0 };

int gpsUpdate = 0;  // Flag to indicate if GPS was updated on the last data

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 bits per second
  Serial2.begin(GPS_BAUDRATE); // Initialize GPS module serial communication

  // Display startup message
  Serial.println(F("Bicycle-safety-monitorV3 - Coded by Jairo Ivo"));
  while (!Serial); // Wait for the serial port to connect. Needed for native USB

  initSDCard(); // Initialize SD card
  checkSDFile(); // Check for data.csv on the SD card, create it if missing

  // Configura os pinos dos LEDs como saídas
  pinMode(ledVerdePin, OUTPUT);
  pinMode(ledVermelhoPin, OUTPUT);
  pinMode(ledAmareloPin, OUTPUT);
  //Configura o botão
  pinMode(botao, INPUT);

  // Inicializa a comunicação I2C com os pinos SDA e SCL específicos e a frequência desejada
  #ifdef _ESP32_HAL_I2C_H_
  Wire.begin(21, 22, 400000); // SDA, SCL, Frequency
  #else
  Wire.begin(); // Use default settings for other boards
  #endif

  //mpu.setWire(&Wire); // Esta linha pode precisar ser ajustada conforme a sua biblioteca
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){

    blink('R', 2);

  } // Para tudo se não conseguir conectar ao MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  blink('Y', 4);
  mpu.calcOffsets(true,true);
  blink('G', 2);
  Serial.println("Done!\n");

  Serial.println(F("Initiating synchronization of the internal RTC with the GPS!"));
  delay(500); // Short delay before starting the synchronization process
  
  int tentativas = 0;

  // Wait until the RTC year is valid (assuming year is at least 2023 for this project)
  while (rtc.getYear() < 2023 || rtc.getYear() > 2025) {
    Serial.print(F(".")); // Imprime um ponto como indicador de progresso
    rtcSyncWithGps(); // Tenta sincronizar o RTC com o GPS

    tentativas++;
    if (tentativas >= 500) {
      Serial.println(F("Muitas tentativas falhas, reiniciando..."));
      blink('R', 4);
      ESP.restart(); // Reinicia o ESP32
    }
  }
  blink('G', 4);

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
    blink('R', 6);
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
    blink('R', 8);
    delay(500);
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
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

void mpuData() {
  mpu.update();

  // Atribui os valores às variáveis
  temperatura = mpu.getTemp();
  
  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();

  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();

  accAngleX = mpu.getAccAngleX();
  accAngleY = mpu.getAccAngleY();

  angleX = mpu.getAngleX();
  angleY = mpu.getAngleY();
  angleZ = mpu.getAngleZ();
}

void blink(char ledColor, int blinkCount) {
    int ledPin;
    
    // Determina qual pino usar com base na cor do LED
    switch (ledColor) {
        case 'G': // Verde
            ledPin = ledVerdePin;
            break;
        case 'R': // Vermelho
            ledPin = ledVermelhoPin;
            break;
        case 'Y': // Amarelo
            ledPin = ledAmareloPin;
            break;
        default:
            // Cor inválida, sai da função
            return;
    }

    if(ledPin == ledVermelhoPin){
      digitalWrite(ledAmareloPin, HIGH); // Liga o LED
      delay(500);                 // Espera meio segundo
      digitalWrite(ledAmareloPin, LOW);  // Desliga o LED
      delay(500);                 // Espera meio segundo
    }

    for (int i = 0; i < blinkCount; i++) {
      digitalWrite(ledPin, HIGH); // Liga o LED
      delay(500);                 // Espera meio segundo
      digitalWrite(ledPin, LOW);  // Desliga o LED
      delay(500);                 // Espera meio segundo
      }
      
    
    if(ledPin == ledVermelhoPin){
      digitalWrite(ledAmareloPin, HIGH); // Liga o LED
      delay(500);                 // Espera meio segundo
      digitalWrite(ledAmareloPin, LOW);  // Desliga o LED
      delay(500);                 // Espera meio segundo
  
    }
}

void loop() {
  // Get the current time in milliseconds since the program started
  unsigned long tempoAtual = millis();

  // Check if it's time to collect and save data based on the predefined interval
  if (tempoAtual - tempoUltimaAmostra >= intervaloAmostras) {
    saveData(); // Save the collected data
    tempoUltimaAmostra = tempoAtual; // Update the time of the last sample collection

    // Increment and check the sample index, reset if it exceeds samples per second
    indiceAmostra++;
    if (indiceAmostra > amostrasPorSegundo) {
      indiceAmostra = 1; // Reset index after reaching the limit
    }

    // Log current RTC date and time to Serial
    Serial.print(F("- RTC date&time: "));
    Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));

    // Update RTC time variables
    rtcmes = rtc.getMonth(); rtcdia = rtc.getDay(); rtcano = rtc.getYear();
    rtchora = rtc.getHour(true); rtcminuto = rtc.getMinute(); rtcsegundo = rtc.getSecond();

    read_dual_sensors();
    mpuData();
    
    // Log the current sample index
    Serial.println(indiceAmostra);

    // Note: The blinkLed function is intentionally not called in the main loop to avoid affecting the sampling rate
  }
  // Reset the GPS update flag
  gpsUpdate = 0;

  // Check if a second has passed to reduce GPS data processing load
  if ((rtc.getSecond()) != rtcsegundo) {
    // Check if there is any data available from the GPS module
    if (Serial2.available() > 0) {
      // Try to parse GPS data
      if (gps.encode(Serial2.read())) {
        // Check if the location data is valid
        if (gps.location.isValid()) {

          // Convert latitude and longitude to string with fixed format
          dtostrf(gps.location.lat(), 12, 8, latitudeStr); dtostrf(gps.location.lng(), 12, 8, longitudeStr);

          gpsUpdate = 1; // Set GPS update flag

          // Check if altitude data is valid
          if (gps.altitude.isValid()) {
            gpsalt = gps.altitude.meters(); // Update altitude
          } else {
            Serial.println(F("- alt: INVALID"));
            delay(150); // Introduce a delay to allow serial print completion
          }
        } else {
          Serial.println(F("- location: INVALID"));
          delay(150); // Introduce a delay to allow serial print completion
        }

        // Check if speed data is valid
        if (gps.speed.isValid()) {
          gpsvel = gps.speed.kmph(); // Update speed
        } else {
          Serial.println(F("- speed: INVALID"));
          delay(150); // Introduce a delay to allow serial print completion
        }

        // Check if GPS date and time data is valid
        if (gps.date.isValid() && gps.time.isValid()) {
          // Update GPS date and time variables
          gpsano = gps.date.year(); gpsmes = gps.date.month(); gpsdia = gps.date.day(); gpshora = gps.time.hour(); gpsminuto = gps.time.minute(); gpssegundo = gps.time.second();
        } else {
          Serial.println(F("- gpsDateTime: INVALID"));
          delay(150); // Introduce a delay to allow serial print completion
        }
      }
    }
  }
}

void rtcSyncWithGps() {
  if (Serial2.available() > 0) {
    //delay(150);
    if (gps.encode(Serial2.read())) {
      delay(150);
      Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year()); Serial.print(F("-")); Serial.print(gps.date.month()); Serial.print(F("-")); Serial.print(gps.date.day()); Serial.print(F(" ")); 
        Serial.print(gps.time.hour()); Serial.print(F(":")); Serial.print(gps.time.minute()); Serial.print(F(":")); Serial.println(gps.time.second());
        rtc.setTime((gps.time.second()), (gps.time.minute()), (gps.time.hour()), (gps.date.day()), ((gps.date.month()) + 1), (gps.date.year()));  // 17th Jan 2021 15:24:30
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
}
