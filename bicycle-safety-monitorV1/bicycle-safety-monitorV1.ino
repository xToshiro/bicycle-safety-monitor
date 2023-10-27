#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <ESP32Time.h>

// Definição dos pinos dos LEDs
const int botao = 32;
const int ledVerdePin = 2;
const int ledVermelhoPin = 12;
const int ledAmareloPin = 15;
// Definição dos pinos RX e TX para o GPS
static const int RXPin = 16, TXPin = 17;

static const uint32_t GPSBaud = 9600;

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 14
#define SHT_LOX2 27

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

File dataFile;

// Tempo de atualização do GPS em milissegundos
const unsigned long intervaloAtualizacao = 200;
unsigned long ultimaAtualizacao = 0;

// Inicialização do objeto GPS
TinyGPSPlus gps;


// ESP32Time rtc;
ESP32Time rtc(-10800);  // offset in seconds GMT-3

#define GPS_BAUDRATE 9600  // the default baudrate of NEO-6M is 9600

// Inicialização do objeto SoftwareSerial
SoftwareSerial ss(RXPin, TXPin);

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
Adafruit_MPU6050 mpu;

String dataMessage;

unsigned long dia = 0;
unsigned long mes = 0;
unsigned long ano = 0;
unsigned long hora = 0;
unsigned long minuto = 0;
unsigned long segundo = 0;

char gpslat = 0;
char gpslong = 0;

char latitudeStr[15];
char longitudeStr[15];

double gpsalt = 0;
float gpsvel = 0;

float accx = 0;
float accy = 0;
float accz = 0;

float rotx = 0;
float roty = 0;
float rotz = 0;

int dist1 = 0;
int dist2 = 0;

float acctemp = 0;

int botaoStatus = 0;

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
void gpsprint(){
      Serial.print("Data: ");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.print(gps.date.year());
      Serial.print(" Hora: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.print(gps.time.second());
      Serial.print(" Latitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude: ");
      Serial.print(gps.location.lng(), 6);
      Serial.print(" Altitude: ");
      Serial.print(gps.altitude.meters());
      Serial.print("m");
      Serial.print(" Velocidade: ");
      Serial.print(gps.speed.kmph());
      Serial.println("km/h");
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
  // Inicialização dos pinos dos LEDs
  pinMode(ledVerdePin, OUTPUT);
  pinMode(ledVermelhoPin, OUTPUT);
  pinMode(ledAmareloPin, OUTPUT);
  pinMode(botao, INPUT);
  ligarLedVermelho();
  delay(1000);
  ligarLedAmarelo();
  delay(1000);
  ligarLedVerde();
  delay(1000);
  // Inicialização da comunicação serial com o monitor serial
  Serial.begin(115200);

  // Inicialização da comunicação serial com o GPS
  ss.begin(GPSBaud);

  // Verifica se o GPS está conectado corretamente
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (ss.available()) {
      // GPS conectado corretamente
      ligarLedVerde();
      Serial.println("GPS conectado corretamente.");
      delay(5000);
      break;
    }
  }

  if (!ss.available()) {
    // GPS não encontrado
    
    ligarLedVermelho();
    delay(500);
    Serial.println("GPS não encontrado ou falha na conexão.");
  }

    Serial.println("Adafruit MPU6050 test!");

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

  initSDCard();
  checkSDFile(); // Checa o arquivo data.csv no cartão de memoria ou cria se ele não existir
  
}

void loop() {
  // Verifica se há dados disponíveis na porta serial do GPS
   while (ss.available() > 0) 
    // Lê os dados do GPS
    if (gps.encode(ss.read()))
  // Verifica se os dados do GPS são válidos
  if (gps.location.isValid()) {
    saveData();
    // Atualiza a cada intervalo definido
    if (millis() - ultimaAtualizacao > intervaloAtualizacao) {
      ultimaAtualizacao = millis();
      // Imprime os dados no monitor serial
      gps.encode(ss.read());

      mes = gps.date.month();
      dia = gps.date.day();
      ano = gps.date.year();
      hora = gps.time.hour();
      minuto = gps.time.minute();
      segundo = gps.time.second();
      //gpslat = gps.location.lat();
      //gpslong = gps.location.lng();

      dtostrf(gps.location.lat(), 12, 8, latitudeStr);
      dtostrf(gps.location.lng(), 12, 8, longitudeStr);

      Serial.println(gps.altitude.meters());
      gpsalt = gps.altitude.meters();
      gpsvel = gps.speed.kmph();

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
      Serial.println(".");
      if(digitalRead(botao) == HIGH){
        botaoStatus = 1;
        digitalWrite(ledAmareloPin, HIGH);
      } else{
        botaoStatus = 0;
      }
      // Ligar o LED Verde
      //ligarLedVerde();
      delay(500);
      ligarLedVerde();
    }
  } else {
    // Dados inválidos do GPS
    Serial.println("Dados inválidos do GPS!");
    //delay(1000);
    // Ligar o LED Vermelho
    ligarLedAmarelo();
  }
}
