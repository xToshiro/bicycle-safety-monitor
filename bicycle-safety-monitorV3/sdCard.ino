void saveData() {
  dataMessage = String(rtcdia) + "/" + String(rtcmes) + "/" + String(rtcano) + "," 
            + String(rtchora) + ":" + String(rtcminuto) + ":" + String(rtcsegundo) + "," 
            + String(gpsdia) + "/" + String(gpsmes) + "/" + String(gpsano) + "," 
            + String(gpshora) + ":" + String(gpsminuto) + ":" + String(gpssegundo) + "," 
            + String(latitudeStr) + "," + String(longitudeStr) + "," 
            + String(gpsalt) + "," + String(gpsvel) + "," + String(gpsUpdate) + "," 
            + String(indiceAmostra) + ","
            + String(temperatura) + "," 
            + String(accX) + "," + String(accY) + "," + String(accZ) + "," 
            + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + "," 
            + String(accAngleX) + "," + String(accAngleY) + "," 
            + String(angleX) + "," + String(angleY) + "," + String(angleZ) + ","
            + String(dist1) + ","  // Adicionando a variável dist1
            + String(dist2) + ","  // Adicionando a variável dist2
            + String(botaoStatus) + "\r\n";  // Adicionando a variável botaoStatus
  Serial.print(F("- data appended: ")); Serial.print(dataMessage);
  appendFile(SD, "/data.csv", dataMessage.c_str());
}

//This function checks if the data.csv file exists along with the right header
//If it doesn't exist, it calls the writeFile function to create the file
void checkSDFile() {
  File file = SD.open("/data.csv");
  if (!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    delay(1000);
    writeFile(SD, "/data.csv", "RTCData, RTCHora, GPSData, GPSHora, Lat, Long, Altgps, Vel, GPSUpdate, IndiceAmostra, Temperatura, AccX, AccY, AccZ, GyroX, GyroY, GyroZ, AccAngleX, AccAngleY, AngleX, AngleY, AngleZ, Dist1, Dist2, BotaoStatus \r\n");

  } else {
    Serial.println("File already exists");
  }
  file.close();
  delay(1000);
}

//Starts the SDcard as soon as the system is powered on
//Verifies that it is present and working
void initSDCard() {
  if (!SD.begin()) {
    blink('R', 2);
  } else {
    return;
  }
  uint8_t cardType = SD.cardType();

  //Checks SD card type and free space
  if (cardType == CARD_NONE) {
    return;
  }
  //Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    //Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    //Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    //Serial.println("SDHC");
  } else {
    //Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}


// Writes the data file.csv if it doesn't exist
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    //Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    //Serial.println("File written");
  } else {
    blink('R', 2);
    //Serial.println("Write failed");
  }
  file.close();
}

// Writes the data generated in dataSave and passed through the dataMensseger string
void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("- appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    //delay(250);
  }
  if (file.print(message)) {
    //Serial.println("- message appended");
    //delay(250);
  } else {
    //delay(250);
    //Serial.println("- append failed");
  }
  file.close();
}