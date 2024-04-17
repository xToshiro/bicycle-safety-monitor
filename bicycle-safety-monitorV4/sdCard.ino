// Returns the filename based on the current date and time
String getFileName() {
  char fileName[25];  // Buffer to store the filename
  // Format the filename with day, month, year, and hour from the RTC data
  snprintf(fileName, sizeof(fileName), "/D%02d-%02d-%04d--H%02d.csv", rtcdia, rtcmes, rtcano, rtchora   );
  return String(fileName);
}

// Check if the data file exists on the SD card and create it with a header if it does not or is empty
void checkSDFile() {
  String fileName = getFileName();  // Get the current file name based on the RTC date and time
  Serial.print("Checking file: "); Serial.println(fileName);

  // Tenta abrir o arquivo apenas para leitura para verificar sua existência
  File file = SD.open(fileName.c_str(), FILE_READ);
  if (!file) {
    Serial.println("File does not exist or cannot be opened for reading.");
    file = SD.open(fileName.c_str(), FILE_WRITE);  // Cria um novo arquivo para escrita
    if (file) {
      Serial.println("File created. Writing header...");
      const char *header = "RTCData, RTCHora, GPSData, GPSHora, Lat, Long, Altgps, Vel, GPSUpdate, IndiceAmostra, Temperatura, AccX, AccY, AccZ, GyroX, GyroY, GyroZ, AccAngleX, AccAngleY, AngleX, AngleY, AngleZ, Dist1, Dist2, BotaoStatus \r\n";
      if (file.print(header)) {
        Serial.println("Header written successfully.");
      } else {
        Serial.println("Failed to write header.");
        blink('R', 2);  // Indicate an error in a non-recoverable loop
      }
      file.close();
    } else {
      Serial.println("Failed to create file.");
      blink('R', 2);  // Indicate an error in a non-recoverable loop
    }
  } else {
    Serial.println("File exists, no need to create.");
    file.close();
  }
}



// Initialize the SD card and check its status and size
void initSDCard() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    blink('R', 2);  // Indicate an error in a non-recoverable loop
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    blink('R', 2);  // Indicate an error in a non-recoverable loop
    return;
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);  // Convert bytes to megabytes
  Serial.printf("SD Card Size: %lluMB\n", cardSize);  // Print the size of the SD card
}

// Write to a file, creating it if it does not exist
void writeFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    //Serial.println("Failed to open file for writing");
    blink('R', 2);  // Indicate an error in a non-recoverable loop
    return;
  }
  if (file.print(message)) {
    //Serial.println("File written successfully");
  } else {
    //Serial.println("Write failed");
    blink('R', 2);  // Indicate an error if writing fails
  }
  file.close();  // Close the file to save the data properly
}


// Adiciona dados ao arquivo
void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //Serial.println("Failed to open file for appending");
    blink('R', 2);
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
    blink('R', 2);
  }
  file.close();
}

void saveData() {
  // Construct the data message to be saved on the SD card
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

  // Display the data message in the Serial Monitor
  Serial.print("Data appended: "); Serial.print(dataMessage);

  // Obtain the filename based on the current date and time
  String fileName = getFileName();

  // Append the data message to the file on the SD card
  appendFile(SD, fileName.c_str(), dataMessage.c_str());
}

