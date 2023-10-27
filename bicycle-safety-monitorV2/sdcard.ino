// Essa função checa se o arquivo data.csv existe junto com o cabeçalho certo
// Caso não exista, ele chama a função writeFile para criar o arquivo
void saveData(){
    //digitalWrite(LED_YELLOW, HIGH);
    
    // Oq custa dar um enter e quebra a linha pra não ficar kilometrica???
    // Pq buga o enter na outra ide, anyway
    dataMessage = String(rtcdia) + "/" + String(rtcmes) + "/" + String(rtcano) + "," + String(rtchora) + ":" + String(rtcminuto) + ":" + String(rtcsegundo) + "-" + String(rtcmillis) + "," + String(gpsdia) + "/" + String(gpsmes) + "/" + String(gpsano) + "," + String(gpshora) + ":" + String(gpsminuto) + ":" + String(gpssegundo) + "," + String(latitudeStr) + "," + String(longitudeStr) + ","
    + String(gpsalt) + "," + String(gpsvel) + "," + String(accx) + "," + String(accy) + "," + String(accz) + "," + String(rotx) + "," + String(roty) + "," + String(rotz) + "," + String(dist1) + "," + String(dist2) + "," + String(acctemp) + "," + String(botaoStatus) + "," + String(gpsUpdate) + "," + "\r\n"; 
    //Serial.print("Saving data: ");
    Serial.println(dataMessage);

    // É aqui que a criança chora e a mãe não vê
    appendFile(SD, "/data.csv", dataMessage.c_str());

    //digitalWrite(LED_YELLOW, LOW);
}

void checkSDFile(){
    File file = SD.open("/data.csv");
  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    delay(2000);
    writeFile(SD, "/data.csv", "RTCData, RTCHora, GPSData, GPSHora, Lat, Long, Alt, Vel, AccX, AccY, AccZ, RotX, RotY, RotZ, Dist1, Dist2, Temp, Botao, GPSUpdate \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
  delay(3000);
}

// Inicia o SDcard assim que o sistema é ligado
// Verifica se ele esta presente e funcionando
void initSDCard(){
   if (!SD.begin()) {
  } else {
    return;
  }
  uint8_t cardType = SD.cardType();

// Verifica o tipo de cartão SD e o espaço livre
  if(cardType == CARD_NONE){
    return;
  }
  //Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    //Serial.println("MMC");
  } else if(cardType == CARD_SD){
    //Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    //Serial.println("SDHC");
  } else {
    //Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}


// Grava o arquivo data.csv caso ele não exista
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    ligarLedVermelho();
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    ligarLedVermelho();
    Serial.println("Write failed");
  }
  file.close();
}

// Grava os dados gerados em dataSave e passados pela string dataMensseger
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    ligarLedVermelho();
    Serial.println("Failed to open file for appending");

    //delay(250);
 
  }
  if(file.print(message)) { 
    Serial.println("Message appended");
    //delay(250);
  } else {
    //delay(500);
    ligarLedVermelho();
    Serial.println("Append failed");
  }
  file.close();
}
