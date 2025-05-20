void configureGPS() {
  // ---------- UBX-CFG-PRT (set baud rate to 115200) ----------
  byte setBaud[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x00,             // CFG-PRT
    0x14, 0x00,             // Payload length = 20 bytes
    0x01,                   // Port ID = 1 (UART)
    0x00,                   // Reserved
    0x00, 0x00,             // txReady
    0xD0, 0x08, 0x00, 0x00, // mode = 8N1
    0x00, 0xC2, 0x01, 0x00, // baudRate = 115200 (0x01C200)
    0x07, 0x00,             // inProtoMask = UBX + NMEA
    0x03, 0x00,             // outProtoMask = UBX + NMEA
    0x00, 0x00,             // flags
    0x00, 0x00,             // reserved
    0x00, 0x00              // checksum placeholders
  };

  // ---------- UBX-CFG-RATE (set update rate to 5Hz) ----------
  byte setRate[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x08,             // CFG-RATE
    0x06, 0x00,             // Payload length = 6 bytes
    0xC8, 0x00,             // measRate = 200ms = 5Hz (0x00C8)
    0x01, 0x00,             // navRate = 1
    0x01, 0x00,             // timeRef = UTC
    0x00, 0x00              // checksum placeholders
  };

  // Compute checksums
  auto addChecksum = [](byte* msg, int len) {
    byte ck_a = 0, ck_b = 0;
    for (int i = 2; i < len - 2; i++) {
      ck_a += msg[i];
      ck_b += ck_a;
    }
    msg[len - 2] = ck_a;
    msg[len - 1] = ck_b;
  };

  addChecksum(setBaud, sizeof(setBaud));
  addChecksum(setRate, sizeof(setRate));

  // Send baud config at current default baud rate (usually 9600)
  setUartMux(1);
  Serial1.end();
  Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
  while(!Serial1) vTaskDelay(pdMS_TO_TICKS(10));

  Serial1.write(setBaud, sizeof(setBaud));
  Serial1.flush();
  delay(200);

  // Reopen Serial1 at new baud rate
  Serial1.end();
  Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
  while(!Serial1) vTaskDelay(pdMS_TO_TICKS(10));

  // Send update rate config at new baud rate
  Serial1.write(setRate, sizeof(setRate));
  Serial1.flush();
}

// Task for GPS reading
void getGPSLoop()
{
  setUartMux(1);
  vTaskDelay(pdMS_TO_TICKS(10));  
  //Serial1.end();
  //Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
  //while(!Serial1) vTaskDelay(pdMS_TO_TICKS(10));
  
  // Flush the serial buffer to get fresh data
  Serial1.flush();
  
  // Reset buffer for new reading
  bool newData = false;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 300) {
    if (Serial1.available())
    {
      char c = Serial1.read();
      // Feed each character to TinyGPS++ object
      if (gps.encode(c)) {
        newData = true;
      }
    }
    else
    {
      // No data available, yield to other tasks
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }

  if (!newData)
  {
    telemetry.foil_speed = 0xFF;
  }
  else if(!gps.speed.isUpdated())
  {
    telemetry.foil_speed = 99;
  }
  else
  {
    telemetry.foil_speed = (uint8_t)gps.speed.kmph();
  }
/*
  // If we received valid GPS data
  if (newData && gps.speed.isUpdated()) 
  {
    telemetry.foil_speed = (uint8_t)gps.speed.kmph();
  } 
  else 
  {
    //telemetry.foil_speed = 0xFF;
  }*/
}

// Function to print satellite information
void printSatelliteInfo() {
  Serial.println("----- GPS Satellite Status -----");
  Serial.print("Satellites in view: ");
  Serial.println(gps.satellites.value());
  
  Serial.print("HDOP (Horizontal Dilution of Precision): ");
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.value());
    Serial.println(" (Lower is better, <1 Excellent, 1-2 Good, 2-5 Moderate, 5-10 Fair, >10 Poor)");
  } else {
    Serial.println("Invalid");
  }
  
  Serial.print("Location validity: ");
  Serial.println(gps.location.isValid() ? "Valid" : "Invalid");
  
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters());
      Serial.println(" meters");
    } else {
      Serial.println("Invalid");
    }
  }
  
  Serial.print("Date/Time validity: ");
  Serial.println(gps.date.isValid() && gps.time.isValid() ? "Valid" : "Invalid");
  
  if (gps.date.isValid() && gps.time.isValid()) {
    char dateTime[30];
    sprintf(dateTime, "%04d-%02d-%02d %02d:%02d:%02d UTC", 
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.print("Date/Time: ");
    Serial.println(dateTime);
  }
  
  Serial.print("Course validity: ");
  Serial.println(gps.course.isValid() ? "Valid" : "Invalid");
  
  if (gps.course.isValid()) {
    Serial.print("Course: ");
    Serial.print(gps.course.deg());
    Serial.println(" degrees");
  }
  
  Serial.print("Chars processed: ");
  Serial.println(gps.charsProcessed());
  Serial.print("Sentences with fix: ");
  Serial.println(gps.sentencesWithFix());
  Serial.print("Failed checksum: ");
  Serial.println(gps.failedChecksum());
  
  Serial.println("-------------------------------");
}