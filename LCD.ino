
uint8_t printLCDPanel(char* buffer, uint8_t size) {
    Wire.beginTransmission(lcdAddress);
    while(bufferFreeBytes() < BUFFER_LENGTH);      // Wait until buffer is empty
    Wire.write(REG_COMMAND);
      
    for(int i = 0; i < size; i++) {    
      if(i != 0 && i % (BUFFER_LENGTH - 1) == 0) {
        Wire.endTransmission();

        while(bufferFreeBytes() < BUFFER_LENGTH);   // Wait until buffer is empty
        Wire.beginTransmission(lcdAddress);
        Wire.write(REG_COMMAND);
      }
      Wire.write(buffer[i]);
    }
    Wire.endTransmission();
    return size;
}

void setcursorLCD(uint8_t row, uint8_t col) {    
    i2cBuffer[0] = LCD_CURSORPOSXY;
    i2cBuffer[1] = ++row;
    i2cBuffer[2] = ++col;
    while(i2cWrite(lcdAddress, REG_COMMAND, i2cBuffer, 3, true));
}

void initLCDPanel() {
    i2cBuffer[0] = LCD_CURSOROFF;
    i2cBuffer[1] = LCD_CLEARDISPLAY;
    i2cBuffer[2] = LCD_BACKLIGHTON;
    while(i2cWrite(lcdAddress, REG_COMMAND, i2cBuffer, 3, true));
}

uint8_t bufferFreeBytes() {
    Wire.requestFrom(lcdAddress, (uint8_t)1);
    if(Wire.available()) {
      return Wire.read();
    }
}

