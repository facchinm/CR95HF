#include "CR95HF.h"
#include "SPI.h"

// Write command to the CR95HF
void CR95HF::writeCmd(unsigned short cmd, unsigned short dataLen) {
  unsigned short i = 0;

  digitalWrite(CS, LOW);
  SPI.beginTransaction(SETTINGS);
  SPI.transfer(0x00);  // Send cmd to CR95HF
  SPI.transfer(cmd);
  SPI.transfer(dataLen);
  while (dataLen == 0) {
    digitalWrite(CS, HIGH);
    break;
  }
  for (i = 0; i < dataLen; i++) {
    SPI.transfer(sdata[i]);
  }
  SPI.endTransaction();
  digitalWrite(CS, HIGH);
}

// Poll the CR95HF
void CR95HF::readCmd() {
  unsigned short i = 0;

  while (1) {
    digitalWrite(CS, LOW);
    SPI.beginTransaction(SETTINGS);
    SPI.transfer(0x03);
    res = SPI.transfer(0);
    SPI.endTransaction();
    digitalWrite(CS, HIGH);

    if ((res & 0x08) >> 3) {
      digitalWrite(CS, LOW);
      SPI.beginTransaction(SETTINGS);
      SPI.transfer(0x02);
      res = SPI.transfer(0);
      dataNum = SPI.transfer(0);
      for (i = 0; i < dataNum; i++)
        rdata[i] = SPI.transfer(0);
      SPI.endTransaction();
      digitalWrite(CS, HIGH);
      break;
    }
    SPI.endTransaction();
    digitalWrite(CS, HIGH);
    delay(10);
  }
}

// Initialize MCU and peripherals
void CR95HF::begin() {
  // Configure GPIO pins
  if (SSI_0 != -1)
    pinMode(SSI_0, OUTPUT);
  if (SSI_1 != -1)
    pinMode(SSI_1, OUTPUT);
  if (IRQ_IN != -1)
    pinMode(IRQ_IN, OUTPUT);

  pinMode(CS, OUTPUT);

  // Set in SPI mode
  if (SSI_0 != -1)
    digitalWrite(SSI_0, HIGH);
  if (SSI_1 != -1)
    digitalWrite(SSI_1, LOW);
  if (IRQ_IN != -1)
    digitalWrite(IRQ_IN, HIGH);
  delay(1);

  if (IRQ_IN != -1)
    digitalWrite(IRQ_IN, LOW);
  delay(1);
  if (IRQ_IN != -1)
    digitalWrite(IRQ_IN, HIGH);
  delay(1);

  SPI.begin();

  while (!EchoResponse()) {   // Until CR95HF is detected
    if (IRQ_IN != -1)
      digitalWrite(IRQ_IN, HIGH);
    delay(1);
    if (IRQ_IN != -1)
      digitalWrite(IRQ_IN, LOW);
    delay(1);
  }

  readSerial();
  Calibration();
  IndexMod_Gain();
  AutoFDet();
  Select_ISO_IEC_18092_Protocol();
}

// Get Echo reponse from CR95HF
char CR95HF::EchoResponse() {

  digitalWrite(CS, LOW);
  SPI.beginTransaction(SETTINGS);
  SPI.transfer(0x00);  // Send cmd to CR95HF
  SPI.transfer(ECHO);
  SPI.endTransaction();
  digitalWrite(CS, HIGH);

  while (1) {
    digitalWrite(CS, LOW);
    SPI.beginTransaction(SETTINGS);
    SPI.transfer(0x03);
    tmp = SPI.transfer(0);
    SPI.endTransaction();
    digitalWrite(CS, HIGH);

    if ((tmp & 0x08) >> 3) {
      digitalWrite(CS, LOW);
      SPI.beginTransaction(SETTINGS);
      SPI.transfer(0x02);
      tmp = SPI.transfer(0);
      SPI.endTransaction();
      digitalWrite(CS, HIGH);
      if (tmp == ECHO) {
        return 1;
      }
      return 0;
    }
  }
}

// Calibrate CR95HF device
void CR95HF::Calibration() {
  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0x00;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();

  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0xFC;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();

  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0x7C;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();

  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0x3C;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();

  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0x5C;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();

  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0x6C;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();

  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0x74;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();

  sdata[0] = 0x03;
  sdata[1] = 0xA1;
  sdata[2] = 0x00;
  sdata[3] = 0xF8;
  sdata[4] = 0x01;
  sdata[5] = 0x18;
  sdata[6] = 0x00;
  sdata[7] = 0x20;
  sdata[8] = 0x60;
  sdata[9] = 0x60;
  sdata[10] = 0x00;
  sdata[11] = 0x70;
  sdata[12] = 0x3F;
  sdata[13] = 0x01;
  writeCmd(Idle, 0x0E);
  readCmd();
}

// Get CR95HF chip ID
char* CR95HF::readSerial() {
  writeCmd(IDN, 0);
  readCmd();
  for (j = 0; j < dataNum; j++) {
    CR95HF_ID[j] = rdata[j];
  }
  return CR95HF_ID;
}

// Select the RF communication protocol (ISO/IEC 14443-A)
void CR95HF::Select_ISO_IEC_14443_A_Protocol() {
  sdata[0] = 0x02;
  sdata[1] = 0x00;
  writeCmd(ProtocolSelect, 2);
  readCmd();

  // Clear read and write buffers
  for (j = 0; j < 18; j++ ) {
    rdata[j] = 0;
    sdata[j] = 0;
  }
}

// Select the RF communication protocol (ISO/IEC 18092)
void CR95HF::Select_ISO_IEC_18092_Protocol() {
  sdata[0] = 0x04;
  sdata[1] = 0x51;
  writeCmd(ProtocolSelect, 2);
  readCmd();

  // Clear read and write buffers
  for (j = 0; j < 18; j++ ) {
    rdata[j] = 0;
    sdata[j] = 0;
  }
}

// Configure IndexMod & Gain
void CR95HF::IndexMod_Gain() {
  sdata[0] = 0x09;
  sdata[1] = 0x04;
  sdata[2] = 0x68;
  sdata[3] = 0x01;
  sdata[4] = 0x01;
  sdata[5] = 0x50;
  writeCmd(WrReg, 6);
  readCmd();
}

// Configure Auto FDet
void CR95HF::AutoFDet() {
  sdata[0] = 0x09;
  sdata[1] = 0x04;
  sdata[2] = 0x0A;
  sdata[3] = 0x01;
  sdata[4] = 0x02;
  sdata[5] = 0xA1;
  writeCmd(WrReg, 6);
  readCmd();
}

// Read the tag ID
void CR95HF::GetTagID() {
  sdata[0] = 0x26;
  sdata[1] = 0x07;
  writeCmd(SendRecv , 2);
  readCmd();

  sdata[0] = 0x93;
  sdata[1] = 0x20;
  sdata[2] = 0x08;
  writeCmd(SendRecv , 3);
  readCmd();

  if (res == 0x80) {
    for (j = 1; j < dataNum - 3; j++) {
      ID += String(rdata[j], HEX);
    }
    TAG_flag = 1;
  }
  else {
    TAG_flag = 0;
    Select_ISO_IEC_18092_Protocol();
  }
}

// Read the NFC Forum tags
void CR95HF::GetNFCTag() {
  sdata[0] = 0x00;
  sdata[1] = 0xFF;
  sdata[2] = 0xFF;
  sdata[3] = 0x00;
  sdata[4] = 0x00;
  writeCmd(SendRecv, 5);
  readCmd();

  if (res == 0x80) {
    for (j = 0; j < dataNum; j++) {
      ID += String(rdata[j], HEX);
    }
    NFC_flag = 1;
  }
  else {
    NFC_flag = 0;
    Select_ISO_IEC_14443_A_Protocol();
  }
}

String CR95HF::getID() {

  String id = "";

  if (!TAG_flag)
    GetNFCTag();                                               // Get NFC ID

  if (!NFC_flag)
    GetTagID();                                                // Get Tag ID

  if (ID.c_str()[0] == 0) {                                    // If there is no tag present
    flag++;                                                    // Increment counter flag
  }
  else {                                                       // If tag is present
    flag = 0;                                                  // Reset counter flag
    id = ID;                                                   // Set current ID as previous ID
  }
  ID = "";                                                     // Terminate the ID string

  // Clear read and write buffers
  for (j = 0; j < 18; j++) {
    rdata[j] = 0;
    sdata[j] = 0;
  }

  return id;
}
