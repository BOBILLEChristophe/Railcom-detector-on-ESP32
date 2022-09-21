/*
   Marcel JEANSON - Christophe BOBILLE 09/2022 © locoduino.org
   Adaptation de : https://github.com/RWVro/DCC_RailCom_Detector/blob/main/Detector.h
*/


#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

#define VERSION "v 0.2"
#define PROJECT "Railcom Detector ESP32"

const byte railComRX  = 14;           // GPIO14 connecté à  RailCom Detector RX
const byte railComInt = 13;           // GPIO13 connecté au BRAKE du LMD18200

uint16_t receiveCnt = 0;
uint16_t displCnt = 0;
bool arrayComplete = false;

uint8_t inByte = 0;

const uint8_t rxArrayMax = 11;
uint8_t rxArray[rxArrayMax];
uint8_t rxArrayCnt = 0;

bool ok_4_8Code = true;
bool no_4_8Code = false;

bool test_4_8Code = true;
bool test_4_8Decimal = true;

int convByte;

int16_t cv = 0;

// Identifiants des données du canal 1
#define CH1_ADR_LOW  4
#define CH1_ADR_HIGH 8
byte dccAddr[2];         // Adresse du décodeur

//==================================== Conversion Arrays =============================

int decodeArray[68] = {172, 170, 169, 165, 163, 166, 156, 154, 153, 149, 147, 150, 142, 141, 139, 177, 178, 180, 184, 116,
                       114, 108, 106, 105, 101, 99, 102, 92, 90, 89, 85, 83, 86, 78, 77, 75, 71, 113, 232, 228, 226, 209, 201,
                       197, 216, 212, 210, 202, 198, 204, 120, 23, 27, 29, 30, 46, 54, 58, 39, 43, 45, 53, 57, 51, 15, 240, 225, 31
                      };       // 31 is end of table (0001 1111)


int convertArray[67] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
                        27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
                        52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66
                       };

//====================================== Convert 4_8 Table to dec. ====================

void check_4_8Code()
{
  test_4_8Code = true;

  int compareValue = inByte;
  int i = 0;
  while (compareValue != decodeArray[i])
  {
    if (decodeArray[i] == 31)             // End of table reached
    {
      test_4_8Code = false;
      goto out;
    }
    i++;
  }
out:;
}

void convert4_8ToDec()
{
  test_4_8Decimal = true;

  int compareValue = inByte;
  int i = 0;
  while (compareValue != decodeArray[i])
  {
    if (decodeArray[i] == 31)             // End of table reached
    {
      test_4_8Decimal = false;
      goto out;
    }
    i++;
  }
  inByte = convertArray[i];
out:;
}

//====================================== Print Array ============================

void printRxArray()
{
  if (rxArray[0]&CH1_ADR_HIGH) dccAddr[0] = rxArray[1] | (rxArray[0] << 6);
  if (rxArray[0]&CH1_ADR_LOW) dccAddr[1] = rxArray[1] | (rxArray[0] << 6);

  cv = (dccAddr[1] - 128) << 8;
  cv += dccAddr[0];

  if (cv < 0) {
    Serial.print("Adresse courte : ");
    Serial.println( dccAddr[0] );
  }
  else        {
    Serial.print("Adresse longue : ");
    Serial.println( cv );
  }
}

//====================================== Integer to Binair ============================

void setIntToBinString()
{
  for (int k = 7; k >= 0; k--)
    Serial.print(bitRead(convByte, k));
  Serial.print(" ");
}

void printRxArrayToBin()
{
  int i = 0;
  while (i <= rxArrayCnt)
  {
    convByte = (rxArray[i]);
    setIntToBinString();
    i ++;
  }
  Serial.println("");
}

void clearRxArray()
{
  int i = 0;
  while (i < rxArrayMax - 1)
  {
    rxArray[i] = 0;
    i ++;
  }
}

//===================================== ISR ==============================

void IRAM_ATTR isr0()
{
  rxArrayCnt = 0;
  while (Serial1.available())
  {
    for (int i  = 0; i < 8; i++)                  // Read 8 bytes
    {
      inByte = Serial1.read();                     // Read byte

      if (inByte == 0 || inByte == 255 || inByte < 0)
        goto next;

      check_4_8Code();

      //      if (test_4_8Code)                             // Check if 4-8 code is ok, if ok , print
      //      {
      //        Serial.print(inByte);                     // Print 4-8 code
      //        Serial.print(" ");
      //      }
      //      Serial.println("");

      convert4_8ToDec();

      if (test_4_8Decimal)                          // If ok_4_8Code is valid byte into  array
      {
        rxArray[rxArrayCnt] = inByte;               // Byte into  array
        if (rxArrayCnt < rxArrayMax)
          rxArrayCnt++;                              // Increment char counter
        //        else
        //          rxArrayCnt = 0;
      }
next :;
    }
  }
  if (rxArrayCnt <= 1)                              // Save only arrays with 2 or more bytes
  {
    clearRxArray();
    return;
  }
  printRxArray();
  //printRxArrayToBin();
  clearRxArray();
}

void setup()
{
  Serial.begin(115200);

  Serial.printf("\n\nProject :    %s", PROJECT);
  Serial.printf("\nVersion :      %s", VERSION);
  Serial.printf("\nFichier :      %s", __FILE__);
  Serial.printf("\nCompiled :     %s", __DATE__);
  Serial.printf(" - %s\n\n", __TIME__);

  pinMode(railComInt, INPUT);
  pinMode(railComRX, INPUT);
  Serial1.begin(250000, SERIAL_8N1, railComRX, 12);    // Define and start ESP32 serial port
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(railComInt), isr0, RISING);
}

void loop()
{}
