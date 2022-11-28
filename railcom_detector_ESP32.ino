/*
   Programme de lecture, de décodage et d'affichage des messages Railcom ©
   qui retourne l'adresse d'un décodeur (adresse courte ou longue) sur un afficheur LED 7 segments

   Fonctionne exclusivement sur ESP32
   © christophe bobille - www.locoduino.org 11/2022

   lib_deps = locoduino/RingBuffer@^1.0.3 / https://github.com/Locoduino/RingBuffer
   

*/

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

#include <Arduino.h>

#define VERSION "v 2.8"
#define PROJECT "Railcom Detector ESP32 (freeRTOS)"

#include <RingBuf.h>
#define NB_ADDRESS_TO_COMPARE 100                // Nombre de valeurs à comparer pour obtenir l'adresse de la loco
RingBuf<uint16_t, NB_ADDRESS_TO_COMPARE> buffer; // Instance

// Identifiants des données du canal 1
#define CH1_ADR_LOW  ( 1 << 2 )
#define CH1_ADR_HIGH ( 1 << 3 )

const byte railComRX = 14; // GPIO14 connecté à RailCom Detector RX
const byte railComTX = 17; // GPIO17 non utilisée mais doit être déclarée

const byte cutOutPin = GPIO_NUM_4;

// Queue
#define QUEUE_SIZE_0 10
QueueHandle_t xQueue_0;

#define QUEUE_SIZE_1 20
QueueHandle_t xQueue_1;

#define QUEUE_SIZE_2 2
QueueHandle_t xQueue_2;


void receiveData(void *p)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  uint8_t inByte {0};
  uint8_t compt {0};
  for (;;)
  {
    while (Serial1.available() > 0)  // Sans détection du cutout
    //while ((Serial1.available() > 0) && (! digitalRead(cutOutPin)))
    {
      if (compt == 0)
        inByte = '\0';
      else
        inByte = (uint8_t)Serial1.read();
      if (compt < 3)
        xQueueSend(xQueue_0, &inByte, 0);
      compt++;
    }
    compt = 0;
    //Serial.println("---------");
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1)); // toutes les x ms
  }
}

void parseData(void *p)
{
  bool start{false};
  byte inByte{0};
  uint8_t rxArray[8] {0};
  uint8_t rxArrayCnt{0};
  byte dccAddr[2] {0};
  int16_t address{0};
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  byte decodeArray[] = {172, 170, 169, 165, 163, 166, 156, 154, 153, 149, 147, 150, 142, 141, 139, 177, 178, 180, 184, 116,
                        114, 108, 106, 105, 101, 99, 102, 92, 90, 89, 85, 83, 86, 78, 77, 75, 71, 113, 232, 228, 226, 209, 201,
                        197, 216, 212, 210, 202, 198, 204, 120, 23, 27, 29, 30, 46, 54, 58, 39, 43, 45, 53, 57, 51, 15, 240, 225, 31
                       }; // 31 is end of table (0001 1111)

  auto check_4_8_code = [&]() -> bool
  {
    uint8_t index = 0;
    while (inByte != decodeArray[index])
    {
      if (decodeArray[index] == 31)
        return false;
      index++;
    }
    inByte = index;
    return true;
  };

  auto printAdress = [&]()
  {
    //Serial.printf("Adresse loco : %d\n", address);
    xQueueSend(xQueue_1, &address, portMAX_DELAY);
  };

  for (;;)
  {
    do
    {
      xQueueReceive(xQueue_0, &inByte, pdMS_TO_TICKS(portMAX_DELAY));

      if (inByte == '\0')
        start = true;
    } while (!start);
    start = false;

    for (byte i = 0; i < 2; i++)
    {
      if (xQueueReceive(xQueue_0, &inByte, pdMS_TO_TICKS(portMAX_DELAY)) == pdPASS)
      {
        if (inByte >= 0x0F && inByte <= 0xF0)
        {
          if (check_4_8_code())
          {
            rxArray[rxArrayCnt] = inByte;
            rxArrayCnt++;
          }
        }
      }
    }

    if (rxArrayCnt == 2)
    {
      if (rxArray[0] & CH1_ADR_HIGH)
        dccAddr[0] = rxArray[1] | (rxArray[0] << 6);
      if (rxArray[0] & CH1_ADR_LOW)
        dccAddr[1] = rxArray[1] | (rxArray[0] << 6);
      address = (dccAddr[1] - 128) << 8;
      if (address < 0)
        address = dccAddr[0];
      else
        address += dccAddr[0];

      bool testOk = true;
      uint16_t j = 0;
      buffer.pop(j);
      buffer.push(address);
      do
      {
        if (buffer[j] != address) {
          testOk = false;
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
        }
        j++;
      } while (testOk && j <= buffer.size());

      if (testOk)
        printAdress();
      // else
      //   Serial.println("NOK");
    }

    rxArrayCnt = 0;
    for (byte i = 0; i < 2; i++)
      rxArray[i] = 0;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // toutes les x ms
  }
}

void printAddress(void *p)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  uint16_t address{0};

  for (;;)
  {
    address = 0;
    xQueueReceive(xQueue_1, &address, pdMS_TO_TICKS(0));
    xQueueSend(xQueue_2, &address, portMAX_DELAY);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // toutes les x ms
  }
}


void displayAddAddress(void *p)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  uint16_t address{0};
  const byte pinOutCathode[] = {22, 21, 32, 25, 26, 23, 33};
  const byte pinOutAnode[] = {19, 18, 12, 13};
  const byte chiffre[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x7, 0x7F, 0x6F};
  uint8_t unit, dizaine, centaine, millier;

  for (auto el : pinOutCathode)
  {
    pinMode(el, OUTPUT);
    digitalWrite(el, HIGH);
  }

  for (auto el : pinOutAnode)
  {
    pinMode(el, OUTPUT);
    digitalWrite(el, LOW);
  }

  auto separateur = [&](uint16_t address)
  {
    if (address > 999)
    {
      millier = address / 1000; // on recupere les milliers
      address -= millier * 1000;
    }
    else
      millier = 0;

    if (address > 99)
    {
      centaine = address / 100; // on recupere les centaines
      address -= centaine * 100;
    }
    else
      centaine = 0;

    if (address > 9)
    {
      dizaine = address / 10; // on recupere les centaines
      address -= dizaine * 10;
    }
    else
      dizaine = 0;

    unit = address; // on recupere les unites
  };

  for (;;)
  {
    xQueueReceive(xQueue_2, &address, pdMS_TO_TICKS(0));
    separateur(address);

    for (byte j = 0; j < 4; j++)
    {
      digitalWrite(pinOutAnode[j], HIGH); // Unités
      bool etat;
      for (byte i = 0; i < 7; i++)
      {
        switch (j)
        {
          case 0:
            etat = (chiffre[unit] & (1 << i)) >> i;
            break;
          case 1:
            if (millier > 0  || centaine > 0 || dizaine > 0)
              etat = (chiffre[dizaine] & (1 << i)) >> i;
            break;
          case 2:
            if (millier > 0 || centaine > 0)
              etat = (chiffre[centaine] & (1 << i)) >> i;
            break;
          case 3:
            if (millier > 0 )
              etat = (chiffre[millier] & (1 << i)) >> i;
            break;
        }
        digitalWrite(pinOutCathode[i], !etat);
        etat = 0;
      }
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1)); // toutes les x ms
      digitalWrite(pinOutAnode[j], LOW);
    }
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.printf("\n\nProject :    %s", PROJECT);
  Serial.printf("\nVersion :      %s", VERSION);
  Serial.printf("\nFichier :      %s", __FILE__);
  Serial.printf("\nCompiled :     %s", __DATE__);
  Serial.printf(" - %s\n\n", __TIME__);

  Serial1.begin(250000, SERIAL_8N1, railComRX, railComTX); // Port série pour la réception des données (250k bauds)
  uint16_t x = 0;
  for (uint8_t i = 0; i < NB_ADDRESS_TO_COMPARE; i++) // On place des zéros dans le buffer de comparaison
    buffer.push(x);
  pinMode(cutOutPin, INPUT_PULLUP);
  xQueue_0 = xQueueCreate(QUEUE_SIZE_0, sizeof(uint8_t));
  xQueue_1 = xQueueCreate(QUEUE_SIZE_1, sizeof(uint16_t));
  xQueue_2 = xQueueCreate(QUEUE_SIZE_2, sizeof(uint16_t));                                   // Création de la file pour les échanges de data entre les 2 tâches
  xTaskCreatePinnedToCore(receiveData,       "ReceiveData",    2 * 1024, NULL, 4, NULL, 1);  // Création de la tâches pour la réception
  xTaskCreatePinnedToCore(parseData,         "ParseData",      2 * 1024, NULL, 5, NULL, 0);  // Création de la tâches pour le traitement
  xTaskCreatePinnedToCore(printAddress,      "PrintAddress",   2 * 1024, NULL, 4, NULL, 0);  // Création de la tâches pour l'affichage (1/2)
  xTaskCreatePinnedToCore(displayAddAddress, "DisplayAddress", 2 * 1024, NULL, 5, NULL, 0);  // Création de la tâches pour l'affichage (2/2)
}

void loop()
{
}
