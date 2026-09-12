/*
   RailCom Detector ESP32 - v4.6

   Evolution de la version v3.1 :
   - reception RailCom par le driver UART ESP-IDF
   - UART1 materiel a 250000 bauds, 8N1, RX GPIO0
   - horloge UART forcee sur APB
   - reception evenementielle via la file d'evenements UART
   - lecture evenementielle, parseur continu independant des frontieres UART_DATA
   - suppression du polling Serial1.available() et des temporisations de reception
   - conservation du decodage 4/8 RailCom canal 1
   - validation d'une nouvelle adresse par 5 reconstructions identiques
   - presence rafraichie des qu'une adresse deja validee est revue
   - perte RailCom apres 1 seconde sans revoir l'adresse validee
   - sortie : Serial uniquement

   © Christophe BOBILLE - Locoduino
   https://github.com/BOBILLEChristophe/Railcom-detector-on-ESP32/edit/main/railcom_detector_ESP32.ino
*/

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

#include <Arduino.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define VERSION "v 4.6-serial"
#define PROJECT "Railcom Detector ESP32 - UART event driven"
#define AUTHOR  "christophe BOBILLE - Locoduino"

// -----------------------------------------------------------------------------
// Configuration RailCom / UART
// -----------------------------------------------------------------------------

constexpr uart_port_t RAILCOM_UART_NUM = UART_NUM_1;
constexpr gpio_num_t RAILCOM_RX_PIN = GPIO_NUM_0;
constexpr uint32_t RAILCOM_BAUD_RATE = 250000;

// Le FIFO materiel de l'ESP32 classique fait 128 octets.
// Le buffer driver doit etre strictement superieur au FIFO.
constexpr int RAILCOM_RX_BUFFER_SIZE = 256;
constexpr int RAILCOM_EVENT_QUEUE_SIZE = 20;

// Seuil volontairement eleve : une rafale RailCom normale (max. 8 octets
// sur un cutout) sera normalement livree par timeout, pas par FIFO plein.
constexpr uint8_t RAILCOM_RX_FIFO_THRESHOLD = 120;

// Timeout exprime en periodes de symbole UART.
// A 250 kbit/s en 8N1 : 1 symbole ~= 40 us.
// Le parseur V4.3 est independant des frontieres d'evenements UART,
// donc un timeout court (~80 us) donne une bonne reactivite sans imposer
// que les deux octets d'un datagramme arrivent dans le meme evenement.
constexpr uint8_t RAILCOM_RX_TIMEOUT_SYMBOLS = 2;

// Nombre de reconstructions identiques consecutives avant validation.
constexpr uint8_t ADDRESS_CONFIRMATIONS = 5;

// Une adresse validee est consideree perdue si elle n'est plus reconstruite
// correctement pendant ce delai.
// 1000 ms evite les extinctions sur un trou ponctuel tout en restant reactif.
constexpr uint32_t RAILCOM_LOSS_TIMEOUT_MS = 1000;

QueueHandle_t railcomUartQueue = nullptr;

// -----------------------------------------------------------------------------
// Etat RailCom
// -----------------------------------------------------------------------------

volatile uint16_t currentAddress = 0;

uint8_t adr1Data = 0;      // Datagramme ID1 : ADR Address High
uint8_t adr2Data = 0;      // Datagramme ID2 : ADR Address Low
bool adr1Valid = false;
bool adr2Valid = false;

uint16_t candidateAddress = 0;
uint8_t confirmationCount = 0;
uint16_t lastValidatedAddress = 0;
uint32_t lastValidatedReceptionMs = 0;

bool waitingSecondSymbol = false;
uint8_t firstDecodedSymbol = 0;

// -----------------------------------------------------------------------------
// Table de decodage RailCom 4/8
// raw UART -> valeur 6 bits (0..63), 64..66 = mots de controle, 255 = invalide
// Table reprise de la version v3.1.
// -----------------------------------------------------------------------------

constexpr uint8_t decodeArray[256] = {
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,  64,
  255, 255, 255, 255, 255, 255, 255,  51, 255, 255, 255,  52, 255,  53,  54, 255,
  255, 255, 255, 255, 255, 255, 255,  58, 255, 255, 255,  59, 255,  60,  55, 255,
  255, 255, 255,  63, 255,  61,  56, 255, 255,  62,  57, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255,  36, 255, 255, 255,  35, 255,  34,  33, 255,
  255, 255, 255,  31, 255,  30,  32, 255, 255,  29,  28, 255,  27, 255, 255, 255,
  255, 255, 255,  25, 255,  24,  26, 255, 255,  23,  22, 255,  21, 255, 255, 255,
  255,  37,  20, 255,  19, 255, 255, 255,  50, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,  14, 255,  13,  12, 255,
  255, 255, 255,  10, 255,   9,  11, 255, 255,   8,   7, 255,   6, 255, 255, 255,
  255, 255, 255,   4, 255,   3,   5, 255, 255,   2,   1, 255,   0, 255, 255, 255,
  255,  15,  16, 255,  17, 255, 255, 255,  18, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255,  43,  48, 255, 255,  42,  47, 255,  49, 255, 255, 255,
  255,  41,  46, 255,  45, 255, 255, 255,  44, 255, 255, 255, 255, 255, 255, 255,
  255,  66,  40, 255,  39, 255, 255, 255,  38, 255, 255, 255, 255, 255, 255, 255,
   65, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};


// Contrôles de cohérence de la table 4/8.
// Ces quatre codes sont ceux réellement observés avec la locomotive d'adresse 12.
static_assert(decodeArray[0x99] == 8,  "RailCom 4/8 : 0x99 doit decoder en 8");
static_assert(decodeArray[0x8E] == 12, "RailCom 4/8 : 0x8E doit decoder en 12");
static_assert(decodeArray[0xA3] == 4,  "RailCom 4/8 : 0xA3 doit decoder en 4");
static_assert(decodeArray[0xAC] == 0,  "RailCom 4/8 : 0xAC doit decoder en 0");

// -----------------------------------------------------------------------------
// Initialisation UART RailCom
// -----------------------------------------------------------------------------

void initRailComUart()
{
  uart_config_t uartConfig = {};
  uartConfig.baud_rate = RAILCOM_BAUD_RATE;
  uartConfig.data_bits = UART_DATA_8_BITS;
  uartConfig.parity = UART_PARITY_DISABLE;
  uartConfig.stop_bits = UART_STOP_BITS_1;
  uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uartConfig.source_clk = UART_SCLK_APB;

  ESP_ERROR_CHECK(uart_param_config(RAILCOM_UART_NUM, &uartConfig));

  // RX uniquement : aucun GPIO TX n'est necessaire.
  ESP_ERROR_CHECK(uart_set_pin(
    RAILCOM_UART_NUM,
    UART_PIN_NO_CHANGE,
    RAILCOM_RX_PIN,
    UART_PIN_NO_CHANGE,
    UART_PIN_NO_CHANGE
  ));

  ESP_ERROR_CHECK(uart_driver_install(
    RAILCOM_UART_NUM,
    RAILCOM_RX_BUFFER_SIZE,
    0,                         // pas de buffer TX
    RAILCOM_EVENT_QUEUE_SIZE,
    &railcomUartQueue,
    0
  ));

  ESP_ERROR_CHECK(uart_set_rx_full_threshold(
    RAILCOM_UART_NUM,
    RAILCOM_RX_FIFO_THRESHOLD
  ));

  ESP_ERROR_CHECK(uart_set_rx_timeout(
    RAILCOM_UART_NUM,
    RAILCOM_RX_TIMEOUT_SYMBOLS
  ));

  uart_flush_input(RAILCOM_UART_NUM);
}

// -----------------------------------------------------------------------------
// Decodage RailCom
// -----------------------------------------------------------------------------

bool decode4of8(uint8_t raw, uint8_t &decoded)
{
  const uint8_t value = decodeArray[raw];

  // Pour une adresse, on attend un symbole de donnees 6 bits.
  // 64, 65 et 66 sont des mots de controle RailCom.
  if (value > 63)
  {
    return false;
  }

  decoded = value;
  return true;
}

bool buildAddress(uint16_t &address)
{
  // ADR1 indique le type d'adresse transmis par RailCom :
  //
  //   0x00       : adresse primaire courte
  //   0x60       : adresse de consist
  //   10xxxxxx   : adresse etendue
  //
  // Toute autre valeur d'ADR1 est rejetee. Le codage 4/8 garantit qu'un
  // symbole est valide, mais pas que son contenu constitue un ADR1 coherent.

  // Adresse primaire courte : ADR2 contient 0AAAAAAA.
  if (adr1Data == 0x00)
  {
    if ((adr2Data & 0x80) != 0)
    {
      return false;
    }

    address = static_cast<uint16_t>(adr2Data & 0x7F);
    return address != 0;
  }

  // Adresse de consist : ADR2 contient RAAAAAAA.
  // Le bit 7 indique le sens relatif dans le consist ; address() retourne
  // uniquement l'adresse du consist.
  if (adr1Data == 0x60)
  {
    address = static_cast<uint16_t>(adr2Data & 0x7F);
    return address != 0;
  }

  // Adresse DCC etendue : ADR1 = 10AAAAAA et ADR2 = AAAAAAAA.
  if ((adr1Data & 0xC0) == 0x80)
  {
    address = static_cast<uint16_t>(
      (static_cast<uint16_t>(adr1Data & 0x3F) << 8) |
      adr2Data
    );

    return address != 0;
  }

  return false;
}

void validateAddress(uint16_t address)
{
  if (address == 0)
  {
    candidateAddress = 0;
    confirmationCount = 0;
    return;
  }

  const uint32_t now = millis();

  // Adresse deja validee :
  // une seule reconstruction complete et correcte suffit pour confirmer
  // que la locomotive est toujours presente.
  if ((lastValidatedAddress != 0) &&
      (address == lastValidatedAddress))
  {
    lastValidatedReceptionMs = now;

    // Si une autre adresse etait en cours de validation, le retour de
    // l'adresse connue casse cette serie.
    candidateAddress = 0;
    confirmationCount = 0;
    return;
  }

  // Nouvelle adresse candidate :
  // elle doit etre reconstruite 5 fois de suite avant d'etre acceptee.
  if (address != candidateAddress)
  {
    candidateAddress = address;
    confirmationCount = 1;
    return;
  }

  if (confirmationCount < ADDRESS_CONFIRMATIONS)
  {
    confirmationCount++;
  }

  if (confirmationCount < ADDRESS_CONFIRMATIONS)
  {
    return;
  }

  // Nouvelle adresse validee.
  lastValidatedAddress = address;
  currentAddress = address;
  lastValidatedReceptionMs = now;

  candidateAddress = 0;
  confirmationCount = 0;

  Serial.printf("Adresse loco validee : %u", address);
}

void checkRailComLoss()
{
  if (lastValidatedAddress == 0)
  {
    return;
  }

  const uint32_t now = millis();

  if (static_cast<uint32_t>(now - lastValidatedReceptionMs) <
      RAILCOM_LOSS_TIMEOUT_MS)
  {
    return;
  }

  // Plus aucune serie valide de confirmations depuis 1 seconde :
  // la locomotive est consideree absente.
  const uint16_t lostAddress = lastValidatedAddress;

  currentAddress = 0;
  lastValidatedAddress = 0;
  lastValidatedReceptionMs = 0;

  Serial.printf("Plus de locomotive detectee (derniere adresse : %u)\n",
                lostAddress);

  candidateAddress = 0;
  confirmationCount = 0;

  adr1Valid = false;
  adr2Valid = false;
  waitingSecondSymbol = false;
}

// -----------------------------------------------------------------------------
// Parseur continu du flux UART RailCom
// -----------------------------------------------------------------------------
//
// Important : un evenement UART_DATA n'est PAS une frontiere de trame RailCom.
// Le driver peut livrer :
//   [99 8E] [A3 AC]
// ou :
//   [F0 A3 AC 99] [8E ...]
// ou encore couper une paire entre deux evenements.
//
// On traite donc les octets comme un flux continu.
//
// Un datagramme RailCom utile pour l'adresse contient deux symboles 4/8 :
//   symbole 0 : IIII DD
//   symbole 1 : DDDDDD
//
// Pour le canal 1 :
//   ID = 1 -> ADR1 (partie haute)
//   ID = 2 -> ADR2 (partie basse)

void processAddressDatagram(uint8_t symbol0, uint8_t symbol1)
{
  const uint8_t identifier = symbol0 >> 2;
  const uint8_t data = static_cast<uint8_t>(
    ((symbol0 & 0x03) << 6) | symbol1
  );

  switch (identifier)
  {
    case 1: // ADR1 : Address High
      adr1Data = data;
      adr1Valid = true;
      break;

    case 2: // ADR2 : Address Low
      adr2Data = data;
      adr2Valid = true;
      break;

    default:
      return;
  }

  if (adr1Valid && adr2Valid)
  {
    uint16_t address = 0;
    const bool addressValid = buildAddress(address);

    // Les deux morceaux sont consommes ensemble.
    adr1Valid = false;
    adr2Valid = false;

    if (addressValid)
    {
      validateAddress(address);
    }
    else
    {
      // Une combinaison ADR1/ADR2 semantiquement invalide casse la serie
      // de validation d'une nouvelle adresse, sans affecter une adresse
      // deja validee ni son watchdog.
      candidateAddress = 0;
      confirmationCount = 0;
    }
  }
}

void feedRailComByte(uint8_t raw)
{
  const uint8_t decoded = decodeArray[raw];

  // Mot invalide ou mot de controle RailCom (64..66).
  // Il sert de separateur naturel et annule une paire incomplete.
  if (decoded > 63)
  {
    waitingSecondSymbol = false;
    return;
  }

  if (!waitingSecondSymbol)
  {
    const uint8_t identifier = decoded >> 2;

    // Pour la detection d'adresse du canal 1, seuls ID1 et ID2
    // peuvent constituer le premier symbole interessant.
    if ((identifier == 1) || (identifier == 2))
    {
      firstDecodedSymbol = decoded;
      waitingSecondSymbol = true;
    }
    return;
  }

  // Nous avons deja un premier symbole ID1/ID2.
  // Tout symbole de donnees 0..63 est valable en deuxieme position.
  processAddressDatagram(firstDecodedSymbol, decoded);

  waitingSecondSymbol = false;
}

// -----------------------------------------------------------------------------
// Tache de reception UART RailCom
// -----------------------------------------------------------------------------

void railComTask(void *parameter)
{
  (void)parameter;

  uart_event_t event;

  for (;;)
  {
    // Le reveil periodique ne sert qu'au watchdog de perte RailCom.
    // La reception UART elle-meme reste entierement evenementielle.
    if (xQueueReceive(
          railcomUartQueue,
          &event,
          pdMS_TO_TICKS(50)
        ) == pdTRUE)
    {
      switch (event.type)
      {
        case UART_DATA:
        {
          size_t remaining = event.size;

          while (remaining > 0)
          {
            uint8_t temp[32];
            const size_t requested =
              (remaining < sizeof(temp)) ? remaining : sizeof(temp);

            const int received = uart_read_bytes(
              RAILCOM_UART_NUM,
              temp,
              requested,
              0
            );

            if (received <= 0)
            {
              break;
            }

            for (int i = 0; i < received; i++)
            {
              feedRailComByte(temp[i]);
            }

            remaining -= static_cast<size_t>(received);
          }
          break;
        }

        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
          uart_flush_input(RAILCOM_UART_NUM);
          xQueueReset(railcomUartQueue);
          adr1Valid = false;
          adr2Valid = false;
          waitingSecondSymbol = false;
          candidateAddress = 0;
          confirmationCount = 0;
          break;

        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
        case UART_BREAK:
        default:
          break;
      }
    }
    checkRailComLoss();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  initRailComUart();

  // La tache UART est bloquee sur la file d'evenements quand aucune donnee
  // RailCom n'arrive : aucun polling periodique n'est necessaire.
  xTaskCreatePinnedToCore(
    railComTask,
    "RailComUART",
    4096,
    nullptr,
    5,
    nullptr,
    1
  );
}

void loop()
{
  // Tout est gere par les taches FreeRTOS et le peripherique UART.
  vTaskDelay(portMAX_DELAY);
}
