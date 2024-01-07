/*
 *  Le code suivant est destiné à la création et au fonctionnement d'un système embarqué pourvu de capteurs
 *  Ce système est constitué d'un Arduino Uno ainsi que des capteurs suivants : Température d'air, pression atmosphérique, hygrométrie dans l'air, localisation gps, horodatage, luminosité.
 *  Il doit, par ailleurs, inclure l'ensemble des éléments nécessaires au rajout des capteurs suivants : Force des courants marins, température de l'eau, force du vent, taux de particules fines.
 *  Le système possède une entrée pour sa configuration et sa maintenance : 2 boutons analogiques
 *  Le système possède les sorties suivantes : carte SD
 *  Le système doit être modulaire et se contenter d'un minimum de place pour fonctionner, la gestion de celle-ci y est donc minutieuse et adaptée à la taille du système
 *
 *  Le système fonctionne selon 4 mode de fonctionnement différents : Standard, maintenance, configuration et économique
 *   - Le mode standard démarre de lui-même à la mise sous tension de la carte électronique et collecte les données de l'ensemble des capteurs. Il les stocke sur la carte SD dans des fichiers
 *     au format .LOG et se limite à 2ko par fichier.
 *   - Le mode économique, accessible depuis le mode standard en appuyant sur le bouton vert 5 secondes, est semblable au mode standard mais réduisant l'utilisation des capteurs. Pour le quitter
 *     il suffit de maintenir le bouton rouge 5 secondes.
 *   - Le mode mainteance, accessible depuis le mode standard en appuyant sur le bouton rouge 5 secondes, permet le changement de la carte SD sans risque de corrompre les données. Il retourne
 *     également les données des capteurs en direct dans l'interface série.
 *   - Le mode configuration est accessible en maintenant le bouton rouge appuyé au démarrage, ce mode permet de configurer les paramètres du système, les capteurs sont désactivés durant toute
 *     son utilisation. Si aucune activité n'est constatée par le système pendant plus de 30 minutes, le système bascule de lui-même en mode standard.
 */

// ! laisser 10 secondes au démarrage pour laisser à l'utilisateur le temps de passer en mode config ou non

#include <Arduino.h>
#include <time.h>
#include <BME280I2C.h>
#include <SPI.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <SdFat.h>
#include <sdios.h>
#include <ChainableLED.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

#define SERIAL_BAUD 9600

// * création d'un second périphérique en connexion port série pour le GPS
SoftwareSerial SoftSerial(4,5); // * pin du GPS
// * Définition du module BME
BME280I2C bme;
// * Définition du module RTC
RTC_DS1307 rtc;
// TODO : finir la connexion au port virtuel en changeant les ports sur l'arduino et dans le code
// * Port série virtuel créé pour le module rtc

uint8_t Nb_Capteurs;
uint8_t indice_max_capteurs = 10;

// * définition des pins pour les boutons
const uint8_t buttonV = 2; // * défintion du bouton 1 (vert) sur le pin d2
const uint8_t buttonR = 3; // * définition du bouton 2 (rouge) sur le pin d3
bool state_BPV = 1;        // * état booléen du bouton vert
bool state_BPR = 1;        // * état booléen du bouton rouge

// * définition du capteur de luminosité
const uint8_t pinLight = A0; // light sensor connect to A0

// * variables liées au paramétrage du système de données
const uint8_t chipSelect = 4;

SdFat sd;             // * File system object
SdFile myFile;        // * Log File
uint8_t revision = 0; // * version du fichier (7 révisions par jour, un log par tranche de 10 min et 24h de log soit 144 log, sachant qu'un fichier peut stocker 21 lignes de log)

// * Variables liées à la led
#define NUM_LEDS 1
ChainableLED leds(7, 8, NUM_LEDS);

// * la variable suivante définit le coef de multiplication du délai entre chaque log
uint8_t timeMultiplier;

/*
 * les lignes suivantes sont les structures servant à la configuration des paramètres de chacun des capteurs
 * sont paramétrées ici les limites des capteurs, le temps entre chaque log, la limite de poids des fichiers
 * ainsi que le time out des capteurs
 */
uint16_t eepromAddress = 0; // * Adresse de début de stockage dans l'EEPROM

// * Paramètres de configuration
typedef struct
{
  uint16_t LOG_INTERVAL;  // * Interval entre les mesures en secondes (en minutes)
  uint16_t FILE_MAX_SIZE; // * Taille maximale du fichier en octets - bit non signé pour les calculs avec la taille
  uint16_t TIME_OUT;      // * Durée au bout de laquelle l’acquisition des données d’un capteur est abandonnée(en s)
  bool LUMIN;             // * État du capteur de luminosité (activé par défaut)
  uint16_t LUMIN_LOW;     // * Seuil de luminosité "faible"
  uint16_t LUMIN_HIGH;    // * Seuil de luminosité "fort"
  bool TEMP_AIR;          // * État du capteur de température de l'air (activé par défaut)
  int16_t MIN_TEMP_AIR;   // * Seuil de température minimale de l'air - Valeurs à multiplier par 10
  int16_t MAX_TEMP_AIR;   // * Seuil de température maximale de l'air - Valeurs à multiplier par 10
  bool HYGR;              // * État du capteur d'hygrométrie (activé par défaut) (pourcentage)
  uint16_t HYGR_MINT;     // * Seuil de température minimale pour les mesures d'hygrométrie - Valeurs à multiplier par 10
  uint16_t HYGR_MAXT;     // * Seuil de température maximale pour les mesures d'hygrométrie - Valeurs à multiplier par 10
  bool PRESSURE;          // * État du capteur de pression atmosphérique (activé par défaut)
  uint32_t PRESSURE_MIN;  // * Seuil de pression atmosphérique minimale
  uint32_t PRESSURE_MAX;  // * Seuil de pression atmosphérique maximale
} configuration;

configuration configure;

//* Configuration par défaut
configuration default_parameters = {
    .LOG_INTERVAL = 600,
    .FILE_MAX_SIZE = 4096,
    .TIME_OUT = 30,
    .LUMIN = true,
    .LUMIN_LOW = 255,
    .LUMIN_HIGH = 768,
    .TEMP_AIR = true,
    .MIN_TEMP_AIR = -10,
    .MAX_TEMP_AIR = 60,
    .HYGR = true,
    .HYGR_MINT = 0,
    .HYGR_MAXT = 50,
    .PRESSURE = true,
    .PRESSURE_MIN = 850,
    .PRESSURE_MAX = 1080};

// Fonction pour afficher la configuration
void printConfig()
{
  Serial.print(F("LOG_INTERVAL : "));
  Serial.println(configure.LOG_INTERVAL);
  Serial.print(F("FILE_MAX_SIZE : "));
  Serial.println(configure.FILE_MAX_SIZE);
  Serial.print(F("TIME_OUT : "));
  Serial.println(configure.TIME_OUT);
  Serial.print(F("LUMIN : "));
  Serial.println(configure.LUMIN);
  Serial.print(F("LUMIN_LOW : "));
  Serial.println(configure.LUMIN_LOW);
  Serial.print(F("LUMIN_HIGH : "));
  Serial.println(configure.LUMIN_HIGH);
  Serial.print(F("TEMP_AIR : "));
  Serial.println(configure.TEMP_AIR);
  Serial.print(F("MIN_TEMP_AIR : "));
  Serial.println(configure.MIN_TEMP_AIR);
  Serial.print(F("MAX_TEMP_AIR : "));
  Serial.println(configure.MAX_TEMP_AIR);
  Serial.print(F("HYGR : "));
  Serial.println(configure.HYGR);
  Serial.print(F("HYGR_MINT : "));
  Serial.println(configure.HYGR_MINT);
  Serial.print(F("HYGR_MAXT : "));
  Serial.println(configure.HYGR_MAXT);
  Serial.print(F("PRESSURE : "));
  Serial.println(configure.PRESSURE);
  Serial.print(F("PRESSURE_MIN : "));
  Serial.println(configure.PRESSURE_MIN);
  Serial.print(F("PRESSURE_MAX : "));
  Serial.println(configure.PRESSURE_MAX);
  Serial.println(F(" "));
}

// lit le buffer jusqu’à pouvoir interpréter le texte comme un bool
bool parseBool()
{
  while (!Serial.available())
  {
  }
  char input = Serial.read();
  if (input == '1')
  {
    return true;
  }
  else
  {
    return false;
  }
}

void registeredConfig()
{
  // * Vérifier s'il y'a une structure enregistré dans la mémoire
  bool hasConfig = EEPROM.read(eepromAddress);
  if (hasConfig)
  { // Une configuration existe dans l'EEPROM
    Serial.println(F("Configuration des paramètres actuels enregistrée dans l'EEPROM :"));
    EEPROM.get(eepromAddress, configure);
    printConfig();
  }
  else
  { // Aucune configuration dans l'EEPROM
    Serial.println(F("Fichier de configuration inexistant"));
  }
}
// Fonction pour modifier les paramètres
void ChangeConfig()
{
  Serial.println(F("Entrez les paramètres de configuration :"));
  // LOG_INTERVAL
  Serial.print(F("LOG_INTERVAL (en secondes) : "));
  while (!Serial.available())
  {
  }
  configure.LOG_INTERVAL = Serial.parseInt();
  Serial.println(configure.LOG_INTERVAL);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // FILE_MAX_SIZE
  Serial.print(F("FILE_MAX_SIZE (en octets) : "));
  while (!Serial.available())
  {
  }
  configure.FILE_MAX_SIZE = Serial.parseInt();
  Serial.println(configure.FILE_MAX_SIZE);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // TIME_OUT
  Serial.print(F("TIME_OUT (en millisecondes) : "));
  while (!Serial.available())
  {
  }
  configure.TIME_OUT = Serial.parseInt();
  Serial.println(configure.TIME_OUT);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // LUMIN
  Serial.print(F("LUMIN (1 pour vrai, 0 pour faux) : "));
  while (!Serial.available())
  {
  }
  configure.LUMIN = parseBool();
  Serial.println(configure.LUMIN);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // LUMIN_LOW
  Serial.print(F("LUMIN_LOW : "));
  while (!Serial.available())
  {
  }
  configure.LUMIN_LOW = Serial.parseInt();
  Serial.println(configure.LUMIN_LOW);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // LUMIN_HIGH
  Serial.print(F("LUMIN_HIGH : "));
  while (!Serial.available())
  {
  }
  configure.LUMIN_HIGH = Serial.parseInt();
  Serial.println(configure.LUMIN_HIGH);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // TEMP_AIR
  Serial.print(F("TEMP_AIR (1 pour vrai, 0 pour faux) : "));
  while (!Serial.available())
  {
  }
  configure.TEMP_AIR = parseBool();
  Serial.println(configure.TEMP_AIR);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // MIN_TEMP_AIR
  Serial.print(F("MIN_TEMP_AIR (pour 26.5° écrire 265) : "));
  while (!Serial.available())
  {
  }
  configure.MIN_TEMP_AIR = Serial.parseInt();
  Serial.println(configure.MIN_TEMP_AIR);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // MAX_TEMP_AIR
  Serial.print(F("MAX_TEMP_AIR (pour 26.5° écrire 265) : "));
  while (!Serial.available())
  {
  }
  configure.MAX_TEMP_AIR = Serial.parseInt();
  Serial.println(configure.MAX_TEMP_AIR);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // HYGR
  Serial.print(F("HYGR (1 pour vrai, 0 pour faux) : "));
  while (!Serial.available())
  {
  }
  configure.HYGR = parseBool();
  Serial.println(configure.HYGR);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // HYGR_MINT
  Serial.print(F("HYGR_MINT (pour 95.6% écrire 956): "));
  while (!Serial.available())
  {
  }
  configure.HYGR_MINT = Serial.parseInt();
  Serial.println(configure.HYGR_MINT);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // HYGR_MAXT
  Serial.print(F("HYGR_MAXT (pour 95.6% écrire 956): "));
  while (!Serial.available())
  {
  }
  configure.HYGR_MAXT = Serial.parseInt();
  Serial.println(configure.HYGR_MAXT);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // PRESSURE
  Serial.print(F("PRESSURE (1 pour vrai, 0 pour faux) : "));
  while (!Serial.available())
  {
  }
  configure.PRESSURE = parseBool();
  Serial.println(configure.PRESSURE);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // PRESSURE_MIN
  Serial.print(F("PRESSURE_MIN : "));
  while (!Serial.available())
  {
  }
  configure.PRESSURE_MIN = Serial.parseInt();
  Serial.println(configure.PRESSURE_MIN);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }

  // PRESSURE_MAX
  Serial.print(F("PRESSURE_MAX : "));
  while (!Serial.available())
  {
  }
  configure.PRESSURE_MAX = Serial.parseInt();
  Serial.println(configure.PRESSURE_MAX);
  // Nettoyer le tampon d'entrée série
  while (Serial.read() != '\n')
  {
  }
}

// * Version du programme
uint8_t version = 1;
uint8_t lot = 1;
char action[10];

// * condition de vérification de la présence de luminosité
bool isSysLighted()
{
  return (analogRead(pinLight) > 50); // * si la luminosité est supérieure à 50, on retourne true
}
// * définition de la fonction créée plus bas dans le code pour l'appel dans le setup
void myInterruptFunction();
// * défintion de la fonction créée plus bas dans le code
void ledColor(byte i);

void setup()
{
  // * Initialisation de l'arduino
  SoftSerial.begin(SERIAL_BAUD); // * cadence des ports série
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Nb_Capteurs += 1;
  bme.begin();
  Nb_Capteurs += 1;
  rtc.begin();
  if (!rtc.begin())
  {
    ledColor(4);
  }
  Nb_Capteurs += 1;
  pinMode(A0, INPUT); 
  while (!sd.begin(4))
  {
    Serial.println(F("Impossible d'initialiser la carte SD."));
    ledColor(9);
  }
  registeredConfig();
  // * Initialisation des bouttons
  pinMode(buttonV, INPUT);
  pinMode(buttonR, INPUT);
  attachInterrupt(digitalPinToInterrupt(buttonV), myInterruptFunction, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonR), myInterruptFunction, CHANGE);
  Serial.println(F("init successful"));
}

// * structure stockant les valeurs des différents capteurs
typedef struct capteurs
{
  char* timeStamp;
  char* humidity;
  char* temperature;
  char* pressure;
  char* lightSensor;
  char* GPSPosition;
  // * Ajoutez des champs conditionnellement si Nb_Capteurs >= 7
#if Nb_Capteurs >= 7
  float WaterForce;
  float WaterTemperature;
  float wideSpeed;
#endif
} capteurs;

// * Les lignes suivantes correspondent au capteur BME280 (humidité, température air, pression atmo)
char* BME280Data(byte value) {
  float tempAir, humAir, pres;
  
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);

  bme.read(pres, tempAir, humAir, tempUnit, presUnit);
  
  static char returnedValueHygro[5]; 
  static char returnedValueTemp[5];  
  static char returnedValuePres[6];  
  
  switch (value) {
    case 1:
      if (configure.HYGR == 1) {
        if (humAir <= configure.HYGR_MINT/10 || humAir >= configure.HYGR_MAXT/10) {
          return "NA";
          if (humAir < 10 || humAir > 90) {
          ledColor(6);
          }
        } else {
          dtostrf(humAir, 4, 1, returnedValueHygro); // * conversion et attribution de la valeur à la chaine returnedValueHygro      
          return returnedValueHygro;
        }
      } else {
        return "OFF";
      }
      break;
    case 2:
      if (configure.TEMP_AIR == 1) {
        if (tempAir <= configure.MIN_TEMP_AIR/10 || tempAir >= configure.MAX_TEMP_AIR/10) {
          return "NA";
          if (tempAir < -35 || tempAir > 120) {
            ledColor(6);
          }
        } else {
          dtostrf(tempAir, 4, 1, returnedValueTemp); // * conversion et attribution de la valeur à la chaine returnedValueTemp
          return returnedValueTemp;
        }
      } else {
        return "OFF";
      }
      break;
    case 3:
      if (configure.PRESSURE == 1) {
        if (pres <= configure.PRESSURE_MIN || pres >= configure.PRESSURE_MAX) {
          return "NA";
          if (pres < 80000 || pres > 120000) {
            ledColor(6);
          }
        } else {
          dtostrf(pres, 5, 0, returnedValuePres); // * conversion et attribution de la valeur à la chaine returnedValuePres
          return returnedValuePres;
        }
      } else {
        return "OFF";
      }
      break;
    default:
      ledColor(6);
      return "";
  }
}

// * fonction retournant la donnée du capteur de luminosité
char* lightSensor() {
  if (configure.LUMIN == 1) {
    uint16_t value = analogRead(pinLight);
    static char light[5];
    snprintf(light, sizeof(light), "%d", value);
      if (value <= configure.LUMIN_LOW || value >= configure.LUMIN_HIGH) {
        return "NA";
      } else if ((value < 10) || (value > 1000)) {
        ledColor(6);
      }else{
        return light;
      }
    } else {
      return "OFF";
    }
}

char * GPSData() {
  static char buffer[64]; // * Tableau de caractères pour stocker la trame
  static int count = 0; // * Index pour suivre la position dans le tableau
  while (SoftSerial.available()) {
    char incomingChar = SoftSerial.read();
    
    if (incomingChar == '$') {
      count = 0; // Réinitialiser l'index quand le préfixe est trouvé
      memset(buffer, 0, sizeof(buffer)); // Réinitialiser le tableau
    }

    buffer[count++] = incomingChar;

    if (incomingChar == '\n') {
      // Trame complète reçue, vérification du préfixe
      if (strncmp(buffer, "$GPGGA", 6) == 0) {
        // C'est une trame GPGGA, nous pouvons la traiter
        return buffer; // Retourner un pointeur vers le contenu du buffer
      }

      count = 0; // Réinitialiser l'index
      memset(buffer, 0, sizeof(buffer)); // Réinitialiser le tableau
    }
  }

  if (Serial.available()) {
    SoftSerial.write(Serial.read());
  }
  
  return NULL; // Retourner NULL si aucune trame GPGGA n'a été trouvée
}
void test(){
  char * namur = GPSData(); // Appeler la fonction pour obtenir la trame GPGGA
  if (namur != NULL) {
    Serial.println(namur); // Afficher la trame GPGGA
  }
}

// * fonction retournant l'heure du système
char * horodatage(){
  DateTime now = rtc.now();
  static char timeStr[9]; // Un tableau de 9 caractères pour stocker l'heure (HH:MM:SS)
  sprintf(timeStr, "%02u:%02u:%02u", now.hour()-1, now.minute(), now.second());
  if (timeStr == NULL) {
      ledColor(4);
  }else{
    return timeStr;
  }
}

// ! changements à vérifier
// * fonction retournant le jour sous le format suivant AAMMDD
char* nameDay()
{
  DateTime now = rtc.now();
  static char date[8];
  sprintf(date, "%d%02d%02d", (now.year() % 100), now.month(), now.day());
  if (date == NULL) {
      ledColor(4);
  }else{
    return date;
  } 
}

// * La fonction suivante a pour but de vérifier l'espace libre sur la carte sd
static uint32_t limitSDVolume = 100;
void verifyFreeSpace()
{
  uint32_t freeKB = sd.vol()->freeClusterCount();
  freeKB *= sd.vol()->sectorsPerCluster() / 2;
  limitSDVolume = freeKB/configure.FILE_MAX_SIZE;
  Serial.println(limitSDVolume);
}

// * la fonction suivante défini la couleur de la led
// leds.setColorRGB(0, red, green, blue);
void ledColor(byte i)
{
  switch (i)
  {
  case 0: // * orange - mode configuration
    leds.setColorRGB(0, 200, 30, 0);
    break;
  case 1: // * jaune - mode maintenance
    leds.setColorRGB(0, 150, 150, 0);
    break;
  case 2: // * vert - mode standard
    leds.setColorRGB(0, 0, 150, 0);
    break;
  case 3: // * bleu - mode éco
    leds.setColorRGB(0, 0, 0, 150);
    break;
  case 4: // * alternance rouge bleu 1Hz - erreur horloge RTC
    leds.setColorRGB(0, 0, 0, 150);
    delay(500);
    leds.setColorRGB(0, 150, 0, 0);
    delay(500);
    break;
  case 5: // * alternance rouge jaune 1Hz - erreur données GPS
    leds.setColorRGB(0, 150, 0, 0);
    delay(500);
    leds.setColorRGB(0, 150, 150, 0);
    delay(500);
    break;
  case 6: // * alternance rouge vert 1 Hz - erreur accès aux données
    leds.setColorRGB(0, 150, 0, 0);
    delay(500);
    leds.setColorRGB(0, 0, 150, 0);
    delay(500);
    break;
  case 7: // * alternance rouge vert (vert deux fois plus long) 1 Hz - erreur données incohérentes
    leds.setColorRGB(0, 150, 0, 0);
    delay(333);
    leds.setColorRGB(0, 0, 150, 0);
    delay(667);
    break;
  case 8: // * alternance rouge blanc 1 Hz - carte SD pleine 
    leds.setColorRGB(0, 150, 0, 0);
    delay(500);
    leds.setColorRGB(0, 150, 150, 150);
    delay(500);
  case 9: // * alternance rouge blanc (blanc deux fois plus long) 1 Hz - erreur d'accès ou d'écriture de la carte SD
    leds.setColorRGB(0, 150, 0, 0);
    delay(333);
    leds.setColorRGB(0, 150, 150, 150);
    delay(666);
  case 10: // * violet - démarrage
    leds.setColorRGB(0, 150, 0, 150);
    delay(1000);
  default:
    break;
  }
}

// * fonction interrupt ne retourne rien et ne reçois rien, il faut donc adapter le code pour reconnaître les boutons
static bool BPR = false;
static bool BPV = false;

void myInterruptFunction()
{
  state_BPV = digitalRead(buttonV);
  state_BPR = digitalRead(buttonR);
  if (state_BPV == LOW)
  {
    BPV = true;
  }
  else if (state_BPR == LOW)
  {
    BPR = true;
  }
  else
  {
    BPR = false;
    BPV = false;
  }
}

// * Déclaration en dehors des conditions pour le système de chronomètre entre l'appuie et le relachement du bouton
static uint8_t elapsedTime = 0;
static uint8_t begin = 0; 
uint8_t end = 0;          
static bool Pushed = true;
static bool BPRStatic;
static bool BPVStatic;

void beginTime()
{
  DateTime now = rtc.now();
  begin = now.second();
  Pushed = false;
  BPVStatic = BPV;
  BPRStatic = BPR;
}

void endTime(bool i)
{
  DateTime now = rtc.now();
  end = now.second();
  Pushed = true;
  elapsedTime = end - begin;
  if (i == 0)
  {
    BPRStatic = 0;
  }
  else if (i == 1)
  {
    BPVStatic = 0;
  }
}

void chronometer()
{
  if ((BPV || BPR) && Pushed){
    beginTime();
  } else if (!BPV && !Pushed && BPVStatic) {
    endTime(0);
  } else if (!BPR && !Pushed && BPRStatic) {
    endTime(1);
  }
}

bool oddOrEven(uint8_t value)
{
  if ((value % 2) != 0)
  {
    return false; // * odd (entrée impaire)
  }
  else
  {
    return true; // * even (entrée paire)
  }
}

/*
 * La fonction suivante décrit le comportement des modes de fonctionnement accessibles depuis le mode standard, soit standard, eco et maintenance
 */

uint8_t compteur = 0; // * compteur de la fonction modeEco permettant de faire varier celle-ci un coup sur deux
void multiMode(byte i)
{
  if (i == 1)
  {
    ledColor(3); // * mode éco
    timeMultiplier = 2;
  }
  else if (i == 2)
  {
    ledColor(2); // * mode standard
    timeMultiplier = 1;
  }
  else if (i == 3)
  {
    ledColor(0); // * mode maintenance
    timeMultiplier = 2;
  }

  compteur++;
  capteurs capteurs;
  capteurs.timeStamp = horodatage();   // * lecture de l'heure
  capteurs.humidity = BME280Data(1);    // * lecture de la humidité
  capteurs.temperature = BME280Data(2); //  * lecture de la temperature
  capteurs.pressure = BME280Data(3);    // * lecture de la pression atmosphérique
  capteurs.lightSensor = lightSensor(); // * lecture du capteur de luminosité
  capteurs.GPSPosition = GPSData(); // * lecture du GPS (GPGGA)

  if (i == 1 || i == 2){
    while (!sd.card()){
      Serial.println(F("Impossible d'ouvrir la carte SD."));
      ledColor(9);
    } 
    // * Création du nom de fichier
    char fileRename[20];
    char fileName[20];
    // * la ligne suivante permet de concaténer la date et la révision du fichier ainsi que l'extension .LOG
    snprintf(fileRename, sizeof(fileRename), "%s_%d.LOG", nameDay(), revision);
    snprintf(fileName, sizeof(fileName),"%s_0.LOG", nameDay());
    // * Ouvre le fichier en mode écriture
    File myFile = sd.open(fileName, FILE_WRITE);
    // * enregistre l'ensemble des données sur la carte SD
    /*if (true){*/
      if (limitSDVolume >= revision){
        if (myFile.fileSize() < (configure.FILE_MAX_SIZE - 40)){
          myFile.print(capteurs.timeStamp);
          myFile.print(" ");
          myFile.print(capteurs.humidity);
          myFile.print(" ");
          myFile.print(capteurs.temperature);
          myFile.print(" ");
          myFile.print(capteurs.pressure);
          myFile.print(" ");
          myFile.print(capteurs.lightSensor);
          myFile.print(" ");
          
          if (i == 1 && oddOrEven(compteur)){ // * compteur pair -> prise en compte de tous les capteurs
            myFile.print(capteurs.GPSPosition);
          } else if (i == 2){
            myFile.print(capteurs.GPSPosition);
            Serial.println(capteurs.GPSPosition);
          }
          myFile.print("\n");
          myFile.close();
          Serial.println("Ligne ecrite");
        }else{
          revision++; // * Augmente le numéro de révision
          //myFile.rename(fileRename);
          myFile.close();
        }/*
      } else if (myFile && limitSDVolume < revision) { // * la carte sd n'est pas accessible
        ledColor(8);
      } else {
        Serial.println("not myFile");
        ledColor(9);
      }*/
    }
  } else if (i == 3) {
      Serial.println(F("Vous pouvez ejecter la carte SD"));
      Serial.print(capteurs.timeStamp);
      Serial.print(" ");
      Serial.print(capteurs.humidity);
      Serial.print(" ");
      Serial.print(capteurs.temperature);
      Serial.print(" ");
      Serial.print(capteurs.pressure);
      Serial.print(" ");
      Serial.print(capteurs.lightSensor);
      Serial.print(" ");
      Serial.print(capteurs.GPSPosition);
      Serial.print("\n");
    }
   delay(configure.LOG_INTERVAL * timeMultiplier);
}


static uint16_t compteur_AFK = 0;
// * mode configuration
void modeConfiguration()
{
  ledColor(1);
  registeredConfig();
  // * Début du mode configuration
  Serial.println(F("RESET, CONFIG or VERSION ?"));
  while (!Serial.available())
  {
    compteur_AFK++;
    if (compteur_AFK > 10){
      break;
    }
    delay(1000);
  } // * Attendre que des données soient disponibles sur le port série

  // * Lecture de l'entrée utilisateur
  memset(action, 0, sizeof(action)); // * Réinitialise le tableau action
  Serial.readBytesUntil('\n', action, sizeof(action) - 1);
  action[strcspn(action, "\n")] = 0; // * Supprimez le caractère de nouvelle ligne
  Serial.print(F("Vous avez saisi : "));
  Serial.println(action);
  Serial.println(F(" "));

  // * Réinitialisation des paramètres à leurs valeurs par défaut
  if (strcmp(action, "RESET") == 0)
  {
    configure = default_parameters;
    lot += 1;
    EEPROM.put(eepromAddress, configure);
    compteur_AFK = 0;
  }
  // * Saisie manuelle des paramètres
  else if (strcmp(action, "CONFIG") == 0)
  {
    ChangeConfig();
    lot += 1;
    EEPROM.put(eepromAddress, configure);
    compteur_AFK = 0;
  }
  // * Afficher la version du programme
  else if (strcmp(action, "VERSION") == 0)
  {
    Serial.print(F("Version du programme: "));
    Serial.println(version);
    Serial.print(F("Numéro de lot (tracabilité de la production): "));
    Serial.println(lot);
    compteur_AFK = 0;
  }
  // * Commande inconnue
  else
  {
    Serial.println(F("Commande non reconnue"));
    compteur_AFK = 0;
  }
}

/*
* Pour la fonction suivante les capteurs GPS est limité à une itération sur deux et le temps entre deux logs est multiplié par deux
*/
static uint8_t Setup = 0;
static bool MaintenanceBreak = false;
static bool ecoBreak = false;
static bool done = false;
void loop()
{
  // * calcul d'écart de temps entre le moment auquel l'utilisateur appuie sur le bouton et celui auquel il le relache
  chronometer();
  test();
  if (elapsedTime >= 1  && BPRStatic && !BPVStatic && Setup < 10)
  {
    elapsedTime = 0;
    while (true)
    {
      modeConfiguration();
      Serial.println(compteur_AFK);
      Serial.println("loop");
      timeMultiplier = 1;
      BPRStatic = false;
      delay(configure.LOG_INTERVAL * timeMultiplier);
      if (compteur_AFK >= 10)
      { // * si le bouton rouge est maintenu 5 secondes, la boucle break et le système repart en mode standard
        elapsedTime = 0;
        break;
      }
    }

    // * mode maintenance
  }
  else if (elapsedTime > 4 && BPRStatic && !MaintenanceBreak && Setup > 10)
  {
    elapsedTime = 0;
    BPRStatic = 0;
    ecoBreak = 0;
    while (true)
    { // * boucle sur le mode maintenance
      multiMode(3);
      chronometer();
      Serial.println(elapsedTime);
      if (elapsedTime > 4 && BPRStatic)
      { // * si le bouton rouge est maintenu 5 secondes, la boucle break et le système repart en mode standard
        MaintenanceBreak = true;
        elapsedTime = 0;
        break;
      }
    }

    // * mode éco
  }
  else if (elapsedTime > 4 && BPVStatic && !ecoBreak && Setup > 10)
  {
    elapsedTime = 0;
    BPVStatic = 0;
    MaintenanceBreak = false;
    while (true)
    { // * boucle sur le mode eco
      multiMode(1);
      chronometer();
      Serial.println(elapsedTime);
      if (elapsedTime > 4 && BPRStatic)
      { // * si le bouton rouge est maintenu 5 secondes, la boucle break et le système repart en mode standard
        ecoBreak = true;
        elapsedTime = 0;
        break;
      }
    }

    // * mode standard
  } else if (Setup > 10) {
    multiMode(2);
  } /*else if (10 < Setup && Setup < 20){
      if (!done){
        verifyFreeSpace();
        done = true;
      }
  } */else {
    Setup++;
    ledColor(10);
  }
}