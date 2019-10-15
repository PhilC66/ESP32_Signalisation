/* Ph CORBEL 10/2019
  Gestion Feux de Signalisation
  (basé sur ESP32_Tunnel)

  2 feux Violet et Blanc
  Etat des feux
              | Violet | Blanc | Feux | Cde
  OFF         |    0   |   0   |  0   |  D
  Violet Fixe |    1   |   0   |  1   |  F
  Blanc Fixe  |    0   |   1   |  2   |  O
  Blanc Cli 1 |    0   |  Cli1 |  3   |  M
  Blanc Cli 2 |    0   |  Cli2 |  4   |  S

  cadence clignotant parametrable
  Cli1 0.5s ON, 0.5s OFF
  Cli1 1s OFF, 0.15s ON/OFF pendant 1s

  Alimentation sur panneaux solaires

  mode deep sleep
  reveille tout les matin 06h55
  reception des SMS en attente
  apres 5 min de fonctionnement (ex: 2mn pour reception/suppression 8 SMS, 4mn 14SMS)
  envoie sms signal vie
  analyse calendrier sauvegardé en SPIFFS

	si jour circulé
  on continue normalement
  en fin de journée retour sleep jusqu'a 06h59

  si non circulé,
  retour SIM800 et ESP32 en sleep reveil toute les heures
  au reveil attendre au moins 30s pour que les SMS arrivent,
  quand plus de SMS et traitement retour sleep 1H00

  mode normal
  SIM800 en reception
  entree coffret, BP local demand Blanc Cli1 sur interruption ESP32

  Surveillance Batterie solaire
	Adc interne instable 2 à 3.5% erreurs!
	mise en place moyenne mobile sur les adc precision <1% avec 4bits

	Circulation = CalendrierCircule ^ flagCircule (OU exclusif)
	CalCircule	|	flagCircule | Circulation
				1			|			0				|			1
				0			|			1				|			1
				0			|			0				|			0
				1			|			1				|			0

	Librairie TimeAlarms.h modifiée
	#define dtNBR_ALARMS 10 (ne fonctionne pas avec 9)   6 à l'origine nombre d'alarmes RAM*11 max is 255

  --- ATTENTION ---
	l'utilisation de l'adc sur GPIO26 pour la mesure du 24V
	est perturbé apres l'utilisation du WIFI
	entrainant une erreur de mesure,
	pouvant ne pas detecter une tension 24V trop basse
	RST reset soft sans effet
	Solution
	reset hard liaison RS<->GPIO13
	apres arret WIFI GPIO13 to LOW
	apres redemarrage adc OK

	apres OTA relancer un RST


	to do

  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (1.0.4 bugg?)
  Arduino IDE 1.8.10 : 980542 74%, 47488 14% sur PC
  Arduino IDE 1.8.10 : 980518 75%, 47488 14% sur raspi

*/

#include <Battpct.h>
#include <Sim800l.h>              //my SIM800 modifié
#include <Time.h>
#include <TimeAlarms.h>
#include <sys/time.h>             //<sys/time.h>
#include <WiFi.h>
#include <EEPROM.h>               // variable en EEPROM
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <Ticker.h>
#include "passdata.h"

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

// #define RESET_PIN     18   // declaré par Sim800l.h
// #define LED_PIN        5   // declaré par Sim800l.h
#define PinBattProc   35   // liaison interne carte Lolin32 adc
#define PinBattSol    39   // Batterie générale 12V adc VN
#define PinBattUSB    36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi
#define PinBp         32   // Entrée Bp Wake up EXT1
#define Pin24V        33   // Mesure Tension 24V
#define PinFBlc       21   // Sortie Commande Feux Blanc
#define PinConvert    19   // Sortie Commande Convertisseur 12/24V
#define PinFVlt       15   // Sortie Commande Feux Violet
#define RX_PIN        16   // TX Sim800
#define TX_PIN        17   // RX Sim800
#define PinReset      13   // Reset Hard
#define SIMPIN        1234 // Code PIN carte SIM

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[4][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB
unsigned int adc_mm[4];            // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
unsigned long timer100  = 0; // pour timer 100ms adc
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le log

const String soft = "ESP32_Signalisation.ino.d32"; // nom du soft
String ver        = "V0-0.2";
int    Magique    = 0004;
const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;
String bufferrcpt;
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans EEPROM
char   SIM800InBuffer[64];          //  for notifications from the SIM800
char   replybuffer[255];            //  Buffer de reponse SIM800
volatile int IRQ_Cpt_Bp  = 0;
volatile unsigned long rebond1 = 0; //	antirebond IRQ
byte DbounceTime = 20;              // antirebond
byte confign = 0;                   // position enregistrement config EEPROM
byte recordn = 100;                 // position enregistrement log EEPROM
bool Allume  = false;
byte BlcPwmChanel = 0;
byte VltPwmChanel = 1;

RTC_DATA_ATTR int  Feux = 0; // Etat des Feux voir tableau au début
RTC_DATA_ATTR bool FlagAlarmeTension       = false; // Alarme tension Batterie
RTC_DATA_ATTR bool FlagLastAlarmeTension   = false;
RTC_DATA_ATTR bool FlagMasterOff           = false; // Coupure Allumage en cas de pb
RTC_DATA_ATTR bool FirstWakeup             = true;  // envoie premier message vie une seule fois
RTC_DATA_ATTR bool flagCircule             = false; // circule demandé -> inverse le calendrier, valable 1 seul jour
RTC_DATA_ATTR bool FileLogOnce             = false; // true si log > seuil alerte

bool FlagAlarme24V           = false; // Alarme tension 24V Allumage
bool FlagLastAlarme24V       = false;
bool FlagBp                  = false; // Bp local enfoncé
bool FlagReset = false;       // Reset demandé
bool jour      = false;				// jour = true, nuit = false

int CoeffTension[4];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

// bool LastWupAlarme = false;   // memo etat Alarme par Wakeup

int    slot = 0;              //this will be the slot number of the SMS

long   TensionBatterie  = 0; // Tension Batterie solaire
long   VBatterieProc    = 0; // Tension Batterie Processeur
long   VUSB             = 0; // Tension USB
long   Tension24        = 0; // Tension 24V Allumage

WebServer server(80);
File UploadFile;

typedef struct										// declaration structure  pour les log
{
  char 		dt[10];									//	DateTime 0610-1702 9+1
  char 		Act[2];									//	Action A/D/S/s 1+1
  char 		Name[15];								//	14 car
} champ;
champ record[5];

struct  config_t           // Structure configuration sauvée en EEPROM
{
  int     magic;           // num magique
  int     anticip;         // temps anticipation du reveille au lancement s
  long    DebutJour;       // Heure message Vie, 7h matin en seconde = 7*60*60
  long    FinJour;         // Heure fin jour, 20h matin en seconde = 20*60*60
  long    RepeatWakeUp;    // Periodicité WakeUp Jour non circulé
  int     timeoutWifi;     // tempo coupure Wifi si pas de mise a jour (s)
  bool    Bp;              // Bp Actif
  int     SlowBlinker;     // ms
  int     FastBlinker;
  int     FastRater;
  int     FVltPWM;         // Modulation Feux Violet %
  int     FBlcPWM;         // Modulation Feux Blanc %
  bool    Pos_Pn_PB[10];   // numero du Phone Book (1-9) à qui envoyer 0/1 0 par defaut
  char 		Idchar[11];			// Id
} ;
config_t config;

Ticker SlowBlink; // Clignotant lent
Ticker FastBlink; // Clignotant rapide
Ticker FastRate;  // Repetition Clignotant rapide
Ticker ADC;       // Lecture des Adc

AlarmId loopPrincipale;	// boucle principale
AlarmId FinJour;				// Fin de journée retour deep sleep

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial *SIM800Serial = &Serial2;// liaison serie FONA SIM800
Sim800l Sim800;  											// to declare the library

//---------------------------------------------------------------------------
void IRAM_ATTR handleInterruptBp() { // Bouton Poussoir

  portENTER_CRITICAL_ISR(&mux);
  if (xTaskGetTickCount() - rebond1 > DbounceTime) {
    IRQ_Cpt_Bp++;
    rebond1 = xTaskGetTickCount(); // equiv millis()
  }
  portEXIT_CRITICAL_ISR(&mux);

}
//---------------------------------------------------------------------------

void setup() {
  message.reserve(140);

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("lancement SIM800"));
  SIM800Serial->begin(9600); // 4800
  Sim800.begin();

  pinMode(PinBp      , INPUT_PULLUP);
  pinMode(PinFBlc    , OUTPUT);
  pinMode(PinFVlt    , OUTPUT);
  pinMode(PinConvert , OUTPUT);
  digitalWrite(PinConvert , LOW); // Alimentation Convert 0
  adcAttachPin(PinBattProc);
  adcAttachPin(PinBattSol);
  adcAttachPin(PinBattUSB);
  adcAttachPin(Pin24V);

  // parametrage PWM pour les feux
  // https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  ledcSetup(BlcPwmChanel, 1000, 8);
  ledcAttachPin(PinFBlc, BlcPwmChanel);
  ledcSetup(VltPwmChanel, 1000, 8);
  ledcAttachPin(PinFVlt, VltPwmChanel);

  ledcWrite(VltPwmChanel, 0); // Feux Violet 0
  ledcWrite(BlcPwmChanel, 0); // Feux Blanc 0

  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read);
  /* Lecture configuration en EEPROM	 */
  EEPROM.begin(512);

  EEPROM.get(confign, config); // lecture config
  recordn = sizeof(config);
  Serial.print("len config ="), Serial.println(sizeof(config));
  EEPROM.get(recordn, record); // Lecture des log
  Alarm.delay(500);
  if (config.magic != Magique) {
    /* verification numero magique si different
    		erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut
    */
    Serial.println(F("Nouvelle Configuration !"));
    config.magic         = Magique;
    config.anticip       = 90;
    config.DebutJour     = 9 * 60 * 60;
    config.FinJour       = 19 * 60 * 60;
    config.RepeatWakeUp  = 60 * 60;
    config.timeoutWifi   = 10 * 60;
    config.Bp            = true;
    config.SlowBlinker   = 500;
    config.FastBlinker   = 150;
    config.FastRater     = 1000;
    config.FBlcPWM       = 75;
    config.FVltPWM       = 75;
    for (int i = 0; i < 10; i++) {// initialise liste PhoneBook liste restreinte
      config.Pos_Pn_PB[i] = 0;
    }
    // config.Pos_Pn_PB[1]  = 1;	// le premier numero du PB par defaut
    String temp          = "TPCF_CV65";
    temp.toCharArray(config.Idchar, 11);
    EEPROM.put(confign, config);
    EEPROM.commit();
    // valeur par defaut des record (log)
    for (int i = 0; i < 5 ; i++) {
      temp = "";
      temp.toCharArray(record[i].dt, 10);
      temp.toCharArray(record[i].Act, 2);
      temp.toCharArray(record[i].Name, 15);
    }
    EEPROM.put(recordn, record);// ecriture des valeurs par defaut
    EEPROM.commit();
  }
  EEPROM.end();
  PrintEEPROM();
  Id  = String(config.Idchar);
  Id += fl;

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname("ESP32_Signalisation");
  ArduinoOTA.setPasswordHash(OTApwdhash);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.print(F("Start updating "));
    Serial.println(type);
  })
  .onEnd([]() {
    Serial.println(F("End"));
    delay(100);
    ResetHard();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if 			(error == OTA_AUTH_ERROR) 		Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR) 		Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR) 	Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR) 	Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR) 			Serial.println(F("End Failed"));
  });

  if (!SPIFFS.begin(true)) {
    Serial.println(F("SPIFFS initialisation failed..."));
    SPIFFS_present = false;
  }
  else {
    Serial.println(F("SPIFFS initialised... file access enabled..."));
    SPIFFS_present = true;
  }

  OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
  // Serial.print(F("temps =")),Serial.println(millis());
  Sim800.reset(SIMPIN);					// lancer SIM800
  MajHeure("");

  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee); // Fin de journée retour deep sleep
  Alarm.enable(FinJour);

  ActiveInterrupt();

  Serial.print(F("flag Circule :")), Serial.println(flagCircule);

}
//---------------------------------------------------------------------------
void loop() {
  recvOneChar();

  if (rebond1 > millis()) rebond1 = millis();

  char* bufPtr = SIM800InBuffer;	//buffer pointer
  if (Serial2.available()) {      	//any data available from the FONA?
    int charCount = 0;
    /* Read the notification into SIM800InBuffer */
    do  {
      *bufPtr = Serial2.read();
      bufferrcpt += *bufPtr;
      Serial.write(*bufPtr);
      delay(1);// Alarm.delay(1);
    } while ((*bufPtr++ != '\n') && (Serial2.available()) && (++charCount < (sizeof(SIM800InBuffer) - 1)));
    /* Add a terminal NULL to the notification string */
    *bufPtr = 0;
    if (charCount > 1) {
      // Serial.print(F("Buffer ="));
      Serial.println(bufferrcpt);
    }
    if ((bufferrcpt.indexOf(F("RING"))) > -1) {	// RING, Ca sonne
      Sim800.hangoffCall();									// on raccroche
    }
    /* Scan the notification string for an SMS received notification.
      If it's an SMS message, we'll get the slot number in 'slot' */
    if (1 == sscanf(SIM800InBuffer, "+CMTI: \"SM\",%d", &slot)) {
      traite_sms(slot);
    }
  }

  if (IRQ_Cpt_Bp > 0 && config.Bp) {
    Serial.print(F("Interruption : "));
    Serial.println(IRQ_Cpt_Bp);
    Feux = 3; // Violet 0, Blanc Manoeuvre Cli lent
    MajLog("Manuel", "Bp local");
    Allumage();
    if (!FlagBp) {
      FlagBp = true;
      envoieGroupeSMS(0, 0);			// envoie groupé
    }
    portENTER_CRITICAL(&mux);
    IRQ_Cpt_Bp = 0;
    portEXIT_CRITICAL(&mux);
  }

  ArduinoOTA.handle();
  Alarm.delay(0);
}	//fin loop
//---------------------------------------------------------------------------
void Acquisition() {
  static int8_t nsms;
  static int cpt = 0; // compte le nombre de passage boucle
  static bool firstdecision = false;
  static byte cptallume = 0; // compte le nombre de passage avec Allume

  if (cpt > 6 && nsms == 0 && !firstdecision) {
    /* une seule fois au demarrage attendre au moins 70s et plus de sms en attente */
    action_wakeup_reason(get_wakeup_reason());
    firstdecision = true;
  }
  cpt ++;

  if (CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0 || CoeffTension[3] == 0) {
    OuvrirFichierCalibration(); // patch relecture des coeff perdu
  }

  if (!Sim800.getetatSIM())Sim800.reset(SIMPIN); // verification SIM
  Serial.print(displayTime(0));
  // Serial.print(F(" Freemem = ")), Serial.println(ESP.getFreeHeap());
  static byte nalaTension = 0;
  static byte nRetourTension = 0;
  TensionBatterie = map(adc_mm[0] / nSample, 0, 4095, 0, CoeffTension[0]);
  VBatterieProc   = map(adc_mm[1] / nSample, 0, 4095, 0, CoeffTension[1]);
  VUSB            = map(adc_mm[2] / nSample, 0, 4095, 0, CoeffTension[2]);
  Tension24       = map(adc_mm[3] / nSample, 0, 4095, 0, CoeffTension[3]);

  // Serial.print(adc_mm[0]),Serial.print(";");
  // Serial.print(adc_mm[1]),Serial.print(";");
  // Serial.println(adc_mm[2]);

  if (Allume) {
    cptallume ++;
    Serial.print(F("Tension 24V :")), Serial.print(float(Tension24 / 100.0)), Serial.print(F(" "));
    Serial.print("coeff 24V="), Serial.println(CoeffTension[3]);
    if (cptallume > 2 && Tension24 < 2000) { // on attend 2 passages pour mesurer 24V
      FlagAlarme24V = true;
    }
  }
  else {
    cptallume = 0;
    FlagAlarme24V = false;
  }

  if (BattPBpct(TensionBatterie, 6) < 25 || VUSB < 4000) { // || VUSB > 6000
    nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
  else if (BattPBpct(TensionBatterie, 6) > 80 && VUSB > 4800) { //  && VUSB < 5400	//hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
    if (nRetourTension == 4) {
      FlagAlarmeTension = false;
      nRetourTension = 0;
      nalaTension = 0;
    }
  }
  else {
    if (nalaTension > 0)nalaTension--;		//	efface progressivement le compteur
  }

  message = F("Batt Solaire = ");
  message += float(TensionBatterie / 100.0);
  message += "V ";
  message += String(BattPBpct(TensionBatterie, 6));
  message += "%";
  message += F(", Batt Proc = ");
  message += (String(VBatterieProc) + "mV ");
  message += String(BattLipopct(VBatterieProc));
  message += (F("%, V USB = "));
  message += (float(VUSB / 1000.0));
  message += ("V");
  message += fl;
  Serial.print(message);

  /* verification nombre SMS en attente(raté en lecture directe)
  	 traitement des sms en memeoire un par un,
  	 pas de traitement en serie par commande 51, traitement beaucoup trop long */
  nsms = Sim800.getNumSms(); // nombre de SMS en attente (1s)
  Serial.print(F("Sms en attente = ")), Serial.println(nsms);

  if (nsms > 0) {	// nombre de SMS en attente
    // il faut les traiter
    int numsms = Sim800.getIndexSms(); // cherche l'index des sms en mémoire
    Serial.print(F("Numero Sms en attente = ")), Serial.println(numsms);
    traite_sms(numsms);// traitement des SMS en attente
  }
  else if (nsms == 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
    FlagReset = false;
    ResetHard();				//	reset hard
  }

  envoie_alarme();

  digitalWrite(LED_PIN, 0);
  Alarm.delay(20);
  digitalWrite(LED_PIN, 1);

  AIntru_HeureActuelle();

  Serial.println();
}
//---------------------------------------------------------------------------
void GestionFeux() {
  // Feux = mode;
  switch (Feux) {
    case 0: // Violet 0, Blanc 0
      Serial.println("Feux Eteint");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 1: // Violet 1, Blanc 0
      Serial.println("Feux Violet");
      ledcWrite(VltPwmChanel, 255 * config.FVltPWM / 100);
      ledcWrite(BlcPwmChanel, 0);
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 2: // Violet 0, Blanc 1
      Serial.println("Feux Blanc");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 255 * config.FBlcPWM / 100);
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 3: // Violet 0, Blanc Cli1
      Serial.println("Feux Blc Clignotant lent");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      FastBlink.detach();
      FastRate.detach();
      SlowBlink.attach_ms(config.SlowBlinker, blink);
      break;
    case 4: // Violet 0, Blanc Cli2
      Serial.println("Feux Blc Clignotant rapide");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      SlowBlink.detach();
      FastRate.attach_ms(config.FastRater, toggle);
      break;
      // default:
      // GestionFeux(0);
  }
}
//---------------------------------------------------------------------------
void toggle() {
  static bool isBlinking = false;
  if (isBlinking) {
    FastBlink.detach();
    isBlinking = false;
  }
  else {
    FastBlink.attach_ms(config.FastBlinker, blink);
    isBlinking = true;
  }
}
//---------------------------------------------------------------------------
void blink() {
  static bool blinker = false;
  if (blinker) {
    ledcWrite(BlcPwmChanel, 0);
    blinker = false;
  } else {
    ledcWrite(BlcPwmChanel, 255 * config.FBlcPWM / 100);
    blinker = true;
  }
}
//---------------------------------------------------------------------------
void Allumage() {
  Serial.println("Allumage");
  Allume = true;
  digitalWrite(PinConvert, HIGH); // Alimentation du convertisseur 12/24V
  GestionFeux();
}
//---------------------------------------------------------------------------
void Extinction() {
  Serial.println("Exctinction");
  Allume = false;
  digitalWrite(PinConvert, LOW); // Arret du convertisseur 12/24V
  Feux = 0;
  MajLog(F("Auto"), "Feux = " + String(Feux));
  envoieGroupeSMS(0, 0);			// envoie groupé obligatoire pour serveur
  // GestionFeux();
}
//---------------------------------------------------------------------------
void traite_sms(byte slot) {
  /* il y a 50 slots dispo
  	si slot=51, demande de balayer tous les slots pour verification (pas utilisé trop long)
  	si slot=99, demande depuis liaison serie en test, traiter sans envoyer de sms
  */

  char number[13];													// numero expediteur SMS
  String textesms;													// texte du SMS reçu
  textesms.reserve(140);
  String numero;
  String nom;

  byte i;
  byte j;
  bool sms = true;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
  int coef = 0; // coeff temporaire
  static byte P = 0; // Pin entrée a utiliser pour calibration
  static byte M = 0; // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  if (slot == 99) sms = false;
  if (slot == 51) { // demande de traitement des SMS en attente
    i = 1;
    j = 50;
  }
  else {
    i = slot;
    j = slot;
  }
  for (byte k = i; k <= j; k++) {
    slot = k;
    // /* Retrieve SMS sender address/phone number. */
    if (sms) {
      numero = Sim800.getNumberSms(slot); // recupere le Numero appelant
      nom = Sim800.getNameSms(slot);      // recupere le nom appelant
      textesms = Sim800.readSms(slot);    // recupere le contenu
      textesms = ExtraireSms(textesms);
      if (nom.length() < 1) { // si nom vide, cherche si numero est num de la SIM
        if (numero == Sim800.getNumTel()) {
          nom = F("Moi meme");
        }
      }
      Serial.print(F("Nom appelant = ")), Serial.println(nom);
      Serial.print(F("Numero = ")), Serial.println(numero);
    }
    else {
      textesms = String(replybuffer);
      nom = F("console");
    }

    if (!(textesms.indexOf(F("TEL")) == 0 || textesms.indexOf(F("tel")) == 0 || textesms.indexOf(F("Tel")) == 0
          || textesms.indexOf(F("Wifi")) == 0 || textesms.indexOf(F("WIFI")) == 0 || textesms.indexOf(F("wifi")) == 0)) {
      textesms.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI" parametres pouvant contenir minuscules
      // textesms.trim();
    }
    textesms.replace(" ", "");// supp tous les espaces
    // if (textesms.indexOf("A") == 0 || textesms.indexOf("D") == 0) {
    // Serial.println(Id.substring(5, 9));
    // temp.toUpperCase();
    // if (textesms.indexOf("A" + temp) == 0) textesms = F("INTRUON");
    // if (textesms.indexOf("D" + temp) == 0) textesms = F("INTRUOFF");
    // }
    Serial.print(F("textesms  = ")), Serial.println(textesms);

    if ((sms && nom.length() > 0) || !sms) {        // si nom appelant existant dans phone book
      numero.toCharArray(number, numero.length() + 1); // on recupere le numéro
      messageId();
      if (textesms.indexOf(F("TIMEOUTWIFI")) > -1) { // Parametre Arret Wifi
        if (textesms.indexOf(char(61)) == 11) {
          int n = textesms.substring(12, textesms.length()).toInt();
          if (n > 9 && n < 3601) {
            config.timeoutWifi = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += F("TimeOut Wifi (s) = ");
        message += config.timeoutWifi;
        message += fl;
      }
      if (textesms.indexOf(F("WIFIOFF")) > -1) { // Arret Wifi
        message += F("Wifi off");
        message += fl;
        EnvoyerSms(number, sms);
        WifiOff();
      }
      else if (textesms.indexOf(F("Wifi")) == 0) { // demande connexion Wifi
        byte pos1 = textesms.indexOf(char(44));//","
        byte pos2 = textesms.indexOf(char(44), pos1 + 1);
        String ssids = textesms.substring(pos1 + 1, pos2);
        String pwds  = textesms.substring(pos2 + 1, textesms.length());
        char ssid[20];
        char pwd[20];
        ssids.toCharArray(ssid, ssids.length() + 1);
        pwds.toCharArray(pwd, pwds.length() + 1);
        ConnexionWifi(ssid, pwd, number, sms); // message généré par routine
      }
      else if (textesms.indexOf(F("TEL")) == 0
               || textesms.indexOf(F("Tel")) == 0
               || textesms.indexOf(F("tel")) == 0) { // entrer nouveau num
        bool FlagOK = true;
        byte j = 0;
        String Send	= "AT+CPBW=";// message ecriture dans le phone book
        if (textesms.indexOf(char(61)) == 4) { // TELn= reserver correction/suppression
          int i = textesms.substring(3).toInt();// recupere n° de ligne
          i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
          //Serial.println(i);
          if (i < 1) FlagOK = false;
          Send += i;
          j = 5;
          // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
          if ( (i != 1) && (textesms.indexOf(F("efface")) == 5
                            || textesms.indexOf(F("EFFACE")) == 5 )) goto fin_tel;
        }
        else if (textesms.indexOf(char(61)) == 3) { // TEL= nouveau numero
          j = 4;
        }
        else {
          FlagOK = false;
        }
        if (textesms.indexOf("+") == j) {			// debut du num tel +
          if (textesms.indexOf(char(44)) == j + 12) {	// verif si longuer ok
            String numero = textesms.substring(j, j + 12);
            String nom = textesms.substring(j + 13, j + 27);// pas de verif si long<>0?
            Send += F(",\"");
            Send += numero;
            Send += F("\",145,\"");
            Send += nom;
            Send += F("\"");
          }
          else {
            FlagOK = false;
          }
        }
        else {
          FlagOK = false;
        }
fin_tel:
        if (!FlagOK) { // erreur de format
          //Serial.println(F("false"));
          messageId();
          message += F("Commande non reconnue ?");// non reconnu
          message += fl;
          EnvoyerSms(number, sms);					// SMS non reconnu
        }
        else {
          Serial.println(Send);
          Sim800.WritePhoneBook(Send);					//ecriture dans PhoneBook
          Alarm.delay(500);
          Sim800.ModeText(); //pour purger buffer fona
          Alarm.delay(500);
          messageId();
          message += F("Nouveau Num Tel: ");
          message += F("OK");
          message += fl;
          EnvoyerSms(number, sms);
        }
      }
      else if (textesms == F("LST?") || textesms == F("LST1")) {	//	Liste des Num Tel
        byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
        for (byte i = 1; i < n + 1; i++) {
          String num = Sim800.getPhoneBookNumber(i);
          // Serial.print(num.length()), Serial.print(" "), Serial.println(num);
          if (num.indexOf("+CPBR:") == 0) { // si existe pas sortir
            Serial.println(F("Failed!"));// next i
            goto fin_i;
          }
          String name = Sim800.getPhoneBookName(i);
          // Serial.println(name);
          message += String(i) + ":";
          message += num;
          message += "," + fl;
          message += name;
          message += fl;
          Serial.println(message);
          if ((i % 3) == 0) {
            EnvoyerSms(number, sms);
            messageId();
          }
        }
fin_i:
        if (message.length() > Id.length() + 20) EnvoyerSms(number, sms);; // SMS final
      }
      else if (textesms.indexOf(F("ETAT")) == 0 || textesms.indexOf(F("ST")) == 0) {// "ETAT? de l'installation"
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SYS")) > -1) {
        Sim800.getetatSIM();// 1s
        byte n = Sim800.getNetworkStatus();// 1.1s
        String Op = Sim800.getNetworkName();// 1.05s
        if (n == 5) {
          message += F(("rmg, "));// roaming 1.0s
        }
        message += Op + fl;
        read_RSSI();
        int Vbat = Sim800.BattVoltage();
        byte Batp = Sim800.BattPct();
        message += F("Batt GSM : ");
        message += Vbat;
        message += F(" mV, ");
        message += Batp;
        message += F(" %");
        message += fl;
        message += F("Ver: ");
        message += ver;
        message += fl;
        message += F("V Batt Sol= ");
        message += String(float(TensionBatterie / 100.0));
        message += F("V, ");
        message += String(BattPBpct(TensionBatterie, 6));
        message += " %";
        message += fl;
        message += F("V USB= ");
        message += (float(VUSB / 1000.0));
        message += "V";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
        String temp = textesms.substring(3);
        if (temp.length() > 0 && temp.length() < 11) {
          Id = "";
          temp.toCharArray(config.Idchar, 11);
          sauvConfig();														// sauvegarde en EEPROM
          Id = String(config.Idchar);
          Id += fl;
        }
        messageId();
        message += F("Nouvel Id");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("LOG")) == 0) {	// demande log des 5 derniers commandes
        File f = SPIFFS.open(filelog, "r");
        message = F("local log size :");
        message += String(f.size()) + fl;
        f.close();
        for (int i = 0; i < 5; i++) {
          message += String(record[i].dt) + "," + String(record[i].Act) + "," + String(record[i].Name) + fl;
        }
        //Serial.println( message);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("ANTICIP")) > -1) { // Anticipation du wakeup
        if (textesms.indexOf(char(61)) == 7) {
          int n = textesms.substring(8, textesms.length()).toInt();
          if (n > 9 && n < 601) {
            config.anticip = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += F("Anticipation WakeUp (s) = ");
        message += config.anticip;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("DEBUT")) == 0) {     //	Heure Message Vie/debutJour
        if (textesms.indexOf(char(61)) == 5) {
          long i = atol(textesms.substring(6).c_str()); //	Heure message Vie
          if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
            config.DebutJour = i;
            sauvConfig();                               // sauvegarde en EEPROM
            AIntru_HeureActuelle();
          }
        }
        message += F("Debut Journee = ");
        message += Hdectohhmm(config.DebutJour);
        message += F("(hh:mm)");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("TIME")) > -1) {
        message += F("Heure Sys = ");
        message += displayTime(0);
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure
        message += F("Mise a l'heure");
        // Sim800.reset(SIMPIN);// lancer SIM800
        MajHeure(Sim800.getTimeSms(slot)); // mise a l'heure du sms
        if (nom != F("Moi meme")) EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("IMEI")) > -1) {
        message += F("IMEI = ");
        String m = Sim800.getIMEI();
        message += m + fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("FIN")) == 0) {			//	Heure Fin de journée
        if ((textesms.indexOf(char(61))) == 3) {
          long i = atol(textesms.substring(4).c_str()); //	Heure
          if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
            config.FinJour = i;
            sauvConfig();															// sauvegarde en EEPROM
            FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee);// init tempo
          }
        }
        message += F("Fin Journee = ");
        message += Hdectohhmm(config.FinJour);
        message += F("(hh:mm)");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MOIS")) == 0) { // Calendrier pour un mois
        /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
          demande calendrier pour un mois donné ; format : MOIS=mm? */
        bool flag = true; // validation du format
        bool W = true; // true Write, false Read

        byte p1 = textesms.indexOf(char(61)); // =
        byte p2 = textesms.indexOf(char(44)); // ,
        if (p2 == 255) {                      // pas de ,
          p2 = textesms.indexOf(char(63));    // ?
          W = false;
        }

        byte m = textesms.substring(p1 + 1, p2).toInt(); // mois

        // printf("p1=%d,p2=%d\n",p1,p2);
        // Serial.println(textesms.substring(p1+1,p2).toInt());
        // Serial.println(textesms.substring(p2+1,textesms.length()).length());
        if (!(m > 0 && m < 13)) flag = false;
        if (W && flag) { // Write
          if (!(textesms.substring(p2 + 1, textesms.length()).length() == 31)) flag = false; // si longueur = 31(jours)

          for (int i = 1; i < 32; i++) { // verification 0/1
            if (!(textesms.substring(p2 + i, p2 + i + 1) == "0" || textesms.substring(p2 + i, p2 + i + 1) == "1")) {
              flag = false;
            }
          }
          if (flag) {
            // Serial.println(F("mise a jour calendrier"));
            for (int i = 1; i < 32; i++) {
              calendrier[m][i] = textesms.substring(p2 + i, p2 + i + 1).toInt();
              // Serial.print(textesms.substring(p2+i,p2+i+1));
            }
            EnregistreCalendrier(); // Sauvegarde en SPIFFS
            message += F("Mise a jour calendrier OK");
          }
        }
        else if (flag) { // ? demande calendrier pour un mois donné
          message += F("mois = ");
          message += m;
          message += fl;
          for (int i = 1; i < 32 ; i++) {
            message += calendrier[m][i];
            if ((i % 5)  == 0) message += " ";
            if ((i % 10) == 0) message += fl;
          }
        }
        if (!flag) {
          message += F("Format non reconnu !");
        }
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms == F("CIRCULE")) {
        bool ok = false;
        /* demande passer en mode Circulé pour le jour courant,
        	sans modification calendrier enregistré en SPIFFS */
        if (!(calendrier[month()][day()] ^ flagCircule)) {
          // calendrier[month()][day()] = 1;
          message += F("OK, Circule");
          flagCircule = !flagCircule;
          ok = true;
        }
        else {
          message += F("Jour deja Circule");
        }
        message += fl;
        EnvoyerSms(number, sms);
        if (ok) {
          if (sms)EffaceSMS(slot);
          action_wakeup_reason(4);
        }
      }
      else if (textesms == F("NONCIRCULE")) {
        bool ok = false;
        /* demande passer en mode nonCirculé pour le jour courant,
          sans modification calendrier enregistré en SPIFFS */
        if (calendrier[month()][day()] ^ flagCircule) {
          // calendrier[month()][day()] = 0;
          message += F("OK, NonCircule");
          flagCircule = !flagCircule;
          ok = true;
        }
        else {
          message += F("Jour deja NonCircule");
        }
        message += fl;
        EnvoyerSms(number, sms);
        if (ok) {
          if (sms)EffaceSMS(slot);
          action_wakeup_reason(4);
        }
      }
      else if (textesms.indexOf(F("TEMPOWAKEUP")) == 0) { // Tempo wake up
        if ((textesms.indexOf(char(61))) == 11) {
          int i = textesms.substring(12).toInt(); //	durée
          if (i > 59 && i <= 36000) { // 1mn à 10H
            config.RepeatWakeUp = i;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        message += F("Tempo repetition Wake up (s)=");
        message += config.RepeatWakeUp;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("LST2")) > -1) { //	Liste restreinte	//  =LST2=0,0,0,0,0,0,0,0,0
        bool flag = true; // validation du format
        if (textesms.indexOf(char(61)) == 4) { // "="
          byte Num[10];
          Sbidon = textesms.substring(5, 22);
          // Serial.print("bidon="),Serial.print(Sbidon),Serial.print("="),Serial.println(Sbidon.length());
          if (Sbidon.length() == 17) {
            int j = 1;
            for (int i = 0; i < 17; i += 2) {
              if (i == 16 && (Sbidon.substring(i, i + 1) == "0"	|| Sbidon.substring(i, i + 1) == "1")) {
                Num[j] = Sbidon.substring(i, i + 1).toInt();
              }
              else if ((Sbidon.substring(i + 1, i + 2) == ",") && (Sbidon.substring(i, i + 1) == "0"	|| Sbidon.substring(i, i + 1) == "1")) {
                //Serial.print(",="),Serial.println(bidon.substring(i+1,i+2));
                //Serial.print("X="),Serial.println(bidon.substring(i,i+1));
                Num[j] = Sbidon.substring(i, i + 1).toInt();
                //Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
                j++;
              }
              else {
                Serial.println(F("Format pas reconnu"));
                flag = false;
              }
            }
            if (flag) {
              //Serial.println("copie des num");
              for (int i = 1; i < 10; i++) {
                config.Pos_Pn_PB[i] = Num[i];
              }
              sauvConfig();															// sauvegarde en EEPROM
            }
          }
        }
        message += F("Liste restreinte");
        message += fl;
        for (int i = 1; i < 10; i++) {
          message += config.Pos_Pn_PB[i];
          if ( i < 9) message += char(44); // ,
        }
        EnvoyerSms(number, sms);
      }
      else if (textesms == F("RST")) {               // demande RESET
        message += F("Le systeme va etre relance");  // apres envoie du SMS!
        message += fl;
        FlagReset = true;                            // reset prochaine boucle
        EnvoyerSms(number, sms);
      }
      else if (!sms && textesms.indexOf(F("CALIBRATION=")) == 0) {
        /* 	Mode calibration mesure tension
        		Seulement en mode serie local
        		recoit message "CALIBRATION=.X"
        		entrer mode calibration
        		Selection de la tenssion à calibrer X
        		X = 1 TensionBatterie : PinBattSol : CoeffTension1
        		X = 2 VBatterieProc : PinBattProc : CoeffTension2
        		X = 3 VUSB : PinBattUSB : CoeffTension3
        		X = 4 Tension24 : Pin24V : CoeffTension4
        		effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
        		recoit message "CALIBRATION=1250" mesure réelle en V*100
        		calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
        		applique nouveau coeff
        		stock en EEPROM
        		sort du mode calibration

        		variables
        		FlagCalibration true cal en cours, false par defaut
        		Static P pin d'entrée
        		static int tensionmemo memorisation de la premiere tension mesurée en calibration
        		int CoeffTension = CoeffTensionDefaut 7000 par défaut
        */
        Sbidon = textesms.substring(12, 16); // texte apres =
        //Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(char(44)),Serial.println(Sbidon.length());
        long tension = 0;
        if (Sbidon.substring(0, 1) == "." && Sbidon.length() > 1) { // debut mode cal
          if (Sbidon.substring(1, 2) == "1" ) {
            M = 1;
            P = PinBattSol;
            coef = CoeffTension[0];
          }
          if (Sbidon.substring(1, 2) == "2" ) {
            M = 2;
            P = PinBattProc;
            coef = CoeffTension[1];
          }
          if (Sbidon.substring(1, 2) == "3" ) {
            M = 3;
            P = PinBattUSB;
            coef = CoeffTension[2];
          }
          if (Sbidon.substring(1, 2) == "4" ) {
            Allumage(); // Allumage
            Alarm.delay(500);
            read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V); // lecture des adc
            M = 4;
            P = Pin24V;
            coef = CoeffTension[3];
          }
          Serial.print("mode = "), Serial.print(M), Serial.println(Sbidon.substring(1, 2));
          FlagCalibration = true;

          coef = CoeffTensionDefaut;
          tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
          // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
          tensionmemo = tension;
        }
        else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 8000) {
          // si Calibration en cours et valeur entre 0 et 5000
          Serial.println(Sbidon.substring(0, 4));
          /* calcul nouveau coeff */
          coef = Sbidon.substring(0, 4).toFloat() / float(tensionmemo) * CoeffTensionDefaut;
          // Serial.print("Coeff Tension = "),Serial.println(coef);
          tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
          CoeffTension[M - 1] = coef;
          FlagCalibration = false;
          Recordcalib();														// sauvegarde en SPIFFS

          if (M == 4) {
            Extinction(); // eteindre
          }
        }
        else {
          message += F("message non reconnu");
          message += fl;
          FlagCalibration = false;
        }
        message += F("Mode Calib Tension ");
        message += String(M) + fl;
        message += F("TensionMesuree = ");
        message += tension;
        message += fl;
        message += F("Coeff Tension = ");
        message += coef;
        if (M == 1) {
          message += fl;
          message += F("Batterie = ");
          message += String(BattPBpct(tension, 6));
          message += "%";
        }
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(Id.substring(5, 9)) == 1) { // cherche CVXX
        if (textesms.indexOf("D") == 0) {
          Feux = 0;
          Extinction(); // Violet 0, Blanc 0
          MajLog(nom, "DCV");
        }
        else if (textesms.indexOf("A") == 0 || textesms.indexOf("F") == 0) {
          Feux = 1;
          Allumage(); // Violet 1, Blanc 0
          MajLog(nom, "FCV");
        }
        else if (textesms.indexOf("O") == 0) {
          Feux = 2;
          Allumage(); // Violet 0, Blanc 1
          MajLog(nom, "OCV");
        }
        else if (textesms.indexOf("M") == 0) {
          Feux = 3;
          Allumage(); // Violet 0, Blanc Manoeuvre Cli lent
          MajLog(nom, "MCV");
        }
        else if (textesms.indexOf("S") == 0) {
          Feux = 4;
          Allumage(); // Violet 0, Blanc Secteur Cli rapide
          MajLog(nom, "SCV");
        }
        else {
          // message += "non reconnu" + fl;
        }
        generationMessage(0);
        envoieGroupeSMS(0, 0);			// envoie groupé obligatoire pour serveur
        // EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("BLCPWM")) == 0) {
        if (textesms.substring(6, 7) == "=") {
          int i = textesms.substring(7, textesms.length()).toInt();
          if (i > 4 && i < 101) {
            config.FBlcPWM = i;
            sauvConfig();
          }
        }
        Allumage();
        message += "Blanc PWM =";
        message += config.FBlcPWM;
        message += "%";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("VLTPWM")) == 0) {
        if (textesms.substring(6, 7) == "=") {
          int i = textesms.substring(7, textesms.length()).toInt();
          if (i > 4 && i < 101) {
            config.FVltPWM = i;
            sauvConfig();
          }
        }
        Allumage();
        message += "Violet PWM =";
        message += config.FVltPWM;
        message += "%";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SLOWBLINKER")) == 0) {
        if (textesms.substring(11, 12) == "=") {
          int i = textesms.substring(12, textesms.length()).toInt();
          if (i > 4 && i < 2001) {
            config.SlowBlinker = i;
            sauvConfig();
          }
        }
        Allumage();
        message += "SlowBlinker =";
        message += config.SlowBlinker;
        message += "ms";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("FASTBLINKER")) == 0) {
        if (textesms.substring(11, 12) == "=") {
          int i = textesms.substring(12, textesms.length()).toInt();
          if (i > 4 && i < 2001) {
            config.FastBlinker = i;
            sauvConfig();
          }
        }
        Allumage();
        message += "FastBlinker =";
        message += config.FastBlinker;
        message += "ms";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("FASTRATER")) == 0) {
        if (textesms.substring(9, 10) == "=") {
          int i = textesms.substring(10, textesms.length()).toInt();
          if (i > 4 && i < 3001) {
            config.FastRater = i;
            sauvConfig();
          }
        }
        Allumage();
        message += "FastRater =";
        message += config.FastRater;
        message += "ms";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("BPACTIF")) == 0) {
        if (textesms.substring(7, 8) == "=") {
          if (textesms.substring(8, 9) == "1") {
            if (!config.Bp) {
              config.Bp = true;
              sauvConfig();
              ActiveInterrupt();
            }
          }
          else if (textesms.substring(8, 9) == "0") {
            if (config.Bp) {
              config.Bp = false;
              sauvConfig();
              DesActiveInterrupt();
            }
          }
        }
        message += "Bouton poussoir local ";
        if (config.Bp) {
          message += "Actif";
        }
        else {
          message += "InActif";
        }
        message += fl;
        EnvoyerSms(number, sms);
      }
      //**************************************
      else {
        message += F("message non reconnu !");
        message += fl;
        if (nom != F("Moi meme")) EnvoyerSms(number, sms);
      }
    }
    else {
      Serial.print(F("Appelant non reconnu ! "));
    }
    if (sms) { // suppression du SMS
      EffaceSMS(slot);
    }
  }
}
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS appartition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;

  if (FlagAlarme24V != FlagLastAlarme24V) {
    SendEtat = true;
    MajLog(F("Auto"), F("Alarme24V"));
    FlagLastAlarme24V = FlagAlarme24V;
  }
  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    MajLog(F("Auto"), F("AlarmeTension"));
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if (SendEtat) { 						// si envoie Etat demandé
    envoieGroupeSMS(0, 0);			// envoie groupé
    SendEtat = false;					// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeSMS(byte grp, bool m) {
  /* m=0 message normal/finanalyse
  	si grp = 0,
    envoie un SMS à tous les numero existant (9 max) du Phone Book
  	SAUF ceux de la liste restreinte
    si grp = 1,
    envoie un SMS à tous les numero existant (9 max) du Phone Book
    de la liste restreinte config.Pos_Pn_PB[x]=1			*/

  byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
  // Serial.print(F("Nombre de ligne PB=")),Serial.println(n);
  for (byte Index = 1; Index < n + 1; Index++) { // Balayage des Num Tel dans Phone Book
    if ((grp == 0 && config.Pos_Pn_PB[Index] == 0) || (grp == 1 && config.Pos_Pn_PB[Index] == 1)) {
      String number = Sim800.getPhoneBookNumber(Index);
      generationMessage(m);
      char num[13];
      number.toCharArray(num, 13);
      EnvoyerSms(num, true);
    }
  }
}
//---------------------------------------------------------------------------
void generationMessage(bool n) {
  // n = 0 message normal
  // n = 1 message fin analyse
  messageId();
  if (FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarme24V) {
    message += F("--KO--------KO--");
  }
  else {
    message += F("-------OK-------");
  }
  message += fl;
  if (FlagBp) {
    message += "Commande locale";
    message += fl;
    FlagBp = false;
  }
  // message += "Allumage = ";
  // message += Allume;
  // message += fl;
  switch (Feux) {
    case 0: // Violet 0, Blanc 0
      message += "D";
      break;
    case 1: // Violet 1, Blanc 0
      message += "F";
      break;
    case 2: // Violet 0, Blanc 1
      message += "O";
      break;
    case 3: // Violet 0, Blanc Manoeuvre Cli lent
      message += "M";
      break;
    case 4: // Violet 0, Blanc Secteur Cli rapide
      message += "S";
      break;
  }
  message += String(Id.substring(5, 9));
  message += fl;
  message += F("Batterie : ");
  if (!FlagAlarmeTension) {
    message += F("OK, ");
    message += String(BattPBpct(TensionBatterie, 6));
    message += "%" + fl;
  }
  else {
    message += F("Alarme, ");
    message += String(BattPBpct(TensionBatterie, 6));
    message += "%";
    message += fl;
    message += F("V USB =");
    message += String(float(VUSB / 1000.0)) + fl;
  }
  if (FlagAlarme24V) {
    message += F("Alarme 24V = ");
    message += String(float(Tension24 / 100.0)) + "V" + fl;
  }
  // if (Allume) {
  // for (int i = 0; i < 5 ; i++) {
  // read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V);
  // Alarm.delay(1);
  // }
  // char bid[8];
  // sprintf(bid, "%.2lf V", float(Tension24) / 100);
  // message += F("Allume : ");
  // message += String(bid);
  // message += fl;
  // }
  if ((calendrier[month()][day()] ^ flagCircule)) {
    message += F("Jour Circule");
  }
  else {
    message += F("Jour Non Circule");
  }
  message += fl;
}
//---------------------------------------------------------------------------
void EnvoyerSms(char *num, bool sms) {

  if (sms) { // envoie sms
    message.toCharArray(replybuffer, message.length() + 1);
    bool OK = Sim800.sendSms(num, replybuffer);
    if (OK)Serial.println(F("send sms OK"));
  }
  Serial.print (F("Message (long) = ")), Serial.println(message.length());
  Serial.println(message);
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  int r;
  byte n = Sim800.getRSSI();
  // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  message += F("RSSI=");
  message += String(n);
  message += ", ";
  message += String(r);
  message += F("dBm");
  message += fl;
}
//---------------------------------------------------------------------------
void MajHeure(String smsdate) {
  if (smsdate.length() > 1) { // si smsdate present mise a l'heure forcé
    Sim800.SetTime(smsdate);
  }
  static bool First = true;
  int ecart;
  Serial.print(F("Mise a l'heure reguliere !, "));
  // setTime(10,10,0,1,1,18);
  int Nday, Nmonth, Nyear, Nminute, Nsecond, Nhour;
  Sim800.RTCtime(&Nday, &Nmonth, &Nyear, &Nhour, &Nminute, &Nsecond);


  printf("%s %02d/%02d/%d %02d:%02d:%02d\n", "MajH1", Nday, Nmonth, Nyear, Nhour, Nminute, Nsecond);
  long debut = millis();
  if (First || Nyear < 17) {
    while (Nyear < 17) {
      Sim800.RTCtime(&Nday, &Nmonth, &Nyear, &Nhour, &Nminute, &Nsecond);
      printf("%s %02d/%02d/%d %02d:%02d:%02d\n", "MajH2", Nday, Nmonth, Nyear, Nhour, Nminute, Nsecond);
      Alarm.delay(1000);
      // if (millis() - debut > 10000) {// supprimé risque de deconnexion reseau plus de redemarage
      // Sim800.setPhoneFunctionality(0);
      // Alarm.delay(1000);
      // Sim800.setPhoneFunctionality(1);
      // Alarm.delay(1000);
      // }
      if (millis() - debut > 15000) {
        Serial.println(F("Impossible de mettre à l'heure !"));
        //on s'envoie à soi même un SMS "MAJHEURE"
        message = F("MAJHEURE");
        char numchar[13];
        String numstring = Sim800.getNumTel();
        numstring.toCharArray(numchar, 13);
        EnvoyerSms(numchar, true);
        break;
      }
    }
    setTime(Nhour, Nminute, Nsecond, Nday, Nmonth, Nyear);
    First = false;
  }
  else {
    //  calcul décalage entre H sys et H reseau en s
    ecart = (Nhour - hour()) * 3600;
    ecart += (Nminute - minute()) * 60;
    ecart += Nsecond - second();
    // ecart += 10;
    Serial.print(F("Ecart s= ")), Serial.println(ecart);
    if (abs(ecart) > 5) {
      // ArretSonnerie();	// Arret Sonnerie propre
      Alarm.disable(loopPrincipale);
      Alarm.disable(FinJour);

      setTime(Nhour, Nminute, Nsecond, Nday, Nmonth, Nyear);

      Alarm.enable(loopPrincipale);
      Alarm.enable(FinJour);
    }
  }
  displayTime(0);
  AIntru_HeureActuelle();
}
//---------------------------------------------------------------------------
long DureeSleep(long Htarget) { // Htarget Heure de reveil visée
  /* calcul durée entre maintenant et Htarget*/
  long SleepTime = 0;
  long Heureactuelle = HActuelledec();
  if (Heureactuelle < Htarget) {
    SleepTime = Htarget - Heureactuelle;
  }
  else {
    if (Heureactuelle < 86400) { // < 24h00
      SleepTime = (86400 - Heureactuelle) + Htarget;
    }
  }
  return SleepTime;
}
//---------------------------------------------------------------------------
long HActuelledec() {
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes
  return Heureactuelle;
}
//---------------------------------------------------------------------------
void SignalVie() {
  Serial.println(F("Signal vie"));
  MajHeure("");
  envoieGroupeSMS(0, 0);
  Sim800.delAllSms();// au cas ou, efface tous les SMS envoyé/reçu
}
//---------------------------------------------------------------------------
void sauvConfig() { // sauve configuration en EEPROM
  EEPROM.begin(512);
  EEPROM.put(confign, config);
  EEPROM.commit();
  EEPROM.end();
}
//---------------------------------------------------------------------------
String displayTime(byte n) {
  // n = 0 ; dd/mm/yyyy hh:mm:ss
  // n = 1 ; yyyy-mm-dd hh:mm:ss
  char bid[20];
  if (n == 0) {
    sprintf(bid, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  }
  else {
    sprintf(bid, "%4d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  }
  return String(bid);
}
//---------------------------------------------------------------------------
void logRecord(String nom, String action) { // renseigne log et enregistre EEPROM
  static int index = 0;
  String temp;
  if (month() < 10) {
    temp =  "0" + String(month());
  }
  else {
    temp = String(month());
  }
  if (day() < 10 ) {
    temp += "0" + String(day());
  }
  else {
    temp += String(day());
  }
  if (hour() < 10) {
    temp += "-0" + String(hour());
  }
  else {
    temp += "-" + String(hour());
  }
  if (minute() < 10) {
    temp += "0" + String(minute());
  }
  else {
    temp += String(minute());
  }
  temp  .toCharArray(record[index].dt, 10);
  nom   .toCharArray(record[index].Name, 15);
  action.toCharArray(record[index].Act, 2);

  EEPROM.begin(512);
  EEPROM.put(recordn, record);// ecriture des valeurs par defaut
  EEPROM.commit();
  EEPROM.end();
  if (index < 4) {
    index ++;
  }
  else {
    index = 0;
  }
}
//---------------------------------------------------------------------------
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println(F("- failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F(" - not a directory"));
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print(F("  DIR : "));
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(F("  FILE: "));
      Serial.print(file.name());
      Serial.print(F("\tSIZE: "));
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  file.close();
}
//---------------------------------------------------------------------------
void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println(F("- failed to open file for reading"));
    return;
  }
  String buf = "";
  int i = 0;
  // Serial.println("- read from file:");
  while (file.available()) {
    int inchar = file.read();
    if (isDigit(inchar)) {
      buf += char(inchar);
      i ++;
    }
  }
  int m = 0;
  int j = 0;
  for (int i = 0; i < 372; i++) { // 12mois de 31 j =372
    j = 1 + (i % 31);
    if (j == 1) m ++;
    calendrier[m][j] = buf.substring(i, i + 1).toInt();
  }
}
//---------------------------------------------------------------------------
void appendFile(fs::FS &fs, const char * path, const char * message) {
  // Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Serial.println("- message appended");
  } else {
    // Serial.println("- append failed");
  }
}
//---------------------------------------------------------------------------
void MajLog(String Id, String Raison) { // mise à jour fichier log en SPIFFS
  /* verification de la taille du fichier */
  File f = SPIFFS.open(filelog, "r");
  Serial.print(F("Taille fichier log = ")), Serial.println(f.size());
  // Serial.print(Id),Serial.print(","),Serial.println(Raison);
  if (f.size() > 150000 && !FileLogOnce) {
    /* si trop grand on efface */
    FileLogOnce = true;
    messageId();
    message += F("Fichier log presque plein\n");
    message += String(f.size());
    message += F("\nFichier sera efface a 300000");
    String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
    char num[13];
    number.toCharArray(num, 13);
    EnvoyerSms(num, true);
  }
  else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
    messageId();
    message += F("Fichier log plein\n");
    message += String(f.size());
    message += F("\nFichier efface");
    String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
    char num[13];
    number.toCharArray(num, 13);
    EnvoyerSms(num, true);
    SPIFFS.remove(filelog);
    FileLogOnce = false;
  }
  f.close();
  /* preparation de la ligne */
  char Cbidon[101]; // 100 char maxi
  sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  Id = ";" + Id + ";";
  Raison += "\n";
  strcat(Cbidon, Id.c_str());
  strcat(Cbidon, Raison.c_str());
  Serial.println(Cbidon);
  appendFile(SPIFFS, filelog, Cbidon);
}
//---------------------------------------------------------------------------
void EnregistreCalendrier() { // remplace le nouveau calendrier

  SPIFFS.remove(filecalendrier);
  Sbidon = "";
  char bid[63];
  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Sbidon += calendrier[m][j];
      if (j < 31)Sbidon += char(59); // ;
    }
    Serial.println(Sbidon);
    Sbidon += fl;
    Sbidon.toCharArray(bid, 63);
    appendFile(SPIFFS, filecalendrier, bid);
    Sbidon = "";
  }

}
//---------------------------------------------------------------------------
void OuvrirCalendrier() {

  // this opens the file "f.txt" in read-mode
  listDir(SPIFFS, "/", 0);
  bool f = SPIFFS.exists(filecalendrier);
  // Serial.println(f);
  File f0 = SPIFFS.open(filecalendrier, "r");

  if (!f || f0.size() == 0) {
    Serial.println(F("File doesn't exist yet. Creating it")); // creation calendrier defaut
    char bid[63];
    Sbidon = "";
    for (int m = 1; m < 13; m++) {
      for (int j = 1; j < 32; j++) {
        if (m == 1 || m == 2 || m == 3 || m == 11 || m == 12) {
          Sbidon += "0;";
        }
        else {
          Sbidon += "1;";
        }
      }
      Serial.println(Sbidon);
      Sbidon += fl;
      Sbidon.toCharArray(bid, 63);
      appendFile(SPIFFS, filecalendrier, bid);
      Sbidon = "";
    }
  }
  readFile(SPIFFS, filecalendrier);

  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Serial.print(calendrier[m][j]), Serial.print(char(44));
    }
    Serial.println();
  }
  listDir(SPIFFS, "/", 0);

}
//---------------------------------------------------------------------------
void FinJournee() {
  // fin de journée retour deep sleep
  jour = false;
  flagCircule = false;
  FirstWakeup = true;
  Serial.println(F("Fin de journee retour sleep"));
  TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);// 1.5mn avant
  Sbidon  = F("FinJour, sleep for ");
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog(F("Auto"), Sbidon);
  if (Allume)Extinction();
  DebutSleep();
}
//---------------------------------------------------------------------------
void PrintEEPROM() {
  Serial.print(F("Version = "))                 , Serial.println(ver);
  Serial.print(F("ID = "))                      , Serial.println(config.Idchar);
  Serial.print(F("magic = "))                   , Serial.println(config.magic);
  Serial.print(F("Debut Jour = "))              , Serial.println(config.DebutJour);
  Serial.print(F("Fin jour = "))                , Serial.println(config.FinJour);
  Serial.print(F("T anticipation Wakeup = "))   , Serial.println(config.anticip);
  Serial.print(F("Tempo repetition Wake up (s)= ")), Serial.println(config.RepeatWakeUp);
  Serial.print(F("Time Out Wifi (s)= "))        , Serial.println(config.timeoutWifi);
  Serial.print(F("Bouton poussoir Actif = "))   , Serial.println(config.Bp);
  Serial.print(F("Vitesse SlowBlinker = "))     , Serial.println(config.SlowBlinker);
  Serial.print(F("Vitesse FastBlinker = "))     , Serial.println(config.FastBlinker);
  Serial.print(F("Vitesse RepetFastBlinker = ")), Serial.println(config.FastRater);
  Serial.print(F("PWM Blanc = "))               , Serial.println(config.FBlcPWM);
  Serial.print(F("PWM Violet = "))              , Serial.println(config.FVltPWM);
  Serial.print(F("Liste Restreinte = "));
  for (int i = 1; i < 10; i++) {
    Serial.print(config.Pos_Pn_PB[i]);
    if (i == 9) {
      Serial.println();
    }
    else {
      Serial.print(F(","));
    }
  }
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid, char* pwd, char* number, bool sms) {

  messageId();
  Serial.print(F("connexion Wifi:")), Serial.print(ssid), Serial.print(char(44)), Serial.println(pwd);
  String ip;
  WiFi.begin(ssid, pwd);
  // WiFi.mode(WIFI_STA);
  byte timeout = 0;
  bool error = false;

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    timeout ++;
    if (timeout > 60) {
      error = true;
      break;
    }
  }
  if (!error) {
    Serial.println();
    Serial.println(F("WiFi connected"));
    Serial.print(F("IP address: "));
    ip = WiFi.localIP().toString();
    Serial.println(ip);
    ArduinoOTA.begin();

    server.on("/",         HomePage);
    server.on("/download", File_Download);
    server.on("/upload",   File_Upload);
    server.on("/fupload",  HTTP_POST, []() {
      server.send(200);
    }, handleFileUpload);
    server.on("/delete",   File_Delete);
    server.on("/dir",      SPIFFS_dir);
    server.on("/cal",      CalendarPage);
    server.on("/Tel_list", Tel_listPage);
    server.on("/timeremaining", handleTime); // renvoie temps restant sur demande
    server.on("/datetime", handleDateTime); // renvoie Date et Heure
    server.on("/wifioff",  WifiOff);
    ///////////////////////////// End of Request commands
    server.begin();
    Serial.println(F("HTTP server started"));

    message += F("Connexion Wifi : ");
    message += fl;
    message += String(ip);
    message += fl;
    message += String(WiFi.RSSI());
    message += F(" dBm");
    message += fl;
    message += F("TimeOut Wifi ");
    message += config.timeoutWifi;
    message += " s";
  }
  else {
    message += F("Connexion Wifi impossible");
  }
  EnvoyerSms(number, sms);

  if (sms) { // suppression du SMS
    /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
      ou OTA sms demande Wifi toujours present */
    EffaceSMS(slot);
  }
  debut = millis();
  if (!error) {
    /* boucle permettant de faire une mise à jour OTA et serveur, avec un timeout en cas de blocage */
    unsigned long timeout = millis();
    while (millis() - timeout < config.timeoutWifi * 1000) {
      // if(WiFi.status() != WL_CONNECTED) break; // wifi a été coupé on sort
      ArduinoOTA.handle();
      server.handleClient(); // Listen for client connections
      delay(1);
    }
    WifiOff();
  }
}
//---------------------------------------------------------------------------
void WifiOff() {
  Serial.println(F("Wifi off"));
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();
  Alarm.delay(100);
  ResetHard();
}
//---------------------------------------------------------------------------
void ResetHard() {
  // GPIO13 to RS reset hard
  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
}
//---------------------------------------------------------------------------
String ExtraireSms(String msgbrut) { //Extraction du contenu du SMS

  int pos[10];									// SMS jusqu'a 5 lignes
  int i = 0;
  for (i = 0; i < 10; i++) {
    if (i == 0) {
      pos[i] = msgbrut.indexOf("\n");
    }
    else {
      pos[i] = msgbrut.indexOf("\n", pos[i - 1] + 1);
    }
    // Serial.print(i),Serial.print(" pos = "),Serial.println(pos[i]);
    if (pos[i] == -1) {
      i --;
      break;
    }
  }

  String message = msgbrut.substring(pos[1] + 1, pos[i - 1] - 1);
  // Serial.print("message extrait = "),Serial.println(message);
  message.replace("\n", "|");				// remplacement des sauts de lignes par |
  message = message.substring(0, message.length() - 2);
  // Serial.print("message extrait sans \n= "),Serial.println(message);

  return message;
}
//---------------------------------------------------------------------------
int moyenneAnalogique(int Pin) {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    // Alarm.delay(1);
    moyenne += analogRead(Pin);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
void OuvrirFichierCalibration() { // Lecture fichier calibration

  if (SPIFFS.exists(filecalibration)) {
    File f = SPIFFS.open(filecalibration, "r");
    for (int i = 0; i < 4; i++) { //Read
      String s = f.readStringUntil('\n');
      CoeffTension[i] = s.toFloat();
    }
    f.close();
  }
  else {
    Serial.print(F("Creating Data File:")), Serial.println(filecalibration); // valeur par defaut
    CoeffTension[0] = CoeffTensionDefaut;
    CoeffTension[1] = CoeffTensionDefaut;
    CoeffTension[2] = CoeffTensionDefaut;
    CoeffTension[3] = CoeffTensionDefaut;
    Recordcalib();
  }
  Serial.print(F("Coeff T Batterie = ")), Serial.print(CoeffTension[0]);
  Serial.print(F(" Coeff T Proc = "))	  , Serial.print(CoeffTension[1]);
  Serial.print(F(" Coeff T VUSB = "))		, Serial.print(CoeffTension[2]);
  Serial.print(F(" Coeff T 24V = "))		, Serial.println(CoeffTension[3]);

}
//---------------------------------------------------------------------------
void Recordcalib() { // enregistrer fichier calibration en SPIFFS

  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.println(CoeffTension[2]);
  f.println(CoeffTension[3]);
  f.close();

}
//---------------------------------------------------------------------------
String Hdectohhmm(long Hdec) {
  String hhmm;
  if (int(Hdec / 3600) < 10) hhmm = "0";
  hhmm += int(Hdec / 3600);
  hhmm += ":";
  if (int((Hdec % 3600) / 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) / 60);
  hhmm += ":";
  if (int((Hdec % 3600) % 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) % 60);
  return hhmm;
}
//---------------------------------------------------------------------------
void DesActiveInterrupt() {
  // if (config.Bp) {
  detachInterrupt(digitalPinToInterrupt(PinBp));
  // }
}
//---------------------------------------------------------------------------
void ActiveInterrupt() {
  if (config.Bp) {
    attachInterrupt(digitalPinToInterrupt(PinBp), handleInterruptBp, FALLING);
  }
}
//---------------------------------------------------------------------------
void AIntru_HeureActuelle() {

  long Heureactuelle = HActuelledec();

  if (config.FinJour > config.DebutJour) {
    if ((Heureactuelle > config.FinJour && Heureactuelle > config.DebutJour)
        || (Heureactuelle < config.FinJour && Heureactuelle < config.DebutJour)) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
  else {
    if (Heureactuelle > config.FinJour && Heureactuelle < config.DebutJour) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
}
//---------------------------------------------------------------------------
void IntruF() { // Charge parametre Alarme Intrusion Jour
  // Nmax = config.Jour_Nmax;
  jour = true;
  // Serial.println(F("Jour"));
}
//---------------------------------------------------------------------------
void IntruD() { // Charge parametre Alarme Intrusion Nuit
  // Nmax = config.Nuit_Nmax;
  jour = false;
  // Serial.println(F("Nuit"));
}
//---------------------------------------------------------------------------
void DebutSleep() {
  // ArretSonnerie();
  
  // selection du pin mask en fonction des capteurs actif
  // const uint64_t ext_wakeup_pin_1_mask = 1ULL << PinBp;
  // const uint64_t ext_wakeup_pin_2_mask = 1ULL << PinPedale2;
  // const uint64_t ext_wakeup_pin_3_mask = 1ULL << PinCoffret;
  // if (config.Intru) {
  // if (config.Coffret && config.Pedale1 && config.Pedale2) {
  // esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask  | ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  // } else if (config.Coffret && config.Pedale1) {
  // esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask  | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  // } else if (config.Coffret && config.Pedale2) {
  // esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  // } else if (config.Pedale1 && config.Pedale2) {
  // esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask  | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  // } else if (config.Coffret) {
  // esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  // } else if (config.Pedale1) {
  // esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  // } else if (config.Pedale2) {
  // esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  // }
  // }
  // else {
  // /* pas de wakeup sur pin */
  // Serial.println(F("pas de pin selectionne pour wakeup!"));
  // }

  /* Garde fou si TIME_TO_SLEEP > 20H00 c'est une erreur, on impose 1H00 */
  if (TIME_TO_SLEEP > 72000) {
    TIME_TO_SLEEP = 3600;
    Sbidon = F("jour ");
    Sbidon += jour;
    Sbidon = F(", Calendrier ");
    Sbidon += calendrier[month()][day()];
    Sbidon = F(", flagCirc ");
    Sbidon += flagCircule;
    MajLog(F("Auto"), Sbidon);
    Sbidon = F("Attention erreur Sleep>20H00 ");
    Sbidon += Hdectohhmm(TIME_TO_SLEEP);
    MajLog(F("Auto"), Sbidon);
  }

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.print(F("Setup ESP32 to sleep for "));
  print_uint64_t(TIME_TO_SLEEP);
  Serial.print(F("s ;"));
  Serial.println(Hdectohhmm(TIME_TO_SLEEP));
  Serial.flush();
  //Go to sleep now
  Serial.println(F("Going to sleep now"));

  byte i = 0;
  while (!Sim800.sleep()) {
    Alarm.delay(100);
    if (i++ > 10) break;
  }
  Serial.flush();
  esp_deep_sleep_start();
  delay(100);

  Serial.println(F("This will never be printed"));
  Serial.flush();

}
//---------------------------------------------------------------------------
void action_wakeup_reason(byte wr) { // action en fonction du wake up
  Serial.print(F("Wakeup :")), Serial.print(wr);
  Serial.print(F(", jour :")), Serial.print(jour);
  Serial.print(F(" ,Calendrier :")), Serial.print(calendrier[month()][day()]);
  Serial.print(F(" ,flagCircule :")), Serial.println(flagCircule);
  byte pin = 0;
  Serial.println(F("***********************************"));
  if (wr == 99 || wr == 32 || wr == 33 || wr == 34) {
    pin = wr;
    wr = 3;
  }
  if (wr == 0)wr = 4; // demarrage normal, decision idem timer

  switch (wr) {
    case 2: break; // ne rien faire ESP_SLEEP_WAKEUP_EXT0

    case 3: // ESP_SLEEP_WAKEUP_EXT1

      /* declenchement externe pendant deep sleep
      	si nuit ou jour noncirculé
      	on reste en fonctionnement pendant TempoAnalyse
      	avant retour deep sleep*/
      // WupAlarme = true;
      // LastWupAlarme = true;
      // Alarm.enable(TempoAnalyse); // debut tempo analyse ->fonctionnement normal
      Sbidon = F("Externe Debut ");
      Sbidon += String(pin);
      MajLog(F("Alarme"), Sbidon);
      // }
      break;

    case 4: // SP_SLEEP_WAKEUP_TIMER
      /* jour noncirculé retour deep sleep pour RepeatWakeUp 1H00
        verifier si wake up arrive avant fin journée marge 1mn*/
      if (FirstWakeup) {
        SignalVie();
        FirstWakeup = false;
      }
      if ((calendrier[month()][day()] ^ flagCircule) && jour ) { // jour circulé (&& jour)
        // Nmax = config.Jour_Nmax; // parametre jour
        Sbidon = F("Jour circule ou demande circulation");
        Serial.println(Sbidon);
        MajLog(F("Auto"), Sbidon);
      }
      else { // non circulé
        Sbidon = F("Jour noncircule ou nuit");
        Serial.println(Sbidon);
        MajLog(F("Auto"), Sbidon);
        // Nmax = config.Nuit_Nmax; // parametre nuit
        calculTimeSleep();
        if (TIME_TO_SLEEP <= config.anticip) { // on continue sans sleep
          Sbidon = F("on continue sans sleep");
          Serial.println(Sbidon);
          MajLog(F("Auto"), Sbidon);
        }
        else {
          DebutSleep();
        }
      }
      break;

    case 5: break;  // ne rien faire ESP_SLEEP_WAKEUP_TOUCHPAD
    case 6: break;  // ne rien faire ESP_SLEEP_WAKEUP_ULP
      // default: break; // demarrage normal
  }
}
//---------------------------------------------------------------------------
void calculTimeSleep() {

  AIntru_HeureActuelle(); // determine si jour/nuit

  if (jour && (HActuelledec() + config.RepeatWakeUp) > config.FinJour) {
    if (HActuelledec() > (config.FinJour - config.anticip)) {
      /* eviter de reporter 24H si on est à moins de anticip de FinJour */
      TIME_TO_SLEEP = 1; // si 1 pas de sleep
    }
    else {
      TIME_TO_SLEEP = DureeSleep(config.FinJour - config.anticip);
      Serial.print(F("time sleep calcul 1 : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else if (!jour) {
    if (HActuelledec() < (config.DebutJour - config.anticip)) {
      TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);
      Serial.print(F("time sleep calcul 2 : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
    else if (HActuelledec() < 86400) {
      TIME_TO_SLEEP = (86400 - HActuelledec()) + config.DebutJour - config.anticip;
      Serial.print(F("time sleep calcul 2bis : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else {
    TIME_TO_SLEEP = config.RepeatWakeUp;
    Serial.print(F("time sleep calcul 3 : ")), print_uint64_t(TIME_TO_SLEEP);
    Serial.println("");
  }

  Sbidon = F("lance timer \"1H\" ");
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog(F("Auto"), Sbidon);
}
//---------------------------------------------------------------------------
int get_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  uint64_t wakeup_pin_mask;
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0  : return ESP_SLEEP_WAKEUP_EXT0; // 2
    case ESP_SLEEP_WAKEUP_EXT1: //{// 3
      wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_pin_mask != 0) {
        int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
        Serial.print(F("Wake up from GPIO ")); Serial.println(String(pin));
        return pin; // pin
      } else {
        Serial.println(F(" Wake up from GPIO ?"));
        return 99; // 99
      }
      break;
    // }
    case ESP_SLEEP_WAKEUP_TIMER    : return ESP_SLEEP_WAKEUP_TIMER; // 4
    case ESP_SLEEP_WAKEUP_TOUCHPAD : return ESP_SLEEP_WAKEUP_TOUCHPAD; // 5
    case ESP_SLEEP_WAKEUP_ULP      : return ESP_SLEEP_WAKEUP_ULP; // 6
    default : return 0; // Serial.println("Wakeup was not caused by deep sleep"); break;// demarrage normal
  }
  Serial.flush();
}

//---------------------------------------------------------------------------
void EffaceSMS(int s) {
  bool err;
  byte n = 0;
  do {
    err = Sim800.delSms(s);
    n ++;
    Serial.print(F("resultat del Sms "));	Serial.println(err);
    if (n > 10) { // on efface tous si echec
      err = Sim800.delAllSms();
      Serial.print(F("resultat delall Sms "));	Serial.println(err);
      break;
    }
  } while (!err);
}
//---------------------------------------------------------------------------
void print_uint64_t(uint64_t num) {

  char rev[128];
  char *p = rev + 1;

  while (num > 0) {
    *p++ = '0' + ( num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev) {
    Serial.print(*p--);
  }
}
//---------------------------------------------------------------------------
void init_adc_mm(void) {
  //initialisation des tableaux
  /* valeur par defaut facultative,
  	permet d'avoir une moyenne proche
  	du resulat plus rapidement
  	val defaut = valdefaut*nSample */
  unsigned int ini_adc1 = 0;// val defaut adc 1
  unsigned int ini_adc2 = 0;// val defaut adc 2
  unsigned int ini_adc3 = 0;// val defaut adc 3
  unsigned int ini_adc4 = 0;// val defaut adc 4
  for (int plus_ancien = 0; plus_ancien < nSample; plus_ancien++) {
    adc_hist[0][plus_ancien] = ini_adc1;
    adc_hist[1][plus_ancien] = ini_adc2;
    adc_hist[2][plus_ancien] = ini_adc3;
    adc_hist[3][plus_ancien] = ini_adc4;
  }
  //on commencera à stocker à cet offset
  adc_mm[0] = ini_adc1;
  adc_mm[1] = ini_adc2;
  adc_mm[2] = ini_adc3;
  adc_mm[3] = ini_adc4;
}
//---------------------------------------------------------------------------
void adc_read() {
  read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V); // lecture des adc
}
//---------------------------------------------------------------------------
void read_adc(int pin1, int pin2, int pin3, int pin4) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[4];
  for (byte i = 0; i < 4; i++) {
    if (i == 0)sample[i] = moyenneAnalogique(pin1);
    if (i == 1)sample[i] = moyenneAnalogique(pin2);
    if (i == 2)sample[i] = moyenneAnalogique(pin3);
    if (i == 3)sample[i] = moyenneAnalogique(pin4);

    //calcul MoyenneMobile
    adc_mm[i] = adc_mm[i] + sample[i] - adc_hist[i][plus_ancien];

    //cette plus ancienne valeur n'est plus utile, on y stocke la plus récente
    adc_hist[i][plus_ancien] = sample[i];
  }
  plus_ancien ++;
  if (plus_ancien == nSample) { //gestion du buffer circulaire
    plus_ancien = 0;
  }
}
//--------------------------------------------------------------------------------//
void messageId() {
  message  = Id;
  message += displayTime(0);
  message += fl;
}
//---------------------------------------------------------------------------
void HomePage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Parametres</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<td>Version</td>");
  webpage += F("<td>");	webpage += ver;	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Id</td>");
  webpage += F("<td>");	webpage += String(config.Idchar);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Debut Jour</td>");
  webpage += F("<td>");	webpage += Hdectohhmm(config.DebutJour);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Anticipation WakeUp (s)</td>");
  webpage += F("<td>");	webpage += String(config.anticip);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Fin Jour</td>");
  webpage += F("<td>");	webpage += Hdectohhmm(config.FinJour);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Vitesse SlowBlinker (ms)</td>");
  webpage += F("<td>");	webpage += String(config.SlowBlinker);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Vitesse FastBlinker (ms)</td>");
  webpage += F("<td>");	webpage += String(config.FastBlinker);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>R&eacute;p&eacute;tition FastBlink (ms)</td>");
  webpage += F("<td>");	webpage += String(config.FastRater);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>PWM Blanc (%)</td>");
  webpage += F("<td>");	webpage += String(config.FBlcPWM);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>PWM Violet (%)</td>");
  webpage += F("<td>");	webpage += String(config.FVltPWM);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Tempo r&eacute;p&eacute;tition Wake up Jour Circul&eacute; (s)</td>");
  webpage += F("<td>");	webpage += String(config.RepeatWakeUp);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>TimeOut Wifi (s)</td>");
  webpage += F("<td>");	webpage += String(config.timeoutWifi);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Bouton Poussoir local</td>");
  webpage += F("<td>");
  if (config.Bp) {
    webpage += F("Actif");
  } else {
    webpage += F("Inactif");
  }
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Liste Restreinte</td>");
  webpage += F("<td>");
  for (int i = 1; i < 10; i++) {
    webpage += String(config.Pos_Pn_PB[i]);
    if (i < 9) {
      webpage += (F(","));
    }
  }
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("</table><br>");

  webpage += F("<a href='/download'><button>Download</button></a>");
  webpage += F("<a href='/upload'><button>Upload</button></a>");
  webpage += F("<a href='/delete'><button>Delete</button></a>");
  webpage += F("<a href='/dir'><button>Directory</button></a>");
  webpage += F("<a href='/Tel_list'><button>Tel_list</button></a>");
  webpage += F("<a href='/cal'><button>Calendar</button></a>");
  webpage += F("<a href='/wifioff'><button>Wifi Off</button></a>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void Tel_listPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Liste des num&eacute;ros t&eacute;l&eacute;phone</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<th> Nom </th>");
  webpage += F("<th> Num&eacute;ro </th>");
  webpage += F("<th> Liste restreinte </th>");
  webpage += F("</tr>");
  byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
  for (byte i = 1; i < n + 1; i++) {
    String num = Sim800.getPhoneBookNumber(i);
    // Serial.print(num.length()), Serial.print(" "), Serial.println(num);
    if (num.indexOf("+CPBR:") == 0) { // si existe pas sortir
      Serial.println(F("Failed!"));// next i
      goto fin_liste;
    }
    String name = Sim800.getPhoneBookName(i);
    // Serial.println(name);
    webpage += F("<tr>");
    webpage += F("<td>"); webpage += name; webpage += F("</td>");
    webpage += F("<td>"); webpage += num ; webpage += F("</td>");
    webpage += F("<td>"); webpage += String(config.Pos_Pn_PB[i]); webpage += F("</td>");
    webpage += F("</tr>");
  }
fin_liste:


  webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void CalendarPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Calendrier</h3><br>");
  webpage += F("<table align='center'>");

  for (int m = 1; m < 13; m ++) {
    webpage += F("<tr>");
    webpage += F("<td>"); webpage += Mois[m]; webpage += F("</td>");
    for (int j = 1; j < 32; j++) {
      webpage += F("<td>");	webpage += calendrier[m][j];	webpage += F("</td>");
      if (j % 5 == 0)webpage += F("<td> </td>");
    }
    webpage += F("</tr>");
  }

  webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download() { // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("download")) DownloadFile(server.arg(0));
  }
  else SelectInput("Enter filename to download", "download", "download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DownloadFile(String filename) {
  if (SPIFFS_present) {
    File download = SPIFFS.open("/" + filename,  "r");
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Upload() {
  append_page_header();
  webpage += F("<h3>Select File to Upload</h3>");
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>");
  webpage += F("<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html", webpage);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void handleFileUpload() { // upload a new file to the Filing system
  HTTPUpload& uploadfile = server.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
  // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print(F("Upload File Name: ")); Serial.println(filename);
    SPIFFS.remove(filename);                  // Remove a previous version, otherwise data is appended the file again
    UploadFile = SPIFFS.open(filename, "w");  // Open the file for writing in SPIFFS (create it, if doesn't exist)
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // If the file was successfully created
    {
      UploadFile.close();   // Close the file again
      Serial.print(F("Upload Size: ")); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<h3>File was successfully uploaded</h3>");
      webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename + "</h2>";
      webpage += F("<h2>File Size: "); webpage += file_size(uploadfile.totalSize) + "</h2><br>";
      append_page_footer();
      server.send(200, "text/html", webpage);
      OuvrirCalendrier();
    }
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir() {
  if (SPIFFS_present) {
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      webpage += F("<h3 class='rcorners_m'>SPIFFS Contents</h3><br>");
      webpage += F("<table align='center'>");
      webpage += F("<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>");
      printDirectory("/", 0);
      webpage += F("</table>");
      SendHTML_Content();
      root.close();
    }
    else
    {
      SendHTML_Header();
      webpage += F("<h3>No Files Found</h3>");
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   // Stop is needed because no content length was sent
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  File root = SPIFFS.open(dirname);
  if (!root) {
    return;
  }
  if (!root.isDirectory()) {
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if (file.isDirectory()) {
      webpage += "<tr><td>" + String(file.isDirectory() ? "Dir" : "File") + "</td><td>" + String(file.name()) + "</td><td></td></tr>";
      printDirectory(file.name(), levels - 1);
    }
    else
    {
      webpage += "<tr><td>" + String(file.name()) + "</td>";
      webpage += "<td>" + String(file.isDirectory() ? "Dir" : "File") + "</td>";
      webpage += "<td>" + file_size(file.size()) + "</td></tr>";
    }
    file = root.openNextFile();
  }
  file.close();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Delete() {
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("delete")) SPIFFS_file_delete(server.arg(0));
  }
  else SelectInput("Select a File to Delete", "delete", "delete");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_file_delete(String filename) { // Delete the file
  if (SPIFFS_present) {
    SendHTML_Header();
    File dataFile = SPIFFS.open("/" + filename, "r"); // Now read data from SPIFFS Card
    if (dataFile)
    {
      if (SPIFFS.remove("/" + filename)) {
        Serial.println(F("File deleted successfully"));
        webpage += "<h3>File '" + filename + "' has been erased</h3>";
        webpage += F("<a href='/delete'>[Back]</a><br><br>");
      }
      else
      {
        webpage += F("<h3>File was not deleted - error</h3>");
        webpage += F("<a href='delete'>[Back]</a><br><br>");
      }
    } else ReportFileNotPresent("delete");
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String command, String arg_calling_name) {
  SendHTML_Header();
  webpage += F("<h3>"); webpage += heading1 + "</h3>";
  webpage += F("<FORM action='/"); webpage += command + "' method='post'>"; // Must match the calling argument e.g. '/chart' calls '/chart' after selection but with arguments!
  webpage += F("<input type='text' name='"); webpage += arg_calling_name; webpage += F("' value=''><br>");
  webpage += F("<type='submit' name='"); webpage += arg_calling_name; webpage += F("' value=''><br><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSPIFFSNotPresent() {
  SendHTML_Header();
  webpage += F("<h3>No SPIFFS Card present</h3>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//---------------------------------------------------------------------------
void handleTime() { // getion temps restant page web
  char time_str[9];
  const uint32_t millis_in_day    = 1000 * 60 * 60 * 24;
  const uint32_t millis_in_hour   = 1000 * 60 * 60;
  const uint32_t millis_in_minute = 1000 * 60;

  static unsigned long t0 = 0;
  if (millis() - debut > config.timeoutWifi * 1000) debut = millis(); // securité evite t<0
  t0 = debut + (config.timeoutWifi * 1000) - millis();
  // Serial.print(debut),Serial.print("|"),Serial.println(t0);

  uint8_t days     = t0 / (millis_in_day);
  uint8_t hours    = (t0 - (days * millis_in_day)) / millis_in_hour;
  uint8_t minutes  = (t0 - (days * millis_in_day) - (hours * millis_in_hour)) / millis_in_minute;
  uint8_t secondes = (t0 - (days * millis_in_day) - ((hours * millis_in_hour)) / millis_in_minute) / 1000 % 60;
  sprintf(time_str, "%02d:%02d:%02d", hours, minutes, secondes);
  // Serial.println(time_str);
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
void handleDateTime() { // getion Date et heure page web
  char time_str[20];
  sprintf(time_str, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
/* --------------------  test local serial seulement ----------------------*/
void recvOneChar() {

  char   receivedChar;
  static String serialmessage = "";
  static bool   newData = false;

  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    if (receivedChar != 10 && receivedChar != 13) {
      serialmessage += receivedChar;
    }
    else {
      newData = true;
      return;
    }
  }
  if (newData == true) {
    Serial.println(serialmessage);
    interpretemessage(serialmessage);
    newData = false;
    serialmessage = "";
  }
}

void interpretemessage(String demande) {
  String bidons;
  //demande.toUpperCase();
  if (demande.indexOf(char(61)) == 0) {
    bidons = demande.substring(1);
    bidons.toCharArray(replybuffer, bidons.length() + 1);
    traite_sms(99);//	traitement SMS en mode test local
  }
}
//---------------------------------------------------------------------------
