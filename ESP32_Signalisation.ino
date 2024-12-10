/* Ph CORBEL 10/2019
  Gestion Feux de Signalisation
  (basé sur ESP32_Tunnel)

  2 feux Violet et Blanc
  Etat des feux
              | Violet | Blanc | Feux | Cde
  OFF         |    0   |   0   |  0   |  D Feux D + Tqt Ouvert (si Tqt)
  Violet Fixe |    1   |   0   |  1   |  F
  Violet Cli  |  Cliv1 |   0   |  7   |  V Feux Violet Cli Marche à Vue
  Blanc Fixe  |    0   |   1   |  2   |  O
  Blanc Cli 1 |    0   |  Cli1 |  3   |  M
  Blanc Cli 2 |    0   |  Cli2 |  4   |  S
  Carré Tqt F |    1   |   0   |  5   |  C pas de cde, affichage seulement
  Carré Tqt O |    0   |   0   |  6   |  Z pas de cde, Feux D + Tqt Fermé

  cadence clignotant parametrable
  SlowBlinker = 500 Cli1 0.5s ON, 0.5s OFF
  Cli2 1s OFF, 0.15s ON/OFF pendant 1s
  FastRater = 1000/1300/1600/1900 FastBlinker = 150
  FastRater = 1000/1300/(1400)/1700/1800 FastBlinker = (200)

  Alimentation sur panneaux solaires

  mode deep sleep
  reveille tout les matin 06h55 (debut-anticip)
  reception des SMS en attente
  apres 5 min de fonctionnement (ex: 2mn pour reception/suppression 8 SMS, 4mn 14SMS)
  envoie sms signal vie
  analyse calendrier sauvegardé en SPIFFS

	si jour circulé
  on continue normalement
  en fin de journée retour sleep jusqu'a debut

  si non circulé,
  retour SIM7000 et ESP32 en sleep reveil toute les heures
  au reveil attendre au moins 30s pour que les SMS arrivent,
  quand plus de SMS et traitement retour sleep 1H00(config.RepeatWakeUp)

  Surveillance Batterie solaire
	Adc interne instable 2 à 3.5% erreurs!
	mise en place moyenne mobile sur les adc precision <1% avec 4bits

	Circulation = CalendrierCircule ^ flagCircule (OU exclusif)
	CalCircule	|	flagCircule | Circulation
				1			|			0				|			1
				0			|			1				|			1
				0			|			0				|			0
				1			|			1				|			0

	Librairie TimeAlarms.h modifiée a priori pas necessaire nonAVR = 12

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

  Version SIM7000G
  Revision:1529B08SIM7000G utilisé pour test

	to do
  FTP ne fonctionne pas?
  
  simplifier majheure au démarrage 
  https://randomnerdtutorials.com/esp32-ntp-timezones-daylight-saving/

  apres OTA wifi ne redemarre pas de temps en temps ?


  23/11/2024
  version V4-00 LTE-M
  Compilation LOLIN D32,default,80MHz, ESP32 2.0.17
  Arduino IDE 1.8.19 : 1111793 octets (84%), 56240 octets (17%) sur PC VScode
  Arduino IDE 1.8.19 : 1112257 octets (84%), 56240 octets (17%) sur Pi Mobile

*/

#include <Arduino.h>

String ver        = "V4-02";
int    Magique    = 5;

#define Exploitation // si defini pour Exploitation, sinon Test
#define TINY_GSM_MODEM_SIM7000

#include <Battpct.h>
#include "defs.h"
#include <TinyGsmClient.h>         // librairie TinyGSM revue PhC 0.12.0
#include <PubSubClient.h>
//#include <Time.h>
#include <TimeAlarms.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <Ticker.h>
#include "passdata.h"
#include <ArduinoJson.h>
#include <credentials_tpcf.h>

#define Serial Serial
#define SerialAT Serial2
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_USE_GPRS true
// #define TINY_GSM_USE_WIFI false
// #define GSM_PIN "1234"

String  webpage = "";
#define ServerVersion "1.0"
bool    LittleFS_present = false;
#include "CSS.h"               // pageweb

#define LED_PIN       5    // 
#define PinChckFblc   4    // Entrée verification Cde Feu Blanc
#define MODEM_PWRKEY  18   // Powerkey SIM7000
#define PinBattProc   35   // liaison interne carte Lolin32 adc
#define PinBattSol    39   // Batterie générale 12V adc VN
#define PinBattUSB    36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi
#define PinIp1        32   // Entrée Ip1 Wake up EXT1 Taquet VP Cv65 1=F
#define PinIp2        33   // Entrée Ip2 Wake up EXT1 Taquet V3 Cv65 1=F
#define Pin24V        26   // Mesure Tension 24V
#define PinFBlc       21   // Sortie Commande Feu Blanc
#define PinConvert    19   // Sortie Commande Convertisseur 12/24V
#define PinFVlt       15   // Sortie Commande Feu Violet
#define RX_PIN        17   // TX Sim7000
#define TX_PIN        16   // RX Sim7000
#define PinReset      13   // Reset Hard
#define PinLum        34   // Mesure Luminosité
#define PinAlimLum    25   // Alimentation LDR
#define PinTest       27   // Test sans GSM cc a la masse
#define NTPServer "pool.ntp.org"
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define FORMAT_LITTLEFS_IF_FAILED true // format LittelFS si lecture impossible
#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];            // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char fileconfig[12]      = "/config.txt";   // fichier en SPIFFS contenant structure config
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le log
char filelumlut[13]      = "/lumlut.txt";   // fichier en SPIFFS LUT luminosité
char filePhoneBook[8]    = "/pb.txt";       // fichier contenant liste N°tel autorisé

char PB_list[10][30];                       // PB liste en ram

const String soft = "ESP32_Signalisation.ino.d32"; // nom du soft

const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;                     //  Message envoyé
String Rmessage;                    //  Message reçu
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans config

bool Allume  = false;
byte BlcPwmChanel = 0;
byte VltPwmChanel = 1;
bool isBlinking = false;
bool blinker = false;

RTC_DATA_ATTR int  Feux = 0; // Etat des Feux voir tableau au début
RTC_DATA_ATTR bool FlagAlarmeTension       = false; // Alarme tension Batterie
RTC_DATA_ATTR bool FlagLastAlarmeTension   = false;
RTC_DATA_ATTR bool FirstWakeup             = true;  // envoie premier message vie une seule fois
RTC_DATA_ATTR bool flagCircule             = false; // circule demandé -> inverse le calendrier, valid 1 seul jour
RTC_DATA_ATTR bool FileLogOnce             = false; // true si log > seuil alerte

bool FlagAlarmeCdeFBlc       = false; // Alarme defaut commande Feu Blanc
bool FlagLastAlarmeCdeFBlc   = false;
bool FlagAlarme24V           = false; // Alarme tension 24V Allumage
bool FlagLastAlarme24V       = false;
bool FlagTqt_1               = false; // Position Taquet false = fermé, entree=0
bool FlagLastTqt_1           = false; // memo last etat
bool FlagTqt_2               = false; // Position Taquet false = fermé, entree=0
bool FlagLastTqt_2           = false; // memo last etat
bool FlagReset               = false; // Reset demandé
bool jour                    = false; // jour = true, nuit = false
bool gsm                     = true;  // carte GSM presente utilisé pour test sans GSM seulement
bool FlagAlarmeGprs          = false; // Alarme confirmée
bool AlarmeGprs              = false; // detection alarme
bool FlagLastAlarmeGprs      = false;
bool FlagAlarmeMQTT          = false;
bool AlarmeMQTT              = false;
bool FlagLastAlarmeMQTT      = false;
String Memo_Demande_Feux[3]  ={"","",""};  // 0 num demandeur,1 nom, 2 feux demandé (O2,M3,S4,V7)
bool FlagDemande_Feux        = false; // si demande encours = true
bool firstdecision           = false; // true si premiere décision apres lancement
bool flagRcvMQTT             = false; // true si reception MQTT longueur>0, false longueur = 0

int CoeffTension[4];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

int    slot = 0;              //this will be the slot number of the SMS

long   TensionBatterie  = 0; // Tension Batterie solaire
long   VBatterieProc    = 0; // Tension Batterie Processeur
long   VUSB             = 0; // Tension USB
long   Tension24        = 0; // Tension 24V Allumage
int    Lum              = 0; // Luminosité 0-100%
int    TableLum[11][2];      // Table PWM en fonction Luminosité

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqttClient(client);
WebServer server(80);
File UploadFile;

struct  config_t           // Structure configuration sauvée dans file config
{
  int     magic;           // num magique
  int     anticip;         // temps anticipation du reveille au lancement s
  long    DebutJour;       // Heure message Vie, 7h matin en seconde = 7*60*60
  long    FinJour;         // Heure fin jour, 20h matin en seconde = 20*60*60
  long    RepeatWakeUp;    // Periodicité WakeUp Jour non circulé
  int     timeoutWifi;     // tempo coupure Wifi si pas de mise a jour (s)
  bool    Ip1;             // E1 Actif
  bool    Ip2;             // E2 Actif
  int     SlowBlinker;     // ms
  int     FastBlinker;     // ms
  int     FastRater;       // ms
  int     FVltPWM;         // Modulation Feu Violet %
  int     FBlcPWM;         // Modulation Feu Blanc %
  bool    LumAuto;         // luminosité Auto=true
  bool    AutoF;           // true Retour automatique F si O/M/S apres TempoAutoF
  int     TempoAutoF;      // temps AutoF (s)
  char    Idchar[11];      // Id
  char    apn[11];         // APN
  char    gprsUser[11];    // user for APN
  char    gprsPass[11];    // pass for APN
  char    ftpServeur[26];  // serveur ftp
  char    ftpUser[9];      // user ftp
  char    ftpPass[16];     // pwd ftp
  int     ftpPort;         // port ftp
  int     TypeBatt;        // Type Batterie 16: Pb 6elts, 24 LiFePO 4elts
  byte    cptAla;          // Compteur alarmes Tracker avant declenchement
  char    mqttServer[26];  // Serveur MQTT
  char    mqttUserName[11];// MQTT User
  char    mqttPass[16];    // MQTT pass
  char    sendTopic[2][12];// output to server
  char    recvTopic[2][12];// input from server
  int     mqttPort;        // Port serveur MQTT
  int     hete;            // decalage Heure été UTC
  int     hhiver;          // decalage Heure hiver UTC
  bool    sendSMS;         // Autorisation envoyer SMS
  bool    autoupload;      // Upload automatique du fichier log
  uint16_t keepAlive;      // Paramètre keep alive de Pubsubclient
} ;
config_t config;

char willTopic[7];

int N_Y, N_M, N_D, N_H, N_m, N_S; // variable Date/Time temporaire
uint32_t lastReconnectMQTTAttempt = 0;
uint32_t lastReconnectGPRSAttempt = 0;

int NbrResetModem = 0;              // Nombre de fois reset modem, remise à 0 signal vie
int Histo_Reseau[5]={0,0,0,0,0};    // Historique Reseau cumul chaque mise à l'heure

Ticker SlowBlink;          // Clignotant lent
Ticker FastBlink;          // Clignotant rapide
Ticker FastRate;           // Repetition Clignotant rapide
Ticker ADC;                // Lecture des Adc

AlarmId loopPrincipale;    // boucle principale
AlarmId DebutJour;         // Debut journée
AlarmId FinJour;           // Fin de journée retour deep sleep
AlarmId Auto_F;            // Tempo AutoF

//---------------------------------------------------------------------------
void MajHeure(bool force = false);
//---------------------------------------------------------------------------
void setup() {
  message.reserve(300); // texte des reponses

  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Version Soft : ")), Serial.println(ver);

  if (!SPIFFS.begin(FORMAT_LITTLEFS_IF_FAILED)) { // Format la première fois utilise LitteFS
    Serial.println(F("SPIFFS initialisation failed..."));
    LittleFS_present = false;
  }
  else {
    Serial.println(F("SPIFFS initialised... file access enabled..."));
    LittleFS_present = true;
  }
  
  pinMode(PinIp1     , INPUT_PULLUP);
  pinMode(PinIp2     , INPUT_PULLUP);
  pinMode(PinFBlc    , OUTPUT);
  pinMode(PinFVlt    , OUTPUT);
  pinMode(PinConvert , OUTPUT);
  pinMode(PinAlimLum , OUTPUT);
  pinMode(PinTest    , INPUT_PULLUP);
  pinMode(PinChckFblc, INPUT_PULLUP);
  digitalWrite(PinConvert , LOW);  // Alimentation Convert 0
  digitalWrite(PinAlimLum , HIGH); // Alimentation de la LDR
  adcAttachPin(PinBattProc);
  adcAttachPin(PinBattSol);
  adcAttachPin(PinBattUSB);
  adcAttachPin(Pin24V);
  adcAttachPin(PinLum);

  if (digitalRead(PinTest) == 0) { // lire strap test, si = 0 test sans carte gsm
    gsm = false;
    setTime(12, 00, 00, 15, 07, 2019); // il faut initialiser la date et heure, jour circule et midi
    Serial.println("Lancement test sans carte gsm");
    Serial.println("mise à l'heure 14/07/2019 12:00:00");
    Serial.println("retirer le cavalier Pin27 et reset");
    Serial.println("pour redemarrer normalement");
  }

  if (gsm) {
    SerialAT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    modem_on();
    Serial.print(F("Reset modem: "));
    Serial.println(modem.setPhoneFunctionality(1,1));// CFUN=1,1 full functionality, reset online mode
    delay(200);
    Serial.print(F("Modem Info: "));
    Serial.println(modem.getModemInfo());
    modem.setNetworkMode(38);  // Network Mode LTE
    modem.setPreferredMode(1); // Mode CAT-M
    modem.disableGPS();        // Arret GPS
  }
  // parametrage PWM pour les feux
  // https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  ledcSetup(BlcPwmChanel, 1000, 8);
  ledcAttachPin(PinFBlc, BlcPwmChanel);
  ledcSetup(VltPwmChanel, 1000, 8);
  ledcAttachPin(PinFVlt, VltPwmChanel);

  ledcWrite(VltPwmChanel, 0); // Feu Violet 0
  ledcWrite(BlcPwmChanel, 0); // Feu Blanc 0

  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read); // lecture des adc toute les 100ms

  /* Lecture configuration file config	 */
  readConfig(); // Lecture de la config
  if (config.magic != Magique) {
    /* verification numero magique si different
      erreur lecture fileconfig ou carte vierge
      on charge les valeurs par défaut
    */
    Serial.println("Nouvelle Configuration !");
    config.magic         = Magique;
    config.anticip       = 2700;
    config.DebutJour     = 8  * 60 * 60;
    config.FinJour       = 19 * 60 * 60;
    config.RepeatWakeUp  = 60 * 60;
    config.timeoutWifi   = 15 * 60;
    config.Ip1           = false;
    config.Ip2           = false;
    config.SlowBlinker   = 500;
    config.FastBlinker   = 150;
    config.FastRater     = 1000;
    config.FBlcPWM       = 75;
    config.FVltPWM       = 75;
    config.LumAuto       = true;
    config.AutoF         = true;
    config.TempoAutoF    = 3600;
    config.TypeBatt      = 16; // Pb par défaut
    config.mqttPort      = tempmqttPort;
    config.hete          = 2; // heure
    config.hhiver        = 1; // heure
    config.sendSMS       = false; // pas d'envoie de SMS
    config.keepAlive     = 300; // 5mn, IMPERATIF pour réduire conso data
    config.autoupload    = false;
    config.cptAla        = 10; // 11*Acquisition time
    String temp          = "TPCF_CV65";
    temp.toCharArray(config.Idchar, 11);
    String tempapn       = "eapn1.net";
    String tempGprsUser  = "";
    String tempGprsPass  = "";
    config.ftpPort       = tempftpPort;
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempGprsUser.toCharArray(config.gprsUser,(tempGprsUser.length() + 1));
    tempGprsPass.toCharArray(config.gprsPass,(tempGprsPass.length() + 1));
    tempServer.toCharArray(config.ftpServeur,(tempServer.length() + 1));
    tempftpUser.toCharArray(config.ftpUser,(tempftpUser.length() + 1));
    tempftpPass.toCharArray(config.ftpPass,(tempftpPass.length() + 1));
    tempServer.toCharArray(config.mqttServer, (tempServer.length() + 1));
    tempmqttUserName.toCharArray(config.mqttUserName, (tempmqttUserName.length() + 1));
    tempmqttPass.toCharArray(config.mqttPass, (tempmqttPass.length() + 1));
    copie_Topic();

    sauvConfig();
  }
  PrintConfig();

  strncpy(willTopic,("S/will"),sizeof(willTopic));// topic commun

  Id  = String(config.Idchar);
  Id += fl;

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(config.Idchar);
  ArduinoOTA.setPasswordHash(OTApwdhash);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.print("Start updating ");
    Serial.println(type);
  })
  .onEnd([]() {
    Serial.println("End");
    delay(1000);
    ESP.restart();
    ResetHard();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });

  OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
  OuvrirLumLUT();             // ouvre le fichier lumLUT en SPIFFS
  Ouvrir_PB();                // ouvre le fichier Phone book
  // Serial.print(F("temps =")),Serial.println(millis());
  if (gsm) {
    Serial.print(("Waiting for network..."));
    if (!modem.waitForNetwork()) {
      Serial.println(F(" fail"));
      delay(100);
      // return;
    } else {
      Serial.println(F(" success"));
    }
    if (modem.isNetworkConnected()) { Serial.println(F("Network connected")); }
    byte n;
    byte cpt = 0;
    do {												// boucle tant que reseau pas connecté
      Alarm.delay(100);
      n = modem.getRegistrationStatus();
      cpt ++;
      if (cpt > 2) break;				// sortie si 2 tentatives demarrage sans reseau
    } while (!(n == 1 || n == 5));
    Serial.print(F("Network status "));
    Serial.print(n);
    Serial.print(F(": "));
    if (n == 0) Serial.println(F("Not registered"));
    if (n == 1) Serial.println(F("Registered (home)"));
    if (n == 2) Serial.println(F("Not registered (searching)"));
    if (n == 3) Serial.println(F("Denied"));
    if (n == 4) Serial.println(F("Unknown"));
    if (n == 5) Serial.println(F("Registered roaming"));

    // Demande Operateur connecté
    Serial.print(F("Operateur :")), Serial.println(modem.getOperator());

    ConnectGPRS();

    if (modem.isGprsConnected()) { Serial.println(F("GPRS connected")); }
    IPAddress local = modem.localIP();
    Serial.println("Local IP:" + local.toString());

    Serial.print("Signal quality:"), Serial.println(read_RSSI());

    mqttClient.setBufferSize(384);
    mqttClient.setKeepAlive(config.keepAlive);                // Set Pubsub keep alive interval
    mqttClient.setServer(config.mqttServer, config.mqttPort); // Set the MQTT broker details.
    mqttClient.setCallback(mqttSubscriptionCallback);         // Set the MQTT message handler function.

    // Synchro heure réseau du modem
    Serial.println(F("Synchro Heure réseau "));
    if(SyncHeureModem(config.hete*4, true)){ // heure été par defaut, first time
      Serial.println(F("OK"));
    } else {Serial.println(F("KO"));}
    Serial.println(modem.getGSMDateTime(TinyGSMDateTimeFormat(0)));

    mqttConnect();
    if (mqttSubscribe(0) == true ) {
      Serial.println("Subscribed");
    }    
    timesstatus();								// Etat synchronisation Heure Sys
    MajHeure();
  }

  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  DebutJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);
  Alarm.enable(DebutJour);

  FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee); // Fin de journée retour deep sleep
  Alarm.enable(FinJour);

  Auto_F = Alarm.timerRepeat(config.TempoAutoF, AutoFermeture);
  Alarm.disable(Auto_F);

  Serial.print(F("flag Circule :")), Serial.println(flagCircule);

  if(!config.Ip1){ // si Ip1 innactive FlagTqt_1 = taquet ouvert
    FlagTqt_1 = true;
    FlagLastTqt_1 = false;
  }

  // if (Feux != 0) { // si une valeur Feux different de 0 en memoire RTC, on Allume les feux
    // Allumage();
  // }
  
  MajLog("Auto","Lancement");
  // reduire consommation eteindre Wifi et BT
  WiFi.mode(WIFI_OFF);
  btStop();
}
//---------------------------------------------------------------------------
void loop() {
  static unsigned int timer0 = millis();
  bool first = true;
  recvOneChar(); // Capture reception liaison serie locale

  if(gsm){
    // make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) { // NETOPEN? Start TCPIP service
      Serial.println(F("GPRS disconnected!"));
      Serial.print(F("Connecting to "));
      Serial.print(config.apn);
      if (!modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {        
        Serial.println(F(" fail"));
        if (millis() - lastReconnectGPRSAttempt > 10000L){
          lastReconnectGPRSAttempt = millis();
          AlarmeGprs = true;
        }
      } else {lastReconnectGPRSAttempt = millis();}// =0 est une erreur
      if (modem.isGprsConnected()) {
        Serial.println(F(" GPRS reconnected"));
        AlarmeGprs = false;
      }
    }
    if (!mqttClient.connected() && ((millis()- timer0) > 5000 || first)){
      mqttConnect(); // Connect if MQTT client is not connected.
      if (mqttSubscribe(0) == true ) {
        Serial.println("Subscribed");
      }
      timer0 = millis();
      first = false;
    }
    mqttClient.loop(); // Call the loop to maintain connection to the server.
  }
  //*************** Verification position taquet ***************
  VerifTaquet_1(); // si Cv65 Taquet Vp
  VerifTaquet_2(); // si Cv65 Taquet V3
  //*************** Verification commande Feu Blanc ***************
  if (Allume) VerifCdeFBlc();

  ArduinoOTA.handle();
  Alarm.delay(0);

  /* calcul temps de boucle */
  // static unsigned long debutloop = millis();
  // static unsigned long cumultimeloop = 0;
  // static int compteurloop = 0;
  // cumultimeloop += millis() - debutloop;
  // if(compteurloop ++ > 10000){
    // Serial.print("temps loop:"),Serial.print((float)cumultimeloop/10000);
    // Serial.print(", cpt   loop:"),Serial.println(compteurloop);
    // compteurloop = 0;
    // cumultimeloop = 0;
  // }
  // debutloop = millis();

}	//fin loop
//---------------------------------------------------------------------------
void Acquisition() {

  if(config.Ip1){ // si E1 active
    Serial.print("Taquet VP: ");
    if(FlagTqt_1){
      Serial.print("Ouvert");
    } else{
      Serial.print("Ferme");
    }
    Serial.print(", lasttaquet VP: ");
    if(FlagLastTqt_1){
      Serial.print("Ouvert");
    } else {
      Serial.print("Ferme");
    }
    Serial.print(", Demande Feux en attente:"),Serial.println(FlagDemande_Feux);
  }
  if(config.Ip2){ // si E2 active
    Serial.print("Taquet V3: ");
    if(FlagTqt_2){
      Serial.print("Ouvert");
    } else{
      Serial.print("Ferme");
    }
    Serial.print(", lasttaquet V3: ");
    if(FlagLastTqt_2){
      Serial.println("Ouvert");
    } else {
      Serial.println("Ferme");
    }
  }

  if(gsm){
    static int cptRegStatusFault = 0;
    Serial.print(F("Cnx reseau  (1)  :")),Serial.println(modem.isNetworkConnected());    // CEREG, CGREG
    Serial.print(F("Reg  status (1/5):")),Serial.println(modem.getRegistrationStatus()); // CEREG, CGREG
    Serial.print(F("Cnx GPRS    (1)  :")),Serial.println(modem.isGprsConnected());       // CGDCONT
    Serial.print(F("cptRegStatusFault:")),Serial.println(cptRegStatusFault);
    // Patch Blocage modem
    if(modem.getRegistrationStatus() != 1 && modem.getRegistrationStatus() != 5){
      if(cptRegStatusFault ++ > config.cptAla){
        cptRegStatusFault = 0;
        NbrResetModem +=1;
        Serial.println(F("Reset modem suite Reg status fault"));
        modem.send_AT(F("+CFUN=1,1"));
        delay(10000);
      }
    } else {
      cptRegStatusFault = 0;// reset compteur
    }
  }

  Serial.print(modem.getOperator());
  IPAddress local = modem.localIP();
  Serial.println("; IP:" + local.toString());
  
  static int cpt = 0; // compte le nombre de passage boucle
  
  static byte cptallume = 0; // compte le nombre de passage avec Allume

  AIntru_HeureActuelle();

  if (cpt > 5 && !firstdecision) {
    /* une seule fois au demarrage attendre au moins 60s */
    action_wakeup_reason(get_wakeup_reason());
    firstdecision = true;
  }
  cpt ++;
  if((config.Ip1 || config.Ip2) && firstdecision) gestionTaquet(); // gestion etat taquet seulement apres demarrage

  if (CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0 || CoeffTension[3] == 0) {
    OuvrirFichierCalibration(); // patch relecture des coeff perdu
  }

  Serial.println(displayTime(0));
  // Serial.print(F(" Freemem = ")), Serial.println(ESP.getFreeHeap());
  static byte nalaTension = 0;
  static byte nRetourTension = 0;
  TensionBatterie = map(adc_mm[0] / nSample, 0, 4095, 0, CoeffTension[0]);
  VBatterieProc   = map(adc_mm[1] / nSample, 0, 4095, 0, CoeffTension[1]);
  VUSB            = map(adc_mm[2] / nSample, 0, 4095, 0, CoeffTension[2]);
  Tension24       = map(adc_mm[3] / nSample, 0, 4095, 0, CoeffTension[3]);
  Lum             = map(adc_mm[4] / nSample, 0 , 4095, 100, 0); // Luminosité 0-100%

  // Serial.print("luminosité = "), Serial.print(Lum);
  // Serial.print(" lumlut = "), Serial.println(lumlut(Lum));

  // en cas de feux fixe rafraichissement commande en fonction lum
  // les feux M et S sont automatiquement ajusté par blink
  if (Feux == 1) Update_FVlt(); // Violet
  if (Feux == 2) Update_FBlc(); // Blanc

  if (Allume) {
    cptallume ++;
    Serial.print(F("Tension 24V :")), Serial.print(float(Tension24 / 100.0));
    Serial.print(" coeff 24V="), Serial.println(CoeffTension[3]);
    if (cptallume > 2 && Tension24 < 2000) { // on attend 3 passages pour mesurer 24V
      FlagAlarme24V = true;
    }
    else if (Tension24 > 2100) {
      FlagAlarme24V = false;
    }
  }
  else {
    cptallume = 0;
    FlagAlarme24V = false;
  }
  int etatbatt = 0;
  if (config.TypeBatt == 16) etatbatt = BattPBpct(TensionBatterie, 6);
  if (config.TypeBatt == 24) etatbatt = BattLiFePopct(TensionBatterie, 4);

  if (etatbatt < 25 || VUSB < 4000) { // || VUSB > 6000
    nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
  else if (etatbatt >= 80 && VUSB >= 4500) { //  && VUSB < 5400	//hysteresis et tempo sur Alarme Batterie
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
  if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
  if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
  message += "%";
  message += F(", Batt Proc = ");
  message += (String(VBatterieProc) + "mV ");
  message += String(BattLipopct(VBatterieProc));
  message += (F("%, V USB = "));
  message += (float(VUSB / 1000.0));
  message += ("V");
  message += fl;
  Serial.print(message);

  if (gsm) {
    // verification index new SMS en attente(raté en lecture directe)
    int smsnum = modem.newMessageIndex(0); // verifie index arrivée sms, -1 si pas de sms
    Serial.print(F("Index last SMS = ")), Serial.println (smsnum);

    if (smsnum >= 0) {	// index du SMS en attente
      // il faut les traiter
      ReadSMS(smsnum);// Lecture et traitement de tous les SMS en attente
    } 
    else if (smsnum < 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
      FlagReset = false;
      ResetHard();					//	reset hard
    }
  }

  if(gsm && firstdecision){ // apres demarrage
    static byte nalaGprs = 0;
    static byte nalaMQTT = 0;
    if (AlarmeGprs) {
      if (nalaGprs ++ > config.cptAla) {
        FlagAlarmeGprs = true;
        nalaGprs = 0;
      }
    } else {
      if (nalaGprs > 0) {
        nalaGprs --;
      } else {
        FlagAlarmeGprs = false;
      }
    }
    if (AlarmeMQTT) {
      mqttConnect(); // tentative reconnexion MQTT
      if (nalaMQTT ++ > config.cptAla) {
        FlagAlarmeMQTT = true;
        // A faire action pour reconnecter MQTT
        nalaMQTT = 0;
      }
      Serial.print(F("AlarmeMQTT: ")),Serial.println(nalaMQTT);
    } else {
      FlagAlarmeMQTT = false;
      FlagAlarmeGprs = false;
      nalaMQTT = 0;
    }
  }
  envoie_alarme();

  digitalWrite(LED_PIN, 0);
  Alarm.delay(20);
  digitalWrite(LED_PIN, 1);

  Serial.println();
}
//---------------------------------------------------------------------------
void GestionFeux() {
  switch (Feux) {
    case 0: // Violet 0, Blanc 0
      Serial.println("Feux Eteint");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFVlt, LOW);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, LOW); // extinction Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 6: // Violet 0, Blanc 0 Taquet Ouvert
      Serial.println("Feux Eteint");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFVlt, LOW);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, LOW); // extinction Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 1: // Violet 1, Blanc 0
      Serial.println("Feu Violet");
      Update_FVlt();
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 5: // Violet 1, Blanc 0 Taquet Fermé
      Serial.println("Feu Violet");
      Update_FVlt();
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 2: // Violet 0, Blanc 1
      Serial.println("Feu Blanc");
      ledcWrite(VltPwmChanel, 0);
      Update_FBlc();
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 3: // Violet 0, Blanc Cli1
      Serial.println("Feu Blc Clignotant lent");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      digitalWrite(PinFVlt, LOW);
      FastBlink.detach();
      FastRate.detach();
      SlowBlink.attach_ms(config.SlowBlinker, blink);
      break;
    case 4: // Violet 0, Blanc Cli2
      Serial.println("Feu Blc Clignotant rapide");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      digitalWrite(PinFVlt, LOW);
      digitalWrite(PinFBlc, LOW);
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      isBlinking = true;
      blinker = false;
      FastRate.attach_ms(config.FastRater, toggle);
      break;
    case 7: // V, Violet Cli, Blanc 0
      Serial.println("Feux Vlt Clignotant lent");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      digitalWrite(PinFVlt, LOW);
      FastBlink.detach();
      FastRate.detach();
      SlowBlink.attach_ms(config.SlowBlinker, blink);
      break;
    default:// idem 0 Violet 0, Blanc 0
      Serial.println("Feux Eteint");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFVlt, LOW);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, LOW); // extinction Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
  }
}
//---------------------------------------------------------------------------
void toggle() {
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
  if (blinker) {
    if(Feux == 3 || Feux == 4){// M ou S
      ledcWrite(BlcPwmChanel, 0);
    } else if (Feux == 7){     // V
      ledcWrite(VltPwmChanel, 0);
    }
    blinker = false;
  } else {
    if(Feux == 3 || Feux == 4){// M ou S
      Update_FBlc();
    } else if (Feux == 7){     // V
      Update_FVlt();
    }
    blinker = true;
  }
}
//---------------------------------------------------------------------------
void Update_FVlt() {
  if (config.LumAuto) {
    ledcWrite(VltPwmChanel, 255 * lumlut(Lum) / 100);
  }
  else {
    ledcWrite(VltPwmChanel, 255 * config.FVltPWM / 100);
  }
}
//---------------------------------------------------------------------------
void Update_FBlc() {
  if (config.LumAuto) {
    ledcWrite(BlcPwmChanel, 255 * lumlut(Lum) / 100);
  }
  else {
    ledcWrite(BlcPwmChanel, 255 * config.FBlcPWM / 100);
  }
}
//---------------------------------------------------------------------------
// Allumage du Feu
void Allumage() {
  if(!Allume){
    Serial.println("Allumage");
    Allume = true;
    digitalWrite(PinConvert, HIGH); // Alimentation du convertisseur 12/24V
  }
  GestionFeux();
}
//---------------------------------------------------------------------------
// Ectinction du Feu
void Extinction() {
  Serial.println("Exctinction");
  if(config.Ip1){  // si E1 active
    if(FlagTqt_1){ // taquet ouvert
      Feux = 0;    // D tout eteint et Taquet Ouvert
    } else {
      Feux = 6;    // Z tout eteint et Taquet Fermé
    }
  } else {
    Feux = 0;      // D tout eteint pas de Taquet
  }
  GestionFeux();
  Allume = false;
  digitalWrite(PinConvert, LOW); // Arret du convertisseur 12/24V
  MajLog(F("Auto"), "Feux = " + String(Feux));
  envoieGroupeMessage(false,true); // envoie serveur
}
//---------------------------------------------------------------------------
// Fermeture Feu Automatique
void AutoFermeture() {
  // fin de TempoAutoF
  // Feux à F si Feux = O/S/V rien faire si M
  if (Feux == 2 || Feux == 4 || Feux == 7) {
    Feux = 1;
    Allumage(); // Violet 1, Blanc 0
    envoieGroupeMessage(false,true); // envoie serveur
    MajLog("AutoF", "FCV");
  }
  Alarm.disable(Auto_F);
}
//---------------------------------------------------------------------------
// Lecture SMS
void ReadSMS(int index){
  // index du SMS
  // verifier appelant connu si OK copier texte sms dans Rmessage
  // effacer SMS
  // et envoyer traite_sms("SMS")

  Sms smsstruct;
  if (!modem.readSMS(&smsstruct,index)){
    Serial.print(F("Didn't find SMS message in slot! "));
    Serial.println(index);
  }
  if(! Cherche_N_PB(smsstruct.sendernumber)){
    Serial.println(F("Appelant inconnu"));
    EffaceSMS(index);
    return;
  }
  Rmessage = smsstruct.message;
  EffaceSMS(index);
  traite_sms("SMS");
}
//---------------------------------------------------------------------------
// Interpretation des messages
// Origine = Local, BLE, SMS, MQTTS (serveur), MQTTU (user)
void traite_sms(String Origine) {
  bool sms = false;
  if(Origine == "SMS") sms = true;
  
  bool smsserveur = false; // true si le sms provient du serveur
  if (Origine == "MQTTS") smsserveur = true;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;           //	memorisation tension batterie lors de la calibration
  int coef = 0;                         // coeff temporaire
  static byte P = 0;                    // Pin entrée a utiliser pour calibration
  static byte M = 0;                    // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  Serial.print("message: "), Serial.print(Rmessage),Serial.print(","),Serial.println(Rmessage.length());

  if (!(Rmessage.indexOf(F("TEL")) == 0 || Rmessage.indexOf(F("tel")) == 0 || Rmessage.indexOf(F("Tel")) == 0
      || Rmessage.indexOf(F("Wifi")) == 0
      || Rmessage.indexOf(F("MQTTDATA")) > -1 || Rmessage.indexOf(F("MQTTSERVEUR")) > -1
      || Rmessage.indexOf(F("GPRSDATA")) > -1 || Rmessage.indexOf(F("FTPDATA")) > -1 || Rmessage.indexOf(F("FTPSERVEUR")) > -1)) {
    Rmessage.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI"... parametres pouvant contenir minuscules
    Rmessage.replace(" ", "");// supp tous les espaces
  }

  messageId();
  if (Rmessage.indexOf(F("TIMEOUTWIFI")) > -1) { // Parametre Arret Wifi
    if (Rmessage.indexOf(char(61)) == 11) {
      int n = Rmessage.substring(12, Rmessage.length()).toInt();
      if (n > 9 && n < 3601) {
        config.timeoutWifi = n;
        sauvConfig();														// sauvegarde config
      }
    }
    message += F("TimeOut Wifi (s) = ");
    message += config.timeoutWifi;
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("WIFIOFF")) > -1) { // Arret Wifi
    message += F("Wifi off");
    message += fl;
    sendReply(Origine);
    WifiOff();
  }
  else if (Rmessage.indexOf(F("Wifi")) == 0) { // demande connexion Wifi
    byte pos1 = Rmessage.indexOf(char(44));//","
    byte pos2 = Rmessage.indexOf(char(44), pos1 + 1);
    if(pos1==255 || pos1<4 || pos2==255 || pos2<4){
      // format incomplet
      message += "erreur format";
      sendReply(Origine);
      return;
    }
    String ssids = Rmessage.substring(pos1 + 1, pos2);
    String pwds  = Rmessage.substring(pos2 + 1, Rmessage.length());
    char ssid[25];
    char pwd[30];
    ssids.toCharArray(ssid, ssids.length() + 1);
    ssids.toCharArray(ssid, ssids.length() + 1);
    pwds.toCharArray(pwd, pwds.length() + 1);
    ConnexionWifi(ssid, pwd, Origine);
  }
  else if (Rmessage.indexOf(F("TEL")) == 0
        || Rmessage.indexOf(F("Tel")) == 0
        || Rmessage.indexOf(F("tel")) == 0) { // entrer nouveau num
    byte lastPBline = last_PB(); // recupere le n° de la derniere ligne du PB
    bool newPB = false;
    bool FlagOK = true;
    bool efface = false;
    byte j = 0;
    String newnumero;
    String newnom;
    int indexreplace = 0;
    if (Rmessage.indexOf(char(61)) == 4) {  // TELn= reserver correction/suppression
      int i = Rmessage.substring(3).toInt();// recupere n° de index
      i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
      if (i < 1) FlagOK = false;
      indexreplace = i;// index du PB a remplacer
      j = 5;
      // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
      if ((i != 1) && (i<=lastPBline) &&(Rmessage.indexOf(F("efface")) == 5 || Rmessage.indexOf(F("EFFACE")) == 5 )) {
        efface = true;
        strcpy(PB_list[i] , "");
        if(i < lastPBline){
          // il faut décaler les lignes vers le bas
          for (int ligne = i;ligne<lastPBline;ligne ++){
            strcpy(PB_list[ligne] , PB_list[ligne+1]);
          }
          if(lastPBline < 9){
            strcpy(PB_list[lastPBline] , "");// efface derniere ligne
          }
        }
        Save_PB();
        message += "ligne effacee";
        goto fin_tel;
      }
    }
    else if (Rmessage.indexOf(char(61)) == 3) { // TEL= nouveau numero
      j = 4;
      newPB = true;
    }
    else {
      FlagOK = false;
    }
    if (Rmessage.indexOf("+") == j) {			          // debut du num tel +
      if (Rmessage.indexOf(char(44)) == j + 12) {	  // verif si longuer ok
        newnumero = Rmessage.substring(j, j + 12);
        newnom = Rmessage.substring(j + 13, j + 27);// tronque à 14 car
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
      message += F("Cde non reconnue/erreur ?");// non reconnu
      message += fl;
      sendReply(Origine);
    }
    else {
      if (!efface) {
        String bidon = newnumero + ";" + newnom;
        if(newPB){ // Nouvelle ligne
          strcpy(PB_list[lastPBline + 1] , bidon.c_str());
        } else {   // Remplacement ligne
          strcpy(PB_list[indexreplace] , bidon.c_str());
        }
        message += "Nouvelle entree Phone Book :" + fl;
        message += bidon;

      }
      Save_PB();
      sendReply(Origine);
    }
  }
  else if (gsm && (Rmessage == F("LST") || Rmessage == F("LST?") || Rmessage == F("LST1"))) {	//	Liste des Num Tel
    Read_PB();
    for(byte i = 1;i<10;i++){
      if(strlen(PB_list[i]) > 0){
        // Serial.println(PB_list[i]);
        message += i;
        message += ":";
        message += String(PB_list[i]);
        message += fl;
      }
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("ETAT")) == 0 || Rmessage.indexOf(F("ST")) == 0) {// "ETAT? de l'installation"
    generationMessage();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SYS")) > -1) {
    if (gsm) {
      message += modem.getOperator(); // Operateur
      message += fl;
      byte n = modem.getRegistrationStatus();        
      if (n == 5) {
        message += F(("rmg, "));// roaming 1.0s
      }
      message += " ";
      message += ConnectedNetwork();
      message += fl;
      message += read_RSSI();														// info RSSI
      message += fl;
      message += F("Batt GSM : ");
      message += String(modem.getBattVoltage());
      message += F(" mV, ");
      message += fl;
    }
    message += F("Ver: ");
    message += ver;
    message += fl;
    message += F("V Batt Sol= ");
    message += String(float(TensionBatterie / 100.0));
    message += F("V, ");
    if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += " %";
    message += fl;
    message += F("V USB= ");
    message += (float(VUSB / 1000.0));
    message += "V";
    message += fl;
    if(Allume){
      message += F("V 24= ");
      message += String(float(Tension24 / 100.0));
      message += "V";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
    String temp = Rmessage.substring(3);
    if (temp.length() > 0 && temp.length() < 11) {
      Id = "";
      temp.toCharArray(config.Idchar, 11);
      mqttSubscribe(1); // unsubscribe
      mqttClient.disconnect();

      copie_Topic(); // Nouvel Id dans Topic
      sauvConfig();														// sauvegarde config

      mqttConnect();
      mqttSubscribe(0); // subscribe

      Id = String(config.Idchar);
      Id += fl;
    }
    messageId();
    message += F("Nouvel Id");
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("LOG")) == 0) {	// demande taille du log
    File f = SPIFFS.open(filelog, "r");       // taille du fichier log en SPIFFS
    message = F("local log size :");
    message += String(f.size()) + fl;
    f.close();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("ANTICIP")) > -1) { // Anticipation du wakeup
    if (Rmessage.indexOf(char(61)) == 7) {
      int n = Rmessage.substring(8, Rmessage.length()).toInt();
      if (n > 9 && n < 3601) {
        config.anticip = n;
        sauvConfig();														// sauvegarde config
      }
    }
    message += F("Anticipation WakeUp (s) = ");
    message += config.anticip;
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("DEBUT")) == 0) {     //	Heure Message Vie/debutJour
    if (Rmessage.indexOf(char(61)) == 5) {
      long i = atol(Rmessage.substring(6).c_str()); //	Heure message Vie
      if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
        config.DebutJour = i;
        sauvConfig();                               // sauvegarde config
        Alarm.disable(DebutJour);
        Alarm.write(DebutJour,config.DebutJour);
        // FinJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);// init tempo
        Alarm.enable(DebutJour);
        AIntru_HeureActuelle();
      }
    }
    message += F("Debut Journee = ");
    message += Hdectohhmm(config.DebutJour);
    message += F("(hh:mm)");
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TIME")) == 0) {
    message += F("Heure Sys = ");
    message += displayTime(0);
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure
    MajHeure(true);			// mise a l'heure forcée
    messageId();
    message += "Mise à l'heure NTP";
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("IMEI")) > -1) {
    message += F("IMEI = ");
    message += modem.getIMEI();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("FIN")) == 0) {			  //	Heure Fin de journée
    if ((Rmessage.indexOf(char(61))) == 3) {
      long i = atol(Rmessage.substring(4).c_str()); //	Heure
      if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
        config.FinJour = i;
        sauvConfig();															  // sauvegarde config
        Alarm.disable(FinJour);
        Alarm.write(FinJour,config.FinJour);
        // FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee);// init tempo
        Alarm.enable(FinJour);
      }
    }
    message += F("Fin Journee = ");
    message += Hdectohhmm(config.FinJour);
    message += F("(hh:mm)");
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("AUTOF")) == 0) {
    if ((Rmessage.indexOf(char(61))) == 5) { // =
      if (Rmessage.substring(6) == "1" || Rmessage.substring(6) == "0") {
        config.AutoF = Rmessage.substring(6).toInt();
        sauvConfig();	// sauvegarde config
      }
    }
    message += "AutoF ";
    if (config.AutoF == 1) {
      message += "Auto";
    }
    else {
      message += "Manu";
    }
    message += fl;
    message +=  "TempoAutoF (s) = ";
    message += config.TempoAutoF + fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TEMPOAUTOF")) == 0) {
    if ((Rmessage.indexOf(char(61))) == 10) { // =
      if (Rmessage.substring(11).toInt() > 100 && Rmessage.substring(11).toInt() < 36000) {
        config.TempoAutoF = Rmessage.substring(11).toInt();
        sauvConfig();	// sauvegarde config
        Alarm.disable(Auto_F);
        Alarm.write(Auto_F,config.TempoAutoF);
      }
    }
    message += "AutoF ";
    if (config.AutoF == 1) {
      message += F("Auto");
    }
    else {
      message += F("Stop");
    }
    message += fl;
    message +=  "TempoAutoF (s) = ";
    message += config.TempoAutoF + fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("LUMACTUELLE")) == 0) {
    message += F("Lum ");
    if (config.LumAuto) {
      message += F("Auto");
    }
    else {
      message += F("Manu");
    }
    message += fl;
    message += F("luminosite = ");
    message += String(Lum);
    message += F("\nlumlut = ");
    message += String(lumlut(Lum));
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("LUMAUTO")) == 0) {
    if ((Rmessage.indexOf(char(61))) == 7) { // =
      if (Rmessage.substring(8) == "1" || Rmessage.substring(8) == "0") {
        config.LumAuto = Rmessage.substring(8).toInt();
        sauvConfig();	// sauvegarde config
      }
    }
    message += F("Luminosite ");
    if (config.LumAuto) {
      message += "Auto";
    }
    else {
      message += "Manu";
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("LUMLUT")) > -1) { // Luminosité Look Up Table
    // format valeur de luminosité Feux pour chaque valeur luminosite ambiante
    // de 100 à 0 pas de 10
    // LUMLUT=95,90,80,75,60,50,40,30,30,30,30
    bool flag = true; // validation du format
    byte nv = 0; // compteur virgule
    byte p1 = 0; // position virgule
    if (Rmessage.indexOf("{") == 0) { // json
      JsonDocument doc;
      int f = Rmessage.lastIndexOf("}");
      // Serial.print("pos }:"),Serial.println(f);
      // Serial.print("json:"),Serial.print(Rmessage.substring(0,f+1)),Serial.println(".");
      DeserializationError err = deserializeJson(doc, Rmessage.substring(0, f + 1));
      if(!err){
        JsonArray LUMLUT = doc["LUMLUT"];
        for (int i = 0; i < 11; i++) {
          TableLum[i][1] = LUMLUT[i];
        }
      }
      else{
        flag = false; // erreur json
      }
    }
    else if ((Rmessage.indexOf(char(61))) == 6) { // =
      Sbidon = Rmessage.substring(7, Rmessage.length());
      for (int i = 0; i < Sbidon.length(); i++) {
        p1 = Sbidon.indexOf(char(44), p1 + 1); // ,
        if ((p1 > 0 && p1 < 255)) {
          nv ++;
          if (nv == 10)break;
        } 
        else {
          flag = false;
        }
      }
      if (flag) { // format ok
        p1 = 0;
        byte p2 = 0;
        for (int i = 0; i < 11; i++) {
          p2 = Sbidon.indexOf(char(44), p1 + 1); // ,
          TableLum[i][1] = Sbidon.substring(p1, p2).toInt();
          // Serial.printf("%s%d,%s%d\n","p1=",p1,"p2=",p2);
          p1 = p2 + 1;
          TableLum[i][0] = 100 - i * 10;
          if (!(TableLum[i][1] >= 0 && TableLum[i][1] < 101)) flag = false;
          // Serial.printf("%03d,%03d\n",TableLum[i][0],TableLum[i][1]);
        }
      }
    }
    if (flag) { // données OK on enregistre
      EnregistreLumLUT();
    }
    else { // données KO on enregistre pas, et on relie les données en mémoire
      OuvrirLumLUT();
    }
    if (smsserveur || !sms) {
      // si serveur reponse json
      JsonDocument doc;
      JsonArray lumlut = doc["lumlut"].to<JsonArray>();
      for (int i = 0; i < 11; i++) {
        lumlut.add(TableLum[i][1]);
      }
      String jsonbidon;
      serializeJson(doc, jsonbidon);
      message += jsonbidon;
    } else {
      message += F("Table Luminosite (%)\n");
      char bid[10];// 1 ligne
      for (int i = 0; i < 11; i++) {
        sprintf(bid, "%03d,%03d\n", TableLum[i][0], TableLum[i][1]);
        message += String(bid);
      }
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("MOIS")) > -1) { // Calendrier pour un mois
    /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
      demande calendrier pour un mois donné ; format : MOIS=mm? */
    bool flag = true; // validation du format
    bool W = true;    // true Write, false Read
    int m = 0;
    if (Rmessage.indexOf("{") == 0) { // json
      JsonDocument doc;
      int f = Rmessage.lastIndexOf("}");
      DeserializationError err = deserializeJson(doc, Rmessage.substring(0, f + 1));
      if(!err){
        m = doc["MOIS"]; // 12
        JsonArray jour = doc["JOUR"];
        for (int j = 1; j < 32; j++) {
          calendrier[m][j] = jour[j - 1];
        }
        // Serial.print("mois:"),Serial.println(m);
        EnregistreCalendrier(); // Sauvegarde en SPIFFS
      }
      else{
        message += " erreur json ";
        flag = false;
      }
    }
    else { // message normal mois=12,31*0/1
      byte p1 = Rmessage.indexOf(char(61)); // =
      byte p2 = Rmessage.indexOf(char(44)); // ,
      if (p2 == 255) {                      // pas de ,
        p2 = Rmessage.indexOf(char(63));    // ?
        W = false;
      }

      m = Rmessage.substring(p1 + 1, p2).toInt(); // mois
      if (!(m > 0 && m < 13)) flag = false;
      if (W && flag) { // Write
        if (!(Rmessage.substring(p2 + 1, Rmessage.length()).length() == 31)) flag = false; // si longueur = 31(jours)

        for (int i = 1; i < 32; i++) { // verification 0/1
          if (!(Rmessage.substring(p2 + i, p2 + i + 1) == "0" || Rmessage.substring(p2 + i, p2 + i + 1) == "1")) {
            flag = false;
          }
        }
        if (flag) {
          // Serial.println(F("mise a jour calendrier"));
          for (int i = 1; i < 32; i++) {
            calendrier[m][i] = Rmessage.substring(p2 + i, p2 + i + 1).toInt();
            // Serial.print(Rmessage.substring(p2+i,p2+i+1));
          }
          EnregistreCalendrier(); // Sauvegarde en SPIFFS
        }
      }
      if(!flag) {
        // printf("flag=%d,W=%d\n",flag,W);
        message += " erreur format ";
      }
    }
    if (flag) { // demande calendrier pour un mois donné
      if (smsserveur || !sms) {
        // si serveur reponse json  {"mois":12,"jour":[1,2,4,5,6 .. 31]}
        JsonDocument doc;
        doc["mois"] = m;
        JsonArray jour = doc["jour"].to<JsonArray>();
        for (int i = 1; i < 32; i++) {
          jour.add(calendrier[m][i]);
        }
        String jsonbidon;
        serializeJson(doc, jsonbidon);
        message += jsonbidon;
      }
      else {
        message += F("mois = ");
        message += m;
        message += fl;
        for (int i = 1; i < 32 ; i++) {
          message += calendrier[m][i];
          if ((i % 5)  == 0) message += " ";
          if ((i % 10) == 0) message += fl;
        }
      }
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage == F("CIRCULE")) {
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
    sendReply(Origine);
    // necessaire pour jour non circulé sur reception circule on lance FCV
    // si reception CIRCULE on ne lance pas FCV avant firstdecision 
    if (ok && firstdecision) {
      SignalVie();
    }
  }
  else if (Rmessage == F("NONCIRCULE")) {
    bool ok = false;
    /* demande passer en mode nonCirculé pour le jour courant,
      sans modification calendrier enregistré en SPIFFS 
      extinction Feux*/
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
    sendReply(Origine);
    if (ok && firstdecision) {
      // Seulement si déjà lancé apres première décision
      // sinon au lancement, on attend première décision
      Extinction();
      action_wakeup_reason(4);
    }
  }
  else if (Rmessage.indexOf(F("TEMPOWAKEUP")) == 0) { // Tempo wake up
    if ((Rmessage.indexOf(char(61))) == 11) {
      int i = Rmessage.substring(12).toInt();         //	durée
      if (i > 59 && i <= 36000) {                     // 1mn à 10H
        config.RepeatWakeUp = i;
        sauvConfig();															    // sauvegarde config
      }
    }
    message += F("Tempo repetition Wake up (s)=");
    message += config.RepeatWakeUp;
    sendReply(Origine);
  }
  else if (Rmessage == F("RST")) {               // demande RESET
    message += F("Le systeme va etre relance");  // apres envoie du SMS!
    message += fl;
    FlagReset = true;                            // reset prochaine boucle
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("CALIBRATION=")) == 0) {
    /* 	Mode calibration mesure tension
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
        stock en SPIFFS
        sort du mode calibration

        variables
        FlagCalibration true cal en cours, false par defaut
        Static P pin d'entrée
        static int tensionmemo memorisation de la premiere tension mesurée en calibration
        int CoeffTension = CoeffTensionDefaut 7000 par défaut
    */
    Sbidon = Rmessage.substring(12, 16); // texte apres =
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
        if(!Allume)digitalWrite(PinConvert, HIGH); // Alimentation du convertisseur 12/24V
        for (int i = 0; i < 5 ; i++) {
          read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V, PinLum); // lecture des adc
          Alarm.delay(100);
        }
        M = 4;
        P = Pin24V;
        coef = CoeffTension[3];
      }
      Serial.print("mode = "), Serial.print(M), Serial.println(Sbidon.substring(1, 2));
      FlagCalibration = true;

      coef = CoeffTensionDefaut;
      tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
      // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
      tensionmemo = tension;
    }
    else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 8000) {
      // si Calibration en cours et valeur entre 0 et 5000
      Serial.println(Sbidon.substring(0, 4));
      /* calcul nouveau coeff */
      coef = Sbidon.substring(0, 4).toFloat() / float(tensionmemo) * CoeffTensionDefaut;
      // Serial.print("Coeff Tension = "),Serial.println(coef);
      tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
      // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
      CoeffTension[M - 1] = coef;
      FlagCalibration = false;
      Recordcalib();														// sauvegarde en SPIFFS

      if (M == 4 && !Allume) {
        digitalWrite(PinConvert, LOW); // Arret du convertisseur 12/24V
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
      if(config.TypeBatt == 16) message += String(BattPBpct(tension, 6));
      if(config.TypeBatt == 24) message += String(BattLiFePopct(tension, 4));
      message += "%";
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(Id.substring(5, 9)) == 1) { // cherche CVXX
    if (Rmessage.indexOf("D") == 0) {
      Extinction(); // Violet 0, Blanc 0
      MajLog(Origine, "DCV");
    }
    else if (Rmessage.indexOf("F") == 0) {
      if(Feux < 5 || Feux == 7){ // si Carré fermé ne rien faire
        EffaceAlaCdeFBlc();
        Feux = 1;
        Allumage(); // Violet 1, Blanc 0
        MajLog(Origine, "FCV");
      }
    }
    else if(Rmessage.indexOf("O") == 0 || Rmessage.indexOf("M") == 0 || Rmessage.indexOf("S") == 0 || Rmessage.indexOf("V") == 0){
      if(FlagTqt_1){ // taquet ouvert
        if (Rmessage.indexOf("O") == 0) {
          EffaceAlaCdeFBlc();
          Feux = 2;
          Allumage(); // Violet 0, Blanc 1
          MajLog(Origine, "OCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        else if (Rmessage.indexOf("M") == 0) {
          EffaceAlaCdeFBlc();
          Feux = 3;
          Allumage(); // Violet 0, Blanc Manoeuvre Cli lent
          MajLog(Origine, "MCV");
          // if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        else if (Rmessage.indexOf("S") == 0) {
          EffaceAlaCdeFBlc();
          Feux = 4;
          Allumage(); // Violet 0, Blanc Secteur Cli rapide
          MajLog(Origine, "SCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        else if (Rmessage.indexOf("V") == 0) {
          EffaceAlaCdeFBlc();
          Feux = 7;
          Allumage(); // Violet Cli, Blanc 0
          MajLog(Origine, "VCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
      } else { // taquet fermé
        FlagDemande_Feux = true;
        Memo_Demande_Feux[0] = Origine;   // nom demandeur
        Memo_Demande_Feux[1] = Origine;   // num demandeur
        Memo_Demande_Feux[2] = Rmessage;  // demande d'origine
        Feux = 5; // Violet 1, Blanc 0
        MajLog(Origine, "CCV demande : " + Rmessage);
        // Serial.println("memo demande feux :");
        // Serial.println(Memo_Demande_Feux[0]);
        // Serial.println(Memo_Demande_Feux[1]);
        // Serial.println(Memo_Demande_Feux[2]);
      }
    }
    else {
      // message += "non reconnu" + fl;
    }
    if (Feux != 0) { // seulement si different de DCV, doublon DCV envoie automatiquement une reponse dans Extinction()
      envoieGroupeMessage(false,true); // envoie serveur
    }
    // evite de repondre 2 fois au serveur
    if (!smsserveur){
      generationMessage();
      sendReply(Origine); // reponse si pas serveur
    }
  }
  else if (Rmessage.indexOf(F("FBLCPWM")) == 0) {
    if (Rmessage.substring(7, 8) == "=") {
      int i = Rmessage.substring(8, Rmessage.length()).toInt();
      if (i > 4 && i < 101) {
        config.FBlcPWM = i;
        sauvConfig();
      }
    }
    // Allumage();
    message += "Blanc PWM =";
    message += config.FBlcPWM;
    message += "%";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("FVLTPWM")) == 0) {
    if (Rmessage.substring(7, 8) == "=") {
      int i = Rmessage.substring(8, Rmessage.length()).toInt();
      if (i > 4 && i < 101) {
        config.FVltPWM = i;
        sauvConfig();
      }
    }
    // Allumage();
    message += "Violet PWM =";
    message += config.FVltPWM;
    message += "%";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SLOWBLINKER")) == 0) {
    if (Rmessage.substring(11, 12) == "=") {
      int i = Rmessage.substring(12, Rmessage.length()).toInt();
      if (i > 199 && i < 2001) {
        config.SlowBlinker = i;
        sauvConfig();
      }
    }
    // Allumage();
    message += "SlowBlinker =";
    message += config.SlowBlinker;
    message += "ms";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("FASTBLINKER")) == 0) {
    if (Rmessage.substring(11, 12) == "=") {
      int i = Rmessage.substring(12, Rmessage.length()).toInt();
      if (i > 149 && i < 2001) {
        config.FastBlinker = i;
        sauvConfig();
      }
    }
    // Allumage();
    message += "FastBlinker =";
    message += config.FastBlinker;
    message += "ms";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("FASTRATER")) == 0) {
    if (Rmessage.substring(9, 10) == "=") {
      int i = Rmessage.substring(10, Rmessage.length()).toInt();
      if (i > 999 && i < 3001) {
        config.FastRater = i;
        sauvConfig();
      }
    }
    // Allumage();
    message += "FastRater =";
    message += config.FastRater;
    message += "ms";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("PARAM")) >= 0) {
    /*TPCF_CV99
19/10/2024 17:10:38
{"param":{"slowblinker":500,"fastblinker":150,"fastrater":1000,"debut":"08:00:00","fin":"19:00:00","autof":true,"tempoautof":3600,"fblcpwm":100,"fvltpwm":75,"lumauto":false},"lumlut":[100,90,80,70,60,50,40,30,20,10,10]}
*/
    //message param divisé en 2 trop long depasse long 1sms 160c
    bool erreur = false;
    // Serial.print("position X:"),Serial.println(Rmessage.substring(7, 8));
    if(Rmessage.substring(7, 8) == "1"){ // PARAM1
    // Serial.print("position ::"),Serial.println(Rmessage.substring(9, 10));
      if (Rmessage.substring(9, 10) == ":") {
        // json en reception sans lumlut
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, Rmessage);
        if(err){
          erreur = true;
        }
        else{
          // Serial.print(F("Deserialization succeeded"));
          JsonObject param = doc["PARAM1"];
          config.SlowBlinker = param["SLOWBLINKER"];
          config.FastBlinker = param["FASTBLINKER"];
          config.FastRater = param["FASTRATER"];
          config.DebutJour = Hhmmtohdec(param["DEBUT"]);
          config.FinJour = Hhmmtohdec(param["FIN"]);
          sauvConfig();
          Alarm.disable(FinJour);
          Alarm.write(FinJour,config.FinJour);
          // FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee);// init tempo
          Alarm.enable(FinJour);
          Alarm.disable(DebutJour);
          Alarm.write(DebutJour,config.DebutJour);
          // FinJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);// init tempo
          Alarm.enable(DebutJour);
        }
      }
      else{
        erreur = true;
      }
    }
    else if(Rmessage.substring(7, 8) == "2"){ // PARAM2
      if (Rmessage.substring(9, 10) == ":") {
        // json en reception sans lumlut
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, Rmessage);
        if(err){
          erreur = true;
        }
        else{
          // Serial.print(F("Deserialization succeeded"));
          JsonObject param  = doc["PARAM2"];
          config.LumAuto    = param["LUMAUTO"];
          config.FBlcPWM    = param["FBLCPWM"];
          config.FVltPWM    = param["FVLTPWM"];
          config.AutoF      = param["AUTOF"];
          config.TempoAutoF = param["TEMPOAUTOF"];
          sauvConfig();
        }
      }
    }
    if(!erreur){
      // ne fonctionne pas
      // const size_t capacity = JSON_ARRAY_SIZE(11) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(11);
      // https://arduinojson.org/v7/assistant/#/step1
      JsonDocument doc;
      JsonObject param = doc["param"].to<JsonObject>();
      param["slowblinker"] = config.SlowBlinker;
      param["fastblinker"] = config.FastBlinker;
      param["fastrater"] = config.FastRater;
      param["debut"] = Hdectohhmm(config.DebutJour);
      param["fin"] = Hdectohhmm(config.FinJour);
      param["autof"] = config.AutoF;
      param["tempoautof"] = config.TempoAutoF;
      param["fblcpwm"] = config.FBlcPWM;
      param["fvltpwm"] = config.FVltPWM;
      param["lumauto"] = config.LumAuto;

      JsonArray param_lumlut = doc["lumlut"].to<JsonArray>();
      for (int i = 0; i < 11; i++) {
        param_lumlut.add(TableLum[i][1]);
      }
      String jsonbidon;
      serializeJson(doc, jsonbidon);
      // serializeJson(doc, Serial);
      message += jsonbidon;
    }
    else{
      message += "erreur json";
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("E1ACTIVE")) == 0) {
    bool valid = false;
    if (Rmessage.substring(8, 9) == "=") {
      if (Rmessage.substring(9, 10) == "1") {
        if (!config.Ip1) {
          config.Ip1 = true;
          FlagTqt_1 = false;
          sauvConfig();
          valid = true;
          MajLog(Origine, Rmessage);
        }
      }
      else if (Rmessage.substring(9, 10) == "0") {
        if (config.Ip1) {
          config.Ip1 = false;
          FlagTqt_1 = true;
          sauvConfig();
          valid = true;
          MajLog(Origine, Rmessage);
        }
      }
      if (valid) {
        sauvConfig();															// sauvegarde config
      }
    }
    message += "Entree 1 ";
    if (config.Ip1) {
      message += "Active";
    }
    else {
      message += "InActive";
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("E2ACTIVE")) == 0) {
    bool valid = false;
    if (Rmessage.substring(8, 9) == "=") {
      if (Rmessage.substring(9, 10) == "1") {
        if (!config.Ip2) {
          config.Ip2 = true;
          sauvConfig();
          valid = true;
        }
      }
      else if (Rmessage.substring(9, 10) == "0") {
        if (config.Ip2) {
          config.Ip2 = false;
          sauvConfig();
          valid = true;
        }
      }
      if (valid) {
        sauvConfig();															// sauvegarde config
      }
    }
    message += "Entree 2 ";
    if (config.Ip2) {
    message += "Active";
    }
    else {
    message += "InActive";
    }
    message += fl;
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("UPLOADLOG")) == 0) {//upload log sur demande
    message += "Fonction non active";
    // message += F("lancement upload log");
    // message += fl;
    // MajLog(Origine, "upload log");// renseigne log
    // Serial.println(F("Starting..."));
    // bool reply = FTP_upload_function(filelog); // Upload fichier
    // Serial.println("The end... Response: " + String(reply));

    // if(reply == true){
    //   message += F("upload OK");
    //   SPIFFS.remove(filelog);          // efface fichier log
    //   MajLog(Origine, "");             // nouveau log
    //   MajLog(Origine, F("upload OK")); // renseigne nouveau log
    // } else {
    //   message += F("upload fail");
    //   MajLog(Origine, F("upload fail"));// renseigne log
    // }
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("COEFF")) == 0) {//Lecture/ecriture des coeff
    // COEFF=xxxx,xxxx,xxxx,xxxx
    if(Rmessage.indexOf(char(61)) == 5){ // =
      Sbidon = Rmessage.substring(6, Rmessage.length());
      Serial.println(Sbidon);
      int tempo[4] = {0,0,0,0};
      byte p1 = 0;
      byte p2 = 0;
      bool flag = true;
      for(int i = 0; i < 4; i++){          
        // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
        p2 = Sbidon.indexOf(char(44), p1 + 1); // ,
        tempo[i] = Sbidon.substring(p1,p2).toInt();
        if(tempo[i] < 0) flag = false;
        if(i!=3 && p2 == 255) flag = false;
        p1 = p2 + 1;          
        // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
      }
      if (flag){ // format OK
        for(int i = 0; i < 4; i++){
          CoeffTension[i] = tempo[i];
        }
        Recordcalib(); // enregistre en SPIFFS
      }
    }
    message += "Coeff calibration:" + fl;
    for(int i = 0; i < 4; i++){
      message += String(CoeffTension[i]);
      if(i < 3 ) message += ",";
    }
    Serial.println(message);
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("UPLOADCOEFF")) == 0) {//upload des coeff
    message += "Fonction non active";
    // message += F("lancement upload Coeff");
    // message += fl;
    // MajLog(Origine, "upload coeff");// renseigne log
    // Serial.println(F("Starting..."));
    // bool reply = FTP_upload_function(filecalibration); // Upload fichier
    // Serial.println("The end... Response: " + String(reply));

    // if(reply == true){
    //   message += F("upload OK");
    //   MajLog(Origine, F("upload Coeff OK"));// renseigne nouveau log
    // } else {
    //   message += F("upload fail");
    //   MajLog(Origine, F("upload Coeff fail"));// renseigne log
    // }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("FTPDATA") > -1) {
  // Parametres FTPDATA=Serveur:User:Pass:port
  // {"FTPDATA":{"serveur":"dd.org","user":"user","pass":"pass","port":00}}
  bool erreur = false;
  bool formatsms = false;
  if (Rmessage.indexOf(":") == 10) { // format json
    JsonDocument doc; // https://arduinojson.org/v7/assistant/#/step1
    DeserializationError err = deserializeJson(doc, Rmessage);
    if (err) {
      erreur = true;
    }
    else {
      JsonObject ftpdata = doc["FTPDATA"];
      strncpy(config.ftpServeur,  ftpdata["serveur"], 26);
      strncpy(config.ftpUser,     ftpdata["user"],    9);
      strncpy(config.ftpPass,     ftpdata["pass"],    16);
      config.ftpPort         =    ftpdata["port"];
      sauvConfig();													// sauvegarde config
    }
  }
  else if ((Rmessage.indexOf(char(61))) == 7) { // format sms
    formatsms = true;
    byte w = Rmessage.indexOf(":");
    byte x = Rmessage.indexOf(":", w + 1);
    byte y = Rmessage.indexOf(":", x + 1);
    byte zz = Rmessage.length();
    if (Rmessage.substring(y + 1, zz).toInt() > 0) { // Port > 0
      if ((w - 7) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16) {
        Sbidon = Rmessage.substring(7, w);
        Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
        Sbidon = Rmessage.substring(w + 1, x);
        Sbidon.toCharArray(config.ftpUser, (Sbidon.length() + 1));
        Sbidon = Rmessage.substring(x + 1, y);
        Sbidon.toCharArray(config.ftpPass, (Sbidon.length() + 1));
        config.ftpPort = Rmessage.substring(y + 1, zz).toInt();
        sauvConfig();													// sauvegarde config
      }
      else {
        erreur = true;
      }
    } else {
      erreur = true;
    }
  }
  if (!erreur) {
    if (formatsms) {
      message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
      message += fl;
      message += F("Parametres FTP :");
      message += fl;
      message += "Serveur:" + String(config.ftpServeur) + fl;
      message += "User:"    + String(config.ftpUser) + fl;
      message += "Pass:"    + String(config.ftpPass) + fl;
      message += "Port:"    + String(config.ftpPort) + fl;
    }
    else {
      JsonDocument doc;
      JsonObject FTPDATA = doc["FTPDATA"].to<JsonObject>();
      FTPDATA["serveur"] = config.ftpServeur;
      FTPDATA["user"]    = config.ftpUser;
      FTPDATA["pass"]    = config.ftpPass;
      FTPDATA["port"]    = config.ftpPort;
      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    }
  }
  else {
    message += "Erreur format";
    message += fl;
  }
  sendReply(Origine);
}
  else if (Rmessage.indexOf("FTPSERVEUR") == 0) { // Serveur FTP
    // case sensitive
    // FTPSERVEUR=xyz.org
    if (Rmessage.indexOf(char(61)) == 10) {
      Sbidon = Rmessage.substring(11);
      Serial.print("ftpserveur:"),Serial.print(Sbidon);
      Serial.print(" ,"), Serial.println(Sbidon.length());
      Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
      sauvConfig();
    }
    message += F("FTPserveur =");
    message += String(config.ftpServeur);
    message += F("\n au prochain demarrage");
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("MQTTDATA") > -1) {
    // Parametres MQTTDATA=serveur:user:pass:port
    // {"MQTTDATA":{"serveur":"xxxx.org","user":"uuu","pass":"passpass","port":9999}}

    bool erreur = false;
    // bool formatsms = false;
    if (Rmessage.indexOf(":") == 11) { // format json
      JsonDocument doc; //https://arduinojson.org/v7/assistant/#/step1
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject mqttdata = doc["MQTTDATA"];
        strncpy(config.mqttServer,     mqttdata["serveur"], 26);
        strncpy(config.mqttUserName,   mqttdata["user"]   , 11);
        strncpy(config.mqttPass,       mqttdata["pass"]   , 16);
        config.mqttPort            =   mqttdata["port"];
        sauvConfig();
      }
    }
    
    if (!erreur) {
      JsonDocument doc;
      JsonObject MQTTDATA = doc["MQTTDATA"].to<JsonObject>();
      MQTTDATA["serveur"] = config.mqttServer;
      MQTTDATA["user"]    = config.mqttUserName;
      MQTTDATA["pass"]    = config.mqttPass;
      MQTTDATA["port"]    = config.mqttPort;
      
      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("TOPIC") > -1) { // MQTT Topic
    // Parametres TOPIC="sendTopic":"sendtopic0,sendtopic1","recvTopic":"recvtopic0,recvtopic1"
    //"{"TOPIC":{"sendTopic0": "sendtopic0", "sendTopic1": "sendtopic1","recvTopic0":"recvtopic0","recvTopic1":"recvtopic1"}}"
    
    bool erreur = false;
    if (Rmessage.indexOf(":") == 8) { // format json
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject TOPIC = doc["TOPIC"];
        strncpy(config.sendTopic[0],TOPIC["sendTopic0"],12);
        strncpy(config.sendTopic[1],TOPIC["sendTopic1"],12);
        strncpy(config.recvTopic[0],TOPIC["recvTopic0"],12);
        strncpy(config.recvTopic[1],TOPIC["recvTopic1"],12);
        sauvConfig();
        copie_Topic();
      }
    }
    if (!erreur){
      JsonDocument doc;
      JsonObject TOPIC = doc["TOPIC"].to<JsonObject>();
      
      TOPIC["sendTopic0"] = config.sendTopic[0];
      TOPIC["sendTopic1"] = config.sendTopic[1];
      TOPIC["recvTopic0"] = config.recvTopic[0];
      TOPIC["recvTopic1"] = config.recvTopic[1];

      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("MQTTSERVEUR") == 0) { // Serveur MQTT
    // case sensitive
    // MQTTSERVEUR=abcd.org
    if (Rmessage.indexOf(char(61)) == 11) {
      Sbidon = Rmessage.substring(12);
      Serial.print("mqttserveur:"),Serial.print(Sbidon);
      Serial.print(" ,"), Serial.println(Sbidon.length());
      Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
      sauvConfig();
    }
    message += F("MQTTserveur =");
    message += String(config.mqttServer);
    message += F("\n au prochain demarrage");
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("GPRSDATA")) > -1) {
    // Parametres GPRSDATA = "APN":"user":"pass"
    // GPRSDATA="sl2sfr":"":""
    // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
    bool erreur = false;
    bool formatsms = false;
    if (Rmessage.indexOf(":") == 11) { // format json
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject gprsdata = doc["GPRSDATA"];
        strncpy(config.apn, gprsdata["apn"], 11);
        strncpy(config.gprsUser, gprsdata["user"], 11);
        strncpy(config.gprsPass, gprsdata["pass"], 11);
        // Serial.print("apn length:"),Serial.println(strlen(gprsdata["apn"]));
        // Serial.print("apn:"),Serial.println(config.apn);
        // Serial.print("user:"),Serial.println(config.gprsUser);
        // Serial.print("pass:"),Serial.println(config.gprsPass);
        sauvConfig();													// sauvegarde config
      }
    }
    else if ((Rmessage.indexOf(char(61))) == 8) { // format sms
      formatsms = true;
      byte cpt = 0;
      byte i = 9;
      do { // compte nombre de " doit etre =6
        i = Rmessage.indexOf('"', i + 1);
        cpt ++;
      } while (i <= Rmessage.length());
      Serial.print("nombre de \" :"), Serial.println(cpt);
      if (cpt == 6) {
        byte x = Rmessage.indexOf(':');
        byte y = Rmessage.indexOf(':', x + 1);
        byte z = Rmessage.lastIndexOf('"');
        // Serial.printf("%d:%d:%d\n",x,y,z);
        // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
        if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
          Sbidon = Rmessage.substring(10, x - 1);
          Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(x + 1 + 1 , y - 1);
          Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(y + 1 + 1, z);
          Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

          // Serial.print("apn:"),Serial.println(config.apn);
          // Serial.print("user:"),Serial.println(config.gprsUser);
          // Serial.print("pass:"),Serial.println(config.gprsPass);

          sauvConfig();													// sauvegarde config
        }
        else {
          erreur = true;
        }
      }
      else {
        erreur = true;
      }
    }
    if (!erreur) {
      if (formatsms) {
        message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant" + fl;
        message += "Parametres GPRS \"apn\":\"user\":\"pass\"";
        message += fl + "\"";
        message += String(config.apn);
        message += "\":\"";
        message += String(config.gprsUser);
        message += "\":\"";
        message += String(config.gprsPass);
        message += "\"" + fl;
      }
      else {
        JsonDocument doc;
        JsonObject gprsdata = doc["GPRSDATA"].to<JsonObject>();
        gprsdata["apn"]  = config.apn;
        gprsdata["user"] = config.gprsUser;
        gprsdata["pass"] = config.gprsPass;
        Sbidon = "";
        serializeJson(doc, Sbidon);
        message += Sbidon;
        message += fl;
      }
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage == "RSTALACDEFBLC") {
    // demande reset Alarme Cde Feu Blanc
    EffaceAlaCdeFBlc();
    message += "Reset Alarme en cours";
    sendReply(Origine);
  }
  else if (Rmessage == "VIDELOG"){
    SPIFFS.remove(filelog);
    FileLogOnce = false;
    message += "Effacement fichier log";
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("AUTOUPLOAD") == 0){ // Auto upload log vers serveur FTP
    message += "Fonction non active";
    // if (Rmessage.indexOf(char(61)) == 10) {
    //   byte c = Rmessage.substring(11).toInt();
    //   if(c==0 || c==1){
    //     config.autoupload = c;
    //     sauvConfig();
    //   }
    // }
    // message += "Autoupload:";
    // message += String(config.autoupload);
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("CPTALATRCK")) == 0 || Rmessage.indexOf(F("CPTALA")) == 0) { // Compteur Ala avant Flag
      if (Rmessage.indexOf(char(61)) == 10) {
        int c = Rmessage.substring(11).toInt();
        if (c > 1 && c < 501) {
          config.cptAla = c;
          sauvConfig();
        }
      }
      message += F("Cpt Ala Tracker (x10s)=");
      message += String(config.cptAla);
      message += fl;
      sendReply(Origine);
    }
  else if (Rmessage.indexOf(F("SETNETWORKMODE")) >= 0) {// Set Prefered network Mode
      if(Rmessage.indexOf(char(61)) == 14){
        int mode = Rmessage.substring(15).toInt();
        if(mode == 2 || mode == 13 || mode == 38 || mode == 51){
          modem.setNetworkMode(mode);
          delay(1000);
        }
      }
      message += String(modem.send_AT(F("+CNMP?")));
      sendReply(Origine);
    }
  else if (Rmessage.indexOf(F("SENDAT")) == 0){
    // envoie commande AT au SIM7000
    // ex: SENDAT=AT+CCLK="23/07/19,10:00:20+04" mise à l'heure
    // attention DANGEREUX pas de verification!
    if (Rmessage.indexOf(char(61)) == 6) {
      String CdeAT = Rmessage.substring(7, Rmessage.length());
      String reply = sendAT(CdeAT,"OK","ERROR",1000);
      // Serial.print("reponse: "),Serial.println(reply);
      message += String(reply);
      sendReply(Origine);
    }
  }
  else if (Rmessage.indexOf(F("MODEMINFO")) == 0){
    // Get Modem Info
    message += modem.getModemInfo();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("CPTRESETMODEM")) == 0){
    // Demande nombre de reset modem
    message += F("Compteur reset Modem : ");
    message += String(NbrResetModem);
    sendReply(Origine);
  }else if (Rmessage.indexOf(F("NETWORKHISTO")) == 0){
    // Demande historique Changement etat reseau
    message_Monitoring_Reseau();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TYPEBATT")) == 0){ // Type Batterie
    if (Rmessage.indexOf(char(61)) == 8) {
      int type = Rmessage.substring(9, Rmessage.length()).toInt();
      if(type == 16 || type == 24){
        config.TypeBatt = type;
        sauvConfig();													// sauvegarde config
      }
    }
    message += "Type Batterie:" + fl;
    if(config.TypeBatt == 16) message += "Pb 12V";
    if(config.TypeBatt == 24) message += "LiFePO 12.8V";
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("AUTORISATIONSMS") == 0) {
    // Autorisation envoie SMS
    if (Rmessage.indexOf(char(61)) == 15) {
      int i = Rmessage.substring(16).toInt();
      if (i == 0){
        config.sendSMS = 0;
      } else if (i == 1){
        config.sendSMS = 1;
      }
      sauvConfig();													// sauvegarde config
      Sbidon = F("Autorisation SMS=");
      Sbidon += String(config.sendSMS);
      MajLog(Origine, Sbidon);// renseigne log
    }
    message += "Autorisation SMS : ";
    message += String(config.sendSMS);
    sendReply(Origine);
  }
  //**************************************
  else {
    message += F("Commande non reconnue ?");		//"Commande non reconnue ?"
    sendReply(Origine);
  }
}
//---------------------------------------------------------------------------
// determine si un message apparition/disparition Alarme doit etre envoyé
void envoie_alarme() {
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
  if (FlagAlarmeCdeFBlc != FlagLastAlarmeCdeFBlc) {
    SendEtat = true;
    MajLog(F("Auto"), "Alarme Cde FBlc");
    FlagLastAlarmeCdeFBlc = FlagAlarmeCdeFBlc;
  }
  if (FlagAlarmeGprs != FlagLastAlarmeGprs) {
    SendEtat = true;
    FlagLastAlarmeGprs = FlagAlarmeGprs;
  }
  if (FlagAlarmeMQTT != FlagLastAlarmeMQTT) {
    SendEtat = true;
    FlagLastAlarmeMQTT = FlagAlarmeMQTT;
  }
  if (SendEtat) { 						  // si envoie Etat demandé
    envoieGroupeMessage(false, true);	 // pasVie, Serveur
    envoieGroupeMessage(false, false); // pasVie, User
    SendEtat = false;					  // efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeMessage(bool vie, bool Serveur) {
  generationMessage();
  if(vie){
    // message += F("Reset modem : ");
    // message += String(NbrResetModem);
    // message += fl;
    // message_Monitoring_Reseau();
  }
  if(config.sendSMS){
  // A Finir 
    // for (byte Index = 1; Index < 10; Index++) {		// Balayage du PB
    //   Phone = {"",""};      
    //   if(modem.readPhonebookEntry(&Phone, Index)){
    //     // Serial.print("groupe:"),Serial.println(grp);
    //     // Serial.print("Index:"),Serial.println(Index);
    //     // Serial.print("PB name:"),Serial.println(Phone.text);
    //     if(Phone.number.length() > 0){
    //       if (grp == 3){ // Serveur
    //         if(Index == 1 && config.messageMode == 1){ // Serveur et MQTT
    //           Envoyer_MQTT();
    //         } else { // SMS
    //           sendReply(Phone.number, true);
    //         }
    //         break; // Sortir, Serveur seulement
    //       } else if (grp == 0){ // tous SMS
    //         sendReply(Phone.number, true);
    //       } else if (grp == 1) { // liste restreinte seulement
    //         for (byte Index2 = 1; Index2 <10;Index2 ++){		// Balayage des Num Tel Autorisés=1 dans Phone Book
    //           // Serial.print(Index),Serial.print(","),Serial.println(Phone.number);            
    //           if (config.Pos_Pn_PB[Index2] == 1){          
    //             sendReply(Phone.number, true);
    //           }
    //         }
    //       }
    //     } else {
    //       break;
    //     }
    //   }
    // }
  }
  Envoyer_MQTT(Serveur);
}
//---------------------------------------------------------------------------
void generationMessage() {
  messageId();
  if (FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarme24V || FlagAlarmeCdeFBlc
      || FlagAlarmeGprs || FlagAlarmeMQTT) {
    message += F("--KO--------KO--");
  }
  else {
    message += F("-------OK-------");
  }
  message += fl;
  
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
    case 5: // Taquet fermé + Violet 1
      message += "C";
      break;
    case 6: // Taquet ouvert + Violet 0 + Blanc 0
      message += "Z";
      break;
    case 7: // Violet Cli + Blanc 0
      message += "V";
      break;
  }
  if(config.Ip2){
    if(FlagTqt_2){ // Taquet ouvert
      message += "O";
    } else {
      message += "F";
    }
  }
  message += String(Id.substring(5, 9));// CVXX
  message += fl;
  message += F("Batterie : ");
  if (!FlagAlarmeTension) {
    message += F("OK, ");
    if(config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if(config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += "%" + fl;
  }
  else {
    message += F("Alarme, ");
    if(config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if(config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += "%";
    message += fl;
    message += F("V USB =");
    message += String(float(VUSB / 1000.0)) + fl;
  }
  if (FlagAlarme24V) {
    message += F("Alarme 24V = ");
    message += String(float(Tension24 / 100.0)) + "V" + fl;
  }
  
  if ((calendrier[month()][day()] ^ flagCircule)) {
    message += "Jour Circule" + fl;
  }
  else {
    message += "Jour Non Circule" + fl;
  }
  if(FlagAlarmeCdeFBlc){
    message += "Defaut Cde Feu Blanc" + fl;
  }
  if(config.sendSMS){
    message += F("Gprs ");
    if (FlagAlarmeGprs) {
      message += F("KO");
    } else {
      message += F("OK");
    }
    message += fl;

    message += F("Mqtt ");
    if (FlagAlarmeMQTT) {
      message += F("KO");
    } else {
      message += F("OK");
    }
    message += fl;
  }  
}
//---------------------------------------------------------------------------
// Envoyer une réponse
// Origine = Local, BLE, SMS, MQTTS (serveur), MQTTU (user)
void sendReply(String Origine) {
  if (gsm) {
    if (Origine == "MQTTS"){ // reponse MQTT      
      Envoyer_MQTT(true); // Serveur
    }
    else if (Origine == "MQTTU"){ // reponse MQTT
      Envoyer_MQTT(false); // User
    }
    else if (Origine == "SMS"){
      if (config.sendSMS){
        // A finir
      }
    }
  }
  Serial.println(F("****************************"));
  Serial.println(message);
  Serial.println(F("****************************"));
}
//---------------------------------------------------------------------------
// Envoyer message en MQTT
// destinataire = true Serveur, false à User
void Envoyer_MQTT(bool dest){
  Serial.println("Sending MQTT len:");
  Serial.println(message.length());
  Serial.print(F("to:"));
  Serial.print(dest ? config.sendTopic[0] : config.sendTopic[1]);
  Serial.print(F(":"));
  if(dest){ // message Serveur
    if (mqttClient.publish(config.sendTopic[0], message.c_str()), true){ // Serveur
      AlarmeMQTT = false;
      Serial.println(F("OK"));
    } else {
      Serial.println(F("KO"));
      AlarmeMQTT = true;
    }
  } else { // message user
    if(mqttClient.publish(config.sendTopic[1], message.c_str()), true){ // User
      AlarmeMQTT = false;
      Serial.println(F("OK"));
    } else {
      Serial.println(F("KO"));
      AlarmeMQTT = true;
    }
  }
}
//---------------------------------------------------------------------------
void MajHeure(bool force) {
  // force = true, force mise à l'heure systeme sur heure modem, meme si defaut NTP
  Monitoring_Reseau();
  static bool First = true;
  if (gsm) {
    Serial.print(F("Mise a l'heure reguliere !, "));
    Serial.println(First);
    if (First) {															  // premiere fois apres le lancement
      SyncHeureModem(config.hete*4, true);
      readmodemtime();	// lire l'heure du modem
      setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
      if(!HeureEte()){
        // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
        SyncHeureModem(config.hhiver*4, true);
        readmodemtime();	// lire l'heure du modem
        setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
      }
      First = false;
    }
    else {
      // calcul décalage entre H sys et H reseau en s
      // resynchroniser H modem avec reseau
      if(HeureEte()){
        // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hete*4));
        if(!SyncHeureModem(config.hete*4, false)){
          if(!force)return; // sortie sans mise à l'heure, continue si forcé
        }
      } else {
        // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
        if(!SyncHeureModem(config.hhiver*4, false)){
          if(!force)return; // sortie sans mise à l'heure, continue si forcé
        }
      }
      readmodemtime();	// lire l'heure du modem
      //  calcul décalage entre H sys et H reseau en s
      int ecart = (N_H - hour()) * 3600;
      ecart += (N_m - minute()) * 60;
      ecart += N_S - second();
      // ecart += 10;
      Serial.print(F("Ecart s= ")), Serial.println(ecart);
      if (abs(ecart) > 5) {
        Alarm.disable(loopPrincipale);
        Alarm.disable(DebutJour);
        Alarm.disable(FinJour);
        Alarm.disable(Auto_F);

        readmodemtime();	// mise à l'heure de l'Arduino
        setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino

        Alarm.enable(loopPrincipale);
        Alarm.enable(DebutJour);
        Alarm.enable(FinJour);
        if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
      }
    }
  }
  displayTime(0);
  timesstatus();
  AIntru_HeureActuelle();
}
//---------------------------------------------------------------------------
// Htarget Heure de reveil visée
long DureeSleep(long Htarget) {
  /* calcul durée entre maintenant et Htarget */
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
// Convertir Heure en s
long HActuelledec() {
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes
  return Heureactuelle;
}
//---------------------------------------------------------------------------
// Action signal vie
void SignalVie() {
  Serial.println(F("Signal vie"));
  if (gsm) {
    MajHeure();
    modem.deleteSmsMessage(0,4);// au cas ou, efface tous les SMS envoyé/reçu
  }

  if ((calendrier[month()][day()] ^ flagCircule) && jour) { // jour circulé
    // 11 jour pour cas lancement de nuit pas d'allumage
    Sbidon = F("Jour circule ou demande circulation");
    Serial.println(Sbidon);
    MajLog(F("Auto"), Sbidon);
    if(!FlagTqt_1){// taquet fermé
      Feux = 5;
      MajLog("Auto", "CCV");
    } else {
      Feux = 1;
      MajLog("Auto", "FCV");
    }
    Allumage(); // Violet 1, Blanc 0
  }
  envoieGroupeMessage(true,true);  // pasVie,Serveur
  envoieGroupeMessage(true,false); // pasVie,User
  action_wakeup_reason(4);
}
//---------------------------------------------------------------------------
// mise forme date/time
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
// Liste directory
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
// Lire le fichier calendrier
void readFileCalendrier(fs::FS &fs, const char * path) {
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
// Append file
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
// Read Config
void readConfig(){
  Serial.printf("Reading file: %s\r\n", fileconfig);

  File file = SPIFFS.open(fileconfig);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  file.read((byte *)&config, sizeof(config));
  file.close();
}
//---------------------------------------------------------------------------
// Sauvegarde Config
void sauvConfig(){
  Serial.printf("Writing file: %s\r\n", fileconfig);

  File file = SPIFFS.open(fileconfig, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.write((byte *)&config, sizeof(config))){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
//---------------------------------------------------------------------------
// Read File
void readFile(fs::FS &fs, const char * path){ // pas utilisé ici
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}
//---------------------------------------------------------------------------
// Write File
void writeFile(fs::FS &fs, const char * path, const char * message){ // pas utilisé ici
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
//---------------------------------------------------------------------------
// mise à jour fichier log en SPIFFS
void MajLog(String Id, String Raison) {
  if(SPIFFS.exists(filelog)){
    /* verification de la taille du fichier */
    File f = SPIFFS.open(filelog, "r");
    Serial.print(F("Taille fichier log = ")), Serial.println(f.size());
    // Serial.print(Id),Serial.print(","),Serial.println(Raison);
    if (f.size() > 150000 && !FileLogOnce) {
      /* si trop grand on efface */
      FileLogOnce = true;
      messageId();
      message += F("KO Fichier log presque plein\n");
      message += String(f.size());
      message += F("\nFichier sera efface a 300000");
      if (gsm) {
        sendReply("MQTTU"); // message U
        sendReply("MQTTS"); // message S
      }
    }
    else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
      messageId();
      message += F("KO Fichier log plein\n");
      message += String(f.size());
      if(config.autoupload){
        message += F("\nFichier upload vers serveur ");
        if(FTP_upload_function(filelog)){
          message += ("OK");
        } else {
          message += ("KO");
        }
      } else {
        message += F("\nFichier efface");
      }
      if (gsm) {
        sendReply("MQTTU"); // message U
        sendReply("MQTTS"); // message S
      }
      f.close();
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
  else{ // fichier n'existe pas, création fichier avec première ligne date et Id
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d;", day(), month(), year(), hour(), minute(), second());
    strcat(Cbidon,config.Idchar);
    strcat(Cbidon,fl.c_str());
    appendFile(SPIFFS, filelog, Cbidon);
    Serial.print("nouveau fichier log:"),Serial.println(Cbidon);
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
    Id = ";" + Id + ";";
    Raison += "\n";
    strcat(Cbidon, Id.c_str());
    strcat(Cbidon, Raison.c_str());
    appendFile(SPIFFS, filelog, Cbidon);
  }
}
//---------------------------------------------------------------------------
// remplace le calendrier
void EnregistreCalendrier() {

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
// Enregistrer le fichier lumlut
void EnregistreLumLUT() {
  SPIFFS.remove(filelumlut);
  char bid[9];
  for (int i = 0; i < 11; i++) {
    sprintf(bid, "%d,%d\n", TableLum[i][0], TableLum[i][1]);
    appendFile(SPIFFS, filelumlut, bid);
  }
}
//---------------------------------------------------------------------------
// Ouvrir le fichier lumlut
void OuvrirLumLUT() {
  if (SPIFFS.exists(filelumlut)) {
    File f = SPIFFS.open(filelumlut, "r");
    for (int i = 0; i < 11; i++) { //Read 11 lignes
      String s = f.readStringUntil('\n');
      int pos = s.indexOf(",");
      TableLum[i][0] = s.substring(0, pos).toInt();
      TableLum[i][1] = s.substring(pos + 1, s.length() ).toInt();
    }
    f.close();
  }
  else {
    Serial.println("Fichier LumLUT n'existe pas, creation val par defaut");
    char bid[9];
    for (int i = 0; i < 11; i++) {
      int v2;
      int v1 = 100 - i * 10;
      if (i < 10) {
        v2 = v1;
      }
      else {
        v2 = 10;
      }
      sprintf(bid, "%d,%d\n", v1, v2);
      appendFile(SPIFFS, filelumlut, bid);
      TableLum[i][0] = v1;
      TableLum[i][1] = v2;
    }
  }
  for (int i = 0; i < 11 ; i++) {
    Serial.print(TableLum[i][0]), Serial.print(","), Serial.println(TableLum[i][1]);
  }
}
//---------------------------------------------------------------------------
// retourne la valeur lut en fonction de lum actuelle
int lumlut(int l) {
  for (int i = 0; i < 11; i++) {
    if (l >= TableLum[i][0]) {
      // Serial.printf("%s%d,%d\n","lumlut=",l,TableLum[i][1]);
      return TableLum[i][1];
    }
  }
  return 0;
}
//---------------------------------------------------------------------------
// Ouvrir le fichier de calendrier
void OuvrirCalendrier() {

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
  readFileCalendrier(SPIFFS, filecalendrier);

  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Serial.print(calendrier[m][j]), Serial.print(char(44));
    }
    Serial.println();
  }
  listDir(SPIFFS, "/", 0);

}
//---------------------------------------------------------------------------
// fin de journée retour deep sleep
void FinJournee() {
  jour = false;
  flagCircule = false;
  FirstWakeup = true;
  digitalWrite(PinAlimLum , LOW); // couper alimentation LDR
  if (Allume)Extinction();
  Serial.println(F("Fin de journee retour sleep"));
  TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);// xx mn avant
  // calculTimeSleep();
  Sbidon  = F("FinJour, sleep for ");
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog(F("Auto"), Sbidon);
  DebutSleep();
}
//---------------------------------------------------------------------------
// Print fichier de config
void PrintConfig() {
  Serial.print(F("Version = "))                 , Serial.println(ver);
  Serial.print(F("ID = "))                      , Serial.println(config.Idchar);
  Serial.print(F("magic = "))                   , Serial.println(config.magic);
  Serial.print(F("Debut Jour = "))              , Serial.println(config.DebutJour);
  Serial.print(F("Fin jour = "))                , Serial.println(config.FinJour);
  Serial.print(F("T anticipation Wakeup = "))   , Serial.println(config.anticip);
  Serial.print(F("Tempo repetition Wake up (s)= ")), Serial.println(config.RepeatWakeUp);
  Serial.print(F("Time Out Wifi (s)= "))        , Serial.println(config.timeoutWifi);
  Serial.print(F("Entrée Externe 1 Active = ")) , Serial.println(config.Ip1);
  Serial.print(F("Entrée Externe 2 Active = ")) , Serial.println(config.Ip2);
  Serial.print(F("Vitesse SlowBlinker = "))     , Serial.println(config.SlowBlinker);
  Serial.print(F("Vitesse FastBlinker = "))     , Serial.println(config.FastBlinker);
  Serial.print(F("Vitesse RepetFastBlinker = ")), Serial.println(config.FastRater);
  Serial.print(F("PWM Blanc = "))               , Serial.println(config.FBlcPWM);
  Serial.print(F("PWM Violet = "))              , Serial.println(config.FVltPWM);
  Serial.print(F("Luminosité Auto = "))         , Serial.println(config.LumAuto);
  Serial.print("Auto F si O/S = ")              , Serial.println(config.AutoF);
  Serial.print("Tempo Auto (s) = ")             , Serial.println(config.TempoAutoF);
  Serial.print("Type Batterie = ");
  if(config.TypeBatt == 16) Serial.println(F("Pb 12V 6elts"));
  if(config.TypeBatt == 24) Serial.println(F("LiFePO 12.8V 4elts"));
  Serial.print(F("GPRS APN = "))                , Serial.println(config.apn);
  Serial.print(F("GPRS user = "))               , Serial.println(config.gprsUser);
  Serial.print(F("GPRS pass = "))               , Serial.println(config.gprsPass);
  Serial.print(F("ftp serveur = "))             , Serial.println(config.ftpServeur);
  Serial.print(F("ftp port = "))                , Serial.println(config.ftpPort);
  Serial.print(F("ftp user = "))                , Serial.println(config.ftpUser);
  Serial.print(F("ftp pass = "))                , Serial.println(config.ftpPass);
  Serial.print(F("mqtt serveur = "))            , Serial.println(config.mqttServer);
  Serial.print(F("mqtt port = "))               , Serial.println(config.mqttPort);
  Serial.print(F("mqtt username = "))           , Serial.println(config.mqttUserName);
  Serial.print(F("mqtt pass = "))               , Serial.println(config.mqttPass);
  Serial.print(F("sendTopic = "))               , Serial.println(config.sendTopic[0]);
  Serial.print(F("sendTopic = "))               , Serial.println(config.sendTopic[1]);
  Serial.print(F("recvTopic = "))               , Serial.println(config.recvTopic[0]);
  Serial.print(F("recvTopic = "))               , Serial.println(config.recvTopic[1]);
  Serial.print(F("Send SMS autorisation = "))   , Serial.println(config.sendSMS);
  Serial.print(F("declage Heure ete = "))       , Serial.println(config.hete);
  Serial.print(F("declage Heure hiver = "))     , Serial.println(config.hhiver);
  Serial.print(F("autoupload = "))              , Serial.println(config.autoupload);
}
//---------------------------------------------------------------------------
// Connexion Wifi
void ConnexionWifi(char* ssid, char* pwd, String origine) {

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
    server.on("/LumLUT",   LumLUTPage);
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
  sendReply(origine);

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
  }
  WifiOff();
}
//---------------------------------------------------------------------------
// Arret du Wifi
void WifiOff() {
  Serial.println(F("Wifi off"));
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();
  delay(1000);// imperatif
  ResetHard();
} 
//---------------------------------------------------------------------------
// Reset hard de ESP32
void ResetHard() {
  // GPIO13 to RS reset hard
  Serial.println("Reset Hard");
  delay(100);// imperatif
  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
  // normalement on n'arrive jamais là
  delay(100);
  ESP.restart();
}
//---------------------------------------------------------------------------
// calcul moyenne 10 mesures consécutives
int moyenneAnalogique(int Pin) {
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    // Alarm.delay(1);
    moyenne += analogRead(Pin);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
// Lecture fichier calibration
void OuvrirFichierCalibration() {

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
// Enregistrement fichier calibration en SPIFFS
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
// convert heure decimale en hh:mm:ss
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
// convert heure hh:mm:ss en decimale
long Hhmmtohdec(String h){
  int H = h.substring(0,2).toInt();
  int M = h.substring(3,5).toInt();
  int S = h.substring(6,8).toInt();
  long hms = H*3600 + M*60 + S;
  return hms;
}
//---------------------------------------------------------------------------
// Heure actuelle jour/nuit
void AIntru_HeureActuelle() {
  long Heureactuelle = HActuelledec();

  if (config.FinJour > config.DebutJour) {
    if ((Heureactuelle > config.FinJour && Heureactuelle > config.DebutJour)
        || (Heureactuelle < config.FinJour && Heureactuelle < config.DebutJour)) {
      // Nuit
      jour = false;
    }
    else {	// Jour
      jour = true;
    }
  }
  else {
    if (Heureactuelle > config.FinJour && Heureactuelle < config.DebutJour) {
      // Nuit
      jour = false;
    }
    else {	// Jour
      jour = true;
    }
  }
}
//---------------------------------------------------------------------------
// lance le mode sleep
void DebutSleep() {

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.print(F("Setup ESP32 to sleep for "));
  print_uint64_t(TIME_TO_SLEEP);
  Serial.print(F("s ;"));
  Serial.println(Hdectohhmm(TIME_TO_SLEEP));
  Serial.flush();

  if (TIME_TO_SLEEP == 1) {
    Serial.println(F("pas de sleep on continue"));
    return;
  }
  //Go to sleep now
  Serial.println(F("Going to sleep now"));

  byte i = 0;
  if (gsm) {
    // mqttClient.disconnect(); // ne pas faire disconnect pour garder session active
    delay(1000);
    while (!modem.poweroff()) { // Power off
      Alarm.delay(100);
      if (i++ > 10) break;
    }
    Serial.print("power off:"),Serial.println(i);
  }
  Serial.flush();
  esp_deep_sleep_start();
  delay(100);

  Serial.println(F("This will never be printed"));
  Serial.flush();
}
//---------------------------------------------------------------------------
// action en fonction du wake up
void action_wakeup_reason(byte wr) {
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
      if (FirstWakeup) { // premier wake up du jour avant DebutJour
        // ne rien faire, attendre DebutJour
        FirstWakeup = false;
        if (HActuelledec() > config.DebutJour) {
          // premier lancement en journée
          SignalVie();
        }
        break;
      }
      if ((calendrier[month()][day()] ^ flagCircule) && jour) { // jour circulé & jour
        Sbidon = F("Jour circule ou demande circulation");
        Serial.println(Sbidon);
        // MajLog(F("Auto"), Sbidon);
        // Feux = 1;
        // Allumage(); // Violet 1, Blanc 0
        // MajLog("Auto", "FCV");
        // envoieGroupeMessage(0, 0);
      }
      else { // non circulé
        Sbidon = F("Jour noncircule ou nuit");
        Serial.println(Sbidon);
        MajLog(F("Auto"), Sbidon);
        calculTimeSleep();
        if (TIME_TO_SLEEP <= config.anticip) { // on continue sans sleep attente finjour
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
// calcul durée de sleep
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

  Sbidon = F("lance timer : ");
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog(F("Auto"), Sbidon);
}
//---------------------------------------------------------------------------
// Récupere raison du wake up
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
// Efface SMS
void EffaceSMS(int index) {
  bool err;
  byte n = 0;
  do {
    err = modem.deleteSmsMessage(index,0);
    n ++;
    Serial.print(F("resultat del Sms "));	Serial.println(err);
    if (n > 10) { // on efface tous si echec
      err = modem.deleteSmsMessage(index,4);
      Serial.print(F("resultat delall Sms "));	Serial.println(err);
      break;
    }
  } while (!err);
}
//---------------------------------------------------------------------------
// Print variable 64bits
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
//initialisation des tableaux
void init_adc_mm(void) {
  /* valeur par defaut facultative,
    permet d'avoir une moyenne proche
    du resulat plus rapidement
  	val defaut = valdefaut*nSample */
  unsigned int ini_adc1 = 0;// val defaut adc 1
  unsigned int ini_adc2 = 0;// val defaut adc 2
  unsigned int ini_adc3 = 0;// val defaut adc 3
  unsigned int ini_adc4 = 0;// val defaut adc 4
  unsigned int ini_adc5 = 0;// val defaut adc 5
  for (int plus_ancien = 0; plus_ancien < nSample; plus_ancien++) {
    adc_hist[0][plus_ancien] = ini_adc1;
    adc_hist[1][plus_ancien] = ini_adc2;
    adc_hist[2][plus_ancien] = ini_adc3;
    adc_hist[3][plus_ancien] = ini_adc4;
    adc_hist[4][plus_ancien] = ini_adc5;
  }
  //on commencera à stocker à cet offset
  adc_mm[0] = ini_adc1;
  adc_mm[1] = ini_adc2;
  adc_mm[2] = ini_adc3;
  adc_mm[3] = ini_adc4;
  adc_mm[4] = ini_adc5;
}
//---------------------------------------------------------------------------
// lecture des adc
void adc_read() {
  read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V, PinLum);
}
//---------------------------------------------------------------------------
// lecture adc
void read_adc(int pin1, int pin2, int pin3, int pin4, int pin5) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[5];
  for (byte i = 0; i < 5; i++) {
    if (i == 0)sample[i] = moyenneAnalogique(pin1);
    if (i == 1)sample[i] = moyenneAnalogique(pin2);
    if (i == 2)sample[i] = moyenneAnalogique(pin3);
    if (i == 3)sample[i] = moyenneAnalogique(pin4);
    if (i == 4)sample[i] = moyenneAnalogique(pin5);

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
//---------------------------------------------------------------------------
// preparation message de réponse
void messageId() {
  message  = Id;
  message += displayTime(0);
  message += fl;
}
//---------------------------------------------------------------------------
// page html
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
  webpage += F("<td>Type Batterie</td>");
  webpage += F("<td>");	
  if(config.TypeBatt == 16) webpage += F("Pb 12V 6elts");
  if(config.TypeBatt == 24) webpage += F("LiFePO 12.8V 4elts");
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Vitesse SlowBlinker (5-2000ms)</td>");
  webpage += F("<td>");	webpage += String(config.SlowBlinker);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Vitesse FastBlinker (5-2000ms)</td>");
  webpage += F("<td>");	webpage += String(config.FastBlinker);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>R&eacute;p&eacute;tition FastBlink (5-3000ms)</td>");
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
  webpage += F("<td>Luminosit&eacute; Auto</td>");
  webpage += F("<td>");	webpage += String(config.LumAuto);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Tempo r&eacute;p&eacute;tition Wake up Jour Circul&eacute; (s)</td>");
  webpage += F("<td>");	webpage += String(config.RepeatWakeUp);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Auto F si O/S</td>");
  webpage += F("<td>");	webpage += String(config.AutoF);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Tempo AutoF(100-36 000 s)</td>");
  webpage += F("<td>");	webpage += String(config.TempoAutoF);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>TimeOut Wifi (s)</td>");
  webpage += F("<td>");	webpage += String(config.timeoutWifi);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Entr&eacute;e 1</td>");
  webpage += F("<td>");
  if (config.Ip1) {
    webpage += F("Active");
  } else {
    webpage += F("Inactive");
  }
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Entr&eacute;e 2</td>");
  webpage += F("<td>");
  if (config.Ip2) {
    webpage += F("Active");
  } else {
    webpage += F("Inactive");
  }
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS APN</td>");
  webpage += F("<td>");	webpage += String(config.apn);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS user</td>");
  webpage += F("<td>");	webpage += String(config.gprsUser);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS pass</td>");
  webpage += F("<td>");	webpage += String(config.gprsPass);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp Serveur</td>");
  webpage += F("<td>");	webpage += String(config.ftpServeur);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp Port</td>");
  webpage += F("<td>");	webpage += String(config.ftpPort);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp User</td>");
  webpage += F("<td>");	webpage += String(config.ftpUser);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp Pass</td>");
  webpage += F("<td>");	webpage += String(config.ftpPass);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mqtt serveur</td>");
  webpage += F("<td>");	webpage += String(config.mqttServer);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mqtt port</td>");
  webpage += F("<td>");	webpage += String(config.mqttPort);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mqtt username</td>");
  webpage += F("<td>");	webpage += String(config.mqttUserName);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mqtt pass</td>");
  webpage += F("<td>");	webpage += String(config.mqttPass);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>send topic</td>");
  webpage += F("<td>");	webpage += String(config.sendTopic[0]);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>send topic</td>");
  webpage += F("<td>");	webpage += String(config.sendTopic[1]);	webpage += F("</td>");
  webpage += F("</tr>");


  webpage += F("<tr>");
  webpage += F("<td>recv topic</td>");
  webpage += F("<td>");	webpage += String(config.recvTopic[1]);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>declage Heure ete</td>");
  webpage += F("<td>");	webpage += String(config.hete);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>declage Heure hiver</td>");
  webpage += F("<td>");	webpage += String(config.hhiver);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>autoupload</td>");
  webpage += F("<td>");	webpage += String(config.autoupload);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("</table><br>");

  webpage += F("<a href='/download'><button>Download</button></a>");
  webpage += F("<a href='/upload'><button>Upload</button></a>");
  webpage += F("<a href='/delete'><button>Delete</button></a>");
  webpage += F("<a href='/dir'><button>Directory</button></a>");
  webpage += F("<a href='/Tel_list'><button>Tel_list</button></a>");
  webpage += F("<a href='/cal'><button>Calendar</button></a>");
  webpage += F("<a href='/LumLUT'><button>LumLUT</button></a>");
  webpage += F("<a href='/wifioff'><button>Wifi Off</button></a>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
// Lumelut en html
void LumLUTPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Table Luminosit&eacute;</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<th> Lum Ambiante % </th>");
  webpage += F("<th> Lum Feux %</th>");
  webpage += F("</tr>");
  for (int i = 0; i < 11; i++) {
    webpage += F("<tr>");
    webpage += F("<td>"); webpage += TableLum[i][0] ; webpage += F("</td>");
    webpage += F("<td>"); webpage += TableLum[i][1] ; webpage += F("</td>");
    webpage += F("</tr>");
  }
  webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
// liste PB en html
void Tel_listPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Liste des num&eacute;ros t&eacute;l&eacute;phone</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<th> Nom </th>");
  webpage += F("<th> Num&eacute;ro </th>");
  webpage += F("<th> Liste restreinte </th>");
  webpage += F("</tr>");
  
  File file = SPIFFS.open(filePhoneBook, "r");
  while (file.available()) {
    String ligne = file.readStringUntil('\n');
    byte pos1 = ligne.indexOf(";");
    String number   = ligne.substring(0,pos1);
    String name = ligne.substring(pos1+1,ligne.length()-1);
    webpage += F("<tr>");
    webpage += F("<td>"); webpage += String(name); webpage += F("</td>");
    webpage += F("<td>"); webpage += String(number); webpage += F("</td>");
    webpage += F("</tr>");
  }
  file.close();

  webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
// Calendrier en html
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
  if (LittleFS_present) {
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
// SPIFFS directory en html
void SPIFFS_dir() {
  if (LittleFS_present) {
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
  if (LittleFS_present) {
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
// SPIFS file size
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//---------------------------------------------------------------------------
// getion temps restant page web
void handleTime() {
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
// getion Date et heure page web
void handleDateTime() {
  char time_str[20];
  sprintf(time_str, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
bool FTP_Connect(){
  char charbidon[100];
  strncpy(charbidon, "+FTPCID=1",12);
  // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
  Sbidon = modem.send_AT(charbidon);
  // Lecture FTPSERVEUR si OK on saute parametrage suivant
  Sbidon = modem.send_AT("+FTPSERV?");
  // Serial.print("ftpserveur:"),Serial.println(Sbidon);
  Sbidon = Sbidon.substring(Sbidon.indexOf("\"") + 1,Sbidon.lastIndexOf("\""));
  Serial.print("Serveur deja parametre sur SIM7000:"),Serial.println(Sbidon);

  if(Sbidon != String(config.ftpServeur)){ // ftpserveur not configure
    sprintf(charbidon,"+FTPSERV=\"%s\"", config.ftpServeur);
    // Serial.println(charbidon);
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
    Sbidon = modem.send_AT(charbidon);
    Serial.print("FTP serveur :"), Serial.println(Sbidon);

    sprintf(charbidon, "+FTPPORT=%i", config.ftpPort);
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
    Sbidon = modem.send_AT(charbidon);
    Serial.print("FTP port :"), Serial.println(Sbidon);

    sprintf(charbidon, "+FTPUN=\"%s\"", config.ftpUser);
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
    Sbidon = modem.send_AT(charbidon);
    Serial.print("FTP user :"), Serial.println(Sbidon);

    sprintf(charbidon, "+FTPPW=\"%s\"", config.ftpPass);
    // modem.sendAT(String(charbidon));
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",10000);
    Serial.print("FTP pass :"), Serial.println(modem.send_AT(String(charbidon)));
  }
  Serial.println("A finir gestion erreur");
  return true;
}
//---------------------------------------------------------------------------
  bool FTP_Quit() {
    Serial.println(modem.send_AT(F("+FTPQUIT")));
    // Serial.println("A finir gestion erreur");
  return true;
}
//---------------------------------------------------------------------------
// FTP upload file
bool FTP_upload_function (char *file2upload){
  // https://github.com/OscarVanL/SIM7000-LTE-Shield/blob/master/Code/Adafruit_FONA.cpp#L1944
  // FTP ne marche pas si MQTT actif?

  Serial.print("file to upload:"),Serial.println(file2upload);
  if(strcmp(file2upload,filecalibration) == 0){ // seulement pour filecalibration pour le moment
    if(!FTP_Connect()){
      return false;
    }
    delay(1000);

    // Upload du fichier
    char charbidon[100];
    char path[50];
    // destination chemin et filename
    sprintf(path,"/%s/",Id);
    sprintf(charbidon, "+FTPPUTPATH=\"%s\"", path);
    Serial.println(charbidon);
    Serial.println(modem.send_AT(String(charbidon)));
    // Serial.print("FTP put path fichier :"), Serial.println(modem.waitResponse("OK","ERROR"));

    sprintf(charbidon, "+FTPPUTNAME=\"%s\"", "coeff.txt");
    Serial.println(charbidon);
    Serial.println(modem.send_AT(String(charbidon)));
    // Serial.print("FTP put name fichier :"), Serial.println(modem.waitResponse("OK","ERROR"));

    // Ouvrir FTP
    int maxlength = modem.setFTPUpload();
    if(maxlength == -1){
      Serial.println("FTP erreur");
      return false;
    }
    Serial.print("maxlength:"),Serial.println(maxlength);

    // envoyer data
    delay(500);
    // pour test
    int CoeffTension[4];          // Coeff calibration Tension
    char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
    if (SPIFFS.exists(filecalibration)) {
      File f = SPIFFS.open(filecalibration, "r");
      for (int i = 0; i < 4; i++) { //Read
        String s = f.readStringUntil('\n');
        CoeffTension[i] = s.toFloat();
      }
      f.close();
    }
    Serial.print(F("Coeff T Batterie = ")), Serial.print(CoeffTension[0]);
    Serial.print(F(" Coeff T Proc = "))	  , Serial.print(CoeffTension[1]);
    Serial.print(F(" Coeff T VUSB = "))		, Serial.print(CoeffTension[2]);
    Serial.print(F(" Coeff T 24V = "))		, Serial.println(CoeffTension[3]);

    char data[1000];
    for(int i=0;i<4;i++){
      strcat(data,String(CoeffTension[i]).c_str());
      strcat(data,"\n");
    }
    Serial.print("data:"),Serial.println(data);
    int length=strlen(data);
    Serial.print("len:"),Serial.println(length);
    if(length<=maxlength){
      sprintf(charbidon, "+FTPPUT=2,%d",length);
      Serial.print("ATcde:"),Serial.println(charbidon);
      Serial.println(modem.send_AT(charbidon));
      SerialAT.println(data);
    }
    // Fermer FTP
    Serial.println(modem.send_AT("+FTPPUT=2,0"));

    FTP_Quit();
  } else {
    Serial.println("pas supporte pour ce fichier");
    return false;
  }
  return true;
}
//---------------------------------------------------------------------------
// envoyer commande AT au modem
String sendAT(String ATcommand, String answer1, String answer2, unsigned int timeout){
  byte reply = 1;
  String content = "";
  char character;

  //Clean the modem input buffer
  while(Serial2.available()>0) Serial2.read();

  //Send the atcommand to the modem
  Serial2.println(ATcommand);
  delay(100);
  unsigned int timeprevious = millis();
  while((reply == 1) && ((millis() - timeprevious) < timeout)){
    while(Serial2.available()>0) {
      character = Serial2.read();
      content.concat(character);
      Serial.print(character);
      delay(10);
    }
  }
  // Serial.print("reponse: "),Serial.println(content);
  // Serial.println("fin reponse");
  return content;
}
//---------------------------------------------------------------------------
// Vérification Allumage Feu Blanc
void VerifCdeFBlc(){
  // si F != 2(Ouvert) on mesure entree PinChckFblc si == 0 Alarme
  // on mesure accumulation sur 2 secondes si ratio < 35%
  static unsigned long tmesure = millis();
  if (tmesure > millis()) tmesure = millis();
  static int compteurmesureres = 0;
  static int accumesureres = 0;
  int periodemesures = 2000;
  if(Feux != 2){
    accumesureres += digitalRead(PinChckFblc);
    compteurmesureres ++;
    if(millis()- tmesure > periodemesures){// periode mesure > periodemesures
      if(compteurmesureres > 25){// >1200
        // pour eviter fausses alarmes quand proc occupé par ailleurs
        // Serial.print("Cpt Cde FBLc:"),Serial.print(compteurmesureres);
        // Serial.print(", accu:"),Serial.print(accumesureres);
        // Serial.print(", %:"),Serial.println((float)accumesureres/compteurmesureres);
        if((float)accumesureres/compteurmesureres < .25 ){// .35, .5 = M, .75 = S
          Serial.println("Alarme Cde Feu Blanc");
          FlagAlarmeCdeFBlc = true;
          if(!FlagLastAlarmeCdeFBlc){ // si premiere fois
            Extinction();
          }
        }
      }
      tmesure = millis();
      compteurmesureres = 0;
      accumesureres = 0;
    }
    // accumesureres += digitalRead(PinChckFblc);
    // compteurmesureres ++;
  }
}
//---------------------------------------------------------------------------
// efface l'alarme Cde Feu Blanc
void EffaceAlaCdeFBlc(){
  // efface l'alarme Cde Feu Blanc, pour rendre de nouveau
  // operationnel l'extinction en cas de nouveau probleme
  FlagAlarmeCdeFBlc = false;
  FlagLastAlarmeCdeFBlc = false;
}
//---------------------------------------------------------------------------
// Vérification position taquet 1
void VerifTaquet_1(){
  static unsigned long startE1 = millis();
  if (startE1 > millis()) startE1 = millis();
  static bool FlagStartE1 = true;
  // lecture entree Ip1
  if (config.Ip1 && digitalRead(PinIp1) == 0 && !FlagTqt_1){
    if(FlagStartE1){
      FlagStartE1 = false;
      startE1 = millis();
    }
    if(millis() - startE1 > 5000){ // temporisation lecture
      FlagTqt_1 = true;
      MajLog("Auto", "Taquet Ouvert");// taquet Ouvert
      FlagStartE1 = true;
    }
  } else if(config.Ip1 && digitalRead(PinIp1) == 1 && FlagTqt_1){
    if(FlagStartE1){
      FlagStartE1 = false;
      startE1 = millis();
    }
    if(millis() - startE1 > 5000){ // temporisation lecture
      FlagTqt_1 = false;
      MajLog("Auto", "Taquet Ferme");// taquet Fermé
      FlagStartE1 = true;
    }
  }
  if(!FlagStartE1 && (millis() - startE1 > 7000)){ // reset tempo lecture
    FlagStartE1 = true;
    // Serial.print("FlagStartE1:"),Serial.println(FlagStartE1);
  }
}
//---------------------------------------------------------------------------
// Vérification position taquet 2
void VerifTaquet_2(){
  static unsigned long startE2 = millis();
  if (startE2 > millis()) startE2 = millis();
  static bool FlagstartE2 = true;
  // lecture entree Ip1
  if (config.Ip2 && digitalRead(PinIp2) == 0 && !FlagTqt_2){
    if(FlagstartE2){
      FlagstartE2 = false;
      startE2 = millis();
    }
    if(millis() - startE2 > 5000){ // temporisation lecture
      FlagTqt_2 = true;
      MajLog("Auto", "Veriftaquet Taquet V3 Ouvert");// taquet Ouvert
      FlagstartE2 = true;
    }
  } else if(config.Ip2 && digitalRead(PinIp2) == 1 && FlagTqt_2){
    if(FlagstartE2){
      FlagstartE2 = false;
      startE2 = millis();
    }
    if(millis() - startE2 > 5000){ // temporisation lecture
      FlagTqt_2 = false;
      MajLog("Auto", "Veriftaquet Taquet V3 Ferme");// taquet Fermé
      FlagstartE2 = true;
    }
  }
  if(!FlagstartE2 && (millis() - startE2 > 7000)){ // reset tempo lecture
    FlagstartE2 = true;
    // Serial.print("FlagstartE2:"),Serial.println(FlagstartE2);
  }
}
//---------------------------------------------------------------------------
void gestionTaquet(){
  static bool flagchange = false;
  if(FlagTqt_1 != FlagLastTqt_1){ // Taquet a changé d'etat
    if(FlagTqt_1){ // taquet ouvert
      if(FlagDemande_Feux){ // demande changement etat feux en cours
        if(Memo_Demande_Feux[2].indexOf("O") == 0){
          // Serial.print("position O:"),Serial.println(Memo_Demande_Feux[2].indexOf("O"));
          Feux = 2;
          Allumage(); // Violet 0, Blanc 1
          MajLog(Memo_Demande_Feux[0], "OCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        else if (Memo_Demande_Feux[2].indexOf("M") == 0) {
          // Serial.print("position M:"),Serial.println(Memo_Demande_Feux[2].indexOf("M"));
          Feux = 3;
          Allumage(); // Violet 0, Blanc Manoeuvre Cli lent
          MajLog(Memo_Demande_Feux[0], "MCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        else if (Memo_Demande_Feux[2].indexOf("S") == 0) {
          // Serial.print("position S:"),Serial.println(Memo_Demande_Feux[2].indexOf("S"));
          Feux = 4;
          Allumage(); // Violet 0, Blanc Secteur Cli rapide
          MajLog(Memo_Demande_Feux[0], "SCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        else if (Memo_Demande_Feux[2].indexOf("V") == 0) {
          // Serial.print("position V:"),Serial.println(Memo_Demande_Feux[2].indexOf("V"));
          Feux = 7;
          Allumage(); // Violet Cli, Blanc 0
          MajLog(Memo_Demande_Feux[0], "VCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        generationMessage();
        sendReply("MQTTS"); // envoie serveur
        if(Memo_Demande_Feux[1] == "MQTTU"){ // reponse User
          sendReply("MQTTU");
        } else if (Memo_Demande_Feux[1] == "Local"){ // reponse Local
          sendReply("Local");
        }
        FlagDemande_Feux = false; // efface demande
      }
      else{ // pas de demande, juste ouverture taquet, Feux = violet
        Feux = 1;
        Serial.println("Ouverture taquet");
        MajLog("Auto", "FCV");
        envoieGroupeMessage(false,true); // envoie serveur
      }
    } else { // Taquet fermé
      Serial.println("Taquet ferme");
      Feux = 5; // Feux F + Carré
      Allumage();
      MajLog("Auto", "CCV");
      envoieGroupeMessage(false,true); // envoie serveur
    }
    flagchange = true;
  }
  FlagLastTqt_1 = FlagTqt_1;

  if(FlagTqt_2 != FlagLastTqt_2){ // Taquet v3 a changé d'etat
    if(FlagTqt_2){ // Taquet ouvert
      MajLog("Auto", "gestiontaquet Taquet V3 ouvert");
    } else {
      MajLog("Auto", "gestiontaquet Taquet V3 ferme");
    }
    envoieGroupeMessage(false,true); // envoie serveur
    FlagLastTqt_2 = FlagTqt_2;
    flagchange = true;
  }
  if(flagchange){
    flagchange = false;
    Acquisition();
  }
}
//---------------------------------------------------------------------------
// etat synchronisation time/heure systeme
void timesstatus() {
  Serial.print(F("Synchro Time  : "));
  switch (timeStatus()) {
    case 0:
      Serial.println(F(" pas synchro"));
      break;
    case 1:
      Serial.println(F(" defaut synchro"));
      break;
    case 2:
      Serial.println(F(" OK"));
      break;
  }
}
//---------------------------------------------------------------------------
// Connexion Gprs
void ConnectGPRS(){
  Serial.print(F("Connecting to "));
  Serial.print(config.apn);
  if (modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
    Serial.println(F(" success"));
  }
  else {
    Serial.println(" fail");
  }
}
//---------------------------------------------------------------------------
// Connexion MQTT, Clean session false
void mqttConnect() {
  if (modem.isGprsConnected()) {
    // Connect to the MQTT broker.
    Serial.print("Attempting MQTT connection...");
    if ( mqttClient.connect(config.Idchar, config.mqttUserName, config.mqttPass,willTopic,1,true,config.Idchar,false)) {
      Serial.println( "Connected with Client ID:  " + String(config.Idchar) + " User " + String(config.mqttUserName) + " Pwd " + String(config.mqttPass));
      AlarmeMQTT = false;
    } else {
      Serial.print( "failed, rc = " );
      // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
      Serial.print( mqttClient.state() );
      Serial.println( " Will try again in 5 seconds" );
      AlarmeMQTT = true;
    }
  }
}
//---------------------------------------------------------------------------
// Subscription MQTT, qos=1
bool mqttSubscribe(bool unsubSub) {
  byte rep = 1;
  // unsubSub = 0 subscribe, = 1 unsubscribe
  if (unsubSub == 0) {
    for(byte i = 0; i<2;i++){
      if (mqttClient.subscribe( config.recvTopic[i] , 1 )){ // Subscribe
        rep *= rep;
      } else {
        rep *= 0;
      }      
      delay(200);
    }
    return rep;
  } else {
    for(byte i = 0; i<2;i++){
      if (mqttClient.unsubscribe(config.recvTopic[i])){ // Unsubscribe
        rep *= rep;
      } else {
        rep *= 0;
      }
      delay(200);
    }
    return rep;
  }
}
//---------------------------------------------------------------------------
// Cherche number existe dans fichier PhoneBook
bool Cherche_N_PB(String number){
  if (SPIFFS.exists(filePhoneBook)) {
    File file = SPIFFS.open(filePhoneBook, "r");
    while (file.available()) {
      String s = file.readStringUntil('\n');
      if(s.indexOf(number)>-1){
        Serial.print("N° trouve:"),Serial.println(s);
        file.close();
        return true;
      }
    }
    file.close();
  }
  return false;
}
//---------------------------------------------------------------------------
bool SyncHeureModem(int Savetime, bool FirstTime){
  int rep = 1;
  int compteur = 0;
  if(modem.isNetworkConnected() && modem.isGprsConnected()){
    do{
      rep = modem.NTPServerSync(NTPServer, Savetime);
      Serial.print("NTP:"),Serial.println(modem.ShowNTPError(rep));
      if (compteur > 10){ // 10 tentatives
        if(FirstTime){
          // echec Synchro, force date 01/08/2022 08:00:00, jour toujours circulé
          modem.send_AT("+CCLK=\"22/08/01,08:00:00+08\"");
          delay(500);
        }
        return false;
      }
      compteur += 1;
      delay(1000);
    } while(rep != 1);
    return true;
  }
  return false;
}
//---------------------------------------------------------------------------
void message_Monitoring_Reseau(){
  message += F("Histo Network Chgt : ");
  message += fl;
  message += F("no service : ");
  message += String(Histo_Reseau[0]);
  message += fl;
  message += F("GSM : ");
  message += String(Histo_Reseau[1]);
  message += fl;
  message += F("EGPRS : ");
  message += String(Histo_Reseau[2]);
  message += fl;
  message += F("LTE-M1 : ");
  message += String(Histo_Reseau[3]);
  message += fl;
  message += F("LTE-NB : ");
  message += String(Histo_Reseau[4]);
  message += fl;
}
//---------------------------------------------------------------------------
void readmodemtime(){
  String modemHDtate = modem.getGSMDateTime(TinyGSMDateTimeFormat(0));
  // convertir format date time yy/mm/dd,hh:mm:ss
  byte i 	= modemHDtate.indexOf("/");
  byte j 	= modemHDtate.indexOf("/", i + 1);
  N_Y		  = modemHDtate.substring(i - 2, i).toInt();
  N_M 		= modemHDtate.substring(i + 1, j).toInt();
  N_D 		= modemHDtate.substring(j + 1, j + 3).toInt();
  i 	  	= modemHDtate.indexOf(":", 6);
  j     	= modemHDtate.indexOf(":", i + 1);
  N_H 		= modemHDtate.substring(i - 2, i).toInt();
  N_m 		= modemHDtate.substring(i + 1, j).toInt();
  N_S 		= modemHDtate.substring(j + 1, j + 3).toInt();
}
//---------------------------------------------------------------------------
bool HeureEte() {
  // Serial.print("mois:"),Serial.print(month());
  // Serial.print(" jour:"),Serial.print(day());
  // Serial.print(" jsem:"),Serial.println(weekday());
  // return true en été, false en hiver (1=dimanche)
  bool Hete = false;
  if (month() > 10 || month() < 3
      || (month() == 10 && (day() - weekday()) > 22)
      || (month() == 3  && (day() - weekday()) < 24)) {
    Hete = false;                      								// c'est l'hiver
  }
  else {
    Hete = true;                       								// c'est l'été
  }
  return Hete;
}
//---------------------------------------------------------------------------
String ConnectedNetwork(){  
  String network = "";
  int n = modem.getNetworkCurrentMode();
  if(n==0){     network = F("no service");}
  else if(n==1){network = F("GSM");}
  else if(n==3){network = F("EGPRS");}
  else if(n==7){network = F("LTE-M1");}
  else if(n==9){network = F("LTE-NB");}
  return network;
}
//---------------------------------------------------------------------------
/* Enregistre mode cnx Reseau
  Histo-Reseau(0) = no service, (1)=GSM, (3)=EGPRS, (7)= LTE-M1, (9)= LTE-NB
  cummul à chaque heure */
void Monitoring_Reseau(){
  int n = modem.getNetworkCurrentMode();
  if(n==0){     Histo_Reseau[0]  ++ ;}
  else if(n==1){Histo_Reseau[1]  ++;}
  else if(n==3){Histo_Reseau[2]  ++;}
  else if(n==7){Histo_Reseau[3]  ++;}
  else if(n==9){Histo_Reseau[4]  ++;}
}
//---------------------------------------------------------------------------
// Reception message subscription
void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int mesLength ) {
  /* 6) Use the mqttSubscriptionCallback function to handle incoming MQTT messages.
    The program runs smoother if the main loop performs the processing steps instead of the callback.
    In this function, use flags to cause changes in the main loop. */
  /**
    Process messages received from subscribed channel via MQTT broker.
      topic - Subscription topic for message.
      payload - Field to subscribe to. Value 0 means subscribe to all fields.
      mesLength - Message length.
  */
  // variable stockage temporaire
  static char temptopic[12];
  static String tempmessage = "";

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" len:");
  Serial.print(mesLength);
  Serial.print(". Message: ");
  
  Sbidon = "";
  
  for (int i = 0; i < mesLength; i++) {
    Serial.print((char)payload[i]);
    Sbidon += (char)payload[i];
  }
  Serial.println();

  /* si len>0, flagRcvMQTT = true, le traitement des commandes bloquantes
    NONCIRCULE, "Wifi,SSID,PW"
    ne seront pas executées,
    ne le seront qu'au retour de flagRcvMQTT = false
    garantie que le message à bien été effacé du serveur
    cela évitera un bouclage sur ce message
    ATTENTION
    entre temps les variables Rmessage et Origine ne doivent pas etres altérées!
    */
  if(Sbidon.length() > 0){
    flagRcvMQTT = true;
    // sauvegarde topic et message
    tempmessage = Sbidon;
    // temptopic   = String(topic);
    strcpy(temptopic,topic);
    Rmessage    = Sbidon;
    // on efface le topic sur le serveur
    if(strcmp(topic,config.recvTopic[0]) == 0){ // Serveur
      Serial.println(mqttClient.publish(config.recvTopic[0],"")); // efface topic sur serveur
    } else if(strcmp(topic,config.recvTopic[1]) == 0){ // User
      Serial.println(mqttClient.publish(config.recvTopic[1],"")); // efface topic sur serveur    
    }
    Serial.println(Rmessage);
    if(flagRcvMQTT){
      // message bloquant sera traité apres retour message len=0.
      Serial.println("message sera traite apres reception message len=0");
      return;
    } else {
      flagRcvMQTT = false;
      Serial.println("message traite maintenant");
      // message non bloquant, on traite de suite
      if(strcmp(temptopic,config.recvTopic[0]) == 0){ // Serveur
        Serial.println("message from serveur");
        traite_sms("MQTTS");
      } else if(strcmp(temptopic,config.recvTopic[1]) == 0){ // User
        Serial.println("message from user");
        traite_sms("MQTTU");
      }
    }
  } else if(Sbidon.length() == 0){
    Serial.println("len = 0");
    if (flagRcvMQTT){
      flagRcvMQTT = false;
      Rmessage = tempmessage;
      // on traite maintenant
      Serial.println("on traite maintenant");
      if(strcmp(temptopic,config.recvTopic[0]) == 0){ // Serveur
        traite_sms("MQTTS");
      } else if(strcmp(temptopic,config.recvTopic[1]) == 0){ // User
        traite_sms("MQTTU");
      }
    }
  }
}
//---------------------------------------------------------------------------
// lire valeur RSSI et remplir message
String read_RSSI() {

  String rssi = "";
  int r;
  byte n = modem.getSignalQuality();
  // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  rssi  = F("RSSI= ");
  rssi += String(n);
  rssi += ", ";
  rssi += String(r);
  rssi += F("dBm");
  return rssi;
}
//---------------------------------------------------------------------------
// Allumage modem
int modem_on() {
    /*
    The indicator light of the board can be controlled
    */
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    /*
    MODEM_PWRKEY IO:18 The power-on signal of the modulator must be given to it,
    otherwise the modulator will not reply when the command is sent
    */
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(300);
    digitalWrite(MODEM_PWRKEY, HIGH);

    /*
    MODEM_FLIGHT IO:25 Modulator flight mode control,
    need to enable modulator, this pin must be set to high
    */
    // pinMode(MODEM_FLIGHT, OUTPUT);
    // digitalWrite(MODEM_FLIGHT, HIGH);

  int i = 10;
  Serial.println("Testing Modem Response...");
  Serial.println("****");
  while (i) {
    SerialAT.println("AT");
    delay(500);
    if (SerialAT.available()) {
      String r = SerialAT.readString();
      Serial.println(r);
      if ( r.indexOf("OK") >= 0 ) {
        // reply = true;
        break;
      }
    }
    delay(500);
    i--;
  }
  Serial.println("****");
  return i;
}
//---------------------------------------------------------------------------
// retourne n° derniere ligne PhoneBook
byte last_PB(){
  Read_PB();
  byte dernier = 0;
  for(byte i = 1;i<10;i++){
    if(strlen(PB_list[i]) > 0){
      dernier = i;
    }
  }
  return dernier;
}
//---------------------------------------------------------------------------
// Vérification fichier PhoneBook existe
void Ouvrir_PB() {
  // par ligne : N°tel;Nom\n
  if (!SPIFFS.exists(filePhoneBook)) {
    // fichier n'existe pas
    Serial.print(F("Creating Data File:")), Serial.println(filePhoneBook); // valeur par defaut
    File file = SPIFFS.open(filePhoneBook, "w+");
    file.println(default_PB);
    file.close();
  }
  Read_PB();
}
//---------------------------------------------------------------------------
// Lecture filePhoneBook copie dans PB_list
void Read_PB(){
  // vide PB_list
  for(byte i = 1;i<10;i++){
    strcpy(PB_list[i] , "");
  }
  // lire fichier
  File file = SPIFFS.open(filePhoneBook, "r");
  byte idx = 0;
  while (file.available()) {
    idx ++;
    String ligne = file.readStringUntil('\n');
    strcpy(PB_list[idx] , ligne.c_str());
  }
  file.close();
}
//---------------------------------------------------------------------------
// Sauvegarde filePhoneBook
void Save_PB(){
  File file = SPIFFS.open(filePhoneBook, "w+");
  for (byte i = 1;i<10;i++){
    if(strlen(PB_list[i])>0){
      file.println(String(PB_list[i]));
    }
  }
  file.close();
}
//---------------------------------------------------------------------------
// Copie valeur topic en config
void copie_Topic(){
  strncpy(config.sendTopic[0],("S/in"),sizeof(config.sendTopic[0]));
  strncpy(config.sendTopic[1],("S/Uin/"),sizeof(config.sendTopic[1]));
  strcat( config.sendTopic[1], &config.Idchar[5]); // S/Uin/CVxx

  strncpy(config.recvTopic[0],("S/Sout/"),sizeof(config.recvTopic[0]));
  strcat( config.recvTopic[0], &config.Idchar[5]); // S/Sout/CVxx
  
  strncpy(config.recvTopic[1],("S/Uout/"),sizeof(config.recvTopic[1]));
  strcat( config.recvTopic[1], &config.Idchar[5]); // S/Uout/CVxx
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
    Rmessage = serialmessage;
    traite_sms("Local");//	traitement en mode local
    newData = false;
    serialmessage = "";
  }
}
//---------------------------------------------------------------------------
