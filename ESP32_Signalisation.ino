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
  reveille tout les matin 06h55
  reception des SMS en attente
  apres 5 min de fonctionnement (ex: 2mn pour reception/suppression 8 SMS, 4mn 14SMS)
  envoie sms signal vie
  analyse calendrier sauvegardé en SPIFFS

	si jour circulé
  on continue normalement
  en fin de journée retour sleep jusqu'a debut

  si non circulé,
  retour SIM800 et ESP32 en sleep reveil toute les heures
  au reveil attendre au moins 30s pour que les SMS arrivent,
  quand plus de SMS et traitement retour sleep 1H00

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


	to do
  passer ESP32 V2+
  passer à nouvelle version SPIFFS LittelFs
  remplacer EEPROM par file https://arduino.stackexchange.com/questions/90204/how-to-store-a-struct-in-a-file
  augmenter moyennage lors de la mesure tension calibration
  passer en 4G
  envoyer/recevoir message en SMS ou SMS+MQTT
  
  V2-18 02/07/2023 installé spare ex Cv45 voir fichier etatversion.txt
  nouveau Magique
  1- Prise en compte nouvelle batterie LiFePO
    nouveau parametre config.TypeBatt: 16 Pb 6elts, 24LiFePO 4elts
  2- seuil alarme retour >=80%

  Compilation LOLIN D32,default,80MHz, IMPORTANT ESP32 1.0.2 (version > bug avec SPIFFS?)
  Arduino IDE 1.8.19 : 1020226 77%, 47936 14% sur PC
  Arduino IDE 1.8.19 : 1019578 77%, 47928 14% sur raspi (sans ULP)

  V2-17 03/02/2023 installé spare ex Cv45
  1- Efface sms en debut de traitement
  2- Renvoie sur liste restreinte message provenant d'un numéro < 8 chiffres (N° Free)

  Compilation LOLIN D32,default,80MHz, IMPORTANT ESP32 1.0.2 (version > bug avec SPIFFS?)
  Arduino IDE 1.8.19 : 1018786 77%, 47936 14% sur PC
  Arduino IDE 1.8.19 : 1017830 77%, 47928 14% sur raspi


  V2-16 15/07/2022 installé Cv65(spare ex Cv55) item 1 et 2 seulement
  1- Verif Feu blanc que si Allume
  2- Comande vide log par SMS
  3- Print __FILE__ au démarrage
  4- Supprimer envoie SMS soimeme si pb majheure,
    remplacé par majheure par defaut 01/08/2022 08:00:00
  5- Ajouter Commande AT par SMS: SENDAT=cdeAT

  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (version > bug avec SPIFFS?)
  Arduino IDE 1.8.19 : 1018062 77%, 47928 14% sur PC
  Arduino IDE 1.8.16 : 1018010 77%, 47928 14% sur raspi

  V2-15 11/01/2022 installé Cv65,66,45,46,55,56 tous VH2.2
  1- Ajout gestion Entre2 Taquet 2, code ajouter F ou O apres code d'origine
  2- Correction bug si Jour NonCircule ouverture taquet1 affichage incorrect
    pas de gestion taquet si NonCircule

  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (version > bug avec SPIFFS?)
  Arduino IDE 1.8.19 : 1017202 77%, 47928 14% sur PC
  Arduino IDE 1.8.*  : * 77%, 47928 14% sur raspi

  V2-14 30/08/2021 installé CV35,46,45,56,55,66,65

  1 - installation VCV
  2 - suppression rearmenent tempo AutoF sur demande ST

  V2-13 12/08/2021 installé Cv65
  1- ajouter dans log, numero appelant non reconnu, lorsque envoie sms soi meme pour majheure

  V2-12 15/02/2021 installé CV65 19/02/2021, CV55-56
  1 - mise en place verification/Alarme defaillance Cde Feu Blanc
  2 - fiabilisation mesure tension lors du calibrage

  V2-12 12/12/2020 pas installé
  remplacer <credentials_ftp.h> par <credentials_tpcf.h>
  correction char ftpUser
  nouveau magic
  V2-11 27/10/2020 installé CV65 30/10/2020
  sur reception sms F si Carré fermé ne rien faire

  V2-10 03/10/2020 installé CV65 05/10/2020
  bug GPRSDATA message retour
  V2-1 02/10/2020 installé CV65
  1- upload log sur demande par sms
  2- entrée E1 contact taquet ouvert pour Cv65
     sur reception O,S,M la commande n'est prise en compte que si E1 fermée(taquet Ouvert)
  3- calibration possible par sms idem Autorail,
     permet recalibration a distance en cas de perte données cal, evite un blocage sur alarme Alim

  02/06/2020
  V1-41 installé 13/07/2020 CV45 CV46
  1- maj valeur defaut anticip=2700
     nouveau magic
  2- corection bug général, pour changer un Alarm.timerRepeat et Alarm.alarmRepeat,
     il faut utiliser la fonction Alarm.write(Id, durée)
     inversion message debut jour
  3- AutoF applicable seulement O et S pas sur M
  
  06/01/2020
  installation Cv65 V1-4

*/

#include <Arduino.h>

String ver        = "V2-18";
int    Magique    = 14;

#include <Battpct.h>
#include <Sim800l.h>              //my SIM800 modifié
#include <Time.h>
#include <TimeAlarms.h>
#include <sys/time.h>             //<sys/time.h>
#include <WiFi.h>
// #include <EEPROM.h>               // variable en EEPROM(SPIFFS)
// #include <SPIFFS.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <Ticker.h>
#include "passdata.h"
#include <ArduinoJson.h>
#include <credentials_tpcf.h>

// #define SPIFFS LittleFS // remplace SPIFFS par LittleFS
String  webpage = "";
#define ServerVersion "1.0"
bool    LittleFS_present = false;
#include "CSS.h"               // pageweb

// #define RESET_PIN     18   // declaré par Sim800l.h
// #define LED_PIN        5   // declaré par Sim800l.h
#define PinChckFblc   4    // Entrée verification Cde Feu Blanc
#define PinBattProc   35   // liaison interne carte Lolin32 adc
#define PinBattSol    39   // Batterie générale 12V adc VN
#define PinBattUSB    36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi
#define PinIp1        32   // Entrée Ip1 Wake up EXT1 Taquet VP Cv65 1=F
#define PinIp2        33   // Entrée Ip2 Wake up EXT1 Taquet V3 Cv65 1=F
#define Pin24V        26   // Mesure Tension 24V
#define PinFBlc       21   // Sortie Commande Feu Blanc
#define PinConvert    19   // Sortie Commande Convertisseur 12/24V
#define PinFVlt       15   // Sortie Commande Feu Violet
#define RX_PIN        16   // TX Sim800
#define TX_PIN        17   // RX Sim800
#define PinReset      13   // Reset Hard
#define PinLum        34   // Mesure Luminosité
#define PinAlimLum    25   // Alimentation LDR
#define PinTest       27   // Test sans GSM cc a la masse

#define SIMPIN        1234 // Code PIN carte SIM

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define FORMAT_LITTLEFS_IF_FAILED true // format LittelFS si lecture impossible
#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];            // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
// unsigned long timer100  = 0; // pour timer 100ms adc
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char fileconfig[12]      = "/config.txt";   // fichier en LittleFS contenant structure config
char filecalendrier[13]  = "/filecal.csv";  // fichier en LittleFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en LittleFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en LittleFS contenant le log
char filelumlut[13]      = "/lumlut.txt";   // fichier en LittleFS LUT luminosité

const String soft = "ESP32_Signalisation.ino.d32"; // nom du soft

const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;
String bufferrcpt;
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans config
char   SIM800InBuffer[64];          //  for notifications from the SIM800
char   replybuffer[255];            //  Buffer de reponse SIM800
// volatile int IRQ_Cpt_Ip1  = 0;      //  IRQ Ip1
// volatile int IRQ_Cpt_Ip2  = 0;      //  IRQ Ip2
// volatile unsigned long rebond1 = 0; //	antirebond IRQ
// volatile unsigned long rebond2 = 0; //	antirebond IRQ
// byte DbounceTime = 20;              // antirebond
// byte confign = 0;                   // position enregistrement config EEPROM
// byte recordn = 200;                 // position enregistrement log EEPROM
bool Allume  = false;
byte BlcPwmChanel = 0;
byte VltPwmChanel = 1;
bool isBlinking = false;
bool blinker = false;

RTC_DATA_ATTR int  Feux = 0; // Etat des Feux voir tableau au début
RTC_DATA_ATTR bool FlagAlarmeTension       = false; // Alarme tension Batterie
RTC_DATA_ATTR bool FlagLastAlarmeTension   = false;
RTC_DATA_ATTR bool FlagMasterOff           = false; // Coupure Allumage en cas de pb
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
bool FlagReset = false;       // Reset demandé
bool jour      = false;				// jour = true, nuit = false
bool gsm       = true;        // carte GSM presente utilisé pour test sans GSM seulement

String Memo_Demande_Feux[3]={"","",""};  // 0 num demandeur,1 nom, 2 feux demandé (O2,M3,S4,V7)
bool FlagDemande_Feux = false;   // si demande encours = true

int CoeffTension[4];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

// bool LastWupAlarme = false;   // memo etat Alarme par Wakeup

int    slot = 0;              //this will be the slot number of the SMS

long   TensionBatterie  = 0; // Tension Batterie solaire
long   VBatterieProc    = 0; // Tension Batterie Processeur
long   VUSB             = 0; // Tension USB
long   Tension24        = 0; // Tension 24V Allumage
int    Lum              = 0; // Luminosité 0-100%
int    TableLum[11][2];      // Table PWM en fonction Luminosité

WebServer server(80);
File UploadFile;

// typedef struct               // declaration structure  pour les log
// {
//   char    dt[10];            // DateTime 0610-1702 9+1
//   char    Act[2];            // Action A/D/S/s 1+1
//   char    Name[15];          // 14 car
// } champ;
// champ record[5];

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
  bool    Pos_Pn_PB[10];   // numero du Phone Book (1-9) à qui envoyer 0/1 0 par defaut
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
} ;
config_t config;

Ticker SlowBlink;          // Clignotant lent
Ticker FastBlink;          // Clignotant rapide
Ticker FastRate;           // Repetition Clignotant rapide
Ticker ADC;                // Lecture des Adc

AlarmId loopPrincipale;    // boucle principale
AlarmId DebutJour;         // Debut journée
AlarmId FinJour;           // Fin de journée retour deep sleep
AlarmId Auto_F;            // Tempo AutoF

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial *SIM800Serial = &Serial2; // liaison serie FONA SIM800
Sim800l Sim800;                          // to declare the library

//---------------------------------------------------------------------------
// void IRAM_ATTR handleInterruptIp1() { // Entrée 1

//   portENTER_CRITICAL_ISR(&mux);
//   if (xTaskGetTickCount() - rebond1 > DbounceTime) {
//     IRQ_Cpt_Ip1++;
//     rebond1 = xTaskGetTickCount(); // equiv millis()
//   }
//   portEXIT_CRITICAL_ISR(&mux);

// }
// void IRAM_ATTR handleInterruptIp2() { // Entrée 2

//   portENTER_CRITICAL_ISR(&mux);
//   if (xTaskGetTickCount() - rebond2 > DbounceTime) {
//     IRQ_Cpt_Ip2++;
//     rebond2 = xTaskGetTickCount(); // equiv millis()
//   }
//   portEXIT_CRITICAL_ISR(&mux);
// }
//---------------------------------------------------------------------------

void setup() {

  message.reserve(140);

  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);

  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) { // Format la première fois utilise LitteFS
    Serial.println(F("LittleFS initialisation failed..."));
    LittleFS_present = false;
  }
  else {
    Serial.println(F("LittleFS initialised... file access enabled..."));
    LittleFS_present = true;
  }
  LittleFS.remove("/somefile.txt");
  pinMode(PinIp1     , INPUT_PULLUP);
  pinMode(PinIp2     , INPUT_PULLUP);
  pinMode(PinFBlc    , OUTPUT);
  pinMode(PinFVlt    , OUTPUT);
  pinMode(PinConvert , OUTPUT);
  pinMode(PinAlimLum , OUTPUT);
  pinMode(PinTest    , INPUT_PULLUP);
  pinMode(PinChckFblc, INPUT_PULLUP);
  digitalWrite(PinConvert , LOW); // Alimentation Convert 0
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
    Serial.println("lancement SIM800");
    SIM800Serial->begin(9600); // 4800
    Sim800.begin();
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
  // readFile(LittleFS,filelog);      // Lecture des log
  // Alarm.delay(500);
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
    for (int i = 0; i < 10; i++) {// initialise liste PhoneBook liste restreinte
      config.Pos_Pn_PB[i] = 0;
    }
    // config.Pos_Pn_PB[1]  = 1;	// le premier numero du PB par defaut
    String temp          = "TPCF_CV65";
    temp.toCharArray(config.Idchar, 11);
    String tempapn       = "free";//"sl2sfr"
    String tempGprsUser  = "";
    String tempGprsPass  = "";
    config.ftpPort       = tempftpPort;
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempGprsUser.toCharArray(config.gprsUser,(tempGprsUser.length() + 1));
    tempGprsPass.toCharArray(config.gprsPass,(tempGprsPass.length() + 1));
    tempServer.toCharArray(config.ftpServeur,(tempServer.length() + 1));
    tempftpUser.toCharArray(config.ftpUser,(tempftpUser.length() + 1));
    tempftpPass.toCharArray(config.ftpPass,(tempftpPass.length() + 1));

    sauvConfig();
    // valeur par defaut des record (log)
    // for (int i = 0; i < 5 ; i++) {
    //   temp = "";
    //   temp.toCharArray(record[i].dt, 10);
    //   temp.toCharArray(record[i].Act, 2);
    //   temp.toCharArray(record[i].Name, 15);
    // }
    // writeFile(LittleFS,filelog,record);// ecriture des valeurs par defaut
    // EEPROM.put(recordn, record);// ecriture des valeurs par defaut
    // EEPROM.commit();
  }
  // EEPROM.end();
  PrintConfig();
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
    delay(100);
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
  OuvrirLumLUT();             // ouvre le fichier lumLUT
  // Serial.print(F("temps =")),Serial.println(millis());
  if (gsm) {
    Sim800.reset(SIMPIN);					// lancer SIM800
    MajHeure("");
  }

  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  DebutJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);
  Alarm.enable(DebutJour);

  FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee); // Fin de journée retour deep sleep
  Alarm.enable(FinJour);

  Auto_F = Alarm.timerRepeat(config.TempoAutoF, AutoFermeture);
  Alarm.disable(Auto_F);

  // ActiveInterrupt();

  Serial.print(F("flag Circule :")), Serial.println(flagCircule);

  if(!config.Ip1){ // si Ip1 innactive FlagTqt_1 = taquet ouvert
    FlagTqt_1 = true;
    FlagLastTqt_1 = false;
  }

  // if (Feux != 0) { // si une valeur Feux different de 0 en memoire RTC, on Allume les feux
    // Allumage();
  // }
  MajLog("Auto","Lancement");
}
//---------------------------------------------------------------------------
void loop() {
  recvOneChar();

  // if (rebond1 > millis()) rebond1 = millis();
  // if (rebond2 > millis()) rebond2 = millis();
//*************** Verification reception SMS ***************
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

  static int8_t nsms;
  static int cpt = 0; // compte le nombre de passage boucle
  static bool firstdecision = false;
  static byte cptallume = 0; // compte le nombre de passage avec Allume

  AIntru_HeureActuelle();

  if (cpt > 6 && nsms == 0 && !firstdecision) {
    /* une seule fois au demarrage attendre au moins 70s et plus de sms en attente */
    action_wakeup_reason(get_wakeup_reason());
    firstdecision = true;
  }
  cpt ++;
  if((config.Ip1 || config.Ip2) && firstdecision) gestionTaquet(); // gestion etat taquet seulement apres demarrage

  if (CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0 || CoeffTension[3] == 0) {
    OuvrirFichierCalibration(); // patch relecture des coeff perdu
  }

  if (gsm) {
    if (!Sim800.getetatSIM())Sim800.reset(SIMPIN); // verification SIM
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

  Serial.print("luminosité = "), Serial.print(Lum);
  Serial.print(" lumlut = "), Serial.println(lumlut(Lum));

  // en cas de feux fixe rafraichissement commande en fonction lum
  // les feux M et S sont automatiquement ajusté par blink
  if (Feux == 1) Update_FVlt(); // Violet
  if (Feux == 2) Update_FBlc(); // Blanc

  // Serial.print(adc_mm[0]),Serial.print(";");
  // Serial.print(adc_mm[1]),Serial.print(";");
  // Serial.println(adc_mm[2]);

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

  message = F(" Batt Solaire = ");
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
    /* verification nombre SMS en attente(raté en lecture directe)
       traitement des sms en memoire un par un,
       pas de traitement en serie par commande 51, traitement beaucoup trop long */
    nsms = Sim800.getNumSms(); // nombre de SMS en attente (1s)
    Serial.print(F("Sms en attente = ")), Serial.println(nsms);

    if (nsms > 0) {	// nombre de SMS en attente
      // il faut les traiter
      int numsms = Sim800.getIndexSms(); // cherche l'index des sms en mémoire
      Serial.print(F("Numero Sms en attente = ")), Serial.println(numsms);
      if(numsms > 10){
        // grand nombre sms en memoire, trop long a traiter
        Serial.print("num sms > 10, efface tout :"),Serial.println(numsms);
        MajLog("Auto", "numsms > 10 efface tous sms");// renseigne log
        Sim800.delAllSms();
      }
      traite_sms(numsms);// traitement des SMS en attente
    }
    else if (nsms == 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
      FlagReset = false;
      ResetHard();				//	reset hard
    }
  }
  else if (FlagReset) {
    FlagReset = false;
    ResetHard();				//	reset hard
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
void Allumage() {
  if(!Allume){
    Serial.println("Allumage");
    Allume = true;
    digitalWrite(PinConvert, HIGH); // Alimentation du convertisseur 12/24V
  }
  GestionFeux();
}
//---------------------------------------------------------------------------
void Extinction() {
  Serial.println("Exctinction");
  if(config.Ip1){ // si E1 active
    if(FlagTqt_1){  // taquet ouvert
      Feux = 0;   // D tout eteint et Taquet Ouvert
    } else {
      Feux = 6;   // Z tout eteint et Taquet Fermé
    }
  } else {
    Feux = 0;     // D tout eteint pas de Taquet
  }
  GestionFeux();
  Allume = false;
  digitalWrite(PinConvert, LOW); // Arret du convertisseur 12/24V
  MajLog(F("Auto"), "Feux = " + String(Feux));
  envoieGroupeSMS(3, 0); // envoie serveur
}
//---------------------------------------------------------------------------
void AutoFermeture() {
  // fin de TempoAutoF
  // Feux à F si Feux = O/S/V rien faire si M
  if (Feux == 2 || Feux == 4 || Feux == 7) {
    Feux = 1;
    Allumage(); // Violet 1, Blanc 0
    envoieGroupeSMS(3, 0); // envoie serveur
    MajLog("AutoF", "FCV");
  }
  Alarm.disable(Auto_F);
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
  bool smsserveur = false; // true si le sms provient du serveur index=1

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
      if (Sim800.getNumberSms(slot) == Sim800.getPhoneBookNumber(1)) {
        smsserveur = true; // si sms provient du serveur index=1
      }
      if (nom.length() < 1) { // si nom vide, cherche si numero est num de la SIM
        if (numero == Sim800.getNumTel()) {
          nom = F("Moi meme");
        }
      }
      Serial.print(F("Nom appelant = ")), Serial.println(nom);
      Serial.print(F("Numero = ")), Serial.println(numero);
      byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
      if(numero.length() < 8){ // numero service free
        for (byte Index = 1; Index < n + 1; Index++) { // Balayage des Num Tel dans Phone Book
          if (config.Pos_Pn_PB[Index] == 1) { // Num dans liste restreinte
            String number = Sim800.getPhoneBookNumber(Index);
            char num[13];
            number.toCharArray(num, 13);
            message = textesms;
            EnvoyerSms(num, true);
            EffaceSMS(slot);
            return; // sortir de la procedure traite_sms
          }
        }
      }
    }
    else {
      textesms = String(replybuffer);
      nom = F("console");
    }

    if (sms && !(textesms.indexOf(F("MAJHEURE")) == 0)) { // suppression du SMS sauf si MAJHEURE
      EffaceSMS(slot);
    }

    if (!(textesms.indexOf(F("TEL")) == 0 || textesms.indexOf(F("tel")) == 0 || textesms.indexOf(F("Tel")) == 0
        || textesms.indexOf(F("Wifi")) == 0 || textesms.indexOf(F("WIFI")) == 0 || textesms.indexOf(F("wifi")) == 0
        || textesms.indexOf(F("GPRSDATA")) > -1 || textesms.indexOf(F("FTPDATA")) > -1 || textesms.indexOf(F("FTPSERVEUR")) > -1)) {
      textesms.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI" parametres pouvant contenir minuscules
      // textesms.trim();
    }
    textesms.replace(" ", "");// supp tous les espaces
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("WIFIOFF")) > -1) { // Arret Wifi
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
        ssids.toCharArray(ssid, ssids.length() + 1);
        pwds.toCharArray(pwd, pwds.length() + 1);
        ConnexionWifi(ssid, pwd, number, sms); // message généré par routine
      }
      else if (gsm && (textesms.indexOf(F("TEL")) == 0
               || textesms.indexOf(F("Tel")) == 0
               || textesms.indexOf(F("tel")) == 0)) { // entrer nouveau num
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
          if (gsm) {
            Sim800.WritePhoneBook(Send);					//ecriture dans PhoneBook
            Alarm.delay(500);
            Sim800.ModeText(); //pour purger buffer fona
            Alarm.delay(500);
          }
          messageId();
          message += F("Nouveau Num Tel: ");
          message += F("OK");
          message += fl;
          EnvoyerSms(number, sms);
        }
      }
      else if (gsm && (textesms == F("LST?") || textesms == F("LST1"))) {	//	Liste des Num Tel
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
        // if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SYS")) > -1) {
        if (gsm) {
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
      // else if (textesms.indexOf(F("LOG")) == 0) {	// demande log des 5 derniers commandes
      //   File f = SPIFFS.open(filelog, "r"); // taille du fichier log en SPIFFS
      //   message = F("local log size :");
      //   message += String(f.size()) + fl;
      //   f.close();
      //   for (int i = 0; i < 5; i++) {
      //     message += String(record[i].dt) + "," + String(record[i].Act) + "," + String(record[i].Name) + fl;
      //   }
      //   //Serial.println( message);
      //   EnvoyerSms(number, sms);
      // }
      else if (textesms.indexOf(F("ANTICIP")) > -1) { // Anticipation du wakeup
        if (textesms.indexOf(char(61)) == 7) {
          int n = textesms.substring(8, textesms.length()).toInt();
          if (n > 9 && n < 3601) {
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("TIME")) == 0) {
        message += F("Heure Sys = ");
        message += displayTime(0);
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure
        message += F("Mise a l'heure");
        // Sim800.reset(SIMPIN);// lancer SIM800
        if (gsm)MajHeure(Sim800.getTimeSms(slot)); // mise a l'heure du sms
        EffaceSMS(slot);
        if (nom != F("Moi meme")) EnvoyerSms(number, sms);
      }
      else if (gsm && textesms.indexOf(F("IMEI")) > -1) {
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("AUTOF")) == 0) {
        if ((textesms.indexOf(char(61))) == 5) { // =
          if (textesms.substring(6) == "1" || textesms.substring(6) == "0") {
            config.AutoF = textesms.substring(6).toInt();
            sauvConfig();	// sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("TEMPOAUTOF")) == 0) {
        if ((textesms.indexOf(char(61))) == 10) { // =
          if (textesms.substring(11).toInt() > 100 && textesms.substring(11).toInt() < 36000) {
            config.TempoAutoF = textesms.substring(11).toInt();
            sauvConfig();	// sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("LUMACTUELLE")) == 0) {
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("LUMAUTO")) == 0) {
        if ((textesms.indexOf(char(61))) == 7) { // =
          if (textesms.substring(8) == "1" || textesms.substring(8) == "0") {
            config.LumAuto = textesms.substring(8).toInt();
            sauvConfig();	// sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("LUMLUT")) > -1) { // Luminosité Look Up Table
        // format valeur de luminosité Feux pour chaque valeur luminosite ambiante
        // de 100 à 0 pas de 10
        // LUMLUT=95,90,80,75,60,50,40,30,30,30,30
        bool flag = true; // validation du format
        byte nv = 0; // compteur virgule
        byte p1 = 0; // position virgule
        if (textesms.indexOf("{") == 0) { // json
          DynamicJsonDocument doc(200);
          int f = textesms.lastIndexOf("}");
          // Serial.print("pos }:"),Serial.println(f);
          // Serial.print("json:"),Serial.print(textesms.substring(0,f+1)),Serial.println(".");
          DeserializationError err = deserializeJson(doc, textesms.substring(0, f + 1));
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
        else if ((textesms.indexOf(char(61))) == 6) { // =
          Sbidon = textesms.substring(7, textesms.length());
          for (int i = 0; i < Sbidon.length(); i++) {
            p1 = Sbidon.indexOf(char(44), p1 + 1); // ,
            if ((p1 > 0 && p1 < 255)) {
              nv ++;
              if (nv == 10)break;
            }
            else {
              flag = false;
            }
            // Serial.printf("%s%d,%s%d\n","p1=",p1,"flag=",flag);
          }
          // Serial.print("flag="),Serial.println(flag);
          // }
          // else {
          // flag = false;
          // }
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
        else { // données KO on enregistre pas, et on relie les donnéesnen mémoire
          OuvrirLumLUT();
        }
        if (smsserveur || !sms) {
          // si serveur reponse json
          DynamicJsonDocument doc(200);
          JsonArray lumlut = doc.createNestedArray("lumlut");
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MOIS")) > -1) { // Calendrier pour un mois
        /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
          demande calendrier pour un mois donné ; format : MOIS=mm? */
        bool flag = true; // validation du format
        bool W = true; // true Write, false Read
        int m = 0;
        if (textesms.indexOf("{") == 0) { // json
          DynamicJsonDocument doc(540);
          int f = textesms.lastIndexOf("}");
          // Serial.print("pos }:"),Serial.println(f);
          // Serial.print("json:"),Serial.print(textesms.substring(0,f+1)),Serial.println(".");,1,1,1,1,1,0,0,0,0,0,0]}";
          DeserializationError err = deserializeJson(doc, textesms.substring(0, f + 1));
          if(!err){
            m = doc["MOIS"]; // 12
            JsonArray jour = doc["JOUR"];
            for (int j = 1; j < 32; j++) {
              calendrier[m][j] = jour[j - 1];
            }
            // Serial.print("mois:"),Serial.println(m);
            EnregistreCalendrier(); // Sauvegarde en SPIFFS
            // message += F("Mise a jour calendrier \nmois:");
            // message += m;
            // message += " OK (json)";
          }
          else{
            message += " erreur json ";
            flag = false;
          }
        }
        else { // message normal mois=12,31*0/1
          byte p1 = textesms.indexOf(char(61)); // =
          byte p2 = textesms.indexOf(char(44)); // ,
          if (p2 == 255) {                      // pas de ,
            p2 = textesms.indexOf(char(63));    // ?
            W = false;
          }

          m = textesms.substring(p1 + 1, p2).toInt(); // mois

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
              // message += F("Mise a jour calendrier mois:");
              // message += m;
              // message += " OK";
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
            DynamicJsonDocument doc(540);
            doc["mois"] = m;
            JsonArray jour = doc.createNestedArray("jour");
            for (int i = 1; i < 32; i++) {
              jour.add(calendrier[m][i]);
            }
            String jsonbidon;
            serializeJson(doc, jsonbidon);
            message += jsonbidon;
            // message +="{\"mois\":" + String(m) + "," +fl;
            // message += "\"jour\":[";
            // for (int i = 1; i < 32 ; i++){
            // message += String(calendrier[m][i]);
            // if (i < 31) message += ",";
            // }
            // message += "]}";
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
          // if (sms)EffaceSMS(slot);
          SignalVie();
          // action_wakeup_reason(4);
        }
      }
      else if (textesms == F("NONCIRCULE")) {
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
        EnvoyerSms(number, sms);
        if (ok) {
          // if (sms){
          //   EffaceSMS(slot);
          // }
          Extinction();
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
      else if (textesms.indexOf(F("CALIBRATION=")) == 0) {
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
          // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(Id.substring(5, 9)) == 1) { // cherche CVXX
        if (textesms.indexOf("D") == 0) {
          Extinction(); // Violet 0, Blanc 0
          MajLog(nom, "DCV");
        }
        else if (textesms.indexOf("F") == 0) {
          if(Feux < 5 || Feux == 7){ // si Carré fermé ne rien faire
            EffaceAlaCdeFBlc();
            Feux = 1;
            Allumage(); // Violet 1, Blanc 0
            MajLog(nom, "FCV");
          }
        }
        else if(textesms.indexOf("O") == 0 || textesms.indexOf("M") == 0 || textesms.indexOf("S") == 0 || textesms.indexOf("V") == 0){
          if(FlagTqt_1){ // taquet ouvert
            if (textesms.indexOf("O") == 0) {
              EffaceAlaCdeFBlc();
              Feux = 2;
              Allumage(); // Violet 0, Blanc 1
              MajLog(nom, "OCV");
              if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
            }
            else if (textesms.indexOf("M") == 0) {
              EffaceAlaCdeFBlc();
              Feux = 3;
              Allumage(); // Violet 0, Blanc Manoeuvre Cli lent
              MajLog(nom, "MCV");
              // if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
            }
            else if (textesms.indexOf("S") == 0) {
              EffaceAlaCdeFBlc();
              Feux = 4;
              Allumage(); // Violet 0, Blanc Secteur Cli rapide
              MajLog(nom, "SCV");
              if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
            }
            else if (textesms.indexOf("V") == 0) {
              EffaceAlaCdeFBlc();
              Feux = 7;
              Allumage(); // Violet Cli, Blanc 0
              MajLog(nom, "VCV");
              if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
            }
          } else { // taquet fermé
            FlagDemande_Feux = true;
            Memo_Demande_Feux[0] = nom;      // nom demandeur
            Memo_Demande_Feux[1] = number;   // num demandeur
            Memo_Demande_Feux[2] = textesms; // demande d'origine
            Feux = 5; // Violet 1, Blanc 0
            MajLog(nom, "CCV demande : " + textesms);
            // Serial.println("memo demande feux :");
            // Serial.println(Memo_Demande_Feux[0]);
            // Serial.println(Memo_Demande_Feux[1]);
            // Serial.println(Memo_Demande_Feux[2]);
          }
        }
        else {
          // message += "non reconnu" + fl;
        }
        generationMessage(0);
        if (Feux != 0) { // seulement si different de DCV, doublon DCV envoie automatiquement une reponse dans Extinction()
          envoieGroupeSMS(3, 0); // envoie serveur
        }
        // evite de repondre 2 fois au serveur
        if (!smsserveur)EnvoyerSms(number, sms); // reponse si pas serveur
      }
      else if (textesms.indexOf(F("FBLCPWM")) == 0) {
        if (textesms.substring(7, 8) == "=") {
          int i = textesms.substring(8, textesms.length()).toInt();
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("FVLTPWM")) == 0) {
        if (textesms.substring(7, 8) == "=") {
          int i = textesms.substring(8, textesms.length()).toInt();
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SLOWBLINKER")) == 0) {
        if (textesms.substring(11, 12) == "=") {
          int i = textesms.substring(12, textesms.length()).toInt();
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("FASTBLINKER")) == 0) {
        if (textesms.substring(11, 12) == "=") {
          int i = textesms.substring(12, textesms.length()).toInt();
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("FASTRATER")) == 0) {
        if (textesms.substring(9, 10) == "=") {
          int i = textesms.substring(10, textesms.length()).toInt();
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("PARAM")) >= 0) {
        //message param divisé en 2 trop long depasse long 1sms 160c
        bool erreur = false;
        // Serial.print("position X:"),Serial.println(textesms.substring(7, 8));
        if(textesms.substring(7, 8) == "1"){ // PARAM1
        // Serial.print("position ::"),Serial.println(textesms.substring(9, 10));
          if (textesms.substring(9, 10) == ":") {
            // json en reception sans lumlut
            DynamicJsonDocument doc(200);
            DeserializationError err = deserializeJson(doc, textesms);
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
        else if(textesms.substring(7, 8) == "2"){ // PARAM2
          if (textesms.substring(9, 10) == ":") {
            // json en reception sans lumlut
            DynamicJsonDocument doc(200);
            DeserializationError err = deserializeJson(doc, textesms);
            if(err){
              erreur = true;
            }
            else{
              // Serial.print(F("Deserialization succeeded"));
              JsonObject param = doc["PARAM2"];
              config.LumAuto = param["LUMAUTO"];
              config.FBlcPWM = param["FBLCPWM"];
              config.FVltPWM = param["FVLTPWM"];
              config.AutoF = param["AUTOF"];
              config.TempoAutoF = param["TEMPOAUTOF"];
              sauvConfig();
            }
          }
        }
        if(!erreur){
          // ne fonctionne pas
          // const size_t capacity = JSON_ARRAY_SIZE(11) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(11);
          // calculer taille https://arduinojson.org/v6/assistant/
          DynamicJsonDocument doc(500);
          JsonObject param = doc.createNestedObject("param");
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

          JsonArray param_lumlut = param.createNestedArray("lumlut");
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("E1ACTIVE")) == 0) {
        bool valid = false;
        if (textesms.substring(8, 9) == "=") {
          if (textesms.substring(9, 10) == "1") {
            if (!config.Ip1) {
              config.Ip1 = true;
              FlagTqt_1 = false;
              sauvConfig();
              // ActiveInterrupt();
              valid = true;
              MajLog(nom, textesms);
            }
          }
          else if (textesms.substring(9, 10) == "0") {
            if (config.Ip1) {
              config.Ip1 = false;
              FlagTqt_1 = true;
              sauvConfig();
              // DesActiveInterrupt();
              valid = true;
              MajLog(nom, textesms);
            }
          }
          if (valid) {
            sauvConfig();															// sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("E2ACTIVE")) == 0) {
        bool valid = false;
        if (textesms.substring(8, 9) == "=") {
          if (textesms.substring(9, 10) == "1") {
            if (!config.Ip2) {
              config.Ip2 = true;
              sauvConfig();
              // ActiveInterrupt();
              valid = true;
            }
          }
          else if (textesms.substring(9, 10) == "0") {
            if (config.Ip2) {
              config.Ip2 = false;
              sauvConfig();
              // DesActiveInterrupt();
              valid = true;
            }
          }
          if (valid) {
            sauvConfig();															// sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (gsm && textesms.indexOf(F("UPLOADLOG")) == 0) {//upload log
        message += F("lancement upload log");
        message += fl;
        MajLog(nom, "upload log");// renseigne log
        Sbidon = String(config.apn);
        Sim800.activateBearerProfile(config.apn); // ouverture GPRS

        Serial.println(F("Starting..."));
        int reply = gprs_upload_function (); // Upload fichier
        Serial.println("The end... Response: " + String(reply));

        if(reply == 0){
          message += F("upload OK");
          LittleFS.remove(filelog);  // efface fichier log
          MajLog(nom, "");         // nouveau log
          MajLog(nom, F("upload OK"));// renseigne nouveau log
        } else{
          message += F("upload fail");
          MajLog(nom, F("upload fail"));// renseigne log
        }
        Sim800.deactivateBearerProfile(); // fermeture GPRS
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("FTPDATA") > -1) {
      // Parametres FTPDATA=Serveur:User:Pass:port
      // {"FTPDATA":{"serveur":"dd.org","user":"user","pass":"pass",,"port":00}}
      bool erreur = false;
      bool formatsms = false;
      if (textesms.indexOf(":") == 10) { // format json
        DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
        DeserializationError err = deserializeJson(doc, textesms);
        if (err) {
          erreur = true;
        }
        else {
          JsonObject ftpdata = doc["FTPDATA"];
          strncpy(config.ftpServeur,  ftpdata["serveur"], 26);
          strncpy(config.ftpUser,     ftpdata["user"],    11);
          strncpy(config.ftpPass,     ftpdata["pass"],    16);
          config.ftpPort         =    ftpdata["port"];
          sauvConfig();													// sauvegarde en EEPROM
        }
      }
      else if ((textesms.indexOf(char(61))) == 7) { // format sms
        formatsms = true;
        byte w = textesms.indexOf(":");
        byte x = textesms.indexOf(":", w + 1);
        byte y = textesms.indexOf(":", x + 1);
        byte zz = textesms.length();
        if (textesms.substring(y + 1, zz).toInt() > 0) { // Port > 0
          if ((w - 7) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16) {
            Sbidon = textesms.substring(7, w);
            Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
            Sbidon = textesms.substring(w + 1, x);
            Sbidon.toCharArray(config.ftpUser, (Sbidon.length() + 1));
            Sbidon = textesms.substring(x + 1, y);
            Sbidon.toCharArray(config.ftpPass, (Sbidon.length() + 1));
            config.ftpPort = textesms.substring(y + 1, zz).toInt();
            sauvConfig();													// sauvegarde en EEPROM
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
          DynamicJsonDocument doc(210);
          JsonObject FTPDATA = doc.createNestedObject("FTPDATA");
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
      EnvoyerSms(number, sms);
    }
      else if (textesms.indexOf("FTPSERVEUR") == 0) { // Serveur FTP
        // case sensitive
        // FTPSERVEUR=xyz.org
        if (textesms.indexOf(char(61)) == 10) {
          Sbidon = textesms.substring(11);
          Serial.print("ftpserveur:"),Serial.print(Sbidon);
          Serial.print(" ,"), Serial.println(Sbidon.length());
          Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
          sauvConfig();
        }
        message += F("FTPserveur =");
        message += String(config.ftpServeur);
        message += F("\n au prochain demarrage");
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("GPRSDATA")) > -1) {
        // Parametres GPRSDATA = "APN":"user":"pass"
        // GPRSDATA="sl2sfr":"":""
        // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
        bool erreur = false;
        bool formatsms = false;
        if (textesms.indexOf(":") == 11) { // format json
          DynamicJsonDocument doc(120);
          DeserializationError err = deserializeJson(doc, textesms);
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
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        else if ((textesms.indexOf(char(61))) == 8) { // format sms
          formatsms = true;
          byte cpt = 0;
          byte i = 9;
          do { // compte nombre de " doit etre =6
            i = textesms.indexOf('"', i + 1);
            cpt ++;
          } while (i <= textesms.length());
          Serial.print("nombre de \" :"), Serial.println(cpt);
          if (cpt == 6) {
            byte x = textesms.indexOf(':');
            byte y = textesms.indexOf(':', x + 1);
            byte z = textesms.lastIndexOf('"');
            // Serial.printf("%d:%d:%d\n",x,y,z);
            // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
            if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
              Sbidon = textesms.substring(10, x - 1);
              Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
              Sbidon = textesms.substring(x + 1 + 1 , y - 1);
              Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
              Sbidon = textesms.substring(y + 1 + 1, z);
              Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

              // Serial.print("apn:"),Serial.println(config.apn);
              // Serial.print("user:"),Serial.println(config.gprsUser);
              // Serial.print("pass:"),Serial.println(config.gprsPass);

              sauvConfig();													// sauvegarde en EEPROM
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
            DynamicJsonDocument doc(120);
            JsonObject gprsdata = doc.createNestedObject("GPRSDATA");
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
        EnvoyerSms(number, sms);
      }
      else if (textesms == "RSTALACDEFBLC") {
        // demande reset Alarme Cde Feu Blanc
        EffaceAlaCdeFBlc();
        message += "Reset Alarme en cours";
        EnvoyerSms(number, sms);
      }
      else if (textesms == "VIDELOG"){
        LittleFS.remove(filelog);
        FileLogOnce = false;
        message += "Effacement fichier log";
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SENDAT")) == 0){
        // envoie commande AT au SIM800
        // ex: SENDAT=AT+CCLK="23/07/19,10:00:20+04" mise à l'heure
        // attention DANGEREUX pas de verification!
        if (textesms.indexOf(char(61)) == 6) {
          String CdeAT = textesms.substring(7, textesms.length());
          String reply = sendAT(CdeAT,"OK","ERROR",1000);
          // Serial.print("reponse: "),Serial.println(reply);
          message += String(reply);
          EnvoyerSms(number, sms);
        }
      }
      else if (textesms.indexOf(F("TYPEBATT")) == 0){ // Type Batterie
        if (textesms.indexOf(char(61)) == 8) {
          int type = textesms.substring(9, textesms.length()).toInt();
          if(type == 16 || type == 24){
            config.TypeBatt = type;
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        message += "Type Batterie:" + fl;
        if(config.TypeBatt == 16) message += "Pb 12V";
        if(config.TypeBatt == 24) message += "LiFePO 12.8V";
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
      Sbidon = F("Appelant non reconnu ! ");
      Sbidon += String(numero);
      Serial.println(Sbidon);
      MajLog("Auto", Sbidon);// renseigne log
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
  if (FlagAlarmeCdeFBlc != FlagLastAlarmeCdeFBlc) {
    SendEtat = true;
    MajLog(F("Auto"), "Alarme Cde FBlc");
    FlagLastAlarmeCdeFBlc = FlagAlarmeCdeFBlc;
  }
  if (SendEtat) { 						// si envoie Etat demandé
    envoieGroupeSMS(0, 0);		// envoie groupé
    SendEtat = false;					// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeSMS(byte grp, bool m) {
  if (gsm) {
    /* m=0 message normal/finanalyse
    	si grp = 0,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
      si grp = 1,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
      de la liste restreinte config.Pos_Pn_PB[x]=1
      si grp = 3,
      Message au Serveur seulement N°1 de la liste			*/

    byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
    // Serial.print(F("Nombre de ligne PB=")),Serial.println(n);
    if (grp == 3) n = 1; // limite la liste à ligne 1
    for (byte Index = 1; Index < n + 1; Index++) { // Balayage des Num Tel dans Phone Book
      if ((grp == 3) || (grp == 0) || (grp == 1 && config.Pos_Pn_PB[Index] == 1)) {
        String number = Sim800.getPhoneBookNumber(Index);
        generationMessage(m);
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
    }
  }
}
//---------------------------------------------------------------------------
void generationMessage(bool n) {
  // n = 0 message normal
  // n = 1 message fin analyse
  messageId();
  if (FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarme24V || FlagAlarmeCdeFBlc) {
    message += F("--KO--------KO--");
  }
  else {
    message += F("-------OK-------");
  }
  message += fl;
  
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
    message += "Jour Circule" + fl;
  }
  else {
    message += "Jour Non Circule" + fl;
  }
  if(FlagAlarmeCdeFBlc){
    message += "Defaut Cde Feu Blanc" + fl;
  }
}
//---------------------------------------------------------------------------
void EnvoyerSms(char *num, bool sms) {

  if (sms && gsm) { // envoie sms
    message.toCharArray(replybuffer, message.length() + 1);
    bool OK = Sim800.sendSms(num, replybuffer);
    if (OK) {
      Serial.print(F("send sms OK:"));
      Serial.println(num);
    }
  }
  Serial.print (F("Message (long) = ")), Serial.println(message.length());
  Serial.println(message);
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  if (gsm) {
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
}
//---------------------------------------------------------------------------
void MajHeure(String smsdate) {
  if (gsm) {
    /*parametrage du SIM800 a faire une fois
      AT+CLTS? si retourne 0
      AT+CLTS=1
      AT+CENG=3
      AT&W pour sauvegarder ce parametre
      si AT+CCLK? pas OK
      avec Fonatest passer en GPRS 'G', envoyer 'Y' la sync doit se faire, couper GPRS 'g'
      't' ou AT+CCLK? doit donner la date et heure réseau
      format date retourné par Fona "yy/MM/dd,hh:mm:ss±zz",
      +CCLK: "14/08/08,02:25:43-16" -16= décalage GMT en n*1/4heures(-4) */
    if (smsdate.length() > 1) { // si smsdate present mise a l'heure forcé
      Sim800.SetTime(smsdate);
      Serial.print("Heure du SMS: "),Serial.println(smsdate);
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
          //on s'envoie à soi même un SMS "MAJHEURE" supprimé V2-16
          // message = F("MAJHEURE");
          // char numchar[13];
          // String numstring = Sim800.getNumTel();
          // numstring.toCharArray(numchar, 13);
          // MajLog("Auto", "envoie sms soi meme pour majheure");// renseigne log
          // EnvoyerSms(numchar, true);

          // Mise à l'heure par défaut V2-16
          Sim800.SetTime("22/08/01,08:00:00+08"); // 01/08 jour toujours circule
          Nyear   = 22;
          Nmonth  = 8;
          Nday    = 1;
          Nhour   = 8;
          Nminute = 0;
          Nsecond = 0;
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
        Alarm.disable(DebutJour);
        Alarm.disable(FinJour);
        Alarm.disable(Auto_F);

        setTime(Nhour, Nminute, Nsecond, Nday, Nmonth, Nyear);

        Alarm.enable(loopPrincipale);
        Alarm.enable(DebutJour);
        Alarm.enable(FinJour);
        if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
      }
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
  if (gsm) {
    MajHeure("");
    // envoieGroupeSMS(0, 0);
    Sim800.delAllSms();// au cas ou, efface tous les SMS envoyé/reçu
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
  envoieGroupeSMS(0, 0);
  action_wakeup_reason(4);
}
//---------------------------------------------------------------------------
// void sauvConfig() { // sauve configuration en EEPROM
//   EEPROM.begin(512);
//   EEPROM.put(confign, config);
//   EEPROM.commit();
//   EEPROM.end();
// }
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
// void logRecord(String nom, String action) { // renseigne log et enregistre EEPROM
//   static int index = 0;
//   String temp;
//   if (month() < 10) {
//     temp =  "0" + String(month());
//   }
//   else {
//     temp = String(month());
//   }
//   if (day() < 10 ) {
//     temp += "0" + String(day());
//   }
//   else {
//     temp += String(day());
//   }
//   if (hour() < 10) {
//     temp += "-0" + String(hour());
//   }
//   else {
//     temp += "-" + String(hour());
//   }
//   if (minute() < 10) {
//     temp += "0" + String(minute());
//   }
//   else {
//     temp += String(minute());
//   }
//   temp  .toCharArray(record[index].dt, 10);
//   nom   .toCharArray(record[index].Name, 15);
//   action.toCharArray(record[index].Act, 2);

//   EEPROM.begin(512);
//   EEPROM.put(recordn, record);// ecriture des valeurs par defaut
//   EEPROM.commit();
//   EEPROM.end();
//   if (index < 4) {
//     index ++;
//   }
//   else {
//     index = 0;
//   }
// }
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

  File file = LittleFS.open(fileconfig);
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

  File file = LittleFS.open(fileconfig, FILE_WRITE);
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
void readFile(fs::FS &fs, const char * path){
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
void writeFile(fs::FS &fs, const char * path, const char * message){
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
void MajLog(String Id, String Raison) { // mise à jour fichier log en LittleFS
  if(LittleFS.exists(filelog)){
    /* verification de la taille du fichier */
    File f = LittleFS.open(filelog, "r");
    Serial.print(F("Taille fichier log = ")), Serial.println(f.size());
    // Serial.print(Id),Serial.print(","),Serial.println(Raison);
    if (f.size() > 150000 && !FileLogOnce) {
      /* si trop grand on efface */
      FileLogOnce = true;
      messageId();
      message += F("Fichier log presque plein\n");
      message += String(f.size());
      message += F("\nFichier sera efface a 300000");
      if (gsm) {
        String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
    }
    else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
      messageId();
      message += F("Fichier log plein\n");
      message += String(f.size());
      message += F("\nFichier efface");
      if (gsm) {
        String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
      LittleFS.remove(filelog);
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
    appendFile(LittleFS, filelog, Cbidon);
  }
  else{ // fichier n'existe pas, création fichier avec pr mière ligne date et Id
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d;", day(), month(), year(), hour(), minute(), second());
    strcat(Cbidon,config.Idchar);
    strcat(Cbidon,fl.c_str());
    appendFile(LittleFS, filelog, Cbidon);
    Serial.print("nouveau fichier log:"),Serial.println(Cbidon);
  }
}
//---------------------------------------------------------------------------
void EnregistreCalendrier() { // remplace le calendrier

  LittleFS.remove(filecalendrier);
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
    appendFile(LittleFS, filecalendrier, bid);
    Sbidon = "";
  }
}
//---------------------------------------------------------------------------
void EnregistreLumLUT() {
  LittleFS.remove(filelumlut);
  char bid[9];
  for (int i = 0; i < 11; i++) {
    sprintf(bid, "%d,%d\n", TableLum[i][0], TableLum[i][1]);
    appendFile(LittleFS, filelumlut, bid);
  }
}
//---------------------------------------------------------------------------
void OuvrirLumLUT() {
  if (LittleFS.exists(filelumlut)) {
    File f = LittleFS.open(filelumlut, "r");
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
      appendFile(LittleFS, filelumlut, bid);
      TableLum[i][0] = v1;
      TableLum[i][1] = v2;
    }
  }
  for (int i = 0; i < 11 ; i++) {
    Serial.print(TableLum[i][0]), Serial.print(","), Serial.println(TableLum[i][1]);
  }
}
//---------------------------------------------------------------------------
int lumlut(int l) {
  // retourn la valeur lut en fonction de lum actuelle
  for (int i = 0; i < 11; i++) {
    if (l >= TableLum[i][0]) {
      // Serial.printf("%s%d,%d\n","lumlut=",l,TableLum[i][1]);
      return TableLum[i][1];;
    }
  }
  return 0;
}
//---------------------------------------------------------------------------
void OuvrirCalendrier() {

  // this opens the file "f.txt" in read-mode
  listDir(LittleFS, "/", 0);
  bool f = LittleFS.exists(filecalendrier);
  // Serial.println(f);
  File f0 = LittleFS.open(filecalendrier, "r");

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
      appendFile(LittleFS, filecalendrier, bid);
      Sbidon = "";
    }
  }
  readFileCalendrier(LittleFS, filecalendrier);

  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Serial.print(calendrier[m][j]), Serial.print(char(44));
    }
    Serial.println();
  }
  listDir(LittleFS, "/", 0);

}
//---------------------------------------------------------------------------
void FinJournee() {
  // fin de journée retour deep sleep
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
  Serial.print(F("GPRS APN = ")), Serial.println(config.apn);
  Serial.print(F("GPRS user = ")), Serial.println(config.gprsUser);
  Serial.print(F("GPRS pass = ")), Serial.println(config.gprsPass);
  Serial.print(F("ftp serveur = ")), Serial.println(config.ftpServeur);
  Serial.print(F("ftp port = ")), Serial.println(config.ftpPort);
  Serial.print(F("ftp user = ")), Serial.println(config.ftpUser);
  Serial.print(F("ftp pass = ")), Serial.println(config.ftpPass);
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
    server.on("/dir",      LittleFS_dir);
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
  EnvoyerSms(number, sms);

  // if (sms) { // suppression du SMS
  //   /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
  //     ou OTA sms demande Wifi toujours present */
  //   EffaceSMS(slot);
  // }
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

  if (LittleFS.exists(filecalibration)) {
    File f = LittleFS.open(filecalibration, "r");
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
void Recordcalib() { // enregistrer fichier calibration en LittleFS
  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = LittleFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.println(CoeffTension[2]);
  f.println(CoeffTension[3]);
  f.close();
}
//---------------------------------------------------------------------------
String Hdectohhmm(long Hdec) {
  // convert heure decimale en hh:mm:ss
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
long Hhmmtohdec(String h){
  // convert heure hh:mm:ss en decimale
  int H = h.substring(0,2).toInt();
  int M = h.substring(3,5).toInt();
  int S = h.substring(6,8).toInt();
  long hms = H*3600 + M*60 + S;
  return hms;
}
//---------------------------------------------------------------------------
// void DesActiveInterrupt() {
//   if (config.Ip1) {
//     detachInterrupt(digitalPinToInterrupt(PinIp1));
//   }
//   if (config.Ip2) {
//     detachInterrupt(digitalPinToInterrupt(PinIp2));
//   }
// }
//---------------------------------------------------------------------------
// void ActiveInterrupt() {
//   if (config.Ip1) {
//     attachInterrupt(digitalPinToInterrupt(PinIp1), handleInterruptIp1, FALLING);
//   }
//   if (config.Ip2) {
//     attachInterrupt(digitalPinToInterrupt(PinIp2), handleInterruptIp2, FALLING);
//   }
// }
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
    while (!Sim800.sleep()) {
      Alarm.delay(100);
      if (i++ > 10) break;
    }
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
      if (FirstWakeup) { // premier wake up du jour avant DebutJour
        // SignalVie();
        // ne rien faire, attendre DebutJour
        FirstWakeup = false;
        if (HActuelledec() > config.DebutJour) {
          // premier lancement en journée
          SignalVie();
        }
        break;
      }
      if ((calendrier[month()][day()] ^ flagCircule) && jour) { // jour circulé & jour
        // Sbidon = F("Jour circule ou demande circulation");
        // Serial.println(Sbidon);
        // MajLog(F("Auto"), Sbidon);
        // Feux = 1;
        // Allumage(); // Violet 1, Blanc 0
        // MajLog("Auto", "FCV");
        // envoieGroupeSMS(0, 0);
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
void adc_read() {
  read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V, PinLum); // lecture des adc
}
//---------------------------------------------------------------------------
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
void Tel_listPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Liste des num&eacute;ros t&eacute;l&eacute;phone</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<th> Nom </th>");
  webpage += F("<th> Num&eacute;ro </th>");
  webpage += F("<th> Liste restreinte </th>");
  webpage += F("</tr>");
  if (gsm) {
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
  if (LittleFS_present) {
    File download = LittleFS.open("/" + filename,  "r");
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportLittleFSNotPresent();
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
    LittleFS.remove(filename);                  // Remove a previous version, otherwise data is appended the file again
    UploadFile = LittleFS.open(filename, "w");  // Open the file for writing in LittleFS (create it, if doesn't exist)
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
void LittleFS_dir() {
  if (LittleFS_present) {
    File root = LittleFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      webpage += F("<h3 class='rcorners_m'>LittleFS Contents</h3><br>");
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
  } else ReportLittleFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  File root = LittleFS.open(dirname);
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
    if (server.hasArg("delete")) LittleFS_file_delete(server.arg(0));
  }
  else SelectInput("Select a File to Delete", "delete", "delete");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void LittleFS_file_delete(String filename) { // Delete the file
  if (LittleFS_present) {
    SendHTML_Header();
    File dataFile = LittleFS.open("/" + filename, "r"); // Now read data from LittleFS Card
    if (dataFile)
    {
      if (LittleFS.remove("/" + filename)) {
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
  } else ReportLittleFSNotPresent();
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
void ReportLittleFSNotPresent() {
  SendHTML_Header();
  webpage += F("<h3>No LittleFS Card present</h3>");
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
//---------------------------------------------------------------------------
byte gprs_upload_function (){
  // https://forum.arduino.cc/index.php?topic=376911.15
  int buffer_space = 1000;
  UploadFile = LittleFS.open(filelog, "r");
  byte reply = 1;
  int i = 0;
  // ne fonctionne pas dans tous les cas ex roaming
  // while (i < 10 && reply == 1){ //Try 10 times...
    // reply = sendATcommand("AT+CREG?","+CREG: 0,1","ERROR", 1000);
    // i++;
    // delay(1000);
  // }
  reply = 0;
if (reply == 0){ // ouverture GPRS
// reply = sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"","OK","ERROR", 1000);
if (reply == 0){
// reply = sendATcommand("AT+SAPBR=3,1,\"APN\",\"sl2sfr\"", "OK", "ERROR", 1000);//Replace with your APN
if (reply == 0){
//reply = sendATcommand("AT+SAPBR=3,1,\"USER\",\"entelpcs\"", "OK", "ERROR", 1000);
if (reply == 0){
//reply = sendATcommand("AT+SAPBR=3,1,\"PWD\",\"entelpcs\"", "OK", "ERROR", 1000);
if (reply == 0){
reply = 2;
i = 0;
while (i < 3 && reply == 2){ //Try 3 times...
  reply = sendATcommand("AT+SAPBR=1,1", "OK", "ERROR", 10000);
  if (reply == 2){
    sendATcommand("AT+SAPBR=0,1", "OK", "ERROR", 10000);
  }
  i++;
}
if (reply == 0){
reply = sendATcommand("AT+SAPBR=2,1", "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPCID=1", "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPSERV=\"" + String(config.ftpServeur) + "\"", "OK", "ERROR", 1000);//replace ftp.sample.com with your server address
if (reply == 0){
reply = sendATcommand("AT+FTPPORT="+ String(config.ftpPort), "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPUN=\"" + String(config.ftpUser) + "\"", "OK", "ERROR", 1000);//Replace 1234@sample.com with your username
if (reply == 0){
reply = sendATcommand("AT+FTPPW=\"" + String(config.ftpPass) + "\"", "OK", "ERROR", 1000);//Replace 12345 with your password
if (reply == 0){
reply = sendATcommand("AT+FTPPUTNAME=\"" + String(filelog) + "\"", "OK", "ERROR", 1000);
if (reply == 0){
  reply = sendATcommand("AT+FTPPUTPATH=\"/" + String(config.Idchar) + "/\"", "OK", "ERROR", 1000);// repertoire "/Id/"
if (reply == 0){
  unsigned int ptime = millis();
  reply = sendATcommand("AT+FTPPUT=1", "+FTPPUT: 1,1", "+FTPPUT: 1,6", 60000);
  Serial.println("Time: " + String(millis() - ptime));
  if (reply == 0){
    if (UploadFile) {
      int i = 0;
      unsigned int ptime = millis();
      long archivosize = UploadFile.size();
      while (UploadFile.available()) {
        while(archivosize >= buffer_space){
          reply = sendATcommand("AT+FTPPUT=2," + String(buffer_space), "+FTPPUT: 2,1", "OK", 3000);
            if (reply == 0) { //This loop checks for positive reply to upload bytes and in case or error it retries to upload
              Serial.println("Remaining Characters: " + String(UploadFile.available()));
              for(int d = 0; d < buffer_space; d++){
                Serial2.write(UploadFile.read());
                archivosize -= 1;
              }
            }
            else {
              Serial.println("Error while sending data:");
              reply = 1;
            }
        }
        if (sendATcommand("AT+FTPPUT=2," + String(archivosize), "+FTPPUT: 2," + String(archivosize), "ERROR", 10000) == 0) {
          for(int t = 0; t < archivosize; t++){
            Serial2.write(UploadFile.read());
          }
        }
      }
    UploadFile.close();
    Serial.println("Time: " + String(millis() - ptime));
    }
  }
}
}
}
}
}
}
}
}
}
}
}
}
}
}
  sendATcommand("AT+SAPBR=0,1", "OK", "ERROR", 10000); // fermeture GPRS
return reply;
}
//---------------------------------------------------------------------------
byte sendATcommand(String ATcommand, String answer1, String answer2, unsigned int timeout){
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
    //Stop reading conditions
    if (content.indexOf(answer1) != -1){
      reply = 0;
    }else if(content.indexOf(answer2) != -1){
      reply = 2;
    }else{
      //Nothing to do...
    }
  }
  return reply;
}
//---------------------------------------------------------------------------
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
void VerifCdeFBlc(){
  // si F != 2(Ouvert) on mesure entree PinChckFblc si == 0 Alarme
  // on mesure accumulation sur 2 secondes si ratio < 35%
  static unsigned long tmesure = millis();
  if (tmesure > millis()) tmesure = millis();
  static int compteurmesureres = 0;
  static int accumesureres = 0;
  int periodemesures = 2000;
  if(Feux != 2){
    if(millis()- tmesure > periodemesures){// periode mesure > periodemesures
      if(compteurmesureres > 1200){
        // pour eviter fausses alarmes quand proc occupé par ailleurs
        Serial.print("Cpt Cde FBLc:"),Serial.print(compteurmesureres);
        Serial.print(", accu:"),Serial.print(accumesureres);
        Serial.print(", %:"),Serial.println((float)accumesureres/compteurmesureres);
        if((float)accumesureres/compteurmesureres < .35 ){// .5 = M, .75 = S
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
    accumesureres += digitalRead(PinChckFblc);
    compteurmesureres ++;
  }
}
//---------------------------------------------------------------------------
void EffaceAlaCdeFBlc(){
  // efface l'alarme Cde Feu Blanc, pour rendre de nouveau
  // operationnel l'extinction en cas de nouveau probleme
  FlagAlarmeCdeFBlc = false;
  FlagLastAlarmeCdeFBlc = false;
}
//---------------------------------------------------------------------------
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
        }else if (Memo_Demande_Feux[2].indexOf("V") == 0) {
          // Serial.print("position V:"),Serial.println(Memo_Demande_Feux[2].indexOf("V"));
          Feux = 7;
          Allumage(); // Violet Cli, Blanc 0
          MajLog(Memo_Demande_Feux[0], "VCV");
          if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
        }
        generationMessage(0);
        char number[13];
        Memo_Demande_Feux[1].toCharArray(number, Memo_Demande_Feux[1].length() + 1);
        bool smsserveur = false;
        if (Memo_Demande_Feux[1] == Sim800.getPhoneBookNumber(1)) {
          smsserveur = true; // si demande provient du serveur index=1
        }
        if(Memo_Demande_Feux[0] != "console"){
          if(!smsserveur){
            EnvoyerSms(number, true); // reponse demandeur si pas serveur
          }
        }
        envoieGroupeSMS(3, 0); // envoie serveur
        FlagDemande_Feux = false; // efface demande
      }
      else{ // pas de demande, juste ouverture taquet, Feux = violet
        Feux = 1;
        Serial.println("Ouverture taquet");
        MajLog("Auto", "FCV");
        generationMessage(0);
        envoieGroupeSMS(3, 0); // envoie serveur
      }
    } else { // Taquet fermé
      Serial.println("Taquet ferme");
      Feux = 5; // Feux F + Carré
      Allumage();
      MajLog("Auto", "CCV");
      envoieGroupeSMS(3, 0); // envoie serveur
    }
  }
  FlagLastTqt_1 = FlagTqt_1;

  if(FlagTqt_2 != FlagLastTqt_2){ // Taquet v3 a changé d'etat
    if(FlagTqt_2){ // Taquet ouvert
      MajLog("Auto", "gestiontaquet Taquet V3 ouvert");
    } else {
      MajLog("Auto", "gestiontaquet Taquet V3 ferme");
    }
    envoieGroupeSMS(3, 0); // envoie serveur
    FlagLastTqt_2 = FlagTqt_2;
  }
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
