/* defs.h */
/* definition des variables, pour SMS, Phone Book, Network */

#include <Arduino.h>

  // enum class SmsStatus : uint8_t {
  // REC_UNREAD  = 0,
  // REC_READ    = 1,
  // STO_UNSENT  = 2,
  // STO_SENT    = 3,
  // ALL         = 4
  // };

  // enum class SmsAlphabet : uint8_t {
  // GSM_7bit   = B00,
  // Data_8bit  = B01,
  // UCS2       = B10,
  // Reserved   = B11
  // };

  // struct Sms {
  //   SmsStatus status;              // <stat>
  //   SmsAlphabet alphabet;          // alphabet part of TP-DCS
  //   String originatingAddress;     // <oa>
  //   String phoneBookEntry;         // <alpha>
  //   String serviceCentreTimeStamp; // <scts>, format: yy/MM/dd,hh:mm:ss±zz; zz: time zone, quarter of an hour
  //   String message;                // <data>
  // };

  struct Sms {
    String message;      // [101]
    String sendernumber; // [14]
    String sendername;   // [21]
    String timestamp;    // [21]
  };

struct PhonebookEntry {
  String number;
  String text;
};

struct NetworkInfo {
  String Net1; // Mode
  String Net3; // MCC-MNC Pays - operateur(01:Orange, 15:Free)
  String Net6; // Band (WCDMA or GSM)
  String Net7; // Band (LTE), PSC (WCDMA)
  String Net8; // ARFCN (LTE OR WCDMA)
};

