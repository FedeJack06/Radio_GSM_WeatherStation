/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////                                                                     ///////////////
  //////////////////////////          Weather station by Fedejack - MASTER 1.7                   ///////////////
  //////////////////////////                                                                     ///////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
//////////////////////////////////////LIBRERIE E DEFINIZIONI////////////////////////////////////////////////////
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "printf.h"
#include <MemoryFree.h>
bool openSerial = 0;

//TIME DS3231
#include <DS3232RTC.h>
#include <TimeLib.h>
DS3232RTC rtc;
#define pin_ds3231 19  //sql pin from rtc

//T/H DHT22 E DS18B20
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 40 // pin digitale a cui è collegato il DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define DHTPIN 38 // pin digitale a cui è collegato il DHT22
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//SHT35
#include "SHT31.h"
SHT31 sht35;
#define SHT31_ADDRESS   0x44

//PIN PLUVIOMETRO E ANEMOMETRO DeA////////
#define pinPluvio 3 // pin a cui è collegato il pluviometro reed
#define pinAnemometro 2 // pin a cui è collegato l'anemometro reed

//PIN DIREZIONE VENTO DeA///////////
#define pinEst A8//pin ANALOGICI collegamento banderuola
#define pinSud A9
#define pinWest A10
#define pinNord A11

//SD
#include <SdFat.h>
//#include </home/federico/Arduino/libraries/SdFat/src/iostream/fstream.h> //if not found fstream
#define SD_SS 36 // pin usato dalla microSD
SdFat SD;

//Radio
//#include "Arduino.h"
#include "LoRa_E220.h"
LoRa_E220 e220ttl(A14, A12, 43, 33, A13); //TX RX AUX M0 M1
#define DESTINATION_ADDL 3

//VOLTAGGIO batteria
const int Pin_Batteria = A5;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////VARIABILI///////////////////////////////////////////////////////
//TEMPORIZZAZIONI
unsigned long previousMeasure = 0;
unsigned long previousDatalog = 0; //necessaria per salvataggio sd
unsigned long previousSend = 0; //start send after calc first average
bool writeFirst = false; //send after 1 min after first average

/////STRUCT RADIO////////////////////////////////////////////////////////////////////////////
struct radioVal { //traferimento dati via radio
  unsigned long date = 0; //trasferimento data allo slave per salvataggio dati
  int TP = 9999; //temperatura sht35 *100
  int TPDHT = 9999; //temperatura dht22 *100
  int TP2 = 9999; //temp ds18b20 *100
  int tpmax = 9999; //*100
  int tpmin = 9999; //*100
  int urmax = 9999; //*100
  int urmin = 9999; //*100
  unsigned int UR = 9999; //umiditá 16 *100
  byte ur2 = 255; //umidita dht22
  unsigned int windmedio = 9999; //velocitá vento m/s *100
  unsigned int windmax = 9999; //raffica giornaliera massima *100
  byte winddir = 255; //indice direzione vento, da usare per estrarre stringhe da Direzioni[]
  unsigned int pioggiaday = 9999; //pioggia del giorno *10
  unsigned int rainrate = 9999; //intensitá di pioggia *10
  unsigned int rainratemax = 9999; //max rr *10

  byte ndata = 255;
  byte errori = 255; //001 = 0 reset status, 0 ersd, 1 errore winddir
  byte volt = 255; //tensione batteria alimetazione
  byte send_count = 255;
  unsigned int memoryFree = freeMemory();
};
radioVal radioTx;

struct radioAck {
  unsigned long date = 0;
  bool resetRx = false;
};
radioAck radioRx;

/////////////////////////////////////////FUNZIONE FUSO ORARIO//////////////////////////////////
byte dstOffset (byte d, byte m, unsigned int y, byte h) {
  /* This function returns the DST offset for the current UTC time.
    This is valid for the EU, for other places see
    http://www.webexhibits.org/daylightsaving/i.html

    Results have been checked for 2012-2030 (but should work since
    1996 to 2099) against the following references:
    - http://www.uniquevisitor.it/magazine/ora-legale-italia.php
    - http://www.calendario-365.it/ora-legale-orario-invernale.html
  */

  // Day in March that DST starts on, at 1 am
  byte dstOn = (31 - (5 * y / 4 + 4) % 7);

  // Day in October that DST ends  on, at 2 am
  byte dstOff = (31 - (5 * y / 4 + 1) % 7);

  if ((m > 3 && m < 10) ||
      (m == 3 && (d > dstOn || (d == dstOn && h >= 1))) ||
      (m == 10 && (d < dstOff || (d == dstOff && h <= 1))))
    return 1;
  else
    return 0;

}

time_t getLocalTime (void) {
  time_t t = rtc.get ();

  TimeElements tm;
  breakTime (t, tm);
  t += (dstOffset (tm.Day, tm.Month, tm.Year + 1970, tm.Hour)) * SECS_PER_HOUR;

  return t;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////INIZIALIZZAZIONI/////////////////////////////////////////////////////////////////////////////////////////
//SD
bool ersd;
void initSD() {
  ersd = SD.begin(SD_SS, SPI_HALF_SPEED);//errore inizializzazione sd da inviare allo slave
  if (openSerial) {
    if (!ersd)
      Serial.println(F("SD error"));
    else
      Serial.println(F("SD ok"));
  }
}

//TEMPERATURA//
void initTemp () {
  sensors.begin();

  Wire.setClock(100000);
  sht35.begin();

  if (openSerial) {
    if ( sht35.isConnected() )
      Serial.println(F("sht35 ok"));
    else
      Serial.println(F("sht35 error"));
  }
}

//UMIDITA'/////
void initHum () {
  dht.begin();
}

//VENTO/////////
void initWind () {
  attachInterrupt(0, windCount, RISING);//interrupt wind speed
  analogReference(INTERNAL1V1);
  pinMode(pinEst, INPUT);//set pin input segnale banderuola
  pinMode(pinSud, INPUT);
  pinMode(pinWest, INPUT);
  pinMode(pinNord, INPUT);
}

//PIOGGIA///////
void initPioggia () {
  attachInterrupt(1, ContaGocce, RISING); //interrupt per il pluviometro
}

//DS3231
void initRTC() {
  rtc.begin();
  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_DATE, 0, 0, 0, 1);
  rtc.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, 0, 0, 1);
  rtc.alarm(DS3232RTC::ALARM_1);
  rtc.alarm(DS3232RTC::ALARM_2);
  rtc.alarmInterrupt(DS3232RTC::ALARM_1, false);
  rtc.alarmInterrupt(DS3232RTC::ALARM_2, false);
  rtc.squareWave(DS3232RTC::SQWAVE_NONE);
  /////////////////////imposto ora attuale
  //setTime(20, 17, 00, 20, 10, 2024); // the setTime() function is part of the Time library.
  //rtc.set(now());								// set the RTC from the system time
  setSyncProvider(getLocalTime);	//causes the Time library to synchronize with the external RTC by calling RTC.get()
  //setSyncInterval(600);         // set the number of seconds between re-sync //5 min default

  // set Alarm 1
  rtc.setAlarm(DS3232RTC::ALM1_EVERY_SECOND, 0, 0, 0, 0);
  //rtc.setAlarm(DS3232RTC::ALM1_MATCH_MINUTES, 0, (minute()/10)*10+10, 0, 0); //start measure next minute multiple of 10
  rtc.alarm(DS3232RTC::ALARM_1);  // clear the alarm flag
  rtc.alarmInterrupt(DS3232RTC::ALARM_1, true);

  //pinMode(pin_ds3231, INPUT_PULLUP); //board HW-084 have pullup
  pinMode(pin_ds3231, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_ds3231), alarm1, FALLING);
}

//Radio
void initRadio() {
  e220ttl.begin();
  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*) c.data;
  if (openSerial) {
    Serial.println(c.status.getResponseDescription());
    Serial.println(c.status.code);
  }

  printParameters(configuration);
  //  ----------------------- FIXED SENDER -----------------------
  configuration.ADDL = 0x02;
  configuration.ADDH = 0x00;
  configuration.CHAN = 23; //433 MHz

  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.subPacketSetting = SPS_200_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_22;

  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

  // Set configuration changed and set to not hold the configuration
  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  if (openSerial) {
    Serial.println(rs.getResponseDescription());
    Serial.println(rs.code);
  }

  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  configuration = *(Configuration*) c.data;
  if (openSerial) {
    Serial.println(c.status.getResponseDescription());
    Serial.println(c.status.code);
    printParameters(configuration);
  }
  c.close();
}

void printParameters(struct Configuration configuration) {
  Serial.println(F("-----------"));

  Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX); Serial.print(F(" ")); Serial.print(configuration.STARTING_ADDRESS, HEX); Serial.print(F(" ")); Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(F(" -> ")); Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN); Serial.print(F(" -> ")); Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN); Serial.print(F(" -> ")); Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN); Serial.print(F(" -> ")); Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN); Serial.print(F(" -> ")); Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN); Serial.print(F(" -> ")); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN); Serial.print(F(" -> ")); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN); Serial.print(F(" -> ")); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN); Serial.print(F(" -> ")); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN); Serial.print(F(" -> ")); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN); Serial.print(F(" -> ")); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

  Serial.println(F("------------"));
}

void initVolt() {
  pinMode (Pin_Batteria, INPUT);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////FUNZIONI PER LOOP: ACQUISIZIONE////////////////////////////////////

//////////////////////////TEMPERATURA///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
float mTp = 0, mTpdht = 0, mTp2 = 0; //somma medie delle temperature e contatore
unsigned int countTp = 0, countTpdht = 0, countTp2 = 0;
float TP = 999, TPDHT = 999, TP2 = 999; //medie delle temperature
float tpmax = -50, tpmin = 130;

void readTemp () {
  sht35.read(false); //lettura in slow mode sht35
  float tp = sht35.getTemperature();

  sensors.requestTemperatures(); //richiesta temp dal sensore dallas
  float tp2 = sensors.getTempCByIndex(0); //temp dallas

  float tpdht = dht.readTemperature(); //temperatura dht22

  //MEDIA
  if (tp >= -40 && tp <= 125) { //check if value in operating range of datasheet
    mTp += tp;
    countTp++;
  }
  TP = mTp / countTp;

  if (tpdht >= -40 && tpdht <= 125) {
    mTpdht += tpdht;
    countTpdht++;
  }
  TPDHT = mTpdht / countTpdht;

  if (tp2 >= -55 && tp2 <= 125) {
    mTp2 += tp2;
    countTp2++;
  }
  TP2 = mTp2 / countTp2;

  //ESTREMI
  if (tp > tpmax && tp >= -40 && tp <= 125)
    tpmax = tp;
  if (tp < tpmin && tp >= -40 && tp <= 125)
    tpmin = tp;
}

//////////////////////////UMIDITA///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
float mUr = 0; //somma medie delle temperature e contatore
unsigned int countUr = 0;
float UR = 999; //medie delle temperature
float urmax = 0, urmin = 100;
float ur2;

void readHum () {
  float ur = sht35.getHumidity(); //sht35
  ur2 = floor(dht.readHumidity()); //umiditá dht22

  //MEDIA
  if (ur >= 0 && ur <= 100) {
    mUr += ur;
    countUr ++;
  }
  UR = mUr / countUr;

  //ESTREMI
  if (ur > urmax)
    urmax = ur;
  if (ur < urmin)
    urmin = ur;
}

//////////////////////////VENTO/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

/////INTERRUPT WIND////////////////////////////////////////////////////////
const float Pi = 3.141593; // Pigreco
const float raggio = 0.055; // raggio dell'anemometro in metri
volatile unsigned long Conteggio = 0;// variabile che contiene il conteggio delle pulsazioni
//volatile unsigned long Tempo = 0; // per conteggiare il tempo trascorso dalla prma pulsazione
//volatile unsigned long tempoPrec = 0;
//volatile float deltaT;
volatile unsigned long prevWindInterrupt = 0; //antidebouncing e max valore 311 km/h
volatile float wind4sec = 0; //vento in m/s calcolato su un periodo di 4 secondi
/*
  METODO CALCOLO MISURANDO IL PERIODO DI UNA ROTAZIONE E PRENDENDO IL VALORE CHE "PASSA" DURANTE IL LOOP
  void windCount() {
    Tempo = millis();
    deltaT = float( Tempo - tempoPrec)/1000; // si conteggia il tempo trascorso fra un interrupt e un altro, poi si prendera l'ultimo valore quando passa il loop la funzione di calcolo del vento
    tempoPrec = Tempo;
  }
*/

void windCount() {
  if ( millis() - prevWindInterrupt > 10) { //anti debounce: max velocità instantanea = 100 m/s = 360 km/h
    Conteggio++;
    prevWindInterrupt = millis();
  }
}

ISR(TIMER1_COMPA_vect) { //funzione chiamata ogni 4 secondi
  //wind4sec = (2*Pi*raggio*Conteggio)/4; //metodo ideale
  wind4sec = float(Conteggio) * 0.2719; //*0.138; fattore correzione misurato con anemometro tarato
  Conteggio = 0;
}

void setWindClock() {
  //set timer 1
  TCCR1A = 0; //timer CTC mode
  TCCR1B = 0b00001101; //prescaler a 1024, (Da 16Mhz - 62,5 ns a 64 micro sec)
  /*
     Presclaer 001 010 011  100  101
                1   8   64  256  1024
     Timer Count = delay richiesto(s) / periodo prescaled(64us) - 1 = 62500-1

  */
  TCNT1 = 0; //contatore settato a zero
  TIMSK1 = 0b00000010; //set contatore uguale a OCR1A fa interrrupt

  OCR1A = 62500 - 1; //max valore del conteggio prima di fare overflow e attivare ISR
}

struct WindValue {
  float val = 999; //velocità vento m/s
  float valmedio = 999; //velocità media vento
  float max = 0; //valore massimo nell'intervallo della media
  byte dir; //indice array direzione del vento
  byte dirmedia; //indice array direzione vento media
  byte err; //indice errore direzione vento
};
WindValue wind;
unsigned int countWind = 0;
float mWind = 0;

void readWind () {
  /*DA USARE CON METODO VELOCITA ISTANTANEA, CALCOLO DELLA VELOCITA MISURANDO IL PERIODO DI UNA ROTAZIONE, SI PRENDE IL VALORE DI V CHE PASSA AL MOMENTO IN CUI SI ESEGUE readWind
    if (deltaT != 0)                   //se é trascorso tempo calcolo la velocitá
    wind.val = (2*Pi*raggio)/deltaT; //si calcola la velocitá come velocitá angolare diviso delta tempo

    else if (deltaT ==0 || deltaT > 2500){ //se non é trascorso tempo oppure velocitá minore 0.5 km/h
    wind.val = 0;
    }*/
  noInterrupts();
  wind.val = wind4sec; //misura velocità contando rotazioni in 4 secondi
  interrupts();

  //MEDIA
  if (wind.val >= 0 && wind.val <= 70) { //limite datasheet
    mWind += wind.val;
    countWind++;
  }

  wind.valmedio = mWind / countWind;

  //MAX
  if (wind.val > wind.max && wind.val <= 70)
    wind.max = wind.val;
}

//DIREZIONE DEL VENTO/////////////////////////////////////////////////////////////////////////////
//ogni misurazione viene incrementato di 1 l'array mediaDir nella posizione corrispondente alla direzione (vedi array direzioni)
//in modo da segnare quale valori direzione viene misurato piú volte (moda), si elimina l'errore di piccole oscillazione della banderuola
int mediaDir[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //array della frequenza della direzione del vento
int maxDir = 0;//indice di direzione che si ripete più volte nell'intervallo
//char direzioni[16][4] = {"NNE","NE","ENE","E","ESE","SE","SSE","S","SSO","SO","OSO","O","ONO","NO","NNO","N"};
//char erdir[5][3] = {"1","eE","eS","eO","eN"};//array di errore se banderuola da valori analogici errati o se scollegati fili input
int gradi[] = {22, 45, 67, 90, 112, 135, 157, 180, 202, 225, 247, 270, 292, 315, 337, 0};

void readWindDir () {
  int analogEst = analogRead(pinEst); // leggo i valori input della banderuola
  int analogSud = analogRead(pinSud);
  int analogWest = analogRead(pinWest);
  int analogNord = analogRead(pinNord);

  //CICLI IF per capire la posizione della banderuola a partire dai valori analogRead
  //Si DEVE eseguire una TARATURA con strumenti diversi dalla banderuola DeAgostini

  if (analogEst > 250 && analogEst < 999 && analogNord > 990) { //NNE
    wind.dir = 0;  //indice byte che indica la posizione all'interno dell'array di stringe Direzioni[] per ottenere la direzione del vento
    mediaDir[0]++; //aumento di uno l'elemento dell'array mediaDir[] che contiene il numero di volte che la banderuola si trova in una posizione
    // poco sotto si conta quale direzione é stata registrata il maggior numero di volte
  }
  else if (analogEst > 250 && analogEst <= 999 && analogNord <= 990 && analogNord > 250) { //NE
    wind.dir = 1;
    mediaDir[1]++;
  }
  else if (analogEst > 999 && analogNord <= 990 && analogNord > 250) { //ENE
    wind.dir = 2;
    mediaDir[2]++;
  }
  else if (analogEst > 999 && analogNord < 250 && analogWest < 250 && analogSud < 250) { //E
    wind.dir = 3;
    mediaDir[3]++;
  }

  else if (analogSud > 250 && analogSud <= 990 && analogEst > 999) { //ESE
    wind.dir = 4;
    mediaDir[4]++;
  }
  else if (analogSud > 250 && analogSud <= 990 && analogEst <= 999 && analogEst > 250) { //SE
    wind.dir = 5;
    mediaDir[5]++;
  }
  else if (analogSud > 990 && analogEst <= 999 && analogEst > 250) { //SSE
    wind.dir = 6;
    mediaDir[6]++;
  }
  else if (analogSud > 990 && analogEst < 250 && analogWest < 250 && analogNord < 250) { //S
    wind.dir = 7;
    mediaDir[7]++;
  }

  else if (analogWest > 250 && analogWest <= 990 && analogSud > 990) { //SSO
    wind.dir = 8;
    mediaDir[8]++;
  }
  else if (analogWest > 250 && analogWest <= 990 && analogSud > 250 && analogSud <= 990) { //SO
    wind.dir = 9;
    mediaDir[9]++;
  }
  else if (analogWest > 990 && analogSud > 250 && analogSud <= 990) { //OSO
    wind.dir = 10;
    mediaDir[10]++;
  }
  else if (analogWest > 990 && analogEst < 250 && analogNord < 250 && analogSud < 250) { //O
    wind.dir = 11;
    mediaDir[11]++;
  }

  else if (analogNord > 250 && analogNord <= 990 && analogWest > 990) { //ONO
    wind.dir = 12;
    mediaDir[12]++;
  }
  else if (analogNord > 250 && analogNord <= 990 && analogWest > 250 && analogWest <= 990) { //NO
    wind.dir = 13;
    mediaDir[13]++;
  }
  else if (analogNord > 990 && analogWest > 250 && analogWest <= 990) { //NNO
    wind.dir = 14;
    mediaDir[14]++;
  }
  else if (analogNord > 990 && analogEst < 250 && analogWest < 250 && analogSud < 250) { //N
    wind.dir = 15;
    mediaDir[15]++;
  }

  //eventuali errori nel caso i fotodiodi delle 4 direzioni non funzionino e non diano il valore aspettato
  if (analogEst <= 10) wind.err = 1; //errore Est
  else if (analogSud <= 10) wind.err = 2; //errore Sus
  else if (analogWest <= 10) wind.err = 3; //errore ovest
  else if (analogNord <= 10) wind.err = 4; //errore nord
  else wind.err = 0;

  //CALCOLO MODA della direzione
  for (byte f = 0; f < 16; f++) {
    if (mediaDir[f] > maxDir) { //controllo quale valore direzione si é ripetuto di piú confrontando i valori dell'array mediaDir
      maxDir = mediaDir[f];
      wind.dirmedia = f;
    }
  }
}

//////////////////////////PIOGGIA///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
//INTENSITA' DELLA PIOGGIA//////////////////////////////////////////////////////////////////////////
volatile unsigned int nBasculate = 0; //numero interrupt, incrementa fino alla lettura readPioggia()
volatile unsigned long prevBascTime = 0;
float rainrate = 0; // variabile per l'intensità della pioggia
float mmGoccia = 0.4; // constante del valore di ogni basculata del pluviometro

//ACCUMULO PIOGGIA//////////////////////////////////////////////////////////////////////////////////

void ContaGocce() {
 if (millis() - prevBascTime > 10) { //antidebaunce 1 secondo --> 1080 mm/h
    prevBascTime = millis();
    nBasculate++;
  }
}

float mmPioggia = 0; // variabile conteggio mm pioggia giorno
float mediaRR = 999;
unsigned int countRR = 0;
float mRR = 0;
float rainrateMax = 0; //rainrate massimo nell'intervallo media
const byte eeAddress1 = 2;
unsigned int prevBasculate = 0, currentBasculate;
bool eseguiResetDay = false;

void readPioggia () {
  noInterrupts();
  currentBasculate = nBasculate;
  nBasculate = 0;
  interrupts();
  //midnight reset
  if (hour() == 0 && eseguiResetDay == true) {
    eseguiResetDay = false;
    mmPioggia = mmGoccia * currentBasculate; //mmPioggia = 0; to not miss data between measurment
  } else {
    mmPioggia += mmGoccia * currentBasculate;
  }

  if (hour() != 0 && eseguiResetDay == false) //if status 1
    eseguiResetDay = true; //status 0

  EEPROM.put(eeAddress1, mmPioggia);
  //rainrate
  if (currentBasculate != prevBasculate) {
    rainrate = mmGoccia * currentBasculate * 3600;	//delta t fra misure successive 1 sec -> 3600s/1s=3600
  } else
    rainrate = 0;
  prevBasculate = currentBasculate;

  //media 1 min
  mRR += rainrate;
  countRR++;
  mediaRR = mRR / countRR;

  //rainrate massimo nell'intervallo della media
  if (rainrate > rainrateMax)
    rainrateMax = rainrate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////ALTRE FUNZIONI///////////////////////////////////////////////////////
//TENSIONE BATTERIA
const long R1 = 180000;
const int R2 = 10000;
float Vmedio;
float Vbatteria;
int countVolt = 0;
void readVolt() {
  Vmedio += analogRead(Pin_Batteria) * (1.1 / 1023.0) * 19; // ((R1 + R2) / R2) = 19/1;
  countVolt++;
  Vbatteria = Vmedio / countVolt;
}

void checkVolt() {
  if (Vbatteria < 12.1) {
    digitalWrite(13, LOW);
  }
  if (Vbatteria >= 12.1) {
    digitalWrite(13, HIGH);
  }
}

//FUNZIONE PER PRENDERE I VALORI DELLA EEPROM PRECEDENTEMENTE MEMORIZZATI//
void Eeprom() {
  EEPROM.get(eeAddress1, mmPioggia);
}

//MEDIA NUMERO DATI RACCOLTI
float ndata = 0;
void nData() {
  unsigned long sommaNdata = 0;
  sommaNdata += countTp;
  sommaNdata += countTpdht;
  sommaNdata += countTp2;
  sommaNdata += countUr;
  sommaNdata += countWind;
  sommaNdata += countRR;
  ndata = sommaNdata / 6;
}

//RESET A DISTANZA
//void(* resetFunc) (void) = 0; //ora inutile perchè attivo watchdog

void resetArduino() {
  if (radioRx.resetRx) {
    //resetFunc();
    while (1);
  }
}

//invio status di reset avvenuto
bool resetStatus = true;
void statusReset() {
  if (millis() > 180000) //dopo 3 min clear flag
    resetStatus = false;
}

//status info da inviare al rx
byte errors = 0;
void errorsStatus() {
  errors = 0;
  if (resetStatus)
    errors += 100;
  if (ersd)
    errors += 10;
  if (wind.err != 0)
    errors += wind.err;
}

byte send_count = 0;
void fillStruct() {
  radioTx.TP = int(TP * 100); //invio dati
  radioTx.TP2 = int(TP2 * 100);
  radioTx.TPDHT = int(TPDHT * 100);
  radioTx.tpmax = int(tpmax * 100);
  radioTx.tpmin = int(tpmin * 100);
  radioTx.UR = int(UR * 100);
  radioTx.urmax = int(urmax * 100);
  radioTx.urmin = int(urmin * 100);
  radioTx.ur2 = byte(ur2);
  radioTx.windmedio = int(wind.valmedio * 100);
  radioTx.windmax = int(wind.max * 100);
  radioTx.winddir = wind.dirmedia;
  radioTx.rainrate = int(mediaRR * 10);
  radioTx.rainratemax = int(rainrateMax * 10);
  radioTx.pioggiaday = int(mmPioggia * 10);
  radioTx.volt = byte(Vbatteria * 10);
  radioTx.ndata = byte(ndata);
  radioTx.errori = errors;
  radioTx.send_count = send_count;
  radioTx.memoryFree = freeMemory();
}

//SD
void DatalogReset() {
  ersd  = SD.begin(SD_SS, SPI_HALF_SPEED);
  if (!ersd) {
    //Serial.println("SD error");
  }
  else {
    //Serial.println("SD ok");
    ofstream dataday ("dataDay.csv", ios_base::app);
    //HEADER
    //Date Time tp tpdht tp2 ur ur2 windSpeed windDir mmPioggia rainrate ndata tpmax tpmin urmax urmin windMax rainrateMax volt ersd resetStatus erWind send_count memoryFree
    dataday << int(year()) << "-" << int(month()) << "-" << int(day()) << " " << int(hour()) << ":" << int(minute()) << ":" << int(second()) << " "
            << TP << " " << TPDHT << " " << TP2 << " " << UR << " " << int(ur2) << " "
            << wind.valmedio << " " << gradi[wind.dirmedia] << " " << mmPioggia << " " << mediaRR  << " " << ndata << " "
            << tpmax << " " << tpmin << " " << urmax << " " << urmin << " " << wind.max << " " << rainrateMax << " "
            << Vbatteria << " " << int(ersd) << " " << int(resetStatus) << " " << int(wind.err) << " " << int(send_count) << " " << int(freeMemory()) << endl;
    dataday.close();
    //Serial.println(F("written"));
  }
  //reset medie
  mTp = 0;
  countTp = 0;
  mTpdht = 0;
  countTpdht = 0;
  mTp2 = 0;
  countTp2 = 0;
  mUr = 0;
  countUr = 0;
  mWind = 0;
  countWind = 0;
  mRR = 0;
  countRR = 0;

  for (int f = 0; f < 16; f++) {
    mediaDir[f] = 0;//azzero array della frequenza direzione del vento
  }
  maxDir = 0;//azzero freq della direzione che si ripete più volte nell'intervallo

  tpmin = 130;
  tpmax = -50;
  urmax = 0;
  urmin = 100;
  wind.max = 0;
  rainrateMax = 0;
  Vmedio = 0;
  countVolt = 0;
}

void sendRadio() {
  // Send message
  ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 23, &radioTx, sizeof(radioTx));
  // Check If there is some problem of succesfully send
  if (openSerial) {
    Serial.println(rs.getResponseDescription());
  }
}

bool checkReceive() {
  if (e220ttl.available() > 1) {
    ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(radioRx));
    if (openSerial) Serial.println(F("riceved"));

    if (rsc.status.code != 1) {
      if (openSerial) Serial.println(rsc.status.getResponseDescription());
      return 0;
    } else {
      radioRx = *(radioAck*) rsc.data;
      if (openSerial) {
        Serial.println(rsc.status.getResponseDescription());
        Serial.println(radioRx.date);
      }
      rsc.close();
      return 1;
    }
  } else
    return 0;
}

volatile bool alarmed1 = false; //interrupt function from ds3231
void alarm1() {
  alarmed1 = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  if (openSerial) Serial.begin (9600);
  Wire.begin();
  //digitalWrite(SDA, 0);//disable internal i2c pull up
  //digitalWrite(SCL, 0);

  //abilito il watchdog e imposto come tempo di reser 8 secondi
  wdt_enable(WDTO_8S);

  pinMode(12, INPUT);
  pinMode(14, OUTPUT); //pin led lampeggio ad ogni loop
  digitalWrite(14, LOW);
  pinMode(15, OUTPUT); //pin led stato invio dati radio
  digitalWrite(15, LOW);
  pinMode(13, OUTPUT); //pin volt check
  digitalWrite(13, LOW);

  setWindClock();
  Eeprom();
  initRTC();
  initSD();
  initTemp();
  initHum();
  initWind();
  initPioggia();
  initVolt();
  initRadio();
  //e220ttl.setMode(MODE_3_SLEEP);
}

unsigned int countSecond = 0;
byte sendTimerSec = 60;	//number of tick before send new data(1 tick 1 sec), 60 sec
byte resendTimer = 20;	//n of tick before resend data with no ack, 10 sec
bool ack = true;		//true ack received, false not received

void loop() {
  if (alarmed1) {
    alarmed1 = false;
    rtc.alarm(DS3232RTC::ALARM_1); //reset the alarm flag
    wdt_reset();
    readTemp();
    readHum ();
    readWind ();
    readWindDir();
    readPioggia();
    readVolt();
    countSecond++;

    if (countSecond >= sendTimerSec) {	//time to send new data
      countSecond = 0;	              //reset timer counter
      radioTx.date = long(now());     //invio data ultimo dato attuale allo slave
      writeFirst = true;
      statusReset();  //recently tx reset status to send
      errorsStatus(); //assembly essor sensor to send
      nData();        //calc mean of measurment number
      fillStruct();   //riempo struct da mandare via radio, rimarrà uguale per 1'
      DatalogReset(); //sd datalog & reset medie 1'
      checkVolt();    //check if batteri low, link to solar panel directly
      //SEND
      sendRadio();
      ack = false;
      send_count = 0;  //reset number of try to sent
      digitalWrite(15, LOW);
    }
    if (!ack) {				//not ack received
      if (checkReceive() != 0 && radioRx.date == radioTx.date) { //received correct ack
        ack = true;
        resetArduino(); //check if arrived command to reset arduino
        if (millis() < 180000) { //avvisa temporaneamente ack success
          digitalWrite(15, HIGH);
        } else digitalWrite(15, LOW);
      }
    }
    if (!ack && countSecond % resendTimer == 0 && countSecond != 0) { //resend old data every resendTimer but not when new data
      sendRadio();    //send old data
      send_count++;    //try to sent +1
    }

    if (openSerial) {
      Serial.print(hour());
      Serial.print(F(":"));
      Serial.print(minute());
      Serial.print(F(":"));
      Serial.print(second());
      Serial.print(F(" "));
      Serial.print(day());
      Serial.print(F("/"));
      Serial.print(month());
      Serial.print(F("/"));
      Serial.println(year());//------
      Serial.print(TP);
      Serial.print(F(" "));
      Serial.print(TPDHT);
      Serial.print(F(" "));
      Serial.print(TP2);
      Serial.print(F(", max "));
      Serial.print(tpmax);
      Serial.print(F(" min "));
      Serial.println(tpmin);
      Serial.print(UR);
      Serial.println(F(" %"));

      Serial.print(wind.val * 3.6);
      Serial.print(F(" km/h max: "));
      Serial.print(wind.max * 3.6);
      Serial.print(F(" "));
      Serial.println(gradi[wind.dirmedia]);
      Serial.print(mmPioggia);
      Serial.print(F(" mm RR: "));
      Serial.println(rainrate);
      Serial.print(F("V: "));
      Serial.print(Vbatteria);
      Serial.print(F(" nD: "));
      Serial.print(ndata);
      Serial.print(F(" err: "));
      Serial.println(radioTx.errori);
      Serial.println(countTp);
      Serial.println(radioRx.resetRx);
      Serial.println(freeMemory());
      Serial.println(F("_______"));
    }
    if (millis() < 180000) digitalWrite(14, !digitalRead(14)); //led cambia stato ad ogni loop
    else digitalWrite(14, LOW);
  }
}
