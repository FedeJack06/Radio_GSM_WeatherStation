/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////                                                                     /////////////
	//////////////////////////          Weather station by Fedejack - SLAVE 1.7                    /////////////
	//////////////////////////                                                                     /////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
//////////////////////////////////////LIBRERIE E DEFINIZIONI////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <TimeLib.h>
#include "printf.h"
#include <MemoryFree.h>

bool serialOpen = false;
////////////////////////////////////////PRIMA DI INIZIARE
bool INIZIALIZZAZIONE = 0;   // mettere 1 al posto di 0 se é la prima volta che si esegue il programma oppure se si é resettato tutto, avvia i processi che devono essere eseguiti una volta
//float CALIBRATION = 3070; //3/6/21 da meteomin.it 3265 taratura per calcolo pressione sul livello del mare, differenza (in Pa) fra pressione slm attuale e pressione letta dal bmp180
float altitudine_bmp = 302; //metri
//non più utilizzato, ora uso bmp.readSealevelPressure(altitudine_barometro));

//DEWPOINT HEATINDEX
#include "DHT.h"
#define DHTTYPE DHT22
DHT dht(1, DHTTYPE);

//PRESSURE BMP180
#include <Adafruit_BMP085.h>
Adafruit_BMP085 barometer;

//SD
#include <SdFat.h>
#define SD_SS 4 // pin usato dalla microSD
SdFat SD;

//DISPLAY TFT 1.8" 
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#define sclk ICSP-3 // 13 se si usa Arduino UNO
#define mosi ICSP-4 // 11 se si usa Arduino UNO
#define rst  5
#define cs   6
#define dc   7
Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);

//Radio 1
#define DESTINATION_ADDL 2
#include "Arduino.h"
#include "LoRa_E220.h"
LoRa_E220 e220ttl(&Serial2, 27, 31, 29); //TX RX, AUX M0 M1

//Radio 2
#include <SX126x.h>
SX126x LoRa;
int8_t nssPin = 39, resetPin = 41, busyPin = 37, irqPin = 19, txenPin = 35, rxenPin = 33;

//use serial3 pin 14 rx of sim800, pin 15 tx of sim800l, D24 reset

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////VARIABILI///////////////////////////////////////////////////////

////////////////////////TEMPORIZZAZIONI/////////////////////////////////////////
const long interval = 600000;//TEMPORIZZAZIONE MEDIE
unsigned long prevResetMedie = 0; //temporizzazione medie
bool onetime = true; //variabile per inizializzare giorni/mesi/anni salvati

/////////////CONTROLLI ED ERRORI/////////////////////////////////////////////////
//ultima data ricezione dati radio
int Year,Month,Day,Hour,Minute,Second;
tmElements_t d;

bool erbmp=false; //errore init bmp180 SLAVE
bool erradioRX=false; //errore init radio SLAVE
bool ersd=false; //errore sd SLAVE
bool writeExtreme = false;
bool readEstremiOk = false;

///////STRUCT PER L'ORARIO DI RILEVAZIONE DEGLI ESTREMI GIORNALIERI E PER I VALORI DI ESSI(ANCHE MENSILI E ANNUI)/////////////
struct TimedValue {
	float max;
	float min;
};
//VALORI GIORNALIERI/////////
TimedValue DayTemp;
TimedValue DayHum;
TimedValue DayDewPoint;
TimedValue DayWind;
TimedValue DayChill; 
TimedValue DayHeatIndex; 
TimedValue DayRainRate; 

/////STRUCT RADIO 1////////////////////////////////////////////////////////////////////////////
struct radioVal { //traferimento dati via radio
	unsigned long date;//trasferimento data allo slave per salvataggio dati 
	int TP; //temperatura sht35 *100
	int TPDHT; //temperatura dht22 *100
	int TP2; //temp ds18b20 *100
	int tpmax; //*100
	int tpmin; //*100
	int urmax; //*100
  	int urmin; //*100
	unsigned int UR; //umiditá 16 *100
	byte ur2; //umidita dht22
	unsigned int windmedio; //velocitá vento m/s *100
	unsigned int windmax; //raffica giornaliera massima *100
	byte winddir; //indice direzione vento, da usare per estrarre stringhe da Direzioni[] 
	unsigned int pioggiaday; //pioggia del giorno *10
	unsigned int rainrate; //intensitá di pioggia *10
	unsigned int rainratemax; //max rr *10
	
	byte ndata;
	byte errori; //001 = 0 reset status, 0 ersd, 1 errore winddir
	byte volt; //tensione batteria alimetazione
	byte send_count;
	unsigned int memoryFree;
	// 32/32 byte occuped in nRF24
};
radioVal radioRx;   // 31 byte 

struct radioAck {
  unsigned long date=0;
  bool resetTx=false;
};
radioAck radioTx;

/////STRUCT RADIO 2////
struct radioValMini { //traferimento dati via radio
  unsigned long date;//trasferimento data allo slave per salvataggio dati 
  int TP; //temperatura sht45 *100 
  int TPBMP; //temperatura bmp180 *100
  int tpmax; //*100
  int tpmin; //*100
  unsigned int UR; //umiditá sht45 *100
  unsigned int urmax; //*100 
  unsigned int urmin; //*100
  unsigned long pressure; //pressione bmp180 *100 [Pa]
  unsigned int mmpioggia; //*100
  unsigned int rainrate; //*100 [mm/h]
  byte ndata; //*10
  unsigned int statusDev; //11001 = 1 lora, 1 reset, 0 sht, 0 bmp, 1 ersd
  int volt; //tensione batteria alimetazione
  byte send_count;
};
radioValMini radioRxMini;


struct radioAckMini {
  unsigned long date=0;
};
radioAckMini radioTxMini;

//////////////INIZIALIZZAZIONI/////////////////////////////////////////////////////
//PIN
void setPin(){
	pinMode(41, OUTPUT);
	pinMode(39, OUTPUT);
	digitalWrite(41, HIGH);
	digitalWrite(39, HIGH);
}
//SD
void initSD() {
	//evita conflitto SPI tra nrf24 e ethernet shield  
	pinMode(4, OUTPUT);
	pinMode(10, OUTPUT);
	digitalWrite(4, HIGH);
	digitalWrite(10, HIGH);
	
	ersd  = SD.begin(SD_SS, SPI_HALF_SPEED); 

	if (!ersd){ 
		Serial.println(F("SD error"));
		tft.setCursor(60,107);
		tft.setTextSize(1);
		tft.setTextColor (ST7735_RED);
		tft.println(F("NO SD CARD"));
		tft.println(F("insert SD to initialize"));
		//while(2);
	}
	else{
		Serial.println(F("SD ok"));  
		/*File logFile = SD.open ("dataDay.csv", FILE_WRITE);
		if (logFile){///CREARE IL FILE csv SCOMMENTANDO LE RIGHE SOTTO PER AVERE I TITOLI DI OGNI COLONNA DOVE SI SCRIVERANNO I VALORI DEI SENSORI. CARICARE IL CODICE PRIMA SCOMMENTATO E POI COMMENTARE LE RIGHE COME SONO ADESSO E RICARICARE
			String header = "Date Time Temperature Temperaturedht Humidity DewPoint HeatIndex Pressure(slm) AverageSpeed(5') MaxWindSpeed(5') WindDir(5') Windchill Rain AverageRRate(5') MaxRR(5')";
			logFile.println(header);
			logFile.close();
		}

		File logFile2 = SD.open ("extDay.csv", FILE_WRITE);
		if (logFile2){///CREARE IL FILE csv SCOMMENTANDO LE RIGHE SOTTO PER AVERE I TITOLI DI OGNI COLONNA DOVE SI SCRIVERANNO I VALORI DEI SENSORI. CARICARE IL CODICE PRIMA SCOMMENTATO E POI COMMENTARE LE RIGHE COME SONO ADESSO E RICARICARE
			String header = "Day MaxTemp timeMaxT MinTemp timeMinT MaxHum timeMaxH MinHum timeMinH MaxDP timeMaxDP MinDP timeMinDP MaxPress timeMaxP MinPress timeMinP MaxWSpeed timeMaxW MinWC timeMinWC MaxRR timeMaxRR mediaTp mmPioggia";
			logFile2.println(header);
			logFile2.close();
		}*/
	}
}

//PRESSIONE////
void initPressure () {
	if (!barometer.begin()) 
		Serial.println("BMP180 error");
	else Serial.println("BMP180 ok");
	erbmp = barometer.begin();
}

//RADIO 1
void initLLCC68() {
	Serial2.begin(9600); //for llcc68
  	e220ttl.begin();
	erradioRX = true;
	ResponseStructContainer c;
	c = e220ttl.getConfiguration();
	Serial.println(c.status.code);
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
  	if (serialOpen){
		Serial.println(c.status.getResponseDescription());
		printParameters(configuration);
  	}

	//  ----------------------- FIXED RECEIVER -----------------------
	configuration.ADDL = 0x03;
	configuration.ADDH = 0x00;
	configuration.CHAN = 23;

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
	if (serialOpen){
		Serial.println(rs.getResponseDescription());
		Serial.println(rs.code);
	}
	c = e220ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	configuration = *(Configuration*) c.data;
	if (serialOpen){
		Serial.println(c.status.getResponseDescription());
		Serial.println(c.status.code);
		printParameters(configuration);
	}
	c.close();
}
void printParameters(struct Configuration configuration) {
	Serial.println(F("----------------------------------------"));
	Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
	Serial.println(F(" "));
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
	Serial.println(F(" "));
	Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
	Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
	Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
	Serial.println(F(" "));
	Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
	Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
	Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
	Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());
	Serial.println(F("----------------------------------------"));
}

void initTFT(){
	tft.initR(INITR_BLACKTAB);// inizializzazione display
	tft.setRotation(1);
	tft.fillScreen(ST7735_BLUE);
	tft.setCursor (10, 20);
	tft.setTextSize(2);
	tft.setTextColor (ST7735_GREEN);
	tft.print (F(" STAZIONE"));
	tft.setCursor (15, 40);
	tft.print (F("  METEO:"));
	delay (100);
	tft.setTextColor (ST7735_YELLOW);
	tft.setCursor (5, 60);
	tft.print (F(" SITUAZIONE    ATTUALE"));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////FUNZIONI PER LOOP: ACQUISIZIONE////////////////////////////////////
//////////////////////////////////////TEMPERATURA///////////////////////////////////////
float tp; //sht35
float tpdht; //dht22
float tp2; //ds18b20
float tpbmp; //bmp180
//float tpint; //temperatura interna
float tpmax=-100, tpmin=100;
//andamento
int tp10min=0, tp5min=0, tpnow=0;
unsigned long intAndamentoTp=0;
char andamentoTp;// = < >
float tpChange;//ANDAMENTO

void readTemp () {
	//DATI ISTANTANEI DAL MASTER
	tp = float(radioRx.TP)/100;
	tpdht = float(radioRx.TPDHT)/100;
	tp2 = float(radioRx.TP2)/100;

	tpbmp = barometer.readTemperature();
	//sensors.requestTemperatures(); //richiesta temp dal sensore dallas interno
	//tpint = sensors.getTempCByIndex(0); //temp dallas

	tpmax = float(radioRx.tpmax)/100;
	tpmin = float(radioRx.tpmin)/100;
	
	//ANDAMENTO 5' e 10'
	if (millis() - intAndamentoTp > 300000) {
		tp10min = tp5min;
		tp5min = tpnow;
		tpnow = int(tp*10);
		intAndamentoTp = millis();
	}

	if (abs(tp*10 - tp10min) > 50)
		andamentoTp = ' ';
	else if (tp*10 - tp10min > 3)
		andamentoTp = '+';
	else if (tp*10 - tp10min < -3)
		andamentoTp = '-';
	else
		andamentoTp = '=';

	if (millis() > 900000)
		tpChange = tp - (float(tp10min)/10);//andamento orario
	
	//ESTREMI GIORNALIERI
	if (tpmin < DayTemp.min)
		DayTemp.min = tpmin;
	if (tpmax > DayTemp.max)
		DayTemp.max = tpmax;
}

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////UMIDITA'//////////////////////////////////////////////
float ur, urmax, urmin;
byte ur2;
//andamento
byte ur10min=0, ur5min=0, urnow=0;
unsigned long intAndamentoUr=0;
char andamentoUr;// = < >
int urChange;

void readHum () {
	ur = float(radioRx.UR)/100;
	urmax = float(radioRx.urmax)/100;
	urmin = float(radioRx.urmin)/100;
	ur2 = radioRx.ur2;

	//ANDAMENTO
	if (millis() - intAndamentoUr > 300000) {
		ur10min = ur5min;
		ur5min = urnow;
		urnow = ur;
		intAndamentoUr = millis();
	}

	if (abs(ur - ur10min) > 20)
		andamentoUr = ' ';
	else if (ur - ur10min > 2)
		andamentoUr = '+';
	else if (ur - ur10min < -2)
		andamentoUr = '-';
	else
		andamentoUr = '=';

	if (millis() > 900000)
		urChange = ur - ur10min;
		
	//ESTREMI GIORNALIERI
	if (urmax > DayHum.max)
		DayHum.max = urmax;
	if (urmin < DayHum.min)
		DayHum.min = urmin;
}

///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////PRESSIONE//////////////////////////////////////////
float pressionelivellodelmare;// variabile per la pressione slm in Pa
float pressione;// pressione bmp180
//andamento
unsigned long intPressure = 0; //intervallo per salvataggio orario della pressione
float previousThree, previousSix; //differenza pressione precedente 3 6 ore
float pNow, pOne, pTwo, pThree, pFour, pFive, pSix; //pressioni precenti di 0 fino a 6 ore fa
//medie in intervallo
unsigned int countPr=0;
float mPr=0;
float PR, SLPR;//medie

void readPressure () {
	pressione = barometer.readPressure();
	pressionelivellodelmare = pressione / pow(1.0 - altitudine_bmp / 44330, 5.255); //normalizzazione livello mare (QFF) from bosch datasheet

	//MEDIA in intervallo attendo link master
	mPr += pressione;
	countPr++;
	PR = mPr/countPr;
	SLPR = PR / pow(1.0 - altitudine_bmp / 44330, 5.255);

	//ANDAMENTO
	previousThree = pressione - pThree;
	previousSix = pressione - pSix;
	//SALVATAGGIO pressione precente da 1 a 6 ore
	if ((millis() - intPressure) > 3600000){ 
		intPressure = millis();
		pSix = pFive; //pressione -6 ore
		pFive = pFour; //pressione -5 ore
		pFour = pThree; //pressione -4 ore
		pThree = pTwo; //pressione -3 ore
		pTwo = pOne; //pressione -2 ore
		pOne = pNow; //pressione -1 ore
		pNow = pressione; //pressione attuale
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////VELOCITA' DEL VENTO////////////////////////////////////
struct WindValue { 
	float val; //da master, mediato su 1'
	float valmedio; //velocità media
	float max; //velocitá massima giornaliera
	char dir[3];
	char dirmedia[3]; //direzione vento media
	char err[3]; //errore direzione vento
};
WindValue wind;

float sommaWind = 0;
unsigned int countWind = 0;

void readWind () {    
	wind.val = float(radioRx.windmedio)/100;
	wind.max = float(radioRx.windmax)/100;

	//MEDIA
	sommaWind += wind.val;
	countWind++;
	wind.valmedio = sommaWind/countWind;

	//ESTREMI GG
	if (wind.max > DayWind.max)
		DayWind.max = wind.max;
}

//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////DIREZIONE DEL VENTO//////////////////////////////////
char direzioni[16][4] = {"NNE","NE","ENE","E","ESE","SE","SSE","S","SSO","SO","OSO","O","ONO","NO","NNO","N"};
int gradi[] = {22,45,67,90,112,135,157,180,202,225,247,270,292,315,337,0};
char erdir[5][3] = {"1","eE","eS","eO","eN"};
//media
int mediaDir[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//array della moda direzione del vento
int maxDir = 0;//indice di direzione che si ripete più volte nell'intervallo

void readWindDir () {
	strcpy(wind.dir, direzioni[radioRx.winddir]);//wind.dir = direzioni[radioRx.winddir];

	//ERRORE
	byte indexErr = radioRx.errori-(radioRx.errori/10)*10; //es. da 104 ottengo 4 (l'unita)
	strcpy(wind.err, erdir[indexErr]);//wind.err = erdir[radioRx.erWinddir];

	//MEDIA
	mediaDir[radioRx.winddir]++; //add 1 at moda array
	//CALCOLO MODA della direzione
	for (int f=0; f<16; f++){
		if (mediaDir[f] > maxDir){//controllo quale valore direzione si é ripetuto di piú confrontando i valori dell'array mediaDir
			maxDir = mediaDir[f];
			strcpy(wind.dirmedia, direzioni[f]);//variabile direzione piú frequente in 10'
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////PIOGGIA///////////////////////////////////////////
float mmPioggia = 0.0; //conteggio mm pioggia in corso
float mmPioggiagiorno; // mm pioggia cumulata nel giorno
float mmPioggiamese; // conteggio mm pioggia mese
float mmPioggiaanno; // conteggio mm pioggia anno
float prevPioggia; // variabile per incremetare i conteggi precedenti
float rainrate = 0; // variabile per l'intensità della pioggia
float rainratemax;

void readPioggia(){
	prevPioggia = mmPioggia;
	mmPioggia = float(radioRx.pioggiaday)/10;
	rainrate = float(radioRx.rainrate)/10;
	rainratemax = float(radioRx.rainratemax)/10;
	
	if (mmPioggia == 0)
		prevPioggia = 0;
	mmPioggiagiorno += (mmPioggia-prevPioggia);
	mmPioggiamese += (mmPioggia-prevPioggia);
	mmPioggiaanno += (mmPioggia-prevPioggia);

	//ESTREMI
	if (rainratemax > DayRainRate.max)
		DayRainRate.max = rainratemax;
}

/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////DEW POINT////////////////////////////////////////
float dewPoint;
//andamento
int dp10min=0, dp5min=0, dpnow=0;
unsigned long intAndamentoDp=0;
char andamentoDp;
int dpChange;

void readDewPoint () {
	float hu = float(ur);
	dewPoint = (pow (hu / 100, 0.125) * (112 + (0.9 * tp)) + 0.1 * tp - 112);//equazione per il calcolo del dewpoint
	
//ANDAMENTO
	if (millis() - intAndamentoDp > 300000) {
		dp10min = dp5min;
		dp5min = dpnow;
		dpnow = int(dewPoint*10);
		intAndamentoDp = millis();
	}

	if (abs(dewPoint*10 - dp10min) > 50)
		andamentoDp = ' ';
	else if (dewPoint*10 - dp10min > 3)
		andamentoDp = '+';
	else if (dewPoint*10 - dp10min < -3)
		andamentoDp = '-';
	else
		andamentoDp = '=';
//andamento orario
	if (millis() > 900000)
		dpChange = dewPoint - (float(dp10min)/10);

//ESTREMI
	if (dewPoint > DayDewPoint.max)
		DayDewPoint.max = dewPoint;
	if (dewPoint < DayDewPoint.min)
		DayDewPoint.min = dewPoint;
}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////WIND CHILL////////////////////////////////////////
float windchill;

void readWindchill () {
	if (wind.val >= 1.3)
		windchill = (13.12 + 0.6215 * tp) - (11.37 * pow(wind.val*3.6, 0.16)) + (0.3965 * tp * pow(wind.val*3.6, 0.16));
	else
		windchill = tp;

//ESTREMI
	if (windchill < DayChill.min)
		DayChill.min = windchill;
}

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////INDICE DI CALORE/////////////////////////////////////
float heatindexc;

void readHeatIndex() {
	int hu = ur;
	float hic = dht.computeHeatIndex(tp, hu, false);//questo calcolo dell'indice di calore viene fornito dalla libreria 'dht'
	heatindexc = hic;

//ESTREMI
	if (heatindexc > DayHeatIndex.max)
		DayHeatIndex.max = heatindexc;
}

float wetbulb;
void readWetbulb(){
	wetbulb = tp * (0.45 + 0.006 * ur * pow(SLPR/106000, 0.5));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////ALTRE FUNZIONI///////////////////////////////////////////////////////
/////////////////////////////TASTO RESET TX
int valButtonOld = LOW;
unsigned long buttonStart = 0, buttonStop = 0;
bool flagWSmini = false;
void button() {
	int valButton = digitalRead(A4);
	if (digitalRead(A4) == HIGH && valButtonOld == LOW){ //when button press the first time (old state low)
		buttonStart = millis();
	}
	else if (digitalRead(A4) == LOW && valButtonOld == HIGH){ //button release
		buttonStop = millis();
		if (buttonStop - buttonStart < 2000){
			flagWSmini = !flagWSmini; //chenge default display
			if (flagWSmini) 
				readDisplayMini(); //change display
			else
				readDisplay(); //change display
		}
		if (buttonStop - buttonStart > 3000){
			if (!flagWSmini)  //in WS mode, change reset command of RX
				radioTx.resetTx = !radioTx.resetTx;
			readDisplay();
		}
	}
	valButtonOld = valButton;
}

/////////////////////////////DECODE ERRORI MASTER
bool ersdTx=false;
bool resetStatusTx = false;
void erroriTx(){
	byte errors = radioRx.errori; // 115 reset sdtx wind
	resetStatusTx = errors/100;
	ersdTx = errors/10 - (errors/100)*10;
	//     elimino unita -  centinaia  *10
	//115->     11               1 * 10
	// error wind decode is in readWind function

	if(resetStatusTx == true){ //se reset remoto a buon fine, non invio piu comando di reset remoto
		radioTx.resetTx = false;
	}
}

//////////////////////////DECODE DATASTAMP FROM RADIO
void getTime() {
	breakTime (radioRx.date, d);//usare d.Hour d.Minute d.Second d.Day d.Month d.Year per registrare valori orari
	//lastRadioRx = String(d.Hour) + ":" + String(d.Minute) + ":" + String(d.Second) + "-" + String(d.Day) + "/" + String(d.Month);
	Year = d.Year+1970;
	Month = d.Month;
	Day = d.Day;
	Hour = d.Hour;
	Minute = d.Minute;
	Second = d.Second;
}

///////////////////////////////LETTURA FILE ESTREMI
void readEstremi () {
	ersd = SD.begin(SD_SS, SPI_HALF_SPEED);
	if (!ersd){
		//Serial.println(F("SD error"));
	}
	else{
		ifstream estremiIn("estremi.dat");
		//if (!estremiIn.is_open()) Serial.println(F("errore apertura estremi.csv"));

		//if (estremiIn.fail()) 
		//  Serial.println (F("bad input estremi"));

			estremiIn   >> DayTemp.max >> DayTemp.min
						>> DayHum.max >> DayHum.min
						>> DayDewPoint.max >> DayDewPoint.min
						>> DayWind.max 
						>> DayChill.min 
						>> DayHeatIndex.max 
						>> DayRainRate.max
						
						>> pOne >> pTwo >> pThree >> pFour >> pFive >> pSix
						>> mmPioggia >> mmPioggiagiorno >> mmPioggiamese >> mmPioggiaanno;

			estremiIn.skipWhite();
			readEstremiOk = true;
		//if (estremiIn.fail())
		//  Serial.println(F("bad END input estremi"));
	}
}

//SD 
void Datalog() {
	ersd  = SD.begin(SD_SS, SPI_HALF_SPEED);
	if (!ersd) {
			//Serial.println("SD error");    
	}
	else {
			//Serial.println("SD ok");
			ofstream dataday ("dataDay.csv", ios_base::app);

			/*dataday << int(Year) << "-" << int(Month) << "-" << int(Day) << " " << int(Hour) << ":" << int(Minute) << ":" << int(Second) << " " << tp << " " << tpdht << " " << tp2 << " " << ur << " "
			<< wind.val << " " << gradi[radioRx.winddir] << " " << mmPioggia << " " << rainrate  << " " << tpmax << " " << tpmin << " " << urmax << " " << urmin << " " << wind.max << " " 
			<< rainratemax << " " << int(radioRx.ndata) << " " << PR << " " << SLPR << " " << float(radioRx.volt)/10 << endl;
*/
			//HEADER
    		//Date Time tp tpdht tp2 ur ur2 pressure slpressure windSpeed windDir mmPioggia rainrate ndata tpmax tpmin urmax urmin windMax rainrateMax volt erbmp ersd ersdTx resetStatusTx send_count
    
			//dataday
			dataday << int(Year) << "-" << int(Month) << "-" << int(Day) << " " << int(Hour) << ":" << int(Minute) << ":" << int(Second) << " "
			<< tp << " " << tpdht << " " << tp2 << " " << ur << " " << int(ur2) << " " << PR << " " << SLPR << " "
			<< wind.val << " " << gradi[radioRx.winddir] << " " << mmPioggia << " " << rainrate << " " << int(radioRx.ndata) << " "
			<< tpmax << " " << tpmin << " " << urmax << " " << urmin << " " << wind.max << " " << rainratemax << " "
			//control
			<< float(radioRx.volt)/10 << " " << int(erbmp) << " " << int(ersd) << " " << int(ersdTx) << " " << int(resetStatusTx) << " " << int(radioRx.send_count) << endl;//<< " " << radioRx.memoryFree << endl;

			dataday.close();
	}
}

/////////////////////////////RESET MEDIE TEMPORANEE PER DISPLAY
void resetMedie() {
	if (millis() - prevResetMedie >= interval) {
		prevResetMedie = millis();

		sommaWind = 0;
		countWind = 0;
		for (int f=0; f<16; f++){
			mediaDir[f] = 0;//azzero array della frequenza direzione del vento
		}
		maxDir = 0;//azzero indice di direzione che si ripete più volte nell'intervallo
	}
}

//////////////////////////reset media quando radio link true
void resetPressure(){
	mPr = 0;
	countPr = 0;
}

///////////////////////////////////////////SALVARE ESTREMI
void writeEstremi () {
	if (SD.begin(SD_SS, SPI_HALF_SPEED)) {
		ofstream estremiOut ("estremi.dat");
		estremiOut << DayTemp.max << " " << DayTemp.min << endl
		<< DayHum.max << " " << DayHum.min << endl
		<< DayDewPoint.max << " " << DayDewPoint.min << endl
		<< DayWind.max << endl
		<< DayChill.min << endl
		<< DayHeatIndex.max << endl
		<< DayRainRate.max << endl
		
		<< pOne << " " << pTwo << " " << pThree << " " << pFour << " " << pFive << " " << pSix << endl
		<< mmPioggia << " " << mmPioggiagiorno << " " << mmPioggiamese << " " << mmPioggiaanno << endl;
		//if (!estremiOut) Serial.println (F("error write estremi"));
	
		estremiOut.close();
	}
}

////////////////////////RESET ESTREMI AL TERMINE DI GIORNO, MESE O ANNO//
bool eseguiResetDay = false;
bool eseguiResetMonth = false;
bool eseguiResetYear = false;
void resetEstremi() {
	if (d.Hour == 0 && eseguiResetDay == true){ //if status 0
		eseguiResetDay = false; //status 1

		mmPioggiagiorno = 0;
		prevPioggia = 0;
		mmPioggia = 0; //fix bug if first mmpioggi != 0
		DayWind.max = wind.val;
		DayTemp.max = tp;
		DayTemp.min = tp;
		DayHum.max = ur;
		DayHum.min = ur;
		DayDewPoint.max = dewPoint;
		DayDewPoint.min = dewPoint;
		DayHeatIndex.max = heatindexc;
		DayChill.min = windchill;
		DayRainRate.max = rainrate;
		intPressure = 0;
		prevResetMedie = 0;
	}
	if (d.Hour != 0 && eseguiResetDay == false) //if status 1
		eseguiResetDay = true; //status 0
	
	if (d.Day == 1 && eseguiResetMonth == true){
		eseguiResetMonth = false;
		mmPioggiamese = 0;
	}
	if (d.Day != 1 && eseguiResetMonth == false)
		eseguiResetMonth = true;

	if (d.Month == 1 && eseguiResetYear == true){
		eseguiResetYear = false;
		mmPioggiaanno = 0;
	}
	if (d.Month != 1 && eseguiResetYear == false)
		eseguiResetYear = true;
}

///////////////////////////////////////DISPLAY/////////////
int sD,sE,sC,sW;
void readDisplay() {
	tft.fillScreen(ST7735_BLACK);
	tft.setTextColor (ST7735_WHITE);
	tft.setCursor(0, 0);
	tft.setTextSize(1);
		tft.print(F("T "));
		tft.print(tp, 1);
		tft.print(F("  "));
		tft.print(tpdht, 1);
		tft.print ((char)248);
		tft.print(F("C"));
		tft.setCursor(82,0);
		tft.print(F(" "));
		tft.setTextColor (ST7735_BLUE);
		tft.print(DayTemp.min, 1);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F(" "));
		tft.setTextColor (ST7735_RED);
		tft.print(DayTemp.max, 1);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F(" "));
		tft.setTextColor (ST7735_YELLOW);
		tft.println(andamentoTp);
		tft.drawLine(148,9,148,41,ST7735_CYAN);
	tft.drawLine(0,8,159,8,0x5d5348);
	tft.setCursor(0,11);    
	tft.setTextColor (ST7735_WHITE);    
	tft.print(F("U"));
	tft.setCursor(17,11);
	tft.print(ur);
	tft.print(F("%"));
	tft.setCursor(72,11);
	tft.print(F(" "));
	tft.setTextColor (ST7735_BLUE);
	tft.print(DayHum.min, 0);
	tft.print(F("%"));
	tft.setTextColor (ST7735_WHITE);
	tft.print(F(" "));
	tft.setTextColor (ST7735_RED);
	tft.print(DayHum.max, 0);
	tft.print(F("%"));
	tft.setTextColor (ST7735_WHITE);
	tft.print(F(" "));
	tft.setTextColor (ST7735_YELLOW);
	tft.print(andamentoUr);
	tft.setCursor(149,11);
	tft.setTextColor (ST7735_RED);
	tft.print(radioTx.resetTx);
	tft.setCursor(154,11);
	tft.setTextColor (ST7735_GREEN);
	tft.println(erbmp);       /////////errore bmp180
		tft.drawLine(0,19,148,19,0x5d5348);
		tft.setCursor(0,22); 
		tft.setTextColor (ST7735_WHITE);
		tft.print(F("DP"));
		tft.setCursor(17,22);
		tft.print(dewPoint, 1); 
		tft.print((char)248);
		tft.print(F("C"));
		tft.setCursor(72,22);
		tft.print(F(" "));
		tft.setTextColor (ST7735_BLUE);
		tft.print(DayDewPoint.min, 1);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F(" "));
		tft.setTextColor (ST7735_RED);
		tft.print(DayDewPoint.max, 1);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F(" "));
		tft.setTextColor (ST7735_YELLOW);
		tft.print(andamentoDp);
		tft.setCursor(149,22);
		tft.setTextColor (ST7735_WHITE);
		tft.print(radioRx.send_count);   ///////errore mancato collegamento radio con master
		tft.setCursor(154,22);
		tft.setTextColor (ST7735_GREEN);
		tft.println(erradioRX);  ////////errore radio init slave
	tft.drawLine(0,30,148,30,0x5d5348);
	tft.setCursor(0,33);     
	tft.setTextColor (ST7735_WHITE);    
	tft.print(F("HI"));
	tft.setCursor(17,33);
	tft.print(heatindexc, 1); 
	tft.print((char)248);
	tft.print(F("C"));
	tft.setCursor(72,33);
	tft.print(F(" "));
	tft.setTextColor (ST7735_RED);
	tft.print(DayHeatIndex.max, 1);
	tft.print((char)248);
	tft.print(F("C"));
	tft.setTextColor (ST7735_WHITE);
	tft.print(F(" "));
	tft.setTextColor (ST7735_YELLOW);
	tft.println(F("="));
	tft.setCursor(149,33);
	tft.setTextColor (ST7735_WHITE);
	tft.print(ersdTx);
	tft.setCursor(154,33);
	tft.setTextColor (ST7735_GREEN);
	tft.println(ersd);
			tft.drawLine(0,41,159,41,ST7735_RED);
			tft.setCursor(0,44);
			tft.setTextColor (ST7735_WHITE);
			tft.print(F("P "));
			tft.setCursor(13,44);
			tft.print(pressionelivellodelmare/100, 1); 
			tft.print(F(" hPa"));
			tft.setCursor(74,44);
			tft.print(F(" "));
			tft.setTextColor (ST7735_YELLOW);
			tft.print(previousThree/100, 1);
			tft.setTextColor (ST7735_WHITE);
			tft.print(F(" "));
			tft.setTextColor (ST7735_GREEN);
			tft.println(previousSix/100, 1);
		tft.setCursor(13,52);
		tft.setTextColor (ST7735_WHITE);
		tft.print(pressione/100, 1);
		tft.print(F("     "));
		tft.print(sD);
		tft.print(F(" "));
		tft.print(sE);
		tft.print(F(" "));
		tft.print(sC);
		tft.setCursor(127,52);
		tft.print(float(radioRx.volt)/10, 1);
		tft.println(F("V"));
	tft.drawLine(0,61,159,61,ST7735_RED);   
	tft.setCursor(0,64);    
	tft.setTextColor (ST7735_WHITE);    
	tft.print(F("W"));
	tft.setCursor(17,64);
	tft.print(wind.val*3.6, 1); 
	tft.print(F("Km/h"));
	tft.setCursor(78,64);
	tft.print(F("G "));
	tft.setTextColor (ST7735_RED);
	tft.print(DayWind.max*3.6, 1);
	tft.setTextColor (ST7735_WHITE);
	tft.print(F(" A "));
	tft.setTextColor (ST7735_GREEN);
	tft.println(wind.valmedio*3.6, 1);
		tft.drawLine(0,72,159,72,0x5d5348);
		tft.setCursor(0,75);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F("Dir "));
		tft.print(wind.dir);
		tft.setCursor(47,75);
		tft.print(F("A"));
		tft.setTextColor (ST7735_GREEN);
		tft.print(wind.dirmedia);
		tft.drawLine(76,75,76,83,ST7735_CYAN);
		tft.setCursor(78,75);
		tft.setTextColor (ST7735_WHITE);
		tft.print("WC ");
		tft.print(windchill, 1);
		tft.print(" ");
		tft.setTextColor (ST7735_BLUE);
		tft.print(DayChill.min, 1);
		tft.drawLine(152,75,152,83,ST7735_CYAN);
		tft.setCursor(153,75);
		tft.setTextColor(ST7735_WHITE);
		tft.println(wind.err);
	tft.drawLine(0,83,159,83,ST7735_RED);
	tft.setCursor(0,86);    
	tft.setTextColor (ST7735_BLUE);    
	tft.print(F("Day"));
	tft.setTextColor (ST7735_BLUE);
	tft.setCursor(45,86);
	tft.print(F("Month"));
	tft.setTextColor (ST7735_BLUE); 
	tft.setCursor(100,86);
	tft.println(F("Year"));
	tft.setCursor(130,86);
	tft.setTextColor (ST7735_WHITE);
	tft.println(float(radioRx.TP2)/100, 1);
		tft.setTextColor (ST7735_WHITE);
		tft.print(mmPioggiagiorno, 1);
		tft.print(F("mm"));
		tft.setCursor(45,94);
		tft.print(mmPioggiamese, 1);
		tft.print(F("mm"));
		tft.setCursor(100,94);
		tft.print(mmPioggiaanno, 1);
		tft.println(F("mm"));
	tft.drawLine(0,104,159,104,0x5d5348); 
	tft.setCursor(0,107);    
	tft.setTextColor (ST7735_WHITE);    
	tft.print(F("RR "));
	tft.print(rainrate, 1); 
	tft.print(F("mm/h"));
	tft.setCursor(77,107);
	tft.print(F("M"));
	tft.setTextColor (ST7735_RED);
	tft.print(DayRainRate.max, 1);
	tft.setTextColor (ST7735_WHITE);
	tft.print(F(" "));
	tft.print(freeMemory());
	//tft.setTextColor (ST7735_GREEN);
	//tft.println(mediaRR, 1);
		tft.drawLine(0,115,159,115,ST7735_RED);
		tft.setCursor(0,118);
		tft.setTextColor (ST7735_YELLOW);
		//tft.print(writeExtreme);
		tft.print(resetStatusTx);
		tft.print(" ");
		tft.setTextColor (ST7735_GREEN);
		tft.print(Hour);
		tft.print(":");
		tft.print(Minute);
		tft.print(":");
		tft.print(Second);
		tft.print("-");
		tft.print(Day);
		tft.print("/");
		tft.print(Month);
		tft.print(" ");
		tft.setTextColor (ST7735_YELLOW);
		tft.print(radioRx.ndata);
		tft.drawLine(118,116,118,126,ST7735_RED);
		tft.setCursor(120,118);
		tft.setTextColor (ST7735_WHITE);
		tft.print(barometer.readTemperature(),1);
		tft.print ((char)248);
		tft.print(F("C"));
}

///////////////////////////////////INIT ESTREMI
void initEstremi(){
	if (INIZIALIZZAZIONE) {
		pNow = pressione;
		pSix = pressione;
		pFive = pressione;
		pFour = pressione;
		pThree = pressione;
		pTwo = pressione;
		pOne = pressione;

		DayTemp.max = -100; //temperatura massima giornaliera
		DayTemp.min = 100; //temperatura minima giornaliera
		DayHum.max = -2; // umidità massima giornaliera
		DayHum.min = 150; // umidità minima giornaliera
		DayDewPoint.max = -100; // punto di rugiada massimo giornaliero
		DayDewPoint.min = 100; // punto di rugiada minimo giornaliero
		DayWind.max = 0; // raffica massima giornaliera
		DayChill.min = 100; // raffreddamento da vento minimo giornaliero
		DayHeatIndex.max = -100; // indice di calore massimo giornaliero
		DayRainRate.max = 0; // intensità della pioggia massima giornaliero
	}
}

///////////////////////////////RADIO 1
bool radioLink=false; //no collegamento radio con master
bool checkReceiveLLCC68() {
	if (e220ttl.available()>1) {
		ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(radioRx));
		if (serialOpen) Serial.println(F("riceved"));
		
		if (rsc.status.code!=1){
			if (serialOpen) Serial.println(rsc.status.getResponseDescription());
			return 0;
		}else{
			// Print the data received
			if (serialOpen) Serial.println(rsc.status.getResponseDescription());
			radioRx = *(radioVal*) rsc.data;
			rsc.close();
			if (serialOpen){
				Serial.println(radioRx.date);
				Serial.println(radioRx.pioggiaday);
			}
			return 1;   
		}
	} else
		return 0;
}
void sendLLCC68() {
	// Send message
    ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 23, &radioTx, sizeof(radioTx));
    // Check If there is some problem of succesfully send
    if (serialOpen) Serial.println(rs.getResponseDescription());
}

int sendATcommand(String ATcommand, const char* expected_answer, const unsigned long timeout, const bool output = false){
  const uint8_t RESPONSE_SIZE = 160;
  //response array could be made global
  char response[RESPONSE_SIZE];
  
  bool answer = false;
  bool answer601 = false;
  uint8_t respx = 0;
  unsigned long previous = 0ul;
  
  //clear whole array
  memset(response, '\0', RESPONSE_SIZE);

  //clean the input buffer (with timeout)
  previous = millis();
  while ( (Serial3.available() > 0) && ((millis() - previous) < 1000ul) )
  {
    Serial3.read();    
  }
  
  //send the AT command
  Serial3.println(ATcommand);    

  previous = millis();
  wdt_reset();
  //this loop waits for the answer with timeout
  do      
  {
	wdt_reset();
    if (Serial3.available() != 0)
    {
      //if there is data in the UART input buffer, reads it and checks for the answer
      response[respx] = Serial3.read();
      respx++;
      
      //prevent buffer overflow
      if (respx >= RESPONSE_SIZE)
      {
        respx = RESPONSE_SIZE - 1;
      }

      //check if the desired answer is in the response of the module
      if (strstr(response, expected_answer) != NULL)
      {
        answer = true;
      }
	  //check if 601 Network Error
	  if (strstr(response, ",601,") != NULL){
        answer601 = true;
      }
    }
  }
  while (!answer && !answer601 && ((millis() - previous) < timeout));
  wdt_reset();
  //terminate the string, respx was limited to (RESPONSE_SIZE - 1)
  response[respx] = '\0';

  if (output){ // se non risposta che cercavo, print risposta
	for(int i=0 ; i<respx ; ++i){
		Serial.print(response[i]);
	}
	Serial.println();
  }

  if (answer)
  	return 1;
  else if (answer601)
  	return 2;
  else 
  	return 0; //timeout
}

int initSIM800L(){
  pinMode(24, INPUT);  //reset pin
  Serial3.begin(9600); //Begin serial communication with Arduino and SIM800L
  delay(1000);
  int ok = sendATcommand("AT", "OK", 2000);
  sendATcommand("AT+CSQ", "OK", 2000); //Signal quality test, value range is 0-31 , 31 is the best
  sendATcommand("AT+CCID", "OK", 2000); //Read SIM information to confirm whether the SIM is plugged
  sendATcommand("AT+CREG?", "OK", 2000); //Check whether it has registered in the network
  sendATcommand("AT+CFUN=1", "OK", 2000); //full funzioni
  sendATcommand("AT+CGATT=1", "OK", 2000); //collegati a gprs
  sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 2000);
  sendATcommand("AT+SAPBR=3,1,\"APN\",\"apn.fastweb.it\"", "OK", 2000);
  sendATcommand("AT+SAPBR=1,1", "OK", 2000);
  sendATcommand("AT+SAPBR=2,1", "OK", 2000); //ok to use??
  return ok;
}

bool resetSIM800(){
	pinMode(24, OUTPUT);
	digitalWrite(24, LOW);
	delay(50);
	pinMode(24, INPUT);
	delay(200);
	int ok = sendATcommand("AT", "OK", 2000);
	sendATcommand("AT+CSQ", "OK", 2000); //Signal quality test, value range is 0-31 , 31 is the best
	sendATcommand("AT+CCID", "OK", 2000); //Read SIM information to confirm whether the SIM is plugged
	sendATcommand("AT+CREG?", "OK", 2000); //Check whether it has registered in the network
	sendATcommand("AT+CFUN=1", "OK", 2000); //full funzioni
	sendATcommand("AT+CGATT=1", "OK", 2000); //collegati a gprs
	sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 2000);
	sendATcommand("AT+SAPBR=3,1,\"APN\",\"apn.fastweb.it\"", "OK", 2000);
	sendATcommand("AT+SAPBR=1,1", "OK", 2000);
	sendATcommand("AT+SAPBR=2,1", "OK", 2000); //ok to use??
	return ok;
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    Serial3.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(Serial3.available()) 
  {
    Serial.write(Serial3.read());//Forward what Software Serial received to Serial Port
  }
}

int serverRequest(){
  sendATcommand("AT+HTTPINIT", "OK", 2000);
  sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 2000);
  sendATcommand("AT+HTTPPARA=\"REDIR\",1", "OK", 2000);
  sendATcommand("AT+HTTPPARA=\"URL\",\"http://212.227.60.35/php-obtain/cremolino.php\"", "OK", 2000);
  sendATcommand("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"", "OK", 2000);

  String postData = "p=date,"+String(Year)+"-"+String(Month)+"-"+String(Day)+"+"+String(Hour)+"%3A"+String(Minute)+"%3A"+String(Second)+",tp,"+String(tp)+",tpdht,"+String(tpdht)+",tp2,"+String(tp2)+",ur,"+String(ur)+",ur2,"+String(ur2)+",dewpoint,"+String(dewPoint)+",heatindex,"+String(heatindexc)+",pressione,"+String(PR)+",pressionelivellodelmare,"+String(SLPR)+",wind,"+String(wind.val)+",windAng,"+String(gradi[radioRx.winddir])+",windchill,"+String(windchill)+",mmPioggia,"+String(mmPioggia)+",rainrate,"+String(rainrate)+",wetbulb,"+String(wetbulb)+",nData,"+String(radioRx.ndata)+",tpmax,"+String(tpmax)+",tpmin," + String(tpmin) + ",urmax," + String(urmax) + ",urmin," + String(urmin) + ",windmax," + String(wind.max) + ",rainratemax," + String(rainratemax)+",bmp_status," + String(erbmp) + ",sd_status," + String(ersd) + ",sd_tx_status," + String(ersdTx) + ",volt," + String(float(radioRx.volt)/10) + ",reset_tx," + String(resetStatusTx)+ ",ack_tx," + String(radioRx.send_count);
  sendATcommand("AT+HTTPDATA=" + String(postData.length()) + ",120000", "OK", 2000);
  sendATcommand(postData, "OK", 2000);
  delay(50); //altrimenti httpaction return 0
  int r = sendATcommand("AT+HTTPACTION=1", "+HTTPACTION: 1,200,", 20000, 1);
  sendATcommand("AT+HTTPREAD", "OK", 2000, 1);
  sendATcommand("AT+HTTPTERM", "OK", 2000);
  return r;
}

ArduinoOutStream coutD(Serial);
void coutData(){
  //dataday
  coutD << int(Year) << "-" << int(Month) << "-" << int(Day) << " " << int(Hour) << ":" << int(Minute) << ":" << int(Second) << " "
  << tp << " " << tpdht << " " << tp2 << " " << ur << " " << int(ur2) << " " << PR << " " << SLPR << " "
  << wind.val << " " << gradi[radioRx.winddir] << " " << mmPioggia << " " << rainrate << " " << int(radioRx.ndata) << " "
  << tpmax << " " << tpmin << " " << urmax << " " << urmin << " " << wind.max << " " << rainratemax << endl;
  //control
  coutD << float(radioRx.volt)/10 << " " << int(erbmp) << " " << int(ersd) << " " << int(ersdTx) << " " << int(resetStatusTx) << " " << int(radioRx.send_count) << endl;//<< " " << radioRx.memoryFree << endl;
}

unsigned long lastLLCC68data = 0; // time of last receiving data
void check_LLCC68data(){
	if (millis() - lastLLCC68data > 21600000){ //6 ore senza ricevere dati
		Serial.println(F("NO LLCC68 data -> reset avr"));
		delay(50);
		while (1){
			;
		}
	}
}

unsigned long lastSX1262data = 0;
void check_SX1262data(){
	if (millis() - lastSX1262data > 21600000){ //6 ore senza ricevere dati
		lastSX1262data = millis();
		Serial.println(F("NO SX1262 data -> reset SX1262"));
		resetSX1262();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {  
	Serial.begin(9600);
	while (!Serial){
		;
	}
	SPI.begin();
	Wire.begin();
	digitalWrite(SDA, 0);//disable internal i2c pull up
	digitalWrite(SCL, 0);
	
	wdt_enable(WDTO_8S);//watchdog a 8 secondi
	
	setPin();
	initEstremi();
	initTFT();
	initSD();
	initLLCC68();
	initSX1262();
	initPressure();
	readEstremi();
	Serial.print(F("initSIM "));
	int sim_status = initSIM800L();
	Serial.println(sim_status);
	if (!sim_status)
		Serial.println(resetSIM800());
}

unsigned long prevTimestamp = 0, prevSecond=0;
unsigned long timeout = 0;	//secondi dall'ultimo contatto radio
bool radioLinkMini = 0;
bool sendSD = 0, sendServerMini = 0;
unsigned long prevSW = -60000, prevSD = -60000;
void loop(){
	if (checkReceiveLLCC68() && millis()){ //radio 1
		Serial.println(F("0.1"));
		wdt_reset();
		radioTx.date = radioRx.date; //ack id to resent
		delay(100);
		sendLLCC68(); //send back packet as ack
		delay(100);
		radioLink = true;
		sendSD = true;
		Serial.println(F("0..1"));
	}
	if (checkReceiveSX1262() == (uint8_t)sizeof(radioRxMini)){ //radio 2
		Serial.println(F("0.2"));
		wdt_reset();
		radioTxMini.date = radioRxMini.date; //ack id to resent
		delay(100);
		sendSX1262(); //send back packet as ack
		delay(100);
		radioLinkMini = true;
		sendServerMini = true;
		Serial.print(F("CS "));
		Serial.println(radioRxMini.send_count);
		Serial.println(F("0..2"));
	}
	if(millis()-prevSecond > 1000){
		wdt_reset();
		//Serial.println(F("Pressure"));
		prevSecond = millis();
		button();			//button reset master and update display 
		readPressure();		//misuro pressione e calcolo media
		resetMedie();		//ogni 10 min resetto medie del display
		check_LLCC68data();
		check_SX1262data();
	}
	if(radioLink){
		wdt_reset();
		Serial.println(F("0...1"));
		radioLink = false;
		timeout = millis();
		lastLLCC68data = timeout; //time of receiving
		erroriTx();			//decodifica errori master
		getTime();			//prendo data attuale dal valore timestamp ricevuto da radio
		if(onetime){		//funzioni da eseguire solo al primo loop, dopo aver ricevuto  i dati
			readEstremi();
			onetime = 0;
		}
		resetEstremi();
		readTemp();
		readHum();
		readWind();
		readWindDir();
		readPioggia();
		readDewPoint();
		readWindchill();
		readHeatIndex();
		readWetbulb();
		if (radioRx.date != prevTimestamp){	//se sono arrivati dati freschi
			prevTimestamp = radioRx.date;
			Datalog();
			resetPressure();	//Problema: pressione mediata su intervallo <= 1' se radiolink manca per più di 1' (interviene sempre questo reset appena radiolink torna attivo)
		}
		if(readEstremiOk)
			writeEstremi();
		if (!flagWSmini) //se attivo display ws
			readDisplay();
		//dati mediati su 1 minuto
    	Serial.println(F("%1")); //marker to start serial read python
		coutData();
		Serial.println(F("0....1"));
	} else if (millis() - timeout > 60000) { //media pressione resettata ogni minuto anche senza radiolink (altrimenti pressione mediata su intervallo > 1' quando radiolink torna attivo)
		timeout = millis();
		resetPressure();
	}
	if (sendSD && millis() - prevSD > 10000){ //try send to server, after data received and every 10 sec if not succes
		Serial.println(F("D."));
		sD = serverRequest();
		Serial.print(F("D.."));
		Serial.println(sD);
		if (sD == 1 )
			sendSD = false;
		else if (sD == 2){
			Serial.print(F("resetting SIM800.."));
			Serial.println(resetSIM800());
		}
		else
			prevSD = millis();
		//Serial.println(freeMemory());
	}
///////////////////////////////////////////////////////// WS MINI STUFF /////////////////////////////////////////////////////	
	if (radioLinkMini){
		wdt_reset();
		radioLinkMini = false;
		lastSX1262data = millis(); //time of receiving
		getWSMiniData();
		getTimeMini();
		datalogMini();
		Serial.println(F("%2"));
		coutDataMini();
	}
	if (sendServerMini && millis() - prevSW > 30000){ //try send to server, after data received and every 30 sec if not succes
		Serial.println(F("W."));
		sW = serverMiniWS();
		Serial.print(F("W.."));
		Serial.println(sW);
		if (sW == 1)
			sendServerMini = false;
		else if (sW == 2){
			Serial.print(F("reset SIM800.."));
			Serial.println(resetSIM800());
		}
		else
			prevSW = millis();

		if (flagWSmini) //se attivo wsmini display
			readDisplayMini();
	}
}

float ws_tp, ws_tpbmp, ws_ur, ws_pressure, ws_mmpioggia, ws_rainrate, ws_tpmax, ws_tpmin, ws_urmax, ws_urmin, ws_volt, ws_slpressure;
int ws_ndata;
unsigned int ws_statusDev;
void getWSMiniData(){
	ws_tp = (float)radioRxMini.TP/100;
	ws_tpbmp = (float)radioRxMini.TPBMP/100;
	ws_ur = (float)radioRxMini.UR/100;
	ws_pressure = (float)radioRxMini.pressure/100;
	ws_mmpioggia = (float)radioRxMini.mmpioggia/100;
	ws_rainrate = (float)radioRxMini.rainrate/100;
	ws_tpmax = (float)radioRxMini.tpmax/100;
	ws_tpmin = (float)radioRxMini.tpmin/100;
	ws_urmax = (float)radioRxMini.urmax/100;
	ws_urmin = (float)radioRxMini.urmin/100;
	ws_ndata = (int)radioRxMini.ndata;
	ws_volt = (float)radioRxMini.volt/100;
	ws_statusDev = radioRxMini.statusDev;
	ws_slpressure = (ws_pressure) / pow(1.0 - 310.0 / 44330, 5.255);
}

//RADIO 2
void initSX1262() {
	// Begin LoRa radio and set NSS, reset, busy, dio1, txen, and rxen pin with connected arduino pins
	if (serialOpen) Serial.println("Begin LoRa radio");

	if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)){
		Serial.println(F("SX1262 fail"));
	}else{
		Serial.println(F("SX1262 ok"));
	}

	// Configure TCXO or XTAL used in RF module
	//Serial.println("Set RF module to use TCXO as clock reference");
	uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
	uint32_t tcxoDelay = SX126X_TCXO_DELAY_10;
	LoRa.setDio3TcxoCtrl(dio3Voltage, tcxoDelay);
	
	// Set frequency to 866.6 Mhz
	//Serial.println("Set frequency to 866.6 Mhz");
	LoRa.setFrequency(866600000);

	// Set TX power, default power for SX1262 and SX1268 are +22 dBm and for SX1261 is +14 dBm
	// This function will set PA config with optimal setting for requested TX power
	//Serial.println("Set TX power to +22 dBm");
	LoRa.setTxPower(22, SX126X_TX_POWER_SX1262);

	// Set RX gain to boosted gain
	//Serial.println("Set RX gain to power saving gain");
	//LoRa.setRxGain(SX126X_RX_GAIN_BOOSTED);
	LoRa.setRxGain(SX126X_RX_GAIN_POWER_SAVING);

	// Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
	//Serial.println("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5");
	uint8_t sf = 7;
	uint32_t bw = 125000;
	uint8_t cr = 5;
	LoRa.setLoRaModulation(sf, bw, cr);

	// Configure packet parameter including header type, preamble length, payload length, and CRC type
	//Serial.println("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on");
	uint8_t headerType = SX126X_HEADER_EXPLICIT;
	uint16_t preambleLength = 12;
	uint8_t payloadLength = 15;
	bool crcType = true;
	LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

	// Set syncronize word for public network (0x3444)
	//Serial.println("Set syncronize word to 0x3444");
	LoRa.setSyncWord(0x1500);

    LoRa.request(SX126X_RX_CONTINUOUS); // Request for receiving new LoRa packet in RX continuous mode, block sometimes
	//LoRa.request(LORA_RX_SINGLE);
}

void resetSX1262(){
	//DEBUG
	Serial.println(F("STATUS R1"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());

	Serial.println(LoRa.reset());
	delay(100);
	initSX1262();
	
	//DEBUG
	Serial.println(F("STATUS R2"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());
}
/*
static uint8_t errorCount = 0;

if (status == SX126X_STATUS_HEADER_ERR || status == SX126X_STATUS_CRC_ERR) {
    errorCount++;
    if (errorCount > 100) {
        Serial.println("Too many errors, resetting SX1262...");
        LoRa.reset();
        initSX1262();
        errorCount = 0;
    }
    return 0;
} else {
    errorCount = 0; // reset on valid packet
}

*/
void printStatus(uint8_t status) {
  switch (status) {
    case SX126X_STATUS_DATA_AVAILABLE:
      Serial.println(F("Status: DATA AVAILABLE (Packet received, data can be retrieved)"));
      break;
    case SX126X_STATUS_CMD_TIMEOUT:
      Serial.println(F("Status: COMMAND TIMEOUT (SPI command timed out)"));
      break;
    case SX126X_STATUS_CMD_ERROR:
      Serial.println(F("Status: COMMAND ERROR (Invalid SPI command)"));
      break;
    case SX126X_STATUS_CMD_FAILED:
      Serial.println(F("Status: COMMAND FAILED (SPI command failed to execute)"));
      break;
    case SX126X_STATUS_CMD_TX_DONE:
      Serial.println(F("Status: TX DONE (Packet transmission done)"));
      break;
    case SX126X_STATUS_MODE_STDBY_RC:
      Serial.println(F("Status: MODE STDBY_RC (Standby using RC oscillator)"));
      break;
    case SX126X_STATUS_MODE_STDBY_XOSC:
      Serial.println(F("Status: MODE STDBY_XOSC (Standby using XOSC)"));
      break;
    case SX126X_STATUS_MODE_FS:
      Serial.println(F("Status: MODE FS (Frequency Synthesizer mode)"));
      break;
    case SX126X_STATUS_MODE_RX:
      Serial.println(F("Status: MODE RX (Receiving)"));
      break;
    case SX126X_STATUS_MODE_TX:
      Serial.println(F("Status: MODE TX (Transmitting)"));
      break;
    default:
      Serial.print(F("Status: UNKNOWN (0x"));
      Serial.print(status, HEX);
      Serial.println(F(")"));
      break;
  }
}

/*
// Status TX and RX operation
#define LORA_STATUS_DEFAULT                     0           // default status (false)
#define LORA_STATUS_TX_WAIT                     1
#define LORA_STATUS_TX_TIMEOUT                  2
#define LORA_STATUS_TX_DONE                     3
#define LORA_STATUS_RX_WAIT                     4
#define LORA_STATUS_RX_CONTINUOUS               5
#define LORA_STATUS_RX_TIMEOUT                  6
#define LORA_STATUS_RX_DONE                     7
#define LORA_STATUS_HEADER_ERR                  8
#define LORA_STATUS_CRC_ERR                     9
#define LORA_STATUS_CAD_WAIT                    10
#define LORA_STATUS_CAD_DETECTED                11
#define LORA_STATUS_CAD_DONE                    12
*/

///////////////////////////////RADIO 2
uint8_t checkReceiveSX1262(){	
  // read() and available() method must be called after request() or listen() method
  // request() command is after send function
  const uint8_t msgLen = LoRa.available();
  if (msgLen) {
    // Show received status in case CRC or header error occur
    uint8_t status = LoRa.status();
    if (status == SX126X_STATUS_CRC_ERR) {
		//LoRa.request(LORA_RX_SINGLE);
		//delay(50);
		Serial.println("CRC error");
	    return 0;
    }
    else if (status == SX126X_STATUS_HEADER_ERR) {
		//LoRa.request(LORA_RX_SINGLE);
		//delay(50);
		Serial.println("Packet header error");
	    return 0;
    } else if (msgLen == (uint8_t)sizeof(radioRxMini)){ //avoid garbage data
		LoRa.read((uint8_t*)&radioRxMini, msgLen);
		//check date, avoid false packet
		/*tmElements_t ccc;
		breakTime (radioRxMini.date, ccc);
		int diffYear = ccc.Year+1970 - Year; //if year changed only by one
		if (abs(diffYear) <= 1) {*/
			// Print packet/signal status including package RSSI and SNR
			Serial.print("RSSI=");
			Serial.print(LoRa.packetRssi());
			Serial.print("dBm SNR=");
			Serial.print(LoRa.snr());
			Serial.println("dB ");

			//DEBUG
			Serial.println(F("STATUS A"));
			printStatus(LoRa.getMode());
			Serial.println(status);

			return msgLen;
		//}else
			//Serial.println(F("Datetime doesn't match"));
	}
  }
  return 0;
}

void sendSX1262(){
	//DEBUG
	Serial.println(F("STATUS send"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());

  	LoRa.beginPacket();
	//DEBUG
	Serial.println(F("STATUS begin"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());

	Serial.println(F("write"));
  	LoRa.write((uint8_t*)&radioTxMini, (uint8_t)sizeof(radioTxMini));
	//DEBUG
	Serial.println(F("STATUS write"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());

	Serial.println(F("endPacket"));
  	Serial.println(LoRa.endPacket(2000));
	//DEBUG
	Serial.println(F("STATUS end"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());

	Serial.println(F("wait"));
  	LoRa.wait(2000); // Wait until modulation process for transmitting packet finish
	Serial.println(F("purge"));
	if (LoRa.available() > 0) { //remove garbage data autoreturn
		LoRa.purge(LoRa.available());
	}
	delay(50);

	//DEBUG
	Serial.println(F("STATUS C"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());

	LoRa.request(SX126X_RX_CONTINUOUS); // Request for receiving new LoRa packet in RX continuous mode, richiamare solo dopo pacchetto inviato
	//LoRa.request(LORA_RX_SINGLE); //Single va richiamata dopo ogni errore o pacchetto ricevuto o pacchetto inviato
	delay(50);
	
	//DEBUG
	Serial.println(F("STATUS D"));
	printStatus(LoRa.getMode());
	Serial.println(LoRa.status());

	if (LoRa.getMode() != SX126X_STATUS_MODE_RX || LoRa.status() != 5) {
		Serial.println(F("CONT fail -> reset SX1262"));
		resetSX1262();
    }

	if (serialOpen){
		Serial.print("Sent timestamp  ");
  		Serial.println(radioRxMini.date);
		Serial.print(" TrTime:"); // Print transmit time
		Serial.print(LoRa.transmitTime());
		Serial.println("ms");
	}
}

//////////////////////////DECODE DATASTAMP FROM RADIO 2
tmElements_t m;
void getTimeMini() {
  breakTime (radioRxMini.date, m);//usare m.Hour m.Minute m.Second m.Day m.Month m.Year per registrare valori orari
}

int serverMiniWS(){
  /*Serial3.println("AT+HTTPINIT"); //Init HTTP service
  resp("OK");
  Serial3.println("AT+HTTPPARA=\"CID\",1");
  resp("OK");
  Serial3.println("AT+HTTPPARA=\"REDIR\",1"); //Auto redirect
  resp("OK");
  Serial3.println("AT+HTTPPARA=\"TIMEOUT\",60"); //timeout
  resp("OK");
  Serial3.println("AT+HTTPPARA=\"URL\",\"http://212.227.60.35/php-obtain/wsmini.php?p=date,"+String(m.Year+1970)+"-"+String(m.Month)+"-"+String(m.Day)+"+"+String(m.Hour)+"%3A"+String(m.Minute)+"%3A"+String(m.Second)+",tp," + String(ws_tp) + ",tpbmp," + String(ws_tpbmp) + ",ur," + String(ws_ur) + ",pressione," + String(ws_pressure) + ",mmpioggia," + String(ws_mmpioggia) + ",rainrate," + String(ws_rainrate) + ",tpmax," + String(ws_tpmax) + ",tpmin," + String(ws_tpmin) + ",urmax," + String(ws_urmax) + ",urmin," + String(ws_urmin) + ",ndata," + String(ws_ndata) + ",volt," + String(ws_volt) + ",status," + String(ws_statusDev) + ",pressionelivellodelmare," + String(ws_slpressure)+"\"");
  resp("OK");   
  Serial3.println("AT+HTTPACTION=0");  //call
  int r = resp("+HTTPACTION: 0,200,0");
  Serial3.println("AT+HTTPTERM");
  resp("HTTPTERM");
  return r;*/
  sendATcommand("AT+HTTPINIT", "OK", 2000);
  sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 2000);
  sendATcommand("AT+HTTPPARA=\"REDIR\",1", "OK", 2000);
  sendATcommand("AT+HTTPPARA=\"URL\",\"http://212.227.60.35/php-obtain/wsmini.php?p=date,"+String(m.Year+1970)+"-"+String(m.Month)+"-"+String(m.Day)+"+"+String(m.Hour)+"%3A"+String(m.Minute)+"%3A"+String(m.Second)+",tp," + String(ws_tp) + ",tpbmp," + String(ws_tpbmp) + ",ur," + String(ws_ur) + ",pressione," + String(ws_pressure) + ",mmpioggia," + String(ws_mmpioggia) + ",rainrate," + String(ws_rainrate) + ",tpmax," + String(ws_tpmax) + ",tpmin," + String(ws_tpmin) + ",urmax," + String(ws_urmax) + ",urmin," + String(ws_urmin) + ",ndata," + String(ws_ndata) + ",volt," + String(ws_volt) + ",status," + String(ws_statusDev) + ",pressionelivellodelmare," + String(ws_slpressure)+"\"", "OK", 4000);
  int r = sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,", 6000, 1);
  //sendATcommand("AT+HTTPREAD", "", 2000);
  sendATcommand("AT+HTTPTERM", "OK", 2000);
  return r;
}

ArduinoOutStream coutS(Serial);
void coutDataMini(){
  coutS << int(m.Year+1970) << "-" << int(m.Month) << "-" << int(m.Day) << " " << int(m.Hour) << ":" << int(m.Minute) << ":" << int(m.Second) << " " << ws_tp << " " << ws_tpbmp << " " << ws_ur << " "
  << ws_pressure << " " << ws_mmpioggia << " " << ws_rainrate << " " << ws_tpmax << " " << ws_tpmin << " " << ws_urmax << " " << ws_urmin << " " << ws_ndata
  << " " << ws_volt << " " << ws_statusDev << " " << ws_slpressure << endl;
}

//SAVE IN SD 2
void datalogMini() { 
  bool sdStatus  = SD.begin(SD_SS, SPI_HALF_SPEED);
  if (sdStatus) {
    ofstream dataday ("wsmini.csv", ios_base::app);

    dataday << int(m.Year+1970) << "-" << int(m.Month) << "-" << int(m.Day) << " " << int(m.Hour) << ":" << int(m.Minute) << ":" << int(m.Second) << " " << ws_tp << " " << ws_tpbmp << " " << ws_ur << " "
    << ws_pressure << " " << ws_mmpioggia << " " << ws_rainrate << " " << ws_tpmax << " " << ws_tpmin << " " << ws_urmax << " " << ws_urmin << " " << ws_ndata
    << " " << ws_volt << " " << ws_statusDev << " " << ws_slpressure << " " << int(radioRxMini.send_count) << endl;

    dataday.close();
    //Serial.println(F("written wsmini"));
  }  
}

///////////////////////////////////////DISPLAY/////////////
void readDisplayMini() {
	tft.fillScreen(ST7735_BLACK);
	tft.setTextColor (ST7735_WHITE);
	tft.setCursor(0, 0);
	tft.setTextSize(1);
		tft.print(F("T "));
		tft.print(ws_tp, 2);
		tft.print(F("  "));
		tft.print(ws_tpbmp, 2);
		tft.print ((char)248);
		tft.print(F("C"));
		tft.setCursor(82,0);
		tft.print(F(" "));
		tft.setTextColor (ST7735_BLUE);
		tft.print(ws_tpmin, 2);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F(" "));
		tft.setTextColor (ST7735_RED);
		tft.print(ws_tpmax, 2);
		tft.drawLine(148,9,148,41,ST7735_CYAN);
	tft.drawLine(0,8,159,8,0x5d5348);
	tft.setCursor(0,11);    
	tft.setTextColor (ST7735_WHITE);    
	tft.print(F("U"));
	tft.setCursor(17,11);
	tft.print(ws_ur, 2);
	tft.print(F("%"));
	tft.setCursor(72,11);
	tft.print(F(" "));
	tft.setTextColor (ST7735_BLUE);
	tft.print(ws_urmin, 0);
	tft.print(F("%"));
	tft.setTextColor (ST7735_WHITE);
	tft.print(F(" "));
	tft.setTextColor (ST7735_RED);
	tft.print(ws_urmax, 2);
	tft.print(F("%"));

	//tft.setTextColor (ST7735_GREEN);
	//tft.println(erbmp);       /////////errore bmp180
		/*tft.drawLine(0,19,148,19,0x5d5348);
		tft.setCursor(0,22); 
		tft.setTextColor (ST7735_WHITE);
		tft.print(F("DP"));
		tft.setCursor(17,22);
		tft.print(dewPoint, 1); 
		tft.print((char)248);
		tft.print(F("C"));
		tft.setCursor(72,22);
		tft.print(F(" "));
		tft.setTextColor (ST7735_BLUE);
		tft.print(DayDewPoint.min, 1);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F(" "));
		tft.setTextColor (ST7735_RED);
		tft.print(DayDewPoint.max, 1);
		tft.setTextColor (ST7735_WHITE);
		tft.print(F(" "));
		tft.setTextColor (ST7735_YELLOW);
		tft.print(andamentoDp);
		tft.setCursor(149,22);
		tft.setTextColor (ST7735_WHITE);
		tft.print(radioRx.send_count);   ///////errore mancato collegamento radio con master
		tft.setCursor(154,22);
		tft.setTextColor (ST7735_GREEN);
		tft.println(erradioRX);*/  ////////errore radio init slave
	/*tft.drawLine(0,30,148,30,0x5d5348);
	tft.setCursor(0,33);     
	tft.setTextColor (ST7735_WHITE);    
	tft.print(F("HI"));
	tft.setCursor(17,33);
	tft.print(heatindexc, 1); 
	tft.print((char)248);
	tft.print(F("C"));
	tft.setCursor(72,33);
	tft.print(F(" "));
	tft.setTextColor (ST7735_RED);
	tft.print(DayHeatIndex.max, 1);
	tft.print((char)248);
	tft.print(F("C"));
	tft.setTextColor (ST7735_WHITE);
	tft.print(F(" "));
	tft.setTextColor (ST7735_YELLOW);
	tft.println(F("="));
	tft.setCursor(149,33);
	tft.setTextColor (ST7735_WHITE);
	tft.print(ersdTx);
	tft.setCursor(154,33);
	tft.setTextColor (ST7735_GREEN);
	tft.println(ersd);*/
			tft.drawLine(0,41,159,41,ST7735_RED);
			tft.setCursor(0,44);
			tft.setTextColor (ST7735_WHITE);
			tft.print(F("P "));
			tft.setCursor(13,44);
			tft.print(ws_slpressure/100, 1); 
			tft.print(F(" hPa"));
		tft.setCursor(13,52);
		tft.setTextColor (ST7735_WHITE);
		tft.print(ws_pressure/100, 2);
		tft.setCursor(127,52);
		tft.print(ws_volt, 2);
		tft.println(F("V"));
	tft.drawLine(0,83,159,83,ST7735_RED);
	tft.setCursor(0,86);    
	tft.setTextColor (ST7735_BLUE);    
	tft.print(F("Day"));
	tft.setTextColor (ST7735_BLUE);
	tft.setCursor(45,86);
	/*tft.print(F("Month"));
	tft.setTextColor (ST7735_BLUE); 
	tft.setCursor(100,86);
	tft.println(F("Year"));
	tft.setCursor(130,86);*/
		tft.setTextColor (ST7735_WHITE);
		tft.print(ws_mmpioggia, 2);
		tft.print(F("mm"));
		/*tft.setCursor(45,94);
		tft.print(mmPioggiamese, 1);
		tft.print(F("mm"));
		tft.setCursor(100,94);
		tft.print(mmPioggiaanno, 1);
		tft.println(F("mm"));*/
	tft.drawLine(0,104,159,104,0x5d5348); 
	tft.setCursor(0,107);    
	tft.setTextColor (ST7735_WHITE);    
	tft.print(F("RR "));
	tft.print(ws_rainrate, 2); 
	tft.print(F("mm/h"));
	/*tft.setCursor(77,107);
	tft.print(F("M"));
	tft.setTextColor (ST7735_RED);
	tft.print(DayRainRate.max, 1);*/
		tft.drawLine(0,115,159,115,ST7735_RED);
		tft.setCursor(0,118);
		tft.setTextColor (ST7735_YELLOW);
		//tft.print(resetStatusTx);
		tft.print(" ");
		tft.setTextColor (ST7735_GREEN);
		tft.print(m.Hour);
		tft.print(":");
		tft.print(m.Minute);
		tft.print(":");
		tft.print(m.Second);
		tft.print("-");
		tft.print(m.Day);
		tft.print("/");
		tft.print(m.Month);
		tft.print(" ");
		tft.setTextColor (ST7735_YELLOW);
		tft.print(ws_ndata);
		tft.print(F("  "));
		tft.print(sW);
		tft.print(F("  R "));
		tft.print(radioRxMini.send_count);
}
