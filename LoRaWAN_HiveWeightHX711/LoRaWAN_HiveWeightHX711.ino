#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <Seeed_BME280.h>
#include <HX711.h>

#define DATA_PIN  GPIO4
#define CLOCK_PIN GPIO3

const float ScaleFactor = 47.1619;       // Temperatur zum Zeitpunkt der Kalibrierung
const int ScaleOffset = 20150; // Korrekturwert zur Temperaturkompensation - '0.0' für Deaktivierung
const float Kalibriertemperatur = 20.93;       // Temperatur zum Zeitpunkt der Kalibrierung
const float KorrekturwertGrammproGrad = 2.44; // Korrekturwert zur Temperaturkompensation - '0.0' für Deaktivierung

/* OTAA para*/
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
//uint32_t appTxDutyCycle = 15000;
uint32_t appTxDutyCycle = 900000; //15 Minutes

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

float temperature, humidity, BMPaltitude;
long pressure;

BME280 bme280;

HX711 scale = HX711();

static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(500);
  if(!bme280.init()){
  }
  delay(500);

  temperature = bme280.getTemperature();
  pressure = bme280.getPressure();
  BMPaltitude = bme280.calcAltitude(pressure);
  humidity = bme280.getHumidity();
  Wire.end();

  scale.begin(DATA_PIN, CLOCK_PIN);
  scale.set_scale(ScaleFactor);
  scale.set_offset(ScaleOffset);
  long weight = scale.get_units(3);
  long CorrectedWeight = weight+((temperature-Kalibriertemperatur)*KorrekturwertGrammproGrad);

  digitalWrite(Vext, HIGH);
  
  uint16_t batteryVoltage = getBatteryVoltage();
  word temp = temperature * 10;
  int hum = round(humidity);
  word alt = BMPaltitude * 10;
  unsigned char *puc;
  appDataSize = 15;

  appData[0] = temp >> 8;
  appData[1] = temp;

  puc = (unsigned char *)(&hum);
  appData[2] = puc[0];
  
  appData[3] = alt >> 8;
  appData[4] = alt;

  appData[5] = pressure >> 24;
  appData[6] = pressure >> 16;
  appData[7] = pressure >> 8;
  appData[8] = pressure;
  
  appData[9] = CorrectedWeight >> 24;
  appData[10] = CorrectedWeight >> 16;
  appData[11] = CorrectedWeight >> 8;
  appData[12] = CorrectedWeight;

  appData[13] = (uint8_t)(batteryVoltage >> 8);
  appData[14] = (uint8_t)batteryVoltage;
}


void setup() {
#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}
