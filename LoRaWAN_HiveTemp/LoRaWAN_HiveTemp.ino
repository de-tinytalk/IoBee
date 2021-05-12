#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <Seeed_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into GPIO1 on the CubeCell
#define ONE_WIRE_BUS GPIO1  // on pin GPIO1 PIN 6 (a 4.7K resistor is necessary)
#define TEMPERATURE_PRECISION 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire  oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

DeviceAddress TE010 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE020 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE030 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE040 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE050 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE060 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE070 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE080 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE090 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE100 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress TE110 = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

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

/*!
   \brief   Prepares the payload of the frame
*/

// function to print the temperature for a device
float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  //Serial.print(tempC);
  //Serial.print("°C, ");

  return tempC;
}

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
    //Serial.println("Device error!");
  }
  delay(500);

  temperature = bme280.getTemperature();
  pressure = bme280.getPressure();
  BMPaltitude = bme280.calcAltitude(pressure);
  humidity = bme280.getHumidity();
  Wire.end();

  //Start OneWire Bus Brood Temp
  sensors.begin();

  // set the resolution to 9 bit per device
  sensors.setResolution(TE010, TEMPERATURE_PRECISION);
  sensors.setResolution(TE020, TEMPERATURE_PRECISION);
  sensors.setResolution(TE030, TEMPERATURE_PRECISION);
  sensors.setResolution(TE040, TEMPERATURE_PRECISION);
  sensors.setResolution(TE050, TEMPERATURE_PRECISION);
  sensors.setResolution(TE060, TEMPERATURE_PRECISION);
  sensors.setResolution(TE070, TEMPERATURE_PRECISION);
  sensors.setResolution(TE080, TEMPERATURE_PRECISION);
  sensors.setResolution(TE090, TEMPERATURE_PRECISION);
  sensors.setResolution(TE100, TEMPERATURE_PRECISION);
  sensors.setResolution(TE110, TEMPERATURE_PRECISION);
  
  sensors.requestTemperatures();

  word te010 = printTemperature(TE010) * 10;
  word te020 = printTemperature(TE020) * 10;
  word te030 = printTemperature(TE030) * 10;
  word te040 = printTemperature(TE040) * 10;
  word te050 = printTemperature(TE050) * 10;
  word te060 = printTemperature(TE060) * 10;
  word te070 = printTemperature(TE070) * 10;
  word te080 = printTemperature(TE080) * 10;
  word te090 = printTemperature(TE090) * 10;
  word te100 = printTemperature(TE100) * 10;
  word te110 = printTemperature(TE110) * 10;
  
  digitalWrite(Vext, HIGH);
  
  uint16_t batteryVoltage = getBatteryVoltage();
  word temp = temperature * 10;
  int hum = round(humidity);
  word alt = BMPaltitude * 10;
  unsigned char *puc;
  appDataSize = 33;
  
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

  appData[9] = te010 >> 8;
  appData[10] = te010;
  
  appData[11] = te020 >> 8;
  appData[12] = te020;
  
  appData[13] = te030 >> 8;
  appData[14] = te030;
  
  appData[15] = te040 >> 8;
  appData[16] = te040;
  
  appData[17] = te050 >> 8;
  appData[18] = te050;
  
  appData[19] = te060 >> 8;
  appData[20] = te060;
  
  appData[21] = te070 >> 8;
  appData[22] = te070;
  
  appData[23] = te080 >> 8;
  appData[24] = te080;
  
  appData[25] = te090 >> 8;
  appData[26] = te090;

  appData[27] = te100 >> 8;
  appData[28] = te100;
  
  appData[29] = te110 >> 8;
  appData[30] = te110;

  appData[31] = (uint8_t)(batteryVoltage >> 8);
  appData[32] = (uint8_t)batteryVoltage;

  /*
  Serial.print("T=");
  Serial.print(temperature);
  Serial.print("C, RH=");
  Serial.print(humidity);
  Serial.print("%, A=");
  Serial.print(BMPaltitude);
  Serial.print(" m, Pressure=");
  Serial.print(pressure);
  Serial.print(" Pa, BatteryVoltage:");
  Serial.println(batteryVoltage);
  */
}


void setup() {
	//Serial.begin(115200);
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
