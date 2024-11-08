/*
 * HelTec Automation(TM) WIFI_LoRa_32 factory test code, witch includ
 * follow functions:
 * 
 * - Basic OLED function test;
 * 
 * - Basic serial port test(in baud rate 115200);
 * 
 * - LED blink test;
 * 
 * - WIFI connect and scan test;
 * 
 * - LoRa Ping-Pong test (DIO0 -- GPIO26 interrup check the new incoming messages);
 * 
 * - Timer test and some other Arduino basic functions.
 *
 * by Aaron.Lee from HelTec AutoMation, ChengDu, China
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
*/

#include "Arduino.h"
#include "WiFi.h"
//#include "images.h"
#include "LoRaWan_APP.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"
// #include "board-config.h"
/********************************* lora  *********************************************/
#define RF_FREQUENCY                                922000000 // Hz
 
#define TX_OUTPUT_POWER                             0        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum {
    LOWPOWER,
    STATE_RX,
    STATE_TX
}

States_t;
int16_t txNumber;
int16_t rxNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

String rssi = "RSSI --";
String packSize = "--";
String packet;
String send_num;
String show_lora = "lora data show";


unsigned int counter = 0;

bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.

long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
uint64_t chipid;
int16_t RssiDetection = 0;

unsigned int sleep_duration = 10; // sleep duration in seconds

void OnTxDone(void) {
  	Serial.print("TX done......");
	state=STATE_RX;
}

void OnTxTimeout(void) {
    Radio.Sleep(); // 
    Serial.print("TX Timeout......");
	state=STATE_TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
	rxNumber++;
    Rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep();

    Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
    Serial.println("wait to send next packet");
	receiveflag = true;
    state=STATE_TX;
}


void lora_init(void) {
	//Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    txNumber=0;
    Rssi=0;
	rxNumber = 0;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(
		MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
		LORA_SPREADING_FACTOR, LORA_CODINGRATE,
		LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
		true, 0, 0, LORA_IQ_INVERSION_ON, 3000
		);

    Radio.SetRxConfig(
		MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
		LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
		LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
		0, true, 0, 0, LORA_IQ_INVERSION_ON, true
		);
	state=STATE_TX;
}

void lora_send(void) {
	txNumber++;
	sprintf(txpacket,"hello %d, Rssi : %d",txNumber,Rssi);
	Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
	Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
}


/********************************* lora  *********************************************/
SSD1306Wire  oled_display(
	0x3c, 			// addr
	500000, 		// frequency
	SDA_OLED, 		// i2c
	SCL_OLED, 		// i2c
	GEOMETRY_64_32, // resolution
	RST_OLED		// rst
);

void WIFISetUp(void) {
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(100);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);
	WiFi.begin("GeekSloth","heltec_test");	//fill in "Your WiFi SSID","Your Password"
	delay(100);
	byte count = 0;
	while(WiFi.status() != WL_CONNECTED && count < 10) {
		count ++;
		delay(500);
		oled_display.drawString(0, 0, "Connecting...");
		oled_display.display();
	}

	oled_display.clear();
	if(WiFi.status() == WL_CONNECTED) {
		oled_display.drawString(0, 8, "OK.");
		oled_display.display();
	} else{
		oled_display.clear();
		oled_display.drawString(0, 8, "Failed");
		oled_display.display();
	}
	oled_display.drawString(0, 16, "WIFI Setup done");
	oled_display.display();
	delay(500);
}

void WIFIScan(unsigned int value) {
	unsigned int i;
    WiFi.mode(WIFI_STA);
	for(i=0;i<value;i++) {
		oled_display.drawString(0, 0, "Scan start...");
		oled_display.display();
		int n = WiFi.scanNetworks();
		oled_display.drawString(0, 30, "Scan done");
		oled_display.display();
		delay(500);
		oled_display.clear();

		if (n == 0)	{
			oled_display.clear();
			oled_display.drawString(0, 0, "no network found");
			oled_display.display();
		} else {
			oled_display.drawString(0, 0, (String)n);
			oled_display.drawString(14, 0, "networks");
            oled_display.drawString(0, 6, "found");
			oled_display.display();
			delay(500);
		}
		oled_display.display();
		delay(800);
		oled_display.clear();
	}
}

bool resend_flag=false;
bool deepsleep_flag=false;
bool interrupt_flag = false;

void interrupt_GPIO0() {
	interrupt_flag = true;
}

void interrupt_handle(void) {
	if(interrupt_flag) {
		interrupt_flag = false;
		if(digitalRead(0)==0) { 
			if(rxNumber <=2) {
				resend_flag=true;
			} else {
				// resend_flag=true;
				deepsleep_flag=true; // use this flag if want to use deepsleep mode
			}
		}
	}

}

void VextON(void) {
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  
}

void VextOFF(void) { //Vext default OFF
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void setup() {
    Serial.begin(115200);
    VextON();
    delay(100);
    lora_init();
    chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

	oled_display.init();
    oled_display.clear();
    oled_display.display();

    oled_display.drawString(0, 0, "Test Start");
	oled_display.display();
    oled_display.clear();
	//delay(500);

	WIFISetUp();
    oled_display.clear();
	WiFi.disconnect(); //
	WiFi.mode(WIFI_STA);
	delay(500);

	WIFIScan(1);

    attachInterrupt(0,interrupt_GPIO0,FALLING);

	packet = (String)RF_FREQUENCY + "Hz";
    oled_display.drawString(0, 0, packet);
    oled_display.display();
    delay(100);
    oled_display.clear();
	pinMode(LED ,OUTPUT);
	digitalWrite(LED, LOW);  
}


void loop() {
	interrupt_handle();
	if(deepsleep_flag) {
		VextOFF();
		Radio.Sleep();

		/*
		* not actually know why these pins are set to ANALOG mode, but it works.
		* may try to remove these lines and see if it still works later.
		*/
		pinMode(RADIO_DIO_1, ANALOG);
		pinMode(RADIO_NSS, ANALOG);
		pinMode(RADIO_RESET, ANALOG);
		pinMode(RADIO_BUSY, ANALOG);
		pinMode(LORA_CLK, ANALOG);
		pinMode(LORA_MISO, ANALOG);
		pinMode(LORA_MOSI, ANALOG);
		pinMode(LED, ANALOG);
		pinMode(Vext, ANALOG);


		detachInterrupt(0);
		Wire.end();
		//Serial.end(true);
		SPI.end();
		WiFi.mode(WIFI_OFF);
		esp_sleep_enable_timer_wakeup(sleep_duration * 1000 * (uint64_t)1000);
		esp_deep_sleep_start();
	}

	if(resend_flag) {
		state = STATE_TX; 
		resend_flag = false;
	}

	if(receiveflag && (state==LOWPOWER)) {
		receiveflag = false;
		packet ="R:";
		int i = 0;
		while(i < rxSize) {
			packet += rxpacket[i];
			i++;
		}
		packSize = "Rssi: ";
		packSize += String(Rssi, DEC);
		send_num = "S: ";
		send_num += String(txNumber, DEC);
		oled_display.drawString(0, 0, "LoRa");
		oled_display.drawString(0, 10, packet);
		oled_display.drawString(0, 20, packSize);
		oled_display.drawString(32, 0, send_num);
		oled_display.display();
		delay(10);
		oled_display.clear();

		if((rxNumber%2)==0) {
			digitalWrite(LED, HIGH);  
		} else {
			digitalWrite(LED, LOW);
		}

	}

	switch(state) {
		case STATE_TX:
			delay(1000);
			txNumber++;
			sprintf(txpacket, "hello %d,Rssi:%d", txNumber, Rssi);
			Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));
			Radio.Send((uint8_t *)txpacket, strlen(txpacket));
			state=LOWPOWER;
			break;
		case STATE_RX:
			Serial.println("into RX mode");
			Radio.Rx(0);
			state=LOWPOWER;
			break;
		case LOWPOWER:
			Radio.IrqProcess();
			break;
		default:
			break;
	}
}
