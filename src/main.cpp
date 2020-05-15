/**
 * @author: Fran Acién and David Arias, with the help of Pablo Alvarez and Dani Kholer
 *  SSTV emitter using arduino DUE
 *
 *
 *  Note: I am using millis() instead of delay because it has more accurate
**/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <AD9850.h>
#include <JPEGDecoder.h>
#include <Adafruit_VC0706.h>
#include <DueTimer.h>
#include <Adafruit_GPS.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <LPS.h>
#include <math.h>
#include <stdio.h>

// Scottie 1 properties
#define COLORCORRECTION 3.1372549

// Scottie 1 mode
#define COLORSCANTIMEPERLINE 138.240        //ms
#define COLORSCANPERPIXEL 432               //microseconds
#define SEPARATORPULSETIME 1.5              //ms
#define SEPARATORPULSEFREQ 1500             //ms
#define SYNCPULSETIME 9                     //ms
#define SYNCPULSEFREQ 1200                  //Hz

// AD9850 consts
#define AD9850_CLK_PIN 51         //Working clock output pin
#define AD9850_FQ_UPDATE_PIN 49   //Frequency update
#define AD9850_DATA_PIN 47        //Serial data output pin
#define AD9850_RST_PIN 45         //Reset output pin

// Sd consts
#define SD_SLAVE_PIN 53
#define SD_CLOCK_PIN 13
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 12

// APRS PWM PIN OUTPUT
#define APRS_PWM_OUT 2

// APRS BAUD RATE FLAGS
#define _1200   1
#define _2400   0

// APRS COMMONT CONSTANTS
#define _FLAG       0x7e
#define _CTRL_ID    0x03
#define _PID        0xf0
#define _DT_EXP     ','
#define _DT_STATUS  '>'
#define _DT_POS     '!'
#define _DT_TEL     'T'

// APRS FLAGS
#define _FIXPOS         1
#define _FIXPOS_STATUS  2
#define _STATUS         3
#define _BEACON         4
#define _TELEMETRY      5

// GPS SERIAL
#define GPSSerial Serial3

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Other stuff
#define LED_BUILTIN 13

// Structures
typedef struct {
  float pressure;
  float altitude;
  float temperature;
} Bar;

// Imu template
template <typename T> struct vector{
  T x, y, z;
};

typedef struct {
  bool fix;
  float lat;
  float lon;
  char latd;
  char lond;
  char lati[9];     // ddmm.ssN/ or ddmm.ssS/   pg 24 APRS PROTOCOL
  char loni[10];  //dddmm.ssE- or dddmm.ssW-
} Gps;

typedef struct{
  char dttype;
  char seqN[5];
  char an_1[4];
  char an_2[4];
  char an_3[4];
  char an_4[4];
  char an_5[4];
  char dig_val[8];
  char comment[128];
} Telpct;

volatile uint8_t phase = 0;

char pic_filename[13];
char pic_decoded_filename[13];

uint8_t frameBuf[81920]; //320*256

volatile byte buffE[320]; // Buffer conintating Red values after torch
volatile byte buffR[320]; // Buffer conintating Red values of the line
volatile byte buffG[320]; // Buffer conintating Green values of the line
volatile byte buffB[320]; // Buffer conintating Blue values of the line

volatile byte sEm = 0;    // State of Emition
                    // 0 not emitting
                    // 1 emitting line (NOT HEADER OR VOX)
                    // 2 Change Color

volatile byte sCol = 0;   // Transmitting color Green
                    // Transmitting color Blue
                    // Transmitting color Red

volatile int tp = 0;     // Index of pixel while transmitting with timer
volatile int line;

// Camera stuff
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

// Adafruit GPS
Adafruit_GPS GPS(&GPSSerial);
Gps gps;

// Magnetometer LIS3MDL
LIS3MDL mag;
vector<int16_t> m; // magnetometer readings

// Accelerometer LSM6DS33
LSM6 imu;
vector<int16_t> a; // accelerometer readings
vector<int16_t> g; // gyro readings

// Barometer LPS25H
LPS ps;
Bar bar;

uint32_t timer = millis();

// AFSK Vars
bool nada = _2400;

/*
 * SQUARE WAVE SIGNAL GENERATION
 *
 * baud_adj lets you to adjust or fine tune overall baud rate
 * by simultaneously adjust the 1200 Hz and 2400 Hz tone,
 * so that both tone would scales synchronously.
 * adj_1200 determined the 1200 hz tone adjustment.
 * tc1200 is the half of the 1200 Hz signal periods.
 *
 *      -------------------------                           -------
 *     |                         |                         |
 *     |                         |                         |
 *     |                         |                         |
 * ----                           -------------------------
 *
 *     |<------ tc1200 --------->|<------ tc1200 --------->|
 *
 * adj_2400 determined the 2400 hz tone adjustment.
 * tc2400 is the half of the 2400 Hz signal periods.
 *
 *      ------------              ------------              -------
 *     |            |            |            |            |
 *     |            |            |            |            |
 *     |            |            |            |            |
 * ----              ------------              ------------
 *
 *     |<--tc2400-->|<--tc2400-->|<--tc2400-->|<--tc2400-->|
 *
 */
const float baud_adj = 0.975; //0.975;
const float adj_1200 = 1.0 * baud_adj;
const float adj_2400 = 1.0 * baud_adj;
unsigned int tc1200 = (unsigned int)(0.5 * adj_1200 * 1000000.0 / 1200.0);
unsigned int tc2400 = (unsigned int)(0.5 * adj_2400 * 1000000.0 / 2400.0);

/*
 * This strings will be used to generate AFSK signals, over and over again.
 */
char mycall[8] = "EA4RCT";
char myssid = 11;

char dest[8] = "APZ";     // Experimental see http://aprs.org/aprs11/tocalls.txt
char dest_beacon[8] = "BEACON";

char digi[8] = "WIDE2";
char digissid = 2;

char comment[128] = "RadioClub EIT";
char mystatus[128] = "..::| Experimental Arduino-APRS |::..";

const char sym_ovl = 'T';
const char sym_tab = 'a';
Telpct telpct;
char heightpckt[8];

uint16_t tel_seq = 0;   // Telemetry seq number

unsigned int tx_delay = 30000;
unsigned int str_len = 400;

char bit_stuff = 0;
unsigned short crc=0xffff;

char charId[13] = "EA4RCT-SSTV-"; // ***** INFORMATION HEADER: MAX 12 CAHARCTERS *****
volatile long syncTime;

//FONTS
const uint8_t b_fonts[43][11] = {
        {0x00, 0x18, 0x24, 0x62, 0x62, 0x62, 0x7E, 0x62, 0x62, 0x62, 0x00}, //00: A
        {0x00, 0x7C, 0x32, 0x32, 0x32, 0x3C, 0x32, 0x32, 0x32, 0x7C, 0x00}, //01: B
        {0x00, 0x3C, 0x62, 0x62, 0x60, 0x60, 0x60, 0x62, 0x62, 0x3C, 0x00}, //02: C
        {0x00, 0x7C, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x7C, 0x00}, //03: D
        {0x00, 0x7E, 0x60, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x7E, 0x00}, //04: E
        {0x00, 0x7E, 0x60, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x60, 0x00}, //05: F
        {0x00, 0x3C, 0x62, 0x62, 0x60, 0x60, 0x66, 0x62, 0x62, 0x3C, 0x00}, //06: G
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x00}, //07: H
        {0x00, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00}, //08: I
        {0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x4C, 0x4C, 0x4C, 0x38, 0x00}, //09: J
        {0x00, 0x62, 0x64, 0x68, 0x70, 0x68, 0x64, 0x62, 0x62, 0x62, 0x00}, //10: K
        {0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00}, //11: L
        {0x00, 0x42, 0x62, 0x76, 0x6A, 0x62, 0x62, 0x62, 0x62, 0x62, 0x00}, //12: M
        {0x00, 0x42, 0x62, 0x72, 0x6A, 0x66, 0x62, 0x62, 0x62, 0x62, 0x00}, //13: N
        {0x00, 0x3C, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x3C, 0x00}, //14: O
        {0x00, 0x7C, 0x62, 0x62, 0x62, 0x7C, 0x60, 0x60, 0x60, 0x60, 0x00}, //15: P
        {0x00, 0x3C, 0x62, 0x62, 0x62, 0x62, 0x62, 0x6A, 0x6A, 0x3C, 0x08}, //16: Q
        {0x00, 0x7C, 0x62, 0x62, 0x62, 0x7C, 0x68, 0x64, 0x62, 0x62, 0x00}, //17: R
        {0x00, 0x3C, 0x62, 0x60, 0x60, 0x3C, 0x06, 0x06, 0x46, 0x3C, 0x00}, //18: S
        {0x00, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, //19: T
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x3C, 0x00}, //20: U
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x22, 0x14, 0x08, 0x00}, //21: V
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x62, 0x6A, 0x76, 0x62, 0x42, 0x00}, //22: W
        {0x00, 0x42, 0x62, 0x74, 0x38, 0x1C, 0x2E, 0x46, 0x42, 0x42, 0x00}, //23: X
        {0x00, 0x42, 0x62, 0x74, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, //24: Y
        {0x00, 0x7E, 0x06, 0x0E, 0x0C, 0x18, 0x30, 0x70, 0x60, 0x7E, 0x00}, //25: Z
        {0x00, 0x3C, 0x62, 0x62, 0x66, 0x6A, 0x72, 0x62, 0x62, 0x3C, 0x00}, //26: 0
        {0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, //27: 1
        {0x00, 0x3C, 0x46, 0x06, 0x06, 0x1C, 0x20, 0x60, 0x60, 0x7E, 0x00}, //28: 2
        {0x00, 0x3C, 0x46, 0x06, 0x06, 0x1C, 0x06, 0x06, 0x46, 0x3C, 0x00}, //29: 3
        {0x00, 0x0C, 0x1C, 0x2C, 0x4C, 0x4C, 0x7E, 0x0C, 0x0C, 0x0C, 0x00}, //30: 4
        {0x00, 0x7E, 0x60, 0x60, 0x60, 0x7C, 0x06, 0x06, 0x46, 0x3C, 0x00}, //31: 5
        {0x00, 0x3C, 0x62, 0x60, 0x60, 0x7C, 0x62, 0x62, 0x62, 0x3C, 0x00}, //32: 6
        {0x00, 0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00}, //33: 7
        {0x00, 0x3C, 0x62, 0x62, 0x62, 0x3C, 0x62, 0x62, 0x62, 0x3C, 0x00}, //34: 8
        {0x00, 0x3C, 0x46, 0x46, 0x46, 0x3E, 0x06, 0x06, 0x46, 0x3C, 0x00}, //35: 9
        {0x00, 0x00, 0x02, 0x06, 0x0E, 0x1C, 0x38, 0x70, 0x60, 0x40, 0x00}, //36: /
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x00, 0x00}, //37: -
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00}, //38: .
        {0x00, 0x3C, 0x46, 0x06, 0x06, 0x0C, 0x10, 0x00, 0x30, 0x30, 0x00}, //39: ?
        {0x00, 0x18, 0x18, 0x18, 0x18, 0x10, 0x10, 0x00, 0x18, 0x18, 0x00}, //40: !
        {0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00}, //41: :
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  //42: space
};

// Nibble font table
const uint8_t l_fonts[23][5] = {
  { 0xE2, 0xA6, 0xA2, 0xA2, 0xE2 }, // 0: 01
  { 0xEE, 0x22, 0xE6, 0x82, 0xEE }, // 1: 23
  { 0xAE, 0xA8, 0xEE, 0x22, 0x2E }, // 2: 45
  { 0x8E, 0x82, 0xE2, 0xA2, 0xE2 }, // 3: 67
  { 0xEE, 0xAA, 0xEE, 0xA2, 0xE2 }, // 4: 89
  { 0x00, 0x22, 0x00, 0x22, 0x04 }, // 5: :;
  { 0x20, 0x4E, 0x80, 0x4E, 0x20 }, // 6: <=
  { 0x8E, 0x42, 0x26, 0x40, 0x84 }, // 7: >?
  { 0x64, 0x9A, 0xBE, 0x8A, 0x7A }, // 8: @A
  { 0xC6, 0xA8, 0xC8, 0xA8, 0xC6 }, // 9: BC
  { 0xCE, 0xA8, 0xAC, 0xA8, 0xCE }, // 10: DE
  { 0xE6, 0x88, 0xCE, 0x8A, 0x86 }, // 11: FG
  { 0xA4, 0xA4, 0xE4, 0xA4, 0xA4 }, // 12: HI
  { 0x69, 0x2A, 0x2C, 0x2A, 0x49 }, // 13: JK
  { 0x8A, 0x8E, 0x8E, 0x8A, 0xEA }, // 14: LM
  { 0x04, 0x9A, 0xDA, 0xBA, 0x94 }, // 15: NO
  { 0xC4, 0xAA, 0xCA, 0x8E, 0x86 }, // 16: PQ
  { 0xC6, 0xA8, 0xC4, 0xA2, 0xAC }, // 17: RS
  { 0xE0, 0x4A, 0x4A, 0x4A, 0x44 }, // 18: TU
  { 0x09, 0xA9, 0xA9, 0x6F, 0x26 }, // 19: vW (sort of..)
  { 0x0A, 0xAA, 0x46, 0xA2, 0x04 }, // 20: XY
  { 0xE6, 0x24, 0x44, 0x84, 0xE6 }, // 21: Z[
  { 0x00, 0x00, 0x00, 0x00, 0x00 }  // 22: SPACE
};

// SSTV Prototypes
uint16_t playPixel(long pixel);
uint16_t scottie_freq(uint8_t c);
void vox_tone();
void scottie1_calibrationHeader();
void transmit_micro(int freq, float duration);
void transmit_mili(int freq, float duration);
void scottie1_transmit_file(char* filename);
void shot_pic();
void jpeg_decode(char* filename, char* fileout);
void writeHeader(File* dst);

// APRS prototypes
void read_mad(vector<int16_t>* m);
void read_imu(vector<int16_t>* a, vector<int16_t>* g);
void read_imu(vector<int16_t>* a);
void read_bar(Bar* res);
void read_gps(Gps* res);
void read_gps();
void read_coords(Gps* res);
void print_sensors(void);  // Print values of mad imu and bar

void set_nada_1200(void);
void set_nada_2400(void);
void set_nada(bool nada);

void send_char_NRZI(unsigned char in_byte, bool enBitStuff);
void send_string_len(const char *in_string, int len);

void calc_crc(bool in_bit);
void send_crc(void);
void randomize(unsigned int &var, unsigned int low, unsigned int high);

void send_packet(char packet_type);
void send_flag(unsigned char flag_len);
void send_header(char msg_type);
void send_payload(char type);

void print_code_version(void);
void print_debug(char type);

void timer1_interrupt(){
  if (sEm == 1){
    if(tp < 320){  // Transmitting pixels
      if(sCol == 0){  // Transmitting color Green
        DDS.setfreq(1500 + 3.13 * buffG[tp], phase);
      } else if(sCol == 1){ // Transmitting color Blue
        DDS.setfreq(1500 + 3.13 * buffB[tp], phase);
      } else if(sCol == 2){ // Transmitting color Red
        DDS.setfreq(1500 + 3.13 * buffE[tp], phase);
      }
    } else if(tp == 320){
      if(sCol == 0){  // Separator pulse after transmit Green
        DDS.setfreq(1500, phase);
      } else if(sCol == 1){ // Sync porch
        DDS.setfreq(1200, phase);
      } else if(sCol == 2){ // // Separator pulse after transmit Red
        DDS.setfreq(1500, phase);
      }
      syncTime = micros();
      sEm = 2;    // State when change color
    }
    tp++;
  }
}

void setup() {
  delay(5000);

  // Set pins as output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(APRS_PWM_OUT, OUTPUT);
  pinMode(SD_SLAVE_PIN, OUTPUT);

  Serial.begin(115200);
  print_code_version();
  delay(250);

  Wire.begin();   // i2C begin

  Serial.println("Initializating IMU...");
  // Start magnetometer
  if (!mag.init()){
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();

  // Start accelerometer
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  // Start barometer
  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();

  // AD9850 initilize
  DDS.begin(AD9850_CLK_PIN, AD9850_FQ_UPDATE_PIN, AD9850_DATA_PIN, AD9850_RST_PIN);

  // Sd initialize
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_SLAVE_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // Setup Timer with the emision interval
  Timer1.attachInterrupt(timer1_interrupt).setPeriod(430); // ***** 354(uS/px) +/- SLANT ADJUST *****
  delay(100);

  Serial.flush();
  Serial.end();
}

void loop() {
  // First emit APRS packet with telemetry and, if GPS is enable, also with GPS data.
  // Then it sends SSTV picture and wait tx_delay

  read_imu(&a, &g);
  read_mad(&m);
  read_bar(&bar);
  read_gps(&gps);

  // Build telemetry packet
  telpct.dttype = _DT_TEL;
  snprintf(telpct.seqN, sizeof(telpct.seqN), "#%03d,", tel_seq);
  snprintf(telpct.an_1, sizeof(telpct.an_1), "%03d,", (int) abs(bar.altitude));
  snprintf(telpct.an_2, sizeof(telpct.an_2), "%03d,", (int) abs(bar.temperature));
  snprintf(telpct.an_3, sizeof(telpct.an_3), "%03d,", (int) abs(a.x));
  snprintf(telpct.an_4, sizeof(telpct.an_4), "%03d,", (int) abs(a.y));
  snprintf(telpct.an_5, sizeof(telpct.an_5), "%03d,", (int) abs(a.z));
  snprintf(telpct.dig_val, sizeof(telpct.dig_val), "%08d", 0); // No digital data
  snprintf(telpct.comment, sizeof(telpct.comment), "%s", comment);
  tel_seq++;
  if(tel_seq == 1000) tel_seq = 0; // Reset telemetry pckt seq number

  if(gps.fix){
    send_packet(_FIXPOS);
    send_packet(_TELEMETRY);
  } else {
    send_packet(_TELEMETRY);
  }

  // Start SSTV interrupt timer
  Timer1.start();
  // Take picture
  shot_pic();
  // Decode picture
  jpeg_decode(pic_filename, pic_decoded_filename);
  // Send decoded picture with SSTV using synthesizer
  scottie1_transmit_file(pic_decoded_filename);
  // Quit SSTV Timer1
  Timer1.stop();

  delay(tx_delay);
  randomize(tx_delay, 5000, 9000);
}

/**
 * Get output frequency given a color component (R, G, B) from 0 to 255
 * @param uint8_t c - single color component from 0 to 255
 * @return uint16_t - scottie1 frequency
**/
uint16_t scottie_freq(uint8_t c){
  return 1500 + (c * COLORCORRECTION);
}

void vox_tone(){
  /** VOX TONE (OPTIONAL) **/
  transmit_mili(1900, 100);
  transmit_mili(1500, 100);
  transmit_mili(1900, 100);
  transmit_mili(1500, 100);
  transmit_mili(2300, 100);
  transmit_mili(1500, 100);
  transmit_mili(2300, 100);
  transmit_mili(1500, 100);
}

void scottie1_calibrationHeader(){
  /** CALIBRATION HEADER **/
  transmit_mili(1900, 300);
  transmit_mili(1200, 10);
  transmit_mili(1900, 300);
  transmit_mili(1200, 30);
  transmit_mili(1300, 30);    // 0
  transmit_mili(1300, 30);    // 0
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1300, 30);    // 0
  transmit_mili(1300, 30);    // Even parity
  transmit_mili(1200, 30);    // VIS stop bit
}

/**
 * Set a frequency for a certain amount of time
 * @param int freq - Frequency
 * @param float duration - duration in microseconds
 */
void transmit_micro(int freq, float duration){
  DDS.setfreq(freq, phase);
  delayMicroseconds(duration);
}

/**
 * Set a frequency for a certain amount of time
 * @param int freq - Frequency
 * @param float duration - duration in milliseconds
 */
void transmit_mili(int freq, float duration){
  DDS.setfreq(freq, phase);
  delay(duration);
}

void scottie1_transmit_file(char* filename){
  /*
  Be aware that you have to read variables on sync torch due its 9 ms instead 1.5 ms of the sync Pulse
  */

  bool head;
  Serial.println("Transmitting picture");

  File myFile = SD.open(filename);
  if (myFile) {
    head = true;

    /** TRANSMIT EACH LINE **/
    while(myFile.available() || line == 255){
      if(head == true){ // Header
        /** VOX TONE (OPTIONAL) **/
        vox_tone();

        /** CALIBRATION HEADER **/
        scottie1_calibrationHeader();

        // Configure syncTime
        syncTime = micros();

        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
          buffR[i] = myFile.read();
          buffG[i] = myFile.read();
          buffB[i] = myFile.read();
        }

        //Serial.println("++");
        //Serial.println(micros() - syncTime); //Cheak reading time

        while(micros() - syncTime < 9000 - 10){}

        // Separator pulse
        DDS.setfreq(1500, phase);
        syncTime = micros();  // Configure syncTime

        line = 0;
        head = false;
      }

      while(micros() - syncTime < 1500 - 10){} // Separator pulse

      // Green Scan
      tp = 0; sCol = 0; sEm = 1;
      while(sEm == 1){};

      // Separator Pulse
      DDS.setfreq(1500, phase); //USELESS
      while(micros() - syncTime < 1500 - 10){}

      // Blue Scan
      tp = 0; sCol = 1; sEm = 1;
      while(sEm == 1){};

      //Evacuate
      for(uint16_t i = 0; i < 320; i++){
        buffE[i] = buffR[i];
      }

      if(line != 255){
        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
          buffR[i] = myFile.read();
          buffG[i] = myFile.read();
          buffB[i] = myFile.read();
        }
      }

      //Serial.println("--");
      //Serial.println(micros() - syncTime); //Cheak reading time

      //Sync pulse
      while(micros() - syncTime < 9000 - 10){}

      // Sync porch
      DDS.setfreq(1500, phase);
      syncTime = micros();
      while(micros() - syncTime < 1500 - 10){}

      // Red Scan
      tp = 0; sCol = 2; sEm = 1;
      while(sEm == 1){};

      line++;
      if(line == 256){
        Serial.println("Finish");
        DDS.setfreq(2, phase);
        DDS.down();
        sEm = 0;
      }
      else {
        // Separator pulse
        DDS.setfreq(1500, phase); //USELESS
        syncTime = micros(); //USELESS
        sEm = 2;
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void jpeg_decode(char* filename, char* fileout){
  uint8 *pImg;
  int x,y,bx,by;
  byte sortBuf[15360]; //320(px)*16(lines)*3(bytes) // Header buffer
  int i,j,k;
  int pxSkip;

  Serial.begin(115200);

  Serial.println("Creating decoded picture file...");

  strcpy(pic_decoded_filename, pic_filename);
  pic_decoded_filename[8] = 'B';
  pic_decoded_filename[9] = 'I';
  pic_decoded_filename[10] = 'N';

  Serial.print("Writting on: ");
  Serial.println(pic_decoded_filename);

  // Open the file for writing
  File imgFile = SD.open(fileout, FILE_WRITE);

  for(i = 0; i < 15360; i++){ // Cleaning Header Buffer array
    sortBuf[i] = 0xFF;
  }

  for(i = 0; i < 12; i++){
    byte fontNumber;
    char ch;
    ch = charId[i];
    for(y = 0; y < 11; y++){
      for(x = 0; x < 8; x++){
        pxSkip = 16 + (320 * (y + 3)) + (3 * 8 * i) + (3 * x); //Width: x3

        uint8_t mask;
        mask = pow(2, 7 - x);

        if(ch >= 65 && ch <= 90){ // A to Z
                fontNumber = ch - 65;
        }
        else if(ch >= 48 && ch <= 57){ //0 to 9
                fontNumber = ch - 22;
        }
        else if(ch == '/'){fontNumber = 36;}
        else if(ch == '-'){fontNumber = 37;}
        else if(ch == '.'){fontNumber = 38;}
        else if(ch == '?'){fontNumber = 39;}
        else if(ch == '!'){fontNumber = 40;}
        else if(ch == ':'){fontNumber = 41;}
        else if(ch == ' '){fontNumber = 42;}
        else              {fontNumber = 42;}

        if((b_fonts[fontNumber][y] & mask) != 0){
          for(j = 0; j < 9; j++){
                  sortBuf[(3 * pxSkip) + j] = 0x00;
          }
        }
      }
    }
  }

  for(k = 0; k < 15360; k++){  // Adding header to the binary file
    imgFile.write(sortBuf[k]);
  }

  writeHeader(&imgFile);  //Writing first 10560 bytes (11*320*3)

  // Decoding start
  JpegDec.decode(filename,0);
  // Image Information
  Serial.print("Width     :");
  Serial.println(JpegDec.width);
  Serial.print("Height    :");
  Serial.println(JpegDec.height);
  Serial.print("Components:");
  Serial.println(JpegDec.comps);
  Serial.print("MCU / row :");
  Serial.println(JpegDec.MCUSPerRow);
  Serial.print("MCU / col :");
  Serial.println(JpegDec.MCUSPerCol);
  Serial.print("Scan type :");
  Serial.println(JpegDec.scanType);
  Serial.print("MCU width :");
  Serial.println(JpegDec.MCUWidth);
  Serial.print("MCU height:");
  Serial.println(JpegDec.MCUHeight);
  Serial.println("");

  Serial.println("Writting bin to SD");

  i = 0;
  j = 0;
  while(JpegDec.read()){
      pImg = JpegDec.pImage ;
      for(by=0; by<JpegDec.MCUHeight; by++){
          for(bx=0; bx<JpegDec.MCUWidth; bx++){
              x = JpegDec.MCUx * JpegDec.MCUWidth + bx;
              y = JpegDec.MCUy * JpegDec.MCUHeight + by;
              if(x<JpegDec.width && y<JpegDec.height){
                  if(JpegDec.comps == 1){ // Grayscale
                      //sprintf(str,"%u", pImg[0]);
                      imgFile.write(pImg, 1);
                  }else{ // RGB
                      //sprintf(str,"%u%u%u", pImg[0], pImg[1], pImg[2]);
                      //imgFile.write(pImg, 3);

                      // First we write on array and then save it
                      // Read 16 lines  and the write it successively
                      pxSkip = ((y - (16 * j)) * 320) + x;
                      sortBuf[(3 * pxSkip) + 0] = pImg[0];
                      sortBuf[(3 * pxSkip) + 1] = pImg[1];
                      sortBuf[(3 * pxSkip) + 2] = pImg[2];

                      i++;
                      if(i == 5120){ //320(px)x16(lines)
                        for(k = 0; k < 15360; k++){
                          imgFile.write(sortBuf[k]);
                        }
                        i = 0;
                        j++; //15(sections)
                      }
                  }
              }
              pImg += JpegDec.comps ;
          }
      }
  }

  Serial.println("Bin has been written on SD");
  imgFile.close();

  Serial.println(' ');

  Serial.flush();
  Serial.end();
}

void shot_pic(){
  Serial.begin(115200);

  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }

  for (int i = 0; i <= 10; i++){
    cam.setImageSize(VC0706_320x240);
  }

  Serial.println("Snap in 3 secs...");
  delay(3000);

  if (! cam.takePicture())
    Serial.println("Failed to snap!");
  else
    Serial.println("Picture taken!");

  // Create an image with the name IMAGExx.JPG`
  strcpy(pic_filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    pic_filename[5] = '0' + i/10;
    pic_filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(pic_filename)) {
      break;
    }
  }

  // Open the file for writing
  File imgFile = SD.open(pic_filename, FILE_WRITE);

  // Get the size of the image (frame) taken
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");

  Serial.print("Picture taken saved on:");
  Serial.println(pic_filename);

  Serial.println(' ');

  Serial.flush();
  Serial.end();
}

void writeHeader(File* dst){
  int x,y;
  byte sortBuf[10560]; //320(px)*11(lines)*3(bytes) // Header buffer
  int i,j,k;
  int pxSkip;

  char res[51] = "LAT: 1234.1234N     LONG: 1234.1234W     ALT:10000";

  for(i = 0; i < 10560; i++){ // Cleaning Header Buffer array
    sortBuf[i] = 0xFF;
  }

  for(i = 0; i < sizeof(res); i++){
    byte fontNumber;
    char ch;
    ch = res[i];
    for(y = 0; y < 5; y++){
      for(x = 0; x < 4; x++){
        //pxSkip = HORIZONTALOFFSET + VERSTICALOFFSET + (BITSPERWORD * i);
        //pxSkip = 16 + (320 * (y + 3)) + (4 * 2 * i) + (2 * x); Width: x2
        pxSkip = 16 + (320 * (y + 3)) + (4 * i) + x;

        // If ch is pair mask is: 11110000, if no 00001111
        uint8_t sl = (ch % 2)? 3 : 7 ;
        uint8_t mask = pow(2, sl - x);

        if(ch >= 48 && ch <=91){
          fontNumber = (ch-48)/2;
        }
        else {
          fontNumber = 22;
        }

        if((l_fonts[fontNumber][y] & mask) != 0){
          for(j = 0; j < 3; j++){
                  sortBuf[(3 * pxSkip) + j] = 0x00;
          }
        }
      }
    }
  }

  for(k = 0; k < 10560; k++){  // Adding header to the binary file
    dst->write(sortBuf[k]);
  }
}

void read_gps(Gps* res){
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  boolean received = false;
  do{
    if(GPS.available()>0){
      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
      // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        if(GPS.parse(GPS.lastNMEA())){
          read_coords(res);
          received = true;
        } else{
          continue;
        }
      }
    }
  }while(!received);

  GPSSerial.flush();
  GPSSerial.end();
}

void read_coords(Gps* res){
  res->fix = GPS.fix;
  if (GPS.fix) {
    res->lat = GPS.latitudeDegrees;
    res->lon = GPS.longitudeDegrees;
    res->latd = GPS.lat;
    res->lond = GPS.lon;
    snprintf(res->lati, sizeof(res->lati), "%07.2f%c", GPS.latitude, GPS.lat);
    snprintf(res->loni, sizeof(res->loni), "%08.2f%c", GPS.latitude, GPS.lon);
  }
}


void read_mad(vector<int16_t>* m){
  mag.read();
  memcpy(m, &mag.m, sizeof(vector<int16_t>));
}

void read_imu(vector<int16_t>* a, vector<int16_t>* g){
  imu.read();
  memcpy(a, &imu.a, sizeof(vector<int16_t>));
  memcpy(g, &imu.g, sizeof(vector<int16_t>));
}

void read_imu(vector<int16_t>* a){
  imu.read();
  memcpy(a, &imu.a, sizeof(vector<int16_t>));
}

void read_bar(Bar* res){
  res->pressure = ps.readPressureMillibars();
  res->altitude = ps.pressureToAltitudeMeters(res->pressure);
  res->temperature = ps.readTemperatureC();

  // Put height into char[] for send it with aprs packet
  snprintf(heightpckt, sizeof(heightpckt), "%07d", (int) res->altitude);
}

void print_sensors(void){
  Serial.begin(115200);
  char buff[80];
  delay(100);

  Serial.println("IMU Values");
  snprintf(buff, sizeof(buff), "A: %6d %6d %6d    G: %6d %6d %6d",
          a.x, a.y, a.z, g.x, g.y, g.z);
  Serial.println(buff);
  Serial.println("--------------");

  Serial.println("MAG Values");
  snprintf(buff, sizeof(buff), "M: %6d %6d %6d",
          m.x, m.y, m.z);
  Serial.println(buff);
  Serial.println("----------------");

  Serial.println("BAR values");
  snprintf(buff, sizeof(buff), "Pressure %f mbar     Altitude: %f m     Temperature: %f deg C",
          bar.pressure, bar.altitude, bar.temperature);
  Serial.println(buff);
  Serial.println("----------------");

  if(gps.fix){
    snprintf(buff, sizeof(buff), "FIX: %d  LAT: %s     LON: %s", gps.fix, gps.lati, gps.loni);
    Serial.println(buff);
  } else{
    Serial.println("No Fix");
  }
  Serial.println("-----------------");

  Serial.flush();
  Serial.end();
}

/*
 *
 */
void set_nada_1200(void){
  digitalWrite(APRS_PWM_OUT, true);
  //PORTB |= (1<<PB4);
  delayMicroseconds(tc1200);
  digitalWrite(APRS_PWM_OUT, LOW);
  //PORTB &= ~(1<<PB4);
  delayMicroseconds(tc1200);
}

void set_nada_2400(void){
  digitalWrite(APRS_PWM_OUT, true);
  //PORTB |= (1<<PB4);
  delayMicroseconds(tc2400);
  digitalWrite(APRS_PWM_OUT, LOW);
  //PORTB &= ~(1<<PB4);
  delayMicroseconds(tc2400);

  digitalWrite(APRS_PWM_OUT, true);
  //PORTB |= (1<<APRS_PWM_OUT);
  delayMicroseconds(tc2400);
  digitalWrite(APRS_PWM_OUT, LOW);
  //PORTB &= ~(1<<APRS_PWM_OUT);
  delayMicroseconds(tc2400);
}

void set_nada(bool nada)
{
  if(nada)
    set_nada_1200();
  else
    set_nada_2400();
}

/*
 * This function will calculate CRC-16 CCITT for the FCS (Frame Check Sequence)
 * as required for the HDLC frame validity check.
 *
 * Using 0x1021 as polynomial generator. The CRC registers are initialized with
 * 0xFFFF
 */
void calc_crc(bool in_bit)
{
  unsigned short xor_in;

  xor_in = crc ^ in_bit;
  crc >>= 1;

  if(xor_in & 0x01)
    crc ^= 0x8408;
}

void send_crc(void)
{
  unsigned char crc_lo = crc ^ 0xff;
  unsigned char crc_hi = (crc >> 8) ^ 0xff;

  send_char_NRZI(crc_lo, true);
  send_char_NRZI(crc_hi, true);
}

void send_header(char msg_type){
  char temp;

  /*
   * APRS AX.25 Header
   * ........................................................
   * |   DEST   |  SOURCE  |   DIGI   | CTRL FLD |    PID   |
   * --------------------------------------------------------
   * |  7 bytes |  7 bytes |  7 bytes |   0x03   |   0xf0   |
   * --------------------------------------------------------
   *
   * DEST   : 6 byte "callsign" + 1 byte ssid
   * SOURCE : 6 byte your callsign + 1 byte ssid
   * DIGI   : 6 byte "digi callsign" + 1 byte ssid
   *
   * ALL DEST, SOURCE, & DIGI are left shifted 1 bit, ASCII format.
   * DIGI ssid is left shifted 1 bit + 1
   *
   * CTRL FLD is 0x03 and not shifted.
   * PID is 0xf0 and not shifted.
   */

  /********* DEST ***********/
  if(msg_type == _BEACON){
    temp = strlen(dest_beacon);
    for(int j=0; j<temp; j++)
      send_char_NRZI(dest_beacon[j] << 1, true);
  }
  else{
    temp = strlen(dest);
    for(int j=0; j<temp; j++)
      send_char_NRZI(dest[j] << 1, true);
  }
  if(temp < 6){
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, true);
  }
  send_char_NRZI('0' << 1, true);



  /********* SOURCE *********/
  temp = strlen(mycall);
  for(int j=0; j<temp; j++)
    send_char_NRZI(mycall[j] << 1, true);
  if(temp < 6){
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, true);
  }
  send_char_NRZI((myssid + '0') << 1, true);


  /********* DIGI ***********/
  temp = strlen(digi);
  for(int j=0; j<temp; j++)
    send_char_NRZI(digi[j] << 1, true);
  if(temp < 6){
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, true);
  }
  send_char_NRZI(((digissid + '0') << 1) + 1, true);

  /***** CTRL FLD & PID *****/
  send_char_NRZI(_CTRL_ID, true);
  send_char_NRZI(_PID, true);
}

void send_payload(char type){
  /*
   * APRS AX.25 Payloads
   *
   * TYPE : POSITION
   * ........................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|
   * --------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |
   * --------------------------------------------------------
   *
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   *
   *
   * TYPE : STATUS
   * ..................................
   * |DATA TYPE |    STATUS TEXT      |
   * ----------------------------------
   * |  1 byte  |       N bytes       |
   * ----------------------------------
   *
   * DATA TYPE  : >
   * STATUS TEXT: Free form text
   *
   *
   * TYPE : POSITION & STATUS
   * ......................................................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|   HEIGHT     | STATUS TEXT  |
   * --------------------------------------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |   7 bytes    |    N bytes   |
   * --------------------------------------------------------------------------------------
   *
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * HEIGHT     : 7 bytes
   * STATUS TEXT: Free form text
   *
   *
   * TYPE : TELEMETRY REPORT
   * ......................................................................................................................................
   * | DATA TYPE |  SEQ No | ANALOG VALUE 1 | ANALOG VALUE 2 | ANALOG VALUE 3 | ANALOG VALUE 4 | ANALOG VALUE 5 | DIGITAL VALUE | COMMENT |
   * |  1 BYTE   | 5 BYTES |    4 BYTES     |    4 BYTES     |    4 BYTES     |    4 BYTES     |    4 BYTES     |    8 BYTES    | N BYTES |
   * |     T     |  #nnn,  |      aaa,      |      aaa,      |      aaa,      |      aaa,      |      aaa,      |    bbbbbbbb   |         |
   * ......................................................................................................................................
   *
   * DATA TYPE: T
   *
   *
   * All of the data are sent in the form of ASCII Text, not shifted.
   *
   */
  if(type == _FIXPOS){
    send_char_NRZI(_DT_POS, true);
    send_string_len(gps.lati, strlen(gps.lati));
    send_char_NRZI(sym_ovl, true);
    send_string_len(gps.loni, strlen(gps.loni));
    send_char_NRZI(sym_tab, true);
    send_string_len(heightpckt, strlen(heightpckt));
  }
  else if(type == _STATUS){
    send_char_NRZI(_DT_STATUS, true);
    send_string_len(mystatus, strlen(mystatus));
  }
  else if(type == _FIXPOS_STATUS){
    send_char_NRZI(_DT_POS, true);
    send_string_len(gps.lati, strlen(gps.lati));
    send_char_NRZI(sym_ovl, true);
    send_string_len(gps.loni, strlen(gps.loni));
    send_char_NRZI(sym_tab, true);
    send_string_len(heightpckt, strlen(heightpckt));
    send_string_len(comment, strlen(comment));
  }
  else if(type == _TELEMETRY){
    send_char_NRZI(telpct.dttype, true);
    send_string_len(telpct.seqN, strlen(telpct.seqN));
    send_string_len(telpct.an_1, strlen(telpct.an_1));
    send_string_len(telpct.an_2, strlen(telpct.an_2));
    send_string_len(telpct.an_3, strlen(telpct.an_3));
    send_string_len(telpct.an_4, strlen(telpct.an_4));
    send_string_len(telpct.an_5, strlen(telpct.an_5));
    send_string_len(telpct.dig_val, strlen(telpct.dig_val));
    send_string_len(telpct.comment, strlen(telpct.comment));
  }
  else{
    send_string_len(mystatus, strlen(mystatus));
  }
}

/*
 * This function will send one byte input and convert it
 * into AFSK signal one bit at a time LSB first.
 *
 * The encode which used is NRZI (Non Return to Zero, Inverted)
 * bit 1 : transmitted as no change in tone
 * bit 0 : transmitted as change in tone
 */
void send_char_NRZI(unsigned char in_byte, bool enBitStuff){
  bool bits;

  for(int i = 0; i < 8; i++)
  {
    bits = in_byte & 0x01;

    calc_crc(bits);

    if(bits)
    {
      set_nada(nada);
      bit_stuff++;

      if((enBitStuff) && (bit_stuff == 5))
      {
        nada ^= 1;
        set_nada(nada);

        bit_stuff = 0;
      }
    }
    else
    {
      nada ^= 1;
      set_nada(nada);

      bit_stuff = 0;
    }

    in_byte >>= 1;
  }
}

void send_string_len(const char *in_string, int len){
  for(int j=0; j<len; j++)
    send_char_NRZI(in_string[j], true);
}

void send_flag(unsigned char flag_len){
  for(int j=0; j<flag_len; j++)
    send_char_NRZI(_FLAG, LOW);
}

/*
 * In this preliminary test, a packet is consists of FLAG(s) and PAYLOAD(s).
 * Standard APRS FLAG is 0x7e character sent over and over again as a packet
 * delimiter. In this example, 100 flags is used the preamble and 3 flags as
 * the postamble.
 */
void send_packet(char packet_type){
  print_debug(packet_type);

  digitalWrite(LED_BUILTIN, HIGH);

  //delay(100);

  /*
   * AX25 FRAME
   *
   * ........................................................
   * |  FLAG(s) |  HEADER  | PAYLOAD  | FCS(CRC) |  FLAG(s) |
   * --------------------------------------------------------
   * |  N bytes | 22 bytes |  N bytes | 2 bytes  |  N bytes |
   * --------------------------------------------------------
   *
   * FLAG(s)  : 0x7e
   * HEADER   : see header
   * PAYLOAD  : 1 byte data type + N byte info
   * FCS      : 2 bytes calculated from HEADER + PAYLOAD
   */

  send_flag(150);
  crc = 0xffff;
  send_header(packet_type);
  send_payload(packet_type);
  send_crc();
  send_flag(3);

  digitalWrite(LED_BUILTIN, 0);
}

/*
 * Function to randomized the value of a variable with defined low and hi limit value.
 * Used to create random AFSK pulse length.
 */
void randomize(unsigned int &var, unsigned int low, unsigned int high){
  randomSeed(analogRead(A0));
  var = random(low, high);
}

void print_code_version(void){
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  Serial.println(" ");

  Serial.println("APRS Transmitter - Started ! \n");
}

void print_debug(char type){
  /*
   * PROTOCOL DEBUG.
   *
   * Will outputs the transmitted data to the serial monitor
   * in the form of TNC2 string format.
   *
   * MYCALL-N>APRS,DIGIn-N:<PAYLOAD STRING> <CR><LF>
   */
  Serial.begin(115200);

  /****** MYCALL ********/
  Serial.print(mycall);
  Serial.print('-');
  Serial.print(myssid, DEC);
  Serial.print('>');

  /******** DEST ********/
  if(type == _BEACON)
    Serial.print(dest_beacon);
  else
    Serial.print(dest);
  Serial.print(',');

  /******** DIGI ********/
  Serial.print(digi);
  Serial.print('-');
  Serial.print(digissid, DEC);
  Serial.print(':');

  /******* PAYLOAD ******/
  if(type == _FIXPOS){
    Serial.print(_DT_POS);
    Serial.print(gps.lati);
    Serial.print(sym_ovl);
    Serial.print(gps.loni);
    Serial.print(sym_tab);
    Serial.print(heightpckt);
  } else if(type == _STATUS) {
    Serial.print(_DT_STATUS);
    Serial.print(mystatus);
  } else if(type == _FIXPOS_STATUS) {
    Serial.print(_DT_POS);
    Serial.print(gps.lati);
    Serial.print(sym_ovl);
    Serial.print(gps.loni);
    Serial.print(sym_tab);
    Serial.print(heightpckt);
    Serial.print(comment);
  } else if(type == _TELEMETRY){
    Serial.print(telpct.dttype);
    Serial.print(telpct.seqN);
    Serial.print(telpct.an_1);
    Serial.print(telpct.an_2);
    Serial.print(telpct.an_3);
    Serial.print(telpct.an_4);
    Serial.print(telpct.an_5);
    Serial.print(telpct.dig_val);
    Serial.print(telpct.comment);
  } else {
    Serial.print(mystatus);
  }

  Serial.println(' ');

  Serial.flush();
  Serial.end();
}
