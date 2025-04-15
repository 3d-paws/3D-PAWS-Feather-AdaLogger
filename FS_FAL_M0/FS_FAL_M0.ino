/*
 *======================================================================================================================
 * FullStation Feather Ada Logger (FAL) - No Network Always on
 *   Board Type : Adafruit Feather M0
 *   Description: 
 *   Author: Robert Bubon
 *   Date:   2023-03-02 RJB Initial, Based on SSG_FAL_MO
 *           2025-04-14 RJB Moved Serial Console Pin to 12 from A4
 *                          Added reading pressure on BMX init
 *                          Wind Initialize now check cf_anemometer_enable before initializing
 *           2025-04-15 RJB Moved Distance from A3 to A4
 *                          Moved Rain from A1 to A3
 *                          Added cf_ds_enable and code to support distance sensor on A4.
 *                    
 * Adafruit Feather M0 Adalogger
 *   https://learn.adafruit.com/adafruit-feather-m0-adalogger/
 *   ATSAMD21G18 ARM Cortex M0 and Chips
 *     https://www.microchip.com/en-us/product/ATsamd21g18
 *     https://www.microchip.com/wwwproducts/en/MCP73831 - Battery Charger
 * 
 * Breakout Boards
 *   RTC DS3231 https://www.adafruit.com/product/5188, https://www.adafruit.com/product/3013
 * 
 * Setting RTC
 *   https://docs.google.com/document/d/175frIAoAJ5y6CAXXmnu5DXZdkZN9XwJHLCq7kx9fo1Q/edit
 * ======================================================================================================================
 */
#define VERSION_INFO "FSFALMO-250415"
#define W4SC false   // Set true to Wait for Serial Console to be connected

#include <SPI.h>
#include <Wire.h>
#include <ArduinoLowPower.h>
#include <SD.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_SI1145.h>

#include <RTClib.h>

#define RTC_I2C_ADDRESS 0x68       // I2C address for PCF8523 and DS3231

#define SSB_PWRON           0x1     // 1      Set at power on, but cleared after first observation
#define SSB_SD              0x2     // 2      Set if SD missing at boot or other SD related issues
#define SSB_RTC             0x4     // 4      Set if RTC missing at boot
#define SSB_OLED            0x8     // 8      Set if OLED missing at boot, but cleared after first observation
#define SSB_N2S             0x10    // 16     Set when Need to Send observations exist
#define SSB_FROM_N2S        0x20    // 32     Set in transmitted N2S observation when finally transmitted
#define SSB_AS5600          0x40    // 64     Set if wind direction sensor AS5600 has issues
#define SSB_BMX_1           0x80    // 128    Set if Barometric Pressure & Altitude Sensor missing
#define SSB_BMX_2           0x100   // 256    Set if Barometric Pressure & Altitude Sensor missing
#define SSB_HTU21DF         0x200   // 512    Set if Humidity & Temp Sensor missing
#define SSB_SI1145          0x400   // 1024   Set if UV index & IR & Visible Sensor missing
#define SSB_MCP_1           0x800   // 2048   Set if Precision I2C Temperature Sensor missing
#define SSB_MCP_2           0x1000  // 4096   Set if Precision I2C Temperature Sensor missing
#define SSB_LORA            0x2000  // 8192   Set if LoRa Radio missing at startup
#define SSB_PMIC            0x3000  // 16384  Set if Power Management IC missing at startup
#define SSB_SHT_1           0x4000  // 32768  Set if SHTX1 Sensor missing
#define SSB_SHT_2           0x8000  // 65536  Set if SHTX2 Sensor missing
#define SSB_HIH8           0x10000  // 131072 Set if HIH8000 Sensor missing

#define LED_PIN                   LED_BUILTIN
#define REBOOT_PIN                A0  // Connect to shoot thy self relay
#define HEARTBEAT_PIN             A6  // Connect to PICAXE-8M PIN-C3
#define OBSERVATION_INTERVAL      60   // Seconds


#define MAX_MSGBUF_SIZE   1024

unsigned int SystemStatusBits = SSB_PWRON; // Set bit 0 for initial value power on. Bit 0 is cleared after first obs
bool JustPoweredOn = true;         // Used to clear SystemStatusBits set during power on device discovery

/*
 * =======================================================================================================================
 *  Globals
 * =======================================================================================================================
 */
char msgbuf[MAX_MSGBUF_SIZE];
char *msgp;                   // Pointer to message text
char Buffer32Bytes[32];       // General storage
int countdown = 30;  //1800       // Exit calibration mode when reaches 0 - protects against burnt out pin or forgotten jumper
unsigned long lastOBS = 0;    // used to time next observation

int cf_anemometer_enable = 1;
int cf_raingauge_enable = 0;
int cf_ds_enable = 0;    // Enable Distance sensor
int cf_ds_type = 0;      // Distance sensor type 0 = 5m (default), 1 = 10m
int cf_ds_baseline = 0;  // Distance sensor baseline. If positive, distance = baseline - ds_median
/*
 * ======================================================================================================================
 *  Serial Console Enable
 * ======================================================================================================================
 */
int  SCE_PIN = 12;
bool SerialConsoleEnabled = false;  // Variable for serial monitor control

/*
 * =======================================================================================================================
 *  Measuring Battery - SEE https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/power-management
 * =======================================================================================================================
 */
#define VBATPIN          A7

/*
 * =======================================================================================================================
 * Distance Sensor
 * 
 * The 5-meter sensors (MB7360, MB7369, MB7380, and MB7389) use a scale factor of (Vcc/5120) per 1-mm.
 * Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 5119mm,  Each unit of the 0-4095 resolution is 1.25mm
 * Feather has 10bit resolution (0-1023), Sensor has a resolution of 0 - 5119mm, Each unit of the 0-1023 resolution is 5mm
 * 
 * The 10-meter sensors (MB7363, MB7366, MB7383, and MB7386) use a scale factor of (Vcc/10240) per 1-mm.
 * Particle 12bit resolution (0-4095), Sensor has a resolution of 0 - 10239mm, Each unit of the 0-4095 resolution is 2.5mm
 * Feather has 10bit resolution (0-1023), Sensor has a resolution of 0 - 10239mm, Each unit of the 0-1023 resolution is 10mm
 * 
 * The distance sensor will report as type sg  for Snow, Stream, or Surge gauge deployments.
 * A Median value based on 60 samples 250ms apart is obtain. Then subtracted from ds_baseline for the observation.
 * =======================================================================================================================
 */
#define DS_PIN     A4
#define DS_BUCKETS 60

unsigned int ds_bucket = 0;
unsigned int ds_buckets[DS_BUCKETS];

/*
 * ======================================================================================================================
 *  RTC Setup
 * ======================================================================================================================
 */
RTC_DS3231 rtc;
DateTime now;
char timestamp[32];
bool RTC_valid = false;
bool RTC_exists = false;

/*
 * ======================================================================================================================                         
 *  OLED Display 
 * ======================================================================================================================
 */
bool DisplayEnabled = true;
#define DISPLAY_TYPE 32  // Set Display Type: 32 = 4 lines, 64 = 8 lines

#define SCREEN_WIDTH 128 // OLED display width, in pixels

#if (DISPLAY_TYPE == 32)
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_I2C_ADDRESS 0x3C // 128x32
#define DISPLAY_LINES 4 
#else
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_I2C_ADDRESS 0x3D // 128x64
#define DISPLAY_LINES 8
#endif

char oled_lines[DISPLAY_LINES][23];
#define OLED_RESET A5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

/*
 * ======================================================================================================================
 *  SD Card
 * ======================================================================================================================
 */
#define SD_ChipSelect 4     // D4
// SD;                      // File system object defined by the SD.h include file.
File SD_fp;
char SD_obsdir[] = "/OBS";  // Store our obs in this directory. At Power on, it is created if does not exist
bool SD_exists = false;     // Set to true if SD card found at boot


/*
 * ======================================================================================================================
 *  Wind Related Setup
 * 
 *  NOTE: With interrupts tied to the anemometer rotation we are essentually sampling all the time.  
 *        We record the interrupt count, ms duration and wind direction every second.
 *        One revolution of the anemometer results in 2 interrupts. There are 2 magnets on the anemometer.
 * 
 *        Station observations are logged every minute
 *        Wind and Direction are sampled every second producing 60 samples 
 *        The one second wind speed sample are produced from the interrupt count and ms duration.
 *        Wind Observations a 
 *        Reported Observations
 *          Wind Speed = Average of the 60 samples.
 *          Wind Direction = Average of the 60 vectors from Direction and Speed.
 *          Wind Gust = Highest 3 consecutive samples from the 60 samples. The 3 samples are then averaged.
 *          Wind Gust Direction = Average of the 3 Vectors from the Wind Gust samples.
 * ======================================================================================================================
 */
#define ANEMOMETER_IRQ_PIN  A2
#define RAINGAUGE_IRQ_PIN   A3 
#define WIND_READINGS       60       // One minute of 1s Samples

typedef struct {
  int direction;
  float speed;
} WIND_BUCKETS_STR;

typedef struct {
  WIND_BUCKETS_STR bucket[WIND_READINGS];
  int bucket_idx;
  float gust;
  int gust_direction;
} WIND_STR;
WIND_STR wind;

/*
 * ======================================================================================================================
 *  Wind Direction - AS5600 Sensor
 * ======================================================================================================================
 */
bool      AS5600_exists     = true;
int       AS5600_ADR        = 0x36;
const int AS5600_raw_ang_hi = 0x0c;
const int AS5600_raw_ang_lo = 0x0d;

/*
 * ======================================================================================================================
 *  Wind Speed Calibration
 * ======================================================================================================================
 */
float ws_calibration = 2.64;       // From wind tunnel testing
float ws_radius = 0.079;           // In meters

/*
 * ======================================================================================================================
 *  Optipolar Hall Effect Sensor SS451A - Interrupt 0 - Rain Gauge
 * ======================================================================================================================
 */
volatile unsigned int raingauge_interrupt_count;
uint64_t raingauge_interrupt_stime; // Send Time
uint64_t raingauge_interrupt_ltime; // Last Time
uint64_t raingauge_interrupt_toi;   // Time of Interupt

/*
 * ======================================================================================================================
 *  Optipolar Hall Effect Sensor SS451A - Interrupt 1 - Anemometer
 * ======================================================================================================================
 */
volatile unsigned int anemometer_interrupt_count;
unsigned long anemometer_interrupt_stime;

/*
 * ======================================================================================================================
 *  raingauge_interrupt_handler() - This function is called whenever a magnet/interrupt is detected by the arduino
 * ======================================================================================================================
 */
void raingauge_interrupt_handler()
{
  if ((millis() - raingauge_interrupt_ltime) > 500) { // Count tip if a half second has gone by since last interupt
    raingauge_interrupt_ltime = millis();
    raingauge_interrupt_count++;
  }   
}


/*
 * ======================================================================================================================
 *  anemometer_interrupt_handler() - This function is called whenever a magnet/interrupt is detected by the arduino
 * ======================================================================================================================
 */
void anemometer_interrupt_handler()
{
  anemometer_interrupt_count++;
}

/*
 * ======================================================================================================================
 *  BMX280 humidity - I2C - Temperature, pressure sensor & altitude - Support 2 of any combination
 * 
 *  https://www.asknumbers.com/PressureConversion.aspx
 *  Pressure is returned in the SI units of Pascals. 100 Pascals = 1 hPa = 1 millibar. 
 *  Often times barometric pressure is reported in millibar or inches-mercury. 
 *  For future reference 1 pascal = 0.000295333727 inches of mercury, or 1 inch Hg = 3386.39 Pascal. 
 *
 *  Looks like you divide by 100 and you get millibars which matches NWS page
 * 
 *  Surface Observations and Station Elevation 
 *  https://forecast.weather.gov/product.php?issuedby=BOU&product=OSO&site=bou 
 * ======================================================================================================================
 */
#define BMX_STATION_ELEVATION 1017.272  // default 1013.25
#define BMX_ADDRESS_1         0x77        // BMP Default Address - Connecting SDO to GND will change BMP to 0x76
#define BMX_ADDRESS_2         0x76        // BME Default Address - Connecting SDO to GND will change BME to 0x77
#define BMP280_CHIP_ID        0x58
#define BME280_BMP390_CHIP_ID 0x60
#define BMP388_CHIP_ID        0x50
#define BMX_TYPE_UNKNOWN      0
#define BMX_TYPE_BMP280       1
#define BMX_TYPE_BME280       2
#define BMX_TYPE_BMP388       3
#define BMX_TYPE_BMP390       4
Adafruit_BMP280 bmp1;
Adafruit_BMP280 bmp2;
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
Adafruit_BMP3XX bm31;
Adafruit_BMP3XX bm32;
byte BMX_1_chip_id = 0x00;
byte BMX_2_chip_id = 0x00;
bool BMX_1_exists = false;
bool BMX_2_exists = false;
byte BMX_1_type=BMX_TYPE_UNKNOWN;
byte BMX_2_type=BMX_TYPE_UNKNOWN;

/*
 * ======================================================================================================================
 *  SHTX - I2C - Temperature & Humidity sensor (SHT31)  - Note the SHT40, SHT45 use same i2c address
 * ======================================================================================================================
 */
#define SHT_ADDRESS_1     0x44
#define SHT_ADDRESS_2     0x45        // ADR pin set high, VDD
Adafruit_SHT31 sht1;
Adafruit_SHT31 sht2;
bool SHT_1_exists = false;
bool SHT_2_exists = false;

/*
 * ======================================================================================================================
 *  HIH8 - I2C - Temperature & Humidity sensor (HIH8000)  - 
 * ======================================================================================================================
 */
#define HIH8000_ADDRESS   0x27
bool HIH8_exists = false;

/*
 * ======================================================================================================================
 *  HTU21D-F - I2C - Humidity & Temp Sensor
 * ======================================================================================================================
 */
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
bool HTU21DF_exists = false;

/*
 * ======================================================================================================================
 *  MCP9808 - I2C - Temperature sensor
 * 
 * I2C Address is:  0011,A2,A1,A0
 *                  0011000 = 0x18  where A2,1,0 = 0 MCP9808_I2CADDR_DEFAULT  
 *                  0011001 = 0x19  where A0 = 1
 * ======================================================================================================================
 */
#define MCP_ADDRESS_1     0x18
#define MCP_ADDRESS_2     0x19        // A0 set high, VDD
Adafruit_MCP9808 mcp1;
Adafruit_MCP9808 mcp2;
bool MCP_1_exists = false;
bool MCP_2_exists = false;

/*
 * ======================================================================================================================
 *  Si1145 - I2C - UV/IR/Visible Light Sensor
 *  The SI1145 has a fixed I2C address (0x60), you can only connect one sensor per microcontroller!
 * ======================================================================================================================
 */
Adafruit_SI1145 uv = Adafruit_SI1145();
bool SI1145_exists = false;
// When we do a read of all three and we get zeros. If these last readings are not zero, we will reinitialize the
// chip. When does a reset on it and then read again.
float si_last_vis = 0.0;
float si_last_ir = 0.0;
float si_last_uv = 0.0;

/*
 *======================================================================================================================
 * myswap()
 *======================================================================================================================
 */
void myswap(unsigned int *p, unsigned int *q) {
  int t;
  
  t=*p;
  *p=*q;
  *q=t;
}

/*
 *======================================================================================================================
 * mysort()
 *======================================================================================================================
 */
void mysort(unsigned int a[], int n)
{
  unsigned int i,j,temp;

  for(i = 0;i < n-1;i++) {
    for(j = 0;j < n-i-1;j++) {
      if(a[j] > a[j+1])
        myswap(&a[j],&a[j+1]);
    }
  }
}

/*
 * =======================================================================================================================
 * isnumeric() - check if string contains all digits
 * =======================================================================================================================
 */
bool isnumeric(char *s) {
  for (int i=0; i< strlen(s); i++) {
    if (!isdigit(*(s+i)) ) {
      return(false);
    }
  }
  return(true);
}


/*
 * ======================================================================================================================
 * Blink() - Count, delay between, delay at end
 * ======================================================================================================================
 */
void Blink(int count, int between)
{
  int c;

  for (c=0; c<count; c++) {
    digitalWrite(LED_PIN, HIGH);
    delay(between);
    digitalWrite(LED_PIN, LOW);
    delay(between);
  }
}

/* 
 *=======================================================================================================================
 * rtc_timestamp() - Read from RTC and set timestamp string
 *=======================================================================================================================
 */
void rtc_timestamp() {
  now = rtc.now(); //get the current date-time

  // ISO_8601 Time Format
  sprintf (timestamp, "%d-%02d-%02dT%02d:%02d:%02d", 
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second());
}

/*
 * ======================================================================================================================
 * OLED_sleepDisplay()
 * ======================================================================================================================
 */
void OLED_sleepDisplay() {
  if (DisplayEnabled) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

/*
 * ======================================================================================================================
 * OLED_wakeDisplay()
 * ======================================================================================================================
 */
void OLED_wakeDisplay() {
  if (DisplayEnabled) {
    display.ssd1306_command(SSD1306_DISPLAYON);
  }
}

/*
 * ======================================================================================================================
 * OLED_spin() 
 * ======================================================================================================================
 */
void OLED_spin() {
  static int spin=0;
    
  if (DisplayEnabled) {
    display.setTextColor(WHITE, BLACK); // Draw 'inverse' text
    if (DISPLAY_LINES == 4) {
      display.setCursor(120,24);
      display.print(" ");
      display.setCursor(120,24); 
    }
    else {
      display.setCursor(120,56);
      display.print(" ");
      display.setCursor(120,56);       
    } 
    switch (spin++) {
      case 0 : msgp = (char *) "|"; break;
      case 1 : msgp = (char *) "/"; break;
      case 2 : msgp = (char *) "-"; break;
      case 3 : msgp = (char *) "\\"; break;
    }
    display.print(msgp);
    display.display();
    spin %= 4;
  }
}

/*
 * ======================================================================================================================
 * OLED_ClearDisplayBuffer() -- Clear display buffer with spaces
 * ======================================================================================================================
 */
void OLED_ClearDisplayBuffer() {
  int r,c;
  
  for (r=0; r<DISPLAY_LINES; r++) {
    for (c=0; c<22; c++) {
      oled_lines [r][c] = ' ';
    }
    oled_lines [r][c] = (char) NULL;
  }
}

/*
 * ======================================================================================================================
 * OLED_update() -- Output oled in memory map to display
 * ======================================================================================================================
 */
void OLED_update() {  
  if (DisplayEnabled) {
    display.clearDisplay();
    display.setCursor(0,0);             // Start at top-left corner
    display.print(oled_lines [0]);
    display.setCursor(0,8);
    display.print(oled_lines [1]);
    display.setCursor(0,16);
    display.print(oled_lines [2]);
    display.setCursor(0,24);  
    display.print(oled_lines [3]);
    
    if (DISPLAY_LINES == 8) {
      display.setCursor(0,32);  
      display.print(oled_lines [4]);
      display.setCursor(0,40);  
      display.print(oled_lines [5]);
      display.setCursor(0,48);  
      display.print(oled_lines [6]);
      display.setCursor(0,56);  
      display.print(oled_lines [7]);     
    }
    display.display();
  }
}

/*
 * ======================================================================================================================
 * OLED_write() 
 * ======================================================================================================================
 */
void OLED_write(const char *str) {
  int c, len, bottom_line = 3;
  
  if (DisplayEnabled) {
    // move lines up
    for (c=0; c<=21; c++) {
      oled_lines [0][c] = oled_lines [1][c];
      oled_lines [1][c] = oled_lines [2][c];
      oled_lines [2][c] = oled_lines [3][c];
      if (DISPLAY_LINES == 8) {
        oled_lines [3][c] = oled_lines [4][c];
        oled_lines [4][c] = oled_lines [5][c];
        oled_lines [5][c] = oled_lines [6][c];  
        oled_lines [6][c] = oled_lines [7][c];  
        bottom_line = 7;          
      }
    }

    // check length on new output line string
    len = strlen (str);
    if (len>21) {
      len = 21;
    }
    for (c=0; c<=len; c++) {
      oled_lines [bottom_line][c] = *(str+c);
    }

    // Adding Padding
    for (;c<=21; c++) {
      oled_lines [bottom_line][c] = ' ';
    }
    oled_lines [bottom_line][22] = (char) NULL;
    
    OLED_update();
  }
}

/*
 * ======================================================================================================================
 * OLED_write_noscroll() -- keep lines 1-3 and output on line 4
 * ======================================================================================================================
 */
void OLED_write_noscroll(const char *str) {
  int c, len, bottom_line = 3;

  if (DISPLAY_LINES == 8) {
    bottom_line = 7;
  }
  
  if (DisplayEnabled) {
    len = strlen (str);
    if (len>21) {
      len = 21;
    }
    
    for (c=0; c<=len; c++) {
      oled_lines [bottom_line][c] = *(str+c);
    }

    // Adding Padding
    for (;c<=21; c++) {
      oled_lines [bottom_line][c] = ' ';
    }
    oled_lines [bottom_line][22] = (char) NULL;
    
    OLED_update();
  }
}

/*
 * ======================================================================================================================
 * Serial_flush() 
 * ======================================================================================================================
 */
void Serial_flush() {
  if (SerialConsoleEnabled) {
    Serial.flush();
  }
}

/*
 * ======================================================================================================================
 * Serial_write() 
 * ======================================================================================================================
 */
void Serial_write(const char *str) {
  if (SerialConsoleEnabled) {
    Serial.print(str);
  }
}

/*
 * ======================================================================================================================
 * Serial_writeln() 
 * ======================================================================================================================
 */
void Serial_writeln(const char *str) {
  if (SerialConsoleEnabled) {
    Serial.println(str);
    Serial.flush();
  }
}

/*
 * ======================================================================================================================
 * Output() - Count, delay between, delay at end
 * ======================================================================================================================
 */
void Output(const char *str) {
  OLED_write(str);
  Serial_writeln(str);
}

/*
 * ======================================================================================================================
 * OutputNS() - Output with no scroll on oled
 * ======================================================================================================================
 */
void OutputNS(const char *str) {
  OLED_write_noscroll(str);
  Serial_write(str);
}

/*
 *=======================================================================================================================
 * vbat_get() -- return battery voltage
 *=======================================================================================================================
 */
float vbat_get() {
  float v = analogRead(VBATPIN);
  v *= 2;    // we divided by 2, so multiply back
  v *= 3.3;  // Multiply by 3.3V, our reference voltage
  v /= 1024; // convert to voltage
  return (v);
}

/* 
 *=======================================================================================================================
 * rtc_initialize()
 *=======================================================================================================================
 */
void rtc_initialize() {

  if (!rtc.begin()) { // Always returns true
     Output("ERR:RTC NOT FOUND");
     SystemStatusBits |= SSB_RTC; // Turn on Bit
     return;
  }
  
  if (!I2C_Device_Exist(RTC_I2C_ADDRESS)) {
    Output("ERR:RTC-I2C NOTFOUND");
    SystemStatusBits |= SSB_RTC; // Turn on Bit
    delay (5000);
    return;
  }

  RTC_exists = true; // We have a clock hardware connected

  rtc_timestamp();
  sprintf (msgbuf, "RTC:%s", timestamp);
  Output (msgbuf);

  // Do a validation check on the year. 
  // Asumption is: If RTC not set, it will not have the current year.

  if ((now.year() >= 2025) && (now.year() <= 2035)) {
    now = rtc.now();
    RTC_valid = true;
  }
  else {
    Output ("NEED TIME->RTC");
  }
}

/*
 * =======================================================================================================================
 * rtc_readserial() - // check for serial input, validate for rtc, set rtc, report result
 * =======================================================================================================================
 */
bool rtc_readserial()
{
  boolean ready = false;
  int cnt = 0;
  char buffer[32];
  char *p, *token;
  int year, month, day, hour, minute, second;
  
  while (Serial.available()) {
    char c = Serial.read();
    buffer[cnt++] = c;
    if ((c == '\n') || (cnt == 31) ){
      buffer[cnt] = '\0';  // Note: there will be a \r\n on end of string in buffer
      cnt = 0;
      Serial.flush(); // if anything left in the Serial buffer, get rid of it

      // Validate User input for a good date and time
      p = &buffer[0];
      token = strtok_r(p, ":", &p);
      if (isnumeric(token) && (year = atoi (token)) && (year >= 2022) && (year <= 2031) ) {   // FOO set back 2022
        token = strtok_r(p, ":", &p);
        if (isnumeric(token) && (month = atoi (token)) && (month >= 1) && (month <= 12) ) {
          token = strtok_r(p, ":", &p);        
          if (isnumeric(token) && (day = atoi (token)) && 
               (
                 ( (day>=1  && day<=31) && (month==1 || month==3 || month==5 || month==7 || month==8 || month==10 || month==12) ) ||
                 ( (day>=1  && day<=30) && (month==4 || month==6 || month==9 || month==11) ) ||
                 ( (day>=1  && day<=28) && (month==2) ) ||
                 ( (day==29)            && (month==2) && ( (year%400==0) || ( (year%4==0) && (year%100!=0) ) ) )
                ) 
             ) {
            token = strtok_r(p, ":", &p);
            hour = atoi (token);
            if ( (isnumeric(token) && (hour >= 0) && (hour <= 23)) ) {
              token = strtok_r(p, ":", &p);
              minute = atoi (token);
              if ( (isnumeric(token) && (minute >= 0) && (minute <= 59)) ) {
                token = strtok_r(p, "\r", &p);
                second = atoi (token);
                if ( (isnumeric(token) && (second >= 0) && (second <= 59)) ) { 
                  sprintf (msgbuf, ">%d.%d.%d.%d.%d.%d", 
                     year, month, day, hour, minute, second);
                  rtc.adjust(DateTime(year, month, day, hour, minute, second));
                  Output("RTC: Set");
                  RTC_valid = true;
                  rtc_timestamp();
                  sprintf (msgbuf, "%s", timestamp);
                  Output (msgbuf);
                  return(true);
                }
                else {
                  sprintf (msgbuf, "Invalid Second: %s", token);
                  Output(msgbuf);
                  return(false);
                }
              }
              else {
                sprintf (msgbuf, "Invalid Minute: %s", token);
                Output(msgbuf);
                return(false);
              }
            }
            else {
              sprintf (msgbuf, "Invalid Hour: %s", token);
              Output(msgbuf);
              return(false);
            }
          }
          else {
            sprintf (msgbuf, "Invalid Day: %s", token);
            Output(msgbuf);
            return(false);
          }
        }
        else {
          sprintf (msgbuf, "Invalid Month: %s", token);
          Output(msgbuf);
          return(false);
        }                
      }
      else {
        sprintf (msgbuf, "Invalid Year: %s", token);
        Output(msgbuf);
        return(false);
      }
    } // if line
  } // while
  return(false);
}

/* 
 *=======================================================================================================================
 * SD_initialize()
 *=======================================================================================================================
 */
void SD_initialize() {
  if (!SD.begin(SD_ChipSelect)) {
    Output ("SD:NF");
    SystemStatusBits |= SSB_SD;
    delay (5000);
  }
  else {
    SD_exists = true;
    if (!SD.exists(SD_obsdir)) {
      if (SD.mkdir(SD_obsdir)) {
        Output ("SD:MKDIR OBS OK");
        Output ("SD:Online");
        SD_exists = true;
      }
      else {
        Output ("SD:MKDIR OBS ERR");
        Output ("SD:Offline");
        SystemStatusBits |= SSB_SD;  // Turn On Bit     
      } 
    }
    else {
      Output ("SD:Online");
      Output ("SD:OBS DIR Exists");
      SD_exists = true;
    }
  }
}

/* 
 *=======================================================================================================================
 * SD_LogObservation()
 *=======================================================================================================================
 */
void SD_LogObservation(char *observations) {
  char SD_logfile[24];
  File fp;

  if (!SD_exists) {
    return;
  }

  if (!RTC_valid) {
    return;
  }

  // Note: "now" is global and is set when ever timestamp() is called. Value last read from RTC.
  sprintf (SD_logfile, "%s/%4d%02d%02d.log", SD_obsdir, now.year(), now.month(), now.day());
  Output (SD_logfile);
  
  fp = SD.open(SD_logfile, FILE_WRITE); 
  if (fp) {
    fp.println(observations);
    fp.close();
    SystemStatusBits &= ~SSB_SD;  // Turn Off Bit
    Output ("OBS Logged to SD");
  }
  else {
    SystemStatusBits |= SSB_SD;  // Turn On Bit - Note this will be reported on next observation
    Output ("OBS Open Log Err");
    // At thins point we could set SD_exists to false and/or set a status bit to report it
    // SD_initialize();  // Reports SD NOT Found. Library bug with SD
  }
}

/* 
 *=======================================================================================================================
 * Wind_SampleSpeed() - Return a wind speed based on interrupts and duration wind
 * 
 * Optipolar Hall Effect Sensor SS451A - Anemometer
 * speed  = (( (signals/2) * (2 * pi * radius) ) / time) * calibration_factor
 * speed in m/s =  (   ( (interrupts/2) * (2 * 3.14156 * 0.079) )  / (time_period in ms / 1000)  )  * 2.64
 *=======================================================================================================================
 */
float Wind_SampleSpeed() {
  unsigned long delta_ms, time_ms;
  float wind_speed;
  
  time_ms = millis();

  // Handle the clock rollover after about 50 days. (Should not be an issue since we reboot every day)
  if (time_ms < anemometer_interrupt_stime) {
    delta_ms = (0xFFFFFFFF - anemometer_interrupt_stime) + time_ms;
  }
  else {
    delta_ms = millis()-anemometer_interrupt_stime;
  }
  
  if (anemometer_interrupt_count && (delta_ms>0)) {
    wind_speed = ( ( anemometer_interrupt_count * 3.14156 * ws_radius)  / 
      (float)( (float)delta_ms / 1000) )  * ws_calibration;
  }
  else {
    wind_speed = 0.0;
  }

  anemometer_interrupt_count = 0;
  anemometer_interrupt_stime = millis(); 
  return (wind_speed);
} 

/* 
 *=======================================================================================================================
 * Wind_SampleDirection() -- Talk i2c to the AS5600 sensor and get direction
 *=======================================================================================================================
 */
int Wind_SampleDirection() {
  int degree;
  
  // Read Raw Angle Low Byte
  Wire.beginTransmission(AS5600_ADR);
  Wire.write(AS5600_raw_ang_lo);
  if (Wire.endTransmission()) {
    if (AS5600_exists) {
      Output ("WD Offline_L");
    }
    AS5600_exists = false;
  }
  else if (Wire.requestFrom(AS5600_ADR, 1)) {
    int AS5600_lo_raw = Wire.read();
  
    // Read Raw Angle High Byte
    Wire.beginTransmission(AS5600_ADR);
    Wire.write(AS5600_raw_ang_hi);
    if (Wire.endTransmission()) {
      if (AS5600_exists) {
        Output ("WD Offline_H");
      }
      AS5600_exists = false;
    }
    else if (Wire.requestFrom(AS5600_ADR, 1)) {
      word AS5600_hi_raw = Wire.read();

      if (!AS5600_exists) {
        Output ("WD Online");
      }
      AS5600_exists = true;           // We made it 
      SystemStatusBits &= ~SSB_AS5600; // Turn Off Bit
      
      AS5600_hi_raw = AS5600_hi_raw << 8; //shift raw angle hi 8 left
      AS5600_hi_raw = AS5600_hi_raw | AS5600_lo_raw; //AND high and low raw angle value

      // Do data integ check
      degree = (int) AS5600_hi_raw * 0.0879;
      if ((degree >=0) && (degree <= 360)) {
        return (degree);
      }
      else {
        return (-1);
      }
    }
  }
  SystemStatusBits |= SSB_AS5600;  // Turn On Bit
  return (-1); // Not the best value to return 
}

/* 
 *=======================================================================================================================
 * Wind_DirectionVector()
 *=======================================================================================================================
 */
int Wind_DirectionVector() {
  double NS_vector_sum = 0.0;
  double EW_vector_sum = 0.0;
  double r;
  float s;
  int d, i, rtod;
  bool ws_zero = true;

  for (i=0; i<WIND_READINGS; i++) {
    d = wind.bucket[i].direction;

    // if at any time 1 of the 60 wind direction readings is -1
    // then the sensor was offline and we need to invalidate or data
    // until it is clean with out any -1's
    if (d == -1) {
      return (-1);
    }
    
    s = wind.bucket[i].speed;

    // Flag we have wind speed
    if (s > 0) {
      ws_zero = false;  
    }
    r = (d * 71) / 4068.0;
    
    // North South Direction 
    NS_vector_sum += cos(r) * s;
    EW_vector_sum += sin(r) * s;
  }
  rtod = (atan2(EW_vector_sum, NS_vector_sum)*4068.0)/71.0;
  if (rtod<0) {
    rtod = 360 + rtod;
  }

  // If all the winds speeds are 0 then we return current wind direction or 0 on failure of that.
  if (ws_zero) {
    return (Wind_SampleDirection()); // Can return -1
  }
  else {
    return (rtod);
  }
}

/* 
 *=======================================================================================================================
 * Wind_SpeedAverage()
 *=======================================================================================================================
 */
float Wind_SpeedAverage() {
  float wind_speed = 0.0;
  for (int i=0; i<WIND_READINGS; i++) {
    // sum wind speeds for later average
    wind_speed += wind.bucket[i].speed;
  }
  return( wind_speed / (float) WIND_READINGS);
}

/* 
 *=======================================================================================================================
 * Wind_Gust()
 *=======================================================================================================================
 */
float Wind_Gust() {
  return(wind.gust);
}

/* 
 *=======================================================================================================================
 * Wind_GustDirection()
 *=======================================================================================================================
 */
int Wind_GustDirection() {
  return(wind.gust_direction);
}

/* 
 *=======================================================================================================================
 * Wind_GustUpdate()
 *   Wind Gust = Highest 3 consecutive samples from the 60 samples. The 3 samples are then averaged.
 *   Wind Gust Direction = Average of the 3 Vectors from the Wind Gust samples.
 * 
 *   Note: To handle the case of 2 or more gusts at the same speed but different directions
 *          Sstart with oldest reading and work forward to report most recent.
 * 
 *   Algorithm: 
 *     Start with oldest reading.
 *     Sum this reading with next 2.
 *     If greater than last, update last 
 * 
 *=======================================================================================================================
 */
void Wind_GustUpdate() {
  int bucket = wind.bucket_idx; // Start at next bucket to fill (aka oldest reading)
  float ws_sum = 0.0;
  int ws_bucket = bucket;
  float sum;

  for (int i=0; i<(WIND_READINGS-2); i++) {  // subtract 2 because we are looking ahead at the next 2 buckets
    // sum wind speeds 
    sum = wind.bucket[bucket].speed +
          wind.bucket[(bucket+1) % WIND_READINGS].speed +
          wind.bucket[(bucket+2) % WIND_READINGS].speed;
    if (sum >= ws_sum) {
      ws_sum = sum;
      ws_bucket = bucket;
    }
    bucket = (++bucket) % WIND_READINGS;
  }
  wind.gust = ws_sum/3;
  
  // Determine Gust Direction 
  double NS_vector_sum = 0.0;
  double EW_vector_sum = 0.0;
  double r;
  float s;
  int d, i, rtod;
  bool ws_zero = true;

  bucket = ws_bucket;
  for (i=0; i<3; i++) {
    d = wind.bucket[bucket].direction;

    // if at any time any wind direction readings is -1
    // then the sensor was offline and we need to invalidate or data
    // until it is clean with out any -1's
    if (d == -1) {
      ws_zero = true;
      break;
    }
    
    s = wind.bucket[bucket].speed;

    // Flag we have wind speed
    if (s > 0) {
      ws_zero = false;  
    }
    r = (d * 71) / 4068.0;
    
    // North South Direction 
    NS_vector_sum += cos(r) * s;
    EW_vector_sum += sin(r) * s;

    bucket = (++bucket) % WIND_READINGS;
  }

  rtod = (atan2(EW_vector_sum, NS_vector_sum)*4068.0)/71.0;
  if (rtod<0) {
    rtod = 360 + rtod;
  }

  // If all the winds speeds are 0 or we has a -1 direction then set -1 dor direction.
  if (ws_zero) {
    wind.gust_direction = -1;
  }
  else {
    wind.gust_direction = rtod;
  }
}

/*
 * ======================================================================================================================
 * Wind_TakeReading() - Wind direction and speed, measure every second             
 * ======================================================================================================================
 */
void Wind_TakeReading() {
  if (cf_anemometer_enable) {
    wind.bucket[wind.bucket_idx].direction = (int) Wind_SampleDirection();
    wind.bucket[wind.bucket_idx].speed = Wind_SampleSpeed();
    wind.bucket_idx = (++wind.bucket_idx) % WIND_READINGS; // Advance bucket index for next reading
  }
}

/* 
 *=======================================================================================================================
 * Wind_Initiailize()
 *=======================================================================================================================
 */
void Wind_Initiailize() {
  float ws;
  
  Output ("Wind:INIT");

  // Clear windspeed counter  
  anemometer_interrupt_count = 0;
  anemometer_interrupt_stime = millis();
  
  // Init default values.
  wind.gust = 0.0;
  wind.gust_direction = -1;
  wind.bucket_idx = 0;

  // Take N 1s samples of wind speed and direction and fill arrays with values.
  for (int i=0; i< WIND_READINGS; i++) {
    delay(1000);
       
    Wind_TakeReading();
  
    if (SerialConsoleEnabled) Serial.print(".");  // Provide Serial Console some feedback as we loop and wait til next observation
    OLED_spin();
  }

  // Now we have N readings we can compute other wind related global varibles
  Wind_TakeReading();

  if (SerialConsoleEnabled) Serial.println();  // Send a newline out to cleanup after all the periods we have been logging

  ws = Wind_SpeedAverage();
  sprintf (Buffer32Bytes, "WS:%d.%02d WD:%d", (int)ws, (int)(ws*100)%100, Wind_DirectionVector());
  Output (Buffer32Bytes);

}

/* 
 *=======================================================================================================================
 * I2C_Device_Exist - does i2c device exist at address
 * 
 *  The i2c_scanner uses the return value of the Write.endTransmisstion to see 
 *  if a device did acknowledge to the address.
 *=======================================================================================================================
 */
bool I2C_Device_Exist(byte address) {
  byte error;

  Wire.begin();                     // Connect to I2C as Master (no addess is passed to signal being a slave)

  Wire.beginTransmission(address);  // Begin a transmission to the I2C slave device with the given address. 
                                    // Subsequently, queue bytes for transmission with the write() function 
                                    // and transmit them by calling endTransmission(). 

  error = Wire.endTransmission();   // Ends a transmission to a slave device that was begun by beginTransmission() 
                                    // and transmits the bytes that were queued by write()
                                    // By default, endTransmission() sends a stop message after transmission, 
                                    // releasing the I2C bus.

  // endTransmission() returns a byte, which indicates the status of the transmission
  //  0:success
  //  1:data too long to fit in transmit buffer
  //  2:received NACK on transmit of address
  //  3:received NACK on transmit of data
  //  4:other error 

  // Partice Library Return values
  // SEE https://docs.particle.io/cards/firmware/wire-i2c/endtransmission/
  // 0: success
  // 1: busy timeout upon entering endTransmission()
  // 2: START bit generation timeout
  // 3: end of address transmission timeout
  // 4: data byte transfer timeout
  // 5: data byte transfer succeeded, busy timeout immediately after
  // 6: timeout waiting for peripheral to clear stop bit

  if (error == 0) {
    return (true);
  }
  else {
    // sprintf (msgbuf, "I2CERR: %d", error);
    // Output (msgbuf);
    return (false);
  }
}


/* 
 *=======================================================================================================================
 * as5600_initialize() - wind direction sensor
 *=======================================================================================================================
 */
void as5600_initialize() {
  Output("AS5600:INIT");
  Wire.beginTransmission(AS5600_ADR);
  if (Wire.endTransmission()) {
    msgp = (char *) "WD:NF";
    AS5600_exists = false;
    SystemStatusBits |= SSB_AS5600;  // Turn On Bit
  }
  else {
    msgp = (char *) "WD:OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * get_Bosch_ChipID ()  -  Return what Bosch chip is at specified address
 *   Chip ID BMP280 = 0x58 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)  
 *   Chip ID BME280 = 0x60 temp, preasure, humidity - I2C ADDRESS 0x77  (SD0 to GND = 0x76)  Register 0xE0 = Reset
 *   Chip ID BMP388 = 0x50 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *   Chip ID BMP390 = 0x60 temp, preasure           - I2C ADDRESS 0x77  (SD0 to GND = 0x76)
 *=======================================================================================================================
 */
byte get_Bosch_ChipID (byte address) {
  byte chip_id = 0;
  byte error;

  Output ("get_Bosch_ChipID()");
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.

  // Important! Need to check the 0x00 register first. Doing a 0x0D (not chip id loaction) on a bmp388 
  // will return a value that could match one of the IDs 

  // Check Register 0x00
  sprintf (msgbuf, "  I2C:%02X Reg:%02X", address, 0x00);
  Output (msgbuf);
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x00);  // BM3 CHIPID REGISTER
  error = Wire.endTransmission();
    //  0:success
    //  1:data too long to fit in transmit buffer
    //  2:received NACK on transmit of address
    //  3:received NACK on transmit of data
    //  4:other error 
  if (error) {
    sprintf (msgbuf, "  EWT_ERR:%d", error);
    Output (msgbuf);
  }
  else if (Wire.requestFrom(address, 1)) {  // Returns the number of bytes returned from the slave device 
    chip_id = Wire.read();
    if (chip_id == BMP280_CHIP_ID) { // 0x58
      sprintf (msgbuf, "  CHIPID:%02X BMP280", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!
    }
    else if (chip_id == BMP388_CHIP_ID) {
      sprintf (msgbuf, "  CHIPID:%02X BMP388", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else if (chip_id == BME280_BMP390_CHIP_ID) {  // 0x60
      sprintf (msgbuf, "  CHIPID:%02X BME/390", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else {
      sprintf (msgbuf, "  CHIPID:%02X InValid", chip_id);
      Output (msgbuf);      
    }
  }
  else {
    sprintf (msgbuf, "  ERR_RF:0");
    Output (msgbuf);
  }

  // Check Register 0xD0
  chip_id = 0;
  sprintf (msgbuf, "  I2C:%02X Reg:%02X", address, 0xD0);
  Output (msgbuf);
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0xD0);  // BM2 CHIPID REGISTER
  error = Wire.endTransmission();
    //  0:success
    //  1:data too long to fit in transmit buffer
    //  2:received NACK on transmit of address
    //  3:received NACK on transmit of data
    //  4:other error 
  if (error) {
    sprintf (msgbuf, "  EWT_ERR:%d", error);
    Output (msgbuf);
  }
  else if (Wire.requestFrom(address, 1)) {  // Returns the number of bytes returned from the slave device 
    chip_id = Wire.read(); 
    if (chip_id == BMP280_CHIP_ID) {  // 0x58
      sprintf (msgbuf, "  CHIPID:%02X BMP280", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!
    }
    else if (chip_id == BMP388_CHIP_ID) {  // 0x50
      sprintf (msgbuf, "  CHIPID:%02X BMP388", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else if (chip_id == BME280_BMP390_CHIP_ID) {  // 0x60
      sprintf (msgbuf, "  CHIPID:%02X BME/390", chip_id);
      Output (msgbuf);
      return (chip_id); // Found a Sensor!   
    }
    else {
      sprintf (msgbuf, "  CHIPID:%02X InValid", chip_id);
      Output (msgbuf);   
    }
  }
  else {
    sprintf (msgbuf, "  ERR_RF:0");
    Output (msgbuf);
  }
  return(0);
}

/* 
 *=======================================================================================================================
 * bmx_initialize() - Bosch sensor initialize
 *=======================================================================================================================
 */
void bmx_initialize() {
  Output("BMX:INIT");
  
  // 1st Bosch Sensor - Need to see which (BMP, BME, BM3) is plugged in
  BMX_1_chip_id = get_Bosch_ChipID(BMX_ADDRESS_1);
  switch (BMX_1_chip_id) {
    case BMP280_CHIP_ID :
      if (!bmp1.begin(BMX_ADDRESS_1)) { 
        msgp = (char *) "BMP1 ERR";
        BMX_1_exists = false;
        SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BMP280;
        msgp = (char *) "BMP1 OK";
        float p = bmp1.readPressure();
      }
    break;

    case BME280_BMP390_CHIP_ID :
      if (!bme1.begin(BMX_ADDRESS_1)) { 
        if (!bm31.begin_I2C(BMX_ADDRESS_1)) {  // Perhaps it is a BMP390
          msgp = (char *) "BMX1 ERR";
          BMX_1_exists = false;
          SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
        }
        else {
          BMX_1_exists = true;
          BMX_1_type = BMX_TYPE_BMP390;
          msgp = (char *) "BMP390_1 OK";
          float p = bm31.readPressure();        
        }      
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BME280;
        msgp = (char *) "BME280_1 OK";
        float p = bme1.readPressure();
      }
    break;

    case BMP388_CHIP_ID :
      if (!bm31.begin_I2C(BMX_ADDRESS_1)) { 
        msgp = (char *) "BM31 ERR";
        BMX_1_exists = false;
        SystemStatusBits |= SSB_BMX_1;  // Turn On Bit          
      }
      else {
        BMX_1_exists = true;
        BMX_1_type = BMX_TYPE_BMP388;
        msgp = (char *) "BM31 OK";
        float p = bm31.readPressure();
      }
    break;

    default:
      msgp = (char *) "BMX_1 NF";
    break;
  }
  Output (msgp);

  // 2nd Bosch Sensor - Need to see which (BMP, BME, BM3) is plugged in
  BMX_2_chip_id = get_Bosch_ChipID(BMX_ADDRESS_2);
  switch (BMX_2_chip_id) {
    case BMP280_CHIP_ID :
      if (!bmp1.begin(BMX_ADDRESS_2)) { 
        msgp = (char *) "BMP2 ERR";
        BMX_2_exists = false;
        SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BMP280;
        msgp = (char *) "BMP2 OK";
        float p = bmp2.readPressure();
      }
    break;

    case BME280_BMP390_CHIP_ID :
      if (!bme2.begin(BMX_ADDRESS_2)) { 
        if (!bm31.begin_I2C(BMX_ADDRESS_2)) {  // Perhaps it is a BMP390
          msgp = (char *) "BMX2 ERR";
          BMX_2_exists = false;
          SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
        }
        else {
          BMX_2_exists = true;
          BMX_2_type = BMX_TYPE_BMP390;
          msgp = (char *) "BMP390_2 OK";
          float p = bm32.readPressure();          
        }
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BME280;
        msgp = (char *) "BME280_2 OK";
        float p = bme2.readPressure();
      }
    break;

    case BMP388_CHIP_ID :
      if (!bm31.begin_I2C(BMX_ADDRESS_2)) { 
        msgp = (char *) "BM31 ERR";
        BMX_2_exists = false;
        SystemStatusBits |= SSB_BMX_2;  // Turn On Bit          
      }
      else {
        BMX_2_exists = true;
        BMX_2_type = BMX_TYPE_BMP388;
        msgp = (char *) "BM31 OK";
        float p = bm32.readPressure();
      }
    break;

    default:
      msgp = (char *) "BMX_2 NF";
    break;
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * htu21d_initialize() - HTU21D sensor initialize
 *=======================================================================================================================
 */
void htu21d_initialize() {
  Output("HTU21D:INIT");
  
  // HTU21DF Humidity & Temp Sensor (I2C ADDRESS = 0x40)
  if (!htu.begin()) {
    msgp = (char *) "HTU NF";
    HTU21DF_exists = false;
    SystemStatusBits |= SSB_HTU21DF;  // Turn On Bit
  }
  else {
    HTU21DF_exists = true;
    msgp = (char *) "HTU OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * mcp9808_initialize() - MCP9808 sensor initialize
 *=======================================================================================================================
 */
void mcp9808_initialize() {
  Output("MCP9808:INIT");
  
  // 1st MCP9808 Precision I2C Temperature Sensor (I2C ADDRESS = 0x18)
  mcp1 = Adafruit_MCP9808();
  if (!mcp1.begin(MCP_ADDRESS_1)) {
    msgp = (char *) "MCP1 NF";
    MCP_1_exists = false;
    SystemStatusBits |= SSB_MCP_1;  // Turn On Bit
  }
  else {
    MCP_1_exists = true;
    msgp = (char *) "MCP1 OK";
  }
  Output (msgp);

  // 2nd MCP9808 Precision I2C Temperature Sensor (I2C ADDRESS = 0x19)
  mcp2 = Adafruit_MCP9808();
  if (!mcp2.begin(MCP_ADDRESS_2)) {
    msgp = (char *) "MCP2 NF";
    MCP_2_exists = false;
    SystemStatusBits |= SSB_MCP_2;  // Turn On Bit
  }
  else {
    MCP_2_exists = true;
    msgp = (char *) "MCP2 OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * si1145_initialize() - SI1145 sensor initialize
 *=======================================================================================================================
 */
void si1145_initialize() {
  Output("SI1145:INIT");
  
  // SSB_SI1145 UV index & IR & Visible Sensor (I2C ADDRESS = 0x60)
  if (! uv.begin()) {
    Output ("SI:NF");
    SI1145_exists = false;
    SystemStatusBits |= SSB_SI1145;  // Turn On Bit
  }
  else {
    SI1145_exists = true;
    Output ("SI:OK");
    si_last_vis = uv.readVisible();
    si_last_ir = uv.readIR();
    si_last_uv = uv.readUV()/100.0;

    sprintf (msgbuf, "SI:VI[%d.%02d]", (int)si_last_vis, (int)(si_last_vis*100.0)%100); 
    Output (msgbuf);
    sprintf (msgbuf, "SI:IR[%d.%02d]", (int)si_last_ir, (int)(si_last_ir*100.0)%100); 
    Output (msgbuf);
    sprintf (msgbuf, "SI:UV[%d.%02d]", (int)si_last_uv, (int)(si_last_uv*100.0)%100); 
    Output (msgbuf);
  }
}

/* 
 *=======================================================================================================================
 * sht_initialize() - SHT31 sensor initialize
 *=======================================================================================================================
 */
void sht_initialize() {
  Output("SHT:INIT");
  
  // 1st SHT31 I2C Temperature/Humidity Sensor (I2C ADDRESS = 0x44)
  sht1 = Adafruit_SHT31();
  if (!sht1.begin(SHT_ADDRESS_1)) {
    msgp = (char *) "SHT1 NF";
    SHT_1_exists = false;
    SystemStatusBits |= SSB_SHT_1;  // Turn On Bit
  }
  else {
    SHT_1_exists = true;
    msgp = (char *) "SHT1 OK";
  }
  Output (msgp);

  // 2nd SHT31 I2C Temperature/Humidity Sensor (I2C ADDRESS = 0x45)
  sht2 = Adafruit_SHT31();
  if (!sht2.begin(SHT_ADDRESS_2)) {
    msgp = (char *) "SHT2 NF";
    SHT_2_exists = false;
    SystemStatusBits |= SSB_SHT_2;  // Turn On Bit
  }
  else {
    SHT_2_exists = true;
    msgp = (char *) "SHT2 OK";
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * hih8_initialize() - HIH8000 sensor initialize
 *=======================================================================================================================
 */
void hih8_initialize() {
  Output("HIH8:INIT");

  if (I2C_Device_Exist(HIH8000_ADDRESS)) {
    HIH8_exists = true;
    msgp = (char *) "HIH8 OK";
  }
  else {
    msgp = (char *) "HIH8 NF";
    HIH8_exists = false;
    SystemStatusBits |= SSB_HIH8;  // Turn On Bit
  }
  Output (msgp);
}

/* 
 *=======================================================================================================================
 * hih8_getTempHumid() - Get Temp and Humidity
 *   Call example:  status = hih8_getTempHumid(&t, &h);
 *=======================================================================================================================
 */
bool hih8_getTempHumid(float *t, float *h) {
  if (HIH8_exists) {
    uint16_t humidityBuffer    = 0;
    uint16_t temperatureBuffer = 0;
  
    Wire.begin();
    Wire.beginTransmission(HIH8000_ADDRESS);

    Wire.write(0x00); // set the register location for read request

    delayMicroseconds(200); // give some time for sensor to process request

    if (Wire.requestFrom(HIH8000_ADDRESS, 4) == 4) {

      // Get raw humidity data
      humidityBuffer = Wire.read();
      humidityBuffer <<= 8;
      humidityBuffer |= Wire.read();
      humidityBuffer &= 0x3FFF;   // 14bit value, get rid of the upper 2 status bits

      // Get raw temperature data
      temperatureBuffer = Wire.read();
      temperatureBuffer <<= 8;
      temperatureBuffer |= Wire.read();
      temperatureBuffer >>= 2;  // Remove the last two "Do Not Care" bits (shift left is same as divide by 4)

      Wire.endTransmission();

      *h = humidityBuffer * 6.10e-3;
      *t = temperatureBuffer * 1.007e-2 - 40.0;
      return (true);
    }
    else {
      Wire.endTransmission();
      return(false);
    }
  }
  else {
    return (false);
  }
}

/*
 * ======================================================================================================================
 * I2C_Check_Sensors() - See if each I2C sensor responds on the bus and take action accordingly             
 * ======================================================================================================================
 */
void I2C_Check_Sensors() {

  // BMX_1 Barometric Pressure 
  if (I2C_Device_Exist (BMX_ADDRESS_1)) {
    // Sensor online but our state had it offline
    if (BMX_1_exists == false) {
      if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
        if (bmp1.begin(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BMP1 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        } 
      }
      else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
        if (bme1.begin(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BME1 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        }          
      }
      else {
        if (bm31.begin_I2C(BMX_ADDRESS_1)) { 
          BMX_1_exists = true;
          Output ("BM31 ONLINE");
          SystemStatusBits &= ~SSB_BMX_1; // Turn Off Bit
        }                  
      }      
    }
  }
  else {
    // Sensor offline but our state has it online
    if (BMX_1_exists == true) {
      BMX_1_exists = false;
      Output ("BMX1 OFFLINE");
      SystemStatusBits |= SSB_BMX_1;  // Turn On Bit 
    }    
  }

  // BMX_2 Barometric Pressure 
  if (I2C_Device_Exist (BMX_ADDRESS_2)) {
    // Sensor online but our state had it offline
    if (BMX_2_exists == false) {
      if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
        if (bmp2.begin(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BMP2 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        } 
      }
      else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
        if (bme2.begin(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BME2 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        }          
      }
      else {
         if (bm32.begin_I2C(BMX_ADDRESS_2)) { 
          BMX_2_exists = true;
          Output ("BM32 ONLINE");
          SystemStatusBits &= ~SSB_BMX_2; // Turn Off Bit
        }                         
      }     
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (BMX_2_exists == true) {
      BMX_2_exists = false;
      Output ("BMX2 OFFLINE");
      SystemStatusBits |= SSB_BMX_2;  // Turn On Bit 
    }    
  }

  // HTU21DF Humidity & Temp Sensor
  if (I2C_Device_Exist (HTU21DF_I2CADDR)) {
    // Sensor online but our state had it offline
    if (HTU21DF_exists == false) {
      // See if we can bring sensor online
      if (htu.begin()) {
        HTU21DF_exists = true;
        Output ("HTU ONLINE");
        SystemStatusBits &= ~SSB_HTU21DF; // Turn Off Bit
      }
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (HTU21DF_exists == true) {
      HTU21DF_exists = false;
      Output ("HTU OFFLINE");
      SystemStatusBits |= SSB_HTU21DF;  // Turn On Bit
    }   
  }

#ifdef NOWAY    // Sensor fails to update if this code is enabled
  // MCP9808 Precision I2C Temperature Sensor
  if (I2C_Device_Exist (MCP_ADDRESS_1)) {
    // Sensor online but our state had it offline
    if (MCP_1_exists == false) {
      // See if we can bring sensor online
      if (mcp1.begin(MCP_ADDRESS_1)) {
        MCP_1_exists = true;
        Output ("MCP ONLINE");
        SystemStatusBits &= ~SSB_MCP_1; // Turn Off Bit
      }
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (MCP_1_exists == true) {
      MCP_1_exists = false;
      Output ("MCP OFFLINE");
      SystemStatusBits |= SSB_MCP_1;  // Turn On Bit
    }   
  }
#endif

  // SI1145 UV index & IR & Visible Sensor
  if (I2C_Device_Exist (SI1145_ADDR)) {
    // Sensor online but our state had it offline
    if (SI1145_exists == false) {
      // See if we can bring sensore online
      if (uv.begin()) {
        SI1145_exists = true;
        Output ("SI ONLINE");
        SystemStatusBits &= ~SSB_SI1145; // Turn Off Bit
      }
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (SI1145_exists == true) {
      SI1145_exists = false;
      Output ("SI OFFLINE");
      SystemStatusBits |= SSB_SI1145;  // Turn On Bit
    }   
  }


  // AS5600 Wind Direction
  if (I2C_Device_Exist (AS5600_ADR)) {
    // Sensor online but our state had it offline
    if (AS5600_exists == false) {
      AS5600_exists = true;
      Output ("WD ONLINE");
      SystemStatusBits &= ~SSB_AS5600; // Turn Off Bit
    }
  }
  else {
    // Sensor offline but we our state has it online
    if (AS5600_exists == true) {
      AS5600_exists = false;
      Output ("WD OFFLINE");
      SystemStatusBits |= SSB_AS5600;  // Turn On Bit
    }   
  }
}

/* 
 *=======================================================================================================================
 * DS_Median()
 *=======================================================================================================================
 */
float DS_Median() {
  int i;

  for (i=0; i<DS_BUCKETS; i++) {
    // delay(500);
    delay(250);
    ds_buckets[i] = (int) analogRead(DS_PIN);
    // sprintf (Buffer32Bytes, "DS[%02d]:%d", i, ds_buckets[i]);
    // OutputNS (Buffer32Bytes);
  }
  
  mysort(ds_buckets, DS_BUCKETS);
  i = (DS_BUCKETS+1) / 2 - 1; // -1 as array indexing in C starts from 0

  if (cf_ds_type) {  // 0 = 5m, 1 = 10m
    return (ds_buckets[i]*10);
  }
  else {
    return (ds_buckets[i]*5);
  }
}

/*
 * ======================================================================================================================
 * OBS_Do() - Collect Observations, Build message, Send to logging site
 * ======================================================================================================================
 */
void OBS_Do (bool log_obs) {
  float batt = 0.0;
  int msgLength;

  // Safty Check for Vaild Time
  if (!RTC_valid) {
    Output ("OBS_Do:Time NV");
    return;
  }
  
  // Build JSON log entry by hand  
  // {"at":"2021-03-05T11:43:59","sg":49,"bp1":3,"bt1":97.875,"bh1":40.20,"bv":3.5,"hth":9 .... }

  sprintf (msgbuf, "{\"at\":\"%s\",", timestamp);

  if (cf_raingauge_enable) {
    float rain = 0.0;
    unsigned long rgds;    // rain gauge delta seconds, seconds since last rain gauge observation logged

    rgds = (millis()-raingauge_interrupt_stime)/1000;
    rain = raingauge_interrupt_count * 0.2;  // Rain Gauge - Each tip is 0.2mm of rain
    raingauge_interrupt_count = 0;
    raingauge_interrupt_stime = lastOBS;
    raingauge_interrupt_ltime = 0;           // used to debounce the tip

    sprintf (msgbuf+strlen(msgbuf), "\"rg\":%u.%02d,", (int)rain, (int)(rain*100)%100);
  }

  if (cf_anemometer_enable) {
    Wind_GustUpdate();                       // Update Gust and Gust Direction readings
    
    float ws = Wind_SpeedAverage();
    int   wd = Wind_DirectionVector();
    float wg = Wind_Gust();
    int   wgd = Wind_GustDirection();
    
    sprintf (msgbuf+strlen(msgbuf), "\"ws\":%u.%02d,\"wd\":%d,\"wg\":%u.%02d,\"wgd\":%d,", 
      (int)ws, (int)(ws*100)%100, wd, (int)wg, (int)(wd*100)%100, wgd);
  }

  if (cf_ds_enable) {
    float ds_median, ds_median_raw;

    ds_median = ds_median_raw = DS_Median();
    if (cf_ds_baseline > 0) {
      ds_median = cf_ds_baseline - ds_median_raw;
    }
    sprintf (msgbuf+strlen(msgbuf), "\"ds\":%d.%02d,\"dsr\":%d.%02d,", 
      (int)ds_median, (int)(ds_median*100)%100,
      (int)ds_median_raw, (int)(ds_median_raw*100)%100);
  }

  //
  // Add I2C Sensors
  //
  if (BMX_1_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_1_chip_id == BMP280_CHIP_ID) {
      p = bmp1.readPressure()/100.0F;       // bp1 hPa
      t = bmp1.readTemperature();           // bt1
    }
    else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_1_type == BMX_TYPE_BME280) {
        p = bme1.readPressure()/100.0F;     // bp1 hPa
        t = bme1.readTemperature();         // bt1
        h = bme1.readHumidity();            // bh1
      }
      if (BMX_1_type == BMX_TYPE_BMP390) {
        p = bm31.readPressure()/100.0F;     // bp1 hPa
        t = bm31.readTemperature();         // bt1       
      }
    }
    else { // BMP388
      p = bm31.readPressure()/100.0F;       // bp1 hPa
      t = bm31.readTemperature();           // bt1
    }
    p = (isnan(p)) ? 0.0 : p;
    t = (isnan(t)) ? -999.99 : t;
    h = (isnan(h)) ? 0.0 : h;

    sprintf (msgbuf+strlen(msgbuf), "\"bp1\":%u.%02d,\"bt1\":%d.%02d,\"bh1\":%d.%02d,",
      (int)p, (int)(p*100)%100,
      (int)t, (int)(t*100)%100,
      (int)h, (int)(h*100)%100);
  }

  if (BMX_2_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_2_chip_id == BMP280_CHIP_ID) {
      p = bmp2.readPressure()/100.0F;       // bp2 hPa
      t = bmp2.readTemperature();           // bt2
    }
    else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_2_type == BMX_TYPE_BME280) {
        p = bme2.readPressure()/100.0F;     // bp2 hPa
        t = bme2.readTemperature();         // bt2
        h = bme2.readHumidity();            // bh2 
      }
      if (BMX_2_type == BMX_TYPE_BMP390) {
        p = bm32.readPressure()/100.0F;       // bp2 hPa
        t = bm32.readTemperature();           // bt2
      }      
    }
    else { // BMP388
      p = bm32.readPressure()/100.0F;       // bp2 hPa
      t = bm32.readTemperature();           // bt2
    }
    p = (isnan(p)) ? 0.0 : p;
    t = (isnan(t)) ? -999.99 : t;
    h = (isnan(h)) ? 0.0 : h;

    sprintf (msgbuf+strlen(msgbuf), "\"bp2\":%u.%02d,\"bt2\":%d.%02d,\"bh2\":%d.%02d,",
      (int)p, (int)(p*100)%100,
      (int)t, (int)(t*100)%100,
      (int)h, (int)(h*100)%100);
  }

  if (HTU21DF_exists) {
    float t = 0.0;
    float h = 0.0;
    
    h = htu.readHumidity();
    h = (isnan(h)) ? 0.0 : h;
    
    t = htu.readTemperature();
    t = (isnan(t)) ? -999.99 : t;

    sprintf (msgbuf+strlen(msgbuf), "\"hh1\":%u.%02d,\"ht1\":%d.%02d,",
      (int)h, (int)(h*100)%100,
      (int)t, (int)(t*100)%100);
  }

  if (SI1145_exists) {
    float si_vis = uv.readVisible();
    float si_ir = uv.readIR();
    float si_uv = uv.readUV()/100.0;

    // Additional code to force sensor online if we are getting 0.0s back.
    if ( ((si_vis+si_ir+si_uv) == 0.0) && ((si_last_vis+si_last_ir+si_last_uv) != 0.0) ) {
      // Let Reset The SI1145 and try again
      Output ("SI RESET");
      if (uv.begin()) {
        SI1145_exists = true;
        Output ("SI ONLINE");
        SystemStatusBits &= ~SSB_SI1145; // Turn Off Bit

        si_vis = uv.readVisible();
        si_ir = uv.readIR();
        si_uv = uv.readUV()/100.0;
      }
      else {
        SI1145_exists = false;
        Output ("SI OFFLINE");
        SystemStatusBits |= SSB_SI1145;  // Turn On Bit
      }
    }

    sprintf (msgbuf+strlen(msgbuf), "\"sv1\":%u.%02d,\"si1\":%d.%02d,\"su1\":%d.%02d,",
      (int)si_vis, (int)(si_vis*100)%100,
      (int)si_ir, (int)(si_ir*100)%100,
      (int)si_uv, (int)(si_uv*100)%100);

    // Save current readings for next loop around compare
    si_last_vis = si_vis;
    si_last_ir = si_ir;
    si_last_uv = si_uv;
  }

  if (MCP_1_exists) {
    float t = 0.0;
    
    t = mcp1.readTempC();
    t = (isnan(t)) ? -999.99 : t;

    sprintf (msgbuf+strlen(msgbuf), "\"mt1\":%u.%02d,",
      (int)t, (int)(t*100)%100);
  }

  if (MCP_2_exists) {
    float t = 0.0;
   
    t = mcp2.readTempC();
    t = (isnan(t)) ? -999.99 : t;
    sprintf (msgbuf+strlen(msgbuf), "\"mt2\":%u.%02d,",
      (int)t, (int)(t*100)%100);
  }

  if (SHT_1_exists) {
    float t = 0.0;
    float h = 0.0;

    t = sht1.readTemperature();
    t = (isnan(t)) ? -999.99 : t;
    h = sht1.readHumidity();
    h = (isnan(h)) ? 0.0 : h;
    
    sprintf (msgbuf+strlen(msgbuf), "\"sht1\":%u.%02d,\"shh1\":%d.%02d,",
      (int)t, (int)(t*100)%100,
      (int)h, (int)(h*100)%100);
  }

  if (SHT_2_exists) {
    float t = 0.0;
    float h = 0.0;

    t = sht2.readTemperature();
    t = (isnan(t)) ? -999.99 : t;
    h = sht2.readHumidity();
    h = (isnan(h)) ? 0.0 : h;
    
    sprintf (msgbuf+strlen(msgbuf), "\"sht2\":%u.%02d,\"shh2\":%d.%02d,",
      (int)t, (int)(t*100)%100,
      (int)h, (int)(h*100)%100);
  }

  if (HIH8_exists) {
    float t = 0.0;
    float h = 0.0;
    bool status = hih8_getTempHumid(&t, &h);
    if (!status) {
      t = -999.99;
      h = 0.0;
    }
    t = (isnan(t)) ? -999.99 : t;
    h = (isnan(h)) ? 0.0 : h;

    sprintf (msgbuf+strlen(msgbuf), "\"ht2\":%u.%02d,\"ht2\":%d.%02d,",
      (int)t, (int)(t*100)%100,
      (int)h, (int)(h*100)%100);
  }

  batt = vbat_get();
  sprintf (msgbuf+strlen(msgbuf), "\"bv\":%d.%02d,\"hth\":%d}", 
    (int)batt, (int)(batt*100)%100, SystemStatusBits);
    
  // Display Time of OBS
  if (log_obs) {
    Output(timestamp);
    SD_LogObservation(msgbuf);  // Log Observation to SD Card
  }
  Serial_writeln (msgbuf);
}

/*
 * ======================================================================================================================
 * JPO_ClearBits() - Clear System Status Bits related to initialization
 * ======================================================================================================================
 */
void JPO_ClearBits() {
  if (JustPoweredOn) {
    JustPoweredOn = false;
    SystemStatusBits &= ~SSB_PWRON;   // Turn Off Power On Bit
    SystemStatusBits &= ~SSB_OLED;    // Turn Off OLED Not Found Bit
    SystemStatusBits &= ~SSB_LORA;    // Turn Off LoRa Not Found Bit
    SystemStatusBits &= ~SSB_BMX_1;   // Turn Off BMX_1 Not Found Bit
    SystemStatusBits &= ~SSB_BMX_2;   // Turn Off BMX_2 Not Found Bit
    SystemStatusBits &= ~SSB_HTU21DF; // Turn Off HTU Not Found Bit
    SystemStatusBits &= ~SSB_SI1145;  // Turn Off SI Not Found Bit
    SystemStatusBits &= ~SSB_MCP_1;   // Turn Off MCP_1 Not Found Bit
    SystemStatusBits &= ~SSB_MCP_2;   // Turn Off MCP_2 Not Found Bit
    SystemStatusBits &= ~SSB_SHT_1;   // Turn Off SHT_1 Not Found Bit
    SystemStatusBits &= ~SSB_SHT_2;   // Turn Off SHT_2 Not Found Bit
    SystemStatusBits &= ~SSB_HIH8;    // Turn Off HIH Not Found Bit
  }
}

/*
 * ======================================================================================================================
 * StationMonitor() - On OLED display station information
 * ======================================================================================================================
 */
void StationMonitor() {
  int r, c, len;
  
  float bmx_pressure = 0.0;
  float bmx_temp = 0.0;
  float bmx_humid = 0.0;
  float htu_humid = 0.0;
  float htu_temp;
  float mcp_temp = 0.0;
  float si_vis = 0.0;
  float si_ir = 0.0;
  float si_uv = 0.0;
  char Buffer16Bytes[16];

  float batt = vbat_get();

  OLED_ClearDisplayBuffer();

  // =================================================================
  // Line 0 of OLED
  // =================================================================
  rtc_timestamp();
  len = (strlen (timestamp) > 21) ? 21 : strlen (timestamp);
  for (c=0; c<=len; c++) oled_lines [0][c] = *(timestamp+c);
  Serial_writeln (timestamp);

  // =================================================================
  // Line 1 of OLED
  // =================================================================
  if (AS5600_exists) {
    sprintf (Buffer16Bytes, "D:%3d", Wind_SampleDirection());
  }
  else {
    sprintf (Buffer16Bytes, "D:NF ");
  }

  sprintf (Buffer32Bytes, "R:%02d,W:%02d,%s %X",
    raingauge_interrupt_count,
    anemometer_interrupt_count,
    Buffer16Bytes,
    SystemStatusBits);
  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [1][c] = *(Buffer32Bytes+c);
  Serial_writeln (Buffer32Bytes);

  // =================================================================
  // Line 2 of OLED
  // =================================================================
  if (BMX_1_exists) {
    switch (BMX_1_chip_id) {
       case BMP280_CHIP_ID :
         bmx_pressure = bmp1.readPressure()/100.0F;           // bmxp1
         bmx_temp = bmp1.readTemperature();                   // bmxt1
       break;
       case BME280_BMP390_CHIP_ID :
         if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
           bmx_pressure = bme1.readPressure()/100.0F;           // bmxp1
           bmx_temp = bme1.readTemperature();                   // bmxt1
           bmx_humid = bme1.readHumidity();                     // bmxh1 
         }
         else { // BMP390
           bmx_pressure = bm31.readPressure()/100.0F;
           bmx_temp = bm31.readTemperature();
         }
       break;
       case BMP388_CHIP_ID :
          bmx_pressure = bm31.readPressure()/100.0F;
          bmx_temp = bm31.readTemperature();
       break;
       default:
          Output ("BMX:WTF"); // This should not happen.
       break;
    }
    sprintf (Buffer32Bytes, "P:%d.%02d T:%d.%02d", 
      (int)bmx_pressure, (int)(bmx_pressure*100)%100,
      (int)bmx_temp, (int)(bmx_temp*100)%100);
  }
  else {
    sprintf (Buffer32Bytes, "BMX:NF");
  }
  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [2][c] = *(Buffer32Bytes+c);
  Serial_writeln (Buffer32Bytes);

  // =================================================================
  // Line 3 of OLED
  // =================================================================
  if (MCP_1_exists) {
    // mcp.shutdown_wake(0);        // wake up, ready to read! - power consumption ~200 micro Ampere
    mcp_temp = mcp1.readTempC();   
    // mcp.shutdown_wake(1);        // shutdown MCP9808 - power consumption ~0.1 mikro Ampere

    sprintf (Buffer16Bytes, "T%d.%02d", (int)mcp_temp, (int)(mcp_temp*100)%100);
  }
  else {
    sprintf (Buffer16Bytes, "MCP:NF");
  }

  if (HTU21DF_exists) {
    htu_humid = htu.readHumidity();
    htu_temp = htu.readTemperature();

    sprintf (Buffer32Bytes, "H:%02d.%02d%s", 
      (int)htu_humid, (int)(htu_humid*100)%100, Buffer16Bytes);
  }
  else {
    sprintf (Buffer32Bytes, "HTU:NF %s", Buffer16Bytes);
  }


  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [3][c] = *(Buffer32Bytes+c);
  Serial_writeln (Buffer32Bytes);


  OLED_update();
}

/*
 * ======================================================================================================================
 * BackGroundWork() - Take Sensor Reading, Do other work, Result should be 1 Second Delay for timming loop          
 * ======================================================================================================================
 */
void BackGroundWork() {
  Wind_TakeReading();
  delay (1000);
}


/*
 * =======================================================================================================================
 * setup()
 * =======================================================================================================================
 */
void setup() 
{
  // Put initialization like pinMode and begin functions here.
  pinMode (LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // serial console enable pin
  pinMode(SCE_PIN, INPUT_PULLUP);   // Internal pullup resistor biases the pin to supply voltage.
                                    // If jumper set to ground, we enable serial console (low = enable)
  if (digitalRead(SCE_PIN) == LOW) {
    SerialConsoleEnabled = true;
  }

  if (DisplayEnabled) {
    if (I2C_Device_Exist (OLED_I2C_ADDRESS)) {
      display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
      display.clearDisplay();
      display.setTextSize(1); // Draw 2X-scale text
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      for (int r=0; r<DISPLAY_LINES; r++) {
        oled_lines[r][0]=0;
      }
      OLED_write("OLED:OK");
    }
    else {
      DisplayEnabled = false;
      SystemStatusBits |= SSB_OLED; // Turn on Bit
    }
  }

  if (SerialConsoleEnabled) {
    Serial.begin(9600);

    // Wait for serial port to be available - Uncomment for testing
#if W4SC
    if (!Serial) {
      OLED_write("Waiting Serial Console");
    }
    while(!Serial) {
      Blink(1, 1000);
    }
#endif

    Output (VERSION_INFO);
    delay (5000);      // Pause so user can see version if not waiting for serial

    if (DisplayEnabled) {
      Serial_writeln ("OLED:Enabled");
    }
    else {
      Serial_writeln ("OLED:Disabled");
    }
    Output ("SC:Enabled");
  }

  Output (VERSION_INFO); // Doing it one more time for the OLED

  // Initialize SD card if we have one.
  SD_initialize();

  // Read RTC and set system clock if RTC clock valid
  rtc_initialize();

  if (RTC_valid) {
    Output("RTC:Valid");
  }
  else {
    Output("RTC:Not Valid");
  }

  rtc_timestamp();
  sprintf (msgbuf, "RTC:%s", timestamp);
  Output(msgbuf);
  delay (2000);

  if (cf_raingauge_enable) {
    // Optipolar Hall Effect Sensor SS451A - Rain Gauge
    raingauge_interrupt_count = 0;
    raingauge_interrupt_stime = millis();
    raingauge_interrupt_ltime = 0;  // used to debounce the tip
    attachInterrupt(RAINGAUGE_IRQ_PIN, raingauge_interrupt_handler, FALLING);
    Output("RG:ENABLED");
  }
  
  if (cf_anemometer_enable) {
    // Optipolar Hall Effect Sensor SS451A - Wind Speed
    anemometer_interrupt_count = 0;
    anemometer_interrupt_stime = millis();
    attachInterrupt(ANEMOMETER_IRQ_PIN, anemometer_interrupt_handler, FALLING);
    Output("WS:ENABLED");
  }

  // Set up distance sensor pin for reading
  if (cf_ds_enable) {
    pinMode(DS_PIN, INPUT);
    Output("DS:ENABLED");
  }

  // I2C Sensor Init
  as5600_initialize();
  bmx_initialize();
  htu21d_initialize();
  mcp9808_initialize();
  sht_initialize();
  hih8_initialize();

  // Skip Wind Init if RTC not valid so user can set RTC
  if (cf_anemometer_enable && RTC_valid) {
    Wind_Initiailize();
  }
}

/*
 * =======================================================================================================================
 * loop()
 * =======================================================================================================================
 */
void loop()
{
  BackGroundWork();
  
  // RTC not set, Get Time for User
  if (!RTC_valid) {
    static bool first = true;
      
    if (first) {
      if (digitalRead(SCE_PIN) != LOW) {
        Serial.begin(9600);
        SerialConsoleEnabled = true;
      }  
    
      Output("SET RTC ENTER:");
      Output("YYYY:MM:DD:HH:MM:SS");
      first = false;
    }
    
    if (rtc_readserial()) { // check for serial input, validate for rtc, set rtc, report result
      Output("!!!!!!!!!!!!!!!!!!!");
      Output("!!! Press Reset !!!");
      Output("!!!!!!!!!!!!!!!!!!!");

      while (true) {
        delay (1000);
      }
    }
  }

  //Calibration mode, You can also reset the RTC here
  else if (countdown && digitalRead(SCE_PIN) == LOW) { 
    // Every minute, Do observation (don't save to SD) and transmit - So we can test LoRa
    I2C_Check_Sensors();

    if ( (countdown%60) == 0) { 
      OBS_Do(false); // Do not log observation
      sprintf (Buffer32Bytes, "NO:%ds", countdown%60);
      Output (Buffer32Bytes);
    }
    
    StationMonitor();
    
    // check for input sting, validate for rtc, set rtc, report result
    if (Serial.available() > 0) {
      rtc_readserial(); // check for serial input, validate for rtc, set rtc, report result
    }
    
    countdown--;
  }

  else { // Normal Operation - Main Work
    rtc_timestamp(); //get the current date-time set "now" and "timestamp"
    
    // This will be invalid if the RTC was bad at poweron and we have not connected to Cell network
    // Upon connection to cell network system time is set and this becomes valid.
    if (RTC_valid) {  
 
      // Perform an Observation, save in OBS structure, Write to SD
      if ( (now.unixtime() - lastOBS) >= OBSERVATION_INTERVAL) {  // 1 minute minus 1 second    
        I2C_Check_Sensors();
        lastOBS = now.unixtime(); // Update time we took observations.
        OBS_Do(true);

        // Shutoff System Status Bits related to initialization after we have logged first observation
        JPO_ClearBits();
      }
    }
    
  }
}
