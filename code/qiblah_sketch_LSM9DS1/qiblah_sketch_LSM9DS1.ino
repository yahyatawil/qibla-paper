/*
  Qibla finder - Open-source qibla finder with tilt compensation using 9-DoF IMU and GPS

  The circuit:
  - Arduino board
  - BMI270 shuttle board
  - Adafruit Mini GPS PA1010D
  - Monochrome 0.91" 128x32 I2C OLED Display

  Documentation:
  - Arabic: https://atadiat.com/ar/open-source-qibla-compass-with-tilt-compensation/.
  - English: https://atadiat.com/en/e-open-source-qibla-compass-with-tilt-compensation/.

  References:
  - https://atadiat.com/en/e-towards-understanding-imu-basics-of-accelerometer-and-gyroscope-sensors/
  - https://atadiat.com/en/e-magnetometer-soft-iron-and-hard-iron-calibration-why-how/
  - https://atadiat.com/en/e-towards-understanding-imu-frames-vpython-visualize-orientation/

  created 14 May 2023
  by Yahya Tawil
*/

#include <ArduinoBLE.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include "Arduino_LSM9DS1.h"
#include "logos.h"
#include "math.h"
#include "mbed.h"
#include "mbed_events.h"
#include "MadgwickAHRS.h"
#include "Wire.h"
#include "FlashIAPBlockDevice.h"

constexpr int kFlashBlockSize = 4096;
#define ROUND_UP(val, block_size) ((((val) + ((block_size) - 1)) / (block_size)) * (block_size))
constexpr int kFlashBufferSize = ROUND_UP(64 * 1024, kFlashBlockSize);
alignas(kFlashBlockSize) const uint8_t flash_buffer[kFlashBufferSize] = {};

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";


// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
/**********************/

// Macros define program behaviour (disagned to enable one at once)
// print the pitch,roll, and yaw beside the compass values
// before and after tilt compensation
#define COMPASS_LOG 0
#define MAG_CALIBRATION_LOG 0
// print on the console the magnetometer values after applying the calibration arrays
#define MAG_CALIBRATED_LOG 0
// print the IMU data in the format needed for calibration software tool
#define MOTION_CAL_SOFT 0
// print GYRO output for calibration purposes
#define GYRO_CAL 0
// send pitch,roll, and yaw only. This is for 3D visualization
// https://atadiat.com/en/e-towards-understanding-imu-frames-vpython-visualize-orientation/
#define VISUAL_3D 0
// Set to 1 if GPS reciver is used
#define GPS_EN 0
/**********************/

// Mag. calibration
// Hard ironing
//B
#define B1 25.48
#define B2 10.12
#define B3 -11.51

//Soft ironing
//H column1
#define H11 1.009
#define H21 0.033
#define H31 -0.003
//H column2
#define H12 0.033
#define H22 0.970
#define H32 0.008
//H column3
#define H13 -0.003
#define H23 0.008
#define H33 1.023
/**********************/

// if GPS is not used go and grab your lat. and long. from:
//https://www.latlong.net/
//double YOUR_LAT = 37.066666; // gazi
//double YOUR_LONG   = 37.383331; // gazi
double YOUR_LAT = 39.925533; // ankara
double YOUR_LONG   = 32.866287; // ankara



// Makkah's lat. and long. in decimal degree
const double MAKKAH_LAT = 21.422510;
const double MAKKAH_LONG = 39.826168;
/**********************/

// qibla tolerance margin
#define TOLERANCE_MARGINE 5
/*********************/

// Declination
// get it from here: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
#define DECLINATION 5.7
/**********************/

//Dimensions for the screen
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define LOGO_WIDTH 32
#define LOGO_HEIGHT 32
/**********************/

#define LATLONG_SETTING_FLAG 1<<0UL
#define QIBLA_SETTING_FLAG 1<<1UL
#define MAT_SETTING_FLAG 1<<2UL

// Buffer used for screen log with sprintf
char log_buffer[128];
/**********************/

// OLED display
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
/**********************/



// Global variables

BLEService SettingsService(deviceServiceUuid);
//BLEByteCharacteristic SettingsCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite);
BLEStringCharacteristic SettingsCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite, 50 );

events::EventQueue eventqueue;
rtos::Thread t;

// form the calibration arrayes
float B[3] = {B1, B2, B3}; // soft
float H[3][3] = {{H11, H12, H13}, {H21, H22, H23}, {H31, H32, H33}}; // hard

float mag_x;
float mag_y;
float mag_z;

int newHeading = 0;
int newHeading2 = 0;

float theta_acc_new;
float phi_acc_new;

//pitch, roll and yaw angles calculated using gyro
float theta_gyro = 0.0;
float phi_gyro = 0.0;
float yaw_gyro = 0.0;

struct bmi2_sens_axes_data
{
  /*! Data in x-axis */
  float x;

  /*! Data in y-axis */
  float y;

  /*! Data in z-axis */
  float z;

};

struct bmi2_sens_data
{
  /*! Accelerometer axes data */
  struct bmi2_sens_axes_data acc;

  /*! Gyroscope axes data */
  struct bmi2_sens_axes_data gyr;
};

/* Create an instance of sensor data structure */
struct bmi2_sens_data sensor_data = { { 0 } };
struct bmi2_sens_axes_data mag_data;

Madgwick orientation_filter;
int Madjwick_p; // pitch calculated by Madgwick filter
int Madjwick_r; // roll calculated by Madgwick filter
int Madjwick_y; // yaw calculated by Madgwick filter

float pitch;
float roll;

int qiblah;

bool update_screen_flag = false;

uint32_t Gpstimer = millis();


// Calculate the qibla angles based on you lat. (la_L) and long (lo_L)
int qiblahFinder(float la_L, float lo_L, char LAT = 'X') {

  bool IsNoth = (la_L > 0.0 || LAT == 'N') ? true : false;

  char log_msg[128];

  sprintf(log_msg, "Find qiblah for location %f,%f \r\n", la_L, lo_L);
  Serial.print(log_msg);
  sprintf(log_msg, "North? %d \r\n", IsNoth);
  Serial.print(log_msg);

  float a = sin((lo_L - MAKKAH_LONG) * PI / 180);
  float b = cos(la_L * PI / 180) * tan(MAKKAH_LAT * PI / 180) - sin(la_L * PI / 180) * cos((la_L - MAKKAH_LAT) * PI / 180);

  sprintf(log_msg, "A/B: %f %f \r\n", a, b);
  Serial.print(log_msg);

  int Q = atan(a / b) * 180 / PI;
  sprintf(log_msg, "Q: %d \r\n", Q);
  Serial.print(log_msg);
  if (la_L < MAKKAH_LAT && lo_L > MAKKAH_LONG) {
    Q = IsNoth ? (360 - Q) : (180 - Q);
  }
  else if (la_L < MAKKAH_LAT && lo_L < MAKKAH_LONG) {
    Q = IsNoth ? Q : (180 + Q);
  }
  else if (la_L > MAKKAH_LAT && lo_L < MAKKAH_LONG) {
    Q = IsNoth ? (180 - Q) : (360 - Q);
  }
  else if (la_L > MAKKAH_LAT && lo_L > MAKKAH_LONG) {
    Q = IsNoth ? (180 + Q) : Q;
  }
  sprintf(log_msg, "Qiblah: %d \r\n", Q);
  Serial.print(log_msg);
  return Q;
}


void setupGPS() {
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);// Request updates on antenna status, comment out to keep quiet
  delay(1000);
  GPS.println(PMTK_Q_RELEASE); // Ask for firmware version
}

void setupDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay(); // Clear the display buffer
  display.display(); // Show the display buffer on the screen

  display.setTextSize(1);
  //display.setFont(&FreeMonoBoldOblique12pt7b);
  display.setTextColor(SSD1306_WHITE);
}

void setupBLE() {
  if (!BLE.begin()) {

    Serial.println("- Starting Bluetooth® Low Energy module failed!");

    while (1);

  }
  BLE.setLocalName("Qibla Finder");

  BLE.setAdvertisedService(SettingsService);
  SettingsService.addCharacteristic(SettingsCharacteristic);
  BLE.addService(SettingsService);
  SettingsCharacteristic.writeValue(" ");
  BLE.advertise();
  Serial.println("Starting Bluetooth® Low Energy module");
  t.start(mbed::callback(&eventqueue, &events::EventQueue::dispatch_forever));
  eventqueue.call_every(1000, CheckConnection);
  eventqueue.call_every(100, topBarUpdate);
  Serial.println("Qibla Finder");
}

void CheckConnection() {
  BLEDevice central = BLE.central();

  //Serial.println("- Discovering central device...");

  delay(500);


  if (central) {

    Serial.println("* Connected to central device!");

    Serial.print("* Device MAC address: ");

    Serial.println(central.address());

    Serial.println(" ");

    while (central.connected()) {
      if (SettingsCharacteristic.written()) {
        String settings = SettingsCharacteristic.value();
        String Lat;
        String Long;
        String q;
        Serial.println(settings);
        // LatLong:Latitude,Longitude .i.e:(LatLong:39.933365,32.859741)
        if (settings.indexOf("reset") != -1)
        {
          store_settings(0);
        }
        else if (settings.indexOf("check") != -1)
        {
          check_stored_settings();
        }
        else if (settings.length() == 27 && settings.indexOf(',') != -1 && settings.indexOf("LatLong:") != -1)
        {
          Lat = settings.substring(8, 17);
          Long = settings.substring(18, 27);
          YOUR_LAT = Lat.toFloat();
          YOUR_LONG = Long.toFloat() ;
          Serial.println(YOUR_LAT,6);
          Serial.println(YOUR_LONG,6);
          store_settings(LATLONG_SETTING_FLAG);
          // write to flash
          // https://github.com/petewarden/arduino_nano_ble_write_flash/blob/main/arduino_nano_ble_write_flash.ino
        }
        // (q:) .i.e:(q:172)
        else if (settings.length() == 5 && settings.indexOf("q:") != -1)
        {
          q = settings.substring(2, 5);
          Serial.println(q.toFloat());
          qiblah = q.toFloat();
          store_settings(QIBLA_SETTING_FLAG);
        }
        // (HardI:H11,H21,H31) .i.e:(HardI:47.63,15.11,-6.80)
        else if (settings.indexOf("HardI:") != -1)
        {
          int comma_position = -1;
          int old_comma_position = 5;
          float stored_HardIron [3];
          for (int comma_i = 0; comma_i < 3; comma_i++)
          {
            comma_position = settings.indexOf(',', comma_position + 1);
            String HardIron ;
            if (comma_i == 2)
            {
              HardIron = settings.substring(old_comma_position + 1, settings.length());
            }
            else
            {
              HardIron = settings.substring(old_comma_position + 1, comma_position);
            }

            old_comma_position = comma_position;
            stored_HardIron[comma_i] = HardIron.toFloat();
            B[comma_i] = HardIron.toFloat();
            Serial.println(HardIron.toFloat());
            store_settings(MAT_SETTING_FLAG);
          }
        }
        // (SoftI:S11,S12,S13,S21,S22,S23,S31,S32,S33) .i.e:(SoftI:0.971,0.051,0.005,0.051,0.005,0.021,1.043)
        else if (settings.indexOf("SoftI:") != -1)
        {
          int comma_position = -1;
          int old_comma_position = 5;
          float stored_SoftIron [3][3];
          String SoftIron ;
          for (int comma_j = 0; comma_j < 3; comma_j++)
          {
            for (int comma_i = 0; comma_i < 3; comma_i++)
            {
              comma_position = settings.indexOf(',', comma_position + 1);

              if (comma_i == 2 && comma_j == 2) {
                SoftIron = settings.substring(old_comma_position + 1, settings.length());
              }
              else
              {
                SoftIron = settings.substring(old_comma_position + 1, comma_position );
              }
              H[comma_j][comma_i] = SoftIron.toFloat();
              Serial.println(SoftIron);
              old_comma_position = comma_position;
              stored_SoftIron[comma_j][comma_i] = SoftIron.toFloat();
              Serial.println(SoftIron.toFloat());
            }
          }
          store_settings(MAT_SETTING_FLAG);
        }

      }
    }
    Serial.println("* Disconnected to central device!");
  }
}

struct screen_info_t {

  bool ble_connected;
  bool qiblah_reached;
  float batt_voltage;
  int heading;
  bool arrow_right_direction;

};

struct screen_info_t screen_info = {
  .ble_connected = false,
  .qiblah_reached = false,
  .batt_voltage = 0.0,
  .heading = 0,
  .arrow_right_direction = 0,
};


#define SAMPLES_PER_SECOND 2
#define ADC_BUFFER_SIZE     1
#define PPI_CHANNEL         (7)
volatile nrf_saadc_value_t adcBuffer[ADC_BUFFER_SIZE];
volatile bool adcFlag = false;

void topBarUpdate()
{
  BLEDevice central = BLE.central();
  //        display.setCursor(0, 40);
  //        sprintf(log_buffer, "vdd:%f", (adcBuffer[0]*0.6/4095)*2*5);
  //        display.println(log_buffer);
  //        display.display();      // Show initial text

  if ( adcFlag )
  {
    adcFlag = false;
    float batt_volt = (adcBuffer[0] * 0.6 / 4095) * 2 * 5 ;
    screen_info.batt_voltage = batt_volt;
    //Serial.print( "batt: " );
    //Serial.println(screen_info.batt_voltage );

  }
  if (central.connected())
  {
    screen_info.ble_connected = true;
  }
  else
  {
    screen_info.ble_connected = false;
  }
  update_screen_flag = true;
}

void update_screen() {

  display.clearDisplay(); // Clear the display buffer

  //  display.display(); // Show the display buffer on the screen


  if (screen_info.batt_voltage > 3.9)
  {
    display.drawBitmap(0, 0, batt_100_logo, 32, 16, SSD1306_WHITE);
    //    display.display(); // Show the display buffer on the screen
  }
  else if (screen_info.batt_voltage > 3.6 && screen_info.batt_voltage < 3.9)
  {
    //
    display.drawBitmap(0, 0, batt_75_logo, 32, 16, SSD1306_WHITE);
    //    display.display(); // Show the display buffer on the screen
  }
  //
  else if (screen_info.batt_voltage > 3.0 && screen_info.batt_voltage < 3.6)
  {
    display.drawBitmap(0, 0, batt_25_logo, 32, 16, SSD1306_WHITE);
    //    display.display(); // Show the display buffer on the screen
  }
  //
  else {
    display.drawBitmap(0, 0, batt_0_logo, 32, 16, SSD1306_WHITE);
    //    display.display(); // Show the display buffer on the screen
  }

  display.setCursor(33, 2);
  sprintf(log_buffer, "q:%d", qiblah);
  display.println(log_buffer);



  if (screen_info.ble_connected)
  {
    display.drawBitmap(112, 0, bluetooth_logo, 16, 16, SSD1306_WHITE);
    //    display.display(); // Show the display buffer on the screen
  }
  if (screen_info.qiblah_reached)
  {
    print_qiblah_on_screen(false);
  }
  else
  {
    print_on_screen(newHeading2);

    if (screen_info.arrow_right_direction )
    {
      display.drawTriangle(98, 64, 98, 44, 118, 54, SSD1306_WHITE);
      display.fillTriangle(98, 64, 98, 44, 118, 54, SSD1306_WHITE);

    }
    else {
      display.drawTriangle(30, 64, 30, 44, 10, 54, SSD1306_WHITE);
      display.fillTriangle(30, 64, 30, 44, 10, 54, SSD1306_WHITE);
    }


  }

  display.display();      // Show initial text

}


extern "C" void SAADC_IRQHandler_v( void )
{
  if ( NRF_SAADC->EVENTS_END != 0 )
  {
    NRF_SAADC->EVENTS_END = 0;
    adcFlag = true;
  }
}

void initTimer4()
{
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  NRF_TIMER4->PRESCALER = 0;
  NRF_TIMER4->CC[0] = 16000000 / SAMPLES_PER_SECOND ; // Needs prescaler set to 0 (1:1) 16MHz clock
  NRF_TIMER4->TASKS_START = 1;
}


void initPPI()
{
  NRF_PPI->CH[PPI_CHANNEL].EEP = ( uint32_t )&NRF_TIMER4->EVENTS_COMPARE[0];
  NRF_PPI->CH[PPI_CHANNEL].TEP = ( uint32_t )&NRF_SAADC->TASKS_START;
  NRF_PPI->FORK[PPI_CHANNEL].TEP = ( uint32_t )&NRF_SAADC->TASKS_SAMPLE;
  NRF_PPI->CHENSET = ( 1UL << PPI_CHANNEL );
}

void initADC()
{
  nrf_saadc_disable();

  NRF_SAADC->RESOLUTION = NRF_SAADC_RESOLUTION_12BIT;

  NRF_SAADC->CH[2].CONFIG = ( SAADC_CH_CONFIG_GAIN_Gain1_2    << SAADC_CH_CONFIG_GAIN_Pos ) |
                            ( SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos ) |
                            ( SAADC_CH_CONFIG_REFSEL_Internal   << SAADC_CH_CONFIG_REFSEL_Pos ) |
                            ( SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos ) |
                            ( SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos ) |
                            ( SAADC_CH_CONFIG_TACQ_40us        << SAADC_CH_CONFIG_TACQ_Pos );

  NRF_SAADC->CH[2].PSELP = SAADC_CH_PSELP_PSELP_VDDHDIV5 << SAADC_CH_PSELP_PSELP_Pos;
  NRF_SAADC->CH[2].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

  NRF_SAADC->RESULT.MAXCNT = ADC_BUFFER_SIZE;
  NRF_SAADC->RESULT.PTR = ( uint32_t )&adcBuffer;

  NRF_SAADC->EVENTS_END = 0;
  nrf_saadc_int_enable( NRF_SAADC_INT_END );
  NVIC_SetPriority( SAADC_IRQn, 1UL );
  NVIC_EnableIRQ( SAADC_IRQn );

  nrf_saadc_enable();

  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
  while ( NRF_SAADC->EVENTS_CALIBRATEDONE == 0 );
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while ( NRF_SAADC->STATUS == ( SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos ) );
}

void store_settings(uint32_t type)
{
  const uint32_t flash_buffer_address = reinterpret_cast<uint32_t>(flash_buffer);
  
  static FlashIAPBlockDevice bd(flash_buffer_address, kFlashBufferSize);
  bd.init();
  uint8_t* ram_buffer = (uint8_t*)(malloc(kFlashBufferSize));
  bd.read(ram_buffer, 0, kFlashBufferSize);
  bd.erase(0, kFlashBufferSize);

  uint32_t* available_settings =  reinterpret_cast<uint32_t*>(ram_buffer);
  float* stored_lat = reinterpret_cast<float*>(ram_buffer) + sizeof(float);
  float* stored_long = reinterpret_cast<float*>(ram_buffer) + 2 * sizeof(float);
  uint32_t* stored_qiblah = reinterpret_cast<uint32_t*>(ram_buffer) + 3 * sizeof(float);
  float* stored_hardiron = reinterpret_cast<float*>(ram_buffer) + 4 * sizeof(float);
  float* stored_softiron = reinterpret_cast<float*>(ram_buffer) + 7 * sizeof(float);

for(int i=0;i<64;i++)
{
  Serial.println(ram_buffer[i]);
}

  Serial.println("store_settings-pre");
  Serial.println(*available_settings);
  
  if (type == 0 )
    *available_settings = 0x00000000;
  else if(type == LATLONG_SETTING_FLAG)
  {
    *stored_lat= YOUR_LAT;
    *stored_long= YOUR_LONG;
    
  *available_settings = *available_settings | LATLONG_SETTING_FLAG;
  }
  else if(type ==  QIBLA_SETTING_FLAG)
  {
    *stored_qiblah = qiblah;
  *available_settings = *available_settings | QIBLA_SETTING_FLAG;
  }
  else if(type ==  MAT_SETTING_FLAG)
  {
    stored_hardiron[0]=B[0] ; stored_hardiron[1]=B[1] ; stored_hardiron[2]=B[2] ; 
    
    stored_softiron[0]= H[0][0]; stored_softiron[1]= H[0][1]; stored_softiron[2]= H[0][2]; 
    stored_softiron[3]= H[1][0]; stored_softiron[4]= H[1][1]; stored_softiron[5]= H[1][2]; 
    stored_softiron[6]= H[2][0]; stored_softiron[7]= H[2][1]; stored_softiron[8]= H[2][2]; 
    
  *available_settings = *available_settings | MAT_SETTING_FLAG;
  }

  bd.program(ram_buffer, 0, kFlashBufferSize);

  Serial.println("store_settings-post");
  Serial.println(*available_settings);


    bd.read(ram_buffer, 0, kFlashBufferSize);

    for(int i=0;i<64;i++)
{
  Serial.println(ram_buffer[i]);
}

  bd.deinit();

  // Deallocate RAM buffer
  free(ram_buffer);
  
}


void check_stored_settings()
{
  const uint32_t flash_buffer_address = reinterpret_cast<uint32_t>(flash_buffer);
  Serial.println(String("flash_buffer_address=0x") + String(flash_buffer_address, 16));
  
  static FlashIAPBlockDevice bd(flash_buffer_address, kFlashBufferSize);
  bd.init();
  uint8_t* ram_buffer = (uint8_t*)(malloc(kFlashBufferSize));
  bd.read(ram_buffer, 0, kFlashBufferSize);

  uint32_t* available_settings =  reinterpret_cast<uint32_t*>(ram_buffer);
  float* stored_lat = reinterpret_cast<float*>(ram_buffer) + sizeof(float);
  float* stored_long = reinterpret_cast<float*>(ram_buffer) + 2 * sizeof(float);
  uint32_t* stored_qiblah = reinterpret_cast<uint32_t*>(ram_buffer) + 3 * sizeof(float);
  float* stored_hardiron = reinterpret_cast<float*>(ram_buffer) + 4 * sizeof(float);
  float* stored_softiron = reinterpret_cast<float*>(ram_buffer) + 7 * sizeof(float);

  Serial.println("available_settings:");
  Serial.println(*available_settings);
  if (*available_settings & MAT_SETTING_FLAG)
  {
    Serial.println("MAT_SETTING_FLAG");
    for (int i = 0; i < 3; i++)
    {
      B[i] = * stored_hardiron;
      Serial.println(*stored_hardiron);
      stored_hardiron ++;
    }
  }

  if (*available_settings & MAT_SETTING_FLAG)
  {
    Serial.println("MAT_SETTING_FLAG");
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        H[i][j] = * stored_softiron;
        Serial.println(*stored_softiron);
        stored_softiron ++;
      }
    }
  }
  //
  if (*available_settings & LATLONG_SETTING_FLAG) {
    Serial.println("LATLONG_SETTING_FLAG");
    YOUR_LAT = *stored_lat;
    Serial.println(YOUR_LAT);
  }
  if (*available_settings & LATLONG_SETTING_FLAG) {
    Serial.println("LATLONG_SETTING_FLAG");
    YOUR_LONG = *stored_long;
    Serial.println(YOUR_LONG);
  }

  if (*available_settings & QIBLA_SETTING_FLAG) {
    Serial.println("QIBLA_SETTING_FLAG");
    Serial.println(qiblah);
    qiblah = *stored_qiblah;
  }

  bd.deinit();
}

void setup() {

  Serial.begin(115200);
  //  while (!Serial);
  Serial.println("Started");
  Serial.println("NRF52840 Rev:");
  Serial.println(NRF_FICR->INFO.VARIANT, HEX);
  uint32_t variant = NRF_FICR->INFO.VARIANT;
  for (int i = 0; i < sizeof(NRF_FICR->INFO.VARIANT); i++) {
    char c = variant >> (8 * i) & 0x000000FF;
    Serial.print(c);
  }
  Serial.println();
  Serial.println("NRF52840 Part:");
  Serial.println(NRF_FICR->INFO.PART, HEX);
  Serial.println("NRF52840 Package:");
  Serial.println(NRF_FICR->INFO.PACKAGE, HEX);

  initADC();
  initTimer4();
  initPPI();

  setupDisplay();

  setupBLE();


  Serial.println("Qiblah:");

  qiblah = qiblahFinder(YOUR_LAT, YOUR_LONG);

  check_stored_settings();
  
  Serial.println(qiblah);

  /* To initialize the hal function */
  //Wire.begin();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  orientation_filter.begin(20);


  print_qiblah_on_screen(true);

  delay(2000);

#if GPS_EN == 1
  setupGPS();
#endif
}


void Calculat_angles() {

  static float theta_acc_old = 0.0;
  static float phi_acc_old = 0.0;
  static float theta_acc = 0.0;
  static float phi_acc = 0.0;

  static float cal_gyro_x = 0.0;
  static float cal_gyro_y = 0.0;
  static float cal_gyro_z = 0.0;

  static float last_timestamp = 0.0;

  float delta_time =  (millis() - last_timestamp) / 1000.;
  last_timestamp  = millis() ;

  theta_acc = atan2(sensor_data.acc.x, sensor_data.acc.z) / 2 / PI * 360;
  phi_acc = atan2(sensor_data.acc.y, sensor_data.acc.z) / 2 / PI * 360;

  cal_gyro_x = sensor_data.gyr.x;//* 360 / (2*3.141592654) ;
  cal_gyro_y = sensor_data.gyr.y;//* 360 / (2*3.141592654) ;
  cal_gyro_z = sensor_data.gyr.z;//* 360 / (2*3.141592654) ;

  theta_gyro = theta_gyro + cal_gyro_y * delta_time  ; // deg/sec
  phi_gyro = phi_gyro + cal_gyro_x * delta_time  ; // deg/sec
  yaw_gyro = yaw_gyro + cal_gyro_z * delta_time  ; // deg/sec

  theta_acc_new = 0.9 * theta_acc_old + 0.1 * theta_acc;
  phi_acc_new = 0.9 * phi_acc_old + 0.1 * phi_acc;

  theta_acc_old = theta_acc_new;
  phi_acc_old = phi_acc_new;

  updateMadjwick();

}

void updateMadjwick() {
  orientation_filter.update(sensor_data.gyr.x, sensor_data.gyr.y, sensor_data.gyr.z,
                            sensor_data.acc.x, sensor_data.acc.y, sensor_data.acc.z,
                            mag_data.x, mag_data.y, mag_data.z);

  Madjwick_p = static_cast<int>(orientation_filter.getPitch()) ;

  Madjwick_r = static_cast<int>(orientation_filter.getRoll());

  Madjwick_y = static_cast<int>(orientation_filter.getYaw());
}

void print_log()
{
#if MOTION_CAL_SOFT == 1
  Serial.print("Raw:");
  int a_x = sensor_data.acc.x * (32768.0 / 4.0);
  int a_y = sensor_data.acc.y * (32768.0 / 4.0);
  int a_z = sensor_data.acc.z * (32768.0 / 4.0);

  Serial.print(a_x );
  Serial.print(',');
  Serial.print(a_y);
  Serial.print(',');
  Serial.print(a_z);
  Serial.print(',');

  int g_x = sensor_data.acc.x * (32768.0 / 2000.0);
  int g_y = sensor_data.acc.y * (32768.0 / 2000.0);
  int g_z = sensor_data.acc.z * (32768.0 / 2000.0);

  Serial.print(g_x);
  Serial.print(',');
  Serial.print(g_y);
  Serial.print(',');
  Serial.print(g_z);
  Serial.print(',');

  Serial.print((int16_t)mag_data.x * 10);
  Serial.print(',');
  Serial.print((int16_t)mag_data.y * 10);
  Serial.print(',');
  Serial.println((int16_t)mag_data.z * 10);
#endif

#if MAG_CALIBRATION_LOG == 1
  Serial.println("Mag:");
  Serial.print(mag_data.x);
  Serial.print(",");
  Serial.print(mag_data.y);
  Serial.print(",");
  Serial.println(mag_data.z);
#endif

#if MAG_CALIBRATED_LOG == 1
  Serial.print(mag_x);
  Serial.print(",");
  Serial.print(mag_y);
  Serial.print(",");
  Serial.println(mag_z);
#endif


#if COMPASS_LOG == 1
  Serial.print("Pich:");
  Serial.print(pitch*180.0/PI/*Madjwick_p*/);
  Serial.print(',');
  Serial.print("Roll:");
  Serial.print(roll*180.0/PI/*Madjwick_r*/);
  Serial.print(',');
  Serial.print("Yaw:");
  Serial.print(yaw_gyro);
  Serial.print(',');
  Serial.print("Compass:");
  Serial.print(newHeading); // without tilt compensation
  Serial.print(',');

  Serial.print("Compass2:");
  Serial.println(newHeading2); // with tilt compensation
#endif

#if GYRO_CAL == 1
  Serial.print(sensor_data.gyr.x);
  Serial.print(',');
  Serial.print(sensor_data.gyr.y);
  Serial.print(',');
  Serial.println(sensor_data.gyr.z);
#endif

#if VISUAL_3D == 1
  Serial.print(Madjwick_r);
  Serial.print(',');
  Serial.print(Madjwick_p);
  Serial.print(',');
  Serial.println(static_cast<int>(yaw_gyro));
#endif

}

void print_qiblah_on_screen(bool show_qibla_word ) {

  //  display.clearDisplay(); // Clear the display buffer
  //
  //  display.display(); // Show the display buffer on the screen

  display.drawBitmap(48, 32, Makkah_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
  display.display(); // Show the display buffer on the screen
  //  delay(100);
  if (show_qibla_word)
  {
    display.drawBitmap(16, 0, qibla_logo, 96, 32, SSD1306_WHITE);
    display.display(); // Show the display buffer on the screen
  }

}
void print_on_screen(float current_angle)
{
  //  display.clearDisplay(); // Clear the display buffer
  //
  //  display.display(); // Show the display buffer on the screen

  //  display.setCursor(0, 0);
  //  sprintf(log_buffer, "Compass1:%d", newHeading);
  //  display.println(log_buffer);
  //display.display();      // Show initial text

  display.setCursor(50, 40);
  sprintf(log_buffer, "%d", newHeading2);
  display.println(log_buffer);
  //display.display();      // Show initial text

  //  display.setCursor(0, 20);
  //  sprintf(log_buffer, "pitch:%d roll:%d", Madjwick_p, Madjwick_r);
  //  display.println(log_buffer);

  if ( current_angle > qiblah && current_angle - qiblah < 180) // left
  {
    screen_info.arrow_right_direction = 0;
  }
  else
  {
    screen_info.arrow_right_direction = 1;
  }

  if ( current_angle < qiblah && qiblah - current_angle < 180) // right
  {
    screen_info.arrow_right_direction = 1;
  }
  else
  {
    screen_info.arrow_right_direction = 0;
  }


}



void calculate_heading()
{
  static int oldHeading = 0; // without tilt comp.
  static int oldHeading2 = 0; // with titl comp.

  float heading = atan2(mag_y, mag_x);

  if (heading < 0) {
    heading = heading + 2 * PI;
  }
  float heading_degree = heading * 180.0 / PI;
  heading_degree = heading_degree + DECLINATION ;

  newHeading = heading_degree;
  newHeading = 0.9 * oldHeading + 0.1 * newHeading;
  oldHeading = newHeading;


  pitch = Madjwick_p * PI / 180.0 ;
  roll = Madjwick_r * PI / 180.0 ;

  pitch = theta_acc_new * PI / 180.0;
  roll = phi_acc_new * PI / 180.0;

  mag_x = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch);
  mag_y = mag_y * cos(roll) - mag_z * sin(roll);

  heading = atan2(mag_y, mag_x);

  if (heading < 0) {
    heading = heading + 2 * PI;
  }
  heading_degree = heading * 180.0 / PI;
  heading_degree = heading_degree + DECLINATION ;

  newHeading2 = heading_degree;
  newHeading2 = 0.9 * oldHeading2 + 0.1 * newHeading2;
  oldHeading2 = newHeading2;
}

void loop() {

#if GPS_EN == 1
  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - Gpstimer > 2000) {
    Gpstimer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");

      Serial.print(GPS.latitudeDegrees);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees);

      YOUR_LAT = GPS.latitudeDegrees;

      YOUR_LONG = GPS.longitudeDegrees;

      qiblah = qiblahFinder(YOUR_LAT, YOUR_LONG, GPS.lat); // update qiblah angle based on current lat. and lon.

    }
  }
#endif

  if (IMU.accelerationAvailable() == true) // IMU dataready interupt flag
  {
    IMU.readAcceleration(sensor_data.acc.x, sensor_data.acc.y, sensor_data.acc.z);
    IMU.readGyroscope(sensor_data.gyr.x, sensor_data.gyr.y, sensor_data.gyr.z);
  }

  if (IMU.magneticFieldAvailable() == true) // mag. dataready interupt flag
  {
    IMU.readMagneticField(mag_data.x, mag_data.y, mag_data.z);

    //apply calibration
    float _mag_x = mag_data.x - 1 * B[0];
    float _mag_y = mag_data.y - 1 * B[1];
    float _mag_z = mag_data.z - 1 * B[2];

    mag_x = H[0][0] * _mag_x + H[0][1] * _mag_y + H[0][2] * _mag_z ;
    mag_y = H[1][0] * _mag_x + H[1][1] * _mag_y + H[1][2] * _mag_z ;
    mag_z = H[2][0] * _mag_x + H[2][1] * _mag_y + H[2][2] * _mag_z ;

    Calculat_angles();
    print_log();
    calculate_heading();
  }

  if (qiblah + TOLERANCE_MARGINE > newHeading2 && newHeading2 > qiblah - TOLERANCE_MARGINE)
  {
    screen_info.qiblah_reached = true;
  }
  else
  {
    screen_info.qiblah_reached = false;
  }
  //  if (update_screen_flag)
  //  {
  //    update_screen_flag = false;
  update_screen();
  //  }
}
