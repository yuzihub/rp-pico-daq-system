#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <ds3231.h>
#include "hardware/adc.h"
#include "hardware/dma.h"


// ------------------------- Declare Constants ------------------------- //

#define spi0   ((spi_inst_t *)spi0_hw)

#define BMI160_COMMAND_REG_ADDR    UINT8_C(0x7E)
#define BMI160_SOFT_RESET_CMD      UINT8_C(0xb6)
#define BMI160_RA_CMD               0x7E
#define BMI160_CMD_SOFT_RESET       0xB6
#define BMI160_GYR_PMU_STATUS_BIT   2
#define BMI160_CMD_ACC_MODE_NORMAL  0x11
#define BMI160_ACC_PMU_STATUS_BIT   4
#define BMI160_RA_ACCEL_RANGE       0X41
#define BMI160_CMD_GYR_MODE_NORMAL  0x15
#define BMI160_RA_GYRO_RANGE        0X43
#define BMI160_RA_PMU_STATUS        0x03
const uint8_t BMI_addr_1 = 0x69; // I2C address of the MPU-6050 with SDO pin HIGH
const int BMI160_SOFT_RESET_DELAY_MS = 15;

typedef enum {
  BMI160_ACCEL_RANGE_2G  = 0X03, /**<  +/-  2g range */
  BMI160_ACCEL_RANGE_4G  = 0X05, /**<  +/-  4g range */
  BMI160_ACCEL_RANGE_8G  = 0X08, /**<  +/-  8g range */
  BMI160_ACCEL_RANGE_16G = 0X0C, /**<  +/- 16g range */
} BMI160AccelRange;

typedef enum {
  BMI160_GYRO_RANGE_2000 = 0, /**<  +/- 2000 degrees/second */
  BMI160_GYRO_RANGE_1000,     /**<  +/- 1000 degrees/second */
  BMI160_GYRO_RANGE_500,      /**<  +/-  500 degrees/second */
  BMI160_GYRO_RANGE_250,      /**<  +/-  250 degrees/second */
  BMI160_GYRO_RANGE_125,      /**<  +/-  125 degrees/second */
} BMI160GyroRange;

//typedef enum{
//  GPIO_SLEW_RATE_SLOW;
//  GPIO_SLEW_RATE_FAST;
//}

float gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z = 0;


const int ISRStart = 6;
const int ISRPause = 7;
const int ISRStop = 8;
const int GreenLED = 10;
const int YellowLED = 11;
const int RedLED = 12;

const int adc_pin_1 = 26; //GP26 or Pin 31
const int adc_pin_2 = 27; //GP27 or Pin 32

const int pain_ind_pin = 22;
const int mpu1Pin = 14;
const int mpu2Pin = 15;

const int chipSelectSD = 17; //chip select pin for SD card SPI
const int SPIMisoPin = 16;
const int SPIMosiPin = 19;
const int SPISckPin = 18;
const int ADCbits = 12;
uint SPIbaudrate = 50000000;


//#if SD_FAT_TYPE == 0
//SdFat sd;
//typedef File file_t;
//#elif SD_FAT_TYPE == 1
//SdFat32 sd;
//typedef File32 file_t;
//#elif SD_FAT_TYPE == 2
//SdExFat sd;
//typedef ExFile file_t;
//#elif SD_FAT_TYPE == 3
//SdFs sd;
//typedef FsFile file_t;
//#else  // SD_FAT_TYPE
//#error Invalid SD_FAT_TYPE
//#endif  // SD_FAT_TYPE

typedef File32 file_t;
SdFat32 sd; //SdFat instance
file_t myFile; //file initialization

const int imuArrSize = 6;
const int micArrSize = 200;

float tempMicArmData[micArrSize];
float tempMicBackData[micArrSize];
int micDataCounter;

struct datastore {
  //struct to store all sensor data before writing them to SD and sending them with bluetooth
  float imuArm[imuArrSize];
  float imuBack[imuArrSize];
  float micArm[micArrSize];
  float micBack[micArrSize];
  float painInd;
};



const size_t FIFO_SIZE_BYTES = 16 * 512;
//longest file name we can give so that we can change it without errors when name it with the help of RTC
char fileName[] = "10-10-1900-24-00-00.bin";
struct ts t;

//flags we set in interrupts
bool createFileFlag = false;
bool startRecFlag = false;
bool pauseRecFlag = false;
bool stopRecFlag = false;
bool sendBLEFlag = false;
bool powerDownFlag = false;
bool setTimersFlag = false;
bool micInterruptFlag = false;

//button debouncing for interrupts
unsigned long start_button_time = 0;
unsigned long start_last_button_time = 0;
unsigned long pause_button_time = 0;
unsigned long pause_last_button_time = 0;
unsigned long stop_button_time = 0;
unsigned long stop_last_button_time = 0;
unsigned long power_button_time = 0;
unsigned long power_last_button_time = 0;

struct datastore myData;

struct repeating_timer ADCtimer;
bool sdRecordFlag = false;
bool sd_Error = false;

void setup(void) {


  pinMode(ISRStart, INPUT_PULLUP);
  pinMode(ISRPause, INPUT_PULLUP);
  pinMode(ISRStop, INPUT_PULLUP);
  //pinMode(12, INPUT_PULLUP);
  pinMode(GreenLED, OUTPUT);
  pinMode(YellowLED, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(pain_ind_pin, INPUT);

  //  pinMode(23, OUTPUT); //enables power supply to PWM mode to reduce ripple
  //  gpio_put(23, HIGH);
  //  gpio_pull_up(23);

  pinMode(mpu1Pin, OUTPUT);
  pinMode(mpu2Pin, OUTPUT);
  gpio_put(mpu1Pin, LOW);
  gpio_put(mpu2Pin, LOW);

  //  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  bool setRX(pin_size_t SPIMisoPin);
  bool setCS(pin_size_t chipSelectSD);
  bool setSCK(pin_size_t SPISckPin);
  bool setTX(pin_size_t SPIMosiPin);


  //  gpio_set_function(4,GPIO_FUNC_I2C);
  //  gpio_set_function(5,GPIO_FUNC_I2C);
  //
  //  gpio_set_slew_rate(4, GPIO_SLEW_RATE_SLOW);
  //  gpio_set_slew_rate(5, GPIO_SLEW_RATE_SLOW);
  //
  //  gpio_set_drive_strength(4, GPIO_DRIVE_STRENGTH_2MA);
  //  gpio_set_drive_strength(5, GPIO_DRIVE_STRENGTH_2MA);

  gpio_set_slew_rate(SPIMisoPin, GPIO_SLEW_RATE_SLOW);
  gpio_set_slew_rate(chipSelectSD, GPIO_SLEW_RATE_SLOW);
  gpio_set_slew_rate(SPISckPin, GPIO_SLEW_RATE_SLOW);
  gpio_set_slew_rate(SPIMosiPin, GPIO_SLEW_RATE_SLOW);

  gpio_set_drive_strength(SPIMisoPin, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(chipSelectSD, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(SPISckPin, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(SPIMosiPin, GPIO_DRIVE_STRENGTH_2MA);

  adc_init();
  adc_set_clkdiv(0);
  // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(adc_pin_1); // Select ADC input 0 (GPIO26)
  adc_gpio_init(adc_pin_2);
  analogReadResolution(ADCbits);

  attachInterrupt(ISRStart, button_ISR_start, LOW);
  attachInterrupt(ISRPause, button_ISR_pause, LOW);
  attachInterrupt(ISRStop, button_ISR_stop, LOW);


  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(1000000); //Serial baud rate is also the baud rate for the Bluetooth
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }
  Serial.println("Serial Connected");
  DS3231_init(DS3231_CONTROL_INTCN);

  //  Serial.print("Initializing SD card...");
  sd_Init();

  //Serial.println("SD initialization done.");

  gpio_put(mpu1Pin, HIGH);
  BMI160_init(BMI160_GYRO_RANGE_250, BMI160_ACCEL_RANGE_2G);
  gpio_put(mpu1Pin, LOW);

  gpio_put(mpu2Pin, HIGH);
  BMI160_init(BMI160_GYRO_RANGE_250, BMI160_ACCEL_RANGE_2G);
  gpio_put(mpu2Pin, LOW);

  //Signaling the user with LEDs that initialization succeeded.
  ok_LED();
  micDataCounter = 0;
  //  painInd = 0;

  add_repeating_timer_us(-50, repeating_timer_callback, NULL, &ADCtimer); //adding timer interrupt for adc
  //The timer interrupt repeats every 50us, allowing us to call a callback function 20000 times every second.
  //The timing restraint depends on how long it takes for the callback function to execute. It can be adjusted
  //according to different use cases.
}

void loop() {
  if (startRecFlag) {//if start
    if (createFileFlag) {//if a new file is to be created
      if (myFile) {//if file is open
        myFile.close(); //close it before doing anything
      }

      DS3231_get(&t); //get the time from RTC and make it the name of the file
      String tempFileName = String(t.mday) + "-" + String(t.mon) + "-" + String(t.year) + "-" + String(t.hour) + "-" + String(t.min) + "-" + String(t.sec) + ".bin";
      tempFileName.toCharArray(fileName, sizeof(fileName)); //convert string to char array

      if (sd.exists(fileName)) {//if a file with the same name exists
        sd.remove(fileName); //remove the file
        if (sd.exists(fileName)) {//if after removing the file is still there, print "File could not be removed!"
          Serial.println("File could not be removed!");
        } else {//if the file is successfully removed, print "File removed!"
          Serial.println("File removed!");
        }
      } else {//if the file with fileName does not exists in the SD card, print "File does not exist!"
        Serial.println("File does not exist!");
      }

      myFile.open(fileName, O_WRONLY | O_CREAT); //open the file with these settings for faster write speeds
      createFileFlag = false; //set createFileFlag to false to prevent creating a new file unless requested
    }
    if (!pauseRecFlag) {//if not paused
      if (setTimersFlag) {//initialize timers
        setTimersFlag = false; //prevent initialization every loop by setting the flag to false
      }

      if (sdRecordFlag) {//with the timer interrupt we make sure to get into this statement every 10ms
        //ensuring that BMI160 is read roughly at 100 Hz

        log_IMU_Arm_Data();
        log_IMU_Back_Data();
        get_Pain_Ind();
        log_Mic_Data();

        if (myFile && !(!myFile.sync() || myFile.getWriteError())) {//if file is open
          //unsigned long timeStamp = micros();
          myFile.write((const uint8_t *)&myData, sizeof(myData)); //write to file
          //timeStamp = micros() - timeStamp;
          //Serial.println(timeStamp);
          sdRecordFlag = false;
          //Serial.write((const uint8_t *)&myData, sizeof(myData)); //send data to bluetooth via serial
        } else {//if file is not open
          sd_Error = true;
          //Serial.println("error opening file"); //print error
          error_LED_on();
          while (sd_Error) {
            if (myFile) {
              myFile.close();
            }
            sd_Init();
          }
          error_LED_off();
        }
      }
    }
  } else {
    if (stopRecFlag && myFile) {//if both stop record and file is open
      myFile.close(); //close file
    }
    sleep_ms(1);
  }
}



bool repeating_timer_callback(struct repeating_timer *t) {//callback function for timer interrupt
  if (micInterruptFlag) {
    takeMicData();
  }
  return true;
}

void button_ISR_start() {//interrupt with button for starting recording in a new file
  start_button_time = millis();
  if (start_button_time - start_last_button_time > 250) {//button debugging if()
    createFileFlag = true;
    startRecFlag = true;
    pauseRecFlag = false;
    stopRecFlag = false;
    setTimersFlag = true;
    micInterruptFlag = true;
    sd_Error = false;
    gpio_put(GreenLED, HIGH);
    gpio_put(YellowLED, LOW);
    gpio_put(RedLED, LOW);
    start_last_button_time = start_button_time;
  }
}

void button_ISR_pause() {//interrupt with button for pausing recording
  pause_button_time = millis();
  if (startRecFlag) {
    if (pause_button_time - pause_last_button_time > 250) {//button debugging if()
      pauseRecFlag = !pauseRecFlag;
      stopRecFlag = false;
      setTimersFlag = true;
      micInterruptFlag = false;
      //pauseResumeFlag = !pauseResumeFlag;
      if (pauseRecFlag) {
        gpio_put(GreenLED, LOW);
        gpio_put(YellowLED, HIGH);
        gpio_put(RedLED, LOW);
      } else if (!pauseRecFlag) {
        gpio_put(GreenLED, HIGH);
        gpio_put(YellowLED, LOW);
        gpio_put(RedLED, LOW);
      }

      pause_last_button_time = pause_button_time;
    }
  }

}

void button_ISR_stop() {//interrupt with button for stopping recording
  stop_button_time = millis();
  if (stop_button_time - stop_last_button_time > 250) {//button debugging if()
    startRecFlag = false;
    pauseRecFlag = false;
    stopRecFlag = true;
    micInterruptFlag = false;
    gpio_put(GreenLED, LOW);
    gpio_put(YellowLED, LOW);
    gpio_put(RedLED, HIGH);
    stop_last_button_time = stop_button_time;
  }
}


//mic data read function
void takeMicData() {
  micInterruptFlag = false;
  adc_select_input(0);
  tempMicArmData[micDataCounter] = adc_read();
  adc_select_input(1);
  tempMicBackData[micDataCounter] = adc_read();
  micDataCounter++;
  if (micDataCounter > micArrSize - 1) {
    micDataCounter = 0;
    sdRecordFlag = true;
  }
  micInterruptFlag = true;
}

//##########BMI160 FUNCTIONS BELOW##########//
void BMI160_init(uint8_t gyro_range, uint8_t accel_range) {
  Wire.beginTransmission(BMI_addr_1);
  Wire.write(BMI160_RA_CMD);
  Wire.write(BMI160_CMD_SOFT_RESET);
  Wire.endTransmission(true);
  sleep_ms(BMI160_SOFT_RESET_DELAY_MS);

  Wire.beginTransmission(BMI_addr_1);
  Wire.write(BMI160_RA_CMD);
  Wire.write(BMI160_CMD_ACC_MODE_NORMAL);
  Wire.endTransmission(true);
  sleep_ms(1);
  while (0x1 != power_status_check(BMI160_ACC_PMU_STATUS_BIT));

  Wire.beginTransmission(BMI_addr_1);
  Wire.write(BMI160_RA_CMD);
  Wire.write(BMI160_CMD_GYR_MODE_NORMAL);
  Wire.endTransmission(true);
  sleep_ms(1);
  while (0x1 != power_status_check(BMI160_GYR_PMU_STATUS_BIT));

  set_gyro_range(gyro_range);
  set_accel_range(accel_range);
}

uint8_t power_status_check(unsigned pos) {
  Wire.beginTransmission(BMI_addr_1);
  Wire.write(BMI160_RA_PMU_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI_addr_1, 1);
  byte powerStatus = Wire.read();
  Wire.endTransmission();
  powerStatus >>= pos;
  powerStatus &= 0x03;

  sleep_ms(1);
  return powerStatus;
}

void set_gyro_range(uint8_t range) {
  Wire.beginTransmission(BMI_addr_1);
  Wire.write(BMI160_RA_GYRO_RANGE);
  Wire.write(range);
  Wire.endTransmission(true);
}

void set_accel_range(uint8_t range) {
  Wire.beginTransmission(BMI_addr_1);
  Wire.write(BMI160_RA_ACCEL_RANGE);
  Wire.write(range);
  Wire.endTransmission(true);
}

void bmi160_read() {
  //int imu_data[6];
  Wire.beginTransmission(BMI_addr_1);
  Wire.write(0x0C);  // starting with register 0x0C. (GYRO)
  Wire.endTransmission(false);
  Wire.requestFrom(BMI_addr_1, 12, true); // request a total of 12 registers
  gyr_x = Wire.read() | Wire.read() << 8;
  gyr_y = Wire.read() | Wire.read() << 8;
  gyr_z = Wire.read() | Wire.read() << 8;
  acc_x = Wire.read() | Wire.read() << 8;
  acc_y = Wire.read() | Wire.read() << 8;
  acc_z = Wire.read() | Wire.read() << 8;
}

void log_IMU_Arm_Data() {
  gpio_put(mpu1Pin, HIGH);
  bmi160_read();
  gpio_put(mpu1Pin, LOW);
  myData.imuArm[0] = gyr_x;
  myData.imuArm[1] = gyr_y;
  myData.imuArm[2] = gyr_z;
  myData.imuArm[3] = acc_x;
  myData.imuArm[4] = acc_y;
  myData.imuArm[5] = acc_z;
}

void log_IMU_Back_Data() {
  gpio_put(mpu2Pin, HIGH);
  bmi160_read();
  gpio_put(mpu2Pin, LOW);
  myData.imuBack[0] = gyr_x;
  myData.imuBack[1] = gyr_y;
  myData.imuBack[2] = gyr_z;
  myData.imuBack[3] = acc_x;
  myData.imuBack[4] = acc_y;
  myData.imuBack[5] = acc_z;
}

void log_Mic_Data() {
  micInterruptFlag = false;
  std::copy(tempMicArmData, tempMicArmData + micArrSize, myData.micArm);
  std::copy(tempMicBackData, tempMicBackData + micArrSize, myData.micBack);
  micInterruptFlag = true;
}

void get_Pain_Ind() {
  if (gpio_get(pain_ind_pin) != 0) {
    myData.painInd = 1;
  } else {
    myData.painInd = 0;
  }
}

void error_LED_on() {
  gpio_put(GreenLED, LOW);
  gpio_put(YellowLED, HIGH);
  gpio_put(RedLED, HIGH);
}

void error_LED_off() {
  gpio_put(YellowLED, LOW);
  gpio_put(RedLED, LOW);
}

void ok_LED() {
  gpio_put(GreenLED, HIGH);
  sleep_ms(1000);
  gpio_put(GreenLED, LOW);
  sleep_ms(1000);
  gpio_put(GreenLED, HIGH);
  sleep_ms(1000);
  gpio_put(GreenLED, LOW);
}

void sd_Init() {
  while (!sd.begin(chipSelectSD, SPIbaudrate)) {//SD initialization
    //Serial.println("SD initialization failed!");
    //Signaling the user with LEDs that initialization failed.
    error_LED_on();
    sleep_ms(100);
  }
  error_LED_off();
  sd_Error = false;
}
