/*************************************************************
   Name: Test_Code_Nano_Chilli
   Device: Arduino Nano Chilli
   Chip: Samd21G18A
   Compile: Arduino Nano Chilli
   Create: 12/02/62
   Edit: 12/02/62 08.55
   By: Nattkarn.P
   Email: nattkarn@gravitechthai.com
**************************************************************/

/***************************** Lib include **************************/
#include <Wire.h>
#include <Arduino.h>
#include <HTS221.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BMP280.h>
#include <KBChain_LCD.h> 
#include <Adafruit_NeoPixel.h>

/**************************** Define ********************************/
//Left Pin
#define D5_PIN    5
#define D6_PIN    6
#define D9_PIN    9
#define TX_PIN    10
#define RX_PIN    11
#define D12_PIN   12
#define D13_PIN   13
#define D3_PIN    D3
#define D7_PIN    D7
#define D8_PIN    D8
//Right Pin
#define D2_PIN    D2
#define D1_PIN    1
#define D0_PIN    0
#define miso_PIN  22
#define mosi_PIN  23
#define sck_PIN   24
#define A5_PIN    19
#define A4_PIN    18
#define A3_PIN    17
#define A2_PIN    16
#define A1_PIN    15
#define A0_PIN    14


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            8
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      1
/********************************************************************/


/*************************** Goble Variable *************************/
double Humidity_data = 0;
double Temperature_data = 0;
double Gyro_data = 0;
double Alti_data = 0;
/************************** Instan Variable *************************/
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_BMP280 bmp;
KBChain_LCD lcd1(0x22, 16, 2);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
/********************************************************************/



/*******************************************************************
 * Function Name: Setup Function
 * Description: Setup All Function
 * Variable: 
 ******************************************************************/
void setup() {
  Serial.begin(9600);
  Wire.begin();
  // put your setup code here, to run once:
  Init_Pin();
  /************************ LCD Display ***************************/
  lcd1.begin();
  lcd1.clear(); 
  lcd1.leftToRight();                 
  lcd1.setCursor(0, 0);               
  lcd1.printstr("Testing...");        
  lcd1.leftToRight();                 
  lcd1.setCursor(0, 1);               
  lcd1.printstr("Nano Chilli");
  /************************ Neo Pixels  ***************************/
  pixels.begin(); 
  /****************************************************************/
  Serial.println("Function Start");
}


/*******************************************************************
 * Function Name: Loop
 * Description: Main Loop
 * Variable: 
 ******************************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  main_function();
}


/*******************************************************************
 * Function Name: main_function
 * Description: Control All Function
 * Variable: 
 ******************************************************************/
void main_function(){
  

  if(lsm.begin())
  {
    test_PIN_function();
    Show_neo();
    test_HTS221_Sensor_function();
    test_lsm91_Sensor_Function();
    test_bmp280_Sensor_Function();
    delay(1000);
    Show_Display(Temperature_data, Gyro_data, Alti_data);
  }else if(!lsm.begin())
  {
    test_PIN_function();
    Show_neo();
  }
  
}


/*******************************************************************
 * Function Name: Init_Pin
 * Description: Initial IO PIN Nano Chilli
 * Variable: 
 ******************************************************************/
void Init_Pin(){
  pinMode(D5_PIN, OUTPUT);
  pinMode(D6_PIN, OUTPUT);
  pinMode(D9_PIN, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, OUTPUT);
  pinMode(D12_PIN, OUTPUT);
  pinMode(D13_PIN, OUTPUT);
  pinMode(D3_PIN, OUTPUT);
  pinMode(D7_PIN, OUTPUT);
  pinMode(D8_PIN, OUTPUT);
  pinMode(D2_PIN, OUTPUT);
  pinMode(D1_PIN, OUTPUT);
  pinMode(D0_PIN, OUTPUT);
  pinMode(A5_PIN, OUTPUT);
  pinMode(A4_PIN, OUTPUT);
  pinMode(A3_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A0_PIN, OUTPUT);

}


/*******************************************************************
 * Function Name: Left_PIN
 * Description: Control IO Left Pin
 * Variable: int state
 ******************************************************************/
void Left_PIN(int state) {
  int _state;
  if (state == 1) {
    _state = HIGH;
  } else if (state == 0) {
    _state = LOW;
  }

  digitalWrite(D5_PIN, _state);
  delay(100);
  digitalWrite(D6_PIN, _state);
  delay(100);
  digitalWrite(D9_PIN, _state);
  delay(100);
  digitalWrite(TX_PIN, _state);
  delay(100);
  digitalWrite(RX_PIN, _state);
  delay(100);
  digitalWrite(D12_PIN, _state);
  delay(100);
  digitalWrite(D13_PIN, _state);
  delay(100);
  digitalWrite(D3_PIN, _state);
  delay(100);
  digitalWrite(D7_PIN, _state);
  delay(100);
  digitalWrite(D8_PIN, _state);
  delay(100);
}


/*******************************************************************
 * Function Name: Right_PIN
 * Description: Control IO Right Pin
 * Variable: int state
 ******************************************************************/
void Right_PIN(int state) {
  int _state;
  if (state == 1) {
    _state = HIGH;
  } else if (state == 0) {
    _state = LOW;
  }

  digitalWrite(D2_PIN, _state);
  delay(100);
  digitalWrite(D1_PIN, _state);
  delay(100);
  digitalWrite(D0_PIN, _state);
  delay(100);
  digitalWrite(A5_PIN, _state);
  delay(100);
  digitalWrite(A4_PIN, _state);
  delay(100);
  digitalWrite(A3_PIN, _state);
  delay(100);
  digitalWrite(A2_PIN, _state);
  delay(100);
  digitalWrite(A1_PIN, _state);
  delay(100);
  digitalWrite(A0_PIN, _state);
  delay(100);
}

/*******************************************************************
 * Function Name: test_PIN_function
 * Description: Function Test IO
 * Variable: 
 ******************************************************************/
void test_PIN_function() {
  /************************ Left PIN *********************************/
  Left_PIN(1);
  delay(500);
  Left_PIN(0);
  delay(500);
  Left_PIN(1);
  delay(500);
  /************************ Right PIN *********************************/
  Right_PIN(1);
  delay(500);
  Right_PIN(0);
  delay(500);
  Right_PIN(1);
  delay(500);
  /************************ End Test PIN *********************************/
}


/*******************************************************************
 * Function Name: Show_Display
 * Description: Show Data to LCD Display
 * Variable: String C1 -> HTS221, String C2 -> LSM9DS1, String C3 -> BMP280
 ******************************************************************/
void Show_Display(double C1, double C2, double C3){
  String HTS = (String)C1;
  String LSM = (String)C2;
  String BMP = (String)C3;

  lcd1.clear();
  lcd1.leftToRight();                 
  lcd1.setCursor(0, 0);               
  lcd1.printstr("Temp  Gyro  Altitude"); 
  lcd1.leftToRight();                 
  lcd1.setCursor(0, 1);               
  lcd1.print(C1);
  lcd1.setCursor(6, 1);               
  lcd1.print(C2);
  lcd1.setCursor(12, 1);               
  lcd1.print(C3);
}


/*******************************************************************
 * Function Name: test_HTS221_Sensor_function
 * Description: Read Data from HTS221 Sensor
 * Variable: 
 ******************************************************************/
void test_HTS221_Sensor_function(){

  /************************ Setup HTS221 ******************************/
  smeHumidity.begin();
  /********************************************************************/

  Humidity_data = smeHumidity.readHumidity();
  Temperature_data = smeHumidity.readTemperature();
  Serial.println("-------------------------- HTS221 ----------------------------------");
  Serial.print("Humidity From HTS221  ");
  Serial.println(Humidity_data);
  Serial.print("Temperature From HTS221  ");
  Serial.println(Temperature_data);
}


/*******************************************************************
 * Function Name: test_lsm91_Sensor_Function
 * Description: Read Data from LSM9DS1 Sensor
 * Variable: 
 ******************************************************************/
void test_lsm91_Sensor_Function(){

/************************ Setup LSM9DS1 *****************************/
  
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  lsm.begin();

  /******************************************************************/

  lsm.read(); /* ask it to read in the data */ 
  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  Serial.println("------------------------- LMS9DS1 ----------------------------------");
  
  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
  Gyro_data = (a.acceleration.x);
}



/*******************************************************************
 * Function Name: test_bmp280_Sensor_Function
 * Description: Read Data from BMP280 Sensor
 * Variable: 
 ******************************************************************/
void test_bmp280_Sensor_Function(){

  /************************ Setup BMP280 ***************************/
  bmp.begin();
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

 /******************************************************************/

  bmp.readTemperature();
  bmp.readPressure();
  bmp.readAltitude(1013.25);
  
  Alti_data = bmp.readAltitude(1013.25);

  Serial.println("-------------------------- BMP280 ----------------------------------");
  Serial.print("Altitude From BMP280   ");
  Serial.println(bmp.readAltitude(1013.25));

  Serial.print("Temperature From BMP280   ");
  Serial.println(bmp.readTemperature());

  Serial.print("Pressure From BMP280   ");
  Serial.println(bmp.readPressure());
  Serial.println("--------------------------------------------------------------------");
}


/*******************************************************************
 * Function Name: Show_neo
 * Description: Show NeoPixels
 * Variable: 
 ******************************************************************/
void Show_neo(){
  int temp_r = random(0,255);
  int temp_g = random(0,255);
  int temp_b = random(0,255);
  Serial.println("");
  Serial.println("Start NeoPixels");
  Serial.println("");
  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(temp_r, temp_g, temp_b)); // Moderately bright green color.
    pixels.setBrightness(100);
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(10); // Delay for a period of time (in milliseconds).
  }
}



