//V0.2
//implementing uart slave
#include <Wire.h>

const int I2C_ADDR = 30;


// parameters
//Final print
const int COLUMNPINS[] =  {1, 0, 2, 3, 4};
const int ROWPINS[] = {9, 10, 11, 13, 14, 15, 0, 1, 2, 3};


//prototype

const int LEN_COLUMNS  = sizeof(COLUMNPINS) / sizeof(int);
const int LEN_ROWS = sizeof(ROWPINS) / sizeof(int);
int ROWSELECT = 0;

volatile int sensor_raw[LEN_ROWS][LEN_COLUMNS];



void i2c_onReceive_callback(int n) {
  ROWSELECT = Wire.read(); // receive byte as a character
}

void i2c_onRequest_callback() {
  for (int m = 0; m < LEN_COLUMNS; m++) {
    byte b = sensor_raw[ROWSELECT][m] / 4;
    Wire.write(b);
  }
}

void getData() {
  for (int i = 0; i < LEN_ROWS; i++) {

    //all rows high impedance
    for (int k = 0; k < LEN_ROWS; k++) {
      pinMode(ROWPINS[k], INPUT);   // set digital pin as input
    }

    // turn on selected row
    pinMode(ROWPINS[i], OUTPUT);   // set digital pin as output
    digitalWrite(ROWPINS[i], HIGH);

    for (int j = 0; j < LEN_COLUMNS; j++) { // read out analog inputs
      sensor_raw[i][j] = analogRead(COLUMNPINS[j]);
    }
  }
}


void setup() {

  //initialiseer i2c
  Wire.begin(I2C_ADDR);                // join i2c bus with address
  Wire.onRequest(i2c_onRequest_callback); // register i2c request
  Wire.onReceive(i2c_onReceive_callback); // register i2c recieve

  // init variabelen matrixen
  for (int i = 0; i < LEN_COLUMNS; i++) {
    for (int j = 0; j < LEN_ROWS; j++) {
      sensor_raw[j][i] = 1;
    }
  }


}

void loop() {
  getData();
  //  printData();
  delay(1);
}



