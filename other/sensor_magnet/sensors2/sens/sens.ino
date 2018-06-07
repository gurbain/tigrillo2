/* Tigrillo_openCM
 
 This script implements the basic functions to interface sensors and actuators over the UART 
 so that they can be used on another board. No major computation or logi is done here.
 
 Created on September 11th 2017
 
 Gabriel Urbain <gabriel.urbain@ugent.be>
 Copyright 2017, Human Brain Projet, SP10
 */

/* -------- Global Variables and Preprocessor Commands -------- */

// Includes
#include "sens.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>



/* -------- OpenCM Required Functions -------- */

void setup() {

	Serial2.begin(BAUDRATE_UART_921600);
	
	pinMode(SENS_PIN_FR, INPUT_ANALOG);
	pinMode(SENS_PIN_FL, INPUT_ANALOG);
	pinMode(SENS_PIN_BR, INPUT_ANALOG);
	pinMode(SENS_PIN_BL, INPUT_ANALOG);
}


void loop() {
	
	Serial2.print(analogRead(SENS_PIN_FR));
	Serial2.print(",");
	Serial2.print(analogRead(SENS_PIN_FL));
	Serial2.print(",");
	Serial2.print(analogRead(SENS_PIN_BR));
	Serial2.print(",");
	Serial2.print(analogRead(SENS_PIN_BL));
	Serial2.print(";");
	delay(10);

}
