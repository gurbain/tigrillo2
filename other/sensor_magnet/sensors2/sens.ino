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

	SerialUSB.begin();
	
	pinMode(SENS_PIN_FR, INPUT_ANALOG);
	pinMode(SENS_PIN_FL, INPUT_ANALOG);
	pinMode(SENS_PIN_BR, INPUT_ANALOG);
	pinMode(SENS_PIN_BL, INPUT_ANALOG);
}


void loop() {
	
	SerialUSB.print(analogRead(SENS_PIN_FR));
	SerialUSB.print(",");
	SerialUSB.print(analogRead(SENS_PIN_FL));
	SerialUSB.print(",");
	SerialUSB.print(analogRead(SENS_PIN_BR));
	SerialUSB.print(",");
	SerialUSB.print(analogRead(SENS_PIN_BL));
	SerialUSB.print(";");
	delay(10);

}
