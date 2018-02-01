/* Tigrillo_openCM
 
 This script implements the basic functions to interface sensors and actuators over the UART 
 so that they can be used on another board. No major computation or logi is done here.
 
 Created on September 11th 2017
 
 Gabriel Urbain <gabriel.urbain@ugent.be>
 Copyright 2017, Human Brain Projet, SP10
 */

/* -------- Global Variables and Preprocessor Commands -------- */

// Includes
#include "tigrillo_opencm.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>



/* -------- OpenCM Required Functions -------- */

void setup() {

	SerialUSB.begin();
	pinMode(0, INPUT_ANALOG);
	pinMode(1, INPUT_ANALOG);
	pinMode(2, INPUT_ANALOG);
	pinMode(3, INPUT_ANALOG);

}


void loop() {
	
	int sens_1 = analogRead(0);
	int sens_2 = analogRead(1);
	int sens_3 = analogRead(2);
	int sens_4 = analogRead(3);
	SerialUSB.print(sens_1);
	SerialUSB.print(",");
	SerialUSB.print(sens_2);
	SerialUSB.print(",");
	SerialUSB.print(sens_3);
	SerialUSB.print(",");
	SerialUSB.print(sens_4);
	SerialUSB.print(";");
	delay(20);

}
