/* Tigrillo_openCM
 
 This script implements the basic functions to interface sensors and actuators over the UART 
 so that they can be used on another board. No major computation or logi is done here.
 
 Created on September 11th 2017
 
 Gabriel Urbain <gabriel.urbain@ugent.be>
 Copyright 2017, Human Brain Projet, SP10
 */

/* -------- Global Variables and Preprocessor Commands -------- */

// Includes
#include "act.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>

// Declare library objects
Dynamixel Dxl(UART_3);

/* -------- OpenCM Required Functions -------- */

void setup() {

	// Setup Interuption for received UART USB message
	SerialUSB.begin();
	SerialUSB.attachInterrupt(processRx);

	// Setup Dynamixel protocol
	Dxl.begin(BAUDRATE_SERVO_115200);
	Dxl.jointMode(ACT_ID_FL);
	Dxl.jointMode(ACT_ID_FR);
	Dxl.jointMode(ACT_ID_BL);
	Dxl.jointMode(ACT_ID_BR);

}


void loop() {

	// Nothing to do in the main loop
	delay(10);

}

/* -------- Custom Processing Functions -------- */

void processRx(byte* buffer, byte size) {

	SerialUSB.println("yoh");

	// Ensure buffer is clean by setting next address value to 0 (not always true but don't know why)
  	buffer[size] = '\0';

	// Process received instruction
	char instruction = (char)buffer[0];
	switch (instruction) {
	
	case 'A':
		if (size > 1) {
			int act_leg[MAX_ARG_SIZE];
			SerialUSB.println((char*)(buffer+1));
			int res = str2i((char*)(buffer+1), act_leg, ACT_NUM);
			if (res != 0) {
				if  (res == -1)
					SerialUSB.println("{'ACK': {'Instruction': 'A', 'Data': 'Error: Too few numbers in argument list!'}}");
				else if (res == -2)
					SerialUSB.println("{'ACK': {'Instruction': 'A', 'Data': 'Error: Too many numbers in argument list!'}}");
				else if (res == -3)
					SerialUSB.println("{'ACK': {'Instruction': 'A', 'Data': 'Error: Wrong character in argument list!'}}");
				else if (res == -4)
					SerialUSB.println("{'ACK': {'Instruction': 'A', 'Data': 'Error: Unknown!'}}");
				break; 
			}
			updateMotors(act_leg);
			char text[120];
			sprintf(text, "{'ACK': {'Instruction': 'A', 'Data': 'Success: %i %i %i %i !'}}", act_leg[0], act_leg[1], act_leg[2], act_leg[3]);
			SerialUSB.println(text);
		}
		break;
		
	default:
		break;
	}

  	// WARNING: Empty buffer after each usage
  	emptyStr((char*)buffer, MAX_ARG_SIZE);

}


void updateMotors(int act_leg[]) {
	
	int mi = SERVO_MIN_LIM;
	int ma = SERVO_MAX_LIM;
	int in = SERVO_MAX_LIM - SERVO_MIN_LIM;
	int ra = SERVO_RANGE;
	
	// Right legs
	Dxl.writeWord(ACT_ID_FR, GOAL_POS_ADDR, (mi + act_leg[ACT_ID_FR-1] * in / ra));
	Dxl.writeWord(ACT_ID_BR, GOAL_POS_ADDR, (mi + act_leg[ACT_ID_BR-1] * in / ra));
	
	// Left legs
	Dxl.writeWord(ACT_ID_FL, GOAL_POS_ADDR, (ma - act_leg[ACT_ID_FL-1] * in / ra));
	Dxl.writeWord(ACT_ID_BL, GOAL_POS_ADDR, (ma - act_leg[ACT_ID_BL-1] * in / ra));

}

void emptyStr(char* tab, int size) {

	for (int i = 0; i < size; i++) {
		tab[i] = '\0';
	}

}


int str2i(char* str, int* tab, int size) {
	
	char *end = str;
	int index = 0;

	// Split srting with symbol "," and convert to integer value 
	while(*end) {
		errno = 0;
		int n = strtol(str, &end, 10);
		if (str == end)
			return -3;
		else if (errno != 0 && n == 0)
			return -4;
		
		tab[index] = n;
		while (*end == ',') {
			end++;
		}
		str = end;
		index ++;
	}
		
	// Check the number of integer in the array
	if (index < size) {
		return -1;
	} else if (index > size) {
		return -2;
	} else {
		return 0;
	}

}
