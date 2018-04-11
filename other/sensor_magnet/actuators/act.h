/* Tigrillo_opencm.h
 
 This script implements the basic functions to interface sensors and actuators over the UART 
 so that they can be used on another board. No major computation or logi is done here.
 
 Created on November 25th, 2017
 
 Gabriel Urbain <gabriel.urbain@ugent.be>
 Copyright 2017, Human Brain Projet, SP10
*/


#ifndef _ACT_H_
#define _ACT_H_


/* -------- Robot Constants -------- */

#define UART_3 3
#define MAX_ARG_SIZE 100
#define BAUDRATE_9600 9600
#define BAUDRATE_SERVO_115200 2
#define BAUDRATE_SERVO_1MB 3

#define SERVO_MIN_LIM 200
#define SERVO_MAX_LIM 820
#define SERVO_RANGE 180
#define ACT_NUM 4
#define ACT_ID_FL 1
#define ACT_ID_FR 2
#define ACT_ID_BL 3
#define ACT_ID_BR 4

#define GOAL_POS_ADDR 30


/* -------- Custom Processing Functions -------- */

void processRx(byte* buffer, byte size);

void updateMotors(int act_leg[]);

/* -------- Custom Utils Functions -------- */

void emptyStr(char* tab, int size);

int str2i(char* str, int* tab, int size);

#endif