/* Tigrillo_opencm.h
 
 This script implements the basic functions to interface sensors and actuators over the UART 
 so that they can be used on another board. No major computation or logi is done here.
 
 Created on November 25th, 2017
 
 Gabriel Urbain <gabriel.urbain@ugent.be>
 Copyright 2017, Human Brain Projet, SP10
*/


#ifndef _TIGRILLO_OPENCM_H_
#define _TIGRILLO_OPENCM_H_


/* -------- Robot Constants -------- */

#define UART_3 3
#define TIMER_1 1
#define MAX_ARG_SIZE 100
#define BAUDRATE_SERVO_115200 2
#define BAUDRATE_SERVO_1MB 3
#define BAUDRATE_UART_57600 57600
#define BAUDRATE_UART_115200 115200
#define BAUDRATE_UART_921600 921600
#define BUFF_SIZE 256

#define SERVO_MIN_LIM 200
#define SERVO_MAX_LIM 820
#define SERVO_RANGE 180
#define ACT_NUM 4
#define ACT_ID_FL 1
#define ACT_ID_FR 2
#define ACT_ID_BL 3
#define ACT_ID_BR 4

#define MIN_SENS_READ_TIME 2000
#define DEF_SENS_READ_TIME 20000
#define SENS_NUM 4
#define SENS_PIN_FL 0
#define SENS_PIN_FR 3
#define SENS_PIN_BL 6
#define SENS_PIN_BR 8

#define GOAL_POS_SLACK 30


/* -------- Custom Processing Functions -------- */

void processRx(byte* buffer, byte size);

void readSensors(void);

void resetSensors(void);

void updateMotors(int act_leg[]);

int changePeriod(int period);


/* -------- Custom Utils Functions -------- */

void printTab(int* tab, int size);

void initTab(int* tab, int size);

void emptyStr(char* tab, int size);

int str2i(char* str, int* tab, int size);

void wait4Bus(void);

void freeBus(void);


#endif