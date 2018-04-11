/* Tigrillo_opencm.h
 
 This script implements the basic functions to interface sensors and actuators over the UART 
 so that they can be used on another board. No major computation or logi is done here.
 
 Created on November 25th, 2017
 
 Gabriel Urbain <gabriel.urbain@ugent.be>
 Copyright 2017, Human Brain Projet, SP10
*/


#ifndef _SENS_H_
#define _SENS_H_


/* -------- Robot Constants -------- */

#define BAUDRATE_UART_57600 57600
#define BAUDRATE_UART_115200 115200
#define BAUDRATE_UART_921600 921600
#define SENS_PIN_FR 0
#define SENS_PIN_FL 1
#define SENS_PIN_BR 2
#define SENS_PIN_BL 3

#endif