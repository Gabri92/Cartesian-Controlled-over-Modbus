/******************************************************//**
 * @file Serial.h
 * @author your name (you@domain.com)
 * @brief 
 * 
 * Manage the serial communication with other hardwares.
 * The messages, both incoming and outgoing, are received/sent
 * as json messages - Header file
 * 
 * @version 0.1
 * @date 2021-12-17
 **********************************************************/

#ifndef SERIAL_H
#define SERIAL_H

/**
 * Libraries
 */
// Arduino libraries
#include <ArduinoJson.h>

/**
 * Variables declaration
 */
extern DynamicJsonDocument inputJson;
extern DynamicJsonDocument outputJson;

/**
 * Function declaration
 */
void incomingSerial();
void outgoingSerial();

#endif