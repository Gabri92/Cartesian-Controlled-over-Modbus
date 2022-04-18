/******************************************************//**
 * @file Thingsboard.h
 * @author Gabriele Baldi (you@domain.com)
 * @brief 
 * 
 * Manage the initialization of the Thingsboard client
 * and the messaging with the interface(Start button) - Header file
 * 
 * @version 0.1
 * @date 2021-12-17
 **********************************************************/

#ifndef THINGSBOARD_H  
#define THINGSBOARD_H

/**
 * Libraries
 */
// Arduino libraries
#include <ArduinoJson.h>
#include <Ethernet.h>
#include <PubSubClient.h>
// Local directories
#include "Serial.h"
#include "FastAPI.h"

/**
 * Variables declaration
 */
extern bool flag_start;
extern bool flag_error;
extern bool flag_home;
extern bool flag_ready;
extern bool flag_cut;
extern bool flag_reset;
extern bool flag_cutEnabled;
extern String ID;
// Broker credentials (Thingsboard)
extern IPAddress MQTT_BROKER;
extern PubSubClient mqtt_client;

/**
 * Function declaration
 */
void initTBClient();
void rpcCallback(char*, byte*, unsigned int);
void sendTelemetry(char*, double);

#endif