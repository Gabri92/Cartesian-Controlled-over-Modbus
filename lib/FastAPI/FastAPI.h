/******************************************************//**
 * @file FastAPI.cpp
 * @author Gabriele Baldi (you@domain.com)
 * @brief 
 * 
 * Manage the Ethernet connection and the commmunication
 * with the FastAPI server - Header file
 * 
 * @version 0.1
 * @date 2021-12-17
**********************************************************/

#ifndef FASTAPI_H
#define FASTAPI_H

/**
 * Libraries
 */
// Arduino libraries
#include <ArduinoJson.h>
#include <Ethernet.h>

/**
 * Variables declaration
 */
// Server's credentials
extern int HTTP_PORT;
extern IPAddress HOST_NAME;
// Method
extern String HTTP_METHOD;
extern String PATH_NAME;
// Ethernet client
extern byte mac[];
extern EthernetClient client;
extern String ACCESS_TOKEN;

/**
 * Function declaration
 */
extern void initEthernet();
extern void httpRequest(String, String);

#endif