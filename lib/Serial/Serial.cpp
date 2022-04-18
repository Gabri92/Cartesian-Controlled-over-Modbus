/******************************************************//**
 * @file Serial.cpp
 * @author Gabriele Baldi (you@domain.com)
 * @brief 
 * 
 * Manage the serial communication with other hardwares.
 * The messages, both incoming and outgoing, are received/sent
 * as json messages
 * 
 * @version 0.1
 * @date 2021-12-17
**********************************************************/

/**
* Header
 */
#include "Serial.h"

/**
 * Variables definition
 */
// Input and output Json buffers definition
DynamicJsonDocument inputJson(1024);
DynamicJsonDocument outputJson(1024);

/**
 * @brief 
 * Manage incoming data from serial port, saving
 * it to a json object
 */
void incomingSerial()
{
  // Check if the other Arduino is transmitting
  if (Serial1.available() > 0) 
  {
    // Read Serial1 and deserialize json
    Serial.print("\nIncoming message from Jetson Nano...");
    String s;
    while((char)Serial1.read() != '{'){}
    s = "{" + Serial1.readStringUntil('}') + "}";
    Serial.println(s.length());
    while(Serial1.available() > 0) {Serial1.read();}
    // Check the incoming message
    Serial.print("I got: @");
    Serial.print(s);
    Serial.println("@");
    // Copy the string message to a json object
    DeserializationError err = deserializeJson(inputJson, s);
    if (err != DeserializationError::Ok) 
    {
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());
      serializeJson(inputJson,Serial);
    }
  }
  while(Serial1.available()>0) {Serial1.read();}
}

/**
 * @brief 
 * Manage outgoing serial data
 * @param message // The outgoing message
 */
void outgoingSerial()
{
  // If the outgoing json is not empty, send it through
  // Serial1
  if(!outputJson.isNull())
  {
    serializeJson(outputJson,Serial1);
    outputJson.clear();
  }
  // Cleaning the buffer
  Serial1.flush();
  while(Serial1.available()>0)
  {
    Serial1.read();
  } 
  /////////////////////////////////////////
  //serializeJsonPretty(outputJson,Serial);
  /////////////////////////////////////////
}