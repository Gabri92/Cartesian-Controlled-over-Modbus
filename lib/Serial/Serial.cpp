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
DynamicJsonDocument inputJson(256);
DynamicJsonDocument outputJson(256);

/**
 * @brief 
 * Manage incoming data from serial port, saving
 * it to a json object
 */
void incomingSerial()
{
  // Check if the other Arduino is transmitting
  if (Serial2.available() > 0) 
  {
    // Read Serial2 and deserialize json
    Serial.print("\nIncoming message from Jetson Nano...");
    String s;
    while((char)Serial2.read() != '{'){}
    s = "{" + Serial2.readStringUntil('}') + "}";
    Serial.println(s.length());
    while(Serial2.available() > 0) {Serial2.read();}
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
  while(Serial2.available()>0) {Serial2.read();}
}

// void incomingSerial()
// {
//   int maxchars = 256;
//   int i = 0;
//   char result[maxchars];
//   // Check if the other Arduino is transmitting
//   while (Serial2.available() > 0) 
//     {
//       char inChar = Serial2.read();
//       if (inChar == '}' || i == maxchars)
//       {
//         result[i] = '}';
//         Serial2.flush();
//       }
//       else
//       {
//         result[i] = inChar;
//         i++;
//       }
//     }
//     Serial.print(result);
//     Serial.print("\t");
//     Serial.println(sizeof(result));
//     DeserializationError err = deserializeJson(inputJson, result);
//     if (err != DeserializationError::Ok) 
//     {
//       Serial.print("deserializeJson() returned ");
//       Serial.println(err.c_str());
//       serializeJson(inputJson,Serial);
//     }
//   while(Serial2.available()>0) {Serial2.read();}
// }

/**
 * @brief 
 * Manage outgoing serial data
 * @param message // The outgoing message
 */
void outgoingSerial()
{
  // If the outgoing json is not empty, send it through
  // Serial2
  if(!outputJson.isNull())
  {
    serializeJson(outputJson,Serial2);
    serializeJson(outputJson,Serial);
    Serial.println();
    outputJson.clear();
  }
  // Cleaning the buffer
  Serial2.flush();
  while(Serial2.available()>0)
  {
    Serial2.read(); // Necessario?
  } 
  /////////////////////////////////////////
  //serializeJsonPretty(outputJson,Serial);
  /////////////////////////////////////////
}