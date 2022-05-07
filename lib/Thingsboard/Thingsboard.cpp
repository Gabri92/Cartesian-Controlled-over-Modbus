/******************************************************//**
 * @file Thingsboard.cpp
 * @author Gabriele Baldi (you@domain.com)
 * @brief 
 * 
 * Manage the initialization of the Thingsboard client
 * and the messaging with the interface(Start button)
 * 
 * @version 0.1
 * @date 2021-12-17
 **********************************************************/

/**
 * Header
 */
#include "Thingsboard.h"

#define light 2
#define vacuum 3
#define cut1 6
#define cut2 7

/**
 * Variables definition
 */
bool flag_start = false;
bool flag_error = false;
bool flag_home = false;
bool flag_cut = false;
bool flag_ready = false;
bool flag_doubleTissue = false;
bool flag_reset = false;
bool flag_cutEnabled = false;
String ID;
// Broker credentials (Thingsboard)
IPAddress MQTT_BROKER(192,168,1,179);
PubSubClient mqtt_client(client);

/**
 * @brief 
 * Initialize thingsboard client and subscripe to necessary topics
 */
void initTBClient()
{
  mqtt_client.setServer(MQTT_BROKER,1883);
  mqtt_client.setCallback(rpcCallback);
  delay(2000);
  Serial.print("Connecting to ThingsBoard node ...");
  // Attempt to connect (clientId, username, password)
  if (mqtt_client.connect("IDX Controller","IDX_MKS", NULL) ) 
  {
    Serial.println( "[DONE]" );
    // Subscribing to receive RPC requests
    mqtt_client.subscribe("v1/devices/me/rpc/request/+");
    mqtt_client.subscribe("v1/devices/me/rpc/response/+");
  } 
  else 
  {
    Serial.print( "[FAILED] [ rc = " );
    Serial.print( mqtt_client.state() );
    Serial.println( " : retrying in 5 seconds]" );
    // Wait 5 seconds before retrying
    delay(5000);
  }
}

/**
 * @brief 
 * Manage the start of the process by pressing
 * the relative interface button
 * @param topic // topic in which the client is listening for start
 * @param payload // content of the message to be listened in the topic
 * @param length // length of the payload
 */
void rpcCallback(char* topic, byte* payload, unsigned int length)
{
  // Receiving the payload and copying it into a json object
  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';
  StaticJsonDocument<200> data;
  deserializeJson(data,json);
  ////////////////////////////////
  //Serial.println("Topic: ");
  //Serial.println(topic);
  //Serial.println("Message: ");
  //Serial.println(json);
  ////////////////////////////////
  // Reading the 'method' field of the json and, consequently, choosing an action
  String methodName = String((const char*)data["method"]);
  if (methodName.equals("setValue")) 
  {
    bool param = data["params"];
    if (param)
    {
      flag_start = true;
      Serial.println("Start");
      //digitalWrite(light,HIGH);
      //inputJson["key"] = "ATRMController";
      //inputJson["value"] = "START";
      //inputJson["blockCode"] = String(random(10000,20000));
      delay(1000);
    }
    else if(!param)
    {
      Serial.println("Stop");
      flag_start = false;
    }
  }
  else if(methodName.equals("setHomePosition"))
  {
    bool param = data["params"];
    if (param)
    {
      flag_home = true;
      Serial.println("Holder in home position");
      delay(1000);
    }
    else if(!param)
    {
      flag_home = false;
    }
  }
  else if(methodName.equals("setCutPosition"))
  {
    bool param = data["params"];
    if (param)
    {
      flag_cut = true;
      Serial.println("Holder in cut position");
      delay(1000);
    }
    else if(!param)
    {
      flag_cut = false;
    }
  }
  else if(methodName.equals("setReady"))
  {
    bool param = data["params"];
    if (param)
    {
      flag_ready = true;
      if(flag_doubleTissue)
      {
        ID = "12344";
      }
      else if(!flag_doubleTissue)
      {
        ID = "12345";
      }
      Serial.println("Block Inserted");
      delay(1000);
    }
    else if(!param)
    {
      Serial.println("Block not Inserted");
      flag_ready = false;
    }
  }  
  else if(methodName.equals("setTissue"))
  {
    bool param = data["params"];
    if (param)
    {
      flag_doubleTissue = true;
      Serial.println("Double Tissue");
    }
    else if(!param)
    {
      flag_doubleTissue = false;
      Serial.println("Single Tissue");
    } 
  }
  else if(methodName.equals("setCut"))
  {
    bool param = data["params"];
    if (param)
    {
      digitalWrite(cut1,LOW);
      digitalWrite(cut2,LOW);
      delay(100);
      digitalWrite(cut1,HIGH);
      digitalWrite(cut2,HIGH);
      flag_cutEnabled = !flag_cutEnabled;
      flag_start = false;
      mqtt_client.publish("v1/devices/me/attributes","{\"button\":\"false\"}");
    } 
  }
  else if(methodName.equals("setVacuum"))
  {
    bool param = data["params"];
    if (param)
    {
      digitalWrite(vacuum,HIGH);
      flag_start = false;
      mqtt_client.publish("v1/devices/me/attributes","{\"button\":\"false\"}");
    }
    else if(!param)
    {
      digitalWrite(vacuum,LOW);
      flag_start = false;
      mqtt_client.publish("v1/devices/me/attributes","{\"button\":\"false\"}");
    } 
  }  
  else if(methodName.equals("rpcReset"))
  {
    bool param = data["params"];
    if (param)
    {
      Serial.println("Reset Done");
      flag_reset = true;
    }
  }
  return;
}

void sendTelemetry(char* key, double value)
{
  // Cast of the value from double to char
  char telemetry[sizeof(value)];
  dtostrf(value,sizeof(value),2,telemetry);
  
  // Copies key and value in a single string
  const char* init = "{\"";
  const char* mid  = "\":\"";
  const char* end ="\"}";
  char buf[sizeof(init) + sizeof(key) + sizeof(mid) + sizeof(value) + sizeof(end)];
  strcpy(buf,init);
  strcat(buf,key);
  strcat(buf,mid);
  strcat(buf,telemetry);
  strcat(buf,end);

  // Send telemetry to Thingsboard
  mqtt_client.publish("v1/devices/me/telemetry", buf);
}