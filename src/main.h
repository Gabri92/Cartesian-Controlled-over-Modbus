/******************************************************/ /**
* @file main.h
* @author Gabriele Baldi (you@domain.com)
* @brief
*
* Manage the controller main and the action derived
* from the comunication with othier interfaces, including
* IA software - Header file
*
* @version 0.1
* @date 2021-12-17
**********************************************************/

#ifndef MAIN_H
#define MAIN_H

/**
 * Libraries
 */
#include <Arduino.h>
#include "FastAPI.h"
#include "Thingsboard.h"
#include "Serial.h"
#include "ModbusIDX.h"
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "SPI.h"

/**
 * Pre-Processor
 */

// #define SERIAL_RX_BUFFER_SIZE 256
// #define SERIAL_TX_BUFFER_SIZE 256
// PIN DEFINITION
#define PIN_TIP_HEATER 10
#define PIN_RES_HEATER 7
#define PIN_LIGHT 66
#define PIN_LASER 35
#define PIN_CURRENT_LIGHT A11
#define PIN_CS_ETHERNET 44

// Extruder PIN
#define E0_STEP_PIN 26
#define E0_DIR_PIN 28
#define E0_ENABLE_PIN 24
#ifndef E0_CS_PIN
#define E0_CS_PIN 42
#endif

/**
 * Variables declaration
 */
// extern enum FiniteStateMachine;
//  Counter variables
extern bool count;
extern unsigned long counter;
extern bool startCount;
// Timing variables
extern unsigned long counterMillis;
extern unsigned long previousMillis;
extern unsigned long monitoring_time;
extern unsigned long prev_monitoring_time;
extern unsigned long timer;
extern unsigned long prev_timer;
extern unsigned long time_extruder;
extern unsigned long time_extruder_prev;
extern unsigned long time_compression;
extern unsigned long time_compression_prev;
// Flag
extern bool flag_jetson;
extern bool flag_timer;
extern bool flag_prox;
extern bool oneTime_action_state;
extern bool oneTime_action_step;
extern bool oneTime_action ;
extern bool PID_On;
extern bool compressed;
extern bool flag_print;
extern bool flag_timer;
extern bool flag_jetson;
extern bool flag_prox;
extern bool flag_PLC_completed;
extern bool flag_laser_check;
extern bool flag_block_positioned;

// Extruder variables
int E0_axis_speed = 3000;
int E0_axis_accel = 1500;
int extrusion_step = 1;
int extrusion_step_total = 2;
bool flag_extrusion_ended = false;

// Refilling positions
double x_refilling;
double y_refilling;
double z_refilling;

// Temperature management variables
#define RREF 430.0
Adafruit_MAX31865 max_T = Adafruit_MAX31865(49, 51, 50, 52); // Punta // pin SPI e CS  // Controllare ordine chip select
Adafruit_MAX31865 max_R = Adafruit_MAX31865(53, 51, 50, 52); // Serbatoio
double temp_T = 0;
double temp_R = 0;
double temp_array_T[10] = {0,1,2,3,4,5,6,7,8,9};
double temp_array_R[10] = {5,5,6,35,54,65,46,37,28,9};
// double zero_array_sum = 0;
int ii = 0;

//*********PID*********//
double Input_T, Output_T; // Tip
double Input_R, Output_R; // Reservoir
double Setpoint = 75;          // Temperature setpoint

// Define the aggressive and conservative Tuning Parameters
double threshold = 5; // soglia passaggio PID tra tuning aggressivo e conservativo
double aggKp = 80, aggKi = 0, aggKd = 0;
double consKp = 80, consKi = 15, consKd = 0;

// Define extruder object
AccelStepper E0axis(AccelStepper::DRIVER, E0_STEP_PIN, E0_DIR_PIN);

// Define target position variables
int PLC_position_x, PLC_position_y, PLC_position_z = 0;
double x_offset = 6, y_offset = 7;
double x_origin = 12.67, y_origin = 79.02; // x = 11.67, y = 78.52
double z_punching = 26;
double pos_x = 0, pos_y = 0;
double puncher_x = 0, puncher_y = 0;
double extruder_x = 0, extruder_y = 0;
double press_x = 0, press_y = 0;

// define light variables
int THRESHOLD_LIGHT = 15; // around 80 mV
int light_intensity = 0;
float voltage_light_read = 0;

// Temperature controller variable definition
PID myPID_T(&Input_T, &Output_T, &Setpoint, consKp, consKi, consKd, DIRECT); // T
PID myPID_R(&Input_R, &Output_R, &Setpoint, consKp, consKi, consKd, DIRECT); // R

// Laser state variables
bool laser_state = false;
bool last_laser_state = false;

// Gripper state variables
int gripper_state_PLC = 0;           //
int gripper_state_PLC_open = 0;      // returned by read register function
int gripper_state_PLC_closed = 0;    // returned by read register function
int gripper_state_error = 0;         // returned by read register function in monitor state
bool expected_gripper_state = false; // expected state
bool flag_debug = false;

extern int action;

/**
 * Functions declaration
 */
void initialization();
void initializeGripper();
void readMsg();
void sendMsg();
void serialHandler();
void Action();

void monitorState();

void checkTemperature();
void checkTemperatureFault(uint8_t, Adafruit_MAX31865);
void tempPIDControl();
void monitorTemp();

void checkCamera();
void askImaging();
void checkLight();
void checkLaser();
void checkRefill();
void setpointTransform();
void initalizeGripper();
void moveToCheckLaser();
void checkGripper(int);

void monitorLight(bool);
void monitorOnGUI();

void motorHoming();
void closeGripper();
void openGripper();

bool pressureRead();
bool pressureStatus(float);
bool lightRead();
bool lightStatus(float);

void Input();
void Reset();
void reconnect();

void timerHandler();
void checkPLCAction(int);

void tempAlarmTIP();
void tempAlarmRES();

byte bToggle(int, int);

#endif
