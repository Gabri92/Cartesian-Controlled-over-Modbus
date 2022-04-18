#ifndef MODBUSIDX_H
#define MODBUSIDX_H

//////////////////////////////////////
// LIBRERIE
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "FastAPI.h"
//////////////////////////////////////

//////////////////////////////////////
// DEFINE DEI BIT E REGISTRI CONDIVISI
// Bit di comando
#define bh_init                0
#define bh_punching            1
#define bh_refilling           2
#define bh_extrusion           3
#define bh_compression         4
#define bh_GRInit              8
#define bh_GRopen              9
#define bh_GRclose             10 
#define bh_home                0
#define bh_moving              1
// Bit di stato    
#define bi_ready               0
#define bi_error               1
#define bi_PunchDone           2
#define bi_RefillingDone       3
#define bi_ExtrusionDone       4
#define bi_CompressionDone     5
#define bi_GrIsOpen            8
#define bi_GrIsClosed          9
#define bi_GrError            10
#define bi_GrSetDone          11
#define bi_GrResetDone        12
#define bi_movingDone         15
// Registri condivisi
#define setpoint_X_Register 0x0
#define setpoint_Y_Register 0x1
#define setpoint_Z_Register 0x2
#define cmd_Register        0x8
#define otherFunc_Register  0x9
#define feedback_X_Register 0x3E8
#define feedback_Y_Register 0x3E9
#define feedback_Z_Register 0x3EA
#define status_Register     0x3F0 // 1008
//////////////////////////////////////

/////////////////////////////////
// DICHIARAZIONE FUNZIONI
extern uint16_t float_to_word(float);
extern uint16_t bitUp(uint16_t,int);
extern uint16_t bitDown(uint16_t,int);
extern uint16_t readInputRegisterBit(uint16_t);
extern void writeToReg(int, uint16_t);
extern void setPosition(); // Da scrivere
extern void clearReg(int);
extern void connectMod();
extern void keyboardInput();

extern void writeCmdUp(int, uint16_t, bool);
extern void writeCmdDown(int, uint16_t);
extern void writeSetPoint();
extern void readEncoder();
/////////////////////////////////

/////////////////////////////////
// DICHIARAZIONE VARIABILI
extern float fposition;
extern uint16_t position_to_send;
extern uint16_t slave_command;
extern uint16_t slave_status;
extern bool power_on;
 
extern double x_Position;
extern double y_Position;
extern double z_Position;

extern uint16_t x_feedback;
extern uint16_t y_feedback;
extern uint16_t z_feedback;

// Define communication variable for MODBUS
//EthernetClient ethClient;

extern ModbusTCPClient modbusTCPClient;
extern IPAddress server;
/////////////////////////////////
#endif
