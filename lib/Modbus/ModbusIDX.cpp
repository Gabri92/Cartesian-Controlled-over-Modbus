/**
 * Header
 */
#include "ModbusIDX.h"

ModbusTCPClient modbusTCPClient(client);
IPAddress server(192, 168, 1, 51);

float fposition;
uint16_t position_to_send;
uint16_t slave_command;
uint16_t slave_status;
bool power_on = false;

double x_Position;
double y_Position;
double z_Position;

uint16_t x_feedback;
uint16_t y_feedback;
uint16_t z_feedback;

/**
 * @brief
 * Conversione del float ad intero ed in seguito a word (uint16_t)
 * @param fnum
 * @return uint16_t
 */
uint16_t float_to_word(float fnum)
{
  uint16_t wnum = 0;
  int inum = fnum * 100;
  byte high = 0;
  byte low = 0;

  high = high | (inum >> 8);
  low = low | inum;

  wnum = (wnum | high) << 8;
  wnum = wnum | low;

  return wnum;
}

/**
 * @brief
 * Alzo il bit 'position' nel registro 'reg' e restituisco il registro aggiornato
 * @param reg
 * @param position
 * @return uint16_t
 */
uint16_t bitUp(uint16_t reg, int position)
{
  uint16_t temp;

  temp = 1 << position;
  reg = reg | temp;

  return reg;
}

/**
 * @brief
 * Abbasso il bit 'position' nel registro 'reg' e restituisco il registro aggiornato
 * @param reg
 * @param position
 * @return uint16_t
 */
uint16_t bitDown(uint16_t reg, int position)
{
  uint16_t temp;

  temp = 1 << position;
  reg = reg & ~(temp);

  return reg;
}

void writeToReg(int reg, uint16_t command)
{
  if (!modbusTCPClient.holdingRegisterWrite(reg, command))
  {
    Serial.print("Failed to write Register! ");
    Serial.println(modbusTCPClient.lastError());
  }
}

void clearReg(int reg)
{
  Serial.println("Debug");
  uint16_t clear = 0;
  if (!modbusTCPClient.holdingRegisterWrite(reg, clear))
  {
    Serial.print("Failed to write Register! ");
    Serial.println(modbusTCPClient.lastError());
  }
}

void connectMod()
{
  // client not connected, start the Modbus TCP client
  Serial.println("Attempting to connect to Modbus TCP server");

  if (!modbusTCPClient.begin(server, 502))
  {
    Serial.println("Modbus TCP Client failed to connect!");
    delay(2000);
  }
  else
  {
    Serial.println("Modbus TCP Client connected");
  }
}

void writeCmdUp(int reg, uint16_t command, bool toggle)
{
  // Bit up in the register
  slave_command = bitUp(slave_command, command);
  Serial.println(slave_command, BIN);
  if (!modbusTCPClient.holdingRegisterWrite(reg, slave_command))
  {
    Serial.print("Failed to write Register! ");
    Serial.println(modbusTCPClient.lastError());
  }
  // If the toggle flag is true, the bit is cleaned in the register after 500 ms
  if (toggle)
  {
    delay(1000);
    slave_command = bitDown(slave_command, command);
    Serial.println(slave_command, BIN);
    if (!modbusTCPClient.holdingRegisterWrite(reg, slave_command))
    {
      Serial.print("Failed to write Register! ");
      Serial.println(modbusTCPClient.lastError());
    }
  }
}

void writeCmdDown(int reg, uint16_t command)
{
  // Bit Down in the register
  slave_command = bitDown(slave_command, command);
  if (!modbusTCPClient.holdingRegisterWrite(reg, slave_command))
  {
    Serial.print("Failed to write Register! ");
    Serial.println(modbusTCPClient.lastError());
  }
}

void writeSetPoint()
{
  // Write X-axis SetPoint
  uint16_t setpoint_x = float_to_word(x_Position);
  if (!modbusTCPClient.holdingRegisterWrite(setpoint_X_Register, setpoint_x))
  {
    Serial.print("Failed to write Register! ");
    Serial.println(modbusTCPClient.lastError());
  }
  // Write Y-axis SetPoint
  uint16_t setpoint_y = float_to_word(y_Position);
  if (!modbusTCPClient.holdingRegisterWrite(setpoint_Y_Register, setpoint_y))
  {
    Serial.print("Failed to write Register! ");
    Serial.println(modbusTCPClient.lastError());
  }
  // Write Z-axis SetPoint
  uint16_t setpoint_z = float_to_word(z_Position);
  if (!modbusTCPClient.holdingRegisterWrite(setpoint_Z_Register, setpoint_z))
  {
    Serial.print("Failed to write Register! ");
    Serial.println(modbusTCPClient.lastError());
  }
}

void readEncoder()
{
  x_feedback = modbusTCPClient.inputRegisterRead(setpoint_X_Register);
  y_feedback = modbusTCPClient.inputRegisterRead(setpoint_Y_Register);
  z_feedback = modbusTCPClient.inputRegisterRead(setpoint_Z_Register);
}

uint16_t readInputRegisterBit(uint16_t status_bit)
{
  uint16_t slave_status = modbusTCPClient.inputRegisterRead(status_Register);
  return ((slave_status >> status_bit) & 1);
}
