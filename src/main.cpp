#include "main.h"
///////////////////////////////////////////////////////

// SPISettings tempSettings(MSBFIRST,SPI_MODE0);
// SPISettings ethSettings(MSBFIRST,SPI_MODE0);

/**
 * Variables definition
 */
// Machine states and check-mask
byte mask = 0x0;
enum FiniteStateMachine : byte
{
  Init = 0x0C,          // 01100
  waitInsertion = 0x0E, // 01110
  indexing = 0x0F,      // 01111
  waitRemoval = 0x1E,   // 11110
  IDLE = 0x00
};
FiniteStateMachine State;
FiniteStateMachine Old_State;
// Timer variables
enum checkVariables : int
{
  Stop = 0,
  Jetson = 1,
  PLC = 2,
  // Holder = 1,
  // Block = 2,
};
checkVariables Check;
unsigned long monitoring_time = 0;
unsigned long prev_monitoring_time = 0;
unsigned long timer = 0;
unsigned long prev_timer = 0;
unsigned long time_extruder = 0;
unsigned long time_extruder_prev = 0;
// Flags
bool oneTime_action_state = true;
bool oneTime_action_step = true;
bool oneTime_action = true;
bool PID_On;

// bool flag_ready = false;
// bool flag_home = false;
bool flag_print = false;
bool flag_timer = false;
bool flag_jetson = false;
bool flag_prox = true;
bool flag_PLC_completed = false; // flag to check through modbusTCP
bool flag_laser_check = false;

bool flag_block_positioned = false; // flag to chesk presence of block

// Extruder variables
int E0_axis_speed = 3000;
int E0_axis_accel = 1500;
int extrusion_step = 1;
int extrusion_step_total = 4;
bool flag_extrusion_ended = false;

// State machine actions counter
int action = 1;
// Messaging variable to monitor sensor and states
uint16_t sensor_code = 0;
uint16_t sensor_code_old = 0;
///////////////////////////////////////////////////////

int time_screen = 100;

void setup()
{
  // put your setup code here, to run once:
  initialization();
}

void loop()
{
  ////////////////////////////////////////////////////////////////////////
  // TO TEST WITH ETHERNET
  // If the connection goes down try to reconnect
  if (Ethernet.linkStatus() != 1 || !mqtt_client.connected() || !modbusTCPClient.connected())
  {
   reconnect();
   connectMod();
  }
  if (flag_start)
  {
    monitoring_time = millis();
    // else
    //{
    //   mqtt_client.loop();
    Input();
    // Main loop - Monitor state, messaging and state machine actions performing
    // if (flag_start && !flag_error)
    //{
    if (monitoring_time - prev_monitoring_time > 1000UL)
    {
      prev_monitoring_time = monitoring_time;
      tempPIDControl();
      monitorState();
      // Serial.print(State);
      // Serial.print("\n");
      // Serial.print("ACTION: ");
      // Serial.print(action);
      // Serial.print("\n");
      // Serial.print(oneTime_action);
      // Serial.print("\n");
      Action();
    }
    // sendMsg();
    // incomingSerial();
    //  serialHandler();
    //  timerHandler();
    //}
    //}
  }
  else
  {
    Input();
    time_extruder = millis();
    if (time_extruder - time_extruder_prev > 1000)
    {
      tempPIDControl();
      monitorTemp();
      time_extruder_prev = time_extruder;
    }
  }
}

/**
 * @brief
 * Create the mask to be compared with the state and
 * log info about the current state
 */
void monitorState()
{

  // Monitor temperatura at tip and reservoir
  monitorTemp();

  // Monitor Light status
  bool expected_light_status = true;
  monitorLight(expected_light_status);

  // Monitor PLC status
  int PLC_state_ready = readInputRegisterBit(bi_ready);

  // Monitor Gripper
  int gripper_state_error = readInputRegisterBit(bi_GrError);
  // if (gripper_state_error != 0)
  //{
  //   Serial.println("Gripper error!!");
  // }

  // Log messages related to the current state
  switch (State)
  {

  case Init:
    if (oneTime_action_state)
    {
    }
    break;

  case waitInsertion:
    if (oneTime_action_state)
    {
      Serial.println("-------------------------------------");
      Serial.println("WAIT FOR BLOCK INSERTION");
      Serial.println("-------------------------------------");
      delay(time_screen);
      // Serial.print("Mask: ");
      // Serial.print(mask, BIN);
      // Serial.print("\t");
      // Serial.print("State: ");
      // Serial.println(State, BIN);
      oneTime_action_state = false;
      flag_print = true;
    }

    // readInputRegisterBit(bi_GrIsOpen);
    // expected_gripper_state = 1; // OPEN
    checkGripper(expected_gripper_state);

    break;

  case indexing:
    if (oneTime_action_state)
    {
      Serial.println("-------------------------------------");
      Serial.println("STARTING AUTOMATIC INDEXING PROCEDURE");
      Serial.println("-------------------------------------");
      delay(time_screen);
      // Serial.print("Mask: ");
      // Serial.print(mask, BIN);
      // Serial.print("\t");
      // Serial.print("State: ");
      // Serial.println(State, BIN);
      oneTime_action_state = false;
      flag_print = true;
    }
    // expected_gripper_state = 0; // CLOSED
    checkGripper(expected_gripper_state);
    break;

  case waitRemoval:
    if (oneTime_action_state)
    {
      Serial.println("-------------------------------------");
      Serial.println("WAIT FOR BLOCK REMOVAL");
      Serial.println("-------------------------------------");
      delay(time_screen);
      // Serial.print("Mask: ");
      // Serial.print(mask, BIN);
      // Serial.print("\t");
      // Serial.print("State: ");
      // Serial.println(State, BIN);
      oneTime_action_state = false;
      flag_print = true;
    }
    // expected_gripper_state = 1; // OPEN
    checkGripper(expected_gripper_state);

    break;

  default:
    break;

    // Send telemetry to TB Interface
    // monitorOnGUI();
  }
  statusMsg();
}

void initialization()
{
  // Initialize Serial, Serial1 and set a timeout
  // for Serial1 communication
  Serial.begin(115200);
  // while (!Serial)
  //{
  // }
  delay(1000);
  Serial1.begin(9600);
  Serial1.setTimeout(10000);

  // Set lase and light pin
  pinMode(PIN_LIGHT, OUTPUT);
  pinMode(PIN_LASER, INPUT);

  // set heater pin
  pinMode(PIN_TIP_HEATER, OUTPUT); // pin Punta
  pinMode(PIN_RES_HEATER, OUTPUT); // pin Serbatoio

  max_T.begin(MAX31865_2WIRE);
  max_R.begin(MAX31865_2WIRE);

  // Set pins initial state
  digitalWrite(PIN_LIGHT, HIGH);

  State = Init;

  ////////////////////////////////////////////////////////////////////////
  // TO TEST WITH INTERNET
  // Initialize Ethernet connection and connect
  // the client to Thingsboard
  initEthernet();
  initTBClient();
  connectMod();
  ////////////////////////////////////////////////////////////////////////

  // Set extruder controller
  pinMode(E0_ENABLE_PIN, OUTPUT);
  E0axis.setMaxSpeed(E0_axis_speed);
  E0axis.setAcceleration(E0_axis_accel);
  E0axis.setSpeed(1000);

  // set temperature controller
  // turn the PID on
  myPID_T.SetMode(AUTOMATIC);
  myPID_R.SetMode(AUTOMATIC);

  pinMode(PIN_CS_ETHERNET, OUTPUT);
  //  pinMode(49, OUTPUT);
  //  pinMode(53, OUTPUT);

  SPI.end();

  // Set flag and interface buttons to the initial state
  // flag_home = false;
  // flag_cut = false;
  // flag_ready = false;
  // flag_start = true;
  // mqtt_client.publish("v1/devices/me/attributes", "{\"home\":\"false\"}");
  // mqtt_client.publish("v1/devices/me/attributes", "{\"cut\":\"false\"}");
  // mqtt_client.publish("v1/devices/me/attributes", "{\"ready\":\"false\"}");
  // mqtt_client.publish("v1/devices/me/attributes", "{\"button\":\"false\"}");
}

/**
 * @brief
 * Finite state machine function - Manage the action
 * to be done in relation to the current state and action
 * value
 */
void Action()
{
  switch (State)
  {

  case IDLE:
    Serial.println("Idle");
    delay(1000);
    break;

  case Init:
    switch (action)
    {
    case 1: // check temperature and wake up jetson
      delay(time_screen);
      if (oneTime_action_step)
      {
        // DEBUG
        Serial.println("Step 1: Initialization");
        oneTime_action_step = false;
        //
      }

      //checkTemperature();
      checkLight();
      checkCamera();

      action += 1;
      oneTime_action_step = true;
      break;

    case 2: // wait jetson to wake up
      if (oneTime_action_step)
      {
        // DEBUG
        Serial.println("Step 2: Wake up Jetson!");
        oneTime_action_step = false;

        //////////////////////////////////////
        // Send 'Wake Up' Message to Jetson
        // outputJson["key"] = "ATRMController";
        // outputJson["value"] = "WakeUp";
        // outgoingSerial();
        // Check = Jetson;
        // prev_timer = millis();
        //////////////////////////////////////
      }

      //////////////////////////////////////
      // TO REMOVE WHEN COMMUNICATION IS UP
      flag_jetson = true;
      /////////////////////////////////////

      if (flag_jetson)
      {
        // If jetson answered go to next action
        Serial.println("Jetson is awake");
        Check = Stop;
        action += 1;
        flag_jetson = false;
        oneTime_action_step = true;
      }
      break;

    case 3: // motor homing
      if (oneTime_action_step)
      {
        Serial.println("Step 3: Motor homing");
        motorHoming();
        delay(500);
        initializeGripper();
        oneTime_action_step = false;
      }

      // while (1)
      //{
      //   Serial.println("BLOCK DEBUG!");
      //   delay(1000);
      // }

      checkPLCAction(bi_ready);

      if (flag_PLC_completed)
      {
        Serial.println("Motor at home!");
        action += 1;
        oneTime_action_step = true;
        flag_PLC_completed = false;
        Check = Stop;
      }

      break;

    case 4: // check laser
      if (oneTime_action_step)
      {
        Serial.println("Step 4: Check laser ...");
        delay(time_screen);
        moveToCheckLaser();
        oneTime_action_step = false;
      }

      // checkPLCAction();
      flag_PLC_completed = true;

      if (flag_PLC_completed)
      {
        // Serial.println("Move completed!");
        // delay(time_screen);
        checkLaser();
        if (flag_laser_check == true)
        {
          // Serial.println("Laser checked!");
          // delay(time_screen);
          flag_laser_check = false;
          action += 1;
          oneTime_action_step = true;
          flag_PLC_completed = false;
        }
      }
      break;

    case 5: // check Gripper DA SISTEMARE
      if (oneTime_action_step)
      {
        Serial.println("Step 5: Check gripper ...");
        delay(time_screen);
        oneTime_action_step = false;
        // initializeGripper();
      }

      // checkPLCAction();
      flag_PLC_completed = true;

      if (flag_PLC_completed)
      {
        Serial.println("Gripper initialized!");
        delay(time_screen);
        // Se il PLC non da risposte qui proseguiamo il ciclo o mettiamo un time?
        action = 1;
        oneTime_action_step = true;
        flag_PLC_completed = false;
        State = waitInsertion;
        oneTime_action_state = true;
      }

      break;
    }
    break;

  case waitInsertion:
    switch (action)
    {
    case 1: // ask for insertion
      if (oneTime_action_step)
      {
        Serial.println("Step 1: Asking for insertion");
        delay(time_screen);
        oneTime_action_state = false;
        prev_timer = millis();
        ////////////////////////////////////////////////////////////////////////////////
        // httpRequest("/msgs_atrm/", String(12002)); // FOR DEBUG
        ////////////////////////////////////////////////////////////////////////////////
        // Serial.println("Wait insertion ... ");
        // delay(time_screen);
      }

      ////////////////////////////////////////////////////////////////////////////////
      // FOR DEBUG!!!
      flag_ready = true;
      ////////////////////////////////////////////////////////////////////////////////

      if (flag_ready)
      {

        Serial.println("Block inserted!");
        delay(time_screen);
        action += 1;
        oneTime_action_step = true;
      }

      break;
    case 2:
      if (oneTime_action_step)
      {
        Serial.println("Closing gripper ...");
        delay(time_screen);
        closeGripper();

        oneTime_action_step = true;
      }

      checkGripper(expected_gripper_state);
      flag_block_positioned = true;

      if (flag_block_positioned) // Chiedere a Dario di come avviene questa conferma
      {
        Serial.println("Block in position!");
        delay(time_screen);
        action = 1;
        oneTime_action_step = true;
        oneTime_action_state = true;
        State = indexing;
      }
      break;
    }

    break;

  case indexing:
    switch (action)
    {

    case 1:
      if (oneTime_action_step)
      {
        oneTime_action_state = false;
        Serial.println("Step 1: Requesting imaging analysis ...");
        delay(time_screen);
        oneTime_action_step = false;
        askImaging();
        Check = Jetson;
        prev_timer = millis();

        //////////////////////////////////////
        // Send 'CheckID' Message to Jetson
        // outputJson["key"] = "ATRMController";
        // outputJson["value"] = "CheckID";
        // outputJson["BlockCode"] = "111111";
        // outgoingSerial();
        // Check = Jetson;
        // prev_timer = millis();
        //////////////////////////////////////
      }

      //////////////////////////////////////
      // TO REMOVE WHEN COMMUNICATION IS UP
      flag_jetson = true;
      /////////////////////////////////////

      if (flag_jetson)
      {
        // If jetson answered go to next action
        Serial.println("Jetson confirmed block in position and ready to process ...");
        delay(time_screen);
        Check = Stop;
        action += 1;
        flag_jetson = false;
        oneTime_action_step = true;
      }
      break;

    case 2:
      if (oneTime_action_step)
      {
        Serial.println("Step 2: Punching ...");
        delay(time_screen);
        oneTime_action_step = false;
        // Transform target position
        // PLC_position_x = pos_x + puncher_x; // TO DO: Aggiungere conversioni e rotazione frame
        // PLC_position_y = pos_y + puncher_y;

        x_Position = 14.09;
        y_Position = 60;
        z_Position = 26.5;

        writeSetPoint();
        delay(2000);
        writeCmdUp(cmd_Register, bh_punching, true);
        flag_PLC_completed = false;

        // Send target position to PLC

        Serial.println("Moving the puncher ...");
        delay(time_screen);
      }

      checkPLCAction(bi_PunchDone);
      // flag_PLC_completed = true;

      if (flag_PLC_completed)
      {
        Serial.println("Punching ended, checking laser ...");
        checkLaser();
        delay(time_screen);
        if (flag_laser_check)
        {
          Serial.println("Punching complete!");
          delay(time_screen);
          flag_laser_check = false;
          flag_PLC_completed = false;
          action += 1;
          oneTime_action_step = true;
          /* code */
        }
      }

      break;

    case 3:

      if (oneTime_action_step)
      {
        Serial.println("Step 5: Starting the refilling phase ...");
        delay(time_screen);
        oneTime_action_step = false;
        // Transform target position
        // PLC_position_x = pos_x + extruder_x; // TO DO: Aggiungere conversioni e rotazione frame
        // PLC_position_y = pos_y + extruder_y;

        // Send target position to PLC
        writeCmdUp(cmd_Register, bh_refilling, true);

        Serial.println("Moving the extruder in position ...");
        delay(time_screen);
      }

      checkPLCAction(bi_RefillingDone);
      // flag_PLC_completed = true;

      if (flag_PLC_completed)
      {
        Serial.println("Extruder in position ...");
        delay(time_screen);
        flag_PLC_completed = false;
        action += 1;
        oneTime_action_step = true;
      }

      break;

      // case 6:

      // //   if (oneTime_action_step)
      // //   {
      // //     Serial.println("Step 6: Move the extruder down ...");
      // //     delay(time_screen);
      // //     oneTime_action_step = false;
      // //     // Transform target position
      // //     PLC_position_z = 0; // TO DO: Aggiungere conversioni e rotazione frame

      // //     // Send target position to PLC

      // //     Serial.println("Moving the extruder in position ...");
      // //     delay(time_screen);
      //   }

      //   checkPLCAction();

      //   if (flag_PLC_completed)
      //   {
      //     Serial.println("Extruder in position ...");
      //     delay(time_screen);
      //     flag_PLC_completed = false;
      //     action += 1;
      //     oneTime_action_step = true;
      //     Serial.println("Ready to extrude!");
      //     delay(time_screen);
      //   }

      //   break;

    case 4:

      if (oneTime_action_step)
      {
        Serial.println("Step 4: Extrude ...");
        delay(time_screen);
        oneTime_action_step = false;

        // Send target position to PLC
        writeCmdUp(cmd_Register, bh_extrusion, true);

        Serial.println("Extruding ...");
        delay(time_screen);
        Serial.println(extrusion_step_total);
        Serial.println("Extruding ...");
        delay(time_screen);
        Serial.println(flag_PLC_completed);
        Serial.println(flag_extrusion_ended);
        Serial.println(extrusion_step);
        Serial.println("Extruding ...");
      }

      if (extrusion_step < extrusion_step_total)
      {
        writeCmdUp(cmd_Register, bh_extrusion, true);
        checkPLCAction(bi_ExtrusionDone);
        Serial.println(flag_PLC_completed);
      }

      if (flag_PLC_completed && flag_extrusion_ended == false)
      {
        Serial.print("Extrusion Number: ");
        Serial.println(extrusion_step);
        delay(time_screen);

        // int number_steps_E0 = -14000; //-14000 //segno MENO per estrusione //segno PIU per SUCK_BACK
        // E0axis.move(number_steps_E0);
        // E0axis.runToPosition();
        flag_extrusion_ended = true;
      }

      // send message to plc to move up FOR EXTRUSION
      if (flag_PLC_completed)
      {
        extrusion_step += 1;
        flag_extrusion_ended = false;
        flag_PLC_completed = false;
      }

      if (extrusion_step_total - extrusion_step == 0)
      {
        Serial.println("Extrusion ended!");
        delay(time_screen);
        extrusion_step = 1;
        flag_PLC_completed = false;
        action += 1;
        oneTime_action_step = true;
      }

      break;

    case 5:

      if (oneTime_action_step)
      {
        Serial.println("Step 5: Compression phase and finish process...");
        delay(time_screen);
        oneTime_action_step = false;
        // Transform target position
        // PLC_position_x = pos_x + press_x; // TO DO: Aggiungere conversioni e rotazione frame
        // PLC_position_y = pos_y + press_y;

        // Send target position to PLC
        writeCmdUp(cmd_Register, bh_compression, true);
        delay(1000);

        // Serial.println("Wax compression ...");
        // Serial.println("Cleaning ...");
        // Serial.println("Let's go home!");
        // motorHoming();
        // delay(time_screen);
      }

      checkPLCAction(bi_CompressionDone);
      // flag_PLC_completed = true;

      if (flag_PLC_completed)
      {
        Serial.println("Motor at home ...");
        delay(time_screen);
        flag_PLC_completed = false;
        action = 1;
        oneTime_action_step = true;
        oneTime_action_state = true;
        State = waitRemoval;
      }

      break;
      // case 3:
      //   if (oneTime_action)
      //   {
      //     Serial.println("Case 2");
      //     oneTime_action = false;
      //   }
      //   if (flag_jetson)
      //   {
      //     // If jetson replies, go to the next state
      //     // Activate the vacuum and start to cut
      //     Check = Stop;
      //     State = Cut;
      //     action = 1;
      //     delay(1000);
      //     flag_cutEnabled = true;
      //     digitalWrite(cut1, LOW);
      //     digitalWrite(cut2, LOW);
      //     delay(100);
      //     digitalWrite(cut1, HIGH);
      //     digitalWrite(cut2, HIGH);
      //     digitalWrite(vacuum, HIGH);
      //     flag_jetson = false;
      //     oneTime_action = true;
      //   }
      break;
    }
    break;

  case waitRemoval:
    switch (action)
    {
    case 1:
      if (oneTime_action_step)
      {
        oneTime_action_state = false;
        Serial.println("Step 1: Wait removal ...");
        Serial.println("Requesting confirm of process correctly executed by Jetson ...");
        delay(time_screen);
        oneTime_action_step = false;

        //////////////////////////////////////
        // Send 'CheckID' Message to Jetson
        // outputJson["key"] = "ATRMController";
        // outputJson["value"] = "CheckRefill";
        // outgoingSerial();
        // Check = Jetson;
        // prev_timer = millis();
        //////////////////////////////////////
      }

      //////////////////////////////////////
      // TO REMOVE WHEN COMMUNICATION IS UP
      flag_jetson = true;
      /////////////////////////////////////

      if (flag_jetson)
      {
        // If jetson answered go to next action
        Serial.println("Jetson confirmed that the process is terminated and correctly executed ...");
        delay(time_screen);
        Check = Stop;
        action += 1;
        flag_jetson = false;
        oneTime_action_step = true;
      }
      break;

    case 2:
      if (oneTime_action_step)
      {
        Serial.println("Step 2: Wait Removal");
        delay(time_screen);
        oneTime_action_step = false;
        oneTime_action_state = false;
        delay(1000);
        openGripper();

        prev_timer = millis();
        // sensor_code = 12001;
        //  'Holder to home' message
        // httpRequest("/msgs_atrm/", String(12001));
      }

      checkPLCAction(bi_GrIsOpen);

      //////////////////////////////////////////////////////
      flag_home = true;
      //////////////////////////////////////////////////////

      if (flag_home)
      {
        Check = Stop;
        State = waitInsertion;
        oneTime_action_step = true;
        oneTime_action_state = true;
        action = 1;
        flag_home = false;
        flag_ready = false;
        flag_start = false;
        Serial.println("Process finished");
        /* code */
      }

      break;

      // case 2:
      //   if (oneTime_action)
      //   {
      //     Serial.println("Case 5");
      //     oneTime_action = false;
      //   }
      //   if (flag_home)
      //   {
      //     // If holder at home go to the 'Wait insertion' state
      //     // and set mask, flags and buttons to the related state
      //     mask &= ~(1 << 4);
      //     Check = Stop;
      //     State = waitInsertion;
      //     action = 1;
      //     flag_home = false;
      //     flag_cut = false;
      //     flag_ready = false;
      //     flag_start = false;
      //     mqtt_client.publish("v1/devices/me/attributes", "{\"home\":\"false\"}");
      //     mqtt_client.publish("v1/devices/me/attributes", "{\"cut\":\"false\"}");
      //     mqtt_client.publish("v1/devices/me/attributes", "{\"ready\":\"false\"}");
      //     mqtt_client.publish("v1/devices/me/attributes", "{\"button\":\"false\"}");
      //     oneTime_action = true;
      //   }
      // break;
    }
    break;
  }
}

void checkTemperature()
{
  Serial.println("Checking temperature sensors ...");
  delay(time_screen);

  ////////////////////////////////////////////////////////////
  // TO TEST WITH SENSOR
  // Initialize Tip sensor
  max_T.begin(MAX31865_2WIRE); // Punta

  // Check and print any faults
  uint8_t fault_T = max_T.readFault();
  checkTemperatureFault(fault_T, max_T);

  if (fault_T == 0) // TO CHECK TESTING!
  {
    Serial.println("Tip sensor OK!");
  }

  // Initialize Reservoir sensor
  max_R.begin(MAX31865_2WIRE);

  // Check and print any faults
  uint8_t fault_R = max_T.readFault();
  checkTemperatureFault(fault_R, max_R);

  if (fault_R == 0)
  {
    Serial.println("Reservoir sensor OK!");
  }
  ////////////////////////////////////////////////////////////
}

void checkTemperatureFault(uint8_t fault, Adafruit_MAX31865 max)
{
  if (fault)
  {
    Serial.print("Temperature sensor fault 0x");
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH)
    {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH)
    {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW)
    {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH)
    {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW)
    {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV)
    {
      Serial.println("Under/Over voltage");
    }
    max.clearFault();
    sensor_code = 11701;
  }
}

void monitorTemp()
{
  // ////////////////////////////////////////////////////////
  // // DEBUG
  // // LETTURA TEMPERATURA
  // // digitalWrite(PIN_CS_ETHERNET, HIGH); // Disable Ethernet SPI

  // // SPI.end();
  // //
  // // max_T.begin(MAX31865_2WIRE);
  // // max_R.begin(MAX31865_2WIRE);

  // // temp_T = max_T.temperature(100, RREF);
  // Serial.print("Temperature_T =");
  // Serial.print(" ");
  // Serial.print(temp_T);
  // Serial.println(" ");

  // // temp_R = max_R.temperature(100, RREF);
  // Serial.print("Temperature_R =");
  // Serial.print(" ");
  // Serial.print(temp_R);
  // Serial.println(" ");

  // // checkTemperature(); // Chiedere a Dario a cosa serve (Forse impalla il codice)

  // // temp_T = 40;
  // // temp_R = 40;
  // tempAlarmRES();
  // tempAlarmTIP();
  // // temp_array_T[ii] = temp_T;
  // // temp_array_R[ii] = temp_R;
  // // ii = ii + 1;

  // // if (ii > 9)
  // // {
  // //   ii = 0;
  // // }

  // // tempAlarm();

  // // digitalWrite(PIN_CS_ETHERNET, HIGH); // Enable Ethernet SPI
  // ////////////////////////////////////////////////////////

  Serial.print("Time=");
  Serial.print(" ");
  Serial.print(time_extruder);
  Serial.print(" ");
  Serial.print("Tatget_Temperature=");
  Serial.print(" ");
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.print("Temperature_T =");

  Serial.print("Output_T_on=");
  Serial.print(" ");
  Serial.print(Output_T);
  Serial.print(" ");

  Serial.print("Output_R_on=");
  Serial.print(" ");
  Serial.print(Output_R);
  Serial.println(";");

  Serial.print("Temperature_T =");
  Serial.print(" ");
  Serial.print(temp_T);
  Serial.print(" ");

  Serial.print("\t");

  Serial.print("Temperature_R =");
  Serial.print(" ");
  Serial.print(temp_R);
  Serial.println(" ");

  // checkTemperature(); // Chiedere a Dario a cosa serve (Forse impalla il codice)

  // temp_T = 40;
  // temp_R = 40;
  tempAlarmRES();
  tempAlarmTIP();
}

void tempAlarmTIP()
{
  static int count;
  static double temp_old;

  if (temp_T == temp_old)
  {
    count++;
  }
  else
  {
    count = 0;
  }

  if (count >= 100)
  {
    Serial.println("Alarm! Check Tip temperature!");
    PID_On = false;
  }

  temp_old = temp_T;
}

void tempAlarmRES()
{
  static int count;
  static double temp_old;

  if (temp_R == temp_old)
  {
    count++;
  }
  else
  {
    count = 0;
  }

  if (count >= 100)
  {
    Serial.println("Alarm! Check Reservoir temperature!");
    PID_On = false;
  }

  temp_old = temp_R;
}

void tempPIDControl()
{
  SPI.end();
  max_T.begin(MAX31865_2WIRE);
  max_R.begin(MAX31865_2WIRE);

  // LETTURA TEMPERATURA
  temp_T = max_T.temperature(100, RREF); // Punta
  temp_R = max_R.temperature(100, RREF); // Serbatoio

  Output_T = 0;
  Output_R = 0;

  //////////////////////////////////////////////////////////////////////
  // Control loop
  if (PID_On)
  {
    Serial.println("-------------------");
    Serial.println("PID ON");
    Serial.println("-------------------");
    analogWrite(10, 0); // Punta
    analogWrite(7, 0);  // Serbatoio
    //*********** ADAPTIVE PID ***********
    // Punta
    Input_T = temp_T;
    double gap_T = abs(Setpoint - Input_T);
    if (gap_T < threshold)
    { // we're close to setpoint, use conservative tuning parameters
      myPID_T.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      // we're far from setpoint, use aggressive tuning parameters
      myPID_T.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID_T.Compute();
    analogWrite(10, Output_T); // Punta

    // Serbatoio
    Input_R = temp_R;
    double gap_R = abs(Setpoint - Input_R);
    if (gap_R < threshold)
    { // we're close to setpoint, use conservative tuning parameters
      myPID_R.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      // we're far from setpoint, use aggressive tuning parameters
      myPID_R.SetTunings(aggKp, aggKi, aggKd);
    }

    myPID_R.Compute();
    analogWrite(7, Output_R); // Serbatoio
  }

  else if (!PID_On) // PID SPENTO
  {
    Serial.println("-------------------");
    Serial.println("PID OFF");
    Serial.println("-------------------");
    analogWrite(10, 0); // Punta
    analogWrite(7, 0);  // Serbatoio
  }
  //////////////////////////////////////////////////////////////////////
}

void checkCamera()
{
  Serial.println("Checking camera ...");
  delay(time_screen);
  outputJson["key"] = "IDXController";
  outputJson["value"] = "WakeUp";
  outgoingSerial();
}

void askImaging()
{
  outputJson["key"] = "IDXController";
  outputJson["value"] = "CheckID"; //
  outgoingSerial();
}

void checkLight()
{
  Serial.println("Turning on lights ...");
  delay(time_screen);
  digitalWrite(PIN_LIGHT, HIGH); // turn on the light and go on
  Serial.println("Checking lights ...");
  delay(time_screen);

  light_intensity = analogRead(PIN_CURRENT_LIGHT);
  voltage_light_read = light_intensity;
  if (voltage_light_read < THRESHOLD_LIGHT)
  {
    Serial.println("Light is turned off ...");
    delay(time_screen);
    sensor_code = 11703; // ERROR MANAGING
  }
}

void monitorGripper(bool expected_gripper_status)
{
}

void monitorLight(bool expected_light_status)
{

  light_intensity = analogRead(PIN_CURRENT_LIGHT);
  voltage_light_read = light_intensity;

  // Serial.println("DEBUG");
  // Serial.println(light_intensity);
  // Serial.println(voltage_light_read);
  // Serial.println("DEBUG");
  // delay(1000);

  if (expected_light_status)
  {
    if (voltage_light_read < THRESHOLD_LIGHT)
    {
      Serial.println("Expected light status ON ...");
      Serial.println("Light OFF ...");
      sensor_code = 11703;
    }
    else
    {
      Serial.println("Expected light status ON ...");
      Serial.println("Light ON ...");
      sensor_code = 11703;
    }
  }
  else
  {
    if (voltage_light_read < THRESHOLD_LIGHT)
    {
      Serial.println("Expected light status OFF ...");
      Serial.println("Light OFF ...");
      sensor_code = 11703;
    }
    else
    {
      Serial.println("Expected light status OFF ...");
      Serial.println("Light ON ...");
      sensor_code = 11703;
    }
  }
}

void motorHoming()
{
  Serial.println("Let's go home ...");
  slave_command = bitUp(slave_command, bh_init);
  writeToReg(cmd_Register, slave_command);
  delay(500);
  // Riabbasso il bit di comando
  slave_command = bitDown(slave_command, bh_init);
  writeToReg(cmd_Register, slave_command);
  Serial.println("Homing done");
}

void moveToCheckLaser()
{
  Serial.println("Let's move to check Laser ...");
  // Da agguiungere il comando di move da inviare al PLC per effettuare il check del laser
  laser_state = digitalRead(PIN_LASER);
  last_laser_state = laser_state;
}

void checkLaser()
{
  Serial.println("Let's check Laser after move ...");
  delay(time_screen);
  laser_state = digitalRead(PIN_LASER);
  ////////////////////////////////////////////////////////////////////////
  if (1) // FOR DEBUG
  {
    // if (laser_state != last_laser_state)
    // {
    flag_laser_check = true;
    Serial.println("Laser catch a hole!");
    delay(time_screen);
    // }
    //////////////////////////////////////////////////////////////////////
  }
  last_laser_state = laser_state;
}

void closeGripper()
{
  Serial.println("Let's close the gripper ...");

  writeToReg(cmd_Register, bitUp(slave_command, bh_GRclose));
  delay(1000);
  // Riabbasso il bit di comando

  writeToReg(cmd_Register, bitDown(slave_command, bh_GRclose));
  delay(1000);
  // Serial.println("Gripper Closed!");
  expected_gripper_state = 0;
}

void openGripper()
{
  Serial.println("Let's open the gripper ...");
  writeToReg(cmd_Register, bitUp(slave_command, bh_GRopen));
  delay(1000);
  // Riabbasso il bit di comando

  writeToReg(cmd_Register, bitDown(slave_command, bh_GRopen));
  delay(1000);
  // Serial.println("Gripper Open!");
  expected_gripper_state = 1;
}

void checkGripper(int expected_gripper_state)
{
  // CLOSED STATE -> gripper_state_PLC = 0
  // OPEN STATE -> gripper_state_PLC = 1
  // UNDEFINED STATE -> gripper_state_PLC = 2
  // leggere lo stato del gripper ed impostarlo su gripper_state_PLC
  gripper_state_PLC_open = readInputRegisterBit(bi_GrIsOpen);
  gripper_state_PLC_closed = readInputRegisterBit(bi_GrIsClosed);

  if (gripper_state_PLC_open == 1 && gripper_state_PLC_closed == 0)
  {
    gripper_state_PLC = 1;
  }
  if (gripper_state_PLC_open == 0 && gripper_state_PLC_closed == 1)
  {
    gripper_state_PLC = 0;
  }
  if (gripper_state_PLC_open == 0 && gripper_state_PLC_closed == 0) // undefined is closed
  {
    gripper_state_PLC = 0;
  }

  switch (gripper_state_PLC)
  {
  case 0:
    if (expected_gripper_state == 0)
    {
      Serial.println("Gripper closed!");
      flag_block_positioned = false;
    }
    if (expected_gripper_state == 1)
    {
      Serial.println("Gripper error!  Gripper should be open");
    }
    break;

  case 1:
    if (expected_gripper_state == 0)
    {
      Serial.println("Gripper error! Gripper should be closed ");
    }
    if (expected_gripper_state == 1)
    {
      Serial.println("Gripper open!");
    }
    break;

  default:
    break;
  }
}

void monitorOnGui()
{
  // Manage the telemetry of data to thingsboard
  sendTelemetry("Light", analogRead(PIN_CURRENT_LIGHT));
  sendTelemetry("Laser", digitalRead(PIN_LASER));
  sendTelemetry("Temp T", temp_T);
  sendTelemetry("Temp R", temp_R);
}

void initializeGripper()
{
  Serial.println("Gripper inizialization ... ");
  writeToReg(cmd_Register, bitUp(slave_command, bh_GRInit));
  delay(500);
  // Riabbasso il bit di comando

  writeToReg(cmd_Register, bitDown(slave_command, bh_GRInit));
  Serial.println("Gripper Open!");
  expected_gripper_state = 1;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Message and Timer handler

void timerHandler()
{
  switch (Check)
  {
  case Stop:
    timer = 0;
    prev_timer = 0;
    break;

  case PLC:
    timer = millis();
    if (timer - prev_timer > 100000UL) ////////// DA DEFINIRE CON TEST!!!
    {
      sensor_code = 11501;
      Check = Stop;
    }
    break;

  case Jetson:
    timer = millis();
    if (timer - prev_timer > 5000UL)
    {
      sensor_code = 11501;
      Check = Stop;
      oneTime_action = true;
    }
    break;
  }
}

/**
 * @brief
 * Choose which action to do based on incoming
 * messages from Jetson
 */
void serialHandler()
{
  if (inputJson.isNull() || inputJson.size() < 2)
  {
    return;
  }
  String key = String((const char *)inputJson["key"]);
  String value = String((const char *)inputJson["value"]);
  if (key.equals("IASoftware"))
  {
    if (value.equals("ON"))
    {
      Serial.println("ON");
      flag_jetson = true;
    }
    else if (value.equals("READY"))
    {
      Serial.println("READY");
      x_Position = atof(((const char *)inputJson["Pos_x"]));
      y_Position = atof((const char *)inputJson["Pos_y"]);
      z_Position = atof((const char *)inputJson["Pos_z"]);
      flag_jetson = true;
    }
    else if (value.equals("Refill OK"))
    {
      Serial.println("Refill OK");
      flag_jetson = true;
    }
    // Error message from Jetson
    else if (value.equals("ERROR"))
    {
      sensor_code = (uint16_t)inputJson["code"]; // Il sensor_code va cambiato tramite il sistema di priorità progettato nel block diagram
      Serial.print("ERROR: ");
      Serial.println(sensor_code);
    }
    // Warning from jetson
    else if (value.equals("WARNING"))
    {
      sensor_code = (uint16_t)inputJson["code"];
      Serial.print("WARNING: ");
      Serial.println(sensor_code);
    }
  }
  inputJson.clear();
}

void checkPLCAction(int bit)
{
  // Leggi il bit da PLC e
  if (readInputRegisterBit(bit))
  {
    Serial.println("PLC completed");
    delay(time_screen);
    flag_PLC_completed = true;
  }
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief
 * Toggle a bit into the mask, if the relative pin is HIGH
 * @param bit - bit position into the mask
 * @param pin - pin to be verified
 * @return byte
 */
byte bToggle(int bit, int pin)
{
  if (digitalRead(pin))
  {
    mask |= (1 << bit);
  }
  else
  {
    mask &= ~(1 << bit);
  }
  return mask;
}

////////////////////////////////////////

/**
 * @brief
 * Tries to reconnect to Ethernet and Thingsboard
 */
void reconnect()
{
  // Loop until we're reconnected, both to Ethernet and then Thingsboard server
  while (!mqtt_client.connected())
  {
    if (!Ethernet.linkStatus())
    {
      Serial.println("Connecting to Ethernet");
      initEthernet();
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if (mqtt_client.connect("IDX controller", "IDX_MKS", NULL))
    {
      // Subscribing to receive RPC requests
      Serial.println("[DONE]");
      mqtt_client.subscribe("v1/devices/me/rpc/request/+");
      mqtt_client.subscribe("v1/devices/me/rpc/response/+");
    }
    else
    {
      Serial.print("[FAILED] [ rc = ");
      Serial.print(mqtt_client.state());
      Serial.println(" : retrying in 5 seconds]");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void Input()
{

  if (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {

    case 's':
      Serial.println("Start again");
      flag_start = !flag_start;
      State = Init;
      action = 1;
      break;

    case 'l':
      digitalWrite(PIN_LIGHT, !digitalRead(PIN_LIGHT));
      break;

    case 'i':
      Old_State = State;
      if (State == IDLE)
      {
        Serial.println("Returning to state machine");
        State = Old_State;
      }
      else
      {
        Serial.println("Becoming Idle");
        State = IDLE;
      }
      break;

    case 'p':
      PID_On = !PID_On;
      break;

    // Initialization
    case 'h':
      Serial.println("Let's go home ...");
      Serial.println(slave_command);
      slave_command = bitUp(slave_command, bh_init);
      Serial.println(slave_command);
      writeToReg(cmd_Register, slave_command);
      delay(500);
      // Riabbasso il bit di comando
      slave_command = bitDown(slave_command, bh_init);
      writeToReg(cmd_Register, slave_command);
      Serial.println("Homing done");
      /* constant-expression */
      /* code */
      break;

    // Disconnect/Reconnect modbus master
    case 'm':
      if (flag_debug)
      {
        flag_debug = false;
      }
      else
      {
        flag_debug = true;
        modbusTCPClient.end();
      }
      break;

    // Extrusion
    case 'e':
      int number_steps_E0 = -14000; //-14000 //segno MENO per estrusione //segno PIU per SUCK_BACK
      E0axis.move(number_steps_E0);
      E0axis.runToPosition();
      break;

    default:
      break;
    }
    /* code */
  }
}

void sendMsg() {}
void statusMsg() {}

// void tempAlarm()
//{
//   double temp_comparison = temp_array_R[0];
//   double zero_array_sum = 0;
//   double zero_array[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int i = 0;
//
//   for (i = 0; i < 10; i = i + 1)
//   {
//     zero_array[i] = temp_array_R[i] - temp_comparison;
//   }
//   for (i = 0; i < 9; i++)
//   {
//     zero_array_sum = zero_array_sum + zero_array[i + 1];
//   }
//
//   // Serial.println(zero_array_sum);
//
//   if (zero_array_sum == 0)
//   {
//     Serial.println("ALARM Reservoir! Check temperature");
//   }
//
//   temp_comparison = temp_array_T[0];
//   zero_array_sum = 0;
//   i = 0;
//
//   for (i = 0; i < 10; i = i + 1)
//   {
//     zero_array[i] = 0;
//     zero_array[i] = temp_array_T[i] - temp_comparison;
//   }
//   for (i = 0; i < 9; i++)
//   {
//     zero_array_sum = zero_array_sum + zero_array[i + 1];
//   }
//
//   // Serial.println(zero_array_sum);
//
//   if (zero_array_sum == 0)
//   {
//     Serial.println("ALARM TIP! Check temperature");
//   }
// }