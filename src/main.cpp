#include "main.h"
///////////////////////////////////////////////////////

/**
 * Variables definition
 */
// Machine states and check-mask

// TEST PUNCH
/////////////////////////
int i = 18;
int l = 18;
int laser_check[20];
////////////////////////
int number_steps_E0 = 0;

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
unsigned long time_compression = 0;
unsigned long time_compression_prev = 0;
unsigned long time_imaging = 0;
unsigned long time_imaging_prev = 0;
// Flags
bool oneTime_action_state = true;
bool oneTime_action_step = true;
bool oneTime_action = true;
bool PID_On;
bool compressed = false;

// bool flag_ready = false;
// bool flag_home = false;
bool flag_print = false;
bool flag_timer = false;
bool flag_jetson = false;
bool flag_prox = true;
bool flag_noRefill = false;
bool flag_PLC_completed = false; // flag to check through modbusTCP
bool flag_laser_check = false;
bool flag_block_positioned = false; // flag to chesk presence of block
bool laser_debug = false;

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
  // for (l=0; l <= 19; l++){
  //   laser_check[l] = 2;
  // }
  // l = 0;
}

void loop()
{
  ////////////////////////////////////////////////////////////////////////
  // TO TEST WITH ETHERNET
  // If the connection goes down try to reconnect
  if (Ethernet.linkStatus() != 1 || !modbusTCPClient.connected())
  {
    reconnect();
    connectMod();
  }

  Input();

  if (flag_start)
  {
    //mqtt_client.loop();
    monitoring_time = millis();
    // Main loop - Monitor state, messaging and state machine actions performing
    if (monitoring_time - prev_monitoring_time > 1000UL)
    {
      prev_monitoring_time = monitoring_time;
      tempPIDControl();
      monitorState();
      Action();
    }
    // sendMsg();
    incomingSerial();
    serialHandler();
    // timerHandler();
  }
  else
  {
    time_extruder = millis();
    if (time_extruder - time_extruder_prev > 1000)
    {
      tempPIDControl();
      monitorTemp();
      time_extruder_prev = time_extruder;
      incomingSerial();
      serialHandler();
      Serial.print("X-Position: ");
      Serial.print(x_Position);
      Serial.print("\t");
      Serial.print("Y-Position: ");
      Serial.print(y_Position);
      Serial.print("\t");
      Serial.print("Z-Position: ");
      Serial.println(z_Position);
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
      Serial.println("-------------------------------------");
      Serial.println("INITIALIZATION");
      Serial.println("-------------------------------------");
      delay(time_screen);
      oneTime_action_state = false;
      flag_print = true;
    }
    break;

  case waitInsertion:
    if (oneTime_action_state)
    {
      Serial.println("-------------------------------------");
      Serial.println("WAIT FOR BLOCK INSERTION");
      Serial.println("-------------------------------------");
      delay(time_screen);
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
}

void initialization()
{
  // Initialize Serial, Serial1 and set a timeout
  // for Serial1 communication
  Serial.begin(115200);
  delay(1000);
  Serial2.begin(9600);
  Serial.setTimeout(10000);
  Serial2.setTimeout(10000);

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
  //initTBClient();
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


  SPI.end();
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
        // flag_jetson = false;
        //
      }

      // checkTemperature();
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
      }

      //////////////////////////////////////
      // TO REMOVE WHEN COMMUNICATION IS UP
      //flag_jetson = true;
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
        oneTime_action_step = false;
      }

      // checkPLCAction();
      flag_PLC_completed = true;

      if (flag_PLC_completed)
      {
        flag_laser_check = true;
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
        initializeGripper();
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
      }

      ////////////////////////////////////////////////////////////////////////////////
      // FOR DEBUG!!!
      //flag_ready = true;
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

      //checkGripper(expected_gripper_state);
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
        flag_jetson = false;
        time_imaging = millis();
        time_imaging_prev = millis();
      }

      time_imaging = millis();
      if(time_imaging - time_imaging_prev > 5000UL)
      {
        time_imaging_prev = time_imaging;
        askImaging();
      }

      //////////////////////////////////////
      // TO REMOVE WHEN COMMUNICATION IS UP
      //flag_jetson = true;
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

        // x_Position = x_vect[i];
        // y_Position = y_vect[i];
        setpointTransform();
        writeSetPoint();
        delay(1000);
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
        }
      }

      break;

    case 3:

      if (oneTime_action_step)
      {
        Serial.println("Step 5: Starting the refilling phase ...");
        delay(time_screen);
        oneTime_action_step = false;

        if(flag_noRefill)
        {
          x_Position = x_refilling;
          y_Position = y_refilling;
          writeSetPoint();
          writeCmdUp(otherFunc_Register, bh_anotherRefill, true);
          flag_PLC_completed = false;
          flag_noRefill = false;
        }
        else
        {
          // Send target position to PLC
          writeCmdUp(cmd_Register, bh_refilling, true);
        }

        Serial.println("Moving the extruder in position ...");
        delay(time_screen);
      }

      checkPLCAction(bi_RefillingDone);

      if (flag_PLC_completed)
      {
        x_refilling = double(modbusTCPClient.inputRegisterRead(feedback_X_Register))/100;
        y_refilling = double(modbusTCPClient.inputRegisterRead(feedback_Y_Register))/100;
        // DEBUG PRINT
        /////////////////////////////////////////////////
        Serial.print("X-Refilling: ");
        Serial.print(x_refilling);
        Serial.print("\t");
        Serial.print("Y-Refilling: ");
        Serial.println(y_refilling);
        Serial.println("Extruder in position ...");
        ////////////////////////////////////////////////
        delay(time_screen);
        flag_PLC_completed = false;
        action += 1;
        oneTime_action_step = true;
      }
      break;

    case 4:

      if (oneTime_action_step)
      {
        Serial.println("Step 4: Extrude ...");
        delay(time_screen);
        oneTime_action_step = false;

        // Send target position to PLC
        int number_steps_E0 = -7000; //-14000 //segno MENO per estrusione //segno PIU per SUCK_BACK 4450, 5550, 4000
        E0axis.move(number_steps_E0);
        E0axis.runToPosition();

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
        while (!flag_PLC_completed)
        {
          checkPLCAction(bi_ExtrusionDone);
        }
        Serial.println(flag_PLC_completed);
      }

      // send message to plc to move up FOR EXTRUSION
      if (flag_PLC_completed)
      {
        Serial.println("Start extrusion...");
        int number_steps_E0 = -3500; //-14000 //segno MENO per estrusione //segno PIU per SUCK_BACK 4450, 5550, 4000
        E0axis.move(number_steps_E0);
        E0axis.runToPosition();
        extrusion_step += 1;
        flag_PLC_completed = false;
      }

      if (extrusion_step_total - extrusion_step == 0)
      {
        writeCmdUp(cmd_Register, bh_extrusion, true);
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
        time_compression = millis();
        time_compression_prev = millis();
        compressed = false;
        writeCmdUp(cmd_Register, bh_compression, true);
      }
      // time_compression = millis();
      // if (time_compression - time_compression_prev > 15000UL && !compressed)
      // {
       // Start compression
      //  writeCmdUp(cmd_Register, bh_compression, true);
      //  time_compression_prev = time_compression;
      //  compressed = true;
      // }

      // if (compressed)
      // {
        checkPLCAction(bi_CompressionDone);
      // }

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
        flag_jetson = false;
        checkRefill();
        Check = Jetson;
        prev_timer = millis();
        time_imaging = millis();
        time_imaging_prev = millis();
      }

      time_imaging = millis();
      if(time_imaging - time_imaging_prev > 5000UL)
      {
        time_imaging_prev = time_imaging;
        checkRefill();
      }
      //////////////////////////////////////
      // TO REMOVE WHEN COMMUNICATION IS UP
      //flag_jetson = true;
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
      else if(flag_noRefill)
      {
         // If jetson answered 'No refill' try another refilling action
        Serial.println("Error! No refilling found");
        delay(time_screen);
        Check = Stop;
        State = indexing;
        action = 3;
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
        Serial.println("Process finished");
      }

      break;
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

  Serial.print("Time=");
  Serial.print(" ");
  Serial.print(time_extruder);
  Serial.print(" ");
  Serial.print("Target_Temperature=");
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

  // Command Variable
  Output_T = 0;
  Output_R = 0;
  analogWrite(10, 0); // Punta
  analogWrite(7, 0);  // Serbatoio

  // LETTURA TEMPERATURA
  temp_T = max_T.temperature(100, RREF); // Punta
  temp_R = max_R.temperature(100, RREF); // Serbatoio

  //////////////////////////////////////////////////////////////////////
  // Control loop
  if (PID_On)
  {
    Serial.println("-------------------");
    Serial.println("PID ON");
    Serial.println("-------------------");
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
  outputJson["value"] = "CheckID";
  outgoingSerial();
}

void checkRefill()
{
  outputJson["key"] = "ATRMController";
  outputJson["value"] = "CheckRefill";
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
  // laser_check[i] = laser_state;
  Serial.print("Laser State: ");
  Serial.println(laser_state);  // ACCESO = 0
  delay(500);
  ////////////////////////////////////////////////////////////////////////
  if (laser_state || laser_debug) // FOR DEBUG
  {
    // if (laser_state != last_laser_state)
    // {
    flag_laser_check = true;
    Serial.println("Laser catch a hole!");
    delay(time_screen);
    laser_debug = false;
    // }
    //////////////////////////////////////////////////////////////////////
  }
  // last_laser_state = laser_state;
}

void setpointTransform()
{
  // SetPoint transform from the POV of the camera to the cartesian operational space
  x_Position = x_origin + ( x_offset - x_Position);
  y_Position = y_origin - (y_offset - y_Position);
  z_Position = 25.5;
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
      x_Position = atof(((const char *)inputJson["pos_x"]));
      y_Position = atof((const char *)inputJson["pos_y"]);
      flag_jetson = true;
    }
    else if (value.equals("RefillOK"))
    {
      Serial.println("Refill OK");
      flag_jetson = true;
    }
    else if (value.equals("NORefill"))
    {
      Serial.println("No Refill");
      //flag_noRefill = true;
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
  //while (!mqtt_client.connected())
  //{
    if (!Ethernet.linkStatus())
    {
      Serial.println("Connecting to Ethernet");
      initEthernet();
    }
}

void Input()
{

  if (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
    case '2':
      flag_jetson = !flag_jetson;
      break;

    case 'r':
      Serial.println("Changing ready flag");
      flag_ready = !flag_ready;
      break;

    case 'l':
      Serial.println("Changing laser status");
      laser_debug = !laser_debug;
      break;

    // Extrusion
    case 'w':
      outputJson["key"] = "IDXController";
      outputJson["value"] = "WakeUp";
      outgoingSerial();
      break;

    case 'c':
      outputJson["key"] = "IDXController";
      outputJson["value"] = "CheckID";
      outgoingSerial();
      break;

    case 's':
      Serial.println("Start again");
      flag_start = !flag_start;
      State = Init;
      action = 1;
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
      openGripper();
      /* constant-expression */
      /* code */
      break;

    // Disconnect/Reconnect modbus master
    case 'd':
      Serial.println("Changing laser state...");
      laser_debug = !laser_debug;
      break;

    // Extrusion
    case '1':
      number_steps_E0 = -14000; //-14000 //segno MENO per estrusione //segno PIU per SUCK_BACK
      E0axis.move(number_steps_E0);
      E0axis.runToPosition();
      break;

    case 'y':
    outputJson["key"] = "ATRMController";
    outputJson["value"] = "CheckRefill";
    outgoingSerial();
    break;

    default:
      break;
    }
    /* code */
  }
}