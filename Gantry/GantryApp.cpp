/* ************************************************************
GantryApp.cpp
**************************************************************

Runs repeated trials of the snake robot experiment using the automated gantry system.

Dependencies:
(fill in later)

Authors: Ian Tomkinson, Zachary Goddard
email: ian.k.t1@gmail.com

Last Modified: Ian Tomkinson, 11/27/2017
Property of CRABLAB: Georgia Institute of Technology, School of Physics


Operation Instructions:

Steps marked with + are probably already done and only included for completeness.
1.	+ Open GantryApp project file if not already open  (D:\Gantry\GantryApp\GantryApp)
2.	+ Make sure that motive and the arduino serial window are closed
3.	+ Load the most recent version of GantryControl.ino on to the Arduino
4.	+ Open the command line if not already open
5.	+ Type D: to switch to the D drive
6.	+ Type cd D:\Gantry\GantryApp\GantryApp\Debug to switch to the appropriate directory
7.	Type GantryApp  to begin the app
8.	Give the name of the most current Optitrack calibration file when prompted (current file is Snake12.ttp )
9.	Give the name of a .csv file to write the optitrack data. This file will be created if it doesn’t exist already. Ex. Ianoutput.csv
10.	Type COM13  when prompted for the Com Port
11.	Program will start to run. Monitor progress in the command line, and follow steps bellow if something goes wrong:

In case of error:
1.	Type control + C to stop the Cpp program from running
2.	Immediately unplug power to all gantry components.
	Note: Steppers and and frigelli will continue to follow their last command even when the program is ended.
*/



/*************************************************************
 Dynamixel Control table
*************************************************************/

#pragma region "Dynamixel Control Table"

#include "stdafx.h"

#define _USE_MATH_DEFINES


#include <stdio.h>
#include <tchar.h>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include "SerialClass.h"	// Library described above
#include <string>
#include "NPTrackingTools.h"
#include "RigidBodySettings.h"
#include "Snake.h"
#include "dynamixel_sdk.h"
#include <thread>         // std::thread


//#include "SerialPort.h"

/*Portname must contain these backslashes, and remember to
replace the following com port*/
//char *port_name = "\\\\.\\COM15";

//String for incoming data
#define MAX_DATA_LENGTH                 255
char incomingSnakeData[MAX_DATA_LENGTH];
char incomingData[MAX_DATA_LENGTH];
char incomingMagDataA[MAX_DATA_LENGTH];
char incomingMagDataB[MAX_DATA_LENGTH];
char incomingWaitData[MAX_DATA_LENGTH];
//char prevData[MAX_DATA_LENGTH];

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING                  46

#define CW_COMPLIANCE_MARGIN			26
#define CCW_COMPLIANCE_MARGIN			27
#define CW_COMPLIANCE_SLOPE				28
#define CCW_COMPLIANCE_SLOPE			29

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING                   1

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000

#define DEVICENAME                      "COM22"      // Check which port is being used on your controller
//#define DEVICENAME                      "COM18"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      400                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#pragma endregion

//define subfunctions
int snakeUpdatePosition(double t, int ContactCondition);
int snakeAmplitudeModulation(double t, int ContactCondition);
int snakeAMM2(double t, float delA, float AMMTest);
void snakeInitialPosition();
//void CollectData(double t, ofstream outputFile);

char wait[10];

#define CheckResult(operation)   \
{\
	NPRESULT result = operation;	\
    if(result != NPRESULT_SUCCESS)  \
	{\
		printf("Error @%s:%i: %s\n\n(Press any key to continue)\n", __FILE__, __LINE__, TT_GetResultString(result));\
		exit (1);\
	}\
}

using namespace std;

/*********************************************************************

State: Defines states for the state machine

	RECEIVING: Read the serial buffer until GRBL stops sending information

	TRANSMITTING: Acquire user input to send to GRBL

	TRACKING_SNAKE: Record snake marker positiion to excel sheet, move snake

	TRACKING GANTRY: Find snake position after trial, move to that location

*********************************************************************/

enum State {
	RECEIVING,
	TRANSMITTING,
	TRACKING_SNAKE,
	TRACKING_GANTRY
};

/********************************************************************

GantryState: Defines states for the gantry

	STANDBY:

	GOTO:

	UPDATE_POSITION:

	ROTATING: Gantry servo is rotating

	DESCENDING:Gantry moving down towards mat

	ASCENDING:Gantry moving up away from mat

	RETURNING:

	SETSPEED:

********************************************************************/

enum GantryState {
	STANDBY,
	GOTO,
	UPDATE_POSITION,
	ROTATING,
	DESCENDING,
	ASCENDING,
	RETURNING,
	SETSPEED
};

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
dynamixel::GroupSyncWrite *groupSyncWrite;

int dxl_comm_result;
uint16_t dxl_model_number;                      // Dynamixel model number

// application reads from the specified serial port and reports the collected data
int main()
{

	/*******************************************************************
	Dynamixel Initialization
	*******************************************************************/

#pragma region "Dynamixel Initialization"

	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupSyncWrite instance
	groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

	dxl_comm_result = COMM_TX_FAIL;             // Communication result
	int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

	double st = 0;
	double dst = 0.001;

	uint8_t dxl_error = 0;                          // Dynamixel error
													// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		cin >> wait;
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		cin >> wait;
		return 0;
	}

	for (int i = 1; i <= 12; i++) {
		// Enable Dynamixel Torque
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i - 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
		else if (dxl_error != 0)
		{
			packetHandler->printRxPacketError(dxl_error);
		}
		else
		{
			printf("Dynamixel has been successfully connected to Motor:%02d \n", i - 1);
		}
	}

	for (int i = 1; i <= 12; i++) {
		// Enable Dynamixel Torque
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i - 1, CW_COMPLIANCE_SLOPE, 0x20, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
		else if (dxl_error != 0)
		{
			packetHandler->printRxPacketError(dxl_error);
		}
		else
		{
			printf("Dynamixel has been successfully connected to Motor:%02d \n", i - 1);
		}
	}

	for (int i = 1; i <= 12; i++) {
		// Enable Dynamixel Torque
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i - 1, CCW_COMPLIANCE_SLOPE, 0x20, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
		else if (dxl_error != 0)
		{
			packetHandler->printRxPacketError(dxl_error);
		}
		else
		{
			printf("Dynamixel has been successfully connected to Motor:%02d \n", i - 1);
		}
	}


#pragma endregion

	// Initialize the snake
	snakeInitialPosition();


	/****************************************************************
	Initializing the Gantry App
	****************************************************************/

#pragma region "Initializing the Gantry App"

	//String to contain input from GRBL
	string input = string();

	//Character identifiers for GRBL interface
	char ok[2] = { 'o', 'k' };
	char error[5] = { 'e','r','r','o','r' };
	string moveX = "move X";
	char cr = '\r';
	char nc = 0;
	char ac = '*';
	char* pcr = &cr;
	char* pnc = &nc;
	char* pac = &ac;

	size_t check_ok;
	size_t check_error;

	//Serial variables
	int input_index = 0;
	int dataLength = 255;
	int readResult = 0;

	char comNum[10];

	bool writeResult;
	bool writeResult3;
	bool writeResultclear;
	bool receiving = true;
	bool localized = false;
	State state = TRACKING_SNAKE;
	GantryState gstate = UPDATE_POSITION;

	//Optitrack project variables
	char project_Path[255];

	//Output file variables
	string filename = string();
	string basefilename = string();
	string finalfilename = string();
	ofstream outputFile;
	ofstream debugLog;



	//Initialize NPTrackingTools
	TT_Initialize();

	cout << "Welcome to the gantry app!\n\n";

	//Open Motive project (Must be in the application directory)
	cout << "Project Path: \n";
	cin >> project_Path;
	cout << project_Path << "\n\n";
	CheckResult(TT_LoadProject(project_Path));

	//Open a file for output data
	cout << "Output Filename: ";
	cin >> filename;
	basefilename = filename;


	/*time_t now = time(0);
	tm *ltm = localtime(&now);
	filename.append(to_string(ltm->tm_hour));
	filename.append(".");
	filename.append(to_string(1 + ltm->tm_min));
	filename.append(".");
	filename.append(to_string(1 + ltm->tm_sec));
	filename.append("Trial");
	//filename.append(to_string(0));
	filename.append(".csv");

	outputFile.open(filename);*/
	debugLog.open("debugLog.txt");
	debugLog << "Open DB" << endl;

	//Request number of trials
	int NumTrials;
	cout << "Number of trials";
	cin >> NumTrials;

	//Initialize Optitrack variables
	bool G_Trackable = false;
	bool G_isTracked = false;
	int numRigidBodies = 5;
	float gyaw, gpitch, groll, gx = 0, gy = 0, gz = 0, gqx, gqy, gqz, gqw, ax = 0, az = 0;
	float g1yaw, g1pitch, g1roll, gx1 = 0, gy1 = 0, gz1 = 0, gqx1, gqy1, gqz1, gqw1;
	float g2yaw, g2pitch, g2roll, gx2 = 0, gy2 = 0, gz2 = 0, gqx2, gqy2, gqz2, gqw2;
	float c1yaw, c1pitch, c1roll, c1x = 0, c1y = 0, c1z = 0, c1qx, c1qy, c1qz, c1qw;
	float c2yaw, c2pitch, c2roll, c2x = 0, c2y = 0, c2z = 0, c2qx, c2qy, c2qz, c2qw;
	float syaw, spitch, sroll, sx = 0, sy = 0, sz = 0, sqx, sqy, sqz, sqw;
	//float hyaw, hpitch, hroll, hx = 0, hy = 0, hz = 0, hqx, hqy, hqz, hqw;
	double t0 = 0;
	double t = 0;
	double prev_t = 0;
	float* pos = new float[2];
	outputFile << "Time,\t\tMarker ID,\t\tX Pos,\t\tY Pos,\t\tZ Pos\n";
	int init_count = 0;
	float* xPos;
	float* yPos;
	float* zPos;
	float angle;
	int numMarkers;
	const int stmax = 5;

	//Create Snake object
	//Snake* snake = new Snake(14, &xPos, &yPos, &zPos, &numMarkers);

	//Select COM number
	cout << "Select Com Port: ";
	cin >> comNum;


	//Adds \\.\ before the COM number
	for (int i = 4; i >= 0; i--) {
		comNum[4 + i] = comNum[i];
		comNum[i] = '\\';
	}

	comNum[2] = '.';
	comNum[9] = '\0';
	if (comNum[8] == '\n') {
		comNum[8] = '\0';
	}

	//Open COM port
	Serial* SP = new Serial(comNum);

	if (SP->IsConnected())
		cout << "We're connected\n\n";
	debugLog << "Connected" << endl;

	srand(time(0));

#pragma endregion


	/******************************************************************
	Manual Snake Loop "Testing new branch"
	******************************************************************/

#pragma region "Manual Snake Loop"

	/*string outputDataSnake = "";
	outputDataSnake.append("moveZ");
	outputDataSnake.append(to_string(1700));
	cout << "moving gantry with snake" << endl;

	outputDataSnake.append(pcr, 0, 1);
	writeResult = SP->WriteData((char*)outputDataSnake.c_str(), outputDataSnake.length());

	while (1) {
		cout << "Waiting..." << endl;
		cin >> wait;
		for (int itr = 0; itr < 35000 / 7; itr++) {
			snakeUpdatePosition(st, stmax);
			st += dst;
			Sleep(3);
		}
		//	//snakeInitialPosition();
	}*/

#pragma endregion

	//hardcoded AMM test
	//float AngleIn[8] = {0, -35, -35, -35, -35, -35, -35, -35 };
	float delA;

	//Begin program loop as long as COM port is open
	for (int trial = 0; trial < NumTrials; trial++)
	{
		if (trial > 0) {
			outputFile.close();
		}
		//reset the file name for the next iteration of the loop
		//cout << "filename before reset :" << ",\t" << filename;
		filename = to_string(trial);
		filename.append(basefilename);
		//cout << "filename after reset :" << ",\t" << filename;

		//create a file for each trial
		time_t now = time(0);
		tm *ltm = localtime(&now);
		filename.append(to_string(ltm->tm_hour));
		filename.append(".");
		filename.append(to_string(1 + ltm->tm_min));
		filename.append(".");
		filename.append(to_string(1 + ltm->tm_sec));
		filename.append("Trial");
		filename.append(to_string(trial));
		filename.append(".csv");

		outputFile.open(filename);

		cout << endl;

		cout << "*******************************************************************" <<  endl;

		cout << filename << endl;

		cout << "*******************************************************************" << endl;

		cout << endl;

		/*
		int count = trial % 8; 

		float p1 = 0.4365;
		//delA = ((3.14159265 / 12.0)*(AngleIn[trial])) / 100;
		delA = ((3.14159265 / 12.0)*(AngleIn[count]));
		delA = (delA / (1 - 3.14159265 / 12 * p1)); // correcting for experimentally observed error
		delA = delA / 100; //convert to the correct input scale

		cout << "delA" << ",\t\t" << delA << "Angle" << ",\t\t" << AngleIn << endl;*/

		/************************************************************************
		Tracking
		*************************************************************************/

		//filename.replace(filename.end()-1, filename.end(),to_string(i));

#pragma region "Tracking"

		//update optitrack, throw away first 5 frames?
		//cout << "Tracking \n";
		TT_Update();
		t = TT_FrameTimeStamp();
		if (t0 != t) {
			t0 = t;
			//if (init_count < 5) {
			//	init_count++;
			//}

				/***************************************************
				state = TRACKING SNAKE
				***************************************************/

#pragma region "TRACKING SNAKE"

				// Disable rigid body tracking for writing optitrack data to file
			cout << "Tracking Snake \n";
			for (int i = 0; i < numRigidBodies; i++) {
				if (TT_RigidBodyEnabled(i)) {
					TT_SetRigidBodyEnabled(i, false);
				}
			}

			//Check the motors are connected
			bool snake_break = false;
			int snake_count = 0;
			int Error_count = 0;
			while (1) {
				for (int i = 1; i <= 12; i++) {
					// Enable Dynamixel Torque
					dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i - 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
					if (dxl_comm_result != COMM_SUCCESS)
					{
						packetHandler->printTxRxResult(dxl_comm_result);
						break;
					}
					else if (dxl_error != 0)
					{
						packetHandler->printRxPacketError(dxl_error);
					}
					else
					{
						printf("Dynamixel has been successfully connected to Motor:%02d \n", i - 1);
						snake_count++;
					}
				}

				if (snake_count == 12) {
					break;
				}
				else {
					snake_count = 0;
					Error_count++;
				}

				if (Error_count > 20) {
					snake_break = true;
					break;
				}
			}

			if (snake_break == true) {
				break;
			}

			// Initialize the snake
			snakeInitialPosition();

			if (st == 0) {
				string outputDataSnake = "";
				outputDataSnake.append("moveZ");
				outputDataSnake.append(to_string(1600));
				//outputDataSnake.append(to_string(2000));
				cout << "moving gantry with snake" << endl;

				outputDataSnake.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDataSnake.c_str(), outputDataSnake.length());
			}


			//bool initialContact = false;
			int ContactCondition = 0;
			double ContactStart;
			double wContactStart;
			double ContactEnd;
			double wContactEnd;
			double ActualStart;
			double wActualStart;
			double ActualEnd;
			double wActualEnd;
			double ActualDuration;
			double wActualDuration;
			double WaitInitialTime;
			double LoopTime = 0;

			string prev_input = "0000";
			string end_state = "";
			string wend_state = "";
			int ContactCounter = 0;
			int wContactCounter = 0;

			// boolean states
			bool InitialContact = false;
			bool wInitialContact = false;

			bool WaitState = false;
			bool FinalContact = false;
			bool InitializeWait = false;


			float AMMStart = 0;
			//only for testing controller
			//bool FinalContact = true;
			//float AMMStart = 1.25;

			int EndCounter = 0;
			int wEndCounter = 0;
			float ContactDuration = 0;
			float wContactDuration = 0;
			float ContactAngle = 0;
			float PosContactAngle = 0;
			float NegContactAngle = 0;
			int sign;
			int wsign;

			//only for testing controller
			//int angle_idx = trial % 8;

			//Threshold
			while (st < stmax) {

				//update optitrack, throw away first 5 frames
				//cout << "Tracking \n";

				//commenting out camera stuff to see if snake performance improves

				TT_Update();
				t = TT_FrameTimeStamp();
				//if (t0 != t) {
					//t0 = t;
					//if (init_count < 5) {
					//	init_count++;
					//}


				numMarkers = TT_FrameMarkerCount();
				xPos = new float[numMarkers];
				yPos = new float[numMarkers];
				zPos = new float[numMarkers];

				//string last_input[6];
				//string prev_input = last_input;

				int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);

				string outputDataContact = "";
				outputDataContact.append("data");

				outputDataContact.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDataContact.c_str(), outputDataContact.length());
				                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
				Sleep(8);
				//Sleep(12);

				read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);

				//turns torque back on if turned off in previous loop
				//dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

				string input(incomingSnakeData);

				//cout << "Read Result: " << read_result << endl;
				//cout << incomingSnakeData << endl;
				//cout << endl;

				string last_input = input.substr(0,4);
				//string prev_input;

				//cout << "Contact State Test: " << last_input << endl;


				//prints out data
				//if (read_result > 0) {
					//cout << to_string(read_result) << endl;
					//cout << incomingSnakeData << endl;


				//cout << last_input << endl << endl;

				//if (last_input == "Contact") {
					//turns torque off if contact with peg
				//	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
				//}

				//input = "";
				//last_input = "";
				//Sleep(10000);
			//}

				//__int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

				if (prev_t == t) {
					//cout << "Skip this loop" << endl;
					continue;
				}

			//Collect data and write to excel, commenting out for IRIM video
				for (int i = 0; i < numMarkers; i++) {
					xPos[i] = TT_FrameMarkerX(i);
					yPos[i] = TT_FrameMarkerY(i);
					zPos[i] = TT_FrameMarkerZ(i);
					//cout << last_input << endl;
					//cout << endl;
					//cout << endl;
					//Sleep();
					outputFile << to_string(t) << ",\t\t" << to_string(i) << ",\t\t" << xPos[i] << ",\t\t" << yPos[i] << ",\t\t" << zPos[i] << ",\t\t" << last_input << endl;
					//outputFile << to_string(t) << ",\t\t" << to_string(i) << ",\t\t" << xPos[i] << ",\t\t" << yPos[i] << ",\t\t" << zPos[i] << ",\t\t" << incomingSnakeData << endl;
					//cout << to_string(t) << ",\t\t" << to_string(i) << ",\t\t" << last_input << endl;
					//cout << to_string(t) << ",\t\t" << last_input << endl;
					//outputFile << to_string(t) << ",\t\t" << last_input << endl;
				}
				delete[] xPos;
				delete[] yPos;
				delete[] zPos;
				string incomingSnakeData = "";


				if ((last_input == "0100") || (last_input == "0010") || (last_input == "0110")) {
					//left front, right front, both
					ContactCounter++;
				}
				else {
					ContactCounter = 0;
				}


				//cout << "Contact Counter: " << ContactCounter << endl;

				// Uncomment for contact sensing

				////////////////////////////////////////////////
				// Detect Contact Start
				///////////////////////////////////////////////

				if ((ContactCounter == 3)&&(InitialContact == false) && (WaitState == false)) {


					InitialContact = true;
					ActualStart = t;
					ContactStart = st;
					cout << "First Contact Made!" << endl;
					cout << "First Start Time: " << st << endl;


					//ContactEnd = ContactStart + 1;

					//old - used for SnakeAmplitudeModulation
					//if ((last_input == "1000") || (last_input == "1100") || (last_input == "0100")) {
					//	ContactCondition = 1;
					//}
					//else if ((last_input == "0001") || (last_input == "0010") || (last_input == "0011")) {
					//	ContactCondition = 2;
					//}
					//else {
					//	ContactCondition = ContactCondition;
					//}
				}
				

				////////////////////////////////////////////////////
				// Detecting End Contact
				////////////////////////////////////////////////////

				if ((InitialContact == true)&&(FinalContact == false) && (WaitState == false)) {
					if ((last_input == "0000") || (last_input == "0001") || (last_input == "1000")) {
						//no contact, right back, left back
						EndCounter++;
					}
					else {
						EndCounter = 0;
					}

					if (EndCounter == 1) {
						end_state = prev_input;
					}




					if (EndCounter == 3) {
						ContactEnd = st;
						ActualEnd = t;
						
						//For the longest 2 durations, switch into the waiting state
						WaitState = true;
						//FinalContact = true;

						//end of contact
						//cout << "Contact Over!" << endl;
						cout << "First End Time: " << st << endl;
						

						cout << "First End State: " << end_state << endl;

						if ((end_state == "0100") || (end_state == "1100")) {
							cout << "Contact Left! Steering Positive" << endl;
							sign = -1;

						}
						else if ((end_state == "0010") || (end_state == "0011")) {
							cout << "Contact Right! Steering Negative" << endl;
							sign = 1;
						}
						else {
							cout << "Controller Error -- Undefined Contact -- No Steering" << endl;
							sign = 0;
						}



						ContactDuration = ContactEnd - ContactStart;
						ActualDuration = ActualEnd - ActualStart;



					}
				}

				////////////////////////////////
				//End of Detecting Contact State
				////////////////////////////////

				////////////////////////////////////////
				// Waiting State
				////////////////////////////////////////
				if ((WaitState == true) && (FinalContact == false)) {

					if (InitializeWait == false) {
						WaitInitialTime = t;
						InitializeWait = true;
					}


					//checking for contact
					if ((last_input == "0100") || (last_input == "0010") || (last_input == "0110")) {
						//left front, right front, both
						wContactCounter++;
					}
					else {
						wContactCounter = 0;
					}


					///////////////////////////////////////////
					// Detecting Second Contact start
					///////////////////////////////////////////
					if ((wContactCounter == 3) && (wInitialContact == false)) {

						wInitialContact = true;
						wActualStart = t;
						wContactStart = st;
						cout << "Second Contact Made!" << endl;
						cout << "Second Start Time: " << st << endl;
					}

					////////////////////////////////////////////
					//Detecting Second Contact End
					////////////////////////////////////////////

					if ((wInitialContact == true) && (FinalContact == false)) {
						if ((last_input == "0000") || (last_input == "0001") || (last_input == "1000")) {
							//no contact, right back, left back
							wEndCounter++;
						}
						else {
							wEndCounter = 0;
						}

						if (wEndCounter == 1) {
							wend_state = prev_input;
						}

					}


					if (wEndCounter == 3) {
						wContactEnd = st;
						wActualEnd = t;

						//For the longest 2 durations, switch into the waiting state
						FinalContact = true;
						//FinalContact = true;

						//end of contact
						//cout << "Contact Over!" << endl;
						cout << "Second End Time: " << st << endl;


						cout << "Second End State: " << end_state << endl;

						if ((wend_state == "0100") || (wend_state == "1100")) {
							cout << "2nd Contact Left! Steering Positive" << endl;
							wsign = -1;

						}
						else if ((wend_state == "0010") || (wend_state == "0011")) {
							cout << "Contact Right! Steering Negative" << endl;
							wsign = 1;
						}
						else {
							cout << "Controller Error -- Undefined Contact -- No Steering" << endl;
							wsign = 0;
						}



						wContactDuration = wContactEnd - wContactStart;
						wActualDuration = wActualEnd - wActualStart;

						if (wActualDuration > ActualDuration) {
							sign = wsign;
							ContactDuration = wContactDuration;
							ActualDuration = wActualDuration;
							cout << "Steering Based on Second Contact" << endl;
						}
						else {
							cout << "First Contact is longer than Second" << endl;
						}


						/////////////////////////////////////////
						//Decision Block
						/////////////////////////////////////////


						// New Stuff
						//ContactAngle = -25.07*(ActualDuration)+1.85;
						PosContactAngle = 14.1*(ActualDuration)+2.3;
						NegContactAngle = -25.2*(ActualDuration)+2.1;
						PosContactAngle = (1.0 / 0.85)*PosContactAngle;
						NegContactAngle = (1.0 / 0.85)*NegContactAngle;


						//only for testing controller
						//ContactAngle = -19.44*(ActualDuration)-1.403;
						//ContactAngle = -25.07*(ActualDuration)+1.85;
						//ContactAngle = AngleIn[angle_idx];


						cout << "Contact Duration: " << ContactDuration << ",\t\t" << "Actual Duration: " << ActualDuration << endl;
						cout << "ContactAngle: " << ContactAngle << endl;

						//Experimental correction term
						ContactAngle = (1.0 / 0.85)*ContactAngle;

						//float p1 = 0.4365;


						//NewStuff
						if (sign > 0) {
							ContactAngle = NegContactAngle;
						}
						else {
							ContactAngle = PosContactAngle;
						}


						delA = ((3.14159265 / 12.0)*(sign*ContactAngle));
						//delA = (delA / (1 - 3.14159265 / 12 * p1)); // correcting for experimentally observed error



						delA = delA / 100; //convert to the correct input scale

						ContactEnd = ContactEnd * 2;
						AMMStart = ceil(ContactEnd);


						//Default Control
						if ((int)AMMStart % 2 == 1) {
							delA = -delA;
						}
						//AMMStart = AMMStart / 2;

						cout << "delA: " << delA << endl;

						//AMMStart = (AMMStart / 2) + 0.5;

						if (delA < 0) {
							//If expanding, steer ASAP
							cout << "Steer now!" << endl;
							AMMStart = AMMStart / 2;
						}
						else {
							//If contracting, steer next point of zero curvature
							cout << "Wait to steer" << endl;
							delA = -delA;
							AMMStart = (AMMStart / 2) + 0.5;
						}



						cout << "AMM Start: " << AMMStart << endl;


					}

					LoopTime = t - WaitInitialTime;

					if (LoopTime > 1.0) {
						WaitState = false;
						FinalContact = true;
						cout << "Only One Contact Detected" << endl;


						/////////////////////////////////////////
						//Decision Block
						/////////////////////////////////////////


						//only for testing controller
						//ContactAngle = -19.44*(ActualDuration)-1.403;


						// New Stuff
						//ContactAngle = -25.07*(ActualDuration)+1.85;
						PosContactAngle = 14.1*(ActualDuration)+2.3;
						NegContactAngle = -25.2*(ActualDuration)+2.1;

						PosContactAngle = (1.0 / 0.85)*PosContactAngle;
						NegContactAngle = (1.0 / 0.85)*NegContactAngle;


						//ContactAngle = AngleIn[angle_idx];


						cout << "Contact Duration: " << ContactDuration << ",\t\t" << "Actual Duration: " << ActualDuration << endl;
						cout << "ContactAngle: " << ContactAngle << endl;

						//Experimental correction term
						//ContactAngle = (1.0 / 0.85)*ContactAngle;

						//float p1 = 0.4365;
						
						//NewStuff
						if (sign > 0) {
							ContactAngle = NegContactAngle;
						}
						else {
							ContactAngle = PosContactAngle;
						}

						delA = ((3.14159265 / 12.0)*(ContactAngle));

						//delA = (delA / (1 - 3.14159265 / 12 * p1)); // correcting for experimentally observed error



						delA = delA / 100; //convert to the correct input scale

						ContactEnd = ContactEnd * 2;
						AMMStart = ceil(ContactEnd);


						//Default Control
						if ((int)AMMStart % 2 == 1) {
							delA = -delA;
						}
						//AMMStart = AMMStart / 2;

						cout << "delA: " << delA << endl;

						//AMMStart = (AMMStart / 2) + 0.5;

						if (delA < 0) {
							//If expanding, steer ASAP
							cout << "Steer now!" << endl;
							AMMStart = AMMStart / 2;
						}
						else {
							//If contracting, steer next point of zero curvature
							cout << "Wait to steer" << endl;
							delA = -delA;
							AMMStart = (AMMStart / 2) + 0.5;
						}
						cout << "AMM Start: " << AMMStart << endl;
					}

				}




				/////////////////////////////////////////
				//Decision Block
				/////////////////////////////////////////


				/*//only for testing controller
				//ContactAngle = -19.44*(ActualDuration)-1.403;
				ContactAngle = -25.07*(ActualDuration)+1.85;
				//ContactAngle = AngleIn[angle_idx];


				cout << "Contact Duration: " << ContactDuration << ",\t\t" << "Actual Duration: " << ActualDuration << endl;
				cout << "ContactAngle: " << ContactAngle << endl;

				//Experimental correction term
				ContactAngle = (1.0 / 0.85)*ContactAngle;

				//float p1 = 0.4365;
				delA = ((3.14159265 / 12.0)*(sign*ContactAngle));
				//delA = (delA / (1 - 3.14159265 / 12 * p1)); // correcting for experimentally observed error



				delA = delA / 100; //convert to the correct input scale

				ContactEnd = ContactEnd * 2;
				AMMStart = ceil(ContactEnd);


				//Default Control
				if ((int)AMMStart % 2 == 1) {
					delA = -delA;
				}
				//AMMStart = AMMStart / 2;

				cout << "delA: " << delA << endl;

				//AMMStart = (AMMStart / 2) + 0.5;

				if (delA < 0) {
					//If expanding, steer ASAP
					cout << "Steer now!" << endl;
					AMMStart = AMMStart / 2;
				}
				else {
					//If contracting, steer next point of zero curvature
					cout << "Wait to steer" << endl;
					delA = -delA;
					AMMStart = (AMMStart / 2) + 0.5;
				}



				cout << "AMM Start: " << AMMStart << endl;

						//cout << "AMM Start" << AMMStart << endl;*/

				//legacy code for SnakeAmplitudeModulation
				//if (st > ContactEnd) {
					//ContactCondition = 0;
				//}



				/*if ((last_input != "0000") && (prev_input != "0000") && (initialContact == false)) {
					initialContact = true;
					ContactStart = st;
					ContactEnd = ContactStart + 1;

				}


				if (ContactEnd > st) {
					//check prev as well
					if ((last_input == "1000") || (last_input == "1100") || (last_input == "0100")) {
						ContactCondition = 1;
					}
					else if ((last_input == "0001") || (last_input == "0010") || (last_input == "0011")) {
						ContactCondition = 2;
					}
					else {
						ContactCondition = ContactCondition;
					}
				}
				else {
					ContactCondition = 0;
				}*/


				// object from the class stringstream
				//stringstream last_input;
				//int Contact = stoi(last_input);
				// The object has the value 12345 and stream
				// it to the integer x
				//int Contact = 0;
				//last_input >> x;

				// Now the variable x holds the value 12345
				//cout << "is there contact? " << ContactResponse << endl;

				//int Contact = (int)last_input;

				//snakeAmplitudeModulation(st, ContactCondition);
				//snakeUpdatePosition(st, ContactCondition);

				/////////////////////////////////////////
				//Implement Controller State
				/////////////////////////////////////////

				if (FinalContact == true) {

					//snakeUpdatePosition(st, ContactCondition);


					//ContactAngle = (1.0/0.8)*AngleIn[angle_idx];
					//use the default expression
					//delA = ((3.14159265 / 12.0)*(ContactAngle));
					//delA = delA / 100; //convert to the correct input scale

					//cout << "delA: " << delA << "     " << "Contact Angle: " << ContactAngle << endl;

					//Commented out for normal behavior
					snakeAMM2(st, delA, AMMStart);
					//cout << "AMM controller" << st << endl;
				}

				////////////////////////////////////////////
				// Default Snake Behavior
				////////////////////////////////////////////

				else {
					snakeUpdatePosition(st, ContactCondition);
					//cout << "Normal Snake: " << st << endl;
				}



				//debugLog << "Snake Update Position. \n" << endl;
				//std::thread first (snakeUpdatePosition,st,stmax);
				//std::thread second (CollectData, st, outputFile);

				//Sleep(9);

				//snakeUpdatePosition(st, ContactCondition);
				st += 2.5*dst;

				prev_t = t;

				//add back in for snake data
				prev_input = last_input;
			}

#pragma endregion



			string outputDataStop = "";
			outputDataStop.append("stop");

			cout << "end of run" << endl;
			outputDataStop.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputDataStop.c_str(), outputDataStop.length());

			Sleep(5000);

			//cout << "End here" << endl;
			//Sleep(20000);

			//cout << "before reset \n\n";

			//reset optitrack
			//TT_Update();
			//Sleep(10000);
			//TT_Shutdown();
			//Sleep(3000);

			//Initialize NPTrackingTools
			TT_Initialize();

			//cout << "gantry app reinitialized! \n\n";
			//Sleep(3000);


			//Open Motive project (Must be in the application directory)
			//cout << "Project Path: \n";
			//cin >> project_Path;
			//cout << project_Path << "\n\n";
			CheckResult(TT_LoadProject(project_Path));

			Sleep(3000);

			TT_Update();

			for (int i = 0; i < numRigidBodies; i++) {
				if (~TT_RigidBodyEnabled(i)) {
					TT_SetRigidBodyEnabled(i, true);
				}
			}

			Sleep(10);
			state = TRACKING_GANTRY;

			/**********************************************
			state = Tracking Gantry
			**********************************************/

#pragma region "Tracking Gantry"

			cout << "Tracking Gantry. \n" << endl;

			st = 0;
			snakeInitialPosition();
			Sleep(1000);
			snakeInitialPosition();
			Sleep(1000);
			snakeInitialPosition();
			Sleep(1000);
			snakeInitialPosition();
			Sleep(1000);

			//is this needed? i forget what it does
			/*while (t0 != t) {
				TT_Update();
				t = TT_FrameTimeStamp();
			}
			t0 = t;
			xPos = new float[numMarkers];
			yPos = new float[numMarkers];
			zPos = new float[numMarkers];

			for (int i = 0; i < numMarkers; i++) {
				xPos[i] = TT_FrameMarkerX(i);
				yPos[i] = TT_FrameMarkerY(i);
				zPos[i] = TT_FrameMarkerZ(i);
			}*/

#pragma region "Target Location"

			Sleep(1000);
			TT_Update();
			Sleep(1000);

			//cout << "contact 1 \n" << endl;

			for (int i = 0; i < 600; i++) {
				TT_Update();
			}

			Sleep(1000);
			TT_Update();
			Sleep(1000);

			// too computationally expensive to be in the loop
			TT_RigidBodyLocation(1, &c1x, &c1y, &c1z, &c1qx, &c1qy, &c1qz, &c1qw, &c1yaw, &c1pitch, &c1roll);
			cout << to_string(t) << ",\t" << "Contact 1" << ",\t" << c1x << ",\t" << c1z << '\n';

			Sleep(1000);
			TT_Update();
			Sleep(1000);

			if (c1z < .1) {

				//could not correctly identify a snake marker

				string outputDataS1 = "";
				outputDataS1.append("moveZ");
				outputDataS1.append(to_string(-150));

				cout << "move to target \n" << endl;

				outputDataS1.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDataS1.c_str(), outputDataS1.length());

				Sleep(10000);


				for (int i = 0; i < 600; i++) {
					TT_Update();
				}

				// search for snake markers again
				TT_RigidBodyLocation(1, &c1x, &c1y, &c1z, &c1qx, &c1qy, &c1qz, &c1qw, &c1yaw, &c1pitch, &c1roll);
				cout << to_string(t) << ",\t" << "Contact 1" << ",\t" << c1x << ",\t" << c1z << '\n';

				Sleep(1000);

				for (int i = 0; i < 600; i++) {
					TT_Update();
				}

				Sleep(1000);

			}


			if (c1z < .1) {
				cout << "failed to correctly identify Snake marker 1" << endl;
				break;
			}


			//cout << "contact 2 \n" << endl;

			TT_RigidBodyLocation(2, &c2x, &c2y, &c2z, &c2qx, &c2qy, &c2qz, &c2qw, &c2yaw, &c2pitch, &c2roll);
			cout << to_string(t) << ",\t" << "Contact 2" << ",\t" << c2x << ",\t" << c2z << '\n';

			Sleep(1000);
			TT_Update();
			Sleep(1000);


			if (c2z < .1) {

				//could not correctly identify a snake marker

				string outputDataS2 = "";
				outputDataS2.append("moveZ");
				outputDataS2.append(to_string(-150));

				cout << "move to target \n" << endl;

				outputDataS2.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDataS2.c_str(), outputDataS2.length());

				Sleep(10000);


				for (int i = 0; i < 600; i++) {
					TT_Update();
				}

				// search for snake markers again
				TT_RigidBodyLocation(2, &c2x, &c2y, &c2z, &c2qx, &c2qy, &c2qz, &c2qw, &c2yaw, &c2pitch, &c2roll);
				cout << to_string(t) << ",\t" << "Contact 2" << ",\t" << c2x << ",\t" << c2z << '\n';

				Sleep(1000);
				TT_Update();
				Sleep(1000);

			}


			if (c2z < .1) {
				cout << "failed to correctly identify Snake marker 2" << endl;
				break;
			}



			pos[0] = (c1x + c2x) / 2;
			pos[1] = (c1z + c2z) / 2;
			//outputFile << to_string(t) << ",\t" << "Target Location" << ",\t" << pos[0] << ",\t" << pos[1] << endl;

			cout << "target location" << ",\t" << pos[0] << ",\t" << pos[1] << '\n';

			//delete[] xPos;
			//delete[] yPos;
			//delete[] zPos;
			if (!TT_RigidBodyEnabled(0)) {
				TT_SetRigidBodyEnabled(0, true);
			}

			cout << "Tracking Gantry.\n";

#pragma endregion

#pragma region "First move to Target"


			TT_RigidBodyLocation(0, &gx, &gy, &gz, &gqx, &gqy, &gqz, &gqw, &gyaw, &gpitch, &groll);
			//outputFile << to_string(t) << ",\t" << "Gantry" << ",\t" << gx << ",\t" << gz << '\n';

			cout << "gantry initial position" << ",t" << gx << ",\t" << gz << endl;

			//course adjustment position
			float dx, dz;

			float gx_first = gx;

			ax = gx;
			az = gz;

			dx = pos[0] - ax;
			dz = pos[1] - az;

			cout << "calculate difference \n" << endl;

			//outputFile << to_string(t) << ",\t" << "Difference" << ",\t" << dx << ",\t" << dz << '\n';
			cout << to_string(t) << ",\t" << "Difference" << ",\t" << dx << ",\t" << dz << '\n';

			// clear serial monitor
			cout << "Clear serial monitor" << endl;
			string outputclear22 = "";
			outputclear22.append(pcr, 0, 1);
			writeResultclear = SP->WriteData((char*)outputclear22.c_str(), outputclear22.length());
			cout << "Write Result Clear: " << to_string(writeResultclear) << endl;

			string outputData = "";
			outputData.append("moveZ");
			outputData.append(to_string(dz * 1000));
			outputData.append(",X");
			//outputData.append(to_string((dx * 1000)));
			outputData.append(to_string(-1*(dx * 1000)));

			cout << "move to target \n" << endl;

			outputData.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData.c_str(), outputData.length());

			Sleep(1000);

			//added hardcoded offset on motor angle based on observed behavior
			double theta;
			int motor;
			theta = ((180.0 / 3.14159)*atan((c2x - c1x) / (c2z - c1z)) + 90);

			outputFile << "Final Angle" << ",\t\t" << theta << endl;


			//motor = int((theta - 38.7931) / (.77586207) + 2);
			//motor = int(1.33*theta + 13);
			//motor = int(1.3556*theta - 52);
			//motor = int(1.3556*theta - 32);
			motor = int(1.35556*theta - 35);

			string outputData2 = "";
			outputData2.append("rots");
			outputData2.append(to_string(motor));

			//outputFile << to_string(t) << ",\t" << "theta" << ",\t" << theta << "motor" << ",\t" << motor << '\n';
			cout << to_string(t) << ",\t" << "theta" << ",\t" << theta << ",\t" << "motor" << ",\t" << motor << '\n';

			outputData2.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData2.c_str(), outputData2.length());

			cout << "Wait while moving to snake" << endl;
			int read_result5 = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);
			cout << "Serial monitor before moving to snake: " << incomingData << endl;

			string Wait3 = "wait";
			writeResult = SP->WriteData((char*)Wait3.c_str(), Wait3.length());
			cout << "To snake Wait write result " << writeResult << endl;

			Sleep(1000);

			string Wait4 = "wait";
			writeResult = SP->WriteData((char*)Wait4.c_str(), Wait4.length());


			double secondsPassed;
			clock_t startTime = clock(); //Start timer

			while (SP->IsConnected()) {
				//char* prevData = (char*)incomingData;
				//Check if data has been read or not
				int read_result = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);

				//prints out data
				if (read_result > 0) {
					cout << to_string(read_result) << endl;
					cout << incomingData << endl;


					string input(incomingData);
					string last_input = input.substr(0, 11);
					cout << last_input << endl;

					if (last_input == "done moving") {
						break;
					}
					string incomingData = "";
					input = "";
					last_input = "";

					TT_RigidBodyLocation(0, &gx, &gy, &gz, &gqx, &gqy, &gqz, &gqw, &gyaw, &gpitch, &groll);

					if (TT_IsRigidBodyTracked(0))
					{
						printf("%s: Pos (%.3f, %.3f, %.3f) Orient (%.1f, %.1f, %.1f)\n",
							TT_RigidBodyName(0), gx, gy, gz, gyaw, gpitch, groll);
					}
					else {
						cout << "Rigid Body Not Found!!" << endl;
						TT_Update();
					}

					TT_Update();

					secondsPassed = (clock() - startTime) / CLOCKS_PER_SEC;
					if (secondsPassed > 100){
						break;
					}
				}
				//puts(prevData);
				//cout << to_string(read_result);
				//wait a bit
				Sleep(1000);
			}

			//Sleep(80000);

			cout << "you have arrived. \n\n";

			Sleep(1000);
			TT_Update();
			Sleep(1000);

			for (int i = 0; i < 600; i++) {
				TT_Update();
			}

#pragma endregion

#pragma region "Fine position adjustment"

			TT_RigidBodyEnabled(0);
			Sleep(1000);
			if (!TT_RigidBodyEnabled(0)) {
				TT_SetRigidBodyEnabled(0, true);
			}

			TT_FlushCameraQueues();

			for (int i = 0; i < 600; i++) {
				TT_Update();
			}

			//fine adjustment position
			TT_RigidBodyLocation(0, &gx, &gy, &gz, &gqx, &gqy, &gqz, &gqw, &gyaw, &gpitch, &groll);
			cout << to_string(t) << ",\t" << "Updated Gantry Position" << ",\t" << gx << ",\t" << gz << '\n';

			ax = gx;
			az = gz;

			dx = pos[0] - ax;
			dz = pos[1] - az;

			int count = 0;
			int endprogram = 0;
			float prevgx;

			while (abs(dx) > .1) {
			
				cout << "Failed to Update Gantry Position" << endl;

				string outputDatafix = "";
				outputDatafix.append("moveX");
				outputDatafix.append("40");
				outputDatafix.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDatafix.c_str(), outputDatafix.length());


				//TT_Shutdown();
				//Sleep(5000);
				//TT_Initialize();

				//if (!TT_RigidBodyEnabled(0)) {
					//TT_SetRigidBodyEnabled(0, true);
				//}

				TT_FlushCameraQueues();

				for (int i = 0; i < 100; i++) {
					TT_Update();
					Sleep(10);
				}

				//fine adjustment position
				TT_RigidBodyLocation(0, &gx1, &gy1, &gz1, &gqx1, &gqy1, &gqz1, &gqw1, &g1yaw, &g1pitch, &g1roll);
				cout << to_string(t) << ",\t" << "Updated Gantry Position (corrected)" << ",\t" << gx1 << ",\t" << gz1 << ",\t" << count <<'\n';
			
				ax = gx1;
				az = gz1;

				dx = pos[0] - ax;
				dz = pos[1] - az;

				if (count == 0) {
					prevgx = gx;
				}


				if (abs(ax - prevgx) < .01) {
					cout << "Gantry Found!" << endl;
					break;
				}

				prevgx = ax;
				count++;

				if (count > 20) {
					endprogram = 1;
					break;
				}

			}



			//cout << gx << ",\t" << gz << '\n';
			//cout << theta << ",\t" << motor << '\n';

			if (endprogram == 1) {
				//fine adjustment position
				TT_RigidBodyLocation(0, &gx2, &gy2, &gz2, &gqx2, &gqy2, &gqz2, &gqw2, &g2yaw, &g2pitch, &g2roll);
				cout << to_string(t) << ",\t" << "Updated Gantry Position (Last Attempt)" << ",\t" << gx2 << ",\t" << gz2 << ",\t" << count << '\n';


				ax = gx2;
				az = gz2;

				dx = pos[0] - ax;
				dz = pos[1] - az;

				if (abs(ax - prevgx) > .01) {
					cout << "Gantry Not Found" << endl;
					break;
				}
			}



			cout << "calculate new difference \n" << endl;

			cout << "Clear serial monitor" << endl;
			string outputclear2 = "";
			outputclear2.append(pcr, 0, 1);
			writeResultclear = SP->WriteData((char*)outputclear2.c_str(), outputclear2.length());

			//outputFile << to_string(t) << ",\t" << "New Difference" << ",\t" << dx << ",\t" << dz << '\n';
			cout << to_string(t) << ",\t" << "New Difference" << ",\t" << dx << ",\t" << dz << '\n';

			string outputData3 = "";
			outputData3.append("moveZ");
			outputData3.append(to_string(dz * 1000));
			outputData3.append(",X-");
			outputData3.append(to_string(dx * 1000 - 3));

			cout << "move to target 2 \n" << endl;

			outputData3.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData3.c_str(), outputData3.length());

			while (SP->IsConnected()) {
				//char* prevData = (char*)incomingData;
				//Check if data has been read or not
				int read_result = SP->ReadData((char*)incomingWaitData, MAX_DATA_LENGTH);



				//prints out data
				if (read_result > 0) {
					cout << to_string(read_result) << endl;
					cout << incomingWaitData << endl;


					string input(incomingData);
					string last_input = input.substr(0, 11);
					cout << last_input << endl;

					if (last_input == "done moving") {
						break;
					}
					string incomingData = "";
					input = "";
					last_input = "";

					TT_RigidBodyLocation(0, &gx, &gy, &gz, &gqx, &gqy, &gqz, &gqw, &gyaw, &gpitch, &groll);

					if (TT_IsRigidBodyTracked(0))
					{
						printf("%s: Pos (%.3f, %.3f, %.3f) Orient (%.1f, %.1f, %.1f)\n",
							TT_RigidBodyName(0), gx, gy, gz, gyaw, gpitch, groll);
					}
					else {
						cout << "Rigid Body Not Found!!" << endl;
						TT_Update();
					}

					TT_Update();


				}
				//puts(prevData);
				//cout << to_string(read_result);
				//wait a bit
				Sleep(1000);
			}

			//Sleep(10000);
			//TT_Update();
			//Sleep(10000);

			for (int i = 0; i < 120; i++) {
				TT_Update();
			}

			TT_RigidBodyLocation(0, &gx, &gy, &gz, &gqx, &gqy, &gqz, &gqw, &gyaw, &gpitch, &groll);
			cout << to_string(t) << ",\t" << "Final Gantry Position" << ",\t" << gx << ",\t" << gz << '\n';
			//cout << gx << ",\t" << gz << '\n';
			//cout << theta << ",\t" << motor << '\n';

#pragma endregion

			cout << "Clear serial monitor" << endl;
			string outputclear23 = "";
			outputclear23.append(pcr, 0, 1);
			writeResultclear = SP->WriteData((char*)outputclear23.c_str(), outputclear23.length());


			//turn on magnets
#pragma region "Turn on Magnets"

			string outputData6 = "";
			outputData6.append("mgon");
			outputData6.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData6.c_str(), outputData6.length());
			cout << "Turn on Magnets" << endl;

#pragma endregion

			cout << "Clear serial monitor then lower gantry" << endl;
			string outputclear24 = "";
			outputclear24.append(pcr, 0, 1);
			int writeResultclear2 = SP->WriteData((char*)outputclear24.c_str(), outputclear24.length());
			cout << "Write Result Clear2: " << to_string(writeResultclear2) << endl;

			Sleep(400);

			//lower gantry
#pragma region "Lower Gantry"
			string outputData4 = "";
			outputData4.append("yneg");
			outputData4.append(pcr, 0, 1);
			cout << "Lower gantry Command" << outputData4 << endl;
			writeResult = SP->WriteData((char*)outputData4.c_str(), outputData4.length());
			cout << "Lowering the Gantry" << endl;


			Sleep(10000);
#pragma endregion

			cout << "Clear serial monitor then turn off frigelli" << endl;
			string outputclear25 = "";
			outputclear25.append(pcr, 0, 1);
			int writeResultclear3 = SP->WriteData((char*)outputclear25.c_str(), outputclear25.length());
			cout << "Write Result Clear3: " << to_string(writeResultclear3) << endl;

			Sleep(400);

			//turn off frigelli
#pragma region "Turn off frigelli"
			string outputData5 = "";
			outputData5.append("ystp");
			outputData5.append(pcr, 0, 1);
			cout << "Turn off frigelli Command" << outputData5 << endl;
			writeResult = SP->WriteData((char*)outputData5.c_str(), outputData4.length());
#pragma endregion

			int magcount = 0;
			int scount = 0;
			bool success = false;
			bool magbreak = false;

			// read serial monitor until it is clear
			for (int i = 0; i < 10; i++) {
				int read_result2 = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
			}

			/*while(1){
			
				float magthreshold = 1.6;

				//request magnet A data
				//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
				string outputDatamaga = "";
				outputDatamaga.append("maga");
				outputDatamaga.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDatamaga.c_str(), outputDatamaga.length());
				Sleep(100);

				//read magnet A data 
				int read_result = SP->ReadData((char*)incomingMagDataA, MAX_DATA_LENGTH);
				cout << "Read Result " << read_result << "	Magnet A Reading: " << incomingMagDataA << endl;

				//string Mag1(incomingMagData);
				//string Mag1Sub = Mag1.substr(0, 3);

				float maga = atof(incomingMagDataA);
				//cout << "Magnet A Reading: " << incomingMagData << endl;
			
				//request magnet B data
				//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
				string outputDatamagb = "";
				outputDatamagb.append("magb");
				outputDatamagb.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDatamagb.c_str(), outputDatamagb.length());
				Sleep(100);

				//read magnet B data 
				read_result = SP->ReadData((char*)incomingMagDataB, MAX_DATA_LENGTH);
				cout << "Read Result: " << read_result << "	 Magnet B Reading: " << incomingMagDataB<< endl;
				float magb = atof(incomingMagDataB);
				
				cout << endl;
				cout << endl;

				cout << "Magnet A = " << to_string(maga) << ",\t"<< to_string(magthreshold) << endl;
				cout << "Magnet B = " << to_string(magb) << ",\t"<< to_string(magthreshold) << endl;

				if ((magcount > 3) && (maga < magthreshold) && (magb < magthreshold)) {
					cout << "Contact Detected" << endl;
					success = true;
				}
				else{ 
					cout << "No Contact Detected" << endl;
					success = false;
					scount = 0;
				}

				if  (success == true){
					scount++;
				}

				if (scount > 0) {
					cout << "Successfully made contact" << endl;
					break;
				}

				magcount++;
				cout << "Mag count: " << magcount << endl;


				if (magcount > 8) {
					magbreak = true;
					break;
				}
			}*/

			float OutputMagA;
			float OutputMagB;
			float magthreshold = 1.65;

			for (int i = 0; i < 20; i++) {


				//request magnet A data
				//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
				string outputDatamaga = "";
				outputDatamaga.append("maga");
				outputDatamaga.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDatamaga.c_str(), outputDatamaga.length());
				Sleep(100);

				//read magnet A data 
				int read_result = SP->ReadData((char*)incomingMagDataA, MAX_DATA_LENGTH);
				//cout << "Read Result " << read_result << "	Magnet A Reading: " << incomingMagDataA << endl;

				//string Mag1(incomingMagData);
				//string Mag1Sub = Mag1.substr(0, 3);

				float maga = atof(incomingMagDataA);
				//cout << "Magnet A Reading: " << incomingMagData << endl;

				if (i == 0) {
					OutputMagA = maga;
				}
				if (i < 4) {
					OutputMagA = OutputMagA += (maga - OutputMagA) * 0.25;
				}
				else {
					if (abs(maga - OutputMagA) < .3) {
						OutputMagA += (maga - OutputMagA) * 0.25;
						//cout << "Update Magnet reading" << endl;
					}//else it stays the same as before
					else {
						OutputMagA = OutputMagA;
						//cout << "Keep Magnet reading the same" << endl;
					}
				}

				//cout << "Magnet A = " << to_string(maga) << ",\t" << to_string(magthreshold) << endl;
				//cout << "Mag   As = " << to_string(OutputMagA) << ",\t" << to_string(magthreshold) << endl;

				//request magnet B data
				//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
				string outputDatamagb = "";
				outputDatamagb.append("magb");
				outputDatamagb.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputDatamagb.c_str(), outputDatamagb.length());
				Sleep(100);

				//read magnet B data 
				read_result = SP->ReadData((char*)incomingMagDataB, MAX_DATA_LENGTH);
				//cout << "Read Result: " << read_result << "	 Magnet B Reading: " << incomingMagDataB << endl;
				float magb = atof(incomingMagDataB);

				if (i == 0) {
					OutputMagB = magb;
				}
				if (i < 4) {
					OutputMagB = OutputMagB += (magb - OutputMagB) * 0.25;
				}
				else {
					if (abs(magb - OutputMagB) < .3) {
						OutputMagB += (magb - OutputMagB) * 0.25;
						//cout << "Update Magnet reading" << endl;
					}//else it stays the same as before
					else {
						OutputMagB = OutputMagB;
						//cout << "Keep Magnet reading the same" << endl;
					}
				}

				//cout << "Mag A" << ",\t" << "Mag As" << ",\t" << "MagB" << ",\t" << "MagBs" << ",\t" << "Magthershold" << endl;

				//cout << "Magnet B = " << to_string(magb) << ",\t" << to_string(magthreshold) << endl;
				cout << to_string(maga) << ",\t" << to_string(OutputMagA) << ",\t" << to_string(magb) << ",\t" << to_string(OutputMagB) << ",\t" << to_string(magthreshold) << endl;
				//cout << "Mag   Bs = " << to_string(OutputMagB) << ",\t" << to_string(magthreshold) << endl;

				Sleep(100);
			}

			if ((OutputMagA < magthreshold) && (OutputMagB < magthreshold)) {
				cout << "Succesfully Made Contact, continuing" << endl;
			}
			else {
				magbreak = true;
				cout << "failed to make successful contact, trying again" << endl;
			}
			

			//Second Attempt to lower frigelli
			if (magbreak == true) {

				cout << "Second attempt to lower the frigelli" << endl;

				//lower frigelli

				// clear serial monitor
				cout << "Clear serial monitor" << endl;
				string outputclear5 = "";
				outputclear5.append(pcr, 0, 1);
				writeResultclear = SP->WriteData((char*)outputclear5.c_str(), outputclear5.length());
				cout << "Write Result Clear: " << to_string(writeResultclear) << endl;

				cout << "Lowering the Frigelli" << endl;
				string outputData95 = "";
				outputData95.append("yneg");
				outputData95.append(pcr, 0, 1);
				writeResult = SP->WriteData((char*)outputData95.c_str(), outputData95.length());
				cout << "Frigelli String" << ",\t" << outputData95 << ",\t" << "Frigelli Write Result" << to_string(writeResult) << endl;
				Sleep(22000);
				cout << "Waiting for snake to touch ground" << endl;


				magcount = 0;
				scount = 0;
				success = false;
				magbreak = false;

				// read serial monitor until it is clear
				for (int i = 0; i < 10; i++) {
					int read_result2 = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
				}

				OutputMagA = 0;
				OutputMagB = 0;
				magthreshold = 1.65;

				for (int i = 0; i < 20; i++) {


					//request magnet A data
					//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
					string outputDatamaga = "";
					outputDatamaga.append("maga");
					outputDatamaga.append(pcr, 0, 1);
					writeResult = SP->WriteData((char*)outputDatamaga.c_str(), outputDatamaga.length());
					Sleep(100);

					//read magnet A data 
					int read_result = SP->ReadData((char*)incomingMagDataA, MAX_DATA_LENGTH);
					//cout << "Read Result " << read_result << "	Magnet A Reading: " << incomingMagDataA << endl;

					//string Mag1(incomingMagData);
					//string Mag1Sub = Mag1.substr(0, 3);

					float maga = atof(incomingMagDataA);
					//cout << "Magnet A Reading: " << incomingMagData << endl;

					if (i == 0) {
						OutputMagA = maga;
					}
					if (i < 4) {
						OutputMagA = OutputMagA += (maga - OutputMagA) * 0.25;
					}
					else {
						if (abs(maga - OutputMagA) < .3) {
							OutputMagA += (maga - OutputMagA) * 0.25;
							//cout << "Update Magnet reading" << endl;
						}//else it stays the same as before
						else {
							OutputMagA = OutputMagA;
							//cout << "Keep Magnet reading the same" << endl;
						}
					}

					//cout << "Magnet A = " << to_string(maga) << ",\t" << to_string(magthreshold) << endl;
					//cout << "Magnet A (smoothed) = " << to_string(OutputMagA) << ",\t" << to_string(magthreshold) << endl;

					//request magnet B data
					//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
					string outputDatamagb = "";
					outputDatamagb.append("magb");
					outputDatamagb.append(pcr, 0, 1);
					writeResult = SP->WriteData((char*)outputDatamagb.c_str(), outputDatamagb.length());
					Sleep(100);

					//read magnet B data 
					read_result = SP->ReadData((char*)incomingMagDataB, MAX_DATA_LENGTH);
					//cout << "Read Result: " << read_result << "	 Magnet B Reading: " << incomingMagDataB << endl;
					float magb = atof(incomingMagDataB);

					if (i == 0) {
						OutputMagB = magb;
					}
					if (i < 4) {
						OutputMagB = OutputMagB += (magb - OutputMagB) * 0.25;
					}
					else {
						if (abs(magb - OutputMagB) < .3) {
							OutputMagB += (magb - OutputMagB) * 0.25;
							//cout << "Update Magnet reading" << endl;
						}//else it stays the same as before
						else {
							OutputMagB = OutputMagB;
							//cout << "Keep Magnet reading the same" << endl;
						}
					}

					//cout << "Magnet B = " << to_string(magb) << ",\t" << to_string(magthreshold) << endl;
					//cout << "Magnet B (smoothed) = " << to_string(OutputMagB) << ",\t" << to_string(magthreshold) << endl;
					//cout << "Mag A" << ",\t" << "Mag As" << ",\t" << "MagB" << ",\t" << "MagBs" << ",\t" << "Magthershold" << endl;
					cout << to_string(maga) << ",\t" << to_string(OutputMagA) << ",\t" << to_string(magb) << ",\t" << to_string(OutputMagB) << ",\t" << to_string(magthreshold) << endl;

					Sleep(100);
				}

				if ((OutputMagA < magthreshold) && (OutputMagB < magthreshold)){
					cout << "Succesfully Made Contact, continuing" << endl;
				}else{
					break;
				}



				/*while (1) {

					float magthreshold = 1.6;

					//request magnet A data
					//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
					string outputDatamaga = "";
					outputDatamaga.append("maga");
					outputDatamaga.append(pcr, 0, 1);
					writeResult = SP->WriteData((char*)outputDatamaga.c_str(), outputDatamaga.length());
					Sleep(100);

					//read magnet A data 
					int read_result = SP->ReadData((char*)incomingMagDataA, MAX_DATA_LENGTH);
					cout << "Read Result " << read_result << "	Magnet A Reading: " << incomingMagDataA << endl;

					//string Mag1(incomingMagData);
					//string Mag1Sub = Mag1.substr(0, 3);

					float maga = atof(incomingMagDataA);
					//cout << "Magnet A Reading: " << incomingMagData << endl;

					//request magnet B data
					//int read_result = SP->ReadData((char*)incomingSnakeData, MAX_DATA_LENGTH);
					string outputDatamagb = "";
					outputDatamagb.append("magb");
					outputDatamagb.append(pcr, 0, 1);
					writeResult = SP->WriteData((char*)outputDatamagb.c_str(), outputDatamagb.length());
					Sleep(100);

					//read magnet B data 
					read_result = SP->ReadData((char*)incomingMagDataB, MAX_DATA_LENGTH);
					cout << "Read Result: " << read_result << "	 Magnet B Reading: " << incomingMagDataB << endl;
					float magb = atof(incomingMagDataB);

					cout << endl;
					cout << endl;

					cout << "Magnet A = " << to_string(maga) << ",\t" << to_string(magthreshold) << endl;
					cout << "Magnet B = " << to_string(magb) << ",\t" << to_string(magthreshold) << endl;

					if ((magcount > 3) && (maga < magthreshold) && (magb < magthreshold)) {
						cout << "Contact Detected" << endl;
						success = true;
					}
					else {
						cout << "No Contact Detected" << endl;
						success = false;
						scount = 0;
					}

					if (success == true) {
						scount++;
					}

					if (scount > 0) {
						cout << "Successfully made contact" << endl;
						break;
					}

					magcount++;
					cout << "Mag count: " << magcount << endl;


					if (magcount > 8) {
						magbreak = true;
						break;
					}
				}*/
			}

			//if (magbreak == true){
				//break;
			//}

			// clear serial monitor
			cout << "Clear serial monitor" << endl;
			string outputclear21 = "";
			outputclear21.append(pcr, 0, 1);
			writeResultclear = SP->WriteData((char*)outputclear21.c_str(), outputclear21.length());
			cout << "Write Result Clear: " << to_string(writeResultclear) << endl;

			//raise gantry
#pragma region "Raise Gantry"
			string outputData7 = "";
			outputData7.append("ypos");
			outputData7.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData7.c_str(), outputData7.length());
			Sleep(13000);
#pragma endregion

			//go to start

#pragma region "Go to Home position"

			/*for (int i = 0; i < 600; i++) {
				TT_Update();
			}

			// too computationally expensive to be in the loop
			TT_RigidBodyLocation(4, &hx, &hy, &hz, &hqx, &hqy, &hqz, &hqw, &hyaw, &hpitch, &hroll);
			//outputFile << to_string(t) << ",\t" << "start" << ",\t" << sx << ",\t" << sz << '\n';

			Sleep(1000);
			TT_Update();
			Sleep(1000);


			dx = hx - gx;
			dz = hz - gz;

			cout << "calculate home difference \n" << endl;

			//outputFile << to_string(t) << ",\t" << "Home difference" << ",\t" << dx << ",\t" << dz << '\n';
			cout << to_string(t) << ",\t" << "Home difference" << ",\t" << dx << ",\t" << dz << '\n';

			string outputDataHome = "";
			outputDataHome.append("moveZ");
			outputDataHome.append(to_string(dz * 1000));
			outputDataHome.append(",X");
			outputDataHome.append(to_string(dx * 1000));

			cout << "Go Home \n" << endl;

			outputDataHome.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputDataHome.c_str(), outputDataHome.length());

			//waits until gantry is done moving
			while (SP->IsConnected()) {
				//char* prevData = (char*)incomingData;
				//Check if data has been read or not
				int read_result = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);

				//prints out data
				if (read_result > 0) {
					cout << to_string(read_result) << endl;
					cout << incomingData << endl;


					string input(incomingData);
					string last_input = input.substr(0, 11);
					cout << last_input << endl;

					if (last_input == "done moving") {
						break;
					}
					string incomingData = "";
					input = "";
					last_input = "";

				}
				//puts(prevData);
				//cout << to_string(read_result);
				//wait a bit
				Sleep(1000);
			}*/


#pragma region "Go to Start position"

			for (int i = 0; i < 600; i++) {
				TT_Update();
			}

			//reorient gantry so that the snake runs straight
			string outputServo2 = "";
			outputServo2.append("rots");
			int servoangle = 120;
			motor = int(1.35556*theta - 35);
			float theta_servo = (motor + 35) / (1.35556);
			outputFile << "Initial Angle" << ",\t\t" << theta_servo << endl;


			//outputServo2.append(to_string(102));
			//outputServo2.append(to_string(115)); // dont change!
			//outputServo2.append(to_string(116)); // dont change!
			//outputServo2.append(to_string(126)); // dont change!
			//outputServo2.append(to_string(116)); // dont change
			//outputServo2.append(to_string(120));

			//outputServo2.append(to_string(121));
			//outputServo2.append(to_string(125));
			outputServo2.append(to_string(servoangle));

			//outputServo2.append(to_string(116));
			//outputServo2.append(to_string(107));

			//outputServo2.append(to_string(118+14)); //new servos- should be 132
			//outputServo2.append(to_string(118 + 21));
			//outputServo2.append(to_string(140));


			outputServo2.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputServo2.c_str(), outputServo2.length());

			// too computationally expensive to be in the loop
			//TT_RigidBodyLocation(3, &sx, &sy, &sz, &sqx, &sqy, &sqz, &sqw, &syaw, &spitch, &sroll);
			cout << "Home Position" << ",\t" << sx << ",\t" << sz << endl;
			//outputFile << to_string(t) << ",\t" << "start" << ",\t" << sx << ",\t" << sz << '\n';

			Sleep(1000);
			TT_Update();
			Sleep(1000);

			TT_RigidBodyLocation(0, &gx, &gy, &gz, &gqx, &gqy, &gqz, &gqw, &gyaw, &gpitch, &groll);
			cout << to_string(t) << ",\t" << "Updated Gantry Position" << ",\t" << gx << ",\t" << gz << '\n';

			sx = 0;
			sz = 0;

			dx = sx - gx;
			dz = sz - gz;

			//int randx = rand() % 15;
			//int randx = rand() % 29;
			//int randz = rand() % 41;
			//int randz = rand() % 41;

			//int randz = -3; //-3 is back of box
			int randz = 0;
			int randx = 5; // 2 is center line- now 4?
			
			//int randx = 0;

			//int randz = 21;
			//int randx = 50;

			//int randx = 13;

			//int randz = 24;
			//int randx = 19;

			//int randx;
			//if (trial < 100) {
				//int randx = 28 + 30;
			//}
			//else{
				//randx = -20;
			//}

			//int randz = 27; 
			//int randx = 5;

			//randx = 9;
			//int randz = 20 + 5;

			cout << "calculate start difference \n" << endl;
			cout << "intial condition" << ",\t" << randx << ",\t" << randz << endl;

			//outputFile << to_string(t) << ",\t" << "Home difference" << ",\t" << dx << ",\t" << dz << '\n';
			cout << to_string(t) << ",\t" << "Start difference" << ",\t" << dx << ",\t" << dz << '\n';

			string outputData11 = "";
			outputData11.append("moveZ");

			//modified to finish closed loop data
			//outputData11.append(to_string(dz * 1000 - 370 + randz * 10 + 40));
			outputData11.append(to_string(dz * 1000 - 370 + randz * 10 +trial * 20 + 40 - 675+10)); //use this one
			//outputData11.append(to_string(dz * 1000 - 370 + randz * 10 + 40 - 675)); //use this one

			//outputData11.append(to_string(dz * 1000 - 370 + randz*10 + trial*10));
			//outputData11.append(to_string(dz * 1000 - 370 + 160 + randz * 10));
			outputData11.append(",X");
			outputData11.append(to_string(-1*(dx * 1000 - 140 + 60 + 120 + randx*10 + 10 - 30 - 170))); // use this one for full box
			//outputData11.append(to_string(dx * 1000 - 140 + 60 + 220 + randx * 10));

			float Zic = -370 + randz * 10 + trial * 20 + 40 - 675 + 10;
			float Xic = -140 + 60 + 120 + randx * 10 + 10 - 30 - 170;

			outputFile << to_string(Xic) << ",\t\t" << to_string(Zic) << endl;


			cout << "Go to start \n" << endl;
			cout << "Offset" << to_string(trial * 20) << '\n';

			outputData11.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData11.c_str(), outputData11.length());
			cout << "Successful output String" << ",\t" << outputData11 << endl;
			cout << "write result " << to_string(writeResult) << endl;

			Sleep(1000);

			cout << "Wait while moving home" << endl;
			int read_result = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);
			cout << "Serial monitor before moving home: "  << incomingData << endl;

			string Wait = "wait";
			writeResult = SP->WriteData((char*)Wait.c_str(), Wait.length());
			cout << "Wait write result " << writeResult << endl;

			Sleep(2000);

			// clear serial monitor
			cout << "Clear serial monitor" << endl;
			string outputclear51 = "";
			outputclear51.append(pcr, 0, 1);
			writeResultclear = SP->WriteData((char*)outputclear51.c_str(), outputclear51.length());
			cout << "Write Result Clear: " << to_string(writeResultclear) << endl;

			//turn off frigelli
#pragma region "Turn off Frigelli"
			string outputData8 = "";
			outputData8.append("ystp");
			outputData8.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData8.c_str(), outputData8.length());
#pragma endregion


			//waits until gantry is done moving
			while (SP->IsConnected()) {
				//char* prevData = (char*)incomingData;
				//Check if data has been read or not
				int read_result = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);

				//prints out data
				if (read_result > 0) {
					cout << "Characters read " << to_string(read_result) << endl;
					cout << incomingData << endl;


					string input(incomingData);
					string last_input = input.substr(0, 11);
					cout << last_input << endl;

					if (last_input == "done moving") {
						break;
					}
					string incomingData = "";
					input = "";
					last_input = "";

				}
				//puts(prevData);
				//cout << to_string(read_result);
				//wait a bit
				Sleep(1000);
			}

			/*Sleep(1000);
			cout << "Inputing Offset value" << endl;


			string outputData21 = "";
			outputData21.append("moveZ");
			outputData21.append(to_string(120 + i * 40));
			outputData21.append(",X");
			outputData21.append(to_string(0));

			cout << "Go to start \n" << endl;
			cout << "Offset" << to_string(i * 20) << '\n';

			outputData21.append(pcr, 0, 1);
			writeResult3 = SP->WriteData((char*)outputData21.c_str(), outputData21.length());
			cout << "Problem output String" << ",\t" << outputData21 << endl;
			cout << "write result " << to_string(writeResult3) << endl;*/



			/*string outputDataadj = "";
			outputDataadj.append("moveZ");
			outputDataadj.append(to_string(120 + i * 40));
			outputDataadj.append(",X");
			outputDataadj.append(to_string(-80));

			//cout << "Adjust by offset \n" << endl;
			cout << "Offset" << to_string(i * 40) << '\n';

			outputDataadj.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputDataadj.c_str(), outputDataadj.length());
			cout << "Problem output string" << ",\t" << outputDataadj << endl;
			cout <<	"write result "<< to_string(writeResult) << endl;*/

			/*string outputDatanew = "";
			outputDatanew.append("moveZ");
			outputDatanew.append(to_string(120 + i*40));
			outputDatanew.append(",X");
			outputDatanew.append(to_string(0));

			outputDatanew.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputDatanew.c_str(), outputDatanew.length());
			cout << outputDatanew << ",\t" << writeResult << endl;

			cout << "Wait while adjusting position" << endl;
			int read_result2 = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);
			cout << "Serial monitor before initial condition: " << incomingData << endl;*/

			/*cout << "Wait while moving home" << endl;
			int read_result3 = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);
			cout << "Read Result" << ",\t" << to_string(read_result3) << endl;
			cout << "Serial monitor before IC: " << ",\t" << incomingData << endl;

			string Wait2 = "wait";
			writeResult = SP->WriteData((char*)Wait2.c_str(), Wait2.length());
			cout << "Wait write result " << writeResult << endl;

			while (SP->IsConnected()) {
				//char* prevData = (char*)incomingData;
				//Check if data has been read or not
				int read_result = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);
				cout << "waiting read result " << read_result << endl;

				//prints out data
				if (read_result > 0) {
					cout << "Characters Read " << to_string(read_result) << endl;
					cout << incomingData << endl;


					string input(incomingData);
					string last_input = input.substr(0, 11);
					cout << last_input << endl;

					if (last_input == "done moving") {
						break;
					}
					string incomingData = "";
					input = "";
					last_input = "";

				}
				//puts(prevData);
				//cout << to_string(read_result);
				//wait a bit
				Sleep(1000);
			}

			//Sleep(60000);

			Sleep(1000);*/

#pragma endregion

			//lower frigelli
#pragma region "Lower Frigelli"
			// clear serial monitor
			cout << "Clear serial monitor" << endl;
			string outputclear = "";
			outputclear.append(pcr, 0, 1);
			writeResultclear = SP->WriteData((char*)outputclear.c_str(), outputclear.length());
			cout << "Write Result Clear: " << to_string(writeResultclear) << endl;

			cout << "Lowering the Frigelli" << endl;
			string outputData9 = "";
			outputData9.append("yneg");
			outputData9.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData9.c_str(), outputData9.length());
			cout << "Frigelli String" << ",\t" << outputData9 << ",\t" << "Frigelli Write Result" << to_string(writeResult) << endl;
			Sleep(22000);
			cout << "Waiting for snake to touch ground" << endl;
#pragma endregion 

			//turn off magnets
#pragma region "Turn off magnets"
			string outputData10 = "";
			outputData10.append("mgof");
			outputData10.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData10.c_str(), outputData10.length());
#pragma endregion

			//raise gantry
#pragma region "Raise Gantry"
			string outputData12 = "";
			outputData12.append("ypos");
			outputData12.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData12.c_str(), outputData12.length());

			Sleep(3000);

			string outputDataHome2 = "";
			outputDataHome2.append("moveX");
			outputDataHome2.append(to_string(900));

			cout << "moving gantry away" << endl;

			outputDataHome2.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputDataHome2.c_str(), outputDataHome2.length());


			Sleep(6000);
#pragma endregion


			//turn off frigelli
#pragma region "Turn of frigelli"
			string outputData13 = "";
			outputData13.append("ystp");
			outputData13.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputData13.c_str(), outputData13.length());

			Sleep(18000);
#pragma endregion

#pragma region "Homing Gantry"

			//waits until gantry is done moving
			/*while (SP->IsConnected()) {
				//char* prevData = (char*)incomingData;
				//Check if data has been read or not
				int read_result = SP->ReadData((char*)incomingData, MAX_DATA_LENGTH);

				//prints out data
				if (read_result > 0) {
					cout << to_string(read_result) << endl;
					cout << incomingData << endl;


					string input(incomingData);
					string last_input = input.substr(0, 11);
					cout << last_input << endl;

					if (last_input == "done moving") {
						break;
					}
					string incomingData = "";
					input = "";
					last_input = "";

				}
			}*/
			//Sleep(25000);


			/*string outputDataServo = "";
			outputDataServo.append("rots");
			outputDataServo.append(to_string(106));

			outputDataServo.append(pcr, 0, 1);
			writeResult = SP->WriteData((char*)outputDataServo.c_str(), outputDataServo.length());
			*/

#pragma endregion



#pragma endregion

		}
#pragma endregion
	}

	delete[] pos;

	//Exit Program if serial communications are lost
	cout << "COM Port disconnected. Press and key and enter to exit.";
	char exit_buffer[10];
	cin >> exit_buffer;
	return 0;
}

void snakeInitialPosition() {

	uint16_t dxl_present_position = 0;             // Present position
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint8_t param_goal_position[2];
	int initcount = 0;

	bool dxl_addparam_result_write = false;                // addParam result

	double B0 = 0.4;
	int N = 12;

	int init_pos[12];
	int cur_pos[12];
	double snake_angle[12];
	int snake_goal[12];
	int goal_pos[12];
	double Pi = M_PI;

	while (initcount < 1000) {
		for (int i = 1; i <= N; i++) {
			snake_angle[i - 1] = B0*sin(2 * Pi*i / N);
			init_pos[i - 1] = (int)(snake_angle[i - 1] * 1024 / 3 + 512);
			//printf("Motor:%02d Init_Position:%03d\n", i - 1, init_pos[i - 1]);

			param_goal_position[0] = DXL_LOBYTE(init_pos[i - 1]);
			param_goal_position[1] = DXL_HIBYTE(init_pos[i - 1]);

			// Add Dynamixels goal position value to the Syncwrite storage
			dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);
			if (dxl_addparam_result_write != true)
			{
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", i - 1);
				Sleep(3000);
				return;
			}
		}
		// Syncwrite goal position
		dxl_comm_result = groupSyncWrite->txPacket();
		if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

		// Clear syncwrite parameter storage
		groupSyncWrite->clearParam();

		initcount++;
	}
}

int snakeUpdatePosition(double t, int ContactCondition) {

	bool dxl_addparam_result_write = false;                // addParam result

	double offset = 0;
	//double offset = 0.2;

	uint16_t dxl_present_position = 0;             // Present position
	uint8_t dxl_error = 0;                          // Dynamixel error

	uint8_t param_goal_position[2];

	double B0 = 0.4;
	int N = 12;

	int init_pos[12];
	int cur_pos[12];
	double snake_angle[12];
	int snake_goal[12];
	int goal_pos[12];
	double Pi = M_PI;
	bool Torque = true;

	/*for (int i = 1; i <= N; i++) {
		snake_angle[i - 1] = B0*sin(2 * Pi*i / N - 2 * Pi*t);
		snake_goal[i - 1] = (int)(snake_angle[i - 1] * 1024 / 3 + 512 + offset);


		//Read present position
		//dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i - 1, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
		//dxl_present_position = dxl_present_position & 0xff;
		//printf("[ID:%03d] Present Position:%03i\n", i - 1, dxl_present_position);

		param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
		param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

		// Add Dynamixels goal position value to the Syncwrite storage
		dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);
		if (dxl_addparam_result_write != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", i - 1);
			Sleep(3000);
			return 0;
		}

		// Syncwrite goal position
		dxl_comm_result = groupSyncWrite->txPacket();
		if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

		// Clear syncwrite parameter storage
		groupSyncWrite->clearParam();


	}
	return 1;*/

	for (int i = 1; i <= N; i++) {

		/*if ((i == 1) && (ContactCondition == 1)) {
			snake_goal[i - 1] = (int)(B0 * 1024 / 3 + 512 + offset);
			param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
			param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

			// Add Dynamixels goal position value to the Syncwrite storage
			dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);

		}
		else if ((i == 1) && (ContactCondition == 2)) {
			snake_goal[i - 1] = (int)(-B0 * 1024 / 3 + 512 + offset);
			param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
			param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

			// Add Dynamixels goal position value to the Syncwrite storage
			dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);

		}*/
		//else {


		//if ((ContactCondition == 1) || (ContactCondition == 2) ) {
			//if (Torque==true) {
				//dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
				//Torque = false;
			//}
			//B0 = .4;

		//}
		//else {
			//B0 = .4;
			//if (Torque == false) {
				//dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
				//Torque = true;
			//}
			//dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		//}

			snake_angle[i - 1] = B0*sin(2 * Pi*i / N - 2 * Pi*t);
			//snake_goal[i - 1] = (int)(snake_angle[i - 1] * 1024 / 3 + 512 + offset);
			snake_goal[i - 1] = (int)(round(snake_angle[i - 1] * 1024 / 3 + 512 + offset));

			param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
			param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

			// Add Dynamixels goal position value to the Syncwrite storage
			dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);
			if (dxl_addparam_result_write != true)
			{
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", i - 1);
				Sleep(3000);
				return 0;
			//}
		}

	}


	// Syncwrite goal position
	dxl_comm_result = groupSyncWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

	// Clear syncwrite parameter storage
	groupSyncWrite->clearParam();
}

int snakeAMM2(double t, float delA, float AMMStart) {

	bool dxl_addparam_result_write = false;                // addParam result

	double offset = 0;
	//double offset = -1.3;

	uint16_t dxl_present_position = 0;             // Present position
	uint8_t dxl_error = 0;                          // Dynamixel error

	uint8_t param_goal_position[2];

	//double delA = 0.1;
	double B0 = 0.4;
	int N = 12;

	int init_pos[12];
	int cur_pos[12];
	double snake_angle[12];
	int snake_goal[12];
	int goal_pos[12];
	double Pi = M_PI;
	bool Torque = true;
	double AMMmin;
	double AMMmax;

	for (int i = 1; i <= N; i++) {

		//AMMmin = (double)(1.0 + (((double)i - 1.0) / (double)N));
		//AMMmax = (double)(1.5 + (((double)i - 1.0) / (double)N));
		AMMmin = (double)(AMMStart + (((double)i - 1.0) / (double)N));
		AMMmax = (double)(AMMStart + 0.5 + (((double)i - 1.0) / (double)N));

		if ((t > AMMmin) && (t < AMMmax)) {
			snake_angle[i - 1] = (B0 + delA)*sin((2 * Pi*i) / N - 2 * Pi*t);
			//cout << "AMM time: " << to_string(t) << "  i = " << to_string(i) << " AMM min: " << to_string(AMMmin) << " AMM max: " << to_string(AMMmax)  << endl;
		}
		else {
			snake_angle[i - 1] = B0*sin((2 * Pi*i) / N - 2 * Pi*t);
			//cout << "Normal time: " << to_string(t) <<  "  i = " << to_string(i) << " AMM min: " << to_string(AMMmin) << " AMM max: " << to_string(AMMmax) << endl;
		}
		snake_goal[i - 1] = (int)(snake_angle[i - 1] * 1024 / 3 + 512 + offset);


		param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
		param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

		// Add Dynamixels goal position value to the Syncwrite storage
		dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);
		if (dxl_addparam_result_write != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", i - 1);
			Sleep(3000);
			return 0;
		}


		// Syncwrite goal position
		dxl_comm_result = groupSyncWrite->txPacket();
		if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

		// Clear syncwrite parameter storage
		groupSyncWrite->clearParam();
	}
}

/*
int snakeAmplitudeModulation(double t, int ContactCondition) {

	bool dxl_addparam_result_write = false;                // addParam result

	double offset = 1.3;

	uint16_t dxl_present_position = 0;             // Present position
	uint8_t dxl_error = 0;                          // Dynamixel error

	uint8_t param_goal_position[2];

	double B0 = 0.4;
	int N = 12;
	double delA = 0.1;

	int init_pos[12];
	int cur_pos[12];
	double snake_angle[12];
	int snake_goal[12];
	int goal_pos[12];
	double Pi = M_PI;
	bool Torque = true;
	int snake_state;

	if (t < 1.0) {
		snake_state = 1; //normal
	}
	if (t > 1.0) {
		snake_state = 2; //amplitude changes
	}

	if(t > 1.5){
		snake_state = 3; //return to normal
	}




	switch (snake_state) {

		case 1: {
			for (int i = 1; i <= N; i++) {

				snake_angle[i - 1] = B0*sin(2 * Pi*i / N - 2 * Pi*t);
				snake_goal[i - 1] = (int)(snake_angle[i - 1] * 1024 / 3 + 512 + offset);


				param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
				param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

				// Add Dynamixels goal position value to the Syncwrite storage
				dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);
				if (dxl_addparam_result_write != true)
				{
					fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", i - 1);
					Sleep(3000);
					return 0;
					//}
				}

			}


			// Syncwrite goal position
			dxl_comm_result = groupSyncWrite->txPacket();
			if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

			// Clear syncwrite parameter storage
			groupSyncWrite->clearParam();

			cout << "case 1 t: " << to_string(t) << endl;
			break;

		}

		case 2: {
			for (int i = 1; i <= N; i++) {


				if ((t - 1) > ((2 * Pi*i) / N)) {
					snake_angle[i - 1] = (delA + B0)*sin(2 * Pi*i / N - 2 * Pi*t);
				}

				else {
					snake_angle[i - 1] = B0*sin(2 * Pi*i / N - 2 * Pi*t);
				}

				snake_goal[i - 1] = (int)(snake_angle[i - 1] * 1024 / 3 + 512 + offset);


				param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
				param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

				// Add Dynamixels goal position value to the Syncwrite storage
				dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);
				if (dxl_addparam_result_write != true)
				{
					fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", i - 1);
					Sleep(3000);
					return 0;
					//}
				}

			}


			// Syncwrite goal position
			dxl_comm_result = groupSyncWrite->txPacket();
			if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

			// Clear syncwrite parameter storage
			groupSyncWrite->clearParam();

			cout << "case 2 t: " << to_string(t) << endl;
			break;

		}

		case 3: {

			for (int i = 1; i <= N; i++) {


				if ((t - 1) > (2 * Pi*i) / N) {
					snake_angle[i - 1] = B0*sin(2 * Pi*i / N - 2 * Pi*t);
				}

				else {
					snake_angle[i - 1] = (delA + B0)*sin(2 * Pi*i / N - 2 * Pi*t);
				}

				snake_goal[i - 1] = (int)(snake_angle[i - 1] * 1024 / 3 + 512 + offset);


				param_goal_position[0] = DXL_LOBYTE(snake_goal[i - 1]);
				param_goal_position[1] = DXL_HIBYTE(snake_goal[i - 1]);

				// Add Dynamixels goal position value to the Syncwrite storage
				dxl_addparam_result_write = groupSyncWrite->addParam(i - 1, param_goal_position);
				if (dxl_addparam_result_write != true)
				{
					fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", i - 1);
					Sleep(3000);
					return 0;
					//}
				}

			}


			// Syncwrite goal position
			dxl_comm_result = groupSyncWrite->txPacket();
			if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

			// Clear syncwrite parameter storage
			groupSyncWrite->clearParam();

			cout << "case 3 t: " << to_string(t) << endl;
			break;

		}

	}
}

void CollectData(double t, ofstream outputFile)
{
	float* xPos;
	float* yPos;
	float* zPos;
	int numMarkers;

	numMarkers = TT_FrameMarkerCount();
	xPos = new float[numMarkers];
	yPos = new float[numMarkers];
	zPos = new float[numMarkers];

	//commenting out for IRIM video
	for (int i = 0; i < numMarkers; i++) {
		xPos[i] = TT_FrameMarkerX(i);
		yPos[i] = TT_FrameMarkerY(i);
		zPos[i] = TT_FrameMarkerZ(i);
		outputFile << to_string(t) << ",\t\t" << to_string(i) << ",\t\t" << xPos[i] << ",\t\t" << yPos[i] << ",\t\t" << zPos[i] << endl;
	}

	delete[] xPos;
	delete[] yPos;
	delete[] zPos;
}
*/