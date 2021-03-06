#ifndef BLDC_H
#define BLDC_H
#include "pid.h"
#include "CppLinuxSerial/SerialPort.hpp"
#include <vector>

using namespace mn::CppLinuxSerial;
using namespace std;

// #define DEBUG

#define RPM_MAX 5000
#define POS_MAX 0x10EA
#define DEFAULT_BAUDRATE BaudRate::B_115200
#define DEFAULT_WAIT_MS 10 // with baudrate 115200
#define DEVICE_1 "/dev/rs485_01"
#define DEVICE_2 "/dev/rs485_02"
#define DEVICE_3 "/dev/ttyUSB2"
#define DEVICE_4 "/dev/rs485_04"

#define MOTOR_GEAR       50
#define RADIUS     0.0625

class BLDC {
	private:
		SerialPort port;

		int id; /*Machine ID*/

		bool readyFlag; /*Flag for ready state*/

		int16_t rpm; /*Speed (rpm)*/
		uint16_t current; /*Current (0~1023, 0.1A Unit)*/
		uint8_t ctrl; /*Control type*/
		int16_t ref; /*Reference speed (rpm)*/
		uint16_t out; /*Control output*/
		uint8_t stat; /*Status of controller*/
		uint32_t pos; /*Motor position*/
		uint8_t brk; /*Break duty (0~255)*/
		uint8_t temp; /*Temperature (0~100)*/

		vector<uint32_t> prst_list; /*List of preset positions*/

		void SendPkg(vector<uint8_t> pkg);

		int RevPkg(string &rev_data);

		void PidCmd(uint8_t cmd); /*Send commands*/

		int ReqData(uint8_t pid, string &rev_data); /*Request PID*/
		
		#ifdef DEBUG
			void PrintMainData(); /*Print out main data. For debug*/
		#endif
	public:

		vector<uint8_t> MakePkg(uint8_t pid, int num, vector<uint8_t> data);
	
		void MotorInit(); /*Init the motor. Fist function to call*/
		void ChangeBaud(uint8_t baud); /*Change Baudrate. 9600, 19200, 38400, 57600, 115200*/
		void VelCmd(int rpm); /*Set velocity*/
		void TqCmd(int tq); /*Set torque*/
		void SetSignCmd(bool sign); /*Set positive moving direction (CW-0, CCW-1). Default: CW*/
		void MotorStop(); /*Stop motor*/
		void MotorBrake(); /*Brake*/

		void GetMainData(string &data); /*Read main data*/
		void ReqPosData(uint32_t &position); /*Read current position*/
		void ReqMainData(); /*Read main data*/
		void ReqTqData(uint16_t &tq); /*Read torque*/
		void ReqRpmData(int16_t &rpm); /*Read current velocity*/
		// uint16_t ReadRpm();

		// void FuncPosCmd(uint32_t target_pos);
		void FuncPosVelCmd(uint32_t target_pos, uint16_t speed); /*Position control with target speed.*/
		void FuncIncPosVelCmd(uint32_t target_pos, uint16_t speed); /*Incremental position control with target speed*/
		void PrstSave(int preset); /*Save current position to the preset address*/
		void PrstDelete(int preset); /*Delete the saved preset. 254: delete all*/
		void TourData(int speed, int delay); /*Set parameters for TOUR*/
		void PrstRecall(int preset); /*Recall the saved preset and move to that position. in_data(1~10)*/
		void TourStart(int preset); /*Move to the positions set by preset. in_data(1~10)*/
		
		void CusPkg(uint8_t pid, int num, vector<uint8_t> data); /*Send a custom packet*/

		BLDC(int id);
		BLDC(const std::string& dev, int id, BaudRate baud); /*Constructor*/
		~BLDC(); /*De-constructor*/
};

void Waitms(int n);

#endif
