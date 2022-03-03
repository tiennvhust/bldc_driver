#include "CppLinuxSerial/SerialPort.hpp"
#include <algorithm>
#include <vector>
#include <string>
#include "bldc_driver/pid.h"
#include "bldc_driver/BLDC.hpp"
#include <iostream>
#include <bits/stdc++.h>
#include <unistd.h>
using namespace mn::CppLinuxSerial;
using namespace std;

uint16_t Byte2Int(uint8_t byLow, uint8_t byHigh) {
	return (byLow | (uint16_t)byHigh<<8);
}
// Make long type Data from four bytes
uint32_t Byte2LInt(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4) {
	return((uint32_t)data1 | (uint32_t)data2 << 8 | (uint32_t)data3 << 16 | (uint32_t)data4 << 24);
}

/*Wait for n ms*/
void Waitms(int n) {
	usleep(n*1000);
}

#ifdef DEBUG

void BLDC::PrintMainData() {
	cout << dec << "Speed (rpm): " << int(rpm) << "\n" \
		 << "Current (mA): " << int(current)*100 << "\n" \
		 << "Control: " << ctrl << "\n" \
		 << "Ref speed (rpm): " << ref << "\n" \
		 << "Control output: " << out << "\n" \
		 << "Status: " << stat << "\n" \
		 << "Position: " << pos << "\n" \
		 << "Break duty: " << brk << "\n" \
		 << "Temperature: " << temp << endl;
}

#endif

int CheckSum(string& data)
{
	uint16_t i;
	uint8_t csum;
	csum = 0;
	for(i=0; i < data.size(); i++) {
		csum += data[i];
	}
	if(csum==0) return 0;
	else return -1;
}

/*Add checksum to packet*/
void AddCheckSum(vector<uint8_t> &data) {
	uint8_t chksum = 0;
	uint16_t i;
	for(i = 0; i < data.size(); i++) chksum += data[i];
	chksum = ~chksum + 1;
	data.push_back(chksum);
	#ifdef DEBUG
	// cout << "Check sum: " << chksum - 0xc9 << " " << data.back() - 0xc9 << endl;
	#endif
}

/*Make a new serial packet*/
vector<uint8_t> BLDC::MakePkg(uint8_t pid, int num, vector<uint8_t> data) {
	vector<uint8_t> pkg;
	pkg.push_back(183);
	pkg.push_back(184);
	pkg.push_back(id);
	pkg.push_back(pid);
	pkg.push_back(num);
	pkg.insert(pkg.end(), data.begin(), data.end());
	AddCheckSum(pkg);
	return pkg;
}

/*Send packet*/
void BLDC::SendPkg(vector<uint8_t> pkg) {
	string str(pkg.begin(), pkg.end());
	port.Write(str);
	#ifdef DEBUG
		cout << "Sent packet: ";
		for(int i = 0; i < pkg.size(); i++)
		{
			cout << hex << int(pkg[i]) << " ";
		}
		cout << endl;
	#endif
}

/*Receive packet*/
int BLDC::RevPkg(string &rev_data) {
	rev_data.erase();
	string buffer;
	int count = 3;
	do {
		Waitms(DEFAULT_WAIT_MS);
		port.Read(buffer);
		rev_data.append(buffer);
		if(--count == 0) {
			rev_data.erase();
			cout << "Warning: Received packet faulty. Skiped packet." << endl;
			return -1;
		}
	} while(CheckSum(rev_data));
	#ifdef DEBUG
		cout << "Received packet: ";
		for(int i = 0; i < rev_data.size(); i++)
		{
			cout << hex << int(rev_data[i]) << " ";
		}
		cout << endl;
	#endif
	return 0;
}

/*Motor init*/
void BLDC::MotorInit() {
	PidCmd(CMD_INIT_SET);
}

/*Translate main data from received packet*/
void BLDC::GetMainData(string &rev_data) {
	string data = rev_data.substr(5);
	rpm = Byte2Int(data[0], data[1]);
	current = Byte2Int(data[2], data[3]);
	ctrl = data[4];
	ref = Byte2Int(data[5], data[6]);
	out = Byte2Int(data[7], data[8]);
	stat = data[9];
	pos = Byte2LInt(data[10], data[11], data[12], data[13]);
	brk = data[14];
	temp = data[15];
	return;
}

/*Send a custom packet*/
void BLDC::CusPkg(uint8_t pid, int num, vector<uint8_t> data) {
	vector<uint8_t> pkg = MakePkg(pid, num, data);
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Change Baudrate
  INPUT: 1~5
  9600, 19200, 38400, 57600, 115200
  */
void BLDC::ChangeBaud(uint8_t baud) {
	vector<uint8_t> data;
	data.push_back(0xaa);
	data.push_back(baud);
	vector<uint8_t> pkg = MakePkg(PID_BAUD_RATE, 2, data);
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Request main data from BLDC
  Input parameters: PID, receive data buffer*/
int BLDC::ReqData(uint8_t pid, string &rev_data) {
	vector<uint8_t> data;
	data.push_back(pid);
	vector<uint8_t> pkg = MakePkg(PID_REQ_PID_DATA, 1, data);
	SendPkg(pkg);
	if(RevPkg(rev_data)) return -1;
	return 0;
}

void BLDC::ReqMainData() {
	string main_data;
	if(!(ReqData(PID_MAIN_DATA, main_data))) {
		GetMainData(main_data);
		#ifdef DEBUG
		PrintMainData();
		#endif
	} else cout << "ERROR: Get data failed." << endl;
}

/*Send PID commands
  Input parameters: command PID*/
void BLDC::PidCmd(uint8_t cmd) {
	vector<uint8_t> data;
	data.push_back(cmd);
	vector<uint8_t> pkg = MakePkg(PID_COMMAND, 1, data);
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Send veloctity commands*/
void BLDC::VelCmd(int rpm) {
	vector<uint8_t> data;
	data.push_back((uint8_t)(rpm & 0xFF));
	data.push_back((uint8_t)(rpm >> 8));
	vector<uint8_t> pkg = MakePkg(PID_VEL_CMD, 2, data);
	SendPkg(pkg);
}

/*Send tourque command*/
void BLDC::TqCmd(int tq) {
	vector<uint8_t> data;
	data.push_back(tq & 0xFF);
	data.push_back(tq >> 8);
	vector<uint8_t> pkg = MakePkg(PID_TQ_CMD, 2, data);
	SendPkg(pkg);
}

/*Set positive moving direction. Default: CW*/
void BLDC::SetSignCmd(bool sign) {
	vector<uint8_t> data;
	data.push_back(sign ? 1 : 0);
	vector<uint8_t> pkg = MakePkg(PID_INV_SIGN_CMD, 1, data);
	SendPkg(pkg);
}


/*Function commands*/
/*Position*/
// void BLDC::FuncPosCmd(uint32_t target_pos) {
// 	vector<uint8_t> data;
// 	uint8_t D1 = target_pos & 0xFF;
// 	uint8_t D2 = (target_pos >> 8) & 0XFF;
// 	uint8_t D3 = (target_pos >> 16) & 0xFF;
// 	uint8_t D4 = target_pos >> 24;
// 	data.push_back(D1);
// 	data.push_back(D2);
// 	data.push_back(D3);
// 	data.push_back(D4);
// 	vector<uint8_t> pkg = MakePkg(PID_POSI_CMD, 4, data);
// 	SendPkg(pkg);
// 	string rev_data;
// 	Waitms(100);
// 	RevPkg(rev_data);
// 	/****/
// 	pkg.clear();
// 	data.clear();
// 	data.push_back(2);
// 	data.push_back(0);
// 	pkg = MakePkg(PID_FUNC_CMD_TYPE, 2, data);
// 	SendPkg(pkg);
// 	Waitms(100);
// 	RevPkg(rev_data);
// 	/*****/
// 	pkg.clear();
// 	data.clear();
// 	data.push_back(1);
// 	data.push_back(0);
// 	pkg = MakePkg(PID_FUNC_CMD, 2, data);
// 	SendPkg(pkg);
// 	Waitms(100);
// 	RevPkg(rev_data);
// }

/*Position control with max. target speed.
  INPUT: position, max speed
  */
void BLDC::FuncPosVelCmd(uint32_t target_pos, uint16_t speed) {
	vector<uint8_t> data;
	data.push_back(target_pos & 0xFF);
	data.push_back((target_pos >> 8) & 0XFF);
	data.push_back((target_pos >> 16) & 0xFF);
	data.push_back(target_pos >> 24);
	data.push_back(speed & 0xFF);
	data.push_back(speed >> 8);
	vector<uint8_t> pkg = MakePkg(PID_POSI_VEL_CMD, 6, data);
	SendPkg(pkg);
}

void BLDC::FuncIncPosVelCmd(uint32_t target_pos, uint16_t speed) {
	vector<uint8_t> data;
	data.push_back(target_pos & 0xFF);
	data.push_back((target_pos >> 8) & 0XFF);
	data.push_back((target_pos >> 16) & 0xFF);
	data.push_back(target_pos >> 24);
	data.push_back(speed & 0xFF);
	data.push_back(speed >> 8);
	vector<uint8_t> pkg = MakePkg(PID_INC_POSI_VEL_CMD, 6, data);
	SendPkg(pkg);
}

/*Read position data*/
void BLDC::ReqPosData(uint32_t &position) {
	string pos_data;
	ReqData(PID_POSI_DATA, pos_data);
	position = Byte2LInt(pos_data[5], pos_data[6], pos_data[7], pos_data[8]);
#ifdef DEBUG
	cout << dec << "Current position: " << position << endl;
#endif
}

/*Read tourque data*/
void BLDC::ReqTqData(uint16_t &tq) {
	string tq_data;
	ReqData(PID_TQ_DATA, tq_data);
	tq = Byte2Int(tq_data[5], tq_data[6]);
#ifdef DEBUG
	cout << dec << "Tourque value: " << tq*100 << endl;
#endif
}

// /*Return velocity data*/
// uint16_t BLDC::ReadRpm() {
// 	return rpm;
// }

void BLDC::ReqRpmData(int16_t &rpm) {
	string rpm_data;
	if(ReqData(PID_INT_RPM_DATA, rpm_data)) cout << "ERROR: Request data failed." << endl;
	rpm = Byte2Int(rpm_data[5], rpm_data[6]);
}


/*Save current position to the preset address
  INPUT: 0~9
  */
void BLDC::PrstSave(int preset) {
	if(preset < 0 | preset > 9) {
		cout << "ERROR: Invalid preset." << endl;
		return;
	}
	vector<uint8_t> data;
	string pos_data;
	data.push_back(preset);
	/*Save current position*/
	vector<uint8_t> pkg = MakePkg(PID_PRESET_SAVE, 1, data);
	/*Save current position to prst_list*/
	ReqPosData(prst_list[preset]);
	#ifdef DEBUG

	for(int i = 0; i < pos_data.size(); i++)
	{
		cout << hex << int(pos_data[i]) << " ";
	}
	cout << endl;
	cout << dec << "Preset position: " << prst_list[preset] << endl;
	#endif
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Delete preset data*/
void BLDC::PrstDelete(int preset) {
	if(preset == 254) prst_list.clear();
	if(preset > 0 & preset < 9) prst_list.erase(prst_list.begin() + preset);
	return;
}

/*Recall the saved preset and move to that position.
  INPUT: 1~10
  */
void BLDC::PrstRecall(int preset) {
	if(preset < 0 | preset > 10) {
		cout << "ERROR: Invalid preset." << endl;
		return;
	}
	vector<uint8_t> data;
	data.push_back(preset);
	vector<uint8_t> pkg = MakePkg(PID_PRESET_RECALL, 1, data);
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Set parameters for TOUR
  function(target speed,
  pause time delay - 0.1s unit)*/
void BLDC::TourData(int speed, int delay) {
	vector<uint8_t> data;
	data.push_back(speed%256);
	data.push_back(speed/256);
	data.push_back(delay%256);
	data.push_back(delay/256);
	vector<uint8_t> pkg = MakePkg(PID_TOUR_DATA, 4, data);
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Move to the positions set by preset
  DATA: 0 - stop function
        1 - start function, end with tq-off state
        2 - start function, end with servo-lock state
        */
void BLDC::TourStart(int preset) {
	vector<uint8_t> data;
	data.push_back(preset);
	vector<uint8_t> pkg = MakePkg(PID_TOUR_START, 1, data);
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Stop motor naturally/ free state/ Tq-off*/
void BLDC::MotorStop() {
	vector<uint8_t> pkg = MakePkg(PID_TQ_OFF, 1, {0});
	SendPkg(pkg);
}

/*Stop motor urgently/ Electric brake*/
void BLDC::MotorBrake() {
	vector<uint8_t> pkg = MakePkg(PID_BRAKE, 1, {0});
	SendPkg(pkg);
	string rev_data;
	RevPkg(rev_data);
}

/*Constructor with device name (ex: /dev/ttyS0), id (ex: 02), baudrate (ex: 9600)*/
BLDC::BLDC(const std::string& dev, int id, BaudRate baud) {
	this->id = id;
	rpm = 0;
	current = 0;
	ctrl = 0;
	ref = 0;
	out = 0;
	stat = 0;
	pos = 0;
	brk = 0;
	temp = 0;
	prst_list.reserve(10);
	port.SetDevice(dev);
	port.SetBaudRate(baud);
	port.SetTimeout(100); // Block when reading until any data is received
	port.Open();
}

BLDC::BLDC(int id) {
	this->id = id;
	rpm = 0;
	current = 0;
	ctrl = 0;
	ref = 0;
	out = 0;
	stat = 0;
	pos = 0;
	brk = 0;
	temp = 0;
	prst_list.reserve(10);
	switch(this->id) {
		case 1:
			port.SetDevice(DEVICE_1);
			break;
		case 2:
			port.SetDevice(DEVICE_2);
			break;
		case 3:
			port.SetDevice(DEVICE_3);
			break;
		case 4:
			port.SetDevice(DEVICE_4);
			break;
		default:
			port.SetDevice(DEVICE_1);
	}
	port.SetBaudRate(DEFAULT_BAUDRATE);
	port.SetTimeout(100); // Block when reading until any data is received
	port.Open();
}

/*De-constructor*/
BLDC::~BLDC(void) {
	port.Close();
}
