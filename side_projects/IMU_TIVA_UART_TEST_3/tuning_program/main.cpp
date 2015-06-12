#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdint.h>
#include "serial.h"
#include "seriallcm.h"
#include "target_t.h"
#include "feedback_t.h"

#define max_int32 2147483647
#define csvout std::cout

void printTemplate(std::ostream &out);

void printCSVHeader();

void printCSVdof(dof_feedback_t message);

void printCSVdji(djiout_feedback_t message);

void str_log_handler(str_log_t *message) { 
	std::cout << message->mess << std::endl;
}

void dof_feedback_handler(dof_feedback_t *message) {
	printCSVdof(*message);
}

void dji_feedback_handler(djiout_feedback_t *message) {
	printCSVdji(*message);
}

void interpretFile(std::istream &in, tuning_t &to);

void fillField(tuning_t &to, std::string field, std::string value);

int main(int argc, char **argv) {
	if (argc < 3) {
		std::cout << "Insufficient Parameter" << std::endl;
		return 0;
	}

	SerialLCM slcm(argv[1]);

	if (!strcmp(argv[2], "-send")) { // send a tuning message!
		tuning_t message;
		std::string infileName;
		if(argc == 3) // No file specified
		{
			// print out template
			std::ofstream out("temp.tun");
			printTemplate(out);
			out.close();
			// let user edit template
			system("vim temp.tun");
			// set to read in from template
			infileName = "temp.tun";
		}else // file specified
		{
			infileName = std::string(argv[3]);
		}
		// file for reading in
		std::ifstream infile(infileName.c_str());
		// interpret file for message
		interpretFile(infile, message);
		// send
		slcm.send(&message);
	}

	if (!strcmp(argv[2], "-recv")) { // receive messages!
		slcm.set_dof_feedback_handler(dof_feedback_handler);
		slcm.set_dji_feedback_handler(dji_feedback_handler);
		slcm.set_str_log_handler(str_log_handler);
		for (;;) {
			slcm.receive_and_process();
		}
	}

}

void printCSVHeader() {
	csvout << "time, state_x, state_xdot, state_y, state_ydot, state_z, state_zdot, state_h, state_hdot, state_r, state_rdot, state_p, state_pdot, rawstate_x, rawstate_xdot, rawstate_y, rawstate_ydot, rawstate_z, rawstate_zdot, rawstate_h, rawstate_hdot, rawstate_r, rawstate_rdot, rawstate_p, rawstate_pdot, ";
	csvout << "setpt_x, setpt_xdot, setpt_y, setpt_ydot, setpt_z, setpt_zdot, setpt_h,";
	csvout << "kpx, kix, kdx, kpxdot, kixdot, kdxdot, kpy, kiy, kdy, kpydot, kpz, kiz, kdz, kpzdot, kizdot, kdzdot, kph, kih, kdh, ";
	csvout << "dji_r, dji_p, dji_hdot, dji_fz";
}

void printCSVdof(dof_feedback_t message) {
	const std::string c = ", ";
	if (message.dof = (int32_t)'x') {
		csvout <<
			message.timestamp << c <<
			// FILTERED
			message.filter_state << c <<//xstates
			message.dot_filter_state << c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			c <<//hstates
			c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// RAW
			message.raw_state << c <<//xstates
			message.dot_raw_state << c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			c <<//hstates
			c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// SETPOINTS
			message.setpt << c <<//x
			message.dot_setpt << c <<//xdot
			c <<//y
			c <<//ydot
			c <<//z
			c <<//zdot
			c <<//h
			// GAINS, p, i, d
			message.kp << c <<//x
			message.ki << c <<
			message.kd << c <<
			message.dot_kp << c <<//xdot
			message.dot_ki << c <<
			message.dot_kd << c <<
			c <<//y
			c <<
			c <<
			c <<//ydot
			c <<
			c <<
			c <<//z
			c <<
			c <<
			c <<//zdot
			c <<
			c <<
			c <<//h
			c <<
			c <<
			// DJI OUTPUT
			c <<//r
			c <<//p
			c <<//hdot
			c <<//fz
			std::endl;
	}
	if (message.dof = (int32_t)'y') {
		csvout <<
			message.timestamp << c <<
			// FILTERED
			c <<//xstates
			c <<
			message.filter_state << c <<//ystates
			message.dot_filter_state << c <<
			c <<//zstates
			c <<
			c <<//hstates
			c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// RAW
			c <<//xstates
			c <<
			message.raw_state << c <<//ystates
			message.dot_raw_state << c <<
			c <<//zstates
			c <<
			c <<//hstates
			c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// SETPOINTS
			c <<//x
			c <<//xdot
			message.setpt << c <<//y
			message.dot_setpt << c <<//ydot
			c <<//z
			c <<//zdot
			c <<//h
			// GAINS, p, i, d
			c <<//x
			c <<
			c <<
			c <<//xdot
			c <<
			c <<
			message.kp << c <<//y
			message.ki << c <<
			message.kd << c <<
			message.dot_kp << c <<//ydot
			message.dot_ki << c <<
			message.dot_kd << c <<
			c <<//z
			c <<
			c <<
			c <<//zdot
			c <<
			c <<
			c <<//h
			c <<
			c <<
			// DJI OUTPUT
			c <<//r
			c <<//p
			c <<//hdot
			c <<//fz
			std::endl;
	}
	if (message.dof = (int32_t)'z') {
		csvout <<
			message.timestamp << c <<
			// FILTERED
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			message.filter_state << c <<//zstates
			message.dot_filter_state << c <<
			c <<//hstates
			c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// RAW
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			message.raw_state << c <<//zstates
			message.dot_raw_state << c <<
			c <<//hstates
			c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// SETPOINTS
			c <<//x
			c <<//xdot
			c <<//y
			c <<//ydot
			message.setpt << c <<//z
			message.dot_setpt << c <<//zdot
			c <<//h
			// GAINS, p, i, d
			c <<//x
			c <<
			c <<
			c <<//xdot
			c <<
			c <<
			c <<//y
			c <<
			c <<
			c <<//ydot
			c <<
			c <<
			message.kp << c <<//z
			message.ki << c <<
			message.kd << c <<
			message.dot_kp << c <<//zdot
			message.dot_ki << c <<
			message.dot_kd << c <<
			c <<//h
			c <<
			c <<
			// DJI OUTPUT
			c <<//r
			c <<//p
			c <<//hdot
			c <<//fz
			std::endl;
	}
	if (message.dof = (int32_t)'h') {
		csvout <<
			message.timestamp << c <<
			// FILTERED
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			message.filter_state << c <<//hstates
			message.dot_filter_state << c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// RAW
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			message.raw_state << c <<//hstates
			message.dot_raw_state << c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// SETPOINTS
			c <<//x
			c <<//xdot
			c <<//y
			c <<//ydot
			c <<//z
			c <<//zdot
			message.setpt << c <<//h
			// GAINS, p, i, d
			c <<//x
			c <<
			c <<
			c <<//xdot
			c <<
			c <<
			c <<//y
			c <<
			c <<
			c <<//ydot
			c <<
			c <<
			c <<//z
			c <<
			c <<
			c <<//zdot
			c <<
			c <<
			message.kp << c <<//h
			message.ki << c <<
			message.kd << c <<
			// DJI OUTPUT
			c <<//r
			c <<//p
			c <<//hdot
			c <<//fz
			std::endl;
	}
	if (message.dof = (int32_t)'r') {
		csvout <<
			message.timestamp << c <<
			// FILTERED
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			message.filter_state << c <<//hstates
			message.dot_filter_state << c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// RAW
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			c <<//hstates
			c <<
			message.raw_state << c <<//rstates
			message.dot_raw_state << c <<
			c <<//pstates
			c <<
			// SETPOINTS
			c <<//x
			c <<//xdot
			c <<//y
			c <<//ydot
			c <<//z
			c <<//zdot
			c <<//h
			// GAINS, p, i, d
			c <<//x
			c <<
			c <<
			c <<//xdot
			c <<
			c <<
			c <<//y
			c <<
			c <<
			c <<//ydot
			c <<
			c <<
			c <<//z
			c <<
			c <<
			c <<//zdot
			c <<
			c <<
			c <<//h
			c <<
			c <<
			// DJI OUTPUT
			c <<//r
			c <<//p
			c <<//hdot
			c <<//fz
			std::endl;
	}
	if (message.dof = (int32_t)'p') {
		csvout <<
			message.timestamp << c <<
			// FILTERED
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			message.filter_state << c <<//hstates
			message.dot_filter_state << c <<
			c <<//rstates
			c <<
			c <<//pstates
			c <<
			// RAW
			c <<//xstates
			c <<
			c <<//ystates
			c <<
			c <<//zstates
			c <<
			c <<//hstates
			c <<
			c <<//rstates
			c <<
			message.raw_state << c <<//pstates
			message.dot_raw_state << c <<
			// SETPOINTS
			c <<//x
			c <<//xdot
			c <<//y
			c <<//ydot
			c <<//z
			c <<//zdot
			c <<//h
			// GAINS, p, i, d
			c <<//x
			c <<
			c <<
			c <<//xdot
			c <<
			c <<
			c <<//y
			c <<
			c <<
			c <<//ydot
			c <<
			c <<
			c <<//z
			c <<
			c <<
			c <<//zdot
			c <<
			c <<
			c <<//h
			c <<
			c <<
			// DJI OUTPUT
			c <<//r
			c <<//p
			c <<//hdot
			c <<//fz
			std::endl;
	}

}

void printCSVdji(djiout_feedback_t message) {
	const std::string c = ", ";
	csvout <<
		message.timestamp << c <<
		// FILTERED
		c <<//xstates
		c <<
		c <<//ystates
		c <<
		c <<//zstates
		c <<
		c <<//hstates
		c <<
		c <<//rstates
		c <<
		c <<//pstates
		c <<
		// RAW
		c <<//xstates
		c <<
		c <<//ystates
		c <<
		c <<//zstates
		c <<
		c <<//hstates
		c <<
		c <<//rstates
		c <<
		c <<//pstates
		c <<
		// SETPOINTS
		c <<//x
		c <<//xdot
		c <<//y
		c <<//ydot
		c <<//z
		c <<//zdot
		c <<//h
		// GAINS, p, i, d
		c <<//x
		c <<
		c <<
		c <<//xdot
		c <<
		c <<
		c <<//y
		c <<
		c <<
		c <<//ydot
		c <<
		c <<
		c <<//z
		c <<
		c <<
		c <<//zdot
		c <<
		c <<
		c <<//h
		c <<
		c <<
		// DJI OUTPUT
		message.roll << c <<//r
		message.pitch << c <<//p
		message.yawdot << c <<//hdot
		message.fz << c <<//fz
		std::endl;
}



void interpretFile(std::istream &in, tuning_t &to) {
	std::string temp;
	std::string field;
	std::string value;

	while (in.good()) {
		in >> temp;
		if (temp != ">") {
			getline(in, temp);
		} else {
			in >> field;
			in >> value;
			fillField(to, field, value);
		}
	}
}

void fillField(tuning_t &to, std::string field, std::string value) {
	if (field == "cmdbit_setpid_x")    to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_X:0;
	if (field == "cmdbit_setpid_y")    to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_Y:0;
	if (field == "cmdbit_setpid_z")    to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_Z:0;
	if (field == "cmdbit_setpid_h")    to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_H:0;
	if (field == "cmdbit_setpid_xdot") to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_XDOT:0;
	if (field == "cmdbit_setpid_ydot") to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_YDOT:0;
	if (field == "cmdbit_setpid_zdot") to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_ZDOT:0;
	if (field == "cmdbit_setpid_hdot") to.cmd |= std::stoi(value)? CMD_TUNING_SETPID_HDOT:0;
	if (field == "cmdbit_setpt_x")     to.cmd |= std::stoi(value)? CMD_TUNING_SETPOINT_X:0;
	if (field == "cmdbit_setpt_y")     to.cmd |= std::stoi(value)? CMD_TUNING_SETPOINT_Y:0;
	if (field == "cmdbit_setpt_z")     to.cmd |= std::stoi(value)? CMD_TUNING_SETPOINT_Z:0;
	if (field == "cmdbit_setpt_h")     to.cmd |= std::stoi(value)? CMD_TUNING_SETPOINT_H:0;
	if (field == "cmdbit_setpt_xdot")  to.cmd |= std::stoi(value)? CMD_TUNING_DOTSETPOINT_X:0;
	if (field == "cmdbit_setpt_ydot")  to.cmd |= std::stoi(value)? CMD_TUNING_DOTSETPOINT_Y:0;
	if (field == "cmdbit_setpt_zdot")  to.cmd |= std::stoi(value)? CMD_TUNING_DOTSETPOINT_Z:0;
	if (field == "cmdbit_setpt_hdot")  to.cmd |= std::stoi(value)? CMD_TUNING_DOTSETPOINT_H:0;
	if (field == "cmdbit_land")        to.cmd |= std::stoi(value)? CMD_TUNING_LAND:0;
	if (field == "cmdbit_takeoff")     to.cmd |= std::stoi(value)? CMD_TUNING_TAKEOFF:0;

	if (field == "setpt_x")    to.x    = std::stof(value);
	if (field == "setpt_y")    to.y    = std::stof(value);
	if (field == "setpt_z")    to.z    = std::stof(value);
	if (field == "setpt_h")    to.h    = std::stof(value);
	if (field == "setpt_xdot") to.xdot = std::stof(value);
	if (field == "setpt_ydot") to.ydot = std::stof(value);
	if (field == "setpt_zdot") to.zdot = std::stof(value);
	
	if (field == "kp_x")    to.KPX    = std::stof(value);
	if (field == "ki_x")    to.KIX    = std::stof(value);
	if (field == "kd_x")    to.KDX    = std::stof(value);
	if (field == "kp_y")    to.KPY    = std::stof(value);
	if (field == "ki_y")    to.KIY    = std::stof(value);
	if (field == "kd_y")    to.KDY    = std::stof(value);
	if (field == "kp_z")    to.KPZ    = std::stof(value);
	if (field == "ki_z")    to.KIZ    = std::stof(value);
	if (field == "kd_z")    to.KDZ    = std::stof(value);
	if (field == "kp_h")    to.KPH    = std::stof(value);
	if (field == "ki_h")    to.KIH    = std::stof(value);
	if (field == "kd_h")    to.KDH    = std::stof(value);
	if (field == "kp_xdot") to.KPXdot = std::stof(value);
	if (field == "ki_xdot") to.KIXdot = std::stof(value);
	if (field == "kd_xdot") to.KDXdot = std::stof(value);
	if (field == "kp_ydot") to.KPYdot = std::stof(value);
	if (field == "ki_ydot") to.KIYdot = std::stof(value);
	if (field == "kd_ydot") to.KDYdot = std::stof(value);
	if (field == "kp_zdot") to.KPZdot = std::stof(value);
	if (field == "ki_zdot") to.KIZdot = std::stof(value);
	if (field == "kd_zdot") to.KDZdot = std::stof(value);
	
}

void printTemplate(std::ostream &out) {
	out << "Tuning Program V2" << std::endl;
	out << "Message" << std::endl;
	out << std::endl;
	out << "Set timestamp to 0 to auto insert last 32 bits of seconds since last epoch" <<
		std::endl;
	out << "> timestamp 0" << std::endl;
	out << std::endl;
	out << "> cmdbit_setpid_x 0" << std::endl;
	out << "> cmdbit_setpid_y 0" << std::endl;
	out << "> cmdbit_setpid_z 0" << std::endl;
	out << "> cmdbit_setpid_h 0" << std::endl;
	out << "> cmdbit_setpid_xdot 0" << std::endl;
	out << "> cmdbit_setpid_ydot 0" << std::endl;
	out << "> cmdbit_setpid_zdot 0" << std::endl;
	out << "> cmdbit_setpid_hdot 0" << std::endl;
	out << "> cmdbit_setpt_x 0" << std::endl;
	out << "> cmdbit_setpt_y 0" << std::endl;
	out << "> cmdbit_setpt_z 0" << std::endl;
	out << "> cmdbit_setpt_h 0" << std::endl;
	out << "> cmdbit_setpt_xdot 0" << std::endl;
	out << "> cmdbit_setpt_ydot 0" << std::endl;
	out << "> cmdbit_setpt_zdot 0" << std::endl;
	out << "> cmdbit_setpt_hdot 0" << std::endl;
	out << "> cmdbit_land 0" << std::endl;
	out << "> cmdbit_takeoff 0" << std::endl;
	out << std::endl;
	out << "> setpt_x 0" << std::endl;
	out << "> setpt_y 0" << std::endl;
	out << "> setpt_z 0" << std::endl;
	out << "> setpt_h 0" << std::endl;
	out << "> setpt_xdot 0" << std::endl;
	out << "> setpt_ydot 0" << std::endl;
	out << "> setpt_zdot 0" << std::endl;
	out << std::endl;
	out << "> kp_x 0" << std::endl;
	out << "> ki_x 0" << std::endl;
	out << "> kd_x 0" << std::endl;
	out << "> kp_y 0" << std::endl;
	out << "> ki_y 0" << std::endl;
	out << "> kd_y 0" << std::endl;
	out << "> kp_z 0" << std::endl;
	out << "> ki_z 0" << std::endl;
	out << "> kd_z 0" << std::endl;
	out << "> kp_h 0" << std::endl;
	out << "> ki_h 0" << std::endl;
	out << "> kd_h 0" << std::endl;
	out << "> kp_xdot 0" << std::endl;
	out << "> ki_xdot 0" << std::endl;
	out << "> kd_xdot 0" << std::endl;
	out << "> kp_ydot 0" << std::endl;
	out << "> ki_ydot 0" << std::endl;
	out << "> kd_ydot 0" << std::endl;
	out << "> kp_zdot 0" << std::endl;
	out << "> ki_zdot 0" << std::endl;
	out << "> kd_zdot 0" << std::endl;
}
