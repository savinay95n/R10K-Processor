#include "physical_register.h"

PhysicalRegister::PhysicalRegister() : regNum(-1), ready(false) {

}

PhysicalRegister::~PhysicalRegister() {
}

std::string PhysicalRegister::toString() {
	std::stringstream str;
	if(regNum == -1)
		str << -1;
	else {
		str << regNum << (ready ? '+' : ' ');
	}
	return str.str();
}
