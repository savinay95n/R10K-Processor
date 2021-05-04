#include "reservation_station.h"

ReservationStation::ReservationStation(std::string FUName, RSType type, uint32_t execTime) :
	name(FUName), type(type), execTime(execTime) {
	busy = false;
	inst = nullptr;
}

ReservationStation::~ReservationStation() {
}

bool ReservationStation::isReadyToExecute() {
	if(inst == nullptr)
		return false;
	if(inst->getSrcPhysicalReg1().getRegNum() != -1 && inst->getSrcPhysicalReg1().isReady() == false)
		return false;
	if(inst->getSrcPhysicalReg2().getRegNum() != -1 && inst->getSrcPhysicalReg2().isReady() == false)
		return false;
	return true;
}

void ReservationStation::free() {
	inst = nullptr;
	busy = false;
}

void ReservationStation::allocate(Instruction* inst) {
	this->inst = inst;
	busy = true;
}

void ReservationStation::broadcastRegReady(uint32_t physicalRegNum) {
	if(inst == nullptr)
		return;
	if(inst->getSrcPhysicalReg1().getRegNum() == physicalRegNum)
		inst->getSrcPhysicalReg1().setReady(true);
	if(inst->getSrcPhysicalReg2().getRegNum() == physicalRegNum)
		inst->getSrcPhysicalReg2().setReady(true);
}

std::string ReservationStation::toString() {
	std::stringstream str;
	str << "[" << name << " busy=" << busy << " ";
	if(inst)
		str << inst->toString() << "]";
	else
		str << "]";
	return str.str();
}
