#include "instruction.h"
#include "reservation_station.h"
#include <sstream>

Instruction::Instruction(uint32_t instrNumber, char type,
		uint32_t srcOp1, uint32_t srcOp2, uint32_t dstOp) :
		instrNumber(instrNumber), type(type), renamed(false),
		allocatedRS(nullptr),
		fetchCycle(-1), decodeCycle(-1), dispatchCycle(-1),
		issueCycle(-1), executeCycle(-1), completeCycle(-1),
		retireCycle(-1), execTime(-1) {
	switch(type) {
	case InstrType_REG:
		this->srcOp1 = srcOp1;
		this->srcOp2 = srcOp2;
		this->immediate = -1;
		this->dstOp = dstOp;
		this->memAccess = false;
		break;
	case InstrType_IMM:
		this->srcOp1 = srcOp1;
		this->srcOp2 = -1;
		this->immediate = srcOp2;
		this->dstOp = dstOp;
		this->memAccess = false;
		break;
	case InstrType_LOAD:
		this->srcOp1 = srcOp1;
		this->srcOp2 = -1;
		this->immediate = srcOp2;
		this->dstOp = dstOp;
		this->memAccess = true;
		break;
	case InstrType_STORE:
		this->srcOp1 = srcOp1;
		this->srcOp2 = dstOp;
		this->immediate = srcOp2;
		this->dstOp = -1;
		this->memAccess = true;
		break;
	default:
		assert(0 && "Unsupported Instruction type");
	}
}

Instruction::~Instruction() {
}

void Instruction::setSrcPhysicalReg1(uint32_t physicalRegNum, bool readyBit) {
	srcPhysicalReg1.setRegNum(physicalRegNum);
	srcPhysicalReg1.setReady(readyBit);
}

void Instruction::setSrcPhysicalReg2(uint32_t physicalRegNum, bool readyBit) {
	srcPhysicalReg2.setRegNum(physicalRegNum);
	srcPhysicalReg2.setReady(readyBit);
}

void Instruction::setDstPhysicalReg(uint32_t physicalRegNum, bool readyBit) {
	dstPhysicalReg.setRegNum(physicalRegNum);
	dstPhysicalReg.setReady(readyBit);
}

void Instruction::setSrcPhysicalReg1(PhysicalRegister physicalReg) {
	setSrcPhysicalReg1(physicalReg.getRegNum(), physicalReg.isReady());
}

void Instruction::setSrcPhysicalReg2(PhysicalRegister physicalReg) {
	setSrcPhysicalReg2(physicalReg.getRegNum(), physicalReg.isReady());
}

void Instruction::setDstPhysicalReg(PhysicalRegister physicalReg) {
	setDstPhysicalReg(physicalReg.getRegNum(), physicalReg.isReady());
}

PhysicalRegister& Instruction::getSrcPhysicalReg1() {
	return srcPhysicalReg1;
}

PhysicalRegister& Instruction::getSrcPhysicalReg2() {
	return srcPhysicalReg2;
}

PhysicalRegister& Instruction::getDstPhysicalReg() {
	return dstPhysicalReg;
}

bool Instruction::isLoadInst() const {
	return type == InstrType_LOAD;
}

bool Instruction::isStoreInst() const {
	return type == InstrType_STORE;
}

bool Instruction::isLoadStoreInst() const {
	return type == InstrType_LOAD || type == InstrType_STORE;
}

bool Instruction::hasIssued() const {
	return issueCycle != -1;
}

bool Instruction::hasCompleted() const {
	return completeCycle != -1;
}

bool Instruction::hasRetired() const {
	return retireCycle != -1;
}

RSType Instruction::getReservationStation() {
	switch(type) {
	case InstrType_REG:
	case InstrType_IMM:
		return RSType_ALU;
	case InstrType_LOAD:
		return RSType_LOAD;
	case InstrType_STORE:
		return RSType_STORE;
	}
	return RSType_UNKNOWN;
}

std::string Instruction::toString() const {
	std::stringstream str;
	if(renamed == false) {
		str << "[inst " << instrNumber << ":\t" << type << " [AR#" << srcOp1;
		switch(type) {
		case InstrType_REG:
			str << " AR#" << srcOp2 << "] -> AR#" << dstOp << "]";
			break;
		case InstrType_IMM:
		case InstrType_LOAD:
			str << "] #" << immediate << " -> AR#" << dstOp << "]";
			break;
		case InstrType_STORE:
			str << " AR#" << srcOp2 << "] #" << immediate << "]";
			break;
		default:
			assert(0 && "Unsupported Instruction type");
		}
	}
	else {
		str << "[inst " << instrNumber << ":\t" << type << " [PR#" <<
				srcPhysicalReg1.getRegNum() << (srcPhysicalReg1.isReady() ? '+' : ' ');
		switch(type) {
		case InstrType_REG:
			str << " PR#" << srcPhysicalReg2.getRegNum() << (srcPhysicalReg2.isReady() ? '+' : ' ') <<
			"] -> PR#" << dstPhysicalReg.getRegNum() << (dstPhysicalReg.isReady() ? '+' : ' ') << "]";
			break;
		case InstrType_IMM:
		case InstrType_LOAD:
			str << "] #" << immediate << " -> PR#" << dstPhysicalReg.getRegNum() << (dstPhysicalReg.isReady() ? '+' : ' ') << "]";
			break;
		case InstrType_STORE:
			str << " PR#" << srcPhysicalReg2.getRegNum() << (srcPhysicalReg2.isReady() ? '+' : ' ') << "] #" << immediate << "]";
			break;
		default:
			assert(0 && "Unsupported Instruction type");
		}
	}

	return str.str();
}
