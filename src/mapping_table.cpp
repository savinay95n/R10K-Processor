#include "mapping_table.h"

MappingTable::MappingTable(std::string name,
		uint32_t numArchRegs, uint32_t numPhysicalRegs) :
	name(name), numArchRegs(numArchRegs), numPhysicalRegs(numPhysicalRegs) {
	mapping.resize(numArchRegs);
	for(int i = 0; i < numArchRegs; i++) {
		mapping[i].setRegNum(i);
		mapping[i].setReady(true);
	}
}

MappingTable::~MappingTable() {
}

bool MappingTable::isReady(uint32_t archRegNum) {
	if(archRegNum >= numArchRegs) {
		std::cerr << name << " " << __func__ << " invalid archRegNum : " << archRegNum << "\n";
		assert(false);
	}
	return mapping[archRegNum].isReady();
}

void MappingTable::setReadyBit(uint32_t physicalRegNum) {
	if(physicalRegNum >= numPhysicalRegs) {
		std::cerr << name << " " << __func__ << " invalid physicalRegNum : " << physicalRegNum << "\n";
		assert(false);
	}
	for(int i = 0; i < numArchRegs; i++)
		if(mapping[i].getRegNum() == physicalRegNum && mapping[i].isReady() == false)
			mapping[i].setReady(true);
}

void MappingTable::clearReadyBit(uint32_t archRegNum) {
	if(archRegNum >= numArchRegs) {
		std::cerr << name << " " << __func__ << " invalid archRegNum : " << archRegNum << "\n";
		assert(false);
	}
	mapping[archRegNum].setReady(false);
}

void MappingTable::setMapping(uint32_t archRegNum, PhysicalRegister physicalReg) {
	if(archRegNum >= numArchRegs) {
		std::cerr << name << " " << __func__ << " invalid archRegNum : " << archRegNum << "\n";
		assert(false);
	}
	if(physicalReg.getRegNum() >= numPhysicalRegs) {
		std::cerr << name << " " << __func__ << " invalid physicalRegNum : " << physicalReg.getRegNum() << "\n";
		assert(false);
	}
	mapping[archRegNum] = physicalReg;
}

PhysicalRegister MappingTable::getMapping(uint32_t archRegNum) {
	if(archRegNum >= numArchRegs) {
		std::cerr << name << " " << __func__ << " invalid archRegNum : " << archRegNum << "\n";
		assert(false);
	}
	return mapping[archRegNum];
}

std::string MappingTable::toString() {
	std::stringstream str;
	str << "[" << name << ":\n";
	for(int i = 0; i < numArchRegs; i++) {
		str << "\tAR#" << i << "->PR#" << mapping[i].getRegNum() <<
				(mapping[i].isReady() ? "+\n" : "\n");
	}
	str << "]";
	return str.str();
}
