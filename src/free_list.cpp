#include "free_list.h"

FreeList::FreeList(uint32_t numArchRegs, uint32_t numPhysicalRegs) :
	numPhysicalRegs(numPhysicalRegs) {
	name = __func__;
	for(int i = numArchRegs; i < numPhysicalRegs; i++) {
		PhysicalRegister physicalReg;
		physicalReg.setRegNum(i);
		physicalReg.setReady(true);
		freeListMap.push_back(physicalReg);
	}
}

FreeList::~FreeList() {
}

bool FreeList::hasRegister() {
	return freeListMap.size();
}

PhysicalRegister FreeList::popRegister() {
	if(freeListMap.empty()) {
		std::cerr << name << " " << __func__ << " pop from empty free list\n";
		assert(false);
	}
	PhysicalRegister regNum = freeListMap.front();
	freeListMap.pop_front();
	return regNum;
}

void FreeList::addRegister(PhysicalRegister& physicalRegNum) {
	if(physicalRegNum.getRegNum() >= numPhysicalRegs) {
		std::cerr << name << " " << __func__ << " invalid physicalRegNum : " << physicalRegNum.getRegNum() << "\n";
		assert(false);
	}
	freeListMap.push_back(physicalRegNum);
}

std::string FreeList::toString() {
	std::stringstream str;
	str << "[" << name << " ";
	for(int i = 0; i < freeListMap.size(); i++) {
		str << freeListMap[i].toString() << " ";
	}
	str << "]";
	return str.str();
}
