#ifndef SRC_FREE_LIST_H_
#define SRC_FREE_LIST_H_

#include <deque>

#include "physical_register.h"
#include "utils.h"

class FreeList {
	std::string name;
	uint32_t numPhysicalRegs;
	std::deque<PhysicalRegister> freeListMap;
public:
	FreeList(uint32_t numArchRegs, uint32_t numPhysicalRegs);
	virtual ~FreeList();

	bool hasRegister();
	PhysicalRegister popRegister();
	void addRegister(PhysicalRegister& physicalReg);

	std::string toString();
};

#endif /* SRC_FREE_LIST_H_ */
