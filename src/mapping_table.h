#ifndef SRC_MAPPING_TABLE_H_
#define SRC_MAPPING_TABLE_H_

#include <vector>

#include "utils.h"
#include "physical_register.h"

class MappingTable {
	std::string name;
	uint32_t numArchRegs;
	uint32_t numPhysicalRegs;
	std::vector<PhysicalRegister> mapping;
public:
	MappingTable(std::string name, uint32_t numArchRegs, uint32_t numPhysicalRegs);
	virtual ~MappingTable();

	bool isReady(uint32_t archRegNum);

	void setReadyBit(uint32_t physicalRegNum);
	void clearReadyBit(uint32_t physicalRegNum);

	void setMapping(uint32_t archRegNum, PhysicalRegister physicalReg);
	PhysicalRegister getMapping(uint32_t archRegNum);

	std::string toString();
};

#endif /* SRC_MAPPING_TABLE_H_ */
