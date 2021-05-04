#ifndef SRC_REORDER_BUFFER_H_
#define SRC_REORDER_BUFFER_H_

#include <vector>

#include "physical_register.h"
#include "instruction.h"
#include "utils.h"

class ROBEntry {
public:
	Instruction* getInst() const {
		return inst;
	}

	void setInst(Instruction* inst) {
		this->inst = inst;
	}

	PhysicalRegister getT() const {
		return t;
	}

	void setT(PhysicalRegister t) {
		this->t = t;
	}

	PhysicalRegister getTold() const {
		return told;
	}

	void setTold(PhysicalRegister told) {
		this->told = told;
	}

	std::string toString() {
		std::stringstream str;
		if(inst != nullptr)
			str << "[" << inst->toString() << " T=" << t.toString() << " Told=" << told.toString() << "]";
		return str.str();
	}

private:
	Instruction* inst;
	PhysicalRegister t;
	PhysicalRegister told;
};

class ReorderBuffer {
	std::vector<ROBEntry> rob;

	uint32_t head;
	uint32_t tail;
	bool full;
	uint32_t robEntries;
public:
	ReorderBuffer(uint32_t robEntries);
	virtual ~ReorderBuffer();

	bool hasFreeEntry();

	void addInstruction(Instruction* inst, PhysicalRegister T, PhysicalRegister Told);

	void retireHeadInstruction();

	ROBEntry* getHead();

	std::string toString();
};

#endif /* SRC_REORDER_BUFFER_H_ */
