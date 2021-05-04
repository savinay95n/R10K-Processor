#ifndef SRC_INSTRUCTION_H_
#define SRC_INSTRUCTION_H_

#include <cassert>
#include <cstdint>
#include <iostream>

#include "utils.h"
#include "physical_register.h"

class ReservationStation;

class Instruction {
	uint32_t instrNumber;
	char type;
	uint32_t srcOp1;
	uint32_t srcOp2;
	uint32_t immediate;
	uint32_t dstOp;
	bool memAccess;

	// For using in reservation station
	PhysicalRegister srcPhysicalReg1;
	PhysicalRegister srcPhysicalReg2;
	PhysicalRegister dstPhysicalReg;

	ReservationStation* allocatedRS;

	bool renamed;
	uint32_t fetchCycle;
	uint32_t decodeCycle;
	uint32_t dispatchCycle;
	uint32_t issueCycle;
	uint32_t executeCycle;
	uint32_t completeCycle;
	uint32_t retireCycle;

	uint32_t execTime;
public:
	Instruction(uint32_t instrNumber, char type,
			uint32_t srcOp1, uint32_t srcOp2, uint32_t dstOp);
	virtual ~Instruction();

	void setSrcPhysicalReg1(uint32_t physicalRegNum, bool readyBit);
	void setSrcPhysicalReg2(uint32_t physicalRegNum, bool readyBit);
	void setDstPhysicalReg(uint32_t physicalRegNum, bool readyBit);

	void setSrcPhysicalReg1(PhysicalRegister physicalReg);
	void setSrcPhysicalReg2(PhysicalRegister physicalReg);
	void setDstPhysicalReg(PhysicalRegister physicalReg);

	PhysicalRegister& getSrcPhysicalReg1();
	PhysicalRegister& getSrcPhysicalReg2();
	PhysicalRegister& getDstPhysicalReg();

	bool isLoadInst() const;
	bool isStoreInst() const;
	bool isLoadStoreInst() const;
	bool hasIssued() const;
	bool hasCompleted() const;
	bool hasRetired() const;

	RSType getReservationStation();

	std::string toString() const;

	uint32_t getCompleteCycle() const {
		return completeCycle;
	}

	void setCompleteCycle(uint32_t completeCycle) {
		this->completeCycle = completeCycle;
	}

	uint32_t getDecodeCycle() const {
		return decodeCycle;
	}

	void setDecodeCycle(uint32_t decodeCycle) {
		this->decodeCycle = decodeCycle;
	}

	uint32_t getDstOp() const {
		return dstOp;
	}

	void setDstOp(uint32_t dstOp) {
		this->dstOp = dstOp;
	}

	uint32_t getDispatchCycle() const {
		return dispatchCycle;
	}

	void setDispatchCycle(uint32_t dispatchCycle) {
		this->dispatchCycle = dispatchCycle;
	}

	uint32_t getExecuteCycle() const {
		return executeCycle;
	}

	void setExecuteCycle(uint32_t executeCycle) {
		this->executeCycle = executeCycle;
	}

	uint32_t getFetchCycle() const {
		return fetchCycle;
	}

	void setFetchCycle(uint32_t fetchCycle) {
		this->fetchCycle = fetchCycle;
	}

	uint32_t getImmediate() const {
		return immediate;
	}

	void setImmediate(uint32_t immediate) {
		this->immediate = immediate;
	}

	uint32_t getIssueCycle() const {
		return issueCycle;
	}

	void setIssueCycle(uint32_t issueCycle) {
		this->issueCycle = issueCycle;
	}

	bool isMemAccess() const {
		return memAccess;
	}

	void setMemAccess(bool memAccess) {
		this->memAccess = memAccess;
	}

	bool isRenamed() const {
		return renamed;
	}

	void setRenamed(bool renamed) {
		this->renamed = renamed;
	}

	uint32_t getRetireCycle() const {
		return retireCycle;
	}

	void setRetireCycle(uint32_t retireCycle) {
		this->retireCycle = retireCycle;
	}

	uint32_t getSrcOp1() const {
		return srcOp1;
	}

	void setSrcOp1(uint32_t srcOp1) {
		this->srcOp1 = srcOp1;
	}

	uint32_t getSrcOp2() const {
		return srcOp2;
	}

	void setSrcOp2(uint32_t srcOp2) {
		this->srcOp2 = srcOp2;
	}

	char getType() const {
		return type;
	}

	void setType(char type) {
		this->type = type;
	}

	uint32_t getExecTime() const {
		return execTime;
	}

	void setExecTime(uint32_t execTime) {
		this->execTime = execTime;
	}

	ReservationStation* getAllocatedRs() const {
		return allocatedRS;
	}

	void setAllocatedRs(ReservationStation* allocatedRs) {
		allocatedRS = allocatedRs;
	}
};

#endif /* SRC_INSTRUCTION_H_ */
