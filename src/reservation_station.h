#ifndef SRC_RESERVATION_STATION_H_
#define SRC_RESERVATION_STATION_H_

#include "instruction.h"
#include "utils.h"

class ReservationStation {
	std::string name;
	RSType type;
	uint32_t execTime;
	bool busy;
	// The instruction that allocate this RS
	Instruction* inst;
public:
	ReservationStation(std::string FUName, RSType type, uint32_t execTime);
	virtual ~ReservationStation();

	bool isReadyToExecute();
	void free();
	void allocate(Instruction* inst);

	void broadcastRegReady(uint32_t physicalRegNum);

	bool isBusy() const {
		return busy;
	}

	void setBusy(bool busy) {
		this->busy = busy;
	}

	Instruction* getInst() {
		return inst;
	}

	void setInst(Instruction* inst) {
		this->inst = inst;
	}

	RSType getType() const {
		return type;
	}

	void setType(RSType type) {
		this->type = type;
	}

	uint32_t getExecTime() const {
		return execTime;
	}

	std::string toString();
};

#endif /* SRC_RESERVATION_STATION_H_ */
