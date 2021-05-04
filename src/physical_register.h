#ifndef SRC_PHYSICAL_REGISTER_H_
#define SRC_PHYSICAL_REGISTER_H_

#include "utils.h"

class PhysicalRegister {
	uint32_t regNum;
	bool ready;
public:
	PhysicalRegister();
	virtual ~PhysicalRegister();

	bool isReady() const {
		return ready;
	}

	void setReady(bool ready) {
		this->ready = ready;
	}

	uint32_t getRegNum() const {
		return regNum;
	}

	void setRegNum(uint32_t regNum) {
		this->regNum = regNum;
	}

	std::string toString();
};

#endif /* SRC_PHYSICAL_REGISTER_H_ */
