#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_

#include <cassert>
#include <cstdint>
#include <iostream>
#include <sstream>

#define PRINT(X) std::cerr << #X << "=" << X << "\n"

#define InstrType_REG 'R'
#define InstrType_IMM 'I'
#define InstrType_LOAD 'L'
#define InstrType_STORE 'S'

enum RSType {
	RSType_ALU,
	RSType_LOAD,
	RSType_STORE,
	RSType_UNKNOWN
};

#endif /* SRC_UTILS_H_ */
