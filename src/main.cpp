#include <iostream>
#include <fstream>
#include <cstring>

#include "utils.h"
#include "cpu.h"

int main(int argc, char** argv) {
	if(argc != 3) {
		std::cout << "Error: Not enough arguments!\n";
		std::cout << "Usage : " << argv[0] << " input_file output_file\n";
		exit(-1);
	}
	std::ifstream in(argv[1]);

	uint32_t numArchRegs, numPhysicalRegs, robEntries, width, numLSQEntries;
	in >> numArchRegs >> numPhysicalRegs >> robEntries >> width >> numLSQEntries;
	CPU* cpu = new CPU(numArchRegs, numPhysicalRegs, robEntries, width, numLSQEntries);
	PRINT(numArchRegs);
	PRINT(numPhysicalRegs);
	PRINT(robEntries);
	PRINT(width);
	PRINT(numLSQEntries);
	char instType;
	uint32_t instSrcOp1, instSrcOp2, instDstOp;
	int count = 0;
	while(in >> instType) {
		in >> instSrcOp1 >> instSrcOp2 >> instDstOp;
		cpu->addInstruction(instType, instSrcOp1, instSrcOp2, instDstOp);
		std::cerr << count << " " << instType << " " << instSrcOp1 << " " << instSrcOp2 << " " << instDstOp << "\n";
		count++;
	}
	cpu->simulate();
	cpu->generateOutputFile(argv[2]);
	return 0;
}
