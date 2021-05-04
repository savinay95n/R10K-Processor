#include "reorder_buffer.h"

ReorderBuffer::ReorderBuffer(uint32_t robEntries) :
	head(0), tail(0), full(false), robEntries(robEntries) {
	rob.resize(robEntries);
}

ReorderBuffer::~ReorderBuffer() {
}

bool ReorderBuffer::hasFreeEntry() {
	return !full;
}

void ReorderBuffer::addInstruction(Instruction* inst, PhysicalRegister T, PhysicalRegister Told) {
	if(full) {
		std::cerr << "Trying to add an instruction to RoB while it is full\n";
		assert(false);
	}
	rob[tail].setInst(inst);
	rob[tail].setT(T);
	rob[tail].setTold(Told);
	tail++;
	tail %= robEntries;
	if(head == tail)
		full = true;
}

void ReorderBuffer::retireHeadInstruction() {
	if(head == tail && full == false) {
		std::cerr << "Trying to retire an instruction from empty RoB\n";
		assert(false);
	}
	head++;
	head %= robEntries;
	full = false;
}

ROBEntry* ReorderBuffer::getHead() {
	if(head == tail && full == false) {
		return nullptr;
	}
	return &(rob[head]);
}

std::string ReorderBuffer::toString() {
	std::stringstream str;
	str << "[ROB: h=" << head << " t=" << tail << " ";
	for(int i = head; i != tail; i++, i %= robEntries) {
		str << "\n\t" << rob[i].toString();
	}
	str << "]";
	return str.str();
}
