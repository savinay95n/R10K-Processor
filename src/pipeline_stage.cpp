#include "pipeline_stage.h"

PipelineStage::PipelineStage(std::string name, uint32_t width) :
	name(name), width(width) {
}

PipelineStage::~PipelineStage() {
}

bool PipelineStage::push(Instruction* inst) {
	queue.push_back(inst);
	return true;
}

bool PipelineStage::isEmpty() {
	return queue.empty();
}

Instruction* PipelineStage::front() {
	if(queue.empty()) {
		std::cerr << name << " " << __func__ << " empty pipeline stage\n";
		assert(0);
	}
	return queue[0];
}

void PipelineStage::pop() {
	if(queue.empty()) {
		std::cerr << name << " " << __func__ << " Pull from empty pipeline stage\n";
		assert(0);
	}
	queue.erase(queue.begin());
}

std::string PipelineStage::toString() {
	std::stringstream str;
	str << "[pipeline_stage " << name << " ";
	for(auto inst : queue) {
		str << inst->toString() << " ";
	}
	str << "]";
	return str.str();
}
