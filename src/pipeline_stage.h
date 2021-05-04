#ifndef SRC_PIPELINE_STAGE_H_
#define SRC_PIPELINE_STAGE_H_

#include <queue>

#include "utils.h"
#include "instruction.h"

class PipelineStage {
	std::string name;
	std::vector<Instruction*> queue;
	uint32_t width;
public:
	PipelineStage(std::string name, uint32_t width);
	virtual ~PipelineStage();

	bool push(Instruction* inst);
	bool isEmpty();
	Instruction* front();
	void pop();

    std::vector<Instruction*>& getAllInstructions() {
		return queue;
}

	std::string toString();
};

#endif /* SRC_PIPELINE_STAGE_H_ */
