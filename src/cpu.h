#ifndef SRC_CPU_H_
#define SRC_CPU_H_

#include "free_list.h"
#include "pipeline_stage.h"
#include "mapping_table.h"
#include "reorder_buffer.h"
#include "reservation_station.h"
#include "utils.h"

class CPU {
	uint32_t numArchRegs;
	uint32_t numPhysicalRegs;
	uint32_t robEntries;
	uint32_t width;
	uint32_t numLSQEntries;
	MappingTable archMappingTable;
	MappingTable mapTable;
	ReorderBuffer rob;
	FreeList freeList;
	std::deque <PhysicalRegister> freePhysRegsPrevCycle;
	PipelineStage fetchStage;
	PipelineStage decodeStage;
	PipelineStage dispatchStage;
	PipelineStage issueStage;
	PipelineStage executeStage;
	PipelineStage completeStage;
	PipelineStage retireStage;
	std::vector<ReservationStation*> reservationStations;

	std::vector<Instruction*> instructionsList;
	uint32_t fetchPtr;

	// Track if we are currently fetching an instruction.
	// Used to detect when we have finished scheduling all instructions.
	bool isFetching;

	// Did any stage in the pipeline progress in last cycle?
	// Used to detect if pipeline is stuck because of bad scheduler design.
	bool hasProgress;

	// Start from cycle 0.
	uint32_t cycle;
public:
	CPU(uint32_t numArchRegs, uint32_t numPhysicalRegs,
			uint32_t robEntries, uint32_t width, uint32_t numLSQEntries);
	virtual ~CPU();

	void addInstruction(char type, uint32_t srcOp1, uint32_t srcOp2, uint32_t dstOp);

	void simulate();
	bool isFinished();

	void tick();

	void fetch();
	void decode();
	void dispatch();
	void issue();
	void execute();
	void complete();
	void retire();

	void generateOutputFile(std::string outputFile);

	std::string toString();
};

#endif /* SRC_CPU_H_ */
