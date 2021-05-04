#include "cpu.h"
#include <fstream>
#include <vector>

CPU::CPU(uint32_t numArchRegs, uint32_t numPhysicalRegs,
		uint32_t robEntries, uint32_t width, uint32_t numLSQEntries) :
	numArchRegs(numArchRegs), numPhysicalRegs(numPhysicalRegs),
	robEntries(robEntries), width(width), numLSQEntries(numLSQEntries),
	archMappingTable("archMapTable", numArchRegs, numPhysicalRegs),
	mapTable("Mapping Table", numArchRegs, numPhysicalRegs),
	rob(robEntries), freeList(numArchRegs, numPhysicalRegs),
	fetchStage("fetch", width),
	decodeStage("decode", width),
	dispatchStage("dispatch", width),
	issueStage("issue", width),
	executeStage("execute", width),
	completeStage("complete", width),
	retireStage("retire", width),
	fetchPtr(0), isFetching(true),
	hasProgress(false), cycle(0)
{
	reservationStations.push_back(new ReservationStation("ALU", RSType_ALU, 1));
	reservationStations.push_back(new ReservationStation("ALU", RSType_ALU, 1));
	reservationStations.push_back(new ReservationStation("LOAD", RSType_LOAD, 2));
	reservationStations.push_back(new ReservationStation("STORE", RSType_STORE, 2));
}

CPU::~CPU() {
}

void CPU::addInstruction(char type, uint32_t srcOp1,
		uint32_t srcOp2, uint32_t dstOp) {
	Instruction* inst = new Instruction(instructionsList.size(), type, srcOp1, srcOp2, dstOp);
	instructionsList.push_back(inst);
}

bool CPU::isFinished() {
	bool finished = true;
	for(int i = 0; finished && i < instructionsList.size(); i++)
		finished &= instructionsList[i]->hasRetired();
	return finished;
}

void CPU::simulate() {
	hasProgress = true;
	while(!isFinished() && hasProgress) {
		hasProgress = false;
		tick();
		// Move on to the next cycle.
		cycle++;
	}
}

void CPU::tick() {
	// add physical registers that are freed in the previous cycle to freeList
	for(PhysicalRegister& pReg : freePhysRegsPrevCycle)
		freeList.addRegister(pReg);
	freePhysRegsPrevCycle.clear();
	// We process pipeline stages in opposite order to (try to) clear up
	// the subsequent stage before sending instruction forward from any
	// given stage.
	retire();
	complete();
	execute();
	issue();
	dispatch();
	decode();
	fetch();
	std::cerr << rob.toString() << "\n";
	std::cerr << "Reservation Stations : [\n";
	for(int i = 0; i < reservationStations.size(); i++) {
		std::cerr << "\t" << reservationStations[i]->toString() << "\n";
	}
	std::cerr << "]\n";
	std::cerr << mapTable.toString() << "\n";
	std::cerr << archMappingTable.toString() << "\n";
	std::cerr << freeList.toString() << "\n\n";
}

void CPU::fetch() {
	for(int i = 0; i < width && isFetching; i++) {
		// hasProgress should set if CPU has progress in any stage at each cycle
		if(fetchPtr >= instructionsList.size()) {
			isFetching = false;
			break;
		}
		bool res = decodeStage.push(instructionsList[fetchPtr]);
		// res is always true in this project
		if(res) {
			hasProgress = true;
			std::cerr << "Cycle #" << cycle << ": fetch   \t" << instructionsList[fetchPtr]->toString() << "\n";
			instructionsList[fetchPtr]->setFetchCycle(cycle);
			fetchPtr++;
		}
		else {
			break;
		}
		if(fetchPtr >= instructionsList.size()) {
			isFetching = false;
		}
	}
}

void CPU::decode() {
	for(int i = 0; i < width; i++) {
		if(decodeStage.isEmpty())
			break;
		Instruction* inst = decodeStage.front();
		bool res = dispatchStage.push(inst);
		// Should not remove the instruction if cannot be added to the next stage
		// The reason could be because of stalls in next stages
		// res is always true in this project
		if(res) {
			inst->setDecodeCycle(cycle);
			std::cerr << "Cycle #" << cycle << ": decode  \t" << inst->toString() << "\n";
			hasProgress = true;
			decodeStage.pop();
		}
		else
			break;
	}
}

void CPU::dispatch() {
	for(int i = 0; i < width; i++) {
		if(dispatchStage.isEmpty())
			break;
		// No free RoB Entry -> stall
		if(!rob.hasFreeEntry()) {
			break;
		}

		Instruction* inst = dispatchStage.front();

		// Check if corresponding RS is free
		RSType requiredType = inst->getReservationStation();
		uint32_t freeRSIndex = -1;
		for(int j = 0; j < reservationStations.size(); j++) {
			if(reservationStations[j]->getType() == requiredType &&
					reservationStations[j]->isBusy() == false) {
				freeRSIndex = j;
				break;
			}
		}
		// required RS is busy -> stall
		if(freeRSIndex == -1) {
			break;
		}

		// Renaming

		// No free register in the free list -> stall
		if(inst->getDstOp() != -1 && freeList.hasRegister() == false) {
			break;
		}
		std::string beforeRenaming = inst->toString();
		inst->setSrcPhysicalReg1(mapTable.getMapping(inst->getSrcOp1()));
		if(inst->getSrcOp2() != -1)
			inst->setSrcPhysicalReg2(mapTable.getMapping(inst->getSrcOp2()));
		PhysicalRegister T;		// By default, T = -1
		PhysicalRegister Told;	// By default, Told = -1
		if(inst->getDstOp() != -1) {
			T = freeList.popRegister();
			T.setReady(false);
			inst->setDstPhysicalReg(T);
			Told = mapTable.getMapping(inst->getDstOp());
			mapTable.setMapping(inst->getDstOp(), T);
		}
		else {
			inst->setDstPhysicalReg(T);
		}
		inst->setRenamed(true);

		// Add instruction to ROB
		rob.addInstruction(inst, T, Told);

		// Add instruction to Reservation Station
		reservationStations[freeRSIndex]->allocate(inst);
		// Instruction need the reservation as well to free it at execute stage
		inst->setAllocatedRs(reservationStations[freeRSIndex]);

		inst->setDispatchCycle(cycle);
		std::cerr << "Cycle #" << cycle << ": dispatch\t" << beforeRenaming << " ->\t" << inst->toString() << "\n";
		hasProgress = true;
		dispatchStage.pop();
	}
}

void CPU::issue() {
	// TODO Your code here
	// Going over all reservation stations and execute the ones that are ready
	// setIssueCycle for the instruction that is issued
	// Uncomment and use the following two lines at the location which you issue an instruction
	// std::cerr << "Cycle #" << cycle << ": issue   \t" << [inst]->toString() << "\n";	// [inst] may need to be changed
	// hasProgress = true;

	/*
	Steps:

	1. Go over the "width" to issue "width" number of instructions
	2. Go over all reservation station entries
	3. Before pushing it to execute stage, check isReadyToExecute() and
       make sure the insruction has not been issued (i.e, hasIssued() returns false)
	4. If isReadyToExecute(), push the instruction to execute stage queue
	5. setIssueCycle for the instruction


	*/
    int width_counter = 0;
	for (int i = 0; i < width; i++){
        for(int j = 0; j < reservationStations.size(); j++){

            Instruction* inst = reservationStations[j]->getInst();
            if(reservationStations[j]->isReadyToExecute() && !(inst->hasIssued())){
				// counts the number of times an instruction is issued.
				// Cannot exceed "width" number of issues
                width_counter += 1;
                if(width_counter > width){
                    return;
                }
				// push to execute stage
                bool res = executeStage.push(inst);

				// res is always true
                if(res){
                    inst->setIssueCycle(cycle);
                    std::cerr << "Cycle #" << cycle << ": issue   \t" << inst->toString() << "\n";	// [inst] may need to be changed
                    hasProgress = true;
                }
            }

        }

	}

}

void CPU::execute() {
	// TODO Your code here
	// setExecuteCycle for the instruction that is started its execution
	// setExecTime of the instruction according to the execution time of RS
	// Free the reservation stations that are executed
	// add executing instructions to completeStage
	// Uncomment and use the following two lines at the location which you execute an instruction
	// std::cerr << "Cycle #" << cycle << ": execute \t" << [inst]->toString() << "\n"; // [inst] may need to be changed
	// hasProgress = true;

	/*

	Steps:
	1. Go over the "width" to execute "width" number of instructions
	2. Check if executeState isEmpty()
    3. Extract the first instruction from the queue
    4. Push it to complete stage queue (Do not check its completion here)
    5. setExecuteCycle for instruction that started its execution
    6. setExecTime from Reservation station execTime
	*/
	for (int i = 0; i < width; i++){
        if(executeStage.isEmpty())
            break;

        Instruction* inst = executeStage.front();
		// push to complete stage
        bool res = completeStage.push(inst);

        /*
        std::cout << "<<<<<<<<<<<<<<<<<< HERE >>>>>>>>>>>>>>>>>>>> \n";
        std::vector<Instruction*>& myqueue = completeStage.getAllInstructions();
        std::cout << myqueue.size() << std::endl;
        std::cout <<ge that finished their execution time and current cycle "<<<<<<<<<<<<<<<<<< HERE >>>>>>>>>>>>>>>>>>>> \n";
        */

        if(res){
			// temp cariable to hold reservation station index
			uint32_t RSIndex = -1;
            RSType myType = inst->getReservationStation();
            inst->setExecuteCycle(cycle);
			
			// find the reservation station index of this instruction based on the type
            for(int j = 0; j < reservationStations.size(); j++) {
                if(reservationStations[j]->getType() == myType){
                    RSIndex = j;
                    break;
                }
            }
            inst->setExecTime(reservationStations[RSIndex]->getExecTime());
            inst->getAllocatedRs()->free();
            std::cerr << "Cycle #" << cycle << ": execute \t" << inst->toString() << "\n"; // [inst] may need to be changed
            hasProgress = true;
			// pop from execute stage
            executeStage.pop();
        }
    }
}

void CPU::complete() {
	// TODO Your code here
	// setCompleteCycle for the instruction that is completed
	// add instructions to completeStage that finished their execution time and current cycle
	// set ready bit of the destination register
	// broadcast the result to mapping table and reservation stations
	// Uncomment and use the following two lines at the location which you execute an instruction
	// std::cerr << "Cycle #" << cycle << ": complete\t" << [inst]->toString() << "\n"; // [inst] may need to be changed
	// hasProgress = true;

    for (int i = 0; i < width; i++){
		
    if(completeStage.isEmpty()){
        break;

    }

	// get the complete instruction queue
    std::vector<Instruction*>& completeStageQueue = completeStage.getAllInstructions();

    // create a counter array
    for(int j = 0; j < completeStageQueue.size(); j++){
        Instruction* inst = completeStageQueue[j];
		// get the executionTime and executionCycle 
		uint32_t executionCycle = inst->getExecuteCycle();
        uint32_t executionTime = inst->getExecTime();
        

		// if current cycle + execution cycle 
        if(cycle >= executionCycle + executionTime){
            // Begin Complete

            PhysicalRegister& destinationRegister = inst->getDstPhysicalReg();
            uint32_t destinationRegNum = destinationRegister.getRegNum();
			
			// Broadcast
			for (int z = 0; z < reservationStations.size(); z++){
            //if(inst->getDstOp() != -1){
                reservationStations[z]->broadcastRegReady(destinationRegNum);
            //}
			}
		
            // Update Mapping Table
			// Check for Store instruction
            if(inst->getDstOp() != -1) {
                mapTable.setReadyBit(destinationRegNum);

            }

        

        // set complete cycle
        inst->setCompleteCycle(cycle);
		
		// Erase completed instrcution from complete queue
        completeStageQueue.erase(completeStageQueue.begin() + j);
		
        std::cerr << "Cycle #" << cycle << ": complete\t" << inst->toString() << "\n"; // [inst] may need to be changed
        hasProgress = true;


        }
        else{
            hasProgress = true;
			}



		}

	}

}

void CPU::retire() {
	// TODO Your code here
	// retire instructions from head of rob
	// setRetireCycle for the instruction that is retired
	// update freePhysRegsPrevCycle array that add the physical registers in current cycle to the free list in the beginning of next cycle
	// update architectural mapping table
	// Uncomment and use the following two lines at the location which you execute an instruction
	// std::cerr << "Cycle #" << cycle << ": retire  \t" << [inst]->toString() << "\n"; // [inst] may need to be changed
	// hasProgress = true;

	/*

    Steps:
    1. Loop over "width" to retire "width" number of instructions
    2. Get head entry
    3. Get the head instruction from this ROB entry
    4. Check if this instruction hasCompleted() ??
    5. If completed, retire it
    6. Push_back the renamed destination physical register of this instruction to deque "freePhysRegsPrevCycle"
    7. Get the new destination register - getT() - PR#
    8. Get the old destination register - getTold() - PR# (We need AR#)
    9. Get the regNums of the two registers:
        a. Get the instruction's type
        b. Get dstOp
        b. If it is store, get srcOp2
    10. Update arch map table by archMapTable[Told_archRegNum] = T
    11. set getRetired() to true
	*/

	for(int i=0; i < width; i++){
        // Initial sanity check
        if(rob.getHead() == nullptr){
            break;
        }

        // This is the actual condition:
        // We enter reture only if the head rob entry is completed
        if(!(rob.getHead()->getInst()->hasCompleted())){
            break;
        }

        ROBEntry* robHead = rob.getHead();
        Instruction* inst = robHead->getInst();

        // retire head
        rob.retireHeadInstruction();

        // get T and Told Physical Registers
        PhysicalRegister& destinationReg = inst->getDstPhysicalReg();
		
        
	// Update Arch Map Table
        // check if destination register is not equal to -1 (Store)
	
        if(inst->getDstOp() != -1){
            uint32_t destinationArchNum = inst->getDstOp();

            archMappingTable.setMapping(destinationArchNum, destinationReg);

        }
		
		PhysicalRegister destinationTold = robHead->getTold();
        // Push the freed register to this temp buffer
        if(inst->getDstOp() != -1){
            freePhysRegsPrevCycle.push_back(destinationTold);
        }

        
		// retire cycle
        inst->setRetireCycle(cycle);

        std::cerr << "Cycle #" << cycle << ": retire  \t" << inst->toString() << "\n"; // [inst] may need to be changed
	    hasProgress = true;



    }

}






void CPU::generateOutputFile(std::string outputFile) {
	std::ofstream out(outputFile);
	if(!out.is_open()) {
		std::cerr << "Cannot open output file to write!\n";
		exit(-1);
	}
	for(auto inst : instructionsList) {
		out << inst->getFetchCycle() << " " <<
				inst->getDecodeCycle() << " " <<
				inst->getDispatchCycle() << " " <<
				inst->getIssueCycle() << " " <<
				inst->getExecuteCycle() << " " <<
				inst->getCompleteCycle() << " " <<
				inst->getRetireCycle() << "\n";
	}
	out.close();
}

std::string CPU::toString() {
	std::stringstream str;
	str << "[OoO CPU cycle=" << cycle << "\n";
	return str.str();
}
