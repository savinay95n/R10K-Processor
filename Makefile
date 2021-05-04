CXX = g++ --std=c++11 -g
LIBS = -lm

TARGET = project3-r10k

BASE_SOURCES = $(wildcard */*.cpp) $(wildcard *.cpp)
BASE_OBJ = ${BASE_SOURCES:.cpp=.o}
BASE_OBJECTS = ${BASE_OBJ:.c=.o}

all: ${TARGET}

${TARGET}: ${BASE_OBJECTS}
	${CXX} ${FLAGS} -o ${TARGET} ${BASE_OBJECTS} ${LIBS}

clean:
	rm -f ${BASE_OBJECTS} ${TARGET}

.cpp.o:
	${CXX} ${FLAGS} -c $< -o $@
.c.o:
	${CXX} ${FLAGS} -c $< -o $@

test: all
	mkdir -p debugOutputs outputs
	./${TARGET} inputs/ex1.txt outputs/ex1.txt > debugOutputs/ex1.txt 2>&1
	./${TARGET} inputs/ex2.txt outputs/ex2.txt > debugOutputs/ex2.txt 2>&1
	./${TARGET} inputs/ex3.txt outputs/ex3.txt > debugOutputs/ex3.txt 2>&1
	./${TARGET} inputs/ex4.txt outputs/ex4.txt > debugOutputs/ex4.txt 2>&1
	./${TARGET} inputs/sample.txt outputs/sample.txt > debugOutputs/sample.txt 2>&1

.PHONY: all clean test
