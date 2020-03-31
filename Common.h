/*******************************************************************
* SSD GCN Project
********************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__

#include <queue>
#include <vector>
#include <cassert>

using namespace std;

/*
* global variables
* defined in IniParser.cpp
*/
extern uint64_t MAX_READ_BYTE; 
extern uint64_t MAX_READ_INT; 
extern uint64_t UNIT_INT_BYTE; 

extern uint64_t A_COL_START;  
extern uint64_t A_ROW_START;  
extern uint64_t X_VAL_START;  
extern uint64_t X_COL_START;  
extern uint64_t X_ROW_START;  
extern uint64_t WEIGHT_START; 
extern uint64_t OUTPUT_START;

enum Type {A_COL, A_ROW, X_VAL, X_COL, X_ROW, WEIGHT, OUTPUT};

//for weight buffer
struct WB_Data {
	uint64_t address;
	uint64_t req; // 몇 번 리퀘스트 받았는가?
};

//{Row, Col} or {weight_h, weight_w}
struct Tuple {
	uint64_t tuple[2];
};

struct AXBuffer {
	uint64_t size;
	uint64_t remain_space;
	uint64_t valindex;
	uint64_t colindex;
	uint64_t rowindex;
};

struct WeightBuffer {
	uint64_t remain_space;
	vector<WB_Data> active;
	vector<WB_Data> expire;
	vector<WB_Data> request;
};

struct OutputBuffer {
	uint64_t remain_space;
	queue<uint64_t> address; 
};

#endif


//퍼포먼스 카운터