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
	uint64_t row;
	uint64_t col;
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

/*
* --weight buffer 구조 간단 설명--
* 각 active, expire에 weight data가 tuple 의 형태로 저장되어 있음
* (active와 expire는 실제로 구현된 형태라기 보다는
*   걍 각 데이터마다 check bit를 구현하기 위해 임시로 차용한 모델)
*
* active 에는 계산에 사용될 data들이 저장되어 있음
*   (자리가 필요해도 비울 수 없는 데이터들이라고 보면 됨)
* expire 에는 이제 계산이 끝나 아직 버퍼에는 남아있지만
*   비워져도 상관없는 데이터들의 모임이라고 보면 됨
* 만약 내가 필요한 데이터가 expire에 있다면 request 할 필요없이
*   expire에서 active로 옮기기만 하면 됨
*/
struct WeightBuffer {
	uint64_t remain_space;
	vector<WB_Data> active;
	vector<WB_Data> expire;
};

struct OutputBuffer {
	uint64_t remain_space;
	queue<uint64_t> address; 
};

#endif


//퍼포먼스 카운터