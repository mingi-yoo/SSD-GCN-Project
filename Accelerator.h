/*******************************************************************
* SSD GCN Project
********************************************************************/

#ifndef __ACCELERATOR_H__
#define __ACCELERATOR_H__

#include <iostream>
#include <string>
#include <vector>
#include "DRAMInterface.h"
#include "BufferInterface.h"
#include "Common.h"

using namespace std;

struct AccFlag
{
	bool a_col_req;
	bool a_row_req;
	bool x_val_req;
	bool x_col_req;
	bool x_row_req;
	bool weight_req;
	bool weight_uncompleted;
	bool mac_req;
	bool mac_1;
	bool mac_2;
	bool next_row;
	bool ax_req_ok;
	bool w_req_ok;
};

struct EndFlag
{
	bool a_col_end;
	bool a_row_end;
	bool x_val_end;
	bool x_col_end;
	bool x_row_end;
};

struct Coordinate
{
	uint64_t row;
	uint64_t col;
	float val;
	Tuple weight;
};

struct MACAuxFlag
{
	bool first_get;
	bool second_get;
	bool fold_start;
	bool v_fold_over;
	bool macisready;
	bool maciszero;
	bool wait;
};

struct TempRegister
{
	bool check1;
	bool check2;
	uint64_t w_start_addr;
	uint64_t w_end_addr;
};

struct WaitTuple
{
	uint64_t o_start_addr;
	uint64_t o_end_addr;
	bool start_comp;
	bool end_comp;
};

struct WaitQueue
{
	vector<WaitTuple> o_addr;
};

class Accelerator {
public :
	Accelerator(uint64_t accdimension, DRAMInterface *dram_, BufferInterface *buffer_);
	~Accelerator();
	bool Run();
private:
	uint64_t num_of_pe;
	uint64_t a_col_addr;
	uint64_t a_row_addr;
	uint64_t x_val_addr;
	uint64_t x_col_addr;
	uint64_t x_row_addr;
	uint64_t remain_col_num;
	uint64_t remain_mac_col;
	uint64_t w_fold;
	uint64_t v_fold;
	uint64_t v_fold_last;
	uint64_t present_w_fold;
	uint64_t present_v_fold;
	uint64_t present_address;
	uint64_t present_col;
	uint64_t present_row;
	uint64_t present_mac_row; 
	uint64_t present_mac_col;
	float present_mac_val;
	uint64_t limit_ax_req;
	uint64_t limit_w_req;
	uint64_t mask;
	bool macover;
	bool programover;
	AXBuffer cheat; //리퀘스트가 더 가능한지 확인하는 변수
	Coordinate present; //MACController 처리용
	DRAMInterface *dram;
	BufferInterface *buffer;
	Type need;
	AccFlag flag;
	EndFlag endflag;
	MACAuxFlag macflag;
	TempRegister temp;
	TempRegister aux_temp;
	WaitQueue wait;
	void RequestControllerRun();
	void MACControllerRun();
	void Request(Type iswhat);
	void RequestWeight(uint64_t address);
	void Reset();
};

#endif
