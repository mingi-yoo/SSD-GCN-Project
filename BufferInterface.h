/*******************************************************************
* SSD GCN Project
********************************************************************/

#ifndef __BUFFERINTERFACE_H__
#define __BUFFERINTERFACE_H__

#include <iostream>
#include <string>
#include "DataReader.h"
#include "Common.h"

using namespace std;

struct BufFlag
{
	bool a_col;
	bool a_row;
	bool x_val;
	bool x_col;
	bool x_row;
	bool weight;
	bool output;
};

struct BufIndex
{
	uint64_t valindex;
	uint64_t colindex;
	uint64_t rowindex;
	uint64_t windex;
};

class BufferInterface {
public:
	DataReader *data;
	Tuple weightsize;
	uint64_t axbuffersize;
	uint64_t weightbuffersize;
	bool isready; //Weight가 버퍼에 들어온 경우
	uint64_t present_ax_req; //현재 accelerator가 리퀘스트 한 정도
	uint64_t present_w_req;
	uint64_t shed_row;
	BufferInterface(uint64_t axbuffersize, 
					uint64_t weightbuffersize, 
					uint64_t outputbuffersize,
					DataReader *data_);
	~BufferInterface();
	void FillBuffer(uint64_t address, Type iswhat);
	uint64_t ShedRow(bool isA);
	bool IsFilled(Type iswhat);
	bool AuxIsFilled(Type iswhat);
	bool XEnd(); //MAC2로 넘어갈 준비가 되었는가
	bool AEnd(); //모든 MAC이 끝났는가
	bool XRowEnd();
	bool XColEnd();
	bool XValEnd();
	bool ARowEnd();
	bool AColEnd();
	bool AuxXRowEnd();
	bool AuxXColEnd();
	bool AuxXValEnd();
	bool AuxARowEnd();
	bool AuxAColEnd();
	void Reset(); //다시 채우는 함수
	uint64_t PopData(Type iswhat);
	float PopValData();
	uint64_t ReadMACData(Type iswhat);
	float ReadValMACData();
	void ClearMACData(bool rowtoo);
	//weight용
	bool canRequest();
	bool isExist(uint64_t address); //weight or xw계산한 값 존재?
	void MACEnd();
	bool Expire(uint64_t address);
	//테스트용
	void print_status();

private:
	AXBuffer axbuffer; //처음 DRAM에서 읽어올 때 저장되는 버퍼
	AXBuffer aux_axbuffer; //첫번째 PopData후 데이터가 옮겨지는 버퍼
	uint64_t val_for_fold;
	uint64_t col_for_fold;
	uint64_t row_for_fold;
	uint64_t space_log;
	WeightBuffer weightbuffer;
	OutputBuffer outputbuffer;
	BufFlag flag;
	BufFlag aux_flag;
	BufIndex present; //첫 번째 버퍼의 위치를 가리키는 인덱스
	BufIndex aux_present; //두 번째 버퍼의 위치를 가리키는 인덱스
	BufIndex log;
	bool mac_start;
	bool isA;
	vector<uint64_t> adjcolindex;
	vector<uint64_t> adjrowindex;
	vector<float> ifvalue;
	vector<uint64_t> ifcolindex;
	vector<uint64_t> ifrowindex;
	void FoldEnd();
};

#endif
