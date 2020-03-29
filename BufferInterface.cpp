/*******************************************************************
* SSD GCN Project
********************************************************************/

#include "BufferInterface.h"

using namespace std;

BufferInterface::BufferInterface(uint64_t axbuffersize, 
								 uint64_t weightbuffersize, 
								 uint64_t outputbuffersize,
								 DataReader *data_)
{
	axbuffer.size = axbuffersize;
	axbuffer.remain_space = axbuffersize;
	axbuffer.valindex = -1;
	axbuffer.colindex = -1;
	axbuffer.rowindex = -1;

	aux_axbuffer.size = axbuffersize;
	aux_axbuffer.remain_space = axbuffersize;
	aux_axbuffer.valindex = 0;
	aux_axbuffer.colindex = 0;
	aux_axbuffer.rowindex = 0;

	weightbuffer.remain_space = weightbuffersize;
	outputbuffer.remain_space = outputbuffersize;
	mac_start = false;
	isA = false;
	data = data_;

	weightsize.tuple[0] = data_->weight_h;
	weightsize.tuple[1] = data_->weight_w;

	this->axbuffersize = axbuffersize;
	this->weightbuffersize = weightbuffersize;

	present_ax_req = 0;
	present_w_req = 0;

	flag = {false, false, false, false, false, false, false};
	aux_flag = {false, false, false, false, false, false, false};
	present = {0, 0, 0, 0};
	aux_present = {0, 0, 0, 0};
	log = {0, 0, 0, 0};
}

BufferInterface::~BufferInterface() {}

void BufferInterface::FillBuffer(uint64_t address, Type iswhat)
{
	switch (iswhat)
	{
		case A_COL:
			col_for_fold = axbuffer.colindex;
			if (data->adjcolindex.size() - axbuffer.colindex -1 < MAX_READ_INT)
				axbuffer.colindex = data->adjcolindex.size() - 1;
			else
				axbuffer.colindex += MAX_READ_INT;
			axbuffer.remain_space -= MAX_READ_BYTE;
			space_log = axbuffer.remain_space;
			flag.a_col = true;
			break;
		case A_ROW:
			row_for_fold = axbuffer.rowindex;
			if (data->adjrowindex.size() - axbuffer.rowindex -1 < MAX_READ_INT)
				axbuffer.rowindex = data->adjrowindex.size() - 1;
			else
				axbuffer.rowindex += MAX_READ_INT;
			axbuffer.remain_space -= MAX_READ_BYTE;
			space_log = axbuffer.remain_space;
			flag.a_row = true;
			break;
		case X_VAL:
			val_for_fold = axbuffer.valindex;
			if (data->ifvalue.size() - axbuffer.valindex -1 < MAX_READ_INT)
				axbuffer.valindex = data->ifvalue.size() - 1;
			else
				axbuffer.valindex += MAX_READ_INT;
			axbuffer.remain_space -= MAX_READ_BYTE;
			space_log = axbuffer.remain_space;
			flag.x_val = true;
			break;
		case X_COL:
			col_for_fold = axbuffer.colindex;
			if (data->ifcolindex.size() - axbuffer.colindex -1 < MAX_READ_INT)
				axbuffer.colindex = data->ifcolindex.size() - 1;
			else
				axbuffer.colindex += MAX_READ_INT;
			axbuffer.remain_space -= MAX_READ_BYTE;
			space_log = axbuffer.remain_space;
			flag.x_col = true;
			break;
		case X_ROW:
			row_for_fold = axbuffer.rowindex;
			if (data->ifrowindex.size() - axbuffer.rowindex -1 < MAX_READ_INT)
				axbuffer.rowindex = data->ifrowindex.size() - 1;
			else
				axbuffer.rowindex += MAX_READ_INT;
			axbuffer.remain_space -= MAX_READ_BYTE;
			space_log = axbuffer.remain_space;
			flag.x_row = true;
			break;
		case WEIGHT:
			uint64_t row = address / (UNIT_INT_BYTE * weightsize.tuple[1]);
			uint64_t col = (address - row * weightsize.tuple[1] * UNIT_INT_BYTE) / UNIT_INT_BYTE;
			WB_Data insert = {row, col, 1};
			weightbuffer.active.push_back(insert);
			if(weightbuffer.remain_space >= MAX_READ_BYTE) // 꽉 차기 전까지만
				weightbuffer.remain_space -= MAX_READ_BYTE;
			flag.weight = true;
			break;
	}
}

bool BufferInterface::IsFilled(Type iswhat)
{
	bool ret;

	switch (iswhat)
	{
		case A_COL:
			ret = flag.a_col;
			break;
		case A_ROW:
			ret = flag.a_row;
			break;
		case X_VAL:
			ret = flag.x_val;
			break;
		case X_COL:
			ret = flag.x_col;
			break;
		case X_ROW:
			ret = flag.x_row;
			break;
		case WEIGHT:
			ret = flag.weight;
			break;
		case OUTPUT:
			ret = flag.output;
			break;
	}

	return ret;
}

bool BufferInterface::AuxIsFilled(Type iswhat)
{
	bool ret;

	switch (iswhat)
	{
		case A_COL:
			ret = aux_flag.a_col;
			break;
		case A_ROW:
			ret = aux_flag.a_row;
			break;
		case X_VAL:
			ret = aux_flag.x_val;
			break;
		case X_COL:
			ret = aux_flag.x_col;
			break;
		case X_ROW:
			ret = aux_flag.x_row;
			break;
	}

	return ret;
}

void BufferInterface::Reset()
{
	axbuffer.size = axbuffersize;
	axbuffer.remain_space = axbuffersize;
	axbuffer.valindex = 0;
	axbuffer.colindex = 0;
	axbuffer.rowindex = 0;

	aux_axbuffer.size = axbuffersize;
	aux_axbuffer.remain_space = axbuffersize;
	aux_axbuffer.valindex = 0;
	aux_axbuffer.colindex = 0;
	aux_axbuffer.rowindex = 0;

	flag = {false, false, false, false, false, false, false};
	aux_flag = {false, false, false, false, false, false, false};
	present = {0, 0, 0, 0};
	aux_present = {0, 0, 0, 0};
	log = {0, 0, 0, 0};

}

uint64_t BufferInterface::PopData(Type iswhat)
{
	uint64_t ret;

	switch (iswhat)
	{
		case A_COL:
			cout<<"A_COL Popped"<<endl;
			aux_axbuffer.colindex = present.colindex;
			ret = data->adjcolindex[present.colindex];
			axbuffer.remain_space += UNIT_INT_BYTE;
			aux_axbuffer.remain_space -= UNIT_INT_BYTE;
			present.colindex++;
			aux_flag.a_col = true;
			if (present.colindex > axbuffer.colindex)
				flag.a_col = false;
			break;
		case A_ROW:
			cout<<"A_ROW Popped"<<endl;
			if (present.rowindex == 0)
				present.rowindex++;
			aux_axbuffer.rowindex = present.rowindex;
			ret = data->adjrowindex[present.rowindex] - data->adjrowindex[present.rowindex-1];
			axbuffer.remain_space += UNIT_INT_BYTE;
			aux_axbuffer.remain_space -= UNIT_INT_BYTE;
			present.rowindex++;
			aux_flag.a_row = true;
			if (present.rowindex > axbuffer.rowindex)
				flag.a_row = false;
			break;
		case X_COL:
			cout<<"X_COL Popped"<<endl;
			aux_axbuffer.colindex = present.colindex;
			ret = data->ifcolindex[present.colindex];
			axbuffer.remain_space += UNIT_INT_BYTE;
			aux_axbuffer.remain_space -= UNIT_INT_BYTE;
			present.colindex++;
			aux_flag.x_col = true;
			if (present.colindex > axbuffer.colindex)
				flag.x_col = false;
			break;
		case X_ROW:
			cout<<"X_ROW Popped"<<endl;
			if (present.rowindex == 0)
				present.rowindex++;
			aux_axbuffer.rowindex = present.rowindex;
			ret = data->ifrowindex[present.rowindex] - data->ifrowindex[present.rowindex-1];
			axbuffer.remain_space += UNIT_INT_BYTE;
			aux_axbuffer.remain_space -= UNIT_INT_BYTE;
			present.rowindex++;
			aux_flag.x_row = true;
			if (present.rowindex > axbuffer.rowindex)
				flag.x_row = false;
			break;
	}
	
	return ret;
}
float BufferInterface::PopValData()
{
	float ret;

	aux_axbuffer.valindex = present.valindex;
	ret = data->ifvalue[present.valindex];
	axbuffer.remain_space += UNIT_INT_BYTE;
	aux_axbuffer.remain_space -= UNIT_INT_BYTE;
	present.valindex++;
	aux_flag.x_val = true;
	if (present.valindex > axbuffer.valindex)
		flag.x_val = false;

	return ret;
}
uint64_t BufferInterface::ReadMACData(Type iswhat)
{
	uint64_t ret;

	switch (iswhat) 
	{
		case A_COL:
			cout<<"Aux A_COL Popped"<<endl;
			ret = data->adjcolindex[aux_present.colindex];
			aux_present.colindex++;
			aux_axbuffer.remain_space += UNIT_INT_BYTE;
			if (aux_present.colindex > aux_axbuffer.colindex)
				aux_flag.a_col = false;
			break;
		case A_ROW:
			cout<<"Aux A_ROW Popped"<<endl;
			if (aux_present.rowindex == 0)
				aux_present.rowindex++;
			ret = data->adjrowindex[aux_present.rowindex] - data->adjrowindex[aux_present.rowindex-1];
			aux_present.rowindex++;
			shed_row = aux_present.rowindex;
			aux_axbuffer.remain_space += UNIT_INT_BYTE;
			if (aux_present.rowindex > aux_axbuffer.rowindex)
				aux_flag.a_row = false;
			break;			
		case X_COL:
			cout<<"Aux X_COL Popped"<<endl;
			ret = data->ifcolindex[aux_present.colindex];
			aux_present.colindex++;
			aux_axbuffer.remain_space += UNIT_INT_BYTE;
			if (aux_present.colindex > aux_axbuffer.colindex)
				aux_flag.x_col = false;
			break;
		case X_ROW:
			cout<<"Aux X_Row Popped"<<endl;
			if (aux_present.rowindex == 0)
				aux_present.rowindex++;
			ret = data->ifrowindex[aux_present.rowindex] - data->ifrowindex[aux_present.rowindex-1];
			aux_present.rowindex++;
			shed_row = aux_present.rowindex;
			aux_axbuffer.remain_space += UNIT_INT_BYTE;
			if (aux_present.rowindex > aux_axbuffer.rowindex)
				aux_flag.x_row = false;
			break;
	}

	return ret;
}

uint64_t BufferInterface::ShedRow(bool isA) // Give hint to MACController
{
	uint64_t ret;
	if (!isA)
		ret = data->ifrowindex[shed_row] - data->ifrowindex[shed_row-1];
	else
		ret = data->adjrowindex[shed_row] - data->adjrowindex[shed_row-1];
}

float BufferInterface::ReadValMACData()
{
	float ret;

	ret = data->ifvalue[aux_present.valindex];
	aux_present.valindex++;
	if (aux_present.valindex > aux_axbuffer.valindex)
		aux_flag.x_val = false;
	
	return ret;
}
void BufferInterface::ClearMACData(bool rowtoo)
{
	log = aux_present;
	if (rowtoo)
		aux_axbuffer.remain_space = aux_axbuffer.size;
	else
		aux_axbuffer.remain_space = aux_axbuffer.size - UNIT_INT_BYTE;
}

bool BufferInterface::XEnd()
{
	// TODO
	if (!isA && 
		present.rowindex >= data->ifrowindex.size() && 
		present.colindex >= data->ifcolindex.size() &&
		present.valindex >= data->ifvalue.size() &&
		aux_present.rowindex >= data->ifrowindex.size() &&
		aux_present.colindex >= data->ifcolindex.size() &&
		aux_present.valindex >= data->ifvalue.size())
	{
		isA = true;
		return true;
	}
	else
		return false; // Dummy
}

bool BufferInterface::AEnd()
{
	// TODO
	if (isA && 
		present.rowindex >= data->adjrowindex.size() && 
		present.colindex >= data->adjcolindex.size() &&
		aux_present.rowindex >= data->adjrowindex.size() &&
		aux_present.colindex >= data->adjcolindex.size())
		return true;
	else
		return false; // Dummy
}

bool BufferInterface::XRowEnd()
{
	//cout<<present.rowindex<<" .... "<<data->ifrowindex.size()<<endl;
	if (!isA && present.rowindex >= data->ifrowindex.size())
		return true;
	else
		return false;
}

bool BufferInterface::XColEnd()
{
	if (!isA && present.colindex >= data->ifcolindex.size())
		return true;
	else
		return false;
}

bool BufferInterface::XValEnd()
{
	if (!isA && present.valindex >= data->ifvalue.size())
		return true;
	else
		return false;
}

bool BufferInterface::ARowEnd()
{
	if (isA && present.rowindex >= data->adjrowindex.size())
		return true;
	else
	{
		return false;
	}
}

bool BufferInterface::AColEnd()
{
	if (isA && present.colindex >= data->adjcolindex.size())
		return true;
	else
		return false;
}

bool BufferInterface::AuxXRowEnd()
{
	if (!isA && aux_present.rowindex >= data->ifrowindex.size())
		return true;
	else
		return false;
}

bool BufferInterface::AuxXColEnd()
{
	if (!isA && aux_present.colindex >= data->ifcolindex.size())
		return true;
	else
		return false;
}

bool BufferInterface::AuxXValEnd()
{
	if (!isA && aux_present.valindex >= data->ifvalue.size())
		return true;
	else
		return false;
}

bool BufferInterface::AuxARowEnd()
{
	if (isA && aux_present.rowindex >= data->adjrowindex.size())
		return true;
	else
		return false;
}

bool BufferInterface::AuxAColEnd()
{
	if (isA && aux_present.colindex >= data->adjcolindex.size())
		return true;
	else
		return false;
}
/* weight buffer 를 위한 함수들

  canRequest() : request를 보낼 수 있는 상태인지 확인
  IsExist() : weight data를 request하기 전에 해당 data가 버퍼내부에 있는지 확인
  expire() : active내의 데이터 중 특정 address 의 data만 expire시키고 싶을 때 쓸 수 있는 함수

*/
bool BufferInterface::canRequest()
{
	if(weightbuffersize - present_w_req >= MAX_READ_BYTE) // 버퍼가 아직 다 안채워진 경우 -> 가능
	{
		return true;
	}
	else if(!weightbuffer.expire.empty()) // 버퍼 내에 삭제 가능한 데이터가 있는 경우 -> 가능
	{
		weightbuffer.expire.erase(weightbuffer.expire.begin());
		present_w_req -= MAX_READ_BYTE;
		return true;
	}
	else // 모든 데이터가 사용중이므로 request 불가?
	{
		return false;
	}
}

bool BufferInterface::isExist(uint64_t address) // for weight address
{
	// weight address인 경우만 취급 // 일단 두번째 mac에 대해서는 고려하지 않음

	uint64_t row = address / (UNIT_INT_BYTE * weightsize.tuple[1]);
	uint64_t col = (address - row * weightsize.tuple[1] * UNIT_INT_BYTE) / UNIT_INT_BYTE;

	vector<WB_Data>::iterator iter;
	for(iter = weightbuffer.active.begin(); iter != weightbuffer.active.end(); iter++)
	{
		if (iter->row == row && iter->col == col) // hit
		{
			iter->req += 1;
			return true;
		}
	}
	for(iter = weightbuffer.expire.begin(); iter != weightbuffer.expire.end(); iter++) {
		if (iter->row == row && iter->col == col) // hit
		{
			WB_Data t = {iter->row, iter->col, 1};
			weightbuffer.active.push_back(t);
			weightbuffer.expire.erase(iter);
			return true;
		}
	}

	// buffer 안에 없음
	return false;
}

bool BufferInterface::Expire(uint64_t address) // 특정 address만 expire하기 위한 용도
{

	uint64_t row = address / (UNIT_INT_BYTE * weightsize.tuple[1]);
	uint64_t col = (address - row * weightsize.tuple[1] * UNIT_INT_BYTE) / UNIT_INT_BYTE;

	vector<WB_Data>::iterator iter;
	for(iter = weightbuffer.active.begin(); iter != weightbuffer.active.end(); iter++) {
		if (iter->row == row && iter->col == col)
		{
			if(iter->req <= 1) // expire 됨
			{
				WB_Data t = {iter->row, iter->col, 0};
				weightbuffer.expire.push_back(t);
				weightbuffer.active.erase(iter);
			}
			else // 아직 남은 request 존재
			{
				iter->req -= 1;
			}
			return true;
		}
	}
	return false;
}


void BufferInterface::print_status()
{
	cout << "==Buffer==" << endl;

	if (flag.a_col)
	{
		cout << "buffer_A_COL : {";
		for (uint64_t i=col_for_fold; i<=axbuffer.colindex; i++)
		{
			cout << data->adjcolindex[i] << ", ";
		}
		cout << "}" << endl;
	}

	if (flag.a_row)
	{
		cout << "buffer_A_ROW : {";
		for (uint64_t i=row_for_fold; i<=axbuffer.rowindex; i++)
		{
			cout << data->adjrowindex[i] << ", ";
		}
		cout << "}" << endl;
	}

	
	if (flag.x_col)
	{
		cout << "buffer_X_COL : {";
		for (uint64_t i=col_for_fold; i<=axbuffer.colindex; i++)
		{
			cout << data->ifcolindex[i] << ", ";
		}
		cout << "}" << endl;
	}

	if (flag.x_row)
	{
		cout << "buffer_X_ROW : {";
		for (uint64_t i=row_for_fold; i<=axbuffer.rowindex; i++)
		{
			cout << data->ifrowindex[i] << ", ";
		}
		cout << "}" << endl;
	}

	if (flag.x_val)
	{
		cout << "buffer_X_VAL : {";
		for (uint64_t i=val_for_fold; i<=axbuffer.valindex; i++)
		{
			cout << data->ifvalue[i] << ", ";
		}
		cout << "}" << endl;
	}

	cout << "buffer_WEIGHT : {";
	for(WB_Data t: weightbuffer.active) {
		cout << "(" << t.row << "," << t.col << "), ";
	}
	for(WB_Data t: weightbuffer.expire) {
		cout << "(" << t.row << "," << t.col << "), ";
	}
	cout << "}" << endl;

	cout << "buffer's remain_space (AX,W) : " << axbuffer.remain_space << ", " << weightbuffer.remain_space << endl;
	cout << "flag : {" << flag.a_col << ", "
						<< flag.a_row << ", "
						<< flag.x_val << ", "
						<< flag.x_col << ", "
						<< flag.x_row << ", "
						<< flag.weight << ", "
						<< flag.output << "}"
						<< endl;

	cout << endl;
}
