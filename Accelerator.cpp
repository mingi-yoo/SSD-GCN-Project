/*******************************************************************
* SSD GCN Project
********************************************************************/

#include "Accelerator.h"

using namespace std;

extern uint64_t cycle;

Accelerator::Accelerator(uint64_t accdimension, DRAMInterface *dram_, BufferInterface *buffer_) 
{
	num_of_pe = accdimension;
	a_col_addr = A_COL_START;
	a_row_addr = A_ROW_START;
	x_val_addr = X_VAL_START;
	x_col_addr = X_COL_START;
	x_row_addr = X_ROW_START;

	w_fold = buffer_->weightsize.tuple[1]/MAX_READ_INT;
	v_fold = MAX_READ_INT/accdimension;
	v_fold_last = (buffer_->weightsize.tuple[1] - w_fold * MAX_READ_INT)/accdimension;
	present_w_fold = 0;
	present_v_fold = 0;
	present_mac_row = -1;

	dram = dram_;
	buffer = buffer_;

	remain_col_num = 0;
	remain_mac_col = 0;
	limit_ax_req = buffer_->axbuffersize;
	limit_w_req = buffer_->weightbuffersize;

	fold_start = false;
	v_fold_over = false;

	cheat.valindex = 0;
	cheat.colindex = 0;
	cheat.rowindex = 0;
	flag = {false, false, false, false, true, false, false, false, true, false, true, true, true};
	endflag = {false, false, false, false, false};
}

Accelerator::~Accelerator() {}

//controller 나눌것
//보내는 것 한 사이클
//컨트롤러 동시접근 -> bank문제

bool Accelerator::Run()
{
	uint64_t address;
	Tuple tuple; 
	uint64_t req_w_col;
	bool ret;

	ret = true;
	/*
	cout << buffer->ARowEnd()<<" "<<buffer->AColEnd()<<endl;
	cout << buffer->AuxARowEnd()<<" "<<buffer->AuxAColEnd()<<endl;
	cout<<"......................"<<endl;
	*/
	if (buffer->XEnd() && v_fold_over)
	{
		present_w_fold++;
		buffer->Reset();
		Reset();
		flag.x_row_req = true;
		flag.x_val_req = false;
		flag.x_col_req = false;
		flag.weight_req = false;
		present_v_fold = 0;
		if (present_w_fold > w_fold)
		{
			present_w_fold = 0;
			flag.x_row_req = false;
			flag.a_row_req = true;
			flag.mac_1 = false;
			flag.mac_2 = true;
		}
	}
	if (buffer->AEnd() && v_fold_over)
	{
		present_w_fold++;
		buffer->Reset();
		Reset();
		flag.a_row_req = true;
		flag.a_col_req = false;
		flag.weight_req = false;
		present_v_fold = 0;
		if (present_w_fold > w_fold)
		{
			ret = false;
		}
	}
	RequestControllerRun();
	MACControllerRun();

	return ret;
}

void Accelerator::RequestControllerRun()
{
	if (buffer->present_ax_req >= limit_ax_req)
	{
		flag.ax_req_ok = false;
	}
	if (!buffer->canRequest())
	{
		flag.w_req_ok = false;
	}
	if (flag.a_row_req && !buffer->IsFilled(A_ROW) && flag.ax_req_ok)
	{
		Request(A_ROW);
		buffer->present_ax_req += MAX_READ_BYTE;
		if (!endflag.a_col_end)
		{
			flag.a_row_req = false;
			flag.a_col_req = true;
		}
	}
	else if (flag.a_col_req && flag.ax_req_ok)
	{
		Request(A_COL);
		buffer->present_ax_req += MAX_READ_BYTE;
		if (buffer->IsFilled(A_ROW) && buffer->IsFilled(A_COL) && flag.ax_req_ok)
		{
			if (remain_col_num == 0)
			{
				remain_col_num = buffer->PopData(A_ROW);
				flag.a_col_req = false;
			}
			else
			{
				flag.weight_req = true;
				flag.a_col_req = false;
			}
		}
	}
	else if (flag.x_row_req && !buffer->IsFilled(X_ROW) && flag.ax_req_ok)
	{
		Request(X_ROW);
		buffer->present_ax_req += MAX_READ_BYTE;
		if (!endflag.x_col_end && !endflag.x_val_end)
		{
			flag.x_row_req = false;
			flag.x_col_req = true;
			flag.x_val_req = true;
		}
	}
	else if (flag.x_col_req && flag.x_val_req && flag.ax_req_ok)
	{
		Request(X_COL);
		Request(X_VAL);
		buffer->present_ax_req += MAX_READ_BYTE;
		buffer->present_ax_req += MAX_READ_BYTE;
		if (buffer->IsFilled(X_ROW) && buffer->IsFilled(X_COL) && buffer->IsFilled(X_VAL))
		{
			if (remain_col_num == 0)
			{
				remain_col_num = buffer->PopData(X_ROW);
				flag.x_col_req = false;
				flag.x_val_req = false;
			}
			else
			{
				flag.weight_req = true;
				flag.x_col_req = false;
				flag.x_val_req = false;
			}
		}
	}
	else if (flag.weight_req && flag.w_req_ok)
	{
		uint64_t address;
		if (flag.mac_1)
		{
			present_col = buffer->PopData(X_COL);
			remain_col_num--;
			buffer->PopValData();
			address = WEIGHT_START + (present_col * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
			if (!buffer->isExist(address))
			{
				dram->DRAMRequest(address, false);
				cout<<"Weight Request... Address: "<<hex<<address<<endl;
			}
			if (remain_col_num == 0)
			{
				if (buffer->IsFilled(X_ROW))
				{
					remain_col_num = buffer->PopData(X_ROW);
					if (remain_col_num == 0 && buffer->IsFilled(X_ROW))
					{
						flag.weight_req = false;	
					}
					else if (remain_col_num == 0 && !buffer->IsFilled(X_ROW) && !endflag.x_row_end)
					{
						flag.x_row_req = true;
						flag.weight_req = false;
					}
				}
				else if (!endflag.x_row_end)
				{
					flag.x_row_req = true;
					flag.weight_req = false;
				}
				else
				{
					flag.weight_req = false;
				}
			}
			else if (!buffer->IsFilled(X_COL) && !buffer->IsFilled(X_VAL) && !endflag.x_col_end && !endflag.x_val_end)
			{
				flag.x_col_req = true;
				flag.x_val_req = true;
				flag.weight_req = false;
			}
			else if (endflag.x_col_end && endflag.x_val_end && remain_col_num == 0)
			{
				flag.weight_req = false;
				if (!buffer->XRowEnd())
					buffer->PopData(X_ROW);
			}
		}
		else
		{
			present_col = buffer->PopData(A_COL);
			remain_col_num--;
			address = OUTPUT_START + (present_col * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
			if (!buffer->isExist(address))
			{
				dram->DRAMRequest(address, false);
				cout<<"Weight Request... Address: "<<hex<<address<<endl;
			}
			if (remain_col_num == 0)
			{
				if (buffer->IsFilled(A_ROW))
				{
					remain_col_num = buffer->PopData(A_ROW);
					if (remain_col_num == 0 && buffer->IsFilled(A_ROW))
						flag.weight_req = false;
					else if (remain_col_num == 0 && !buffer->IsFilled(A_ROW) && !endflag.a_row_end)
					{
						flag.a_row_req = true;
						flag.weight_req = false;
					}
				}
				else if (!endflag.a_row_end)
				{
					flag.a_row_req = true;
					flag.weight_req = false;
				}
				else
				{
					flag.weight_req = false;
				}
			}
			else if (!buffer->IsFilled(A_COL) && !endflag.a_col_end)
			{
				flag.a_col_req = true;
				flag.weight_req = false;
			}
			else if (endflag.a_col_end && remain_col_num == 0) 
			{
				flag.weight_req = false;
				if (!buffer->ARowEnd())
					buffer->PopData(A_ROW);
			}
		}
		buffer->present_w_req += MAX_READ_BYTE;	
	}
	else
	{
		if (flag.mac_1)
		{
			if (buffer->IsFilled(X_ROW) && buffer->IsFilled(X_COL) && buffer->IsFilled(X_VAL))
			{
				if (remain_col_num == 0)
				{
					remain_col_num = buffer->PopData(X_ROW);
					cout<<"..........."<<endl;
					if (remain_col_num != 0)
					{
						flag.weight_req = true;
						flag.x_col_req = false;
						flag.x_val_req = false;
					}
				}
				else
				{
					flag.weight_req = true;
					flag.x_col_req = false;
					flag.x_val_req = false;
				}
					
			}
		}
		else
		{
			if (buffer->IsFilled(A_ROW) && buffer->IsFilled(A_COL) && flag.ax_req_ok)
			{	
				if (remain_col_num == 0)
				{
					remain_col_num = buffer->PopData(A_ROW);
					if (remain_col_num != 0)
					{
						flag.weight_req = true;
						flag.a_col_req = false;
					}
				}
				else
				{
					flag.weight_req = true;
					flag.a_col_req = false;
				}
			}
		}
	}
}

void Accelerator::MACControllerRun()
{
	uint64_t address;

	if (buffer->isready)
	{
		if (flag.mac_1)
		{;
			if (present_v_fold == 0 && remain_mac_col == 0 && buffer->AuxIsFilled(X_ROW))
			{
				remain_mac_col = buffer->ReadMACData(X_ROW);
				present_mac_row++;
				if (remain_mac_col == 0)
				{
					v_fold_over = true;
					present_mac_val = 0;
					present_mac_col = 0;
					address = OUTPUT_START + (present_mac_row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
					cout<<"MAC1 Running... Row: "<<dec<<present_mac_row<<" is zero row..."<<endl;
					dram->DRAMRequest(address, true);
					cout<<"Output Request... Address: "<<hex<<address<<endl;
					if (buffer->AuxXRowEnd() || buffer->ShedRow(false) != 0)
						buffer->isready = false;
					return;
				}
			}
			if (present_v_fold == 0)
			{
				v_fold_over = false;
				present_mac_val = buffer->ReadValMACData();
				present_mac_col = buffer->ReadMACData(X_COL);
			}
			cout<<"MAC1 Running... v_fold: "<< present_v_fold<<
			" w_fold: "<<present_w_fold<<
			", Row: "<<present_mac_row<<
			", Column: "<<present_mac_col<<
			", Value: "<<present_mac_val<<endl;
			present_v_fold++;
			if ((present_v_fold > v_fold && present_w_fold < w_fold) 
				|| (present_v_fold > v_fold_last && present_w_fold == w_fold))
			{
				present_v_fold = 0;
				remain_mac_col--;
				v_fold_over = true;
				address = WEIGHT_START + (present_mac_col * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
				buffer->Expire(address);
				buffer->isready = false;
				if (remain_mac_col == 0)
				{
					cout<<"Row "<<dec<<present_mac_row<<" is Complete."<<endl;
					address = OUTPUT_START + (present_mac_row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
					dram->DRAMRequest(address, true);
					cout<<"Output Request... Address: "<<hex<<address<<endl;
					if (buffer->AuxXValEnd() && buffer->AuxXColEnd() && !buffer->AuxXRowEnd())
					{
						buffer->isready = true;
					}				
				}
			}
		}
		else
		{
			if (present_v_fold == 0 && remain_mac_col == 0 && buffer->AuxIsFilled(A_ROW))
			{
				remain_mac_col = buffer->ReadMACData(A_ROW);
				present_mac_row++;
				if (remain_mac_col == 0)
				{
					v_fold_over = true;
					present_mac_val = 0;
					present_mac_col = 0;
					address = OUTPUT_START + (present_mac_row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
					dram->DRAMRequest(address, true);
					cout<<"MAC2 is running ... row "<<dec<<present_mac_col<<" is zero"<<endl;
					if (buffer->AuxARowEnd() || buffer->ShedRow(true) != 0)
						buffer->isready = false;
					return;
				}
				else
				{
					v_fold_over = false;
					present_mac_col = buffer->ReadMACData(A_COL);
				}
			}
			if (present_v_fold == 0)
			{
				v_fold_over = false;
				present_mac_col = buffer->ReadMACData(A_COL);
			}
			cout<<"MAC2 Running... v_fold: "<< present_v_fold<<
			" w_fold: "<<present_w_fold<<
			", Row: "<<present_mac_row<<
			", Column: "<<present_mac_col<<endl;
			present_v_fold++;
			if ((present_v_fold > v_fold && present_w_fold < w_fold) 
				|| (present_v_fold > v_fold_last && present_w_fold == w_fold))
			{
				present_v_fold = 0;
				remain_mac_col--;
				v_fold_over = true;
				address = OUTPUT_START + (present_mac_col * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
				buffer->Expire(address);
				buffer->isready = false;
				if (remain_mac_col == 0)
				{
					cout<<"Row "<<dec<<present_mac_row<<" is Complete."<<endl;
					address = OUTPUT_START + (present_mac_row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
					dram->DRAMRequest(address, true);	
					if (buffer->AuxAColEnd() && !buffer->AuxARowEnd())
					{
						buffer->isready = true;
					}	
				}
			}
		}
	}
}

void Accelerator::Request(Type iswhat)
{
	uint64_t address;

	switch (iswhat)
	{
		case A_COL:
			address = a_col_addr;
			a_col_addr += MAX_READ_BYTE;
			cheat.colindex += MAX_READ_INT;
			if (cheat.colindex >= buffer->data->adjcolindex.size())
			{
				flag.a_col_req = false;
				endflag.a_col_end = true;
			}
			dram->DRAMRequest(address, false);
			cout<<"Request A_COLUMN. Address: "<<hex<<address<<endl;
			break;
		case A_ROW:
			address = a_row_addr;
			a_row_addr += MAX_READ_BYTE;
			cheat.rowindex += MAX_READ_INT;
			if (cheat.rowindex >= buffer->data->adjrowindex.size())
			{
				flag.a_row_req = false;
				endflag.a_row_end = true;
			}
			dram->DRAMRequest(address, false);
			cout<<"Request A_ROW. Address: "<<hex<<address<<endl;
			break;
		case X_VAL:
			address = x_val_addr;
			x_val_addr += MAX_READ_BYTE;
			cheat.valindex += MAX_READ_INT;
			if (cheat.valindex >= buffer->data->ifvalue.size())
			{
				flag.x_val_req = false;
				endflag.x_val_end = true;
			}
			dram->DRAMRequest(address, false);
			cout<<"Request X_VALUE. Address: "<<hex<<address<<endl;
			break;
		case X_COL:
			address = x_col_addr;
			x_col_addr += MAX_READ_BYTE;
			cheat.colindex += MAX_READ_INT;
			if (cheat.colindex >= buffer->data->ifcolindex.size())
			{
				flag.x_col_req = false;
				endflag.x_col_end = true;
			}
			dram->DRAMRequest(address, false);
			cout<<"Request X_COLUMN. Address: "<<hex<<address<<endl;
			break;
		case X_ROW:
			address = x_row_addr;
			x_row_addr += MAX_READ_BYTE;
			cheat.rowindex += MAX_READ_INT;
			if (cheat.rowindex >= buffer->data->ifrowindex.size())
			{
				flag.x_row_req = false;
				endflag.x_row_end = true;
			}
			dram->DRAMRequest(address, false);
			cout<<"Request X_ROW. Address: "<<hex<<address<<endl;
			break;
	}
}

void Accelerator::Reset()
{
	a_col_addr = A_COL_START;
	a_row_addr = A_ROW_START;
	x_val_addr = X_VAL_START;
	x_col_addr = X_COL_START;
	x_row_addr = X_ROW_START;
}
