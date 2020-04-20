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
	if (buffer_->weightsize.tuple[1] == w_fold * MAX_READ_BYTE && w_fold > 0)
		w_fold--;
	v_fold = MAX_READ_INT/accdimension;
	if (MAX_READ_INT == v_fold * accdimension && v_fold > 1)
		v_fold--;
	v_fold_last = (buffer_->weightsize.tuple[1] - w_fold * MAX_READ_INT)/accdimension;
	if ((buffer_->weightsize.tuple[1] - w_fold * MAX_READ_INT) == v_fold_last * accdimension && v_fold_last > 0)
		v_fold_last--;
	present_w_fold = 0;
	present_mac_row = -1;
	present_row = -1;

	dram = dram_;
	buffer = buffer_;
	buffer->mac1_count = 0;
	buffer->mac2_count = 0;

	remain_col_num = 0;
	remain_mac_col = 0;
	limit_ax_req = buffer_->axbuffersize;
	limit_w_req = buffer_->weightbuffersize;
	mask = (0xffffffffffffffff - MAX_READ_BYTE + 1);

	cheat.valindex = 0;
	cheat.colindex = 0;
	cheat.rowindex = 0;
	flag = {false, false, false, false, true, false, false, false, true, false, true, true, true};
	endflag = {false, false, false, false, false};
	macover = false;
	programover = false;
	parallel = (uint64_t)floor((double)num_of_pe/MAX_READ_INT);
	if (num_of_pe < w_fold)
		parallel++;
	present = new Coordinate[parallel];
	remain_mac_col = new uint64_t[buffer->num_of_xrow];
	for (int i = 0; i < parallel; i++)
	{
		present[i].present_v_fold = 0;
		present[i].first_get = true;
		present[i].second_get = false;
		present[i].fold_start = false;
		present[i].macisready = false;
		present[i].maciszero = false;
		present[i].isend = false;
	}
	iswait = false;
	check_over = parallel;
	check_row = 0;
	row_num = 0;
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

	RequestControllerRun();
	MACControllerRun();

	if (flag.mac_1 && macover && !programover)
	{
		present_w_fold++;
		buffer->Reset();
		Reset();
		flag.x_row_req = true;
		flag.x_val_req = false;
		flag.x_col_req = false;
		flag.weight_req = false;
		endflag.x_row_end = false;
		endflag.x_col_end = false;
		endflag.x_val_end = false;
		buffer->present_ax_req = 0;
		if (present_w_fold > w_fold)
		{
			programover = true;
		}
	}
	if (flag.mac_1 && programover && buffer->mac1_count == 0 && wait.o_addr.empty())
	{
		present_w_fold = 0;
		flag.x_row_req = false;
		flag.a_row_req = true;
		flag.mac_1 = false;
		flag.mac_2 = true;
		buffer->isA = true;
		programover = false;
		cout<<"MAC1 End...."<<endl;
	}
	if (flag.mac_2 && macover && !programover)
	{
		present_w_fold++;
		buffer->Reset();
		Reset();
		flag.a_row_req = true;
		flag.a_col_req = false;
		flag.weight_req = false;
		endflag.a_row_end = false;
		endflag.a_col_end = false;
		if (present_w_fold > w_fold)
		{
			programover = true;			
		}
	}
	if (flag.mac_2 && programover && buffer->mac2_count == 0 && wait.o_addr.empty())
	{
		ret = false;
	}
	
	return ret;
}

void Accelerator::RequestControllerRun()
{
	if (programover)
		return;
	if (flag.mac_1)
	{
		if (buffer->XRowEnd() && buffer->XColEnd() && buffer->XValEnd() && !flag.weight_uncompleted)
			return;
		if ((buffer->IsFilled(X_ROW) && buffer->IsFilled(X_COL) && buffer->IsFilled(X_VAL) && !flag.weight_uncompleted) ||
			(remain_col_num != 0 && buffer->IsFilled(X_COL) && buffer->IsFilled(X_VAL) && !flag.weight_uncompleted))
		{
			flag.x_row_req = false;
			flag.x_col_req = false;
			flag.weight_req = true;
			//cout<<"1"<<endl;
		}
		if (flag.x_row_req && !endflag.x_row_end && ((buffer->present_ax_req + buffer->HowMuchAXFilled()) <= (limit_ax_req - MAX_READ_BYTE)))
		{
			Request(X_ROW);
			buffer->present_ax_req += MAX_READ_BYTE;
			if (!endflag.x_col_end)
			{
				flag.x_row_req = false;
				flag.x_col_req = true;
				//cout<<"2"<<endl;
			}
		}
		else if (flag.x_col_req && !endflag.x_col_end && ((buffer->present_ax_req + buffer->HowMuchAXFilled()) <= (limit_ax_req - 2 * MAX_READ_BYTE)))
		{
			Request(X_COL);
			Request(X_VAL);
			buffer->present_ax_req += 2 * MAX_READ_BYTE;
			if (buffer->IsFilled(X_ROW) && buffer->IsFilled(X_COL) && buffer->IsFilled(X_VAL))
			{
				flag.x_col_req = false;
				flag.weight_req = true;
				//cout<<"3"<<endl;
			}
		}
		else if (flag.weight_req && !buffer->AuxIsFulled())
		{
			if (remain_col_num == 0)
			{
				remain_col_num = buffer->PopData(X_ROW);
				present_row++;
				//cout<<"4"<<endl;
				if (remain_col_num == 0)
				{
					//cout<<"5"<<endl;
					return;
				}
			}
			//cout<<"6"<<endl;
			present_col = buffer->PopData(X_COL);
			buffer->PopValData();
			remain_col_num--;
			uint64_t address = WEIGHT_START + (present_col * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
			temp.w_start_addr = address & mask;
			temp.w_end_addr = (address + 63) & mask;
			buffer->PassWeightAddress(temp.w_start_addr, temp.w_end_addr);
			if (!buffer->isExist(temp.w_start_addr) && !buffer->Requested(temp.w_start_addr))
				temp.check1 = true;
			else
				temp.check1 = false;
			if (temp.w_start_addr != temp.w_end_addr)
			{
				if (!buffer->isExist(temp.w_end_addr) && !buffer->Requested(temp.w_end_addr))
					temp.check2 = true;
				else
					temp.check2 = false;
			}
			else
				temp.check2 = false;
			if (temp.check1 && temp.check2 && buffer->canRequest(true))
			{
				RequestWeight(temp.w_start_addr);
				RequestWeight(temp.w_end_addr);
			}
			else if (temp.check1 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_start_addr);
			}
			else if (temp.check2 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_end_addr);
			}
			else
			{
				if (temp.check1 || temp.check2)
				{
					flag.weight_req = false;
					flag.weight_uncompleted = true;
					//cout<<"7"<<endl;
					return;
				}
			}
			if (remain_col_num == 0 && !buffer->IsFilled(X_ROW) && !endflag.x_row_end)
			{
				flag.weight_req = false;
				flag.x_row_req = true;
				//cout<<"8"<<endl;
			}
			else if (!buffer->IsFilled(X_COL) && 
					 !buffer->IsFilled(X_VAL) &&
					 !endflag.x_col_end &&
					 !endflag.x_row_end)
			{
				flag.weight_req = false;
				flag.x_col_req = true;
				//cout<<"9"<<endl;
			}
			else if ((remain_col_num == 0 && 
					 !buffer->IsFilled(X_ROW) && 
					 endflag.x_row_end &&
					 buffer->IsFilled(X_COL) &&
					 buffer->IsFilled(X_VAL)) ||
					 (buffer->XRowEnd() &&
					 buffer->XColEnd() &&
					 buffer->XValEnd()))
			{
				flag.weight_req = false;
				//cout<<"10"<<endl;
			}
		}
		else if (flag.weight_uncompleted)
		{
			if (temp.check1 && temp.check2 && buffer->canRequest(true))
			{
				RequestWeight(temp.w_start_addr);
				RequestWeight(temp.w_end_addr);
				flag.weight_uncompleted = false;
				temp.check1 = false;
				temp.check2 = false;
			}
			else if (temp.check1 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_start_addr);
				flag.weight_uncompleted = false;
				temp.check1 = false;
			}
			else if (temp.check2 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_end_addr);
				flag.weight_uncompleted = false;
				temp.check2 = false;
			}
			if (!flag.weight_uncompleted)
			{
				if (remain_col_num == 0 && !buffer->IsFilled(X_ROW) && !endflag.x_row_end)
				{
					flag.weight_req = false;
					flag.x_row_req = true;
					//cout<<"11"<<endl;
				}
				else if (!buffer->IsFilled(X_COL) && 
						 !buffer->IsFilled(X_VAL) &&
						 !endflag.x_col_end &&
						 !endflag.x_row_end)
				{
					flag.weight_req = false;
					flag.x_col_req = true;
					//cout<<"12"<<endl;
				}
				else if ((remain_col_num == 0 && 
						 !buffer->IsFilled(X_ROW) && 
						 endflag.x_row_end &&
						 buffer->IsFilled(X_COL) &&
						 buffer->IsFilled(X_VAL)) ||
						 (buffer->XRowEnd() &&
						 buffer->XColEnd() &&
						 buffer->XValEnd()))
				{
					flag.weight_req = false;
					//cout<<"13"<<endl;
				}
			}
		}
		else
		{
			//cout<<"over"<<endl; 
		}
	}
	else
	{
		if (buffer->ARowEnd() && buffer->AColEnd() && !flag.weight_uncompleted)
			return;
		if ((buffer->IsFilled(A_ROW) && buffer->IsFilled(A_COL) && !flag.weight_uncompleted) ||
			(remain_col_num != 0 && buffer->IsFilled(A_COL) && !flag.weight_uncompleted))
		{
			flag.a_row_req = false;
			flag.a_col_req = false;
			flag.weight_req = true;
		}
		if (flag.a_row_req && !endflag.a_row_end && ((buffer->present_ax_req + buffer->HowMuchAXFilled()) <= (limit_ax_req - MAX_READ_BYTE)))
		{
			Request(A_ROW);
			buffer->present_ax_req += MAX_READ_BYTE;
			if (!endflag.a_col_end)
			{
				flag.a_row_req = false;
				flag.a_col_req = true;
			}
		}
		else if (flag.a_col_req && !endflag.a_col_end && ((buffer->present_ax_req + buffer->HowMuchAXFilled()) <= (limit_ax_req - MAX_READ_BYTE)))
		{
			Request(A_COL);
			buffer->present_ax_req += MAX_READ_BYTE;
			if (buffer->IsFilled(A_ROW) && buffer->IsFilled(A_COL))
			{
				flag.a_col_req = false;
				flag.weight_req = true;
			}
		}
		else if (flag.weight_req && !buffer->AuxIsFulled())
		{
			if (remain_col_num == 0)
			{
				remain_col_num = buffer->PopData(A_ROW);
				present_row++;
				if (remain_col_num == 0)
				{
					return;
				}
			}
			present_col = buffer->PopData(A_COL);
			remain_col_num--;
			uint64_t address = OUTPUT_START + (present_col * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
			temp.w_start_addr = address & mask;
			temp.w_end_addr = (address + 63) & mask;
			buffer->PassWeightAddress(temp.w_start_addr, temp.w_end_addr);
			if (!buffer->isExist(temp.w_start_addr) && !buffer->Requested(temp.w_start_addr))
				temp.check1 = true;
			else
				temp.check1 = false;
			if (temp.w_start_addr != temp.w_end_addr)
			{
				if (!buffer->isExist(temp.w_end_addr) && !buffer->Requested(temp.w_end_addr))
					temp.check2 = true;
				else
					temp.check2 = false;
			}
			else
				temp.check2 = false;
			if (temp.check1 && temp.check2 && buffer->canRequest(true))
			{
				RequestWeight(temp.w_start_addr);
				RequestWeight(temp.w_end_addr);
				temp.check1 = false;
				temp.check2 = false;
			}
			else if (temp.check1 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_start_addr);
				temp.check1 = false;
			}
			else if (temp.check2 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_end_addr);
				temp.check2= false;
			}
			else
			{
				if (temp.check1 || temp.check2)
				{
					flag.weight_req = false;
					flag.weight_uncompleted = true;
					return;
				}
			}
			if (remain_col_num == 0 && !buffer->IsFilled(A_ROW) && !endflag.a_row_end)
			{
				flag.weight_req = false;
				flag.a_row_req = true;
			}
			else if (!buffer->IsFilled(A_COL) && 
					 !endflag.a_col_end)
			{
				flag.weight_req = false;
				flag.a_col_req = true;
			}
			else if ((remain_col_num == 0 && 
					 !buffer->IsFilled(A_ROW) && 
					 endflag.a_row_end &&
					 buffer->IsFilled(A_COL)) ||
					 (buffer->ARowEnd() &&
					 buffer->AColEnd()))
			{
				flag.weight_req = false;
			}
		}
		else if (flag.weight_uncompleted)
		{
			if (temp.check1 && temp.check2 && buffer->canRequest(true))
			{
				RequestWeight(temp.w_start_addr);
				RequestWeight(temp.w_end_addr);
				flag.weight_uncompleted = false;
				temp.check1 = false;
				temp.check2 = false;
			}
			else if (temp.check1 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_start_addr);
				flag.weight_uncompleted = false;
				temp.check1 = false;
			}
			else if (temp.check2 && buffer->canRequest(false))
			{
				RequestWeight(temp.w_end_addr);
				flag.weight_uncompleted = false;
				temp.check2 = false;
			}
			if (!flag.weight_uncompleted)
			{
				if (remain_col_num == 0 && !buffer->IsFilled(A_ROW) && !endflag.a_row_end)
				{
					flag.weight_req = false;
					flag.a_row_req = true;
				}
				else if (!buffer->IsFilled(A_COL) && 
						 !endflag.a_col_end)
				{
					flag.weight_req = false;
					flag.a_col_req = true;
				}
				else if ((remain_col_num == 0 && 
						 !buffer->IsFilled(A_ROW) && 
						 endflag.x_row_end &&
						 buffer->IsFilled(A_COL)) ||
						 (buffer->ARowEnd() &&
					 	 buffer->AColEnd()))
				{
					flag.weight_req = false;
				}
			}
		}
	}
	
}

void Accelerator::MACControllerRun()
{
	uint64_t address;

	if (programover && wait.o_addr.empty())
		return;
	for (int i = 0; i < parallel; i++)
	{
		if (!iswait)
		{
			if (flag.mac_1)
			{
				if (!present[i].macisready) //맨 처음 상태 
				{
					if (present[i].first_get) //present라는 변수안에 처리해야할 데이터 집어넣음
					{
						if (buffer->AuxIsFilled(X_ROW) && check_row == 0)
						{
							check_row = buffer->ReadMACData(X_ROW);
							present_mac_row++;
							remain_mac_col[present_mac_row] = check_row;
						}
						else if (!buffer->AuxIsFilled(X_ROW) && check_row == 0)
							continue;
						present[i].remain_mac_col = check_row;
						present[i].row = present_mac_row;
						present[i].first_get = false;
						present[i].second_get = true;
						if (check_row > 0) 
							check_row--;
						if (present[i].remain_mac_col == 0)
						{
							present[i].maciszero = true;
						}
					}
					if (present[i].remain_mac_col != 0 && 
						buffer->AuxIsFilled(X_COL) && 
						buffer->AuxIsFilled(X_VAL) && 
						present[i].second_get &&
						!present[i].maciszero) //만일 zero row가 아니면
					{
						present[i].col = buffer->ReadMACData(X_COL);
						present[i].val = buffer->ReadValMACData();
						present[i].second_get = false;
						present[i].fold_start = true;
					}
					if (buffer->AuxIsFilled(WEIGHT) && present[i].fold_start && !present[i].maciszero)
					{
						present[i].fold_start = false;
						present[i].weight = buffer->ReadWeightTuple();
						if (buffer->isReady(present[i].weight.tuple[0]) && buffer->isReady(present[i].weight.tuple[1])) //준비가 되었는가?
							present[i].macisready = true;
					}
					else if (!present[i].first_get && !present[i].second_get && !present[i].fold_start && !present[i].maciszero) //준비 되었는가?
					{
						if (buffer->isReady(present[i].weight.tuple[0]) && buffer->isReady(present[i].weight.tuple[1]))
							present[i].macisready = true;
					}
				}
				if (present[i].macisready) // 준비됐으면 계산
				{
					cout<<"MAC1 Running... v_fold: "<<dec<<present[i].present_v_fold<<
					", w_fold: "<<dec<<present_w_fold<<
					", Row: "<<dec<<present[i].row<<
					", Col: "<<dec<<present[i].col<<
					", Val: "<<present[i].val<<endl;
					present[i].present_v_fold++;
					if ((present[i].present_v_fold > v_fold && present_w_fold < w_fold) 
						|| (present[i].present_v_fold > v_fold_last && present_w_fold == w_fold))
					{
						present[i].present_v_fold = 0;
						remain_mac_col[present[i].row]--;
						present[i].second_get = true;
						present[i].first_get = true;
						present[i].macisready = false;
						present[i].fold_start = false;
						buffer->Expire(present[i].weight.tuple[0]);
						if (present[i].weight.tuple[0] != present[i].weight.tuple[1])
							buffer->Expire(present[i].weight.tuple[1]);
						present[i].macisready = false;
						if (remain_mac_col[present[i].row] == 0)
						{
							cout<<"Row "<<dec<<present[i].row<<" is Complete."<<endl;
							row_num++;
							address = OUTPUT_START + (present[i].row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
							aux_temp.w_start_addr = address & mask;
							aux_temp.w_end_addr = (address + 63) & mask;
							if (aux_temp.w_start_addr != aux_temp.w_end_addr)
							{
								WaitTuple t;
								t.o_start_addr = aux_temp.w_start_addr;
								t.o_end_addr = aux_temp.w_end_addr;
								t.start_comp = false;
								t.end_comp = false;
								if (!buffer->IsOutputFulled())
								{
									buffer->FillOutputBuffer();
									vector<WaitTuple>::iterator find;
									bool check1 = false, check2 = false;
									for (find = wait.o_addr.begin(); find != wait.o_addr.end(); find++)
									{
										if (find->o_start_addr == aux_temp.w_start_addr ||
											find->o_end_addr == aux_temp.w_start_addr)
											check1 = true;
										if (find->o_start_addr == aux_temp.w_end_addr ||
											find->o_end_addr == aux_temp.w_end_addr)
											check2 = true;
									}
									if (!check1)
									{
										dram->DRAMRequest(aux_temp.w_start_addr, false);
									}
									if (!check2)
									{
										dram->DRAMRequest(aux_temp.w_end_addr, false);
									}
									vector<uint64_t>::iterator iter;
									check1 = false;
									check2 = false;
									for (iter = buffer->req_output.begin(); iter != buffer->req_output.end(); iter++)
									{
										if (*iter == aux_temp.w_start_addr)
											check1 = true;
										if (*iter == aux_temp.w_end_addr)
											check2 = true;
									}
									if (!check1)
										buffer->req_output.push_back(aux_temp.w_start_addr);
									if (!check2)
										buffer->req_output.push_back(aux_temp.w_end_addr);
								}
								else
								{
									iswait = true;
								}
								wait.o_addr.push_back(t);
							}
							else
							{
								buffer->mac1_count++;
								dram->DRAMRequest(address, true);
							}
							if (row_num == buffer->num_of_xrow)
								macover = true;
						}
					}
				}
				else if (present[i].maciszero) // zero인 경우 계산 
				{
					present[i].maciszero = false;
					present[i].first_get = true;
					address = OUTPUT_START + (present[i].row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
					cout<<"MAC1 Running... Row: "<<dec<<present[i].row<<" is zero row...."<<endl;
					row_num++;
					aux_temp.w_start_addr = address & mask;
					aux_temp.w_end_addr = (address + 63) & mask;
					if (aux_temp.w_start_addr != aux_temp.w_end_addr)
					{
						WaitTuple t;
						t.o_start_addr = aux_temp.w_start_addr;
						t.o_end_addr = aux_temp.w_end_addr;
						t.start_comp = false;
						t.end_comp = false;
						if (!buffer->IsOutputFulled())
						{
							buffer->FillOutputBuffer();
							vector<WaitTuple>::iterator find;
							bool check1 = false, check2 = false;
							for (find = wait.o_addr.begin(); find != wait.o_addr.end(); find++)
							{
								if (find->o_start_addr == aux_temp.w_start_addr ||
									find->o_end_addr == aux_temp.w_start_addr)
									check1 = true;
								if (find->o_start_addr == aux_temp.w_end_addr ||
									find->o_end_addr == aux_temp.w_end_addr)
									check2 = true;
							}
							if (!check1)
							{
								dram->DRAMRequest(aux_temp.w_start_addr, false);
							}
							if (!check2)
							{
								dram->DRAMRequest(aux_temp.w_end_addr, false);
							}
							vector<uint64_t>::iterator iter;
							check1 = false;
							check2 = false;
							for (iter = buffer->req_output.begin(); iter != buffer->req_output.end(); iter++)
							{
								if (*iter == aux_temp.w_start_addr)
									check1 = true;
								if (*iter == aux_temp.w_end_addr)
									check2 = true;
							}
							if (!check1)
								buffer->req_output.push_back(aux_temp.w_start_addr);
							if (!check2)
								buffer->req_output.push_back(aux_temp.w_end_addr);
						}
						else
						{
							iswait = true;
						}
						wait.o_addr.push_back(t);
					}
					else
					{
						buffer->mac1_count++;
						dram->DRAMRequest(address, true);
					}
					if (row_num == buffer->num_of_xrow)
						macover = true;
				}
			}
			else
			{
				if (!present[i].macisready)
				{
					if (present[i].first_get)
					{
						if (buffer->AuxIsFilled(A_ROW) && check_row == 0)
						{
							check_row = buffer->ReadMACData(A_ROW);
							present_mac_row++;
							remain_mac_col[present_mac_row] = check_row;
						}
						else if (!buffer->AuxIsFilled(A_ROW) && check_row == 0)
							continue;
						present[i].remain_mac_col = check_row;
						present[i].row = present_mac_row;
						present[i].first_get = false;
						present[i].second_get = true;
						if (present[i].remain_mac_col == 0)
						{
							present[i].maciszero = true;
						}
						if (check_row > 0)
							check_row--;
					}
					if (present[i].remain_mac_col != 0 && 
						buffer->AuxIsFilled(A_COL) &&
						present[i].second_get &&
						!present[i].maciszero)
					{
						present[i].col = buffer->ReadMACData(A_COL);
						present[i].second_get = false;
						present[i].fold_start = true;
					}
					if (buffer->AuxIsFilled(WEIGHT) && present[i].fold_start && !present[i].maciszero)
					{
						present[i].fold_start = false;
						present[i].weight = buffer->ReadWeightTuple();
						if (buffer->isReady(present[i].weight.tuple[0]) && buffer->isReady(present[i].weight.tuple[1]))
							present[i].macisready = true;
					}
					else if (!present[i].first_get && !present[i].second_get && !present[i].fold_start && !present[i].maciszero)
					{
						if (buffer->isReady(present[i].weight.tuple[0]) && buffer->isReady(present[i].weight.tuple[1]))
							present[i].macisready = true;
					}
				}
				if (present[i].macisready)
				{
					cout<<"MAC2 Running... v_fold: "<<dec<<present[i].present_v_fold<<
					", w_fold: "<<dec<<present_w_fold<<
					", Row: "<<dec<<present[i].row<<
					", Col: "<<dec<<present[i].col<<endl;
					present[i].present_v_fold++;
					if ((present[i].present_v_fold > v_fold && present_w_fold < w_fold) 
						|| (present[i].present_v_fold > v_fold_last && present_w_fold == w_fold))
					{
						present[i].present_v_fold = 0;
						remain_mac_col[present[i].row]--;
						present[i].second_get = true;
						present[i].first_get = true;
						present[i].macisready = false;
						present[i].fold_start = false;
						buffer->Expire(present[i].weight.tuple[0]);
						if (present[i].weight.tuple[0] != present[i].weight.tuple[1])
							buffer->Expire(present[i].weight.tuple[1]);
						present[i].macisready = false;
						if (remain_mac_col[present[i].row] == 0)
						{
							cout<<"Row "<<dec<<present[i].row<<" is Complete."<<endl;
							row_num++;
							address = OUTPUT2_START + (present[i].row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
							aux_temp.w_start_addr = address & mask;
							aux_temp.w_end_addr = (address + 63) & mask;
							if (aux_temp.w_start_addr != aux_temp.w_end_addr)
							{
								WaitTuple t;
								t.o_start_addr = aux_temp.w_start_addr;
								t.o_end_addr = aux_temp.w_end_addr;
								t.start_comp = false;
								t.end_comp = false;
								if (!buffer->IsOutputFulled())
								{
									buffer->FillOutputBuffer();
									vector<WaitTuple>::iterator find;
									bool check1 = false, check2 = false;
									for (find = wait.o_addr.begin(); find != wait.o_addr.end(); find++)
									{
										if (find->o_start_addr == aux_temp.w_start_addr ||
											find->o_end_addr == aux_temp.w_start_addr)
											check1 = true;
										if (find->o_start_addr == aux_temp.w_end_addr ||
											find->o_end_addr == aux_temp.w_end_addr)
											check2 = true;
									}
									if (!check1)
									{
										dram->DRAMRequest(aux_temp.w_start_addr, false);
									}
									if (!check2)
									{
										dram->DRAMRequest(aux_temp.w_end_addr, false);
									}
									vector<uint64_t>::iterator iter;
									check1 = false;
									check2 = false;
									for (iter = buffer->req_output.begin(); iter != buffer->req_output.end(); iter++)
									{
										if (*iter == aux_temp.w_start_addr)
											check1 = true;
										if (*iter == aux_temp.w_end_addr)
											check2 = true;
									}
									if (!check1)
										buffer->req_output.push_back(aux_temp.w_start_addr);
									if (!check2)
										buffer->req_output.push_back(aux_temp.w_end_addr);
								}
								else
								{
									iswait = true;
								}
								wait.o_addr.push_back(t);
							}
							else
							{
								buffer->mac2_count++;
								dram->DRAMRequest(address, true);
							}
							if (row_num == buffer->num_of_arow)
								macover = true;
						}
					}
				}
				else if (present[i].maciszero)
				{
					present[i].maciszero = false;
					present[i].first_get = true;
					address = OUTPUT2_START + (present[i].row * buffer->weightsize.tuple[1] + present_w_fold * MAX_READ_INT) * UNIT_INT_BYTE;
					cout<<"MAC2 Running... Row: "<<dec<<present[i].row<<" is zero row...."<<endl;
					row_num++;
					aux_temp.w_start_addr = address & mask;
					aux_temp.w_end_addr = (address + 63) & mask;
					if (aux_temp.w_start_addr != aux_temp.w_end_addr)
					{
						WaitTuple t;
						t.o_start_addr = aux_temp.w_start_addr;
						t.o_end_addr = aux_temp.w_end_addr;
						t.start_comp = false;
						t.end_comp = false;
						if (!buffer->IsOutputFulled())
						{
							buffer->FillOutputBuffer();
							vector<WaitTuple>::iterator find;
							bool check1 = false, check2 = false;
							for (find = wait.o_addr.begin(); find != wait.o_addr.end(); find++)
							{
								if (find->o_start_addr == aux_temp.w_start_addr ||
									find->o_end_addr == aux_temp.w_start_addr)
									check1 = true;
								if (find->o_start_addr == aux_temp.w_end_addr ||
									find->o_end_addr == aux_temp.w_end_addr)
									check2 = true;
							}
							if (!check1)
							{
								dram->DRAMRequest(aux_temp.w_start_addr, false);
							}
							if (!check2)
							{
								dram->DRAMRequest(aux_temp.w_end_addr, false);
							}
							vector<uint64_t>::iterator iter;
							check1 = false;
							check2 = false;
							for (iter = buffer->req_output.begin(); iter != buffer->req_output.end(); iter++)
							{
								if (*iter == aux_temp.w_start_addr)
									check1 = true;
								if (*iter == aux_temp.w_end_addr)
									check2 = true;
							}
							if (!check1)
								buffer->req_output.push_back(aux_temp.w_start_addr);
							if (!check2)
								buffer->req_output.push_back(aux_temp.w_end_addr);
						}
						else
						{
							iswait = true;
						}
						wait.o_addr.push_back(t);
					}
					else
					{
						buffer->mac2_count++;
						dram->DRAMRequest(address, true);
					}
					if (row_num == buffer->num_of_arow)
						macover = true;
				}
			}
		}
		else
		{
			if (!buffer->IsOutputFulled())
			{
				buffer->FillOutputBuffer();
				vector<WaitTuple>::iterator find;
				bool check1 = false, check2 = false;
				for (find = wait.o_addr.begin(); find != wait.o_addr.end(); find++)
				{
					if (find->o_start_addr == aux_temp.w_start_addr ||
						find->o_end_addr == aux_temp.w_start_addr)
						check1 = true;
					if (find->o_start_addr == aux_temp.w_end_addr ||
						find->o_end_addr == aux_temp.w_end_addr)
						check2 = true;
				}
				if (!check1)
				{
					dram->DRAMRequest(aux_temp.w_start_addr, false);
				}
				if (!check2)
				{
					dram->DRAMRequest(aux_temp.w_end_addr, false);
				}
				vector<uint64_t>::iterator iter;
				check1 = false;
				check2 = false;
				for (iter = buffer->req_output.begin(); iter != buffer->req_output.end(); iter++)
				{
					if (*iter == aux_temp.w_start_addr)
						check1 = true;
					if (*iter == aux_temp.w_end_addr)
						check2 = true;
				}
				if (!check1)
					buffer->req_output.push_back(aux_temp.w_start_addr);
				if (!check2)
					buffer->req_output.push_back(aux_temp.w_end_addr);
				iswait = false;
			}
		}
	}
	if (buffer->IsFilled(OUTPUT))
	{
		 vector<WaitTuple>::iterator iter;
		 uint64_t address = buffer->outputbuffer.address.front();
		 for (iter = wait.o_addr.begin(); iter != wait.o_addr.end(); iter++)
		 {
		 	if (iter->o_start_addr == address)
		 		iter->start_comp = true;
		 	if (iter->o_end_addr == address)
		 		iter->end_comp = true;
		 	if (iter->start_comp && iter->end_comp)
		 	{
		 		wait.o_addr.erase(iter);
		 		buffer->RemoveOutputBuffer();
		 		if (iter == wait.o_addr.end())
		 			break;
		 		iter--;
		 	}
		 }
		 buffer->RemoveOutputBuffer();
		 buffer->outputbuffer.address.erase(buffer->outputbuffer.address.begin());
		 dram->DRAMRequest(address, true);
		 if (!buffer->isA)
		 	buffer->mac1_count++;
		 else
		 	buffer->mac2_count++;
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
				endflag.a_col_end = true;
			dram->DRAMRequest(address, false);
			cout<<"Request A_COLUMN. Address: "<<hex<<address<<endl;
			break;
		case A_ROW:
			address = a_row_addr;
			a_row_addr += MAX_READ_BYTE;
			cheat.rowindex += MAX_READ_INT;
			if (cheat.rowindex >= buffer->data->adjrowindex.size())
				endflag.a_row_end = true;;
			dram->DRAMRequest(address, false);
			cout<<"Request A_ROW. Address: "<<hex<<address<<endl;
			break;
		case X_VAL:
			address = x_val_addr;
			x_val_addr += MAX_READ_BYTE;
			cheat.valindex += MAX_READ_INT;
			if (cheat.valindex >= buffer->data->ifvalue.size())
				endflag.x_val_end = true;
			dram->DRAMRequest(address, false);
			cout<<"Request X_VALUE. Address: "<<hex<<address<<endl;
			break;
		case X_COL:
			address = x_col_addr;
			x_col_addr += MAX_READ_BYTE;
			cheat.colindex += MAX_READ_INT;
			if (cheat.colindex >= buffer->data->ifcolindex.size())
				endflag.x_col_end = true;
			dram->DRAMRequest(address, false);
			cout<<"Request X_COLUMN. Address: "<<hex<<address<<endl;
			break;
		case X_ROW:
			address = x_row_addr;
			x_row_addr += MAX_READ_BYTE;
			cheat.rowindex += MAX_READ_INT;
			if (cheat.rowindex >= buffer->data->ifrowindex.size())
				endflag.x_row_end = true;
			dram->DRAMRequest(address, false);
			cout<<"Request X_ROW. Address: "<<hex<<address<<endl;
			break;
	}
}

void Accelerator::RequestWeight(uint64_t address)
{
	dram->DRAMRequest(address, false);
	buffer->Request(address);
	cout<<"Request WEIGHT. Address: "<<hex<<address<<endl;
}

void Accelerator::Reset()
{
	a_col_addr = A_COL_START;
	a_row_addr = A_ROW_START;
	x_val_addr = X_VAL_START;
	x_col_addr = X_COL_START;
	x_row_addr = X_ROW_START;
	present_row = -1;
	present_mac_row = -1;
	cheat.rowindex = 0;
	cheat.colindex = 0;
	cheat.valindex = 0;
	macover = false;
	for (int i = 0; i < parallel; i++)
	{
		present[i].present_v_fold = 0;
		present[i].first_get = true;
		present[i].second_get = false;
		present[i].fold_start = false;
		present[i].macisready = false;
		present[i].maciszero = false;
		present[i].isend = false;
	}
	check_over = parallel;
	check_row = 0;
	row_num = 0;
}
