/*******************************************************************
* SSD GCN Project
********************************************************************/

#include "DRAMInterface.h"

using namespace std;

DRAMInterface::DRAMInterface(const string& configfile, 
							 const string& systemfile, 
							 const string& logfile, 
							 const string& outputfile, 
							 unsigned memorysize, 
							 float clk_period_in_ns,
							 BufferInterface *buffer_)
{
	dram = DRAMSim::getMemorySystemInstance(configfile, systemfile, logfile, outputfile, memorysize);
	dram->setCPUClockSpeed(static_cast<uint64_t>(1.0/(clk_period_in_ns * 1e-9)));
	read_cb = new DRAMSim::Callback<DRAMInterface, void, unsigned, uint64_t, uint64_t>(this, &DRAMInterface::ReadCompleteCallback);
  	write_cb = new DRAMSim::Callback<DRAMInterface, void, unsigned, uint64_t, uint64_t>(this, &DRAMInterface::WriteCompleteCallback);
  	dram->RegisterCallbacks(read_cb, write_cb, NULL);;
	buffer = buffer_;
}

DRAMInterface::~DRAMInterface()
{
	delete dram;
	delete read_cb;
	delete write_cb;
}

void DRAMInterface::UpdateCycle()
{
	dram->update();
}

void DRAMInterface::DRAMRequest(uint64_t address, bool is_write)
{
	dram->addTransaction(is_write, address);
}

void DRAMInterface::ReadCompleteCallback(unsigned id, uint64_t address, uint64_t clock_cycle) 
{ 
	Type belong = WhereisItBelong(address);
	string print;
	if (belong == OUTPUT)
		belong = WEIGHT;
	buffer->FillBuffer(address, belong);
	if (belong == WEIGHT)
	{
		//buffer->present_w_req -= MAX_READ_BYTE; 필요없음
		buffer->isready = true;
	}
	else
	{
		buffer->present_ax_req -= MAX_READ_BYTE;
	}
	if (belong == WEIGHT)
		print = "WEIGHT";
	else if (belong == X_VAL)
		print = "X_VALUE";
	else if (belong == X_COL)
		print = "X_COLUMN";
	else if (belong == X_ROW)
		print = "X_ROW";
	else if (belong == A_COL)
		print = "A_COLUMN";
	else if (belong == A_ROW)
		print = "A_ROW";

	cout<<"Cycle: "<<clock_cycle<<". Data Read Complete. Type: "<<print<<" Address: "<<address<<endl;
}

void DRAMInterface::WriteCompleteCallback(unsigned id, uint64_t address, uint64_t clock_cycle) 
{
	cout<<"Cycle: "<<clock_cycle<<". Output Write Complete. Address: "<<address<<endl;
}

Type DRAMInterface::WhereisItBelong(uint64_t address) {
  if (address < X_VAL_START)
    return WEIGHT;
  else if (address < X_COL_START)
    return X_VAL;
  else if (address < X_ROW_START)
    return X_COL;
  else if (address < A_COL_START)
    return X_ROW;
  else if (address < A_ROW_START)
    return A_COL;
  else if (address < OUTPUT_START)
    return A_ROW;
  else
  	return OUTPUT;
}