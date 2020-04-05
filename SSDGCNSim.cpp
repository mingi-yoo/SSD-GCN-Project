/*******************************************************************
* SSD GCN Project
********************************************************************/

#include "SSDGCNSim.h"

using namespace std;

uint64_t cycle;
uint64_t read_count;
uint64_t write_count;
uint64_t dram_use_byte;

SSDGCNSim::SSDGCNSim(IniParser *iniparser, DataReader *datareader) {
    cout << "==== Initializing SSDGCNSim ====" << endl;
    buffer = new BufferInterface(iniparser->axbuffer,
                                 iniparser->weightbuffer,
                                 iniparser->outputbuffer,
                                 datareader);
    cout << "Buffer is ready!"<<endl;
  	dram = new DRAMInterface("ini/DDR3_micron_64M_8B_x4_sg15.ini", 
                             "system.ini", 
                             "./DRAMSim2/", 
                             "SSDGCNSim", 
                              32768,
                              iniparser->clk_period_in_ns,
                              buffer);
    cout << "DRAMSim is ready!"<< endl;
    acc = new Accelerator(iniparser->accdimension, dram, buffer);
    cout << "Accelerator module is ready!" << endl;
    cout << "==== SSDGCNSim is Ready!! ====" << endl << endl;

  }

SSDGCNSim::~SSDGCNSim() {
  delete buffer;
  delete dram;
  delete acc;
}

void SSDGCNSim::RunSimulator()
{
  cout<<"SSDGCNSim Start..."<<endl;
  while (acc->Run())
  {
    cycle++;
    dram->UpdateCycle();
  }
  cout<<"End... Total Cycle: "<<dec<<cycle<<endl;

  int i = 0;
  struct stat st = {0};
  if (stat("result", &st) == -1)
  {
    mkdir("result", 0700);
  }
  string path = "result/output0.txt";
  while (access(path.c_str(), F_OK) != -1)
  {
    i++;
    path = "result/output"+to_string(i)+".txt";
  }
  ofstream output(path);

  double bps = static_cast<double>(dram_use_byte)/cycle; 

  output<<"Total Cycle: "<<dec<<cycle<<endl;
  output<<"Total Read Request Count: "<<dec<<read_count<<endl;
  output<<"Total Write Request Count: "<<dec<<write_count<<endl;
  output<<"Total DRAM Usage: "<<dec<<dram_use_byte<<"Byte"<<endl;
  output<<"DRAM Usage Byte per second: "<<dec<<bps<<"GByte/s"<<endl;

  output.close();
}
