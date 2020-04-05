/*******************************************************************
* SSD GCN Project
********************************************************************/

#ifndef __SSDGCNSIM_H__
#define __SSDGCNSIM_H__

#include <math.h>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "IniParser.h"
#include "DataReader.h"
#include "DRAMInterface.h"
#include "BufferInterface.h"
#include "Accelerator.h"
#include "Common.h"

using namespace std;

class SSDGCNSim {
public:
	SSDGCNSim(IniParser *iniparser, DataReader * datareader);
	~SSDGCNSim();
	void RunSimulator();
private:
	DRAMInterface *dram;
	BufferInterface *buffer;
	Accelerator *acc;
};

#endif
