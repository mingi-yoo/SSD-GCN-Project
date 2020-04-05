#include "IniParser.h"
#include "DataReader.h"
#include "SSDGCNSim.h"
#include "BufferInterface.h"
#include "Common.h"

using namespace std;

extern char *optarg;

void print_initialize(IniParser *i, DataReader *d);

int main(int argc, char** argv)
{
	int option = 0;
	string ini;
	string data;
	
	if (argc == 1)
	{
		cout<<"You must follow this form: \'./sim -i inifile_path -d datafile_path\'"<<endl;
		return 0;
	}

	while ((option = getopt(argc, argv, "i:d:")) != EOF)
	{
		switch (option)
		{
			case 'i':
				ini = optarg;
				break;
			case 'd':
				data = optarg;
				break;
			case '?':
				cout<<"You must follow this form: \'./sim -i inifile_path -d datafile_path\'"<<endl;
				return 0;
		}
	}
	IniParser *iniparser = new IniParser(ini);
	DataReader *datareader = new DataReader(data);

	print_initialize(iniparser,datareader);

	SSDGCNSim *sim = new SSDGCNSim(iniparser, datareader);
	sim->RunSimulator();

	return 0;
}

void print_initialize(IniParser *i, DataReader *d) {
	vector<uint64_t> tempI;
	vector<float> tempF;

	cout << "==IniParser==" << endl;
	cout << "axbuffer : " << i->axbuffer << endl;
	cout << "weightbuffer : " << i->weightbuffer << endl;
	cout << "outputbuffer : " << i->outputbuffer << endl;
	cout << "accdimension : " << i->accdimension << endl;
	cout << "clk_period_in_ns : " << i->clk_period_in_ns << endl;
	cout << "outputfilename : " << i->outputfilename << endl;
	cout << endl;
	cout << "==DataReader==" << endl;
	cout << "weight_h : " << d->weight_h << endl;
	cout << "weight_w : " << d->weight_w << endl;
	tempF = d->ifvalue; 
	cout << "ifvalue : {";
	for (uint64_t i=0; i < tempF.size(); i++) {
		cout << tempF[i] << ", ";
	}
	cout << "}" << endl;
	tempI = d->ifcolindex;
	cout << "ifcolindex : {";
	for (uint64_t i=0; i < tempI.size(); i++) {
		cout << tempI[i] << ", ";
	}
	cout << "}" << endl;
	tempI = d->ifrowindex;
	cout << "ifrowindex : {";
	for (uint64_t i=0; i < tempI.size(); i++) {
		cout << tempI[i] << ", ";
	}
	cout << "}" << endl;
	tempI = d->adjcolindex;
	cout << "adjcolindex : {";
	for (uint64_t i=0; i < tempI.size(); i++) {
		cout << tempI[i] << ", ";
	}
	cout << "}" << endl;
	tempI = d->adjrowindex;
	cout << "adjrowindex : {";
	for (uint64_t i=0; i < tempI.size(); i++) {
		cout << tempI[i] << ", ";
	}
	cout << "}" << endl;

	cout << endl;
}

