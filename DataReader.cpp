
/*******************************************************************
* SSD GCN Project
********************************************************************/

#include "DataReader.h"

using namespace std;

DataReader::DataReader(string path) { ReadData(path); }

DataReader::~DataReader() {}

vector<float> DataReader::GetFloatVal(string line, char delimiter) 
{
	vector<float> internal;
	stringstream ss(line);
	string temp;

	while (getline(ss, temp, delimiter)) {
		if (temp == "\n")
			break;
		internal.push_back(stof(temp));
	}
	cout<<"OK"<<endl;

	return internal;
}

vector<uint64_t> DataReader::GetUint64Val(string line, char delimiter)
{
	vector<uint64_t> internal;
	stringstream ss(line);
	string temp;

	while (getline(ss, temp, delimiter)) {
		if (temp == "\n")
			break;
		internal.push_back(stoull(temp));
	}

	return internal;
}

bool DataReader::ReadData(string path)
{
	ifstream openFile(path);
	if (openFile.is_open()) {
		string line;
		getline(openFile, line);
		weight_h = stoi(line);
		getline(openFile, line);
		weight_w = stoi(line);
		getline(openFile, line);
		ifvalue = GetFloatVal(line, ' ');
		getline(openFile, line);
		ifcolindex = GetUint64Val(line, ' ');
		getline(openFile, line);
		ifrowindex = GetUint64Val(line, ' ');
		getline(openFile, line);
		adjcolindex = GetUint64Val(line, ' ');
		getline(openFile, line);
		adjrowindex = GetUint64Val(line, ' ');
		openFile.close();
		return true;
	}
	else {
		throw invalid_argument("Cannot open datafile.");
	}
}
