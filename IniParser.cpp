/*******************************************************************
* SSD GCN Project
********************************************************************/

#include "IniParser.h"

using namespace std;

uint64_t MAX_READ_BYTE; 
uint64_t MAX_READ_INT; 
uint64_t UNIT_INT_BYTE; 

uint64_t A_COL_START;  
uint64_t A_ROW_START;  
uint64_t X_VAL_START;  
uint64_t X_COL_START;  
uint64_t X_ROW_START;  
uint64_t WEIGHT_START; 
uint64_t OUTPUT_START;
uint64_t OUTPUT2_START;

IniParser::IniParser(string path) { ReadIni(path); }

IniParser::~IniParser() {}

bool IniParser::ReadIni(string path)
{
	ifstream openFile(path);
	if (openFile.is_open()) {
		string line;
		while (getline(openFile, line)) {
			string delimiter = " = ";
			if (string::npos == line.find(delimiter)) 
				delimiter = "=";
			string token1 = line.substr(0, line.find(delimiter));
			string token2 = line.substr(line.find(delimiter)+delimiter.length(), line.length());
			m_table[token1] = token2;
		}
		openFile.close();
		ParseIni();
		return true;
	}
	else {
		throw invalid_argument("Cannot open inifile");
	}
}

bool IniParser::Contain(string name)
{
	if (m_table.find(name) == m_table.end()) 
		return false;
	return true;
}

bool IniParser::GetBool(string name)
{
	if (Contain(name)) {
		if (m_table[name][0] == 't' || m_table[name][0] == 'T')
			return true;
		else
			return false;
	}
	else {
		throw invalid_argument("Not exist.");
	}
}

string IniParser::GetString(string name)
{
	if (Contain(name)) {
		if (m_table[name].find("\"") == string::npos)
			return m_table[name];
		else
			return m_table[name].substr(1, m_table[name].length() - 2);
	}
	else {
		throw invalid_argument("Not exist.");
	}
}

float IniParser::GetFloat(string name)
{
	if (Contain(name))
		return stof(m_table[name]);
	else
		throw invalid_argument("Not exist.");
}

int IniParser::GetInt(string name)
{
	if (Contain(name))
		return stoi(m_table[name]);
	else
		throw invalid_argument("Not exist.");
}

uint64_t IniParser::GetUint64(string name)
{
	if (Contain(name))
		return stoull(m_table[name], NULL, 0);
	else
		throw invalid_argument("Not exist.");
}

void IniParser::ParseIni()
{
	axbuffer = GetUint64("AXBufferSize");
	weightbuffer = GetUint64("WeightBufferSize");
	outputbuffer = GetUint64("OutputBufferSize");
	accdimension = GetUint64("AccDimension");
	clk_period_in_ns = GetFloat("ClockPeriod");
	MAX_READ_BYTE = GetUint64("MaximumByteRead");
	MAX_READ_INT = GetUint64("MaximumNumberRead");
	UNIT_INT_BYTE = GetUint64("UnitByteforNumber");
	A_COL_START = GetUint64("AColStartAddress");
	A_ROW_START = GetUint64("ARowStartAddress");
	X_VAL_START = GetUint64("XValStartAddress");
	X_COL_START = GetUint64("XColStartAddress");
	X_ROW_START = GetUint64("XRowStartAddress");
	WEIGHT_START = GetUint64("WeightStartAddress");
	OUTPUT_START = GetUint64("XOutputStartAddress");
	OUTPUT2_START = GetUint64("AOutputStartAddress");
}
