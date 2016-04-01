#pragma once

struct TOPOData
{
	uint16_t idLink, 
		idSrc, 
		idDest, 
		cost;
};
struct PointData
{
	struct Out
	{
		uint16_t dest;
		uint16_t rid;
		uint16_t dis;
	} out[128];
	uint16_t id;
	uint8_t cnt = 0;
};
//auto ppp = sizeof(PointData);
struct DemandData
{
	uint16_t idFrom,
		idTo,
		count,
		idNeed[52];
	uint8_t map[600];
};
int poo = sizeof(DemandData);
struct ResData
{
	uint16_t idLink[638],
		count = 0,
		cost = 0;
};

class Util
{
	static char outfname[256];
	static uint64_t t_begin;
	//static TOPOData topo[4800];
public:
	static void Init(const char * fname);

	static uint64_t GetElapse();

	static int16_t ReadFile(const char * fname, PointData * points, uint16_t &maxid);
	static int16_t ReadFile(const char * fname, DemandData & dmd);
	static int16_t WriteFile(const ResData * path = nullptr);
};