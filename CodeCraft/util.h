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
	} out[8];
	uint16_t id;
	uint8_t cnt = 0;
};
//auto ppp = sizeof(PointData);
struct DemandData
{
	uint16_t idFrom,
		idTo,
		count,
		idNeed[61];
};
struct ResData
{
	uint16_t idLink[638],
		count = 0,
		cost;
};

class Util
{
	static char outfname[256];
	static uint64_t t_begin;
	static TOPOData topo[4800];
public:
	static bool isChk;
	static void Init(const char * fname);

	static uint64_t GetElapse();

	static int16_t ReadFile(const char * fname, PointData * points);
	static int16_t ReadFile(const char * fname, DemandData & dmd);
	static int16_t WriteFile(const ResData * path = nullptr);
};