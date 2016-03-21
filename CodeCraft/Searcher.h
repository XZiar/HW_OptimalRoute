#pragma once
#include "util.h"

struct PMap
{
	static const uint8_t mask[8];
	union
	{
		uint8_t datB[80];
		uint64_t datL[10];
#if defined(SSE)
		__m128i datSSE[5];
#endif
	};

	PMap();
	PMap(const PMap& ori);
	void Merge(const PMap & left, const PMap & right);
	bool Test(const uint16_t id) const;
	bool Test(const PMap & right) const;
};

struct PathData
{
	PMap pmap;
	//float weight = 0.0f;
	uint16_t from,
		to,
		cost,
		cnt,
		mid[32];

	PathData();
	void Clean();
	void Merge(const PathData & left, const PathData & right);
	void Set(const uint16_t id, const bool type);
	bool Test(const uint16_t id) const;

	bool operator<(const PathData &pd) const
	{
		return cost < pd.cost;
	}
	bool operator>(const PathData &pd) const
	{
		return cost > pd.cost;
	}
};
//auto spd = sizeof(PathData);

class Searcher
{
private:
	PathData pmain;
public:
	struct PathFirst
	{
		PathData paths[95];
		uint16_t from,
			maxcost = 0;
		uint8_t cnt = 0;
		bool hasEnd = false;
	}paths1[52];
	PathFirst *path1[600];

	struct PathSecond
	{
		PathData paths[5000];
		PathData endpaths[100];
		uint16_t from,
			maxcost = 0,
			cnt = 0,
			endcnt = 0;
	}paths2[52];
	PathSecond *path2[600];
	

	uint8_t maxlevel, maxwide;
	PathFirst * curPit;
	PathData curPath;
	void fastDFS(uint16_t curID);

	struct PathLast
	{
		PathData *pstack[64];
		PMap pmap[64];
		uint16_t curCost = 0,
			minCost = 4000;
		uint8_t cnt = 0,
			endcnt = 0,
			cntlim = 0;
	}pather;

	void fastDFSless(PathFirst &pf);

	void fastDFSmore(PathSecond &ps);

	void FormRes();
public:
	PointData points[600];
	DemandData demand;

	Searcher();
	~Searcher();

	void Init();
	void Step1(uint8_t maxdepth, uint8_t maxwidth);
	void StepLess();
	void Step2(uint8_t step, uint16_t maxwidth);
	void StepMore();
};

