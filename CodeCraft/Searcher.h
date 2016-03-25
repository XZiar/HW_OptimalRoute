#pragma once
#include "util.h"

struct _MM_ALIGN32 PMap
{
#if defined(__GNUC__)
	static const uint8_t mask[8];
#endif
	union
	{
		uint8_t datB[80];
		uint64_t datL[10];
#if defined(SSE)
		__m128i datSSE[5];
#   ifdef AVX
		__m256 datAVX[3];
#   endif
#endif
	};

	PMap();
	PMap(const PMap& ori);
	void Clean();
	void Merge(const PMap & left, const PMap & right);
	void Set(const uint16_t id, bool type);
	bool Test(const uint16_t id) const;
	bool Test(const PMap & right) const;
};

struct _MM_ALIGN32 PathData
{
	PMap pmap;
	uint16_t from,
		to,
		cost,
		mid[28];
	uint8_t cnt,
		isEnd;

	PathData();
	void Clean();
	void Merge(const PathData & left, const PathData & right);

	bool operator<(const PathData &pd) const
	{
		return cost == pd.cost ? cnt < pd.cnt : cost < pd.cost;
	}
};
//auto spd = sizeof(PathData);

class Searcher
{
private:
	
public:
	struct _MM_ALIGN32 PathFirst
	{
		PathData paths[130];
		uint16_t from,
			maxcost = 0;
		uint8_t cnt = 0,
			endcnt = 0,
			hasEnd = 0;
	}paths1[52];
	//int kkk = sizeof(PathFirst);
	
	void fastDFS(uint16_t curID);

	struct _MM_ALIGN32 PathLast
	{
		PMap pmap[64];
		PathData *pstack[64];
		uint16_t curCost = 0,
			minCost = 4000,
			lastCost = 4000;
		uint8_t cnt = 0,
			endcnt = 0,
			cntlim = 0;
	}pather;

	PathFirst *path1[600];
	PathData pmain;
	PathData curPath;
	PathFirst * curPit;
	uint8_t maxlevel, maxwide;

	void fastDFSless(PathFirst &pf);
	void fastDFSlessEND(PathFirst &pf);

	void FormRes();
public:
	PointData points[600];
	DemandData demand;
	uint64_t loopcount = 0;
	uint32_t anscnt = 0;
	Searcher();
	~Searcher();

	void Init();
	void Step1(uint8_t maxdepth, uint8_t maxwidth);
	void StepLess();
};

