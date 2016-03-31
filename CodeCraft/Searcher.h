#pragma once
#include "util.h"
struct _MM_ALIGN32 PMap512;
struct _MM_ALIGN32 PMap256;
struct _MM_ALIGN32 PMap
{
	union
	{
		uint32_t datI[20];
		uint64_t datL[10];
#if defined(SSE)
		__m128i datSSE[5];
#   ifdef AVX
		__m256 datAVX[3];
		__m256i datAVXi[3];
#   endif
#endif
	};

	PMap();
	PMap(const PMap & ori);
	PMap & operator= (const PMap & from);
	void Clean();
	void Merge(const PMap & left, const PMap & right);
	void Merge(const PMap512 & left, const PMap & right);
	void Merge(const PMap256 & left, const PMap & right);
	void Set(const uint16_t id, bool type);
	bool Test(const uint16_t id) const;
	bool Test(const PMap & right) const;
};
struct _MM_ALIGN32 PMap512
{
	union
	{
		uint32_t datI[16];
		__m256 datAVX[2];
		__m256i datAVXi[2];
	};
	PMap512();
	PMap512(const PMap & from);
	void Set(const uint16_t id, bool type);
	bool Test(const uint16_t id) const;
	bool Test(const PMap & right) const;
};
struct _MM_ALIGN32 PMap256
{
	union
	{
		__m256 datAVX;
		__m256i datAVXi;
	};
	PMap256();
	PMap256(const PMap & from);
	bool Test(const PMap & right) const;
};

struct _MM_ALIGN32 PathData
{
	PMap pmap;
	union
	{
		struct
		{
			uint16_t from, to, cost,
				mid[27];
			uint8_t cnt, isEnd, ecut;
		};
		__m256i datAVX[2];
	};

	PathData();
	PathData & operator= (const PathData & from);
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
	struct SimArg
	{
		uint16_t RemainCost;
		uint8_t curlevel, epcnt;
	};
	//int tkp = sizeof(SimArg);
public:
	struct _MM_ALIGN32 PathFirst
	{
		PathData paths[250];
		PathData *epaths[160];
		uint16_t from,
			maxcost = 0;
		uint8_t ecnt,
			cnt = 0,
			endcnt = 0,
			hasEnd = 0;
	}paths1[52];
	//int kkk = sizeof(PathFirst);
	
	void fastDFS(PointData::Out *po, const PointData::Out *poend);

	struct _MM_ALIGN32 PathLast
	{
		PMap pmap[64];
		PathData *pstack[60];
	}pather;

	PathFirst *path1[600];
	PathData pmain;
	PathData curPath;
	PathFirst * curPit;
	uint8_t maxlevel, maxwide, limcut, toEPcnt;

	uint16_t fastDFSv768(PathData * __restrict p, const PathData * __restrict pend, SimArg arg);//VectorTest of 768bit
	uint16_t fastDFSv768e(PathData * __restrict pcur[], PathData * __restrict pend[], SimArg arg);//Ecut-VectorTest of 768bit
	uint16_t fastDFSv512(PathData * __restrict p, const PathData * __restrict pend, SimArg arg);//VectorTest of 512bit
	uint16_t fastDFSv512e(PathData * __restrict pcur[], PathData * __restrict pend[], SimArg arg);//Ecut-VectorTest of 512bit
	uint16_t fastDFSv256(PathData * __restrict p, const PathData * __restrict pend, SimArg arg);//VectorTest of 512bit
	uint16_t fastDFSb512(PathData * __restrict p, const PathData * __restrict pend, SimArg arg);//BitTest of 512bit
	uint16_t fastDFSEND(PathData * __restrict p, const PathData * __restrict pend, SimArg arg);

	void FormRes();
public:
	PointData points[600];
	DemandData demand;
	uint64_t loopLVcnt[60], EPconflic = 0, VTestCnt = 0, loopcount = 0;
	Searcher();
	~Searcher();

	void Init();
	void Step1(uint8_t maxdepth, uint8_t maxwidth);
	void StepEnd(const uint16_t maxid);
};

