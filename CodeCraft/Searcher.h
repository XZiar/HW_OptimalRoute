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
			uint16_t from, to,
				mid[12];
			uint8_t cnt, cost, isEnd, toidx;
		};
		__m256i datAVX;
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
//auto spd = sizeof(PMap);
//auto spd = sizeof(PathData);

class Searcher
{
private:
	uint64_t DMDmask[56];
	struct SimArg
	{
		uint16_t RemainCost;
		uint8_t curlevel, epcnt;
	};
	//int tkp = sizeof(SimArg);
public:
	static const uint16_t pPERpf = 320;
	struct _MM_ALIGN32 PathFirst
	{
		PathData paths[162];
		PathData *epaths[160];
		uint16_t from,
			cnt = 0,
			ecutCnt;
		uint8_t 
			endcnt = 0,
			hasEnd = 0;
	}paths1[52];
	int kkk = sizeof(PathFirst);
	struct _MM_ALIGN32 PathFirstTmp
	{
		PathData paths[pPERpf];
		uint16_t from,
			cnt = 0,
			maxcost = 0;
		uint8_t
			endcnt = 0,
			hasEnd = 0;
	}curPit;
	
	
	void fastDFS(PointData::Out *po, const PointData::Out *poend);

	PathData *pstack[60];

	PathFirst *path1[600];
	PathData pmain;
	PathData curPath;
	PMap TMPpmap;
	//PathFirst * curPit;
	uint8_t maxlevel, maxwide, toEPcnt, cutLim_min;

	uint16_t fastDFSv256(PathData * __restrict p, const PathData * __restrict pend, SimArg arg);//VectorTest of 256bit
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
private:
	template <typename T> uint16_t fastDFSv(PathData * __restrict p, const PathData * __restrict pend, SimArg arg)//VectorTest
	{
		const T curPMAP(TMPpmap);
		const uint8_t nextlevel = arg.curlevel + 1;
		if (arg.curlevel == cutLim_min)
		{//early cut
			for (uint8_t a = 0; a < demand.count; a++)
			{
				PathFirst &pf = paths1[a];
				uint16_t ecnt = 0;
				if (!curPMAP.Test(pf.from) || pf.from == p->from)
					for (uint8_t b = pf.endcnt; b < pf.cnt; b++)
					{
						PathData &op = pf.paths[b];
						if (curPMAP.Test(op.pmap))
							pf.epaths[ecnt++] = &op;
					}
				pf.ecutCnt = ecnt;
			}
			PathFirst &cpf = *path1[p->from];//printf("cut finish at %lld\n", Util::GetElapse());
			return fastDFSe<T>(&cpf.epaths[0], &cpf.epaths[cpf.ecutCnt], 0, arg);
		}
		
		for (; p < pend; p++)
		{
			if (arg.RemainCost <= p->cost)//cost too much
				break;//according to order, later ones cost more
		#ifndef FIN
			VTestCnt++;
		#endif
			if (!curPMAP.Test(p->pmap))//has overlap points
				continue;
			//reach next point
			PathFirst &npf = *path1[p->to];
			pstack[arg.curlevel] = p;//add go though
			TMPpmap.Merge(curPMAP, p->pmap);
			const SimArg narg{ uint16_t(arg.RemainCost - p->cost), nextlevel, uint8_t(arg.epcnt - npf.hasEnd) };
		#ifndef FIN
			loopLVcnt[nextlevel]++;
			loopcount++;
		#endif
			if (narg.epcnt != 0)
				arg.RemainCost = p->cost + fastDFSv<T>(&npf.paths[npf.endcnt], &npf.paths[npf.cnt], narg);
		}
		return arg.RemainCost;//refresh lastCost
	}
	template <typename T> uint16_t fastDFSe(PathData * __restrict pcur[], PathData * __restrict pend[], const uint64_t dmdMap, SimArg arg)//Ecut-VectorTest
	{
		const T curPMAP(TMPpmap);
		const uint8_t nextlevel = arg.curlevel + 1;
		for (; pcur < pend; pcur++)
		{
			const PathData * __restrict p = *pcur;
			if (arg.RemainCost <= p->cost)//cost too much
				break;//according to order, later ones cost more
		#ifndef FIN
			VTestCnt++;
		#endif
			if (dmdMap & DMDmask[p->toidx])//already go through
			continue;
			if (!curPMAP.Test(p->pmap))//has overlap points
				continue;
			//reach next point
			PathFirst &npf = *path1[p->to];
			pstack[arg.curlevel] = (PathData *)p;//add go though
			TMPpmap.Merge(curPMAP, p->pmap);
			const SimArg narg{ uint16_t(arg.RemainCost - p->cost), nextlevel, uint8_t(arg.epcnt - npf.hasEnd) };
		#ifndef FIN
			loopLVcnt[nextlevel]++;
			loopcount++;
		#endif
			if (nextlevel != demand.count)
			{
				if (npf.ecutCnt != 0 && narg.epcnt != 0)
					arg.RemainCost = p->cost + fastDFSe<T>(&npf.epaths[0], &npf.epaths[npf.ecutCnt], dmdMap | DMDmask[p->toidx], narg);
			}
			else
				arg.RemainCost = p->cost + fastDFSEND(&npf.paths[0], &npf.paths[npf.endcnt], narg);
		}
		return arg.RemainCost;//refresh lastCost
	}
};

