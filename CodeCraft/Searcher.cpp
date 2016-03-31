#include "rely.h"
#include "Searcher.h"

static const uint32_t PMPmask[32] =
{
	0x80000000,0x40000000,0x20000000,0x10000000,0x8000000,0x4000000,0x2000000,0x1000000,
	0x800000,  0x400000,  0x200000,  0x100000,  0x80000,  0x40000,  0x20000,  0x10000,
	0x8000,    0x4000,    0x2000,    0x1000,    0x800,    0x400,    0x200,    0x100,
	0x80,      0x40,      0x20,      0x10,      0x8,      0x4,      0x2,      0x1
};
PMap::PMap()
{
	Clean();
}
PMap::PMap(const PMap & ori)
{
	_mm256_store_si256(&datAVXi[0], _mm256_load_si256(&ori.datAVXi[0]));
	_mm256_store_si256(&datAVXi[1], _mm256_load_si256(&ori.datAVXi[1]));
	_mm256_store_si256(&datAVXi[2], _mm256_load_si256(&ori.datAVXi[2]));
}
PMap & PMap::operator=(const PMap & from)
{
#if defined(SSE)
#   ifdef AVX
	_mm256_stream_si256(&datAVXi[0], _mm256_load_si256(&from.datAVXi[0]));
	_mm256_stream_si256(&datAVXi[1], _mm256_load_si256(&from.datAVXi[1]));
	_mm256_stream_si256(&datAVXi[2], _mm256_load_si256(&from.datAVXi[2]));
#   else
	_mm_store_si128(&datSSE[0], _mm_load_si128(&from.datSSE[0]));
	_mm_store_si128(&datSSE[1], _mm_load_si128(&from.datSSE[1]));
	_mm_store_si128(&datSSE[2], _mm_load_si128(&from.datSSE[2]));
	_mm_store_si128(&datSSE[3], _mm_load_si128(&from.datSSE[3]));
	_mm_store_si128(&datSSE[4], _mm_load_si128(&from.datSSE[4]));
#   endif
#else
	memcpy(datB, ori.datB, sizeof(datB));
#endif	
	return *this;
}
void PMap::Clean()
{
#if defined(SSE)
#   ifdef AVX
	const __m256 dat = _mm256_setzero_ps();
	_mm256_store_ps((float*)&datAVX[0], dat);
	_mm256_store_ps((float*)&datAVX[1], dat);
	_mm256_store_ps((float*)&datAVX[2], dat);
#   else
	const __m128i dat = _mm_setzero_si128();
	_mm_store_si128(&datSSE[0], dat);
	_mm_store_si128(&datSSE[1], dat);
	_mm_store_si128(&datSSE[2], dat);
	_mm_store_si128(&datSSE[3], dat);
	_mm_store_si128(&datSSE[4], dat);
#   endif
#else
	memset(datB, 0, sizeof(datB));
#endif
}
void PMap::Merge(const PMap & left, const PMap & right)
{
#if defined(SSE)
#   ifdef AVX
	__m256 t0 = _mm256_or_ps(_mm256_load_ps((float*)&left.datAVX[0]), _mm256_load_ps((float*)&right.datAVX[0])),
		t1 = _mm256_or_ps(_mm256_load_ps((float*)&left.datAVX[1]), _mm256_load_ps((float*)&right.datAVX[1])),
		t2 = _mm256_or_ps(_mm256_load_ps((float*)&left.datAVX[2]), _mm256_load_ps((float*)&right.datAVX[2]));
	_mm256_store_ps((float*)&datAVX[0], t0);
	_mm256_store_ps((float*)&datAVX[1], t1);
	_mm256_store_ps((float*)&datAVX[2], t2);
#   else
	_mm_store_si128(&datSSE[0], _mm_or_si128(_mm_load_si128(&left.datSSE[0]), _mm_load_si128(&right.datSSE[0])));
	_mm_store_si128(&datSSE[1], _mm_or_si128(_mm_load_si128(&left.datSSE[1]), _mm_load_si128(&right.datSSE[1])));
	_mm_store_si128(&datSSE[2], _mm_or_si128(_mm_load_si128(&left.datSSE[2]), _mm_load_si128(&right.datSSE[2])));
	_mm_store_si128(&datSSE[3], _mm_or_si128(_mm_load_si128(&left.datSSE[3]), _mm_load_si128(&right.datSSE[3])));
	_mm_store_si128(&datSSE[4], _mm_or_si128(_mm_load_si128(&left.datSSE[4]), _mm_load_si128(&right.datSSE[4])));
#   endif
#else
	for (int a = 0; a < 10; a++)
		datL[a] = left.datL[a] | right.datL[a];
#endif
}
void PMap::Merge(const PMap512 & left, const PMap & right)
{
	__m256 t0 = _mm256_or_ps(_mm256_load_ps((float*)&left.datAVX[0]), _mm256_load_ps((float*)&right.datAVX[0])),
		t1 = _mm256_or_ps(_mm256_load_ps((float*)&left.datAVX[1]), _mm256_load_ps((float*)&right.datAVX[1]));
	_mm256_store_ps((float*)&datAVX[0], t0);
	_mm256_store_ps((float*)&datAVX[1], t1);
}
void PMap::Merge(const PMap256 & left, const PMap & right)
{
	__m256 t0 = _mm256_or_ps(_mm256_load_ps((float*)&left.datAVX), _mm256_load_ps((float*)&right.datAVX[0]));
	_mm256_store_ps((float*)&datAVX[0], t0);
}
void PMap::Set(uint16_t id, bool type)
{
	if (type)
		datI[id >> 5] |= PMPmask[id & 0x1f];
	else
		datI[id >> 5] &= ~PMPmask[id & 0x1f];
}
bool PMap::Test(uint16_t id) const
{
	return (datI[id >> 5] & PMPmask[id & 0x1f]) != 0x0;
}
bool PMap::Test(const PMap & right) const
{
#if defined(SSE)
#   ifdef AVX
	__m256 t0 = _mm256_and_ps(_mm256_load_ps((float*)&datAVX[0]), _mm256_load_ps((float*)&right.datAVX[0])),
		t1 = _mm256_and_ps(_mm256_load_ps((float*)&datAVX[1]), _mm256_load_ps((float*)&right.datAVX[1])),
		t2 = _mm256_and_ps(_mm256_load_ps((float*)&datAVX[2]), _mm256_load_ps((float*)&right.datAVX[2]));
	__m256i a0 = _mm256_castps_si256(t0);
	if (!_mm256_testz_si256(a0, a0))
		return false;
	__m256i a1 = _mm256_castps_si256(_mm256_or_ps(t1, t2));
	return  _mm256_testz_si256(a1, a1);
#   else
	__m128i t0 = _mm_and_si128(_mm_load_si128(&datSSE[0]), _mm_load_si128(&right.datSSE[0])),
		t1 = _mm_and_si128(_mm_load_si128(&datSSE[1]), _mm_load_si128(&right.datSSE[1])),
		t2 = _mm_and_si128(_mm_load_si128(&datSSE[2]), _mm_load_si128(&right.datSSE[2])),
		t3 = _mm_and_si128(_mm_load_si128(&datSSE[3]), _mm_load_si128(&right.datSSE[3])),
		t4 = _mm_and_si128(_mm_load_si128(&datSSE[4]), _mm_load_si128(&right.datSSE[4]));
	__m128i p0 = _mm_or_si128(t0, t1),
		p1 = _mm_or_si128(t2, t3);
	__m128i a0 = _mm_or_si128(p0, t4);
	return (_mm_testz_si128(a0, a0) && _mm_testz_si128(p1, p1));//all zero
#    endif
#else
	for (int a = 0; a < 10; a++)
		if (datL[a] & right.datL[a])
			return false;
	return true;
#endif
}


PMap512::PMap512()
{
	const __m256 dat = _mm256_setzero_ps();
	_mm256_store_ps((float*)&datAVX[0], dat);
	_mm256_store_ps((float*)&datAVX[1], dat);
}
PMap512::PMap512(const PMap & from)
{
	_mm256_store_si256(&datAVXi[0], _mm256_load_si256(&from.datAVXi[0]));
	_mm256_store_si256(&datAVXi[1], _mm256_load_si256(&from.datAVXi[1]));
}
void PMap512::Set(uint16_t id, bool type)
{
	if (type)
		datI[id >> 5] |= PMPmask[id & 0x1f];
	else
		datI[id >> 5] &= ~PMPmask[id & 0x1f];
}
bool PMap512::Test(uint16_t id) const
{
	return (datI[id >> 5] & PMPmask[id & 0x1f]) != 0x0;
}
bool PMap512::Test(const PMap & right) const
{
	__m256 t0 = _mm256_and_ps(_mm256_load_ps((float*)&datAVX[0]), _mm256_load_ps((float*)&right.datAVX[0])),
		t1 = _mm256_and_ps(_mm256_load_ps((float*)&datAVX[1]), _mm256_load_ps((float*)&right.datAVX[1]));
	__m256i a0 = _mm256_castps_si256(t0),
		a1 = _mm256_castps_si256(t1);
	if (!_mm256_testz_si256(a0, a0))
		return false;
	return  _mm256_testz_si256(a1, a1);
}

PMap256::PMap256()
{
	_mm256_store_ps((float*)&datAVX, _mm256_setzero_ps());
}
PMap256::PMap256(const PMap & from)
{
	_mm256_store_si256(&datAVXi, _mm256_load_si256(&from.datAVXi[0]));
}
bool PMap256::Test(const PMap & right) const
{
	__m256 t0 = _mm256_and_ps(_mm256_load_ps((float*)&datAVX), _mm256_load_ps((float*)&right.datAVX[0]));
	__m256i a0 = _mm256_castps_si256(t0);
	return _mm256_testz_si256(a0, a0);
}


PathData::PathData()
{
	Clean();
}

PathData & PathData::operator=(const PathData & from)
{
	pmap = from.pmap;
#ifdef AVX
	_mm256_stream_si256(&datAVX[0], _mm256_load_si256(&from.datAVX[0]));
	_mm256_stream_si256(&datAVX[1], _mm256_load_si256(&from.datAVX[1]));
#else
	memcpy(datB, ori.datB, sizeof(datB));
#endif	
	return *this;
}

void PathData::Clean()
{
	pmap.Clean();
	cost = cnt = isEnd = ecut = 0;
}

void PathData::Merge(const PathData & left, const PathData & right)
{
	from = left.from, to = right.to;
	cost = left.cost + right.cost;
	cnt = left.cnt + right.cnt;
	memcpy(mid, left.mid, sizeof(uint16_t)*left.cnt);
	memcpy(mid + left.cnt, right.mid, sizeof(uint16_t)*right.cnt);
	pmap.Merge(left.pmap, right.pmap);
}




Searcher::Searcher()
{
}


Searcher::~Searcher()
{
}

static inline bool POutJudge(PointData::Out &o1, PointData::Out &o2)
{
	return o1.dis < o2.dis;
}
void Searcher::Init()
{
	pmain.from = demand.idFrom, pmain.to = demand.idTo;
	demand.idNeed[demand.count] = demand.idFrom;
	pmain.pmap.Set(demand.idTo, true);
	for (int a = 0; a < demand.count; a++)
		pmain.pmap.Set(demand.idNeed[a], true);

	for (PointData &p : points)
	{
		if (p.cnt > 1)
			sort(p.out, p.out + p.cnt, POutJudge);
	}
}

void Searcher::fastDFS(PointData::Out *po, const PointData::Out *poend)
{
	for (; po < poend; po++)
	{
		if (curPit->maxcost - curPath.cost < po->dis)//this is too long
			continue;//according to order, later ones are much longer
		uint16_t thisID = po->dest;
		if (curPath.pmap.Test(thisID))//already go through
			continue;
		if (pmain.pmap.Test(thisID))//reach need-point
		{
			curPath.cost += po->dis;//add cost
			curPath.mid[curPath.cnt++] = po->rid;//add go though
			curPath.pmap.Set(thisID, true);//set bitmap
			curPath.to = thisID;//add destination
			curPath.isEnd = (thisID == pmain.to ? 0x7f : 0x0);

			curPit->paths[curPit->cnt++] = curPath;//add path
			if (curPit->cnt > 249)//has space
			{//full,but cost lower
				sort(curPit->paths, curPit->paths + 250);//sort to find out the longest
				curPit->maxcost = curPit->paths[maxwide - 1].cost;//refresh max-cost
				curPit->cnt = maxwide;
			}

			curPath.pmap.Set(thisID, false);//clear bitmap
			curPath.cnt--;//rollback go though
			curPath.cost -= po->dis;//rollback cost
			continue;
		}
		//reach normal-point
		else if (curPath.cnt < maxlevel && thisID != pmain.from)//still can go deeper and not toward start-point
		{
			curPath.cost += po->dis;//add cost
			curPath.mid[curPath.cnt++] = po->rid;//add go though
			curPath.pmap.Set(thisID, true);//set bitmap
			
			PointData &np = points[thisID];
			fastDFS(&np.out[0], &np.out[np.cnt]);//into next point

			curPath.pmap.Set(thisID, false);//clear bitmap
			curPath.cnt--;//rollback go though
			curPath.cost -= po->dis;//rollback cost
			continue;
		}
	}
	//finish this point
	return;
}

static inline bool Separator(const PathData &pa, const PathData &pb)
{
	return pa.isEnd == pb.isEnd ? (pa.cost == pb.cost ? pa.cnt < pb.cnt : pa.cost < pb.cost) : (pa.isEnd > pb.isEnd);
	//return pa.isEnd == pb.isEnd ? (pa.cnt == pb.cnt ? pa.cost < pb.cost : pa.cnt < pb.cnt) : (pa.isEnd > pb.isEnd);
}
void Searcher::Step1(uint8_t maxdepth, uint8_t maxwidth)
{
	maxlevel = maxdepth;
	maxwide = maxwidth;
	toEPcnt = 0;
	for (int a = 0; a <= demand.count; a++)
	{
		curPath.Clean();
		curPit = path1[demand.idNeed[a]] = &paths1[a];
		curPit->maxcost = 100, curPit->cnt = 0;
		curPit->from = curPath.from = demand.idNeed[a];

		PointData &np = points[curPath.from];
		fastDFS(&np.out[0], &np.out[np.cnt]);
		
		sort(curPit->paths, curPit->paths + curPit->cnt, Separator);
		curPit->cnt = min(curPit->cnt, maxwide);

		int b = 0;
		while (b < curPit->cnt && curPit->paths[b].isEnd)
			b++;
		curPit->endcnt = b;
		if (b > 0 && a != demand.count)
		{
			curPit->hasEnd = 0x1;
			++toEPcnt;
		}
		else
			curPit->hasEnd = 0x0;
		//printf("%dth,%d,sort:%d\n", a, curPath.from, anscnt);
	}
}


uint16_t Searcher::fastDFSv768(PathData * __restrict p, const PathData * __restrict pend, SimArg arg)
{
	const PMap curPMAP(pather.pmap[arg.curlevel]);
	const uint8_t nextlevel = arg.curlevel + 1;
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
		pather.pstack[arg.curlevel] = p;//add go though
		pather.pmap[nextlevel].Merge(curPMAP, p->pmap);
		const SimArg narg{ uint16_t(arg.RemainCost - p->cost), nextlevel, uint8_t(arg.epcnt - npf.hasEnd) };
	#ifndef FIN
		loopLVcnt[nextlevel]++;
		loopcount++;
	#endif
		if (nextlevel != demand.count)
		{
			if (narg.epcnt != 0)
				arg.RemainCost = p->cost + fastDFSv768(&npf.paths[npf.endcnt], &npf.paths[npf.cnt], narg);
		}
		else
			arg.RemainCost = p->cost + fastDFSEND(&npf.paths[0], &npf.paths[npf.endcnt], narg);
	}//finish this point
	return arg.RemainCost;//refresh lastCost
}
uint16_t Searcher::fastDFSv512(PathData * __restrict p, const PathData * __restrict pend, SimArg arg)
{
	const PMap512 curPMAP(pather.pmap[arg.curlevel]);
	const uint8_t nextlevel = arg.curlevel + 1;

	if (arg.curlevel == demand.limcut)
	{//early cut
		for (uint8_t a = 0; a < demand.count; a++)
		{
			PathFirst &pf = paths1[a];
			uint8_t ecnt = 0;
			if (!curPMAP.Test(pf.from) || pf.from == p->from)
			{
				for (uint8_t b = pf.endcnt; b < pf.cnt; b++)
				{
					PathData &op = pf.paths[b];
					if (curPMAP.Test(op.pmap))
						pf.epaths[ecnt++] = &op;
				}
			}
			pf.ecnt = ecnt;
		}
		PathFirst *cpf = path1[p->from];
		//printf("cut finish at %lld, cut %d\n", Util::GetElapse(), eccnt);
		return fastDFSv512e(&cpf->epaths[0], &cpf->epaths[cpf->ecnt], arg);//refresh lastCost
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
		pather.pstack[arg.curlevel] = p;//add go though
		pather.pmap[nextlevel].Merge(curPMAP, p->pmap);
		const SimArg narg{ uint16_t(arg.RemainCost - p->cost), nextlevel, uint8_t(arg.epcnt - npf.hasEnd) };
	#ifndef FIN
		loopLVcnt[nextlevel]++;
		loopcount++;
	#endif
		if (nextlevel != demand.count)
		{
			if (narg.epcnt != 0)
				arg.RemainCost = p->cost + fastDFSv512(&npf.paths[npf.endcnt], &npf.paths[npf.cnt], narg);
		}
		else
			arg.RemainCost = p->cost + fastDFSEND(&npf.paths[0], &npf.paths[npf.endcnt], narg);
	}
	//finish this point
	return arg.RemainCost;//refresh lastCost
}
uint16_t Searcher::fastDFSv512e(PathData * __restrict pcur[], PathData * __restrict pend[], SimArg arg)
{
	const PMap512 curPMAP(pather.pmap[arg.curlevel]);
	const uint8_t nextlevel = arg.curlevel + 1;

	for (; pcur < pend; pcur++)
	{
		const PathData * __restrict p = *pcur;
		if (arg.RemainCost <= p->cost)//cost too much
			break;//according to order, later ones cost more
	#ifndef FIN
		VTestCnt++;
	#endif
		if (!curPMAP.Test(p->pmap))//has overlap points
			continue;
		//reach next point
		PathFirst &npf = *path1[p->to];
		pather.pstack[arg.curlevel] = (PathData *)p;//add go though
		pather.pmap[nextlevel].Merge(curPMAP, p->pmap);
		const SimArg narg{ uint16_t(arg.RemainCost - p->cost), nextlevel, uint8_t(arg.epcnt - npf.hasEnd) };
	#ifndef FIN
		loopLVcnt[nextlevel]++;
		loopcount++;
	#endif
		if (nextlevel != demand.count)
		{
			if (narg.epcnt != 0)
				arg.RemainCost = p->cost + fastDFSv512e(&npf.epaths[0], &npf.epaths[npf.ecnt], narg);
		}
		else
			arg.RemainCost = p->cost + fastDFSEND(&npf.paths[0], &npf.paths[npf.endcnt], narg);
	}
	//finish this point
	return arg.RemainCost;//refresh lastCost
}
uint16_t Searcher::fastDFSv256(PathData * __restrict p, const PathData * __restrict pend, SimArg arg)
{
	const PMap256 curPMAP(pather.pmap[arg.curlevel]);
	const uint8_t nextlevel = arg.curlevel + 1;
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
		pather.pstack[arg.curlevel] = p;//add go though
		pather.pmap[nextlevel].Merge(curPMAP, p->pmap);
		const SimArg narg{ uint16_t(arg.RemainCost - p->cost), nextlevel, uint8_t(arg.epcnt - npf.hasEnd) };
	#ifndef FIN
		loopLVcnt[nextlevel]++;
		loopcount++;
	#endif
		if (nextlevel != demand.count)
		{
			if (narg.epcnt != 0)
				arg.RemainCost = p->cost + fastDFSv256(&npf.paths[npf.endcnt], &npf.paths[npf.cnt], narg);
		}
		else
			arg.RemainCost = p->cost + fastDFSEND(&npf.paths[0], &npf.paths[npf.endcnt], narg);
	}
	//finish this point
	return arg.RemainCost;//refresh lastCost
}
uint16_t Searcher::fastDFSb512(PathData * __restrict p, const PathData * __restrict pend, SimArg arg)
{
	static PMap512 dmdPMAP;
	const PMap512 curPMAP(pather.pmap[arg.curlevel]);
	const uint8_t nextlevel = arg.curlevel + 1;
	for (; p < pend; p++)
	{
		if (arg.RemainCost <= p->cost)//cost too much
			break;//according to order, later ones cost more
	#ifndef FIN
		VTestCnt++;
	#endif
		if (dmdPMAP.Test(p->to))//already go through
			continue;
		if (!curPMAP.Test(p->pmap))//has overlap points
			continue;

		//reach next point
		PathFirst &npf = *path1[p->to];
		pather.pstack[arg.curlevel] = p;//add go though
		pather.pmap[nextlevel].Merge(curPMAP, p->pmap);
		const SimArg narg{ uint16_t(arg.RemainCost - p->cost), nextlevel, uint8_t(arg.epcnt - npf.hasEnd) };
	#ifndef FIN
		loopLVcnt[nextlevel]++;
		loopcount++;
	#endif
		if (nextlevel != demand.count)
		{
			if (narg.epcnt != 0)
			{
				dmdPMAP.Set(p->to, true);
				arg.RemainCost = p->cost + fastDFSb512(&npf.paths[npf.endcnt], &npf.paths[npf.cnt], narg);
				dmdPMAP.Set(p->to, false);
			}
		}
		else
			arg.RemainCost = p->cost + fastDFSEND(&npf.paths[0], &npf.paths[npf.endcnt], narg);
	}
	//finish this point
	return arg.RemainCost;//refresh lastCost
}
uint16_t Searcher::fastDFSEND(PathData * __restrict p, const PathData * __restrict pend, SimArg arg)
{
	const PMap curPMAP(pather.pmap[arg.curlevel]);
	for (; p < pend; p++)
	{
		if (arg.RemainCost <= p->cost)//cost too much
			break;//according to order, later ones cost more
		if (!curPMAP.Test(p->pmap))//has overlap points
			continue;

		//final step,find a shorter route
		pather.pstack[arg.curlevel] = p;
		FormRes();
		
		return p->cost;//according to order, later ones cost more
	}
	//finish this point
	return arg.RemainCost;//refresh lastCost
}


void Searcher::FormRes()
{
	ResData res = ResData();
	for (int a = 0; a <= demand.count; a++)
	{
		PathData &p = *pather.pstack[a];
		res.cost += p.cost;
		for (int b = 0; b < p.cnt; b++)
		{
			res.idLink[res.count++] = p.mid[b];
		}
	}
	Util::WriteFile(&res);
}

void Searcher::StepEnd(const uint16_t maxid)
{
	pather.pmap[0].Clean();
	PathFirst *pf = path1[pmain.from];
	SimArg arg{ 1000, 0, toEPcnt };

	if (maxid > 510)//>512
	{
	#ifdef FIN
		arg.RemainCost = 640;
	#endif
		fastDFSv768(&pf->paths[pf->endcnt], &pf->paths[pf->cnt], arg);
	}
	else if (maxid > 256)//>256
	{
		demand.limcut = demand.count - 12;
	#ifdef FIN
		arg.RemainCost = 500;
	#endif
		//if (maxid / demand.count < 50)
			fastDFSv512(&pf->paths[pf->endcnt], &pf->paths[pf->cnt], arg);
		//else
			//fastDFSb512(&pf->paths[pf->endcnt], &pf->paths[pf->cnt], arg);
	}
	else//<256
	{	
	#ifdef FIN
		arg.RemainCost = 300;
	#endif
		fastDFSv256(&pf->paths[pf->endcnt], &pf->paths[pf->cnt], arg);
	}
}

