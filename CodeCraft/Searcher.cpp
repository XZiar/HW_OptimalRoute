#include "rely.h"
#include "Searcher.h"

#if defined(__GNUC__)
const uint8_t PMap::mask[8] = { 0x80,0x40,0x20,0x10,0x8,0x4,0x2,0x1 };
#endif
PMap::PMap()
{
	Clean();
}
PMap::PMap(const PMap & ori)
{
#if defined(SSE)
#   ifdef AVX
	_mm256_store_ps((float*)&datAVX[0], _mm256_load_ps((float*)&ori.datAVX[0]));
	_mm256_store_ps((float*)&datAVX[1], _mm256_load_ps((float*)&ori.datAVX[1]));
	_mm256_store_ps((float*)&datAVX[2], _mm256_load_ps((float*)&ori.datAVX[2]));
#   else
	_mm_store_si128(&datSSE[0], _mm_load_si128(&ori.datSSE[0]));
	_mm_store_si128(&datSSE[1], _mm_load_si128(&ori.datSSE[1]));
	_mm_store_si128(&datSSE[2], _mm_load_si128(&ori.datSSE[2]));
	_mm_store_si128(&datSSE[3], _mm_load_si128(&ori.datSSE[3]));
	_mm_store_si128(&datSSE[4], _mm_load_si128(&ori.datSSE[4]));
#   endif
#else
	memcpy(datB, ori.datB, sizeof(datB));
#endif	
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
void PMap::Set(uint16_t id, bool type)
{
#if defined(__GNUC__)
	if (type)
		datB[id >> 3] |= mask[id & 0x7];
	else
		datB[id >> 3] &= ~mask[id & 0x7];
#else
	long long * ptr = (long long*)&datL[id >> 6];
	if (type)
		_bittestandset64(ptr, id & 0x3f);
	else
		_bittestandreset64(ptr, id & 0x3f);
#endif
}
bool PMap::Test(uint16_t id) const
{
#if defined(__GNUC__)
	return (datB[id >> 3] & mask[id & 0x7]) != 0x0;
#else
	return _bittest64((long long*)&datL[id >> 6], id & 0x3f);
#endif
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


PathData::PathData()
{
	Clean();
}

void PathData::Clean()
{
	pmap.Clean();
	cost = cnt = isEnd = 0;
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


void Searcher::Init()
{
	pmain.from = demand.idFrom, pmain.to = demand.idTo;
	demand.idNeed[demand.count] = demand.idFrom;
	pmain.pmap.Set(demand.idTo, true);
	for (int a = 0; a < demand.count; a++)
		pmain.pmap.Set(demand.idNeed[a], true);
}

void Searcher::fastDFS(uint16_t curID)
{
	PointData &p = points[curID];
	for (uint8_t a = 0; a < p.cnt; a++)
	{
		if (curPit->cnt >= maxwide && curPit->maxcost - curPath.cost < p.out[a].dis)//this is too long
			break;//according to order, later ones are much longer
		uint16_t &thisID = p.out[a].dest;
		if (curPath.pmap.Test(thisID))//already go through
			continue;
		if (pmain.pmap.Test(thisID))//reach need-point
		{
			curPath.cost += p.out[a].dis;//add cost
			curPath.mid[curPath.cnt++] = p.out[a].rid;//add go though
			curPath.pmap.Set(thisID, true);//set bitmap
			curPath.to = thisID;//add destination
			curPath.isEnd = (thisID == pmain.to ? 0x7f : 0x0);
			
			if (curPit->cnt < maxwide)//has space
			{
				curPit->paths[curPit->cnt++] = curPath;//add psth
				curPit->maxcost = max(curPath.cost, curPit->maxcost);//refresh max-cost
			}
			else
			{//full,but cost lower
				sort(curPit->paths, curPit->paths + maxwide);//sort to find out the longest
				curPit->paths[maxwide - 1] = curPath;//replace
				curPit->maxcost = max(curPath.cost, curPit->paths[maxwide - 2].cost);//refresh max-cost
			}

			curPath.pmap.Set(thisID, false);//clear bitmap
			curPath.cnt--;//rollback go though
			curPath.cost -= p.out[a].dis;//rollback cost
			continue;
		}
		//reach normal-point
		else if (curPath.cnt < maxlevel && thisID != pmain.from)//still can go deeper and not toward start-point
		{
			curPath.cost += p.out[a].dis;//add cost
			curPath.mid[curPath.cnt++] = p.out[a].rid;//add go though
			curPath.pmap.Set(thisID, true);//set bitmap
			
			fastDFS(thisID);//into next point

			curPath.pmap.Set(thisID, false);//clear bitmap
			curPath.cnt--;//rollback go though
			curPath.cost -= p.out[a].dis;//rollback cost
			continue;
		}
	}
	//finish this point
	return;
}
static bool Separator(const PathData &pa, const PathData &pb)
{
	return pa.isEnd == pb.isEnd ? (pa.cost == pb.cost ? pa.cnt < pb.cnt : pa.cost < pb.cost) : (pa.isEnd > pb.isEnd);
}
void Searcher::Step1(uint8_t maxdepth, uint8_t maxwidth)
{
	maxlevel = maxdepth;
	maxwide = maxwidth;
	pather.endcnt = 0;
	for (int a = 0; a <= demand.count; a++)
	{
		curPit = path1[demand.idNeed[a]] = &paths1[a];

		curPath.Clean();
		//curPath.Set(demand.idNeed[a], true);
		curPit->from = curPath.from = demand.idNeed[a];
		
		fastDFS(curPath.from);
		sort(curPit->paths, curPit->paths + curPit->cnt, Separator);

		for (int b = 0; b < curPit->cnt; b++)
		{
			if (curPit->paths[b].to == pmain.to)
			{
				curPit->hasEnd = true;
				//break;
			}
			else
			{
				curPit->endcnt = b;
				break;
			}
		}
		if (curPit->hasEnd == true)
		{
			if (a != demand.count)
				++pather.endcnt;
			else
				curPit->hasEnd = false;
		}
		//printf("pf%d:cnt%d,endcnt:%d,%d\n", a, curPit->cnt, curPit->endcnt, curPit->hasEnd);
		
		curPit->cnt = min(curPit->cnt, maxwidth);
	}
}


void Searcher::fastDFSless(PathFirst &pf)
{
	uint8_t a, enda;
	if (pather.cnt == demand.count)
	{
		a = 0, enda = pf.endcnt;
	}
	else
	{
		a = pf.endcnt, enda = pf.cnt;
	}
	while (a < enda)
	{
		PathData &p = pf.paths[a++];
		/*_mm_prefetch((char*)&p, _MM_HINT_NTA);
		_mm_prefetch((char*)(&p) + 64, _MM_HINT_NTA);*/
		
		if (pather.lastCost <= p.cost)//cost too much
			break;//according to order, later ones cost more
		if (!pather.pmap[pather.cnt].Test(p.pmap))//has overlap points
			continue;

		if (p.isEnd == 0x7f)
		{
			/*if (pather.cnt < demand.count)//can't go to dest now
			{
				printf("should not happen!\n");
				continue;
			}*/
			//final step,find a shorter route
			pather.minCost = pather.curCost + p.cost;
			pather.lastCost = p.cost;//refresh lastCost
			pather.pstack[pather.cnt++] = &p;

			FormRes();
			pather.cnt--;//rollback go though
			break;//according to order, later ones cost more
		}
		else//reach next point
		{
			/*if (pather.cnt == demand.count)//should go to dest now
			{
				printf("should not happen!\n");
				continue;
			}*/
			PathFirst &npf = *path1[p.to];
			if (npf.hasEnd)
			{
				if (pather.endcnt == 1 && pather.cnt + 1 < demand.count)//no way to dest now
					continue;
				pather.endcnt--;
			}
			pather.curCost += p.cost;//add cost
			pather.pmap[pather.cnt + 1].Merge(pather.pmap[pather.cnt], p.pmap);
			pather.pstack[pather.cnt++] = &p;//add go though
			pather.lastCost -= p.cost;
#ifndef FIN
			
			/*if (pather.cnt < demand.count - 13)
				printf("@@@ %d here at %dth when %lld\n", pather.cnt, a, Util::GetElapse());*/
			
#endif
			/*_mm_prefetch((char*)&npf.paths, _MM_HINT_NTA);
			_mm_prefetch((char*)(&npf.paths) + 64, _MM_HINT_NTA);*/
			fastDFSless(npf);

			pather.lastCost += p.cost;//rollback lastCost
			pather.cnt--;//rollback go though
			pather.curCost -= p.cost;//rollback cost

			if (npf.hasEnd)
				pather.endcnt++;
			continue;
		}
	}
	//finish this point
	
	return;
}

void Searcher::FormRes()
{
	ResData res = ResData();
	for (int a = 0; a < pather.cnt; a++)
	{
		PathData &p = *pather.pstack[a];
		for (int b = 0; b < p.cnt; b++)
		{
			res.idLink[res.count++] = p.mid[b];
		}
	}
	res.cost = pather.minCost;
	Util::WriteFile(&res);
}

void Searcher::StepLess()
{
	pather.cnt = 0;
	fastDFSless(*path1[pmain.from]);
}
