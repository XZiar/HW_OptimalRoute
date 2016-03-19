#include "rely.h"
#include "Searcher.h"

const uint8_t PMap::mask[8] = { 0x80,0x40,0x20,0x10,0x8,0x4,0x2,0x1 };
PMap::PMap()
{
	memset(datB, 0, sizeof(datB));
}
PMap::PMap(const PMap & ori)
{
	memcpy(datB, ori.datB, sizeof(datB));
}
void PMap::Merge(const PMap & left, const PMap & right)
{
#if defined(SSE)
	datSSE[0] = _mm_or_si128(left.datSSE[0], right.datSSE[0]);
	datSSE[1] = _mm_or_si128(left.datSSE[1], right.datSSE[1]);
	datSSE[2] = _mm_or_si128(left.datSSE[2], right.datSSE[2]);
	datSSE[3] = _mm_or_si128(left.datSSE[3], right.datSSE[3]);
	datSSE[4] = _mm_or_si128(left.datSSE[4], right.datSSE[4]);
#else
	for (int a = 0; a < 10; a++)
		datL[a] = left.datL[a] | right.datL[a];
#endif
}
bool PMap::Test(uint16_t id) const
{
	return datB[id >> 3] & mask[id & 0x7];
}
bool PMap::Test(const PMap & right) const
{
#if defined(SSE)
	__m128i p0 = _mm_or_si128(_mm_and_si128(datSSE[0], right.datSSE[0]), _mm_and_si128(datSSE[1], right.datSSE[1])),
		p1 = _mm_or_si128(_mm_and_si128(datSSE[2], right.datSSE[2]), _mm_and_si128(datSSE[3], right.datSSE[3])),
		t4 = _mm_and_si128(datSSE[4], right.datSSE[4]);
	__m128i t0 = _mm_or_si128(p0, t4);
		
	return (_mm_testz_si128(t0, t0) && _mm_testz_si128(p1, p1));//all zero
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
	memset(&pmap, 0, sizeof(pmap));
	cost = cnt = 0;
}

void PathData::Set(uint16_t id, bool type)
{
	if (type)
		pmap.datB[id >> 3] |= PMap::mask[id & 0x7];
	else
		pmap.datB[id >> 3] &= ~PMap::mask[id & 0x7];
}

bool PathData::Test(uint16_t id) const
{
	return pmap.Test(id);
}




Searcher::Searcher()
{
	//path1 = new PathFirst[600];
}


Searcher::~Searcher()
{
	//delete path1;
}


void Searcher::Init()
{
	pmain.from = demand.idFrom, pmain.to = demand.idTo;
	demand.idNeed[demand.count] = demand.idFrom;
	pmain.Set(demand.idTo, true);
	for (int a = 0; a < demand.count; a++)
		pmain.Set(demand.idNeed[a], true);
}

void Searcher::fastDFS(uint16_t curID)
{
	PointData &p = points[curID];
	for (uint8_t a = 0; a < p.cnt; a++)
	{
		if (curPit->cnt == maxwide && curPit->maxcost - curPath.cost < p.out[a].dis)//this is too long
			break;//according to order, later ones are much longer
		uint16_t &thisID = p.out[a].dest;
		if (curPath.Test(thisID))//already go through
			continue;
		if (pmain.Test(thisID))//reach need-point
		{
			curPath.cost += p.out[a].dis;//add cost
			curPath.weight = curPath.cost;
			//curPath.weight = curPath.cost * maxlevel / (curLoop * 1.0f);
			curPath.mid[curPath.cnt++] = p.out[a].rid;//add go though
			curPath.Set(thisID, true);//set bitmap
			curPath.to = thisID;//add destination
			
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

			curPath.Set(thisID, false);//clear bitmap
			curPath.cnt--;//rollback go though
			curPath.cost -= p.out[a].dis;//rollback cost
			continue;
		}
		//reach normal-point
		else if (curPath.cnt < maxlevel && thisID != pmain.from)//still can go deeper and not toward start-point
		{
			curPath.cost += p.out[a].dis;//add cost
			curPath.mid[curPath.cnt++] = p.out[a].rid;//add go though
			curPath.Set(thisID, true);//set bitmap
			
			fastDFS(thisID);//into next point

			curPath.Set(thisID, false);//clear bitmap
			curPath.cnt--;//rollback go though
			curPath.cost -= p.out[a].dis;//rollback cost
			continue;
		}
	}
	//finish this point
	return;
}

void Searcher::Step1(uint8_t maxdepth, uint8_t maxwidth)
{
	maxlevel = maxdepth;
	maxwide = maxwidth;
	path2.endcnt = 0;
	for (int a = 0; a <= demand.count; a++)
	{
		curPit = path1[demand.idNeed[a]] = &paths1[a];
		//curPit = &path1[demand.idNeed[a]];

		curPath.Clean();
		//curPath.Set(demand.idNeed[a], true);
		curPit->from = curPath.from = demand.idNeed[a];
		
		fastDFS(curPath.from);
		if (a != demand.count)// not for the start
			for (int b = 0; b < curPit->cnt; b++)
				if (curPit->paths[b].to == pmain.to)
				{
					curPit->hasEnd = true;
					++path2.endcnt;
					break;
				}
		sort(curPit->paths, curPit->paths + curPit->cnt);
	}
}


void Searcher::fastDFS2(PathFirst &pf)
{
	for (uint8_t a = 0; a < pf.cnt; a++)
	{
		PathData &p = pf.paths[a];
		if (path2.minCost - path2.curCost <= p.cost)//cost too much
			break;//according to order, later ones cost more
		uint16_t &thisID = p.to;
		if (!path2.pmap[path2.cnt].Test(p.pmap))//has overlap points
			continue;
		if (thisID == pmain.to)
		{
			if(path2.cnt < demand.count)//can't go to dest now
				continue;
			//final step,find a shorter route
			path2.minCost = path2.curCost + p.cost;
			path2.pstack[path2.cnt++] = &p;
			//printf("find result cost %3d at %lld\n", path2.curCost + p.cost, Util::GetElapse());
			//TODO:find new answer!
			FormRes();
			path2.cnt--;//rollback go though
			continue;
		}
		else//reach next point
		{
			PathFirst &npf = *path1[p.to];
			if (npf.hasEnd)
			{
				if (path2.endcnt == 1 && path2.cnt + 1 < demand.count)//can't go to dest now
					continue;
				path2.endcnt--;
			}
			path2.curCost += p.cost;//add cost
			path2.pmap[path2.cnt + 1].Merge(path2.pmap[path2.cnt], p.pmap);
			path2.pstack[path2.cnt++] = &p;//add go though

			/*if (path2.cnt < demand.count - 13)
				printf("@@@ %d here at %dth when %lld\n", path2.cnt, a, Util::GetElapse());*/

			fastDFS2(npf);

			path2.cnt--;//rollback go though
			path2.curCost -= p.cost;//rollback cost

			if (npf.hasEnd)
				path2.endcnt++;
			continue;
		}
	}
	//finish this point
	
	return;
}

void Searcher::FormRes()
{
	ResData res = ResData();
	for (int a = 0; a < path2.cnt; a++)
	{
		PathData &p = *path2.pstack[a];
		for (int b = 0; b < p.cnt; b++)
		{
			res.idLink[res.count++] = p.mid[b];
		}
	}
	res.cost = path2.minCost;
	Util::WriteFile(&res);
}

void Searcher::Step2()
{
	path2.cnt = 0;
	fastDFS2(*path1[pmain.from]);
}


