#include "rely.h"

#include "util.h"

using namespace std::chrono;

char Util::outfname[256];
uint64_t Util::t_begin;
TOPOData Util::topo[4800];
void Util::Init(const char * fname)
{
	t_begin = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	strcpy(outfname, fname);
	WriteFile();
}


uint64_t Util::GetElapse()
{
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - t_begin;
}

int16_t Util::ReadFile(const char* fname, PointData * points, uint16_t &maxid)
{
	FILE * fp = fopen(fname, "r");
	if (fp == NULL)
		return -1;

	int16_t cnt = 0;
	int16_t idLink, idSrc, idDest, cost;
	//for (; fscanf(fp, "%hd,%hd,%hd,%hd", &idLink, &idDest, &idSrc, &cost) != EOF; )
	for (; fscanf(fp, "%hd,%hd,%hd,%hd", &idLink, &idSrc, &idDest, &cost) != EOF; )
	{
		topo[idLink].idSrc = idSrc, topo[idLink].idDest = idDest, topo[idLink].cost = cost;
		PointData &p = points[idSrc];
		p.id = idSrc;
		for (int a = 0; a < p.cnt; a++)
			if(p.out[a].dest == idDest)
			{
				if (cost < p.out[a].dis)//fewer cost link
				{
					p.out[a].dis = cost;
					p.out[a].rid = idLink;
				}
				goto END_FIND;
			}
		maxid = max(maxid, idDest);
		maxid = max(maxid, idSrc);
		p.out[p.cnt].dest = idDest;
		p.out[p.cnt].dis = cost;
		p.out[p.cnt++].rid = idLink;
	END_FIND:
		cnt++;
	}
	fclose(fp);
	return cnt;
}

int16_t Util::ReadFile(const char* fname, DemandData &dmd)
{
	FILE * fp = fopen(fname, "r");
	if (fp == NULL)
		return -1;

	int16_t cnt = 0;
	memset(&dmd, 0, sizeof(dmd));
	char str[512];
	//fscanf(fp, "%hd,%hd,", &dmd.idTo, &dmd.idFrom);
	fscanf(fp, "%hd,%hd,", &dmd.idFrom, &dmd.idTo);
	fscanf(fp, "%s", str);
	uint16_t pid = 0;
	for (int a = 0; a < strlen(str); a++)
	{
		if (str[a] != '|')
			pid = pid * 10 + str[a] - '0';
		else
		{
			dmd.map[pid] = cnt;
			dmd.idNeed[cnt++] = pid;
			pid = 0;
		}
	}
	dmd.map[pid] = cnt;
	dmd.idNeed[cnt] = pid;
	dmd.map[dmd.idTo] = dmd.count = ++cnt;
	fclose(fp);
	return cnt;
}

int16_t Util::WriteFile(const ResData * path)
{
	FILE * fp = fopen(outfname, "w");
	if (fp == NULL)
	{
		printf("\t####\tError: open %s\n", outfname);
		return -1;
	}
	if (path == nullptr)
		fprintf(fp, "NA\n");
	else
	{
		printf("write ans cost %3d with %3d links at %lldms\n", path->cost, path->count, GetElapse());
		/*for (int a = path->count; a-- > 0;)
			fprintf(fp, a == 0? "%d\n" : "%d|", path->idLink[a]);*/
		for (int a = 0; a < path->count; a++)
			fprintf(fp, path->count - a == 1 ? "%d\n" : "%d|", path->idLink[a]);
	}
	fclose(fp);
	return 0;
}
