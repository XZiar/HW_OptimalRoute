#include "rely.h"

#include "util.h"

using namespace std::chrono;

char Util::outfname[256];
uint64_t Util::t_begin;
TOPOData Util::topo[4800];
bool Util::isChk = false;
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

int16_t Util::ReadFile(const char* fname, PointData * points)
{
	FILE * fp = fopen(fname, "r");
	if (fp == NULL)
		return -1;

	int cnt = 0;
	int idLink, idSrc, idDest, cost;
	while (fscanf(fp, "%d,%d,%d,%d", &idLink, &idSrc, &idDest, &cost) != EOF)
	{
		topo[cnt].idLink = idLink, topo[cnt].idSrc = idSrc, topo[cnt].idDest = idDest, topo[cnt].cost = cost;
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

	int cnt = 0;
	memset(&dmd, 0, sizeof(dmd));
	char str[512];
	fscanf(fp, "%d,%d,", &dmd.idFrom, &dmd.idTo);
	fscanf(fp, "%s", str);
	for (int a = 0; a < strlen(str);a++)
	{
		if (str[a] != '|')
			dmd.idNeed[cnt] = dmd.idNeed[cnt] * 10 + str[a] - '0';
		else
			cnt++;
	}
	dmd.count = ++cnt;
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
		for (int a = 0; a < path->count; a++)
			fprintf(fp, path->count - a == 1 ? "%d\n" : "%d|", path->idLink[a]);
		if (isChk)
		{
			//check answer

		}
	}
	fclose(fp);
	return 0;
}
