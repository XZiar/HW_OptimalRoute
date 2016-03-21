#include "rely.h"
#include "util.h"
#include "Searcher.h"
#include <string>


static Searcher searcher;

int main(int argc, char *argv[])
{
	bool isDebug = false,
		isStay = false,
		isMore = false;
	string fn_topo(argv[1]), fn_dmd(argv[2]), fn_out(argv[3]);
#ifndef FIN
	for (int a = 4; a < argc; a++)
	{
		if (strcmp(argv[a], "debug") == 0)
			isDebug = true;
		else if (strcmp(argv[a], "chk") == 0)
			Util::isChk = true;
		else if (strcmp(argv[a], "stay") == 0)
			isStay = true;
		else if (strcmp(argv[a], "more") == 0)
			isMore = true;
		else
		{
			fn_topo = argv[a] + fn_topo;
			fn_dmd = argv[a] + fn_dmd;
			fn_out = argv[a] + fn_out;
		}
	}
#endif
	const char * fn1 = fn_topo.c_str(),
		* fn2 = fn_dmd.c_str(),
		* fn3 = fn_out.c_str();


	Util::Init(fn3);

	printf("Open topo File : %s\n", fn1);
	int16_t linknum = Util::ReadFile(fn1, searcher.points);
	if(linknum == -1)
		printf("\t####\tError: open %s\n", fn1);
	else
		printf("get %d link from topo\n", linknum);

	printf("Open topo File : %s\n", fn2);
	int16_t dmdnum = Util::ReadFile(fn2, searcher.demand);
	if (dmdnum == -1)
		printf("\t####\tError: open %s\n", fn2);
	else
		printf("get %d need from demand\n", dmdnum);

	if (isDebug)
	{
		printf("From %3d to %3d\nNeed:\t", searcher.demand.idFrom, searcher.demand.idTo);
		for (int a = 0; a < dmdnum; ++a)
			printf(a == dmdnum - 1 ? "%3d\n" : "%3d  ", searcher.demand.idNeed[a]);
	}

	printf("Loading Cost: %lld ms\n", Util::GetElapse());

	searcher.Init();
	uint16_t width = dmdnum * 8 / 5;
	printf("Try depth %d and width %d\n", 16, width);
	searcher.Step1(16, width);
	printf("First Cost: %lld ms\n", Util::GetElapse());

	if (isDebug)
	{
		for (int a = 0; a <= dmdnum; a++)
		{
			uint16_t idFrom = searcher.demand.idNeed[a];
			Searcher::PathFirst &pf = *searcher.path1[idFrom];
			printf("%3d : %d path, %s\n", idFrom, pf.cnt, pf.hasEnd ? "hasEnd" : "noEnd");
			for (int b = 0; b < pf.cnt; b++)
			{
				PathData &p = pf.paths[b];
				printf("\t%3d(%3d):", p.to, p.cost);
				for (int c = 0; c < p.cnt; c++)
					printf(" %4d", p.mid[c]);
				printf("\n");
			}
		}
	}
	if(!isMore)
		searcher.StepLess();
	else
	{
		width = dmdnum * dmdnum * 2;
		width = min(width, 5000);
		printf("Try width %d\n", width);
		searcher.Step2(2, width);
		printf("Second Cost: %lld ms\n", Util::GetElapse());

		searcher.StepMore();
	}

	printf("Totol: %lld ms\n", Util::GetElapse());
	if(isStay)
		getchar();
	return 0;
}