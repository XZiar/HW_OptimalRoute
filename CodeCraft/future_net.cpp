#include "rely.h"
#include "util.h"
#include "Searcher.h"
#include <string>
#include <thread>


static Searcher searcher;

int main(int argc, char *argv[])
{
	bool isDebug = false,
		isStay = false;
	string fn_topo(argv[1]), fn_dmd(argv[2]), fn_out(argv[3]);
#ifndef FIN
	for (int a = 4; a < argc; a++)
	{
		if (strcmp(argv[a], "debug") == 0)
			isDebug = true;
		else if (strcmp(argv[a], "stay") == 0)
			isStay = true;
		else if (strcmp(argv[a], "timer1") == 0)
		{
			thread([&]()
			{
				uint64_t last = 0;
				while (true)
				{
					this_thread::sleep_for(chrono::milliseconds(1000));
					uint64_t cur = searcher.loopcount;
					printf("loop %9lld at %lldms\n", cur - last, Util::GetElapse());
					last = cur;
				}
			}).detach();
		}
		else if (strcmp(argv[a], "timer2") == 0)
		{
			thread([&]()
			{
				uint32_t last = 0;
				while (true)
				{
					this_thread::sleep_for(chrono::milliseconds(1000));
					uint32_t cur = searcher.anscnt;
					printf("ans get %2d of %4d at %lldms\n", cur - last, cur, Util::GetElapse());
					last = cur;
				}
			}).detach();
		}
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

	if (!isStay && dmdnum >= 38)
	{
		thread thr = thread([]()
		{
			this_thread::sleep_for(chrono::milliseconds(9800));
			exit(0);
		});
		thr.detach();
	}
	
	printf("Loading Cost: %lld ms\n", Util::GetElapse());

	searcher.Init();

	uint16_t width, depth = 16;
	{
		uint16_t w1, w2;
		w1 = sqrt(linknum * dmdnum) / 2.56;
		w2 = dmdnum * 4.5;
		width = max(w1, w2);
		width = min(width, 164);
	}
	printf("Try depth %d and width %d\n", depth, width);

	searcher.Step1(depth, width);

	printf("First Cost: %lld ms\n", Util::GetElapse());
	//return 0;
#ifndef FIN
	if (isDebug)
	{
		FILE * fp = fopen("log.txt", "w");
		for (int a = 0; a <= dmdnum; a++)
		{
			uint16_t idFrom = searcher.demand.idNeed[a];
			Searcher::PathFirst &pf = *searcher.path1[idFrom];
			fprintf(fp, "%3d : %d path, %s\n", idFrom, pf.cnt, pf.hasEnd ? "hasEnd" : "noEnd");
			for (int b = 0; b < pf.cnt; b++)
			{
				PathData &p = pf.paths[b];
				fprintf(fp, "\t%3d(%3d):", p.to, p.cost);
				for (int c = 0; c < p.cnt; c++)
					fprintf(fp, " %4d", p.mid[c]);
				fprintf(fp, "\n");
			}
		}
		fclose(fp);
		getchar();
		return 0;
	}

#endif


	searcher.StepLess();

	printf("Totol: %lld ms\n", Util::GetElapse());
	if(isStay)
		getchar();
	return 0;
}
