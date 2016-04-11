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
				uint64_t lastLP = 0, lastVT = 0;
				while (true)
				{
					this_thread::sleep_for(chrono::milliseconds(1000));
					uint64_t cur = searcher.loopcount;
					uint64_t curt = Util::GetElapse();
					printf("loop %5lldK at %4llds,avg:%5lldK/s\n", (cur - lastLP) / 1000, curt / 1000, cur / curt);
					lastLP = cur;
					cur = searcher.VTestCnt;
					printf("test %5lldM at %4llds,avg:%5lldM/s\n", (cur - lastVT) / 1000000, curt / 1000, cur / curt / 1000);
					lastVT = cur;
				}
			}).detach();
		}
		else if (strcmp(argv[a], "timer2") == 0)
		{
			thread([&]()
			{
				uint64_t lastLP = 0, lastVT = 0;
				while (true)
				{
					this_thread::sleep_for(chrono::milliseconds(1000));
					uint64_t cur = searcher.actCut;
					uint64_t curt = Util::GetElapse();
					printf("actCut %5lldK at %4llds,avg:%5lldK/s\n", (cur - lastLP) / 1000, curt / 1000, cur / curt);
					lastLP = cur;
					cur = searcher.estCut;
					printf("estCut %5lldK at %4llds,avg:%5lldK/s\n", (cur - lastVT) / 1000, curt / 1000, cur / curt);
					lastVT = cur;
				}
			}).detach();
		}
		else if (strcmp(argv[a] + 2, "lvcnt") == 0)
		{
			uint8_t lv = (argv[a][0] - '0') * 10 + (argv[a][1] - '0');
			thread([&]()
			{
				uint64_t cur[4];
				char str[255];
				sprintf(str, "[%2d]%%5lld,[%2d]%%5lld,[%2d]%%5lld,[%2d]%%5lld at %%4llds\n", lv - 1, lv, lv + 1, lv + 2);
				while (true)
				{
					this_thread::sleep_for(chrono::milliseconds(1000));
					cur[0] = searcher.loopLVcnt[lv - 1];
					cur[1] = searcher.loopLVcnt[lv];
					cur[2] = searcher.loopLVcnt[lv + 1];
					cur[3] = searcher.loopLVcnt[lv + 2];
					uint64_t curt = Util::GetElapse();
					printf(str, cur[0], cur[1], cur[2], cur[3], curt / 1000);
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
	uint16_t maxid = 0;

	printf("Open topo File : %s\n", fn1);
	int16_t linknum = Util::ReadFile(fn1, searcher.points, maxid);
	if(linknum == -1)
		printf("\t####\tError: open %s\n", fn1);
	else
		printf("get %hd link from topo, maxid id %hd\n", linknum, maxid);

	printf("Open topo File : %s\n", fn2);
	int16_t dmdnum = Util::ReadFile(fn2, searcher.demand);
	if (dmdnum == -1)
		printf("\t####\tError: open %s\n", fn2);
	else
		printf("get %hd need from demand\n", dmdnum);
#ifndef FIN
	if (!isStay)
	{
		thread thr = thread([]()
		{
			this_thread::sleep_for(chrono::milliseconds(9800));
			exit(0);
		});
		thr.detach();
	}
#endif
	printf("Loading Cost: %lld ms\n", Util::GetElapse());

	searcher.Init();

	uint16_t width, depth = 11;
	{
		uint16_t w1, w2, w3;
		w1 = uint16_t(sqrt(linknum * dmdnum * 1.0) / 2.4);
		w2 = uint16_t(dmdnum * 4.8);
		width = max(w1, w2);
		width = min(width, 120);
	}
	printf("Try depth %hd and width %hd\n", depth, width);

	searcher.Step1(depth, width);

	printf("First Cost: %lld ms\n", Util::GetElapse());

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
				{
					uint16_t rid = p.mid[c];
					fprintf(fp, " %4d", Util::topo[rid].idDest);
				}
				fprintf(fp, "\n");
			}
		}
		fclose(fp);
		getchar();
		return 0;
	}

#endif


	searcher.StepEnd(maxid);
	printf("Totol: %lld ms\n", Util::GetElapse());
	if(isStay)
		getchar();
	return 0;
}
