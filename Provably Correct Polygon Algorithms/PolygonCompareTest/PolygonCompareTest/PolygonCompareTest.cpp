#include "stdafx.h"

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#include "stdio.h"
#include "memory.h"
#include "malloc.h"
#include "string.h"
#define STATIC
#include "PolygonCompare.h"
#include "PolygonSvg.h"
#include "MyDynamicMemory.h"

int main(int argc, char* argv[]) {
	FILE *svg, *log;
	char *argvs[5];
	Coord **polylineA, **polylineB;
	int i, j, flags, nPolyA, nPolyB, *nPolylinePointsA, *nPolylinePointsB;
	PolygonStruct *psA, *psB, *psAmnsB, *psBmnsA, *psAandB, *psAplsB;
	WingedEdgeStruct *wE;

	fopen_s(&svg, argv[1], "r");
	char *dot = strrchr(argv[1], '.');
	strcpy_s(dot, strlen(dot)+1, ".log");
	fopen_s(&log, argv[1], "w");
	flags = CLEAN_REDUNDNT | CLEAN_COLINEAR | CLEAN_FIGURE_8 | CLEAN_DEGENERT;

	// read in polylines for each polygon (in this case they may be intermixed; sort by color.)
	argvs[0] = (char *) svg;
	SvgReadPolylines(polylineA, nPolylinePointsA, nPolyA, polylineB, nPolylinePointsB, nPolyB, argvs);

	// clean ploygons
	nPolyA = GeometryClean(polylineA, nPolylinePointsA, nPolyA, flags);
	nPolyB = GeometryClean(polylineB, nPolylinePointsB, nPolyB, flags);
	// make nested polygon structures from polylines
	psA = PolygonTreeFromCoords(polylineA, nPolylinePointsA, nPolyA);
	psB = PolygonTreeFromCoords(polylineB, nPolylinePointsB, nPolyB);
	// compute "winged edge" parameters for each intersection
	wE = PolygonCompareGenWingedEdges(psA, "A", psB, "B", log);
	// compute the operands
	psAplsB = PolygonTreeOpUnion(psA, psB, wE, log, "A+B");
	psAandB = PolygonTreeOpInter(psA, psB, wE, log, "A&B");
	psAmnsB = PolygonTreeOpMinus(psA, psB, wE, log, "A-B");
	psBmnsA = PolygonTreeOpMinus(psB, psA, wE, log, "B-A");
	// save output
	for (char *clr = strchr(argvs[4], '#') + 1; *clr != ';'; clr++)
		*clr = '0';	// change output color in the header
	SvgSave(psAplsB, "A+B", argv[1], argvs);
	SvgSave(psAandB, "A&B", argv[1], argvs);
	SvgSave(psAmnsB, "A-B", argv[1], argvs);
	SvgSave(psBmnsA, "B-A", argv[1], argvs);
	fclose(log);

	// free
#ifdef FREE
	char **b = NULL;
	MyDynamicMemoryCheck(1);
	for (i = 1; i < 5; i++)
		MyFree(argvs[i]);
	SvgPolylineArrayFree(polylineA, nPolylinePointsA, nPolyA);
	SvgPolylineArrayFree(polylineB, nPolylinePointsB, nPolyB);
	WingEdgeFree(wE);
	// free polygon stack
	PolygonPop(1);
	// free trees
	PolygonTreeTraverse(psA, 0, PolygonFreeStrAndPoints, 0, b);
	PolygonTreeTraverse(psB, 0, PolygonFreeStrAndPoints, 0, b);
	PolygonTreeTraverse(psAmnsB, 0, PolygonFreeStrAndPoints, 0, b);
	PolygonTreeTraverse(psBmnsA, 0, PolygonFreeStrAndPoints, 0, b);
	PolygonTreeTraverse(psAandB, 0, PolygonFreeStrAndPoints, 0, b);
	PolygonTreeTraverse(psAplsB, 0, PolygonFreeStrAndPoints, 0, b);
	MyDynamicMemoryCheck(0);
	MyDynamicMemoryCheck(-1);
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

	// getchar();
	return 0;
}
