#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "limits.h"
#include "assert.h"
#include "memory.h"
#include "PolygonCompare.h"
#include "PolygonSvg.h"
#include "MyDynamicMemory.h"

void SvgPolygonSet(PolygonStruct *polyStrArg, int deapthArg, int argcArg, char **&argvArg) {
	FILE *out = (FILE *)argvArg[0];
	char *a0 = argvArg[0];

	fputs("        <!-- Polyline -->\n", out);
	char *lineBgn = argvArg[3], *lineEnd = argvArg[4];
	fputs(lineBgn, out);
	for (int i = 0; i < polyStrArg->n; i++) {
		fprintf(out, "%s%d,%d ", i ? "L" : "M", polyStrArg->point[i].x, polyStrArg->point[i].y);
	}
	fputs(lineEnd, out);
}

void SvgPolylineArrayFree(Coord **polyline, int *nPolylinePoints, int nPoly) {
	for (int i = 0; i < nPoly; i++)
		MyFree(polyline[i]);
	MyFree(polyline);
	MyFree(nPolylinePoints);
}

int SvgReadPolylines(Coord **&polylineA, int *&nPointsA, int &nPolylinesA, Coord **&polylineB, int *&nPointsB, int &nPolylinesB, char *argv[5]) {
	FILE *svg = (FILE *)argv[0];
	char line[10000], color[2][7], *svgHeader = (char *)MyMalloc(9, 1), *svgTrailer, *lineBgn = NULL, *lineEnd, *context;
	int svgHeaderLen = 0, nColors = 0, iPoly, iPoint, nPoints, nPoly[2] = { 0, 0 }, *nPts[2] = { NULL, NULL };
	Coord **polyline[2];

	polyline[0] = polyline[1] = NULL;
	svgHeader[0] = color[0][0] = color[1][0] = '\0';

	while (1) {
		if (fgets(line, sizeof(line), svg) == NULL)
			return 1;
		if (strstr(line, "<!-- Polyline -->"))
			break;
		svgHeaderLen += strlen(line);
		svgHeader = (char *)MyRealloc(18, (void *)svgHeader, svgHeaderLen + 1);
		strcat_s(svgHeader, svgHeaderLen + 1, line);
	}
	while (1) {
		if (fgets(line, sizeof(line), svg) == NULL)
			return 1;
		char *colorLoc = strchr(line, '#') + 1, *z;

		if (lineBgn == NULL) {
			// lineBgn = _strdup(line);
			lineBgn = (char *)MyRealloc(21, NULL, strlen(line) + 1);
			strcpy_s(lineBgn, strlen(line) + 1, line);
			lineBgn = strtok_s(lineBgn, "M", &context);
			// lineEnd = _strdup(line);
			// lineEnd = strchr(lineEnd, 'Z');
			z = strchr(line, 'Z');
			lineEnd = (char *)MyRealloc(22, NULL, strlen(z) + 1);
			strcpy_s(lineEnd, strlen(z) + 1, z);
		}
		*(colorLoc + 6) = '\0';
		if (strcmp(colorLoc, color[0]) == 0)
			iPoly = 0;
		else if (strcmp(colorLoc, color[1]) == 0)
			iPoly = 1;
		else if (nColors == 2)
			return 2;
		else if (nColors == 0) {
			strcpy_s(color[0], sizeof(color[0]), colorLoc);
			iPoly = 0;
			nColors = 1;
		}
		else {
			strcpy_s(color[1], sizeof(color[1]), colorLoc);
			iPoly = 1;
			nColors = 2;
		}

		char *c = strchr(line, 'M');
		for (nPoints = 0, c++; *c != '\0'; c++) {
			if (*c == ',')
				nPoints++;
			else if (*c == 'Z')
				break;
		}
		Coord *coord = (Coord *)MyMalloc(10, nPoints*sizeof(Coord));
		char *scan = strtok_s(line, "M", &context);	// <path...
		for (iPoint = 0; iPoint < nPoints; iPoint++) {
			char *x = strtok_s(NULL, "L, ", &context);
			char *y = strtok_s(NULL, "L, ", &context);
			coord[iPoint].x = atoi(x);
			coord[iPoint].y = atoi(y);
		}
		//		ps[iPoly] = (PolygonStruct **)MyRealloc(ps[iPoly], (nPoly[iPoly] + 1)*sizeof(PolygonStruct));
		//		ps[iPoly][nPoly[iPoly]] = PolygonNew(NULL, NULL, NULL, NULL, CoordToPoint(coord, nPoints), nPoints);
		polyline[iPoly] = (Coord **)MyRealloc(19, polyline[iPoly], (nPoly[iPoly] + 1)*sizeof(Coord *));
		nPts[iPoly] = (int *)MyRealloc(20, nPts[iPoly], (nPoly[iPoly] + 1)*sizeof(int *));
		polyline[iPoly][nPoly[iPoly]] = coord;
		nPts[iPoly][nPoly[iPoly]] = nPoints;
		nPoly[iPoly]++;
		if (fgets(line, sizeof(line), svg) == NULL)
			return 1;
		if (strstr(line, "<!-- Polyline -->") == NULL)
			break;
	}
	// svgTrailer = _strdup(line);
	svgTrailer = (char *)MyRealloc(23, NULL, strlen(line) + 1);
	strcpy_s(svgTrailer, strlen(line) + 1, line);
	while (1) {
		if (fgets(line, sizeof(line), svg) == NULL)
			break;
		svgTrailer = (char *)MyRealloc(24, svgTrailer, strlen(svgTrailer) + strlen(line) + 1);
		strcat_s(svgTrailer, strlen(svgTrailer) + strlen(line) + 1, line);
	}
	polylineA = polyline[0]; nPointsA = nPts[0]; nPolylinesA = nPoly[0];
	polylineB = polyline[1]; nPointsB = nPts[1]; nPolylinesB = nPoly[1];
	argv[1] = svgHeader; argv[2] = svgTrailer; argv[3] = lineBgn; argv[4] = lineEnd;
	return 0;
}

int SvgSave(PolygonStruct *path, char *label, char *inFile, char *argvs[]) {
	char *dot, name[_MAX_PATH], tag[_MAX_PATH];
	FILE *out;

	sprintf_s(tag, sizeof(tag), "_%s.svg", label);
	strcpy_s(name, sizeof(name), inFile);
	dot = strrchr(name, '.');
	strcpy_s(dot, sizeof(name)-strlen(name) + strlen(dot), tag);
	errno_t err = fopen_s(&out, name, "w");
	argvs[0] = (char *)out;
	fputs(argvs[1], out);		// svgHeader
	PolygonTreeTraverse(path, 1, SvgPolygonSet, 5, (char **&)argvs);
	fputs(argvs[2], out);		// svgTrailer
	fclose(out);
	return 0;
}
