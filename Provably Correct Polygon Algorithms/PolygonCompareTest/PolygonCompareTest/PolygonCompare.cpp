#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "limits.h"
#include "assert.h"
#include "memory.h"
#include "PolygonCompare.h"
#include "MyDynamicMemory.h"

static PolygonStruct **stack = NULL;
static int nStack = 0, nStackMax = 0;
static FILE *LOG = stdout;
static WingedEdgeStruct *currentWE;

#define WEPOINT(we,i,bgnend,k) we->intrRay[i].polyStr->point[(we->intrRay[i].bgnend+k)%(we->intrRay[i].polyStr->n)]

void AddCertainMidPointsProc(PolygonStruct *A, int deapth, int argcArg, char **&argvArg) {
	for (int im1 = A->n - 1, i = 0; i<A->n; i++) {
		if (A->point[im1].iWE == -2 && A->point[i].iWE == -2) {
			short x0 = A->point[im1].x, x1 = A->point[i].x, y0 = A->point[im1].y, y1 = A->point[i].y, x = (x0 + x1) / 2, y = (y0 + y1) / 2;
			Point p = { x, y, 0, -1 };

			A->point = PolygonAddPointToPoly(i, p, A->point, A->n);
		}
		im1 = i;
	}
}

bool AnyPointMatches(PolygonStruct *A, PolygonStruct *B, int criteria) {
	for (int i = 0; i<A->n; i++)
	if ( PolygonWherePoint(B->point, B->n, A->point[i])&criteria)
		return true;
	return false;
}

Point *CoordToPoint(Coord *coord, int n) {
	Point *point = (Point *)MyMalloc(1, sizeof(Point)*n);

	for (int i = 0; i<n; i++) {
		point[i].x = coord[i].x * 2;
		point[i].y = coord[i].y * 2;
		point[i].track = 0;
		point[i].iWE = -1;
	}
	return point;
}

Point CoordToPoint1(Coord coord) {
	Point point = { coord.x, coord.y, 0, -1 };
	return point;
}

inline int FindWing(PolygonStruct *which, WingedEdgeStruct *wE, int j, int inc) {
	// find which wing stored in the winged edge structure matches that matches the candidate
	int l, iEnd = (j - inc + which->n) % which->n;

	for (l = 0; l<wE->nRay; l++)
	if (wE->intrRay[l].polyStr == which && wE->intrRay[l].iEnd == iEnd)
		break;
	return l;
}

int FindWingInOrder(int l, int *order) {
	// find which element in the order array matches the candidate
	for (int i = 0; true; i++)
	if (order[i] == l)
		return i;
}

PolygonStruct *FollowEdge(int i, PolygonStruct *pA, PolygonStruct *pB, PathSide side, WingedEdgeStruct *wEArray) {
	// given the index of a point in pA, follow along the polygon edge segments until you reach an intersection.
	// then use the winged edge structure to find which way to go. if the path is to be inside, follow the ray that is
	// most CCW from where you came. that is, go to the left. if following the outside, follow the ray that is the least
	// CCW from where you came. that is, go right.
	PolygonStruct *which = pA, *pC = PolygonNew(NULL, NULL, NULL, NULL, NULL, 0);
	int j = i, k = 0, l, m, next, inc = 1;

	while (1) {
		pC->point = PolygonAddPointToPoly(k, which->point[j], pC->point, pC->n);
		which->point[j].track = 1;
		k++;
		if (which->point[j].iWE >= 0) {
			WingedEdgeStruct *wE = &wEArray[which->point[j].iWE];
			// int order[99]; // malloc and free really better
			wE->track = 1;
			l = FindWing(which, wE, j, inc);
			int *order = (int *) MyMalloc(2, wE->nRay*sizeof(int));
			WingEdgeInitOrder(order, wE);
			m = FindWingInOrder(l, order);
			next = side == PathTurnRgt ? order[(m + 1) % wE->nRay] : order[(m - 1 + wE->nRay) % wE->nRay];
			which = wE->intrRay[next].polyStr;
			j = wE->intrRay[next].iCrs;
			inc = wE->intrRay[next].iEnd - wE->intrRay[next].iCrs;
			MyFree(order);
			//which->point[j].track = 1;
		}
		j = (j + inc + which->n) % which->n;
		if (which->point[j].x == pC->point[0].x && which->point[j].y == pC->point[0].y)
			break;
	}
	return pC;
}

void GeometryBoundPolygon(Point *poly, int n) {
	int i;

	for (i = 0; i<n; i++) {
		poly[i].x = max(min(poly[i].x, SHRT_MAX), SHRT_MIN);
		poly[i].y = max(min(poly[i].y, SHRT_MAX), SHRT_MIN);
	}
}

int GeometryClean(Coord **&polyline, int *&nPoints, int nPolylines, int flags) {
	int i, j, k, jOld;
	TwoLinesRelationship tlr;
	Point crossPoint;
	bool figure8Change = false;

	for (i = 0; i < nPolylines; i++) {
		if (flags&CLEAN_REDUNDNT)
			nPoints[i] = GeometryUnredun(polyline[i], nPoints[i]);
		if (flags&CLEAN_COLINEAR)
			nPoints[i] = GeometryUncolinear(polyline[i], nPoints[i]);
		if (flags&CLEAN_FIGURE_8) {
			jOld = nPoints[i] - 1;
			for (j = 0; j < nPoints[i]; j++) {
				for (k = j + 1; k < nPoints[i] - 1; k++) {
					tlr = GeometryTwoLinesRelated(CoordToPoint1(polyline[i][jOld]), CoordToPoint1(polyline[i][j]), CoordToPoint1(polyline[i][k]), CoordToPoint1(polyline[i][k + 1]));
					switch (tlr) {
					case TLmis: case TL0tB: case TL0t2: case TL0t3: case TL1t3: case TLhuh: case TL3tA: case TLcol:
						break;
					case TLcrs:
						crossPoint = GeometryCrossPoint(CoordToPoint1(polyline[i][jOld]), CoordToPoint1(polyline[i][j]), CoordToPoint1(polyline[i][k]), CoordToPoint1(polyline[i][k+1]));
						polyline[i] = (Coord *)MyRealloc(11, polyline[i], (nPoints[i] + 1)*sizeof(Coord));
						memmove_s(&polyline[i][j + 1], (nPoints[i] - j)*sizeof(Coord), &polyline[i][j], (nPoints[i] - j)*sizeof(Coord));
						polyline[i][j].x = crossPoint.x;
						polyline[i][j].y = crossPoint.y;
						nPoints[i]++;
						k++;
						// fall through
					case TL2tA:
					case TL1tB:
					case TL1t2:
						// Coord *dbg = polyline[i];
						figure8Change = true;
						nPoints = (int *)MyRealloc(12, nPoints, (nPolylines + 1)*sizeof(int));
						nPoints[nPolylines] = tlr == TL1tB ? k - j : k - j + 1;
						polyline = (Coord **)MyRealloc(13, polyline, (nPolylines+1)*sizeof(Coord *));
						polyline[nPolylines] = (Coord *)MyMalloc(3, nPoints[nPolylines] * sizeof(Coord));
						memcpy_s(polyline[nPolylines], nPoints[nPolylines] * sizeof(Coord), &polyline[i][j], nPoints[nPolylines] * sizeof(Coord));
						nPolylines++;
						if (tlr == TL2tA) {
							memmove_s(&polyline[i][j], nPoints[i] * sizeof(Point), &polyline[i][k], (nPoints[i] - k)*sizeof(Point));
							nPoints[i] = j + nPoints[i] - k;
						}
						else {
							memmove_s(&polyline[i][j + 1], (nPoints[i] - 1 - (j + 1)) * sizeof(Point), &polyline[i][k + 1], (nPoints[i] - (k + 1))*sizeof(Point));
							nPoints[i] = j + nPoints[i] - k;
						}
						k++;
					}
				}
				jOld = j;
			}
			if (figure8Change) {
				if (flags&CLEAN_REDUNDNT)
					nPoints[i] = GeometryUnredun(polyline[i], nPoints[i]);
				if (flags&CLEAN_COLINEAR)
					nPoints[i] = GeometryUncolinear(polyline[i], nPoints[i]);
			}
		}
		if (flags&CLEAN_DEGENERT) {
			if (nPoints[i] < 3) {
				memmove_s(&nPoints[i], (nPolylines - i - 1)*sizeof(int), &nPoints[i + 1], (nPolylines - i - 1)*sizeof(int));
				memmove_s(&polyline[i], (nPolylines - i - 1)*sizeof(Coord *), &nPoints[i + 1], (nPolylines - i - 1)*sizeof(Coord *));
				nPolylines--;
				i--;
			}
		}
	}
	return nPolylines;
}

int GeometryCompareRays(const void *elem1, const void *elem2) {
	// called by qsort; compare 2 rays eminating from a point to a reference ray; which is less CCW?
	// we have access to currentWE, a pointer to the current winged edge structure describing the intersection.
	// elem1 is an index specifying the first ray. for n rays, one is the reference ray and n-1 rays are sorted.
	// so 0<=elem1<n-1. our reference ray is at [0], so we add 1 to elem1 to find the index in the currentWE array.
	// if we find two rays pointing in the same direction, we have common edges of both polygons. then we have to
	// follow the common edge segments until they diverge and return the value as ig the last common edge were
	// the reference ray and the diverging edges were what's referenced by elem1 and elem2.
	int i = *((int *)elem1) + 1;
	int j = *((int *)elem2) + 1;
	Point p0 = WEPOINT(currentWE, 0, iCrs, 0);
	Point p1 = WEPOINT(currentWE, 0, iEnd, 0);
	Point p2 = WEPOINT(currentWE, i, iEnd, 0);
	Point p3 = WEPOINT(currentWE, j, iEnd, 0);
	LessCCW lessCCW = GeometryLessCounterClockwise(p0, p1, p2, p3);

	switch (lessCCW) {
		case LessCCWtrue:
			return -1;
		case LessCCWcolinear: {
			int iDelta = currentWE->intrRay[i].iEnd - currentWE->intrRay[i].iCrs;
			int jDelta = currentWE->intrRay[j].iEnd - currentWE->intrRay[j].iCrs;
			int k;

			for (k = 1; lessCCW == LessCCWcolinear; k++) {
				p0 = WEPOINT(currentWE, i, iCrs, k*iDelta);
				p1 = WEPOINT(currentWE, i, iEnd, k*iDelta);
				p2 = WEPOINT(currentWE, i, iEnd, (k + 1)*iDelta);
				p3 = WEPOINT(currentWE, j, iEnd, (k + 1)*jDelta);
				lessCCW = GeometryLessCounterClockwise(p0, p1, p2, p3);
			}
			return lessCCW;
		}
		case LessCCWfalse:
			return 1;
		case LessCCWhuh:
			// error
			return 0;
	}
	return 0;
}

Point GeometryCrossPoint(Point p0, Point p1, Point p2, Point p3) {
	// compute the crossing point between line p0 to p1 and line p2 to p3
	// the 2 lines must be known to cross (so we don't check to prevent division by 0)
	// px=((x0*y1-y0*x1)*(x2-x3)-(x0-x1)*(x2*y3-y2*x3))/((x0-x1)*(y2-y3)-(y0-y1)*(x2-x3))
	// py=((x0*y1-y0*x1)*(y2-y3)-(y0-y1)*(x2*y3-y2*x3))/ditto
	long denom = ((long)p0.x - p1.x)*(p2.y - p3.y) - ((long)p0.y - p1.y)*(p2.x - p3.x);
	long term0 = (long)p0.x*p1.y - (long)p0.y*p1.x;
	long term1 = (long)p2.x*p3.y - (long)p2.y*p3.x;
	long px = (term0*(p2.x - p3.x) - (p0.x - p1.x)*term1) / denom;
	long py = (term0*(p2.y - p3.y) - (p0.y - p1.y)*term1) / denom;
	Point cross = { (short)px, (short)py, -1, 0 };

	return cross;
}

void GeometryInit() {
	twoLinesRel[PointRgt][PointRgt][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointLft][PointRgt][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointRgt][PointCtr][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointCtr][PointCtr][PointRgt][PointRgt] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointRgt][PointLft][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointCtr][PointLft][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointLft][PointLft][PointRgt][PointRgt] = TLmis;
	twoLinesRel[PointRgt][PointRgt][PointCtr][PointRgt] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointCtr][PointRgt] = TLhuh;
	twoLinesRel[PointLft][PointRgt][PointCtr][PointRgt] = TLhuh;
	twoLinesRel[PointRgt][PointCtr][PointCtr][PointRgt] = TL1t2;
	twoLinesRel[PointCtr][PointCtr][PointCtr][PointRgt] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointCtr][PointRgt] = TLhuh;
	twoLinesRel[PointRgt][PointLft][PointCtr][PointRgt] = TL2tA;
	twoLinesRel[PointCtr][PointLft][PointCtr][PointRgt] = TL0t2;
	twoLinesRel[PointLft][PointLft][PointCtr][PointRgt] = TLmis;
	twoLinesRel[PointRgt][PointRgt][PointLft][PointRgt] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointLft][PointRgt] = TLhuh;
	twoLinesRel[PointLft][PointRgt][PointLft][PointRgt] = TLcrs;
	twoLinesRel[PointRgt][PointCtr][PointLft][PointRgt] = TL1tB;
	twoLinesRel[PointCtr][PointCtr][PointLft][PointRgt] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointLft][PointRgt] = TLhuh;
	twoLinesRel[PointRgt][PointLft][PointLft][PointRgt] = TLcrs;
	twoLinesRel[PointCtr][PointLft][PointLft][PointRgt] = TL0tB;
	twoLinesRel[PointLft][PointLft][PointLft][PointRgt] = TLmis;
	twoLinesRel[PointRgt][PointRgt][PointRgt][PointCtr] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointRgt][PointCtr] = TL0t3;
	twoLinesRel[PointLft][PointRgt][PointRgt][PointCtr] = TL3tA;
	twoLinesRel[PointRgt][PointCtr][PointRgt][PointCtr] = TLhuh;
	twoLinesRel[PointCtr][PointCtr][PointRgt][PointCtr] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointRgt][PointCtr] = TL1t3;
	twoLinesRel[PointRgt][PointLft][PointRgt][PointCtr] = TLhuh;
	twoLinesRel[PointCtr][PointLft][PointRgt][PointCtr] = TLhuh;
	twoLinesRel[PointLft][PointLft][PointRgt][PointCtr] = TLmis;
	twoLinesRel[PointRgt][PointRgt][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointCtr][PointRgt][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointLft][PointRgt][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointRgt][PointCtr][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointCtr][PointCtr][PointCtr][PointCtr] = TLcol;
	twoLinesRel[PointLft][PointCtr][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointRgt][PointLft][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointCtr][PointLft][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointLft][PointLft][PointCtr][PointCtr] = TLhuh;
	twoLinesRel[PointRgt][PointRgt][PointLft][PointCtr] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointLft][PointCtr] = TLhuh;
	twoLinesRel[PointLft][PointRgt][PointLft][PointCtr] = TLhuh;
	twoLinesRel[PointRgt][PointCtr][PointLft][PointCtr] = TL1t3;
	twoLinesRel[PointCtr][PointCtr][PointLft][PointCtr] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointLft][PointCtr] = TLhuh;
	twoLinesRel[PointRgt][PointLft][PointLft][PointCtr] = TL3tA;
	twoLinesRel[PointCtr][PointLft][PointLft][PointCtr] = TL0t3;
	twoLinesRel[PointLft][PointLft][PointLft][PointCtr] = TLmis;
	twoLinesRel[PointRgt][PointRgt][PointRgt][PointLft] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointRgt][PointLft] = TL0tB;
	twoLinesRel[PointLft][PointRgt][PointRgt][PointLft] = TLcrs;
	twoLinesRel[PointRgt][PointCtr][PointRgt][PointLft] = TLhuh;
	twoLinesRel[PointCtr][PointCtr][PointRgt][PointLft] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointRgt][PointLft] = TL1tB;
	twoLinesRel[PointRgt][PointLft][PointRgt][PointLft] = TLcrs;
	twoLinesRel[PointCtr][PointLft][PointRgt][PointLft] = TLhuh;
	twoLinesRel[PointLft][PointLft][PointRgt][PointLft] = TLmis;
	twoLinesRel[PointRgt][PointRgt][PointCtr][PointLft] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointCtr][PointLft] = TL0t2;
	twoLinesRel[PointLft][PointRgt][PointCtr][PointLft] = TL2tA;
	twoLinesRel[PointRgt][PointCtr][PointCtr][PointLft] = TLhuh;
	twoLinesRel[PointCtr][PointCtr][PointCtr][PointLft] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointCtr][PointLft] = TL1t2;
	twoLinesRel[PointRgt][PointLft][PointCtr][PointLft] = TLhuh;
	twoLinesRel[PointCtr][PointLft][PointCtr][PointLft] = TLhuh;
	twoLinesRel[PointLft][PointLft][PointCtr][PointLft] = TLmis;
	twoLinesRel[PointRgt][PointRgt][PointLft][PointLft] = TLmis;
	twoLinesRel[PointCtr][PointRgt][PointLft][PointLft] = TLmis;
	twoLinesRel[PointLft][PointRgt][PointLft][PointLft] = TLmis;
	twoLinesRel[PointRgt][PointCtr][PointLft][PointLft] = TLmis;
	twoLinesRel[PointCtr][PointCtr][PointLft][PointLft] = TLhuh;
	twoLinesRel[PointLft][PointCtr][PointLft][PointLft] = TLmis;
	twoLinesRel[PointRgt][PointLft][PointLft][PointLft] = TLmis;
	twoLinesRel[PointCtr][PointLft][PointLft][PointLft] = TLmis;
	twoLinesRel[PointLft][PointLft][PointLft][PointLft] = TLmis;
	// point =  2         3         3
	// line =   1         1         2
	lessCCW[PointLft][PointLft][PointLft] = LessCCWtrue;
	lessCCW[PointCtr][PointLft][PointLft] = LessCCWtrue;
	lessCCW[PointRgt][PointLft][PointLft] = LessCCWfalse;
	lessCCW[PointOpp][PointLft][PointLft] = LessCCWhuh;
	lessCCW[PointLft][PointCtr][PointLft] = LessCCWhuh;
	lessCCW[PointCtr][PointCtr][PointLft] = LessCCWhuh;
	lessCCW[PointRgt][PointCtr][PointLft] = LessCCWfalse;
	lessCCW[PointOpp][PointCtr][PointLft] = LessCCWhuh;
	lessCCW[PointLft][PointRgt][PointLft] = LessCCWtrue;
	lessCCW[PointCtr][PointRgt][PointLft] = LessCCWhuh;
	lessCCW[PointRgt][PointRgt][PointLft] = LessCCWtrue;
	lessCCW[PointOpp][PointRgt][PointLft] = LessCCWtrue;
	lessCCW[PointLft][PointOpp][PointLft] = LessCCWtrue;
	lessCCW[PointCtr][PointOpp][PointLft] = LessCCWhuh;
	lessCCW[PointRgt][PointOpp][PointLft] = LessCCWhuh;
	lessCCW[PointOpp][PointOpp][PointLft] = LessCCWhuh;
	lessCCW[PointLft][PointLft][PointCtr] = LessCCWcolinear;
	lessCCW[PointCtr][PointLft][PointCtr] = LessCCWhuh;
	lessCCW[PointRgt][PointLft][PointCtr] = LessCCWhuh;
	lessCCW[PointOpp][PointLft][PointCtr] = LessCCWhuh;
	lessCCW[PointLft][PointCtr][PointCtr] = LessCCWhuh;
	lessCCW[PointCtr][PointCtr][PointCtr] = LessCCWcolinear;
	lessCCW[PointRgt][PointCtr][PointCtr] = LessCCWhuh;
	lessCCW[PointOpp][PointCtr][PointCtr] = LessCCWhuh;
	lessCCW[PointLft][PointRgt][PointCtr] = LessCCWhuh;
	lessCCW[PointCtr][PointRgt][PointCtr] = LessCCWhuh;
	lessCCW[PointRgt][PointRgt][PointCtr] = LessCCWcolinear;
	lessCCW[PointOpp][PointRgt][PointCtr] = LessCCWhuh;
	lessCCW[PointLft][PointOpp][PointCtr] = LessCCWhuh;
	lessCCW[PointCtr][PointOpp][PointCtr] = LessCCWhuh;
	lessCCW[PointRgt][PointOpp][PointCtr] = LessCCWhuh;
	lessCCW[PointOpp][PointOpp][PointCtr] = LessCCWcolinear;
	lessCCW[PointLft][PointLft][PointRgt] = LessCCWfalse;
	lessCCW[PointCtr][PointLft][PointRgt] = LessCCWhuh;
	lessCCW[PointRgt][PointLft][PointRgt] = LessCCWfalse;
	lessCCW[PointOpp][PointLft][PointRgt] = LessCCWfalse;
	lessCCW[PointLft][PointCtr][PointRgt] = LessCCWfalse;
	lessCCW[PointCtr][PointCtr][PointRgt] = LessCCWhuh;
	lessCCW[PointRgt][PointCtr][PointRgt] = LessCCWhuh;
	lessCCW[PointOpp][PointCtr][PointRgt] = LessCCWhuh;
	lessCCW[PointLft][PointRgt][PointRgt] = LessCCWtrue;
	lessCCW[PointCtr][PointRgt][PointRgt] = LessCCWtrue;
	lessCCW[PointRgt][PointRgt][PointRgt] = LessCCWfalse;
	lessCCW[PointOpp][PointRgt][PointRgt] = LessCCWhuh;
	lessCCW[PointLft][PointOpp][PointRgt] = LessCCWhuh;
	lessCCW[PointCtr][PointOpp][PointRgt] = LessCCWhuh;
	lessCCW[PointRgt][PointOpp][PointRgt] = LessCCWfalse;
	lessCCW[PointOpp][PointOpp][PointRgt] = LessCCWhuh;
	lessCCW[PointLft][PointLft][PointOpp] = LessCCWhuh;
	lessCCW[PointCtr][PointLft][PointOpp] = LessCCWhuh;
	lessCCW[PointRgt][PointLft][PointOpp] = LessCCWfalse;
	lessCCW[PointOpp][PointLft][PointOpp] = LessCCWhuh;
	lessCCW[PointLft][PointCtr][PointOpp] = LessCCWhuh;
	lessCCW[PointCtr][PointCtr][PointOpp] = LessCCWhuh;
	lessCCW[PointRgt][PointCtr][PointOpp] = LessCCWhuh;
	lessCCW[PointOpp][PointCtr][PointOpp] = LessCCWfalse;
	lessCCW[PointLft][PointRgt][PointOpp] = LessCCWtrue;
	lessCCW[PointCtr][PointRgt][PointOpp] = LessCCWhuh;
	lessCCW[PointRgt][PointRgt][PointOpp] = LessCCWhuh;
	lessCCW[PointOpp][PointRgt][PointOpp] = LessCCWhuh;
	lessCCW[PointLft][PointOpp][PointOpp] = LessCCWhuh;
	lessCCW[PointCtr][PointOpp][PointOpp] = LessCCWtrue;
	lessCCW[PointRgt][PointOpp][PointOpp] = LessCCWhuh;
	lessCCW[PointOpp][PointOpp][PointOpp] = LessCCWhuh;
}

LessCCW GeometryLessCounterClockwise(Point p0, Point p1, Point p2, Point p3) {
	// given a reference line p0 to p1, compare lines p0 to p2 and p0 to p3 to see which is less CCW
	PointReLine re12 =  GeometryWherePointReLine2(p0, p1, p2);
	PointReLine re13 =  GeometryWherePointReLine2(p0, p1, p3);
	PointReLine re23 =  GeometryWherePointReLine2(p0, p2, p3);

	return lessCCW[re12][re13][re23];
}

bool GeometryPointOnLineSegment(Point lineBgn, Point lineEnd, Point p) {
	// return true if p on the line segment
	if ( GeometryWherePointReLine(lineBgn, lineEnd, p) != PointCtr)
		return false;
	if (lineBgn.x != lineEnd.x)
		return (p.x - lineBgn.x)*(lineEnd.x - p.x)>0;
	else
		return (p.y - lineBgn.y)*(lineEnd.y - p.y)>0;
}

long GeometryTriangleAreaX2Signed(Point p0, Point p1, Point p2) {
	// compute twice the area of a triangle
	return ((long)p1.x - p0.x)*((long)p2.y - p0.y) - ((long)p2.x - p0.x)*((long)p1.y - p0.y);
}

TwoLinesRelationship GeometryTwoLinesRelated(Point p0, Point p1, Point p2, Point p3) {
	// find the exact relationship between 2 line segments
	PointReLine sidePoint0 =  GeometryWherePointReLine(p2, p3, p0);
	PointReLine sidePoint1 =  GeometryWherePointReLine(p2, p3, p1);
	PointReLine sidePoint2 =  GeometryWherePointReLine(p0, p1, p2);
	PointReLine sidePoint3 =  GeometryWherePointReLine(p0, p1, p3);
	return twoLinesRel[sidePoint0][sidePoint1][sidePoint2][sidePoint3];
}

int GeometryUnredun(Coord *poly, int n) {
	int i;

	while (poly[0].x == poly[n - 1].x && poly[0].y == poly[n - 1].y)
		n--;
	for (i = 1; i<n; i++)
	if (poly[i].x == poly[i - 1].x && poly[i].y == poly[i - 1].y) {
		memmove(&poly[i - 1], &poly[i], (n - i)*sizeof(Coord));
		n--;
	}
	return n;
}

int GeometryUncolinear(Coord *poly, int n) {
	int i;

	for (i = 0; i<n; i++) {
		int ip1 = (i + 1) % n, ip2 = (i + 2) % n;

		if (GeometryTriangleAreaX2Signed(CoordToPoint1(poly[i]), CoordToPoint1(poly[ip1]), CoordToPoint1(poly[ip2])) == 0) {
			if (ip1 != n - 1)
				memmove(&poly[ip1], &poly[ip2], (n - ip1 + 1)*sizeof(Point));
			n--;
			i--;
		}
	}
	return n;
}

bool GeometryUnFigure8(Coord *polyline, int &nPoints) {
	int i, j, iOld = nPoints-1;
	TwoLinesRelationship tlr;

	for (i = 0; i < nPoints; i++) {
		for (j = i + 1; j < nPoints - 2; j++) {
			tlr = GeometryTwoLinesRelated(CoordToPoint1(polyline[iOld]), CoordToPoint1(polyline[i]), CoordToPoint1(polyline[j]), CoordToPoint1(polyline[j + 1]));
			switch (tlr) {
			case TLmis: case TL0tB: case TL2tA: case TL0t2: case TL0t3: case TL1t3: case TLhuh:
			case TLcrs:
			case TL1tB:
			case TL3tA:
			case TLcol:
				break;
			case TL1t2:
				break;
			}
		}
	}
	return false;
}

PointReLine  GeometryWherePointReLine(Point lineBgn, Point lineEnd, Point p) {
	// find if a point is left or right of a line or on its extension
	long areaX2 = GeometryTriangleAreaX2Signed(lineBgn, lineEnd, p);
	if (areaX2>0)
		return PointLft;
	if (areaX2<0)
		return PointRgt;
	return PointCtr;
}

PointReLine  GeometryWherePointReLine2(Point p0, Point p1, Point p2) {
	// used for rays; find if a point is left or right of a line or on its extension
	// then divide the collinear case into same direction or opposite direction
	PointReLine wprl =  GeometryWherePointReLine(p0, p1, p2);

	if (wprl == PointCtr) {
		if (p0.x == p1.x)
			wprl = INORDER(p1.y, p0.y, p2.y) ? PointOpp : wprl;
		else
			wprl = INORDER(p1.x, p0.x, p2.x) ? PointOpp : wprl;
	}
	return wprl;
}

void HitSearchFindHits(PolygonStruct *polyStr, int depth, int argc, char **&argv) {
	Point *pt = (Point *)argv[0];
	Hit *hitList = (Hit *)argv[1];
	int i, nHits = (int)argv[2];

	for (i = 0; i < polyStr->n; i++) {
		if (pt->x == polyStr->point[i].x && pt->y == polyStr->point[i].y) {
			hitList = (Hit *)MyRealloc(40, hitList, (nHits + 1)*sizeof(Hit));
			hitList[nHits].ps = polyStr;
			hitList[nHits].iPoint = i;
			nHits++;
		}
	}
	argv[1] = (char *&)hitList;
	argv[2] = (char *&)nHits;
}

void HitSearchFindStart(PolygonStruct *A, int depth, int argc, char **&argVal) {
	char **argv = (char **) MyMalloc(45, 3 * sizeof(void *));

	for (int i = 0; i < A->n; i++) {
		if (A->point[i].iWE>=0)
			break;
		argv[0] = (char *)&A->point[i];
		argv[1] = NULL;
		argv[2] = 0;
		PolygonTreeTraverse((PolygonStruct *)argVal[0], 0, HitSearchFindHits, 3, (char **&)argv);
		if (argv[2]) {
			WingedEdgeStruct *wE = (WingedEdgeStruct *)argVal[1];
			Hit *hitList;
			int iHit, nHits, nWE = (int)argVal[2];

			PolygonTreeTraverse(A, 0, HitSearchFindHits, 3, (char **&)argv);
			hitList = (Hit *)argv[1];
			nHits = (int)argv[2];
			wE = (WingedEdgeStruct *)MyRealloc(17, wE, (nWE + 1)*sizeof(WingedEdgeStruct));
			wE[nWE].intrRay = (IntersectRay *)MyMalloc(7, 2 * nHits * sizeof(IntersectRay));
			wE[nWE].order = (int *)MyMalloc(8, 2 * nHits * sizeof(int));
			wE[nWE].nRay = 2 * nHits;
			wE[nWE].track = 0;
			for (iHit = 0; iHit < nHits; iHit++) {
				int iPt, iPtM1, iPtP1;

				iPt = hitList[iHit].iPoint;
				iPtM1 = iPt ? iPt - 1 : hitList[iHit].ps->n - 1;
				iPtP1 = (iPt + 1) % hitList[iHit].ps->n;
				WingEdgeSetRay(&wE[nWE], 2*iHit  , hitList[iHit].ps, iPt, iPtM1);
				WingEdgeSetRay(&wE[nWE], 2*iHit+1, hitList[iHit].ps, iPt, iPtP1);
				hitList[iHit].ps->point[iPt].iWE = nWE;
			}
			WingEdgeOrderRays(&wE[nWE]);
			nWE++;
			argVal[1] = (char *)wE;
			argVal[2] = (char *)nWE;
			MyFree(hitList);
		}
	}
	MyFree(argv);
}

Point *PolygonAddPointToPoly(int i, Point p, Point* poly, int &n) {
	poly = (Point *)MyRealloc(14, poly, (n + 2)*sizeof(Point)); // 2 troubles me; 1 should work but doesn't
	if (i!=n)
		memmove(&poly[i + 1], &poly[i], (n - i)*sizeof(Point));
	poly[i] = p;
	n++;
	return poly;
}

bool PolygonAddPolygonIntersections(PolygonStruct *polyStr0, PolygonStruct *polyStr1) {
	// find all intersection points between 2 polygons; if the polygon does not have a vertex at the intersection, add the point
	int i0, i1;
	int n0 = polyStr0->n;
	int n1 = polyStr1->n;

	for (i0 = 0; i0<polyStr0->n; i0++) {
		int i0p1 = (i0 + 1) % polyStr0->n;

		for (i1 = 0; i1<polyStr1->n; i1++) {
			int i1p1 = (i1 + 1) % polyStr1->n;
			TwoLinesRelationship tlr = GeometryTwoLinesRelated(polyStr0->point[i0], polyStr0->point[i0p1], polyStr1->point[i1], polyStr1->point[i1p1]);
			switch (tlr) {
			case TLmis: case TL0tB: case TL2tA: case TL0t2: case TL0t3: case TL1t2: case TL1t3: case TLhuh:
				// ignore
				break;
			case TLcrs: {
				Point cross = GeometryCrossPoint(polyStr0->point[i0], polyStr0->point[i0p1], polyStr1->point[i1], polyStr1->point[i1p1]);

				cross.track = 0;
				cross.iWE = -1;
				polyStr0->point = PolygonAddPointToPoly(i0p1, cross, polyStr0->point, polyStr0->n);
				polyStr1->point = PolygonAddPointToPoly(i1p1, cross, polyStr1->point, polyStr1->n);
				polyStr0->point[i0p1].iWE = polyStr1->point[i1p1].iWE = -2;
				break;
			}
			case TL1tB:
				polyStr1->point = PolygonAddPointToPoly(i1p1, polyStr0->point[i0p1], polyStr1->point, polyStr1->n);
				polyStr0->point[i0p1].iWE = polyStr1->point[i1p1].iWE = -2;
				break;
			case TL3tA:
				polyStr0->point = PolygonAddPointToPoly(i0p1, polyStr1->point[i1p1], polyStr0->point, polyStr0->n);
				polyStr0->point[i0p1].iWE = polyStr1->point[i1p1].iWE = -2;
				break;
			case TLcol: {
				bool horizontal = polyStr0->point[i0].y == polyStr0->point[i0p1].y;
				bool between0, between1, between2, between3;

				if (horizontal) {
					between0 = INORDER(polyStr1->point[i1].x, polyStr0->point[i0].x, polyStr1->point[i1p1].x);
					between1 = INORDER(polyStr1->point[i1].x, polyStr0->point[i0p1].x, polyStr1->point[i1p1].x);
					between2 = INORDER(polyStr0->point[i0].x, polyStr1->point[i1].x, polyStr0->point[i0p1].x);
					between3 = INORDER(polyStr0->point[i0].x, polyStr1->point[i1p1].x, polyStr0->point[i0p1].x);
				}
				else {
					between0 = INORDER(polyStr1->point[i1].y, polyStr0->point[i0].y, polyStr1->point[i1p1].y);
					between1 = INORDER(polyStr1->point[i1].y, polyStr0->point[i0p1].y, polyStr1->point[i1p1].y);
					between2 = INORDER(polyStr0->point[i0].y, polyStr1->point[i1].y, polyStr0->point[i0p1].y);
					between3 = INORDER(polyStr0->point[i0].y, polyStr1->point[i1p1].y, polyStr0->point[i0p1].y);
				}
				if (between0) {
					polyStr1->point = PolygonAddPointToPoly(i1p1, polyStr0->point[i0], polyStr1->point, polyStr1->n);
					polyStr0->point[i0].iWE = polyStr1->point[i1p1].iWE = -2;
					i1++;
				}
				if (between1) {
					polyStr1->point = PolygonAddPointToPoly(i1p1, polyStr0->point[i0p1], polyStr1->point, polyStr1->n);
					polyStr0->point[i0p1].iWE = polyStr1->point[i1p1].iWE = -2;
					i1++;
				}
				if (between2) {
					polyStr0->point = PolygonAddPointToPoly(i0p1, polyStr1->point[i1], polyStr0->point, polyStr0->n);
					polyStr0->point[i0p1].iWE = polyStr1->point[i1].iWE = -2;
					i0++;
				}
				if (between3) {
					polyStr0->point = PolygonAddPointToPoly(i0p1, polyStr1->point[i1p1], polyStr0->point, polyStr0->n);
					polyStr0->point[i0p1].iWE = polyStr1->point[i1p1].iWE = -2;
					i0++;
				}
				break;
			}
			}
		}
	}
	return polyStr0->n>n0 || polyStr1->n>n1;
}

long PolygonAreaX2Signed(Point *poly, int n) {
	// compute twice the area of a polygon
	int i;
	long sum = 0;

	for (i = 0; i<n - 1; i++)
		sum += (poly[i].x*poly[i + 1].y) - (poly[i + 1].x*poly[i].y);
	return sum + (poly[n - 1].x*poly[0].y) - (poly[0].x*poly[n - 1].y);
}

WingedEdgeStruct *PolygonCompareGenWingedEdges(PolygonStruct *A, const char *a, PolygonStruct *B, const char *b, FILE *log) {
	WingedEdgeStruct *wE = NULL;
	int i, nWE = 0;
	char **text;

	LOG = log;
	LOGPRINTF(log, "original polygons\n");
	LOGTRAVERSE(A, 1, PolygonTreePrint, (int)false, text = (char **)a);
	LOGTRAVERSE(B, 1, PolygonTreePrint, (int)false, text = (char **)b);
	LOGPRINTF(log, "\nadd intersections\n");
	PolygonTreeAddIntersections(A, B, true);
	LOGTRAVERSE(A, 1, PolygonTreePrint, (int)false, text = (char **)a);
	LOGTRAVERSE(B, 1, PolygonTreePrint, (int)false, text = (char **)b);
	LOGPRINTF(log, "\nadd points between intersections\n");
	PolygonTreeAddCertainMidPoints(A);
	PolygonTreeAddCertainMidPoints(B);
	LOGTRAVERSE(A, 1, PolygonTreePrint, (int)false, text = (char **)a);
	LOGTRAVERSE(B, 1, PolygonTreePrint, (int)false, text = (char **)b);
	LOGPRINTF(log, "\nadd track, winged edge index\n");
	WingEdgeStructsCreate(wE, nWE, A, B, true);
	LOGTRAVERSE(A, 1, PolygonTreePrint, (int)true, text = (char **)a);
	LOGTRAVERSE(B, 1, PolygonTreePrint, (int)true, text = (char **)b);
	LOGPRINTF(log, "\nrays\n");
	for (i = 0; i<nWE; i++) {
		LOGPRINTF(log, "{%2d,%2d}: ", wE[i].intrRay[0].polyStr->point[wE[i].intrRay[0].iCrs].x, wE[i].intrRay[0].polyStr->point[wE[i].intrRay[0].iCrs].y);
		for (int j = 0; j<wE[i].nRay; j++)
			LOGPRINTF(log, "{%2d,%2d} ", wE[i].intrRay[j].polyStr->point[wE[i].intrRay[j].iEnd].x, wE[i].intrRay[j].polyStr->point[wE[i].intrRay[j].iEnd].y);
		LOGPRINTF(log, "\n");
	}
	LOGPRINTF(log, "\nray ccw orders\n");
	for (i = 0; i<nWE; i++) {
		LOGPRINTF(log, "{%2d,%2d}: ", wE[i].intrRay[0].polyStr->point[wE[i].intrRay[0].iCrs].x, wE[i].intrRay[0].polyStr->point[wE[i].intrRay[0].iCrs].y);
		LOGPRINTF(log, "0 %d %d %d\n", wE[i].order[0] + 1, wE[i].order[1] + 1, wE[i].order[2] + 1);
	}
	wE = (WingedEdgeStruct *)MyRealloc(30, wE, (nWE + 1)*sizeof(WingedEdgeStruct));
	wE[nWE].intrRay = NULL;
	wE[nWE].nRay = 0;
	wE[nWE].order = NULL;
	wE[nWE].track = 0;
	return wE;
}

void PolygonFreeStrAndPoints(PolygonStruct *polyStr0, int deapth, int argc, char **&argv) {
	MyFree(polyStr0->point);
	MyFree(polyStr0);
}

PolygonStruct *PolygonFollowEdgeFindStart(int deapth, PolygonStruct *pA, PolygonStruct *pB, PointRelPoly start, PolygonStruct *B, PathSide follow, WingedEdgeStruct *wEArray) {
	// find a point on pA that is inside pB (or outside, depending on argument side)
	// then follow that point by calling FollowEdge
	PolygonStruct *pSet = NULL;

	for (int i = 0; i<pA->n; i++)
	if (pA->point[i].track == 0) {
		PointRelPoly prp =  PolygonTreeWherePoint(B, pA->point[i], 0);
		if (prp == start && (pA->point[i].iWE >= 0 ? wEArray[pA->point[i].iWE].track == 0 : true)) {
			pSet = FollowEdge(i, pA, pB, follow, wEArray);
			if (pSet)
				PolygonPush(pSet);
		}
	}
	return pSet;
}

bool PolygonInsidePolygon(PolygonStruct *A, PolygonStruct *B) {
	// meant for output of tracings, not arbitrary user input; see PolygonTreeNest
	// only compares the 2 root polygons
	for (int i = 0; i<A->n; i++)
	if ( PolygonWherePoint(B->point, B->n, A->point[i]) == PointInSide)
		return true;
	return false;
}

PolygonStruct *PolygonNew(PolygonStruct *parent, PolygonStruct *child, PolygonStruct *prevSibling, PolygonStruct *nextSibling, Point *point, int n) {
	PolygonStruct *newPoly = (PolygonStruct *)MyMalloc(4, sizeof(PolygonStruct));
	static int id = 0;

	id++;
	memset(newPoly, 0, sizeof(PolygonStruct));
	newPoly->id = id;
	if (parent) {
		newPoly->parent = parent;
		parent->child = newPoly;
	}
	if (child) {
		newPoly->child = child;
		child->parent = newPoly;
	}
	if (prevSibling) {
		newPoly->prevSibling = prevSibling;
		prevSibling->nextSibling = newPoly;
	}
	if (nextSibling) {
		newPoly->nextSibling = nextSibling;
		nextSibling->prevSibling = newPoly;
	}
	newPoly->n = n;
	newPoly->point = point;
	return newPoly;
}

void PolygonPrint(char *name, PolygonStruct *ps, bool details, FILE *log) {
	if (ps) {
		fprintf(log, "%s %2d %2d ", name, ps->id, ps->n);
		if (details)
		for (int i = 0; i<ps->n; i++)
			fprintf(log, "{%d,%d}%d,%2d", ps->point[i].x, ps->point[i].y, ps->point[i].track, ps->point[i].iWE);
		else
		for (int i = 0; i<ps->n; i++)
			fprintf(log, "{%d,%d},", ps->point[i].x, ps->point[i].y);
		printf("\n");
	}
	else
		printf("%s null\n", name);
}

void PolygonPrintRaw(FILE *log, const char *name) {
	PolygonStruct **A = NULL;
	int i, n;

	for (i = 0;; i++) {
		A = (PolygonStruct **)MyRealloc(15, A, (i + 1)*sizeof(PolygonStruct));
		A[i] = PolygonPop(0);
		if (A[i] == NULL)
			break;
	}
	n = i + 1;
	for (i = 0; i<n - 1; i++)
		PolygonPrint((char *)name, A[i], false, log);
	for (i = 0; i<n; i++)
		PolygonPush(A[n - 1 - i]);
	MyFree(A);
}

void PolygonPlaceInStruct(PolygonStruct *A, PolygonStruct * &set) {
	PolygonStruct *B, *oldB = NULL;

	if (set == NULL) {
		set = A;
		return;
	}
	if (PolygonInsidePolygon(set, A)) {
		if (set->parent) {
			set->parent->child = A;
			A->parent = set;
		}
		A->child = set;
		set->parent = A;
		if (A->parent == NULL)
			set = A;
		return;
	}
	for (B = set; B != NULL; B = B->nextSibling) {
		if (PolygonInsidePolygon(A, B)) {
			if (B->child) {
				PolygonPlaceInStruct(A, B->child);
				return;
			}
			else {
				B->child = A;
				A->parent = B;
				return;
			}
		}
		oldB = B;
	}
	oldB->nextSibling = A;
	A->prevSibling = oldB;
	return;
}

void PolygonPlaceInStructInsert(PolygonStruct *A, int deapth, int argcArg, char **&argvArg) {
	PolygonStruct *set = (PolygonStruct *)argvArg;

	A->parent = A->child = A->nextSibling = A->prevSibling = NULL;
	if (set == NULL) {
		argvArg = (char **)A;
		return;
	}
	PolygonPlaceInStruct(A, set);
	argvArg = (char **)set;
}

void PolygonPlaceInStructSpin(PolygonStruct *A, int deapth, int argcArg, char **&argvArg) {
	long area = PolygonAreaX2Signed(A->point, A->n);

	if ((deapth & 1) == 0 && area>0)	// outside outlines are CCW
		return;
	if (deapth & 1 && area<0)			// holes are CW
		return;
	for (int i = 0; i<A->n / 2; i++) {
		Point temp = A->point[i];

		A->point[i] = A->point[A->n - 1 - i];
		A->point[A->n - 1 - i] = temp;
	}
}

PolygonStruct *PolygonPop(bool init) {
	if (init) {
		nStackMax = nStack = 0;
		MyFree(stack);
		stack = NULL;
		return NULL;
	}
	if (nStack == 0)
		assert(nStack != 0);
	nStack--;
	PolygonStruct *C = stack[nStack];
	//	printf("poping %x\n", (unsigned int) C);
	return C;
}

void PolygonPush(PolygonStruct *datum) {
	if (nStack >= nStackMax) {
		nStackMax = nStack + 100;
		stack = (PolygonStruct **)MyRealloc(16, stack, nStackMax*sizeof(PolygonStruct *));
	}
	stack[nStack] = datum;
	nStack++;
	//	printf("pushed %x\n", (unsigned int) datum);
}

void PolygonTreeAddCertainMidPoints(PolygonStruct *A) {
	char **b = NULL;
	PolygonTreeTraverse(A, 0, AddCertainMidPointsProc, 0, b);
}

bool PolygonTreeAddIntersections(PolygonStruct *polyStr0, PolygonStruct *polyStr1, bool recurseA) {
	PolygonStruct *p0, *p1;

	for (p0 = polyStr0; p0 != NULL; p0 = p0->nextSibling) {
		for (p1 = polyStr1; p1 != NULL; p1 = p1->nextSibling) {
			PolygonAddPolygonIntersections(p0, p1);
			if (p1->child)
				PolygonTreeAddIntersections(polyStr0, p1->child, false);
		}
		if (p0->child)
			PolygonTreeAddIntersections(p0->child, polyStr1, true);
	}
	return true;
}

PolygonStruct *PolygonTreeFromCoords(Coord **&polyline, int *nPoints, int &nPolylines) {
	int iPoly;
	PolygonStruct *pS, *pSwas;

	for (iPoly = 0; iPoly < nPolylines; iPoly++) {
		pS = PolygonNew(NULL, NULL, NULL, NULL, CoordToPoint(polyline[iPoly], nPoints[iPoly]), nPoints[iPoly]);
		if (iPoly>0) {
			pSwas->prevSibling = pS;
			pS->nextSibling = pSwas;
		}
		pSwas = pS;
	}
	pS = PolygonTreeNest(pS);
	return pS;
}

void PolygonTreePrint(PolygonStruct *polyStrArg, int deapthArg, int argcArg, char **&argvArg) {
	char *name = (char *)argvArg, title[99];
	bool details = argcArg ? true : false;

	sprintf_s(title, sizeof(title)-1, "%s %d", name, deapthArg);
	PolygonPrint(title, polyStrArg, details, LOG);
}

PolygonStruct *PolygonSetPairTracer(int deapth, PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, PointRelPoly start, PolygonStruct *root, PathSide follow, bool bothWays, bool recurseA) {
	PolygonStruct *p0, *p1, *C = NULL, *path;

	for (p0 = A; p0 != NULL; p0 = p0->nextSibling) {
		for (p1 = B; p1 != NULL; p1 = p1->nextSibling) {
			path = PolygonTreePairTracer(deapth, A, root, p0, p1, wE, start, follow, bothWays);
			if (p1->child && PolygonTreeInteract(A, p1->child)) {
				PolygonSetPairTracer(deapth + 1, A, p1->child, wE, start, root, follow, bothWays, false);
			}
			//MyFree(path);
		}
		if (recurseA && p0->child && PolygonTreeInteract(p0->child, B)) {
			PolygonSetPairTracer(deapth + 1, p0->child, B, wE, start, root, follow, bothWays, true);
		}
	}
	return C;
}

PolygonStruct *PolygonSetPairOps(PolygonStruct *polyStr0, PolygonStruct *polyStr1, PolyOp op) {
	return NULL;
}

void PolygonSetRotation(Point *poly, int n, Rotation direction) {
	long areaX2 = PolygonAreaX2Signed(poly, n);
	Point p;
	int i;

	if ((areaX2>0 && direction == RotateLft) || (areaX2<0 && direction == RotateRgt))
		return;
	for (i = 0; i<n / 2; i++) {
		memcpy(&p, &poly[i], sizeof(Point));
		memcpy(&poly[i], &poly[n - 1 - i], sizeof(Point));
		memcpy(&poly[n - 1 - i], &p, sizeof(Point));
	}
}

bool PolygonTreeInteract(PolygonStruct *A, PolygonStruct *B) {
	if (AnyPointMatches(A, B, PointOnVert | PointInSide) || AnyPointMatches(B, A, PointOnVert | PointInSide))	// PointOnEdge not needed because intersections have already been added
		return true;
	return false;
}

PolygonStruct *PolygonTreeNest(PolygonStruct *A) {
	// meant for output of tracings, not arbitrary user input, so can assume:
	// - no 2 polygons have the same outline (regardless of the order of the points)
	// - no ploygon has both a point inside and a point outside another polygon
	// - so for 2 polygons either A has a point inside or outside B, or vice versa
	PolygonStruct *B = NULL;
	char **b = (char **)B;

	if (A == NULL)
		return A;
	PolygonTreeTraverse(A, 0, PolygonPlaceInStructInsert, 0, b);
	B = (PolygonStruct *)b;
	PolygonTreeTraverse(B, 0, PolygonPlaceInStructSpin, 0, b = NULL);
	return B;
}

PolygonStruct *PolygonTreeOpInter(PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, FILE *log, char *tag) {
	PolygonStruct *C;
	char **b = NULL;

	PolygonTreeTraverse(A, 0, PolygonZeroTracks, 0, b);
	PolygonTreeTraverse(B, 0, PolygonZeroTracks, 0, b);
	PolygonPush(0);
	PolygonSetPairTracer(0, A, B, wE, PointInSide, B, PathTurnLft, false, true);
	PolygonSetPairTracer(0, B, A, wE, PointInSide, A, PathTurnLft, false, true);
	if (log)
		PolygonPrintRaw(log, tag);
	C = PopPolygons();
	PolygonTreeNest(C);
	return C;
}

PolygonStruct *PolygonTreeOpMinus(PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, FILE *log, char *tag) {
	PolygonStruct *C;
	char **b = NULL;

	PolygonTreeTraverse(A, 0, PolygonZeroTracks, 0, b);
	PolygonTreeTraverse(B, 0, PolygonZeroTracks, 0, b);
	PolygonPush(0);
	PolygonSetPairTracer(0, A, B, wE, PointOutside, B, PathTurnLft, false, true);
	if (log)
		PolygonPrintRaw(log, tag);
	C = PopPolygons();
	PolygonTreeNest(C);
	return C;
}

PolygonStruct *PolygonTreeOpUnion(PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, FILE *log, char *tag) {
	PolygonStruct *C;
	char **b = NULL;

	PolygonTreeTraverse(A, 0, PolygonZeroTracks, 0, b);
	PolygonTreeTraverse(B, 0, PolygonZeroTracks, 0, b);
	PolygonPush(0);
	PolygonSetPairTracer(0, A, B, wE, PointOutside, B, PathTurnRgt, false, true);
	PolygonSetPairTracer(0, B, A, wE, PointOutside, A, PathTurnRgt, false, true);
	if (log)
		PolygonPrintRaw(log, tag);
	C = PopPolygons();
	C = PolygonTreeNest(C);
	return C;
}

PolygonStruct *PolygonTreePairTracer(int deapth, PolygonStruct *A, PolygonStruct *B, PolygonStruct *p0, PolygonStruct *p1, WingedEdgeStruct *wE, PointRelPoly start, PathSide follow, bool bothWays) {
	PolygonStruct *path;

	path = PolygonFollowEdgeFindStart(deapth, p0, p1, start, B, follow, wE);
	//	if (path)
	//		PolygonPush(path);
	//	if (path)
	//		printf("PolygonTreePairTracer Pushed polygon starting at {%2d,%2d}\n", path->point[0].x, path->point[0].y);
	//	if (bothWays)
	//		path = PolygonTreePrepend(PolygonFollowEdgeFindStart(deapth, p1, p0, start, A, follow, wE), path);
	//path = PolygonTreePrepend(PolygonFollowEdgeFindStart(deapth, p0, p1, PointOnVert, B, follow, wE), path);
	return path;
}

PolygonStruct *PolygonTreePrepend(PolygonStruct *A, PolygonStruct *B) {
	// prepend A to B
	if (A == NULL)
		return B;
	if (B == NULL)
		return A;
	B->prevSibling = A;
	A->nextSibling = B;
	return A;
}

void PolygonTreeTraverse(PolygonStruct *polyStr, int deapth, void proc(PolygonStruct *polyStrArg, int deapthArg, int argcArg, char **&argvArg), int argc, char **&argv) {
	// this slightly complex traverse allows the proc to delete the structure.
	PolygonStruct *polyStrPtr, *polyStrPtrWas;

	if (polyStr && polyStr->nextSibling) {
		for (polyStrPtr = polyStr->nextSibling; polyStrPtr != NULL; polyStrPtr = polyStrPtr->nextSibling) {
			if (polyStrPtr->prevSibling->child)
				PolygonTreeTraverse(polyStrPtr->prevSibling->child, deapth + 1, proc, argc, argv);
			proc(polyStrPtr->prevSibling, deapth, argc, argv);
			polyStrPtrWas = polyStrPtr;
		}
		polyStrPtr = polyStrPtrWas;
	}
	else {
		polyStrPtr = polyStr;
	}
	if (polyStrPtr) {
		if (polyStrPtr->child)
			PolygonTreeTraverse(polyStrPtr->child, deapth + 1, proc, argc, argv);
		proc(polyStrPtr, deapth, argc, argv);
	}
}

PointRelPoly  PolygonTreeWherePoint(PolygonStruct *polySctrSet, Point p0, int depth) {
	PolygonStruct *polySctr = NULL;

	for (polySctr = polySctrSet; polySctr != NULL; polySctr = polySctr->nextSibling) {
		switch ( PolygonWherePoint(polySctr->point, polySctr->n, p0)) {
		case PointInSide:
			if (polySctr->child)
				return  PolygonTreeWherePoint(polySctr->child, p0, depth + 1);
			else
				return (depth & 1) ? PointOutside : PointInSide;
		case PointOnVert:
			return PointOnVert;
		case PointOnEdge:
			return PointOnEdge;
		case PointOutside:
			// continue on to next sibling
			continue;
		}
	}
	return (depth & 1) ? PointInSide : PointOutside;
}

int PolygonUnredun(Point *poly, int n) {
	int i;

	while (poly[0].x == poly[n - 1].x && poly[0].y == poly[n - 1].y)
		n--;
	for (i = 1; i<n; i++)
		if (poly[i].x == poly[i - 1].x && poly[i].y == poly[i - 1].y) {
			memmove(&poly[i - 1], &poly[i], (n - i)*sizeof(Point));
			n--;
		}
	return n;
}

int PolygonUncolinear(Point *poly, int n) {
	int i;

	for (i = 0; i<n; i++) {
		int ip1 = (i + 1) % n, ip2 = (i + 2) % n;

		if (GeometryTriangleAreaX2Signed(poly[i], poly[ip1], poly[ip2]) == 0) {
			if (ip1 != n - 1)
				memmove(&poly[ip1], &poly[ip2], (n - ip1 + 1)*sizeof(Point));
			n--;
			i--;
		}
	}
	return n;
}

PointRelPoly  PolygonWherePoint_(Point *poly, int n, Point p2) {
	// find if a point is inside or outside a polygon;
	// it is already known that p2 is not on a polygon vertex or edge
	Point p3 = { SHRT_MAX, p2.y };
	int j, k, count = 0;
	PointReLine cameFrom;
	TwoLinesRelationship rel;

	for (k = 0; k<n; k++)
	if (poly[k].y != p2.y)
		break;
	for (j = 0; j<n; j++) {
		int i = (j + k) % n, ip1 = (i == n - 1) ? 0 : i + 1;

		rel = GeometryTwoLinesRelated(poly[i], poly[ip1], p2, p3);
		switch (rel) {
		case TLmis:
			break;
		case TLcrs:
			count++;
			break;
		case TL0tB:
			// leaving being on edge
			if ( GeometryWherePointReLine(p2, p3, poly[ip1]) != cameFrom)
				count++;
			break;
		case TL1tB:
			// starting being on edge
			cameFrom =  GeometryWherePointReLine(p2, p3, poly[i]);
			break;
		case TLcol:
			// continuing on edge or complete miss
			break;
		case TL0t2:	// p2 does not touch a vertex or edge
		case TL1t2:
		case TL2tA:
		case TL0t3:	// p3 is know to be outside
		case TL1t3:
		case TL3tA:
		case TLhuh:
			// error
			break;
		}
	}
	return (count & 1) ? PointInSide : PointOutside;
}

PointRelPoly  PolygonWherePoint(Point *poly, int n, Point p0) {
	// take care of special cases; then use  PolygonWherePoint_
	int i;

	for (i = 0; i<n; i++)
	if (poly[i].x == p0.x && poly[i].y == p0.y)
		return PointOnVert;
	for (i = 0; i<n - 1; i++)
	if (GeometryPointOnLineSegment(poly[i], poly[i + 1], p0))
		return PointOnEdge;
	if (GeometryPointOnLineSegment(poly[n - 1], poly[0], p0))
		return PointOnEdge;
	return  PolygonWherePoint_(poly, n, p0);
}

void PolygonZeroTracks(PolygonStruct *polyStr0, int deapth, int argc, char **&angv) {
	for (int i = 0; i<polyStr0->n; i++)
		polyStr0->point[i].track = 0;
}

PolygonStruct *PopPolygons() {
	PolygonStruct *C = NULL, *D;

	for (D = PolygonPop(0); D != NULL; D = PolygonPop(0)) {
		D->n = PolygonUncolinear(D->point, D->n);
		for (int i = 0; i<D->n; i++) {
			D->point[i].x /= 2;
			D->point[i].y /= 2;
		}
		C = PolygonTreePrepend(D, C);
	}
	return C;
}

void WingEdgeFree(WingedEdgeStruct *wE) {
	for (int i = 0; wE[i].order != NULL; i++) {
		MyFree(wE[i].intrRay);
		MyFree(wE[i].order);
	}
	MyFree(wE);
}

inline void WingEdgeInitOrder(int *order, WingedEdgeStruct *wE) {
	// initialize the order array for later sorting
	order[0] = 0;
	for (int m = 0; m<wE->nRay - 1; m++)
		order[m + 1] = wE->order[m] + 1;
}

bool WingEdgeOrderRays(WingedEdgeStruct *wE) {
	// use qsort to order the rays in CCW order from the reference ray at [0] 
	int i, n = wE->nRay;

	currentWE = wE;
	for (i = 0; i<n; i++)
		wE->order[i] = i;
	qsort((void *)wE->order, wE->nRay - 1, sizeof(int), GeometryCompareRays);
	return true;
}

void WingEdgeSetRay(WingedEdgeStruct *wE, int i, PolygonStruct *polyStr, int iCrs, int iEnd) {
	wE->intrRay[i].polyStr = polyStr;
	wE->intrRay[i].iCrs = iCrs;
	wE->intrRay[i].iEnd = iEnd;
	//wE->intrRay[i].polyStr->point[iCrs].wE = wE;
}

bool WingEdgeStructsCreate(WingedEdgeStruct *&wE, int &nWE, PolygonStruct *A, PolygonStruct *B, bool recurseA) {
	char **argv = (char **)MyMalloc(50, 3 * sizeof(char *));		// B, wE, nWE

	argv[0] = (char *)B;
	argv[1] = NULL;
	argv[2] = 0;
	PolygonTreeTraverse(A, 0, HitSearchFindStart, 3, (char **&)argv);
	wE = (WingedEdgeStruct *)argv[1];
	nWE = (int)argv[2];
	MyFree(argv);
	return true;
}
