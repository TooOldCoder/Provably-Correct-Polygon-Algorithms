typedef struct CoordTag {
	short		x;
	short		y;
} Coord;

struct WingedEdgeStructTag;

typedef struct PointTag {
	short		x;
	short		y;
	int		track;
	int		iWE;
} Point;

typedef struct PolygonTreeTag {
	int					id;
	int					n;
	Point					*point;
	PolygonTreeTag	*parent;
	PolygonTreeTag	*child;
	PolygonTreeTag	*prevSibling;
	PolygonTreeTag	*nextSibling;
}	PolygonStruct;

typedef struct IntersectRayTag {
	PolygonStruct	*polyStr;
	int				iCrs;
	int				iEnd;
}	IntersectRay;

typedef struct WingedEdgeStructTag {
	IntersectRay	*intrRay;
	int				*order;
	int				nRay;
	int				track;
}	WingedEdgeStruct;

typedef struct HitTag {
	PolygonStruct *ps;
	int iPoint;
} Hit;

typedef enum { PolyOpUnion, PolyOpInter, PolyOpMinus } PolyOp;
typedef enum { PathTurnLft, PathTurnRgt } PathSide;
typedef enum { LessCCWtrue, LessCCWfalse, LessCCWcolinear, LessCCWhuh } LessCCW;
typedef enum { PointLft = 0, PointCtr = 1, PointRgt = 2, PointOpp = 3 } PointReLine;
typedef enum { PointOnVert = 1, PointOnEdge = 2, PointInSide = 4, PointOutside = 8 } PointRelPoly;
typedef enum { TLmis, TLcrs, TL0tB, TL1tB, TL2tA, TL3tA, TL0t2, TL0t3, TL1t2, TL1t3, TLcol, TLhuh } TwoLinesRelationship;
typedef enum { RotateLft, RotateRgt } Rotation;

Point *CoordToPoint(Coord *coord, int n);
void GeometryBoundPolygon(Point *poly, int n);
int GeometryClean(Coord **&polyline, int *&nPoints, int nPolylines, int flags);
Point GeometryCrossPoint(Point p0, Point p1, Point p2, Point p3);
void GeometryInit();
LessCCW GeometryLessCounterClockwise(Point p0, Point p1, Point p2, Point p3);
long GeometryTriangleAreaX2Signed(Point p0, Point p1, Point p2);
TwoLinesRelationship GeometryTwoLinesRelated(Point p0, Point p1, Point p2, Point p3);
PointReLine GeometryWherePointReLine2(Point p0, Point p1, Point p2);
PointReLine  GeometryWherePointReLine(Point lineBgn, Point lineEnd, Point p);
int GeometryUnredun(Coord *poly, int n);
int GeometryUncolinear(Coord *poly, int n);
PointRelPoly  PolygonWherePoint(Point *poly, int n, Point p0);
PointRelPoly  PolygonTreeWherePoint(PolygonStruct *polySctrSet, Point p0, int depth);
bool GeometryPointOnLineSegment(Point lineBgn, Point lineEnd, Point p);
Point *PolygonAddPointToPoly(int i, Point p, Point* poly, int &n);
bool PolygonAddPolygonIntersections(PolygonStruct *p0, PolygonStruct *p1);
long PolygonAreaX2Signed(Point *poly, int n);
WingedEdgeStruct *PolygonCompareGenWingedEdges(PolygonStruct *A, const char *a, PolygonStruct *B, const char *b, FILE *log);
PolygonStruct *PolygonFollowEdgeFindStart(PolygonStruct *pA, PolygonStruct *pB, PointRelPoly start, PolygonStruct *B, PathSide follow, WingedEdgeStruct *wEArray);
bool PolygonInsidePolygon(PolygonStruct *A, PolygonStruct *B);
PolygonStruct *PolygonNew(PolygonStruct *parent, PolygonStruct *child, PolygonStruct *prevSibling, PolygonStruct *nextSibling, Point *point, int n);
bool PolygonOrderCrossing(PolygonStruct *polyStr0, int i0m1, PolygonStruct *polyStr1, int i1m1, int *order);
void PolygonPrint(char *name, PolygonStruct *ps, bool details, FILE *log);
PolygonStruct *PolygonPop(bool init);
void PolygonPush(PolygonStruct *datum);
PolygonStruct *PolygonSetPairTracer(int depth, PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, PointRelPoly start, PolygonStruct *root, PathSide follow, bool bothWays, bool recurseA);
void PolygonSetRotation(Point *poly, int n, Rotation direction);
void PolygonTreeAddCertainMidPoints(PolygonStruct *A);
bool PolygonTreeAddIntersections(PolygonStruct *polyStr0, PolygonStruct *polyStr1, bool recurseA);
void PolygonFreeStrAndPoints(PolygonStruct *polyStr0, int deapth, int argc, char **&angv);
PolygonStruct *PolygonTreeFromCoords(Coord **&polyline, int *nPoints, int &nPolylines);
bool PolygonTreeInteract(PolygonStruct *A, PolygonStruct *B);
PolygonStruct *PolygonTreeOpInter(PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, FILE *log, char *tag);
PolygonStruct *PolygonTreeOpMinus(PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, FILE *log, char *tag);
PolygonStruct *PolygonTreeOpUnion(PolygonStruct *A, PolygonStruct *B, WingedEdgeStruct *wE, FILE *log, char *tag);
PolygonStruct *PolygonTreeNest(PolygonStruct *A);
PolygonStruct *PolygonTreePairTracer(int depth, PolygonStruct *A, PolygonStruct *B, PolygonStruct *p0, PolygonStruct *p1, WingedEdgeStruct *wE, PointRelPoly start, PathSide follow, bool bothWays);
PolygonStruct *PolygonTreePrepend(PolygonStruct *A, PolygonStruct *B);
void PolygonTreePrint(PolygonStruct *polyStrArg, int deapthArg, int argcArg, char **&argvArg);
void PolygonTreeTraverse(PolygonStruct *polyStr, int depth, void proc(PolygonStruct *polyStrArg, int depth, int argcArg, char **&argvArg), int argc, char **&argv);
int PolygonUncolinear(Point *poly, int n);
int PolygonUnredun(Point *poly, int n);
void PolygonZeroTracks(PolygonStruct *polyStr0, int depth, int argc, char **&angv);
PolygonStruct *PopPolygons();
void WingEdgeFree(WingedEdgeStruct *wE);
inline void WingEdgeInitOrder(int *order, WingedEdgeStruct *wE);
bool WingEdgeOrderRays(WingedEdgeStruct *wE);
void WingEdgeSetRay(WingedEdgeStruct *wE, int i, PolygonStruct *polyStr, int iCrs, int iEnd);
bool WingEdgeStructsCreate(WingedEdgeStruct *&wE, int &nWE, PolygonStruct *polyStr0, PolygonStruct *polyStr1, bool recurseA);

#ifdef STATIC
TwoLinesRelationship twoLinesRel[3][3][3][3] = {
	TLmis, TLmis, TLmis, TLmis, TLhuh, TLmis, TLmis, TLmis, TLmis,
	TLmis, TLhuh, TLhuh, TL1t2, TLhuh, TLhuh, TL1tB, TL1t3, TLmis,
	TLmis, TLhuh, TLcrs, TL2tA, TLhuh, TLhuh, TLcrs, TL3tA, TLmis,
	TLmis, TL0t3, TL0tB, TLhuh, TLhuh, TL0t2, TLhuh, TLhuh, TLmis,
	TLhuh, TLhuh, TLhuh, TLhuh, TLcol, TLhuh, TLhuh, TLhuh, TLhuh,
	TLmis, TLhuh, TLhuh, TL0t2, TLhuh, TLhuh, TL0tB, TL0t3, TLmis,
	TLmis, TL3tA, TLcrs, TLhuh, TLhuh, TL2tA, TLcrs, TLhuh, TLmis,
	TLmis, TL1t3, TL1tB, TLhuh, TLhuh, TL1t2, TLhuh, TLhuh, TLmis,
	TLmis, TLmis, TLmis, TLmis, TLhuh, TLmis, TLmis, TLmis, TLmis
};
LessCCW lessCCW[4][4][4] = {
	LessCCWtrue, LessCCWcolinear, LessCCWfalse, LessCCWhuh,
	LessCCWhuh, LessCCWhuh, LessCCWfalse, LessCCWhuh,
	LessCCWtrue, LessCCWhuh, LessCCWtrue, LessCCWtrue,
	LessCCWtrue, LessCCWhuh, LessCCWhuh, LessCCWhuh,
	LessCCWtrue, LessCCWhuh, LessCCWhuh, LessCCWhuh,
	LessCCWhuh, LessCCWcolinear, LessCCWhuh, LessCCWhuh,
	LessCCWhuh, LessCCWhuh, LessCCWtrue, LessCCWhuh,
	LessCCWhuh, LessCCWhuh, LessCCWhuh, LessCCWtrue,
	LessCCWfalse, LessCCWhuh, LessCCWfalse, LessCCWfalse,
	LessCCWfalse, LessCCWhuh, LessCCWhuh, LessCCWhuh,
	LessCCWtrue, LessCCWcolinear, LessCCWfalse, LessCCWhuh,
	LessCCWhuh, LessCCWhuh, LessCCWfalse, LessCCWhuh,
	LessCCWhuh, LessCCWhuh, LessCCWfalse, LessCCWhuh,
	LessCCWhuh, LessCCWhuh, LessCCWhuh, LessCCWfalse,
	LessCCWtrue, LessCCWhuh, LessCCWhuh, LessCCWhuh,
	LessCCWhuh, LessCCWcolinear, LessCCWhuh, LessCCWhuh
};
#else
extern TwoLinesRelationship twoLinesRel[3][3][3][3];
extern LessCCW lessCCW[4][4][4];
#endif

#define LEN(array)	(sizeof(array)/sizeof(array[0]))
#define SWAP(a,b)  { a = (a)^(b); b = (b)^(a); a = (a)^(b); }
#define INORDER(a,b,c) ((((long) (a)-(long) (b))*((long) (b)-(long) (c)))>0)
#ifndef min
#define min(a,b) (((a)<(b))? (a):(b))
#endif
#ifndef max
#define max(a,b) (((a)>(b))? (a):(b))
#endif
#define LOGPRINTF if (log) fprintf
#define LOGTRAVERSE if (log) PolygonTreeTraverse
#define CLEAN_REDUNDNT 1
#define CLEAN_COLINEAR 2
#define CLEAN_FIGURE_8 4
#define CLEAN_DEGENERT 8
#ifdef _DEBUG
#define FREE
#endif
