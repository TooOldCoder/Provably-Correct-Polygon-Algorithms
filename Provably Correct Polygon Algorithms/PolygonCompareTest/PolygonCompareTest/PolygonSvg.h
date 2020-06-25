void SvgPolygonSet(PolygonStruct *polyStrArg, int deapthArg, int argcArg, char **&argvArg);
void SvgPolylineArrayFree(Coord **polyline, int *nPolylinePoints, int n);
int SvgReadPolylines(Coord **&polylineA, int *&nPointsA, int &nPolylinesA, Coord **&polylineB, int *&nPointsB, int &nPolylinesB, char *argv[5]);
int SvgSave(PolygonStruct *path, char *label, char *inFile, char *argvs[]);
