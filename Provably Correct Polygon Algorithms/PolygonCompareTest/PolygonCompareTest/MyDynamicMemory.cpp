#include "stdio.h"
#include "MyDynamicMemory.h"
#include <exception>
#include <iostream>

using namespace std;

typedef struct MallocRecordTag {
	int				id;
	int				len;
	void				*ptr;
}	MallocRecord;

static FILE *dLOG = NULL;
static int nMallocRecords = 0;
static MallocRecord *mallocRecord = NULL;

void MyDynamicMemoryOpen() {
	int err;

	if (dLOG == NULL) {
		err = fopen_s(&dLOG, "C:\\Data\\Polygon\\dLOG.txt", "w");
		if (err)
			exit(998);
		fclose(dLOG);
	}
	err = fopen_s(&dLOG, "C:\\Data\\Polygon\\dLOG.txt", "a");
	if (err)
		exit(999);
}

int MyDynamicMemoryCheck(int pass) {
#ifdef _DEBUG
	int i, j, k;

	switch (pass) {
	case -1:
		free(mallocRecord);
		nMallocRecords = 0;
		break;
	case 0:
		MyDynamicMemoryOpen();
		for (k = i = 0; i < nMallocRecords; i++) {
			if (mallocRecord[i].ptr != NULL) {
				fprintf(dLOG, "%3d %p %d\n", mallocRecord[i].id, mallocRecord[i].ptr, mallocRecord[i].len);
				k++;
			}
		}
		fprintf(dLOG, "%d %d\n", k, nMallocRecords);
		fclose(dLOG);
		break;
	case 1:
		for (i = 0; i < nMallocRecords; i++) {
			for (j = i + 1; j < nMallocRecords; j++) {
				if (mallocRecord[i].ptr && mallocRecord[i].ptr == mallocRecord[j].ptr)
					return i;
			}
		}
		break;
	case 2:
		for (i = 0; i < nMallocRecords; i++) {
			if (mallocRecord[i].ptr != NULL)
				return i;
		}
	}
#endif
	return 0;
}

void MyFree(void *address) {
#ifdef _DEBUG
	int i;

	MyDynamicMemoryOpen();
	fprintf(dLOG, "%p MyFree\n", address);
	fclose(dLOG);
	for (i = 0; i < nMallocRecords; i++) {
		if (address == mallocRecord[i].ptr)
			break;
	}
	if (i == nMallocRecords) {
		// bad address
		exit(1);
	}
	mallocRecord[i].ptr = NULL;
#endif
	try {
		free(address);
	}
	// catch (exception& e) {
	// catch (int e) {
	catch (...) {
		;
	}
}

void *MyMalloc(int id, int nBytes) {
	void *address = malloc(nBytes);

	if (address == NULL) {
		// raise flag
		exit(2);
	}
#ifdef _DEBUG
	mallocRecord = (MallocRecord *)realloc(mallocRecord, (nMallocRecords + 1)*sizeof(MallocRecord));
	if (mallocRecord == NULL) {
		// raise flag
		exit(3);
	}
	mallocRecord[nMallocRecords].id = id;
	mallocRecord[nMallocRecords].len = nBytes;
	mallocRecord[nMallocRecords].ptr = address;
	nMallocRecords++;
	MyDynamicMemoryOpen();
	fprintf(dLOG, "%p MyMalloc  %3d %d\n", address, id, nBytes);
	fclose(dLOG);
#endif
	return address;
}

void *MyRealloc(int id, void *address, int nBytes) {
	if (address == NULL) {
		return MyMalloc(id, nBytes);
	}
	void *newAddress = (void *)realloc(address, nBytes);

	if (newAddress == NULL) {
		// realloc failed
		exit(4);
	}
#ifdef _DEBUG
	int i;

	for (i = 0; i < nMallocRecords; i++) {
		if (address == mallocRecord[i].ptr)
			break;
	}
	if (i == nMallocRecords) {
		// bad address
		exit(5);
	}
	mallocRecord[i].ptr = newAddress;
	mallocRecord[i].len = nBytes;
	MyDynamicMemoryOpen();
	fprintf(dLOG, "%p MyRealloc %3d %d %p\n", newAddress, id, nBytes, address);
	fclose(dLOG);
#endif
	return newAddress;
}
