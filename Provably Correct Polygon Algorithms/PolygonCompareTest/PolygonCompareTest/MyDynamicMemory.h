#ifndef MYDYNAMICMEMORY_H

#define MYDYNAMICMEMORY_H

int MyDynamicMemoryCheck(int pass);
void MyFree(void *address);
void *MyMalloc(int id, int nBytes);
void *MyRealloc(int id, void *address, int nBytes);

#endif