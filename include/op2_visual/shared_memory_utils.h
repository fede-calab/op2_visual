// File: include/op2_visual/shared_memory_utils.h
#ifndef SHARED_MEMORY_UTILS_H
#define SHARED_MEMORY_UTILS_H

#include <semaphore.h>

// Define your shared memory and semaphore names
#ifndef SHARED_MEMORY_NAME
  #define SHARED_MEMORY_NAME "/shared_double_value"
#endif

#ifndef MUTEX_NAME
  #define MUTEX_NAME "/shared_double_mutex"
#endif

// Open the shared memory in read-only mode; return pointer and semaphore handle
bool openSharedMemory(double*& sharedValue, sem_t*& mutex);

// Safely read a double value from the shared memory
double readSharedMemoryValue(double* sharedValue, sem_t* mutex);

// Close (unmap/unlink) shared memory and semaphore
void closeSharedMemory(double* sharedValue, sem_t* mutex);

#endif // SHARED_MEMORY_UTILS_H