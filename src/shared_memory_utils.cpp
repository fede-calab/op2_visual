// File: src/shared_memory_utils.cpp
#include "op2_visual/shared_memory_utils.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

bool openSharedMemory(double*& sharedValue, sem_t*& mutex)
{
    int shm_fd = shm_open(SHARED_MEMORY_NAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory.\n";
        return false;
    }

    sharedValue = static_cast<double*>(
        mmap(NULL, sizeof(double), PROT_READ, MAP_SHARED, shm_fd, 0));
    if (sharedValue == MAP_FAILED) {
        std::cerr << "Failed to map shared memory.\n";
        return false;
    }

    mutex = sem_open(MUTEX_NAME, 0);
    if (mutex == SEM_FAILED) {
        std::cerr << "Failed to open mutex.\n";
        return false;
    }

    return true;
}

double readSharedMemoryValue(double* sharedValue, sem_t* mutex)
{
    double value = 0.0;
    sem_wait(mutex);
    value = *sharedValue;
    sem_post(mutex);
    return value;
}

void closeSharedMemory(double* sharedValue, sem_t* mutex)
{
    if (sharedValue) {
        munmap(sharedValue, sizeof(double));
    }
    if (mutex) {
        sem_close(mutex);
    }
}