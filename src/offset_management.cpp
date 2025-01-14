// Author: Federico Calabrese

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>

#define SHARED_MEMORY_NAME "/shared_double_value"
#define MUTEX_NAME "/shared_double_mutex"

int main() {
    // Open shared memory 
    int shm_fd = shm_open(SHARED_MEMORY_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to create shared memory" << std::endl;
        return 1;
    }

    // Configure the size of the shared memory segment
    if (ftruncate(shm_fd, sizeof(double)) == -1) {
        std::cerr << "Failed to set shared memory size" << std::endl;
        return 1;
    }

    // Map the shared memory 
    double* shared_value = (double*)mmap(NULL, sizeof(double), 
                                         PROT_READ | PROT_WRITE, 
                                         MAP_SHARED, shm_fd, 0);
    if (shared_value == MAP_FAILED) {
        std::cerr << "Failed to map shared memory" << std::endl;
        return 1;
    }

    // Open the named semaphore for mutex
    sem_t* mutex = sem_open(MUTEX_NAME, O_CREAT, 0666, 1);
    if (mutex == SEM_FAILED) {
        std::cerr << "Failed to create mutex" << std::endl;
        return 1;
    }

    // Input loop
    double input_value;
    while (true) {
        std::cout << "Enter a value between 0 and 1 (or enter a negative value to exit): ";
        std::cin >> input_value;

        // Exit condition
        if (input_value < 0) {
            break;
        }

        // Validate input
        if (input_value < 0 || input_value > 1) {
            std::cout << "Invalid input. Value must be between 0 and 1." << std::endl;
            continue;
        }

        // Acquire mutex before writing
        sem_wait(mutex);
        *shared_value = input_value;
        sem_post(mutex);
    }

    // Cleanup
    munmap(shared_value, sizeof(double));
    close(shm_fd);
    sem_close(mutex);
    sem_unlink(MUTEX_NAME);
    shm_unlink(SHARED_MEMORY_NAME);

    return 0;
}