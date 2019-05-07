#include "shm.h"

CShm::CShm(const char *name, int SIZE_)
{
	SIZE = SIZE_;
	int initMem[SIZE];

	for (int i=0; i<SIZE; i++)
	{
		initMem[i] = 2000;
	}

	shm_fd = shm_open(name, O_CREAT | O_RDWR | O_TRUNC, 0666);
	if (shm_fd == -1) {
		printf("cons: Shared memory failed: %s\n", strerror(errno));
		exit(1);
	}
	ftruncate(shm_fd, SIZE);
	shm_base = static_cast<char*>(mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
	if (shm_base == MAP_FAILED) {
		printf("cons: Map failed: %s\n", strerror(errno));
		exit(1);
	}
	memcpy(shm_base, initMem, SIZE);
}

int CShm::read(int32_t *des, int size)
{
	memcpy(des, shm_base, size);
}

int CShm::read8(int8_t *des, int size)
{
	memcpy(des, shm_base, size);
}

int CShm::write(int32_t *src, int size)
{
	memcpy(shm_base, src, size);
}

int CShm::write8(int8_t *src, int size)
{
	memcpy(shm_base, src, size);
}

CShm::~CShm()
{
	/* remove the mapped memory segment from the address space of the process */
	if (munmap(shm_base, SIZE) == -1) {
		printf("prod: Unmap failed: %s\n", strerror(errno));
		exit(1);
	}
	/* close the shared memory segment as if it was a file */
	if (close(shm_fd) == -1) {
		printf("prod: Close failed: %s\n", strerror(errno));
		exit(1);
  	}
}
