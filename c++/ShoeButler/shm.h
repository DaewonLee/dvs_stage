#ifndef CSHM_H_
#define CSHM_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <errno.h>
#include <termios.h>
#include <sys/time.h>

class CShm
{
public:
	CShm(const char *name, int SIZE_);
	~CShm();
	
	//const char *name;
	int SIZE;
	int shm_fd;
	char *shm_base;
	int write(int32_t *src, int size_);
	int write8(int8_t *src, int size_);
	int read(int32_t *des, int size_);
	int read8(int8_t *des, int size_);
	
};

#endif
