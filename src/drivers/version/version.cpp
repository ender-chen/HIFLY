#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>

#include <systemlib/err.h>

extern "C" __EXPORT int version_main(int argc, char *argv[]);

int
version_main(int argc, char *argv[]){
	float version;

	version = 4.22;
	errx(1, "%.2lf", (double)version);
}
