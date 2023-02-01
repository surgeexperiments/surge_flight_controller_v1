// baselibc stdout wrapper

#include <stdio.h>

static size_t stdio_write(FILE *instance, const char *bp, size_t n)
{
	// TODO: write to UART
	return n;
}

static struct File_methods stdio_methods = {
        &stdio_write, NULL
};

static struct File _stdout = {
        &stdio_methods
};

static struct File _stderr = {
        &stdio_methods
};

FILE* const stdout = &_stdout;
FILE* const stderr = &_stderr;

