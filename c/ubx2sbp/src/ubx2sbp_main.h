#ifndef UBX2SBP_MAIN_H
#define UBX2SBP_MAIN_H

#include <stdint.h>

typedef int (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

int ubx2sbp_main(int argc,
                 char **argv,
                 const char *additional_opts_help,
                 readfn_ptr readfn,
                 writefn_ptr writefn,
                 void *context);

#endif
