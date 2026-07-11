#ifndef COMMON_STATUS_H
#define COMMON_STATUS_H

typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR = -1,
    STATUS_BUSY = -2,
    STATUS_TIMEOUT = -3,
    STATUS_INVALID_ARG = -4,
    STATUS_NOT_READY = -5,
} status_t;

#endif
