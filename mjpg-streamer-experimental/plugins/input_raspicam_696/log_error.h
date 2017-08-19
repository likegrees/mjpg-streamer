#ifndef LOG_ERROR_H
#define LOG_ERROR_H

#include <syslog.h>

#define LOG_STATUS(...) \
    { char _bf[1024] = {0}; \
      snprintf(_bf, sizeof(_bf)-1, __VA_ARGS__); \
      fprintf(stderr, "%s", _bf); \
      syslog(LOG_INFO, "%s", _bf); }

#define LOG_ERROR(...) \
    { char _bf[1024] = {0}; \
      snprintf(_bf, sizeof(_bf)-1, __VA_ARGS__); \
      fprintf(stderr, "%s", _bf); \
      syslog(LOG_INFO, "%s+%d: %s", __FILE__, __LINE__, _bf); }

#endif
