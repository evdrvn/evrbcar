#ifndef __EVRBCAR_ELOG_H__
#define __EVRBCAR_ELOG_H__

#include <stdarg.h>
#include <evdsptc.h>
#include <evrbcar.h>

#define LOG_SIZE (1024)
#define RING_BUFFER_SIZE (32)
#define EVENT_LOG_LENGTH (64)

extern bool print_event_log(evdsptc_event_t* event);
extern void push_event_log(const char *fmt, ...);
extern void init_event_log(void);
extern void destroy_event_log(void);

#endif/*__EVRBCAR_ELOG_H__*/
