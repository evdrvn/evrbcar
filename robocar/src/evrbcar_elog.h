#ifndef __EVRBCAR_ELOG_H__
#define __EVRBCAR_ELOG_H__

#include <stdarg.h>
#include <evdsptc.h>
#include <evrbcar.h>

#define ELOG_BUFSIZE (64)
#define ELOG_LINELEN (128)

extern bool print_event_log(evdsptc_event_t* event);
extern void push_event_log(const char *fmt, ...);
extern void init_event_log(void);
extern void destroy_event_log(void);

#endif/*__EVRBCAR_ELOG_H__*/
