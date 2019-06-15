#include "evrbcar_elog.h"

static char EVENT_LOG_BUFFER[ELOG_BUFSIZE][ELOG_LINELEN];
static volatile unsigned int event_log_index = 0;
static pthread_mutex_t elogmtx;
static evdsptc_context_t elogth;

bool print_event_log(evdsptc_event_t* event){
    printf("%s\n", EVENT_LOG_BUFFER[(unsigned int)evdsptc_event_getparam(event)]);
    fflush(stdout);
    return true;
}

void push_event_log(const char *fmt, ...){
    va_list ap;
    unsigned int index;
    evdsptc_event_t *ev; 

    pthread_mutex_lock(&elogmtx);
    index = event_log_index % ELOG_BUFSIZE;
    event_log_index++;
    pthread_mutex_unlock(&elogmtx);
    
    va_start(ap, fmt);
    vsnprintf(EVENT_LOG_BUFFER[index], ELOG_LINELEN - 1, fmt, ap);
    va_end(ap);

    ev = malloc(sizeof(evdsptc_event_t));
    if(ev != NULL){
        evdsptc_event_init(ev, print_event_log, (void*)index, true, NULL);
        evdsptc_post(&elogth, ev);
    }
}

void init_event_log(void){
    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
    evdsptc_setmutexattrinitializer(&mutexattr);

    pthread_mutex_init(&elogmtx, &mutexattr);
    evdsptc_create(&elogth, NULL, NULL, NULL);
}

void destroy_event_log(void){
    evdsptc_destroy(&elogth, true);
}
