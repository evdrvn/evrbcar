#include <signal.h>
#include <evdsptc.h>
#include <pthread.h>
#include "evrbcar.h"

#define TICK_NS (16 * 1000 * 1000LL)

static float speed_rotation[] = {0.0F, 0.4F, 0.8F, 0.4F};
static volatile sig_atomic_t finalize = 0;
static float level = 0.0F;
static unsigned int current_index = 0;

static void signal_handler(int signum) {
    (void)signum;
    finalize = true;
}
static bool periodic_routine(evdsptc_event_t* event){
    t_evrbcar_udp_context* udpctx = (t_evrbcar_udp_context*)evdsptc_event_getparam(event);
    evrbcar_udp_cmd_line_trace(udpctx, level);
    return (bool)finalize;
}

int main(int argc, char *argv[]){
    struct timespec interval = { interval.tv_sec = 0, interval.tv_nsec = TICK_NS};
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t prdth;
    evdsptc_event_t ev;
    unsigned short port = EVRBCAR_UDP_PORT;
    char *address = "127.0.0.1";
    t_evrbcar_udp_context udpctx;
    char c;
    level = speed_rotation[0];

    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
    evdsptc_setmutexattrinitializer(&mutexattr);

    if(argc > 1) address = argv[1]; 
    if(0 > evrbcar_udp_init(&udpctx, address, port)) return -1; 

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    evdsptc_create_periodic(&prdth, NULL, NULL, NULL, &interval);

    th = evdsptc_getthreads(&prdth)[0];
    param.sched_priority = 10;
    if(0 != pthread_setschedparam(th, SCHED_RR, &param)){
        printf("\nwarning : you get better performance to run as root via RT-Preempt.\n");
    }

    evdsptc_event_init(&ev, periodic_routine, (void*)&udpctx, false, NULL);
    evdsptc_post(&prdth, &ev);

    while(!finalize){
        printf("Input key q(uit), or other(changing speed): ");
        c = getchar();
        if(c == 'q' || c == 'Q') break;
        level = speed_rotation[++current_index % 4];
    }
    finalize = true;
    evdsptc_event_waitdone(&ev);
    evrbcar_udp_cmd_line_trace(&udpctx, 0.0F);
    evdsptc_destroy(&prdth, true);

    return 0;
}
