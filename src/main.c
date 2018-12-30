#include <wiringPi.h>
#include "evdsptc.h"

#define INTERVAL_NS (16 * 1000 * 1000LL)
#define NS_AS_SEC (1000 * 1000 * 1000LL)

typedef struct position_counter {
    int last_input;
    int count;
    int pin;
} t_position_counter;

typedef struct handle_timer_context {
    long long int max;
    long long int min;
    struct timespec prev;
    bool initial;
    t_position_counter counter[2];
} t_handle_timer_context;

static long long int timespec_diff(struct timespec *t1, struct timespec *t2){
    return  t2->tv_nsec - t1->tv_nsec + (t2->tv_sec - t1->tv_sec) * NS_AS_SEC;
}

static bool handle_timer(evdsptc_event_t* event){
    struct timespec now;
    struct handle_timer_context* htctx = (struct handle_timer_context*)evdsptc_event_getparam(event);
    long long int interval = INTERVAL_NS;
    int i, input;

    clock_gettime(CLOCK_REALTIME, &now);
    if(htctx->initial) htctx->initial = false;
    else interval = timespec_diff(&htctx->prev, &now);
    htctx->prev = now;
    if(interval < htctx->min) htctx->min = interval;
    if(interval > htctx->max) htctx->max = interval;
    for(i = 0; i < 2; i++){
        input = digitalRead(htctx->counter[i].pin);
        if(htctx->counter[i].last_input != input){
           htctx->counter[i].last_input = input; 
           htctx->counter[i].count++;
           printf("counter[%d] is changed to %d\n", i, htctx->counter[i].count);
        }
    }
    return false;
}

int main(void){
    struct handle_timer_context htctx;
    struct timespec interval = { interval.tv_sec = 0, interval.tv_nsec = INTERVAL_NS};
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t ctx;
    evdsptc_event_t ev;
    int i;

    if(wiringPiSetupGpio() == -1) return 1; 
    
    htctx.initial = true;
    htctx.max = INTERVAL_NS;
    htctx.min = INTERVAL_NS;
    for(i = 0; i < 2; i++){
        htctx.counter[i].count = 0;
        htctx.counter[i].pin = 23 + i;
        pinMode(htctx.counter[i].pin, INPUT); 
        htctx.counter[i].last_input = digitalRead(htctx.counter[i].pin);
    }

    evdsptc_setmutexattrinitializer(&mutexattr);
    evdsptc_create_periodic(&ctx, NULL, NULL, NULL, &interval);

    th = evdsptc_getthreads(&ctx)[0];
    param.sched_priority = 80;
    if(0 != pthread_setschedparam(th, SCHED_RR, &param)){
        printf("\nwarning : you get better performance to run as root via RT-Preempt.\n");
    }

    evdsptc_event_init(&ev, handle_timer, (void*)&htctx, false, NULL);
    evdsptc_post(&ctx, &ev);
    evdsptc_event_waitdone(&ev);

    evdsptc_destroy(&ctx, true);

    printf("\n min = %lld, max = %lld\n", htctx.min, htctx.max);
    return 0;
}
