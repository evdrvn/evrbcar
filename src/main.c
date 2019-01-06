#include <limits.h>
#include <string.h>
#include <wiringPi.h>
#include <evdsptc.h>
#include <drv8830-i2c.h>

#define TICK_NS (16 * 1000 * 1000LL)
#define NS_AS_SEC (1000 * 1000 * 1000LL)
#define TICK_SEC (TICK_NS / (float)NS_AS_SEC)
#define DISTANCE_PER_COUNT (215 / 40.0F) // mm
#define LOG_SIZE (1024)
#define PERIODS_SPEED_AVERAGE (6)

typedef struct posctrl_log {
    float voltage;
    float position;
    float speed;
    float e;
} t_posctrl_log;

typedef struct pidctrl_context {
    float kp;
    float ki;
    float kd;
    float i;
    float d;
    float prev;
    bool unclamped;
    float clamp_lo;
    float clamp_hi;
    float alpha;
} t_pidctrl_context;

typedef struct posctrl_context {
    int pin;
    int last_input;
    unsigned int period;
    int count;
    float position;
    float target_position;
    float speed; 
    float voltage;
    t_pidctrl_context pidctx;
    t_posctrl_log log[LOG_SIZE];
    drv8830_conn_t conn;
} t_posctrl_context;

typedef struct periodic_context {
    long long int max;
    long long int min;
    struct timespec prev;
    bool initial;
    t_posctrl_context posctx[2];
} t_periodic_context;

static void init_pidctrl_context(t_pidctrl_context *ctx, 
        float kp, float ki, float kd, float clamp_lo, float clamp_hi, float smooth){
    ctx->kp = kp;
    ctx->ki = ki;
    ctx->kd = kd;
    ctx->i = 0.0F;
    ctx->d = 0.0F;
    ctx->prev = 0.0F;
    ctx->unclamped = true;
    ctx->clamp_lo = clamp_lo;
    ctx->clamp_hi = clamp_hi;
    ctx->alpha = smooth;
}

static float do_pidctrl(t_pidctrl_context *ctx, float e){
    float u;

    if(ctx->unclamped) ctx->i += TICK_SEC * e;
    ctx->d = ctx->alpha * (e - ctx->prev)/TICK_SEC + (1.0F - ctx->alpha) * ctx->d;
    u = ctx->kp * e + ctx->ki * ctx->i + ctx->kd * ctx->d;
    if(ctx->clamp_lo < u && u < ctx->clamp_hi) ctx->unclamped = true;
    else ctx->unclamped = false;
    ctx->prev = e;

    return u;
}

static long long int timespec_diff(struct timespec *t1, struct timespec *t2){
    return  t2->tv_nsec - t1->tv_nsec + (t2->tv_sec - t1->tv_sec) * NS_AS_SEC;
}

static bool handle_timer(evdsptc_event_t* event){
    struct timespec now;
    struct periodic_context* htctx = (struct periodic_context*)evdsptc_event_getparam(event);
    long long int interval = TICK_NS;
    int i, input;
    bool finalize = false;
    float e;

    clock_gettime(CLOCK_REALTIME, &now);
    if(htctx->initial) htctx->initial = false;
    else interval = timespec_diff(&htctx->prev, &now);
    htctx->prev = now;
    if(interval < htctx->min) htctx->min = interval;
    if(interval > htctx->max) htctx->max = interval;
    for(i = 0; i < 2; i++){
        input = digitalRead(htctx->posctx[i].pin);
        if(htctx->posctx[i].last_input != input){
            htctx->posctx[i].last_input = input; 
            htctx->posctx[i].count++;
            htctx->posctx[i].position = htctx->posctx[i].count * DISTANCE_PER_COUNT;
        }
        htctx->posctx[i].speed = 
            (htctx->posctx[i].position - htctx->posctx[i].log[(htctx->posctx[i].period - PERIODS_SPEED_AVERAGE) % LOG_SIZE].position) / 
            (PERIODS_SPEED_AVERAGE * TICK_SEC);

        //if(htctx->posctx[i].period == 1) htctx->posctx[i].voltage = 16.0F;
        //if(htctx->posctx[i].period == 500) htctx->posctx[i].voltage = 24.0F;
        //if(htctx->posctx[i].period == 1000) htctx->posctx[i].voltage = 0.0F;

        if(htctx->posctx[i].period >= 1) htctx->posctx[i].target_position = htctx->posctx[i].target_position + 10.0F * TICK_SEC;
        e = htctx->posctx[i].target_position - htctx->posctx[i].position;
        htctx->posctx[i].voltage = do_pidctrl(&htctx->posctx[i].pidctx, e);
        
        drv8830_move(&htctx->posctx[i].conn, (int)htctx->posctx[i].voltage);

        htctx->posctx[i].log[htctx->posctx[i].period % LOG_SIZE].voltage = htctx->posctx[i].voltage;
        htctx->posctx[i].log[htctx->posctx[i].period % LOG_SIZE].position = htctx->posctx[i].position;
        htctx->posctx[i].log[htctx->posctx[i].period % LOG_SIZE].speed = htctx->posctx[i].speed;
        htctx->posctx[i].log[htctx->posctx[i].period % LOG_SIZE].e = e;
        htctx->posctx[i].period++;
         
        if(htctx->posctx[i].period >= LOG_SIZE) finalize = true;
    }
    return finalize;
}

int main(void){
    struct periodic_context htctx;
    struct timespec interval = { interval.tv_sec = 0, interval.tv_nsec = TICK_NS};
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t ctx;
    evdsptc_event_t ev;
    int i;

    if(wiringPiSetupGpio() == -1) return 1; 
   
    htctx.initial = true;
    htctx.max = TICK_NS;
    htctx.min = TICK_NS;
    for(i = 0; i < 2; i++){
        htctx.posctx[i].period = 0;
        htctx.posctx[i].count = 0;
        htctx.posctx[i].pin = 23 + i;
        pinMode(htctx.posctx[i].pin, INPUT); 
        htctx.posctx[i].last_input = digitalRead(htctx.posctx[i].pin);
        htctx.posctx[i].voltage = 0.0;;
        htctx.posctx[i].position = 0.0;;
        htctx.posctx[i].speed = 0.0;;
        init_pidctrl_context(&htctx.posctx[i].pidctx, 
                0.223F, 0.007F, 1.784F, -32.0F, 32.0F, 1.0F); 
        memset(&htctx.posctx[i].log, 0, sizeof(htctx.posctx[i].log));
        if(0 > drv8830_open(&htctx.posctx[i].conn, "/dev/i2c-1", DRV8830_ADDRESS + 4 + 2 * i, 2)) return 2; 
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

    for(i = 0; i < LOG_SIZE; i++){
        printf("%f, %f, %f, %f\n",
                htctx.posctx[0].log[(htctx.posctx[0].period + i) % LOG_SIZE].voltage,
                htctx.posctx[0].log[(htctx.posctx[0].period + i) % LOG_SIZE].position,
                htctx.posctx[0].log[(htctx.posctx[0].period + i) % LOG_SIZE].speed,
                htctx.posctx[0].log[(htctx.posctx[0].period + i) % LOG_SIZE].e);
    }

    return 0;
}
