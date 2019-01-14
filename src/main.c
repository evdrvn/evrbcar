#include <limits.h>
#include <string.h>
#include <signal.h>
#include <wiringPi.h>
#include <evdsptc.h>
#include <drv8830-i2c.h>
#include <civetweb.h>

#define TICK_NS (16 * 1000 * 1000LL)
#define NS_AS_SEC (1000 * 1000 * 1000LL)
#define TICK_SEC (TICK_NS / (float)NS_AS_SEC)
#define DISTANCE_PER_COUNT (215 / 40.0F) // mm
#define LOG_SIZE (1024)
#define PERIODS_SPEED_AVERAGE (16)
#define DRIVE_VOLTAGE (3.00F)
#define DECEL_VOLTAGE (1.80F)
#define STOP_START (DISTANCE_PER_COUNT * 2.0F)
#define DECEL_START (STOP_START * 3.0F)
#define MOTOR_NUM (2)
#define I2C_DEVNAME "/dev/i2c-1"
#define MAX_WS_CLIENTS (2)

static float DIRECTION_CORRECTION[MOTOR_NUM] = {-1.0F, 1.0F};
static int I2C_ADDRESS[MOTOR_NUM] = {0x64, 0x66};
static int GPIO_PIN[MOTOR_NUM] = {23, 24};

typedef enum drive_status{
    STATE_COAST = 0,
    STATE_DRIVE,
    STATE_DECEL,
    STATE_STOP,
} t_drive_status;

typedef struct posctrl_log {
    float voltage;
    float position;
    float speed;
} t_posctrl_log;

typedef struct posctrl_context {
    int pin;
    int last_input;
    unsigned int period;
    int count;
    float position;
    float speed; 
    float voltage;
    float voltage_offset;
    t_posctrl_log log[LOG_SIZE];
    drv8830_conn_t conn;
} t_posctrl_context;

typedef struct periodic_context {
    long long int max;
    long long int min;
    struct timespec prev;
    bool initial;
    float move_remaining;
    t_drive_status state;
    int coin_count;
    t_posctrl_context posctx[2];
    pthread_mutex_t mutex;
} t_periodic_context;

typedef struct ws_client {
    struct mg_connection *conn;
    int state;
} t_ws_client;

static struct periodic_context prdctx;
static t_ws_client ws_clients[MAX_WS_CLIENTS];
static volatile sig_atomic_t finalize = 0;

static long long int timespec_diff(struct timespec *t1, struct timespec *t2){
    return  t2->tv_nsec - t1->tv_nsec + (t2->tv_sec - t1->tv_sec) * NS_AS_SEC;
}

static void move_forward_to(t_periodic_context* prdctx, float distance){
    prdctx->move_remaining += distance * 2.0F; 
}

static void turn(t_periodic_context* prdctx, float voltage_offset){
    prdctx->posctx[0].voltage_offset = voltage_offset / -2.0F; 
    prdctx->posctx[1].voltage_offset = voltage_offset /  2.0F; 
}

static bool periodic_routine(evdsptc_event_t* event){
    struct timespec now;
    t_periodic_context* prdctx = (struct periodic_context*)evdsptc_event_getparam(event);
    long long int interval = TICK_NS;
    int i, input;
    float v;

    clock_gettime(CLOCK_REALTIME, &now);
    if(prdctx->initial) prdctx->initial = false;
    else interval = timespec_diff(&prdctx->prev, &now);
    prdctx->prev = now;
    if(interval < prdctx->min) prdctx->min = interval;
    if(interval > prdctx->max) prdctx->max = interval;
    for(i = 0; i < 2; i++){
        input = digitalRead(prdctx->posctx[i].pin);
        if(prdctx->posctx[i].last_input != input){
            prdctx->posctx[i].last_input = input; 
            prdctx->posctx[i].count++;
            prdctx->posctx[i].position = prdctx->posctx[i].count * DISTANCE_PER_COUNT;
            if(prdctx->move_remaining > 0.0F) prdctx->move_remaining -= DISTANCE_PER_COUNT;
        }
        prdctx->posctx[i].speed = 
            (prdctx->posctx[i].position - prdctx->posctx[i].log[(prdctx->posctx[i].period - PERIODS_SPEED_AVERAGE) % LOG_SIZE].position) / 
            (PERIODS_SPEED_AVERAGE * TICK_SEC);
    }   
    
    if(prdctx->move_remaining > STOP_START * 2.0F) prdctx->state = STATE_DRIVE;
    if(prdctx->state > STATE_COAST && prdctx->state < STATE_DECEL && prdctx->move_remaining < DECEL_START * 2.0F) prdctx->state = STATE_DECEL;
    if(prdctx->state > STATE_COAST && prdctx->state < STATE_STOP && prdctx->move_remaining < STOP_START * 2.0F){
        prdctx->state = STATE_STOP;
        prdctx->coin_count = 5;
    }
    if(prdctx->state == STATE_STOP && prdctx->coin_count-- <= 0) prdctx->state = STATE_COAST;
    
    for(i = 0; i < 2; i++){

        if(prdctx->state == STATE_DRIVE) prdctx->posctx[i].voltage = DRIVE_VOLTAGE;
        else if(prdctx->state == STATE_DECEL) prdctx->posctx[i].voltage = DECEL_VOLTAGE;
        else if(prdctx->state == STATE_STOP){
            prdctx->posctx[i].voltage = 0.0F;
        }

        if(prdctx->state > STATE_STOP) v = (prdctx->posctx[i].voltage + prdctx->posctx[i].voltage_offset) * DIRECTION_CORRECTION[i];
        else v = prdctx->posctx[i].voltage;
        drv8830_move(&prdctx->posctx[i].conn, v);

        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].voltage = prdctx->posctx[i].voltage;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].position = prdctx->posctx[i].position;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].speed = prdctx->posctx[i].speed;

        prdctx->posctx[i].period++;
    }
    return (bool)finalize;
}

static void createresponse(char* buf, unsigned int size){
    pthread_mutex_lock(&prdctx.mutex);
    pthread_mutex_unlock(&prdctx.mutex);
    return;
}

static int handler(struct mg_connection *conn, void *ignored)
{
    char msg[BUFSIZ];
    unsigned long len;

    createresponse(msg, BUFSIZ);

    len = (unsigned long)strlen(msg);
    mg_printf(conn,
            "HTTP/1.1 200 OK\r\n"
            "Content-Length: %lu\r\n"
            "Content-Type: application/json\r\n"
            "Connection: close\r\n\r\n",
            len);

    mg_write(conn, msg, len);

    return 200;
}

static int WebSocketConnectHandler(const struct mg_connection *conn, void *cbdata)
{
    struct mg_context *ctx = mg_get_context(conn);
    int reject = 1;
    int i;

    mg_lock_context(ctx);
    for (i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_clients[i].conn == NULL) {
            ws_clients[i].conn = (struct mg_connection *)conn;
            ws_clients[i].state = 1;
            mg_set_user_connection_data(ws_clients[i].conn,
                    (void *)(ws_clients + i));
            reject = 0;
            break;
        }
    }
    mg_unlock_context(ctx);
    return reject;
}

static void WebSocketReadyHandler(struct mg_connection *conn, void *cbdata)
{
    char msg[BUFSIZ];
    createresponse(msg, BUFSIZ);
    t_ws_client *client = mg_get_user_connection_data(conn);

    mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, msg, strlen(msg));

    client->state = 2;
}


static int WebsocketDataHandler(struct mg_connection *conn, int bits, char *data, size_t len, void *cbdata)
{
    char buf[BUFSIZ]; 
    char *cmd = NULL;
    char *c;
    char *val;
    float v;

    if(len >= BUFSIZ) len = BUFSIZ - 1;
    memcpy(buf, data, len);
    buf[len] = '\0';
    printf("%s", buf);
    
    cmd = strtok_r(buf, ":", &c);
    val = strtok_r(NULL, ",", &c);
    v = atof(val);

    if(0 == strncmp("move_to", cmd, 6)) {
        printf(" -> val = %f\n", v);
        move_forward_to(&prdctx, v * STOP_START * 3.0F * 2.0F);
    }
    else if(0 == strncmp("turn", cmd, 4)) {
        printf(" -> val = %f\n", v);
        turn(&prdctx, v * 0.6F);
    }else printf("\n");
    return 1;
}


static void WebSocketCloseHandler(const struct mg_connection *conn, void *cbdata)
{
    struct mg_context *ctx = mg_get_context(conn);
    t_ws_client *client = mg_get_user_connection_data(conn);

    mg_lock_context(ctx);
    client->state = 0;
    client->conn = NULL;
    mg_unlock_context(ctx);
}

void signal_handler(int signum) {
    finalize = 1;
}

int main(void){
    struct timespec interval = { interval.tv_sec = 0, interval.tv_nsec = TICK_NS};
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t ctx;
    evdsptc_event_t ev;
    int i,j;
    struct mg_context *mgctx;
    const char *options[] = { 
        "document_root", "./htdocs",
        "request_timeout_ms", "10000",
        "websocket_timeout_ms", "30000"
    };
    
    mg_init_library(0);
    mgctx = mg_start(NULL, 0, options);

    if(wiringPiSetupGpio() == -1) return 1; 

    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);

    prdctx.initial = true;
    prdctx.max = TICK_NS;
    prdctx.min = TICK_NS;
    pthread_mutex_init(&prdctx.mutex, &mutexattr);
    for(i = 0; i < 2; i++){
        prdctx.posctx[i].period = 0;
        prdctx.posctx[i].count = 0;
        prdctx.posctx[i].pin = GPIO_PIN[i];
        pinMode(prdctx.posctx[i].pin, INPUT); 
        prdctx.posctx[i].last_input = digitalRead(prdctx.posctx[i].pin);
        prdctx.posctx[i].voltage = 0.0F;
        prdctx.posctx[i].voltage_offset = 0.0F;
        prdctx.posctx[i].position = 0.0F;
        prdctx.posctx[i].speed = 0.0F;
        memset(&prdctx.posctx[i].log, 0, sizeof(prdctx.posctx[i].log));
        if(0 > drv8830_open(&prdctx.posctx[i].conn, I2C_DEVNAME, I2C_ADDRESS[i], 2)) return 2; 
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    evdsptc_setmutexattrinitializer(&mutexattr);
    evdsptc_create_periodic(&ctx, NULL, NULL, NULL, &interval);

    th = evdsptc_getthreads(&ctx)[0];
    param.sched_priority = 80;
    if(0 != pthread_setschedparam(th, SCHED_RR, &param)){
        printf("\nwarning : you get better performance to run as root via RT-Preempt.\n");
    }

    evdsptc_event_init(&ev, periodic_routine, (void*)&prdctx, false, NULL);
    evdsptc_post(&ctx, &ev);

    mg_set_request_handler(mgctx, "/evrbcar", handler, NULL);
    mg_set_websocket_handler(mgctx,
            "/websocket",
            WebSocketConnectHandler,
            WebSocketReadyHandler,
            WebsocketDataHandler,
            WebSocketCloseHandler,
            0);
    
    evdsptc_event_waitdone(&ev);
    evdsptc_destroy(&ctx, true);

    for(j = 0; i < MOTOR_NUM; j++){
        drv8830_move(&prdctx.posctx[i].conn, 0.0F);
        for(i = 0; i < LOG_SIZE; i++){
            printf("%f, %f, %f\n",
                prdctx.posctx[j].log[(prdctx.posctx[j].period + i) % LOG_SIZE].voltage,
                prdctx.posctx[j].log[(prdctx.posctx[j].period + i) % LOG_SIZE].position,
                prdctx.posctx[j].log[(prdctx.posctx[j].period + i) % LOG_SIZE].speed);
        }
    }

    return 0;
}
