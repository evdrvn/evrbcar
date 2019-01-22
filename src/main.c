#include <limits.h>
#include <string.h>
#include <signal.h>
#include <math.h>
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
#define TURN_RESOLUTION (5.0F)
#define MOTOR_NUM (2)
#define I2C_DEVNAME "/dev/i2c-1"
#define MAX_WS_CLIENTS (2)
#define RING_BUFFER_SIZE (32)
#define EVENT_LOG_LENGTH (64)

static float DIRECTION_CORRECTION[MOTOR_NUM] = {-1.0F, 1.0F};
static int I2C_ADDRESS[MOTOR_NUM] = {0x64, 0x66};
static int ROTENCDR_GPIO_PIN[MOTOR_NUM] = {23, 24};
static int LINESENS_GPIO_PIN[3] = {17, 27, 22};
static char EVENT_LOG_BUFFER[RING_BUFFER_SIZE][64];

typedef enum drive_status{
    STATE_REMOTE_IDLE = 0,
    STATE_REMOTE_DRIVE,
    STATE_REMOTE_DECEL,
    STATE_REMOTE_STOP,
    STATE_LINE_IDLE,
    STATE_LINE_DRIVE,
    STATE_LINE_STOP,
} t_drive_status;

typedef struct posctrl_log {
    float voltage;
    float position;
    float speed;
} t_posctrl_log;

typedef struct posctrl_context {
    int rot_encoder_gpio;
    int last_input;
    unsigned int period;
    int count;
    float position;
    float speed; 
    float voltage;
    float voltage_offset;
    float last_voltage_offset;
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
    unsigned char line_sens;
    int line_sens_continuous_cnt;
    unsigned char last_line_sens;
    int last_line_sens_continuous_cnt;
} t_periodic_context;

typedef struct ws_client {
    struct mg_connection *conn;
    int state;
} t_ws_client;

static struct periodic_context prdctx;
static t_ws_client ws_clients[MAX_WS_CLIENTS];
static volatile sig_atomic_t finalize = 0;

static void printb(unsigned char v, unsigned int width) {
    unsigned int mask = (int)1 << (width - 1);
    do{
        putchar((mask & v) ? '1' : '0');
        mask = mask >> 1;
    }
    while (--width > 0);
}

static long long int timespec_diff(struct timespec *t1, struct timespec *t2){
    return  t2->tv_nsec - t1->tv_nsec + (t2->tv_sec - t1->tv_sec) * NS_AS_SEC;
}

static void move_forward_to(t_periodic_context* prdctx, float distance){
    prdctx->move_remaining += distance * 2.0F; 
}

static void turn_at_offset(t_periodic_context* prdctx, float voltage_offset){
    bool target = 0; 
    if(voltage_offset < 0.0F) target = 1;
    else target = 0;
    prdctx->posctx[target].last_voltage_offset = prdctx->posctx[target].voltage_offset; 
    prdctx->posctx[target].voltage_offset = fabs(voltage_offset) * -1.0F; 
    prdctx->posctx[!target].voltage_offset = 0.0F; 
    if(prdctx->posctx[target].last_voltage_offset != prdctx->posctx[target].voltage_offset){ 
        printf(" -> voltage_offset = {%f, %f}\n", 
                prdctx->posctx[0].voltage_offset, 
                prdctx->posctx[1].voltage_offset); 
    }
}

static float curve_to_offset(float level){
    return level * (DRIVE_VOLTAGE - DECEL_VOLTAGE) / (TURN_RESOLUTION - 1.0F);
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
    
    pthread_mutex_lock(&prdctx->mutex);
    
    input = 0; 
    for(i = 0; i < 3; i++){
        input = input << 1;
        input = input | (1 & (!digitalRead(LINESENS_GPIO_PIN[i])));
    }
    prdctx->last_line_sens = prdctx->line_sens;
    prdctx->line_sens = input;
 
    for(i = 0; i < 2; i++){
        input = digitalRead(prdctx->posctx[i].rot_encoder_gpio);
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
    
    if(prdctx->state < STATE_LINE_IDLE){
        if(prdctx->move_remaining > STOP_START * 2.0F) prdctx->state = STATE_REMOTE_DRIVE;
        if(prdctx->state > STATE_REMOTE_IDLE && prdctx->state < STATE_REMOTE_DECEL && prdctx->move_remaining < DECEL_START * 2.0F) prdctx->state = STATE_REMOTE_DECEL;
        if(prdctx->state > STATE_REMOTE_IDLE && prdctx->state < STATE_REMOTE_STOP && prdctx->move_remaining < STOP_START * 2.0F){
            prdctx->state = STATE_REMOTE_STOP;
            prdctx->coin_count = 5;
        }
        if(prdctx->state == STATE_REMOTE_STOP && prdctx->coin_count-- <= 0) prdctx->state = STATE_REMOTE_IDLE;
    }else{
        if(prdctx->last_line_sens != prdctx->line_sens){
            prdctx->last_line_sens_continuous_cnt = prdctx->line_sens_continuous_cnt;
            prdctx->line_sens_continuous_cnt = 0;
            if(prdctx->state == STATE_LINE_IDLE) prdctx->state = STATE_LINE_DRIVE;
            printf("line_sens changed to :%d = ", prdctx->line_sens);
            printb(prdctx->line_sens, 3);
            printf("\n");
        }

        if(prdctx->line_sens == 0){
            if(prdctx->line_sens_continuous_cnt > 30) prdctx->state = STATE_LINE_IDLE;
            else if(prdctx->last_line_sens == 4) turn_at_offset(prdctx, curve_to_offset(-5.0F));
            else if(prdctx->last_line_sens == 1) turn_at_offset(prdctx, curve_to_offset(5.0F));
        }
        else if(prdctx->line_sens == 1) turn_at_offset(prdctx, curve_to_offset(4.0F));
        else if(prdctx->line_sens == 3){
            if(prdctx->last_line_sens == 1) turn_at_offset(prdctx, curve_to_offset(-2.0F));
            else turn_at_offset(prdctx, curve_to_offset(-1.0F));
        }
        else if(prdctx->line_sens == 4) turn_at_offset(prdctx, curve_to_offset(-4.0F));
        else if(prdctx->line_sens == 6){
            if(prdctx->last_line_sens == 4) turn_at_offset(prdctx, curve_to_offset(2.0F));
            else turn_at_offset(prdctx, curve_to_offset(1.0F));
        }
        else{
            if(prdctx->last_line_sens_continuous_cnt > 0){
                prdctx->last_line_sens_continuous_cnt -= 2;
                if(prdctx->last_line_sens == 3) turn_at_offset(prdctx, curve_to_offset(-1.0F));
                else if(prdctx->last_line_sens == 6) turn_at_offset(prdctx, curve_to_offset(1.0F));
                else turn_at_offset(prdctx, curve_to_offset(0.0F));
            }
            else turn_at_offset(prdctx, curve_to_offset(0.0F));
        }
        prdctx->line_sens_continuous_cnt++;
    }

    for(i = 0; i < 2; i++){
        if(prdctx->state == STATE_REMOTE_DRIVE) prdctx->posctx[i].voltage = DRIVE_VOLTAGE;
        else if(prdctx->state == STATE_REMOTE_DECEL) prdctx->posctx[i].voltage = DECEL_VOLTAGE;
        else if(prdctx->state == STATE_REMOTE_STOP) prdctx->posctx[i].voltage = 0.0F;
        else if(prdctx->state == STATE_LINE_IDLE) prdctx->posctx[i].voltage = 0.0F;
        else if(prdctx->state == STATE_LINE_DRIVE) prdctx->posctx[i].voltage = DRIVE_VOLTAGE;
        else if(prdctx->state == STATE_LINE_STOP) prdctx->posctx[i].voltage = 0.0F;

        v = (prdctx->posctx[i].voltage + prdctx->posctx[i].voltage_offset);
        if(fabs(v) < DECEL_VOLTAGE) v = 0.0F;
        v *= DIRECTION_CORRECTION[i];
        drv8830_move(&prdctx->posctx[i].conn, v);

        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].voltage = prdctx->posctx[i].voltage;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].position = prdctx->posctx[i].position;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].speed = prdctx->posctx[i].speed;

        prdctx->posctx[i].period++;
    }
    
    pthread_mutex_unlock(&prdctx->mutex);
   
    return (bool)finalize;
}

static void createresponse(char* buf, unsigned int size){
    (void)buf;
    (void)size;
    return;
}

static int handler(struct mg_connection *conn, void *ignored)
{
    char msg[BUFSIZ];
    unsigned long len;
    (void)ignored;

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
    (void)cbdata;
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
    (void)cbdata;

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
    (void)conn;
    (void)bits;
    (void)cbdata;

    if(len >= BUFSIZ) len = BUFSIZ - 1;
    memcpy(buf, data, len);
    buf[len] = 0;
    printf("%s\n", buf);
    fflush(stdout);
    
    cmd = strtok_r(buf, ":", &c);
    val = strtok_r(NULL, ",", &c);
    v = atof(val);

    if(0 == strncmp("move_to", cmd, 6)) {
        move_forward_to(&prdctx, v * STOP_START * 3.0F * 2.0F);
    }
    else if(0 == strncmp("turn", cmd, 4)) {
        turn_at_offset(&prdctx, curve_to_offset(v));
    }
    return 1;
}


static void WebSocketCloseHandler(const struct mg_connection *conn, void *cbdata)
{
    struct mg_context *ctx = mg_get_context(conn);
    t_ws_client *client = mg_get_user_connection_data(conn);
    (void)cbdata;

    mg_lock_context(ctx);
    client->state = 0;
    client->conn = NULL;
    mg_unlock_context(ctx);
}

void signal_handler(int signum) {
    (void)signum;
    finalize = 1;
}

int main(int argc, char *argv[]){
    struct timespec interval = { interval.tv_sec = 0, interval.tv_nsec = TICK_NS};
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t ctx;
    evdsptc_event_t ev;
    int i,j;
    struct mg_context *mgctx;
    bool dump = false;

    const char *options[] = { 
        "document_root", "./htdocs",
        "request_timeout_ms", "10000",
        "websocket_timeout_ms", "900000"
    };
    
    mg_init_library(0);
    mgctx = mg_start(NULL, 0, options);
    
    if(wiringPiSetupGpio() == -1) return 1; 
    
    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);

    if(argc > 1 && 0 == index(argv[1], '-')){
        if(NULL != index(argv[1], 'l')){
            printf("line tracing mode\n");
            prdctx.state = STATE_LINE_DRIVE;
        }
        if(NULL != index(argv[1], 'd')){
            dump = true; 
        }
    }

    prdctx.initial = true;
    prdctx.max = TICK_NS;
    prdctx.min = TICK_NS;
    prdctx.line_sens = 7;
    prdctx.last_line_sens = 7;
    pthread_mutex_init(&prdctx.mutex, &mutexattr);

    for(i = 0; i < 3; i++){
        pinMode(LINESENS_GPIO_PIN[i], INPUT); 
    }

    for(i = 0; i < 2; i++){
        prdctx.posctx[i].period = 0;
        prdctx.posctx[i].count = 0;
        prdctx.posctx[i].rot_encoder_gpio = ROTENCDR_GPIO_PIN[i];
        pinMode(prdctx.posctx[i].rot_encoder_gpio, INPUT); 
        prdctx.posctx[i].last_input = digitalRead(prdctx.posctx[i].rot_encoder_gpio);
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

    for(j = 0; j < MOTOR_NUM; j++){
        drv8830_move(&prdctx.posctx[j].conn, 0.0F);
        if(dump){
            for(i = 0; i < LOG_SIZE; i++){
                printf("%f, %f, %f\n",
                        prdctx.posctx[j].log[(prdctx.posctx[j].period + i) % LOG_SIZE].voltage,
                        prdctx.posctx[j].log[(prdctx.posctx[j].period + i) % LOG_SIZE].position,
                        prdctx.posctx[j].log[(prdctx.posctx[j].period + i) % LOG_SIZE].speed);
            }
        }
    }

    return 0;
}
