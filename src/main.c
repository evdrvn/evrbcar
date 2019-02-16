#include <limits.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <signal.h>
#include <math.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <wiringPi.h>
#include <evdsptc.h>
#include <drv8830-i2c.h>
#include <civetweb.h>

#define TICK_NS (16 * 1000 * 1000LL)
#define NS_AS_SEC (1000 * 1000 * 1000LL)
#define TICK_SEC (TICK_NS / (float)NS_AS_SEC)
#define DISTANCE_PER_COUNT (215.0F / 40.0F) // mm
#define LOG_SIZE (1024)
#define PERIODS_SPEED_AVERAGE (16)
#define DRIVE_VOLTAGE_MAX (4.80F)
#define DECEL_VOLTAGE (1.80F)
#define STOP_START (DISTANCE_PER_COUNT * 2.0F)
#define PERIODS_STOPPING (5)
#define DECEL_START (STOP_START * 3.0F)
#define CTRL_RESOLUTION (5.0F)
#define MOTOR_NUM (2)
#define LINESENS_NUM (3)
#define I2C_DEVNAME "/dev/i2c-1"
#define MAX_WS_CLIENTS (1)
#define RING_BUFFER_SIZE (32)
#define EVENT_LOG_LENGTH (64)
#define UDP_PORT (65001)

static float DIRECTION_CORRECTION[MOTOR_NUM] = {-1.0F, 1.0F};
static int I2C_ADDRESS[MOTOR_NUM] = {0x65, 0x60};
static int ROTENCDR_GPIO_PIN[MOTOR_NUM] = {23, 24};
static int LINESENS_GPIO_PIN[3] = {22, 27, 17};
static char EVENT_LOG_BUFFER[RING_BUFFER_SIZE][64];
static unsigned int event_log_index = 0;
static pthread_mutex_t elogmtx;
static evdsptc_context_t elogth;

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
    t_drive_status state;
    float voltage;
    int count;
    float speed;
} t_posctrl_log;

typedef struct posctrl_context {
    int rot_encoder_gpio;
    int last_input;
    unsigned int period;
    int count;
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
    float target_voltage;
    int stopping_count;
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

static bool print_event_log(evdsptc_event_t* event){
    printf("%s\n", EVENT_LOG_BUFFER[(unsigned int)evdsptc_event_getparam(event)]);
    fflush(stdout);
    return true;
}

static void push_event_log(const char *fmt, ...){
    va_list ap;
    unsigned int index;
    evdsptc_event_t *ev; 

    pthread_mutex_lock(&elogmtx);
    index = event_log_index % RING_BUFFER_SIZE;
    event_log_index++;
    pthread_mutex_unlock(&elogmtx);
    
    va_start(ap, fmt);
    vsnprintf(EVENT_LOG_BUFFER[index], EVENT_LOG_LENGTH - 1, fmt, ap);
    va_end(ap);

    ev = malloc(sizeof(evdsptc_event_t));
    if(ev != NULL){
        evdsptc_event_init(ev, print_event_log, (void*)index, true, NULL);
        evdsptc_post(&elogth, ev);
    }
}

static long long int timespec_diff(struct timespec *t1, struct timespec *t2){
    return  t2->tv_nsec - t1->tv_nsec + (t2->tv_sec - t1->tv_sec) * NS_AS_SEC;
}

static void move_forward_to(t_periodic_context* prdctx, float distance){
    prdctx->move_remaining = distance * 2.0F; 
    push_event_log(" -> state = %d, remaining = %f", prdctx->state, prdctx->move_remaining);
}

static void turn_at_offset(t_periodic_context* prdctx, float voltage_offset){
    bool target = 0; 

    if(voltage_offset < 0.0F) target = 1;
    else target = 0;
    prdctx->posctx[target].last_voltage_offset = prdctx->posctx[target].voltage_offset; 
    prdctx->posctx[target].voltage_offset = fabs(voltage_offset) * -1.0F; 
    prdctx->posctx[!target].voltage_offset = 0.0F; 

    if(prdctx->posctx[target].last_voltage_offset != prdctx->posctx[target].voltage_offset){ 
        push_event_log(" -> voltage_offset = {%f, %f}", 
                prdctx->posctx[0].voltage_offset, 
                prdctx->posctx[1].voltage_offset); 
    }
}

static float get_curve_voltage_offset(float target_voltage, float level){
    return level * (target_voltage - DECEL_VOLTAGE) / (CTRL_RESOLUTION - 1.0F);
}

static float get_drive_voltage(float level){
    if(level < 1.0F) return 0.0F;
    return level * (DRIVE_VOLTAGE_MAX - DECEL_VOLTAGE) / (CTRL_RESOLUTION - 1.0F) + DECEL_VOLTAGE;
}

static void line_tracing(t_periodic_context* prdctx){
    if(prdctx->last_line_sens != prdctx->line_sens){
        prdctx->last_line_sens_continuous_cnt = prdctx->line_sens_continuous_cnt;
        prdctx->line_sens_continuous_cnt = 0;
        if(prdctx->state == STATE_LINE_IDLE && prdctx->line_sens != 0 && prdctx->line_sens != 7){
            prdctx->state = STATE_LINE_DRIVE;
            //prdctx->target_voltage = DRIVE_VOLTAGE_MAX;
        }
        push_event_log("line_sens changed to %d", prdctx->line_sens);
    }

    if(prdctx->line_sens == 0){
        if(prdctx->line_sens_continuous_cnt > 30) prdctx->state = STATE_LINE_IDLE;
        else if(prdctx->last_line_sens == 4) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -CTRL_RESOLUTION));
        else if(prdctx->last_line_sens == 1) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, CTRL_RESOLUTION));
    }
    else if(prdctx->line_sens == 1) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, CTRL_RESOLUTION * 0.8F));
    else if(prdctx->line_sens == 3){
        if(prdctx->last_line_sens == 1){
            if(prdctx->last_line_sens_continuous_cnt > 0){
                turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -CTRL_RESOLUTION * 0.4F));
                prdctx->last_line_sens_continuous_cnt -= 2;
            }
            else turn_at_offset(prdctx, 0.0F);
        }
        else turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, CTRL_RESOLUTION * 0.2F));
    }
    else if(prdctx->line_sens == 4) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -CTRL_RESOLUTION * 0.8F));
    else if(prdctx->line_sens == 6){
        if(prdctx->last_line_sens == 4){
            if(prdctx->last_line_sens_continuous_cnt > 0){
                turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, CTRL_RESOLUTION * 0.4F));
                prdctx->last_line_sens_continuous_cnt -= 2;
            }
            else turn_at_offset(prdctx, 0.0F);
        }
        else turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -CTRL_RESOLUTION * 0.2F));
    }
    else{
        if(prdctx->last_line_sens_continuous_cnt > 0){
            if(prdctx->last_line_sens_continuous_cnt > 0){
                if(prdctx->last_line_sens == 3) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -CTRL_RESOLUTION * 0.2F));
                else if(prdctx->last_line_sens == 6) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, CTRL_RESOLUTION * 0.2F));
                prdctx->last_line_sens_continuous_cnt -= 2;
            }
            else turn_at_offset(prdctx, 0.0F);
        }
        else turn_at_offset(prdctx, 0.0F);
    }
    prdctx->line_sens_continuous_cnt++;
}

static bool udp_routine(evdsptc_event_t* event){
    struct sockaddr clitSockAddr;
    int sock = (int)evdsptc_event_getparam(event);
    unsigned int sockaddrLen = sizeof(clitSockAddr);
    char buffer[BUFSIZ];
    
    while(0 > recvfrom(sock, buffer, BUFSIZ, 0, &clitSockAddr, &sockaddrLen)){

    }
    return (bool)finalize;
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
    for(i = 0; i < LINESENS_NUM; i++){
        input = input << 1;
        input = input | (1 & (!digitalRead(LINESENS_GPIO_PIN[i])));
    }
    prdctx->last_line_sens = prdctx->line_sens;
    prdctx->line_sens = input;
 
    for(i = 0; i < MOTOR_NUM; i++){
        input = digitalRead(prdctx->posctx[i].rot_encoder_gpio);
        if(prdctx->posctx[i].last_input != input){
            prdctx->posctx[i].last_input = input; 
            if(prdctx->posctx[i].voltage >= 0.0F) prdctx->posctx[i].count++;
            else prdctx->posctx[i].count--;
            if(prdctx->move_remaining > 0.0F && prdctx->move_remaining != FLT_MAX) prdctx->move_remaining -= DISTANCE_PER_COUNT;
        }
        prdctx->posctx[i].speed = 
            (prdctx->posctx[i].count - prdctx->posctx[i].log[(prdctx->posctx[i].period - PERIODS_SPEED_AVERAGE) % LOG_SIZE].count) * DISTANCE_PER_COUNT / (PERIODS_SPEED_AVERAGE * TICK_SEC);
    }   
    
    if(prdctx->state < STATE_LINE_IDLE){
        if(prdctx->move_remaining > STOP_START * 2.0F) prdctx->state = STATE_REMOTE_DRIVE;
        if(prdctx->state > STATE_REMOTE_IDLE && prdctx->state < STATE_REMOTE_DECEL && prdctx->move_remaining < DECEL_START * 2.0F) prdctx->state = STATE_REMOTE_DECEL;
        if(prdctx->state > STATE_REMOTE_IDLE && prdctx->state < STATE_REMOTE_STOP && prdctx->move_remaining < STOP_START * 2.0F){
            prdctx->state = STATE_REMOTE_STOP;
            prdctx->stopping_count = PERIODS_STOPPING;
        }
        if(prdctx->state == STATE_REMOTE_STOP && prdctx->stopping_count-- <= 0) prdctx->state = STATE_REMOTE_IDLE;
    }else{
        line_tracing(prdctx);
    }

    for(i = 0; i < 2; i++){
        if(prdctx->state == STATE_REMOTE_DRIVE) prdctx->posctx[i].voltage = prdctx->target_voltage; 
        else if(prdctx->state == STATE_REMOTE_DECEL) prdctx->posctx[i].voltage = DECEL_VOLTAGE;
        else if(prdctx->state == STATE_REMOTE_STOP){
            prdctx->posctx[i].voltage = 0.0F;
            turn_at_offset(prdctx, 0.0F);
        }
        else if(prdctx->state == STATE_LINE_IDLE) prdctx->posctx[i].voltage = 0.0F;
        else if(prdctx->state == STATE_LINE_DRIVE) prdctx->posctx[i].voltage = prdctx->target_voltage;
        else if(prdctx->state == STATE_LINE_STOP){
            prdctx->posctx[i].voltage = 0.0F;
            turn_at_offset(prdctx, 0.0F);
        }

        v = prdctx->posctx[i].voltage; 
        if(v > 0.0F) v += prdctx->posctx[i].voltage_offset;
        if(fabs(v) < DECEL_VOLTAGE) v = 0.0F;
        v *= DIRECTION_CORRECTION[i];
        drv8830_move(&prdctx->posctx[i].conn, v);

        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].state = prdctx->state;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].voltage = prdctx->posctx[i].voltage;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].count = prdctx->posctx[i].count;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].speed = prdctx->posctx[i].speed;

        prdctx->posctx[i].period++;
    }
    
    pthread_mutex_unlock(&prdctx->mutex);
   
    return (bool)finalize;
}

void cmd_move_to(float distance){
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state >= STATE_LINE_IDLE){
        prdctx.state = STATE_REMOTE_IDLE; 
        turn_at_offset(&prdctx, 0.0F);
    }
    prdctx.target_voltage = DRIVE_VOLTAGE_MAX;
    move_forward_to(&prdctx, distance);
    pthread_mutex_unlock(&prdctx.mutex);
}

void cmd_move_at(float target_voltage){
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state >= STATE_LINE_IDLE){
        prdctx.state = STATE_REMOTE_IDLE; 
        turn_at_offset(&prdctx, 0.0F);
    }
    move_forward_to(&prdctx, FLT_MAX);
    prdctx.target_voltage = get_drive_voltage(target_voltage);
    pthread_mutex_unlock(&prdctx.mutex);
}

void cmd_line_trace(float target_voltage){
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state < STATE_LINE_IDLE) turn_at_offset(&prdctx, 0.0F);
    prdctx.state = STATE_LINE_IDLE;
    prdctx.target_voltage = get_drive_voltage(target_voltage);
    pthread_mutex_unlock(&prdctx.mutex);
}

void cmd_turn(float level){
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state < STATE_LINE_IDLE) turn_at_offset(&prdctx, get_curve_voltage_offset(prdctx.target_voltage, level));
    pthread_mutex_unlock(&prdctx.mutex);
}

static void createresponse(char* buf, unsigned int size){
    (void)buf;
    (void)size;
    return;
}

static int HttpHandler(struct mg_connection *conn, void *ignored)
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
    char *valstr;
    float v = 0.0F;
    (void)conn;
    (void)cbdata;

    switch (((unsigned char)bits) & 0x0F) {
    case WEBSOCKET_OPCODE_TEXT:
        if(len >= BUFSIZ) len = BUFSIZ - 1;
        memcpy(buf, data, len);
        buf[len] = 0;

        cmd = strtok_r(buf, ":", &c);
        if(cmd != NULL){
            valstr = strtok_r(NULL, ",", &c);
            if(valstr != NULL){
                v = atof(valstr);
                valstr = strtok_r(NULL, ",", &c);
            }
        }
        else cmd = "";

        if(0 == strncmp("move_to", cmd, 6)) {
            push_event_log("%s", buf);
            cmd_move_to(pow(2.0F, v) * 10.0F);
        }
        else if(0 == strncmp("move_at", cmd, 6)) {
            push_event_log("%s", buf);
            cmd_move_at(v);
        }
        else if(0 == strncmp("line_trace", cmd, 10)) {
            push_event_log("%s", buf);
            cmd_line_trace(v);
        }
        else if(0 == strncmp("turn", cmd, 4)) {
            push_event_log("%s", buf);
            cmd_turn(v);
        }
    default:
        break;
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

    pthread_mutex_lock(&prdctx.mutex);
    move_forward_to(&prdctx, 0.0F);
    prdctx.target_voltage = 0.0F;
    pthread_mutex_unlock(&prdctx.mutex);
}

void signal_handler(int signum) {
    (void)signum;
    finalize = 1;
}

void sockaddr_init (const char *address, unsigned short port, struct sockaddr *sockaddr) {

    struct sockaddr_in sockaddr_in;
    sockaddr_in.sin_family = AF_INET;

    if (inet_aton(address, &sockaddr_in.sin_addr) == 0) {
        if (strcmp(address, "") == 0 ) {
            sockaddr_in.sin_addr.s_addr = htonl(INADDR_ANY);
        } else {
            fprintf(stderr, "Invalid IP Address.\n");
            exit(EXIT_FAILURE);
        }
    }

    if (port == 0) {
        fprintf(stderr, "invalid port number.\n");
        exit(EXIT_FAILURE);
    }
    sockaddr_in.sin_port = htons(port);

    *sockaddr = *((struct sockaddr *)&sockaddr_in);
}

int main(int argc, char *argv[]){
    struct timespec interval = { interval.tv_sec = 0, interval.tv_nsec = TICK_NS};
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t prdth;
    evdsptc_context_t udpth;
    evdsptc_event_t ev;
    int i,j;
    struct mg_context *mgctx;
    bool dump = false;
    const char *address = "";
    unsigned short port = UDP_PORT;
    struct sockaddr servSockAddr;
    int server_sock;

    const char *options[] = { 
        "document_root", "./htdocs",
        "request_timeout_ms", "5000",
        "websocket_timeout_ms", "15000"
    };
    
    mg_init_library(0);
    mgctx = mg_start(NULL, 0, options);
    
    if(wiringPiSetupGpio() == -1) return 1; 
    
    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
    evdsptc_setmutexattrinitializer(&mutexattr);
    pthread_mutex_init(&elogmtx, &mutexattr);
    evdsptc_create(&elogth, NULL, NULL, NULL);

    prdctx.initial = true;
    prdctx.max = TICK_NS;
    prdctx.min = TICK_NS;
    prdctx.line_sens = 7;
    prdctx.target_voltage = 0.0F;
    prdctx.last_line_sens = 7;

    if(argc > 1 && argv[1] == index(argv[1], '-')){
        if(NULL != index(argv[1], 'l')){
            push_event_log("line tracing mode");
            cmd_line_trace(DRIVE_VOLTAGE_MAX);
        }
        if(NULL != index(argv[1], 'd')){
            dump = true; 
        }
    }

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
        prdctx.posctx[i].speed = 0.0F;
        memset(&prdctx.posctx[i].log, 0, sizeof(prdctx.posctx[i].log));
        if(0 > drv8830_open(&prdctx.posctx[i].conn, I2C_DEVNAME, I2C_ADDRESS[i], 2)) return 2; 
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    server_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_init(address, port, &servSockAddr);
    if (bind(server_sock, &servSockAddr, sizeof(servSockAddr)) < 0) {
        perror("bind() failed.");
        exit(EXIT_FAILURE);
    }

    evdsptc_create_periodic(&prdth, NULL, NULL, NULL, &interval);
    evdsptc_create_periodic(&udpth, NULL, NULL, NULL, &interval);

    th = evdsptc_getthreads(&prdth)[0];
    param.sched_priority = 80;
    if(0 != pthread_setschedparam(th, SCHED_RR, &param)){
        printf("\nwarning : you get better performance to run as root via RT-Preempt.\n");
    }

    evdsptc_event_init(&ev, periodic_routine, (void*)&prdctx, false, NULL);
    evdsptc_post(&prdth, &ev);

    evdsptc_event_init(&ev, udp_routine, (void*)server_sock, false, NULL);
    evdsptc_post(&udpth, &ev);

    mg_set_request_handler(mgctx, "/evrbcar", HttpHandler, NULL);
    mg_set_websocket_handler(mgctx,
            "/websocket",
            WebSocketConnectHandler,
            WebSocketReadyHandler,
            WebsocketDataHandler,
            WebSocketCloseHandler,
            0);
    
    evdsptc_event_waitdone(&ev);
    evdsptc_destroy(&prdth, true);
    usleep(500 * 1000);
    evdsptc_destroy(&elogth, true);

    for(j = 0; j < MOTOR_NUM; j++){
        drv8830_move(&prdctx.posctx[j].conn, 0.0F);
    }
    if(dump){
        for(i = 0; i < LOG_SIZE; i++){
            printf("%d, %f, %d, %f, %f, %d, %f\n",
                    prdctx.posctx[0].log[(prdctx.posctx[0].period + i) % LOG_SIZE].state,
                    prdctx.posctx[0].log[(prdctx.posctx[0].period + i) % LOG_SIZE].voltage,
                    prdctx.posctx[0].log[(prdctx.posctx[0].period + i) % LOG_SIZE].count,
                    prdctx.posctx[0].log[(prdctx.posctx[0].period + i) % LOG_SIZE].speed,
                    prdctx.posctx[1].log[(prdctx.posctx[1].period + i) % LOG_SIZE].voltage,
                    prdctx.posctx[1].log[(prdctx.posctx[1].period + i) % LOG_SIZE].count,
                    prdctx.posctx[1].log[(prdctx.posctx[1].period + i) % LOG_SIZE].speed
                    );
        }
    }

    printf("periodic interval min/max [ns] = {%lld, %lld}\n", prdctx.min, prdctx.max);

    return 0;
}
