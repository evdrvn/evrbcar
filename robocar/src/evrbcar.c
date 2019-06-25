#include <limits.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <signal.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <wiringPi.h>
#include <evdsptc.h>
#include <drv8830-i2c.h>
#include <civetweb.h>
#include <bno055-i2c.h>
#include <vl53l0x_api.h>

#include "evrbcar.h"
#include "evrbcar_elog.h"
#include "evrbcar_imu.h"
#include "evrbcar_tof.h"

#define TICK_NS (16 * 1000 * 1000LL)
#define NS_AS_SEC (1000 * 1000 * 1000LL)
#define TICK_SEC (TICK_NS / (float)NS_AS_SEC)
#define PULSE_PER_REVOLUTION (40.0F)
#define WHEEL_CIRCUMFERENCE (0.215F) //m
#define WHEEL_RADIUS (WHEEL_CIRCUMFERENCE / M_PI / 2.0F) //m
#define WHEEL_TREAD (0.132) //m
#define TOF_MEASURE_OFFSET (30) //mm
#define DRIVE_VOLTAGE_MAX (6.40F)
#define DRIVE_VOLTAGE_MIN (1.20F)
#define DECEL_VOLTAGE (1.80F)
#define STOP_START (WHEEL_CIRCUMFERENCE / PULSE_PER_REVOLUTION * 2.0F)
#define PERIODS_STOPPING (5)
#define DECEL_START (STOP_START * 3.0F)
#define MOTOR_NUM (2)
#define LINESENS_NUM (3)
#define I2C_DEVNAME "/dev/i2c-1"
#define MAX_WS_CLIENTS (1)
#define PERIODS_UDP_TIMEOUT (16)
#define EPS (0.00001F)
#define LOG_SIZE (1024)
#define SCAN_SPEED_THRESHOLD (2.2F * 3.0F)
#define SCAN_SPEED_LPF_COEF (1.0F / 8.0F)
#define SCAN_SPEED_TIMEOUT (8)

static float DIRECTION_CORRECTION[MOTOR_NUM] = {-1.0F, 1.0F};
static int I2C_ADDRESS[MOTOR_NUM] = {0x64, 0x66};
static int ROTENCDR_GPIO_PIN[MOTOR_NUM] = {23, 24};
static int LINESENS_GPIO_PIN[3] = {22, 27, 17};
static volatile int udp_timeout_count= -1;
static volatile bool enable_ext_linesens = false;
static volatile int ext_linesens = 7;
static bno055_conn_t imuctx;
static VL53L0X_Dev_t tofctx;
static t_evrbcar_udp_context scan_udpctx = {0};
 
typedef enum drive_status{
    STATE_REMOTE_IDLE = 0,
    STATE_REMOTE_DRIVE,
    STATE_REMOTE_DECEL,
    STATE_REMOTE_STOP,
    STATE_REMOTE_SCAN,
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
    float speed;
    float rotspeed;
    float position[3];
    float delta_yaw_lpf;
    pthread_mutex_t mutex;
    char linesens;
    char last_linesens;
    int linesens_continuous_cnt;
    int last_linesens_continuous_cnt;
    uint32_t tof;
    int count;
    int skip_count;
    float scan_angle;
    float scan_angle_prev;
    int scan_stage;
    int scan_periods;
    t_scan_data scanbuf;
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
    prdctx->move_remaining = distance * 2.0F; 
    //push_event_log(" -> state = %d, remaining = %f", prdctx->state, prdctx->move_remaining);
}

static void turn_at_offset(t_periodic_context* prdctx, float voltage_offset){
    int decel_target = 0; 

    if(voltage_offset < 0.0F) decel_target = 0;
    else decel_target = 1;
    prdctx->posctx[decel_target].last_voltage_offset = prdctx->posctx[decel_target].voltage_offset; 
    if(fabs(prdctx->target_voltage) > EPS){  
        prdctx->posctx[decel_target].voltage_offset = fabs(voltage_offset) * -1.0F; 
        prdctx->posctx[!decel_target].voltage_offset = 0.0F; 
    }else{
        prdctx->posctx[decel_target].voltage_offset = fabs(voltage_offset) * -0.5F; 
        prdctx->posctx[!decel_target].voltage_offset = fabs(voltage_offset) * 0.5F; 
    }
#if 0
    if(prdctx->posctx[decel_target].last_voltage_offset != prdctx->posctx[decel_target].voltage_offset){ 
       push_event_log(" -> voltage_offset = %f {%f, %f}", 
                voltage_offset, 
                prdctx->posctx[0].voltage_offset, 
                prdctx->posctx[1].voltage_offset); 
    }
#endif
}

static float get_curve_voltage_offset(float target_voltage, float level){
    if(fabs(target_voltage) < EPS) return level * (DRIVE_VOLTAGE_MAX - DRIVE_VOLTAGE_MIN);
    return level * (fabs(target_voltage) - DRIVE_VOLTAGE_MIN);
}

static float get_drive_voltage(float level){
    float v = 0.0F;
    if(fabs(level) < EPS) return v;
    v = fabs(level) * (DRIVE_VOLTAGE_MAX - DECEL_VOLTAGE) + DECEL_VOLTAGE;
    if(level < 0.0F) v = v * -1.0F;
    return v;
}

static void line_tracing(t_periodic_context* prdctx){
    if(prdctx->last_linesens != prdctx->linesens){
        prdctx->last_linesens_continuous_cnt = prdctx->linesens_continuous_cnt;
        prdctx->linesens_continuous_cnt = 0;
        if(prdctx->state == STATE_LINE_IDLE && prdctx->linesens != 0 && prdctx->linesens != 7){
            prdctx->state = STATE_LINE_DRIVE;
        }
        push_event_log("linesens changed to %d (%d, %f)", prdctx->linesens, prdctx->state, prdctx->target_voltage);
    }

    if(prdctx->linesens == 0){
        if(prdctx->linesens_continuous_cnt > 30) prdctx->state = STATE_LINE_IDLE;
        else if(prdctx->last_linesens == 4) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -1.0F));
        else if(prdctx->last_linesens == 1) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, 1.0F));
    }
    else if(prdctx->linesens == 1) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, 0.8F));
    else if(prdctx->linesens == 3){
        if(prdctx->last_linesens == 1){
            if(prdctx->last_linesens_continuous_cnt > 0){
                turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -0.4F));
                prdctx->last_linesens_continuous_cnt -= 2;
            }
            else turn_at_offset(prdctx, 0.0F);
        }
        else turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, 0.2F));
    }
    else if(prdctx->linesens == 4) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -0.8F));
    else if(prdctx->linesens == 6){
        if(prdctx->last_linesens == 4){
            if(prdctx->last_linesens_continuous_cnt > 0){
                turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, 0.4F));
                prdctx->last_linesens_continuous_cnt -= 2;
            }
            else turn_at_offset(prdctx, 0.0F);
        }
        else turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -0.2F));
    }
    else{
        if(prdctx->last_linesens_continuous_cnt > 0){
            if(prdctx->last_linesens_continuous_cnt > 0){
                if(prdctx->last_linesens == 3) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, -0.2F));
                else if(prdctx->last_linesens == 6) turn_at_offset(prdctx, get_curve_voltage_offset(prdctx->target_voltage, 0.2F));
                prdctx->last_linesens_continuous_cnt -= 2;
            }
            else turn_at_offset(prdctx, 0.0F);
        }
        else turn_at_offset(prdctx, 0.0F);
    }
    prdctx->linesens_continuous_cnt++;
}

static void cmd_stop(){
    //push_event_log("stop: ");
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state < STATE_REMOTE_SCAN){
        move_forward_to(&prdctx, 0.0F);
        prdctx.target_voltage = 0.0F;

        pthread_mutex_unlock(&prdctx.mutex);
    }
}

static void cmd_move_to(float distance){
    float v = DRIVE_VOLTAGE_MAX;
    //push_event_log("move_to: %f", distance);
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state < STATE_REMOTE_SCAN){
        if(prdctx.state >= STATE_LINE_IDLE){
            prdctx.state = STATE_REMOTE_IDLE; 
            turn_at_offset(&prdctx, 0.0F);
        }
        if(distance < 0.0F) v = v * -1.0F;
        prdctx.target_voltage = v;
        move_forward_to(&prdctx, distance);
    } 
    pthread_mutex_unlock(&prdctx.mutex);
}

static void cmd_move_at(float level){
    //push_event_log("move_at: %f", level);
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state < STATE_REMOTE_SCAN){
        if(prdctx.state >= STATE_LINE_IDLE){
            prdctx.state = STATE_REMOTE_IDLE; 
            turn_at_offset(&prdctx, 0.0F);
        }
        move_forward_to(&prdctx, FLT_MAX);
        prdctx.target_voltage = get_drive_voltage(level);
    }
    pthread_mutex_unlock(&prdctx.mutex);
}

static void cmd_line_trace(float level, int linesens){
    static float vlv;
    if(vlv != level || (linesens > 0 && ext_linesens != linesens)){
        push_event_log("line_trace: %f, %d", level, ext_linesens);
    }
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state < STATE_LINE_IDLE){
        turn_at_offset(&prdctx, 0.0F);
        prdctx.state = STATE_LINE_IDLE;
    }
    prdctx.target_voltage = get_drive_voltage(level);
    if(0 > linesens) enable_ext_linesens = false;
    else enable_ext_linesens = true;
    vlv = level;
    ext_linesens = linesens;
    pthread_mutex_unlock(&prdctx.mutex);
}

static void cmd_turn_impl(float level){
    //push_event_log("turn: %f", level);
    if(prdctx.state < STATE_LINE_IDLE) turn_at_offset(&prdctx, get_curve_voltage_offset(prdctx.target_voltage, level));
}

static void cmd_turn(float level){
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state < STATE_REMOTE_SCAN) cmd_turn_impl(level);
    pthread_mutex_unlock(&prdctx.mutex);
}

static void cmd_connect(struct sockaddr_in *clitSockAddr){
    struct sockaddr_in addr;
    char clitaddr[256];
    int sock;
    t_evrbcar_cmd_response res; 
   
    inet_ntop(AF_INET, &clitSockAddr->sin_addr, clitaddr, sizeof(clitaddr));
    push_event_log("connect: from client %s:%d", clitaddr, clitSockAddr->sin_port);

    sock = socket(AF_INET, SOCK_DGRAM, 0);

    addr.sin_family = AF_INET;
    addr.sin_port = htons(CLIENT_UDP_PORT);
    addr.sin_addr = clitSockAddr->sin_addr;

    res.mode = EVRBCAR_CMD_CONNECT; 
    sendto(sock, (char *)&res, sizeof(t_evrbcar_cmd_response), 0, (struct sockaddr *)&addr, sizeof(addr));
}

static void cmd_scan(void){
    push_event_log("scan: state = %d", prdctx.state);
    pthread_mutex_lock(&prdctx.mutex);
    if(prdctx.state == STATE_REMOTE_IDLE){
        prdctx.state = STATE_REMOTE_SCAN; 
        prdctx.target_voltage = 0.0F;
        prdctx.scan_stage = 1;
        prdctx.scan_periods = 0;
        prdctx.scanbuf.num = 0;
        prdctx.scan_angle_prev= 0.0F;
        prdctx.delta_yaw_lpf = 0.0F;
        push_event_log("scan: started, state = %d, stage = %d", prdctx.state, prdctx.scan_stage);
    }
    pthread_mutex_unlock(&prdctx.mutex);
}

static bool udp_routine(evdsptc_event_t* event){
    struct sockaddr_in clitSockAddr;
    int sock = (int)evdsptc_event_getparam(event);
    unsigned int sockaddrLen = sizeof(clitSockAddr);
    char buffer[BUFSIZ];
    t_evrbcar_cmd_request *req;
   
    if(udp_timeout_count >= 0) udp_timeout_count++;
    while(0 < recvfrom(sock, buffer, BUFSIZ, 0, (struct sockaddr *)&clitSockAddr, &sockaddrLen)){
       req = (t_evrbcar_cmd_request*)buffer;
       switch(req->mode){
       case EVRBCAR_CMD_MOVE_TO:
           cmd_move_to(req->fvalue[0]);
           break;
       case EVRBCAR_CMD_MOVE_AT:
           cmd_move_at(req->fvalue[0]);
           break;
       case EVRBCAR_CMD_LINE_TRACE:
           cmd_line_trace(req->fvalue[0], -1);
           break;
       case EVRBCAR_CMD_EXT_LINE_TRACE:
           cmd_line_trace(req->fvalue[0], req->ivalue[0]);
           break;
       case EVRBCAR_CMD_TURN:
           cmd_turn(req->fvalue[0]);
           break;
       case EVRBCAR_CMD_MOVE_TURN:
           cmd_move_at(req->fvalue[0]);
           cmd_turn(req->fvalue[1]);
           break;
       case EVRBCAR_CMD_CONNECT:
           cmd_connect(&clitSockAddr);
           break;
       case EVRBCAR_CMD_SCAN:
           cmd_scan();
           break;
       default:
           break;
       }
       udp_timeout_count = 0;
    }
    if(udp_timeout_count > PERIODS_UDP_TIMEOUT){
        udp_timeout_count = -1;
        cmd_stop();
    }
    return (bool)finalize;
}

static bool periodic_routine(evdsptc_event_t* event){
    struct timespec now;
    t_periodic_context* prdctx = (struct periodic_context*)evdsptc_event_getparam(event);
    long long int interval = TICK_NS;
    int i, input;
    float v; 
    double euler[3];
    int error;
    uint32_t tof;
    float delta_yaw;

    clock_gettime(CLOCK_REALTIME, &now);
    if(prdctx->initial) {
        prdctx->initial = false;
        prdctx->count = 0;
        prdctx->skip_count = 0;
    }
    else interval = timespec_diff(&prdctx->prev, &now);
    prdctx->prev = now;
    if(interval < prdctx->min) prdctx->min = interval;
    if(interval > prdctx->max) prdctx->max = interval;

    error = evrbcar_imu_measure(&imuctx, euler);
    evrbcar_tof_measure(&tofctx, &tof);
    tof += TOF_MEASURE_OFFSET;
    
    pthread_mutex_lock(&prdctx->mutex);
  
    prdctx->tof = tof;
    prdctx->scanbuf.scan = false;
   
    /* noise canceling */
    delta_yaw = euler[2] - prdctx->position[2];
    if(delta_yaw < -180.0F) delta_yaw += 360.0F;
    if(delta_yaw >  180.0F) delta_yaw -= 360.0F;
    if(error <= 0) prdctx->scan_stage = -1;
    else if(fabs(delta_yaw) > (SCAN_SPEED_TIMEOUT + SCAN_SPEED_TIMEOUT / 3.0F * prdctx->skip_count)){
        if(prdctx->skip_count >= SCAN_SPEED_TIMEOUT){
            prdctx->scan_stage = -1; 
            prdctx->skip_count = 0;
            prdctx->position[2] = euler[2];
        }else{
            prdctx->skip_count++;
            prdctx->position[2] += prdctx->delta_yaw_lpf; 
            //push_event_log("skip!! %f, %f, %d, %d", euler[2], prdctx->position[2], prdctx->skip_count, error);
        }
    }else{
        prdctx->skip_count = 0;
        prdctx->delta_yaw_lpf = prdctx->delta_yaw_lpf * (1.0F - SCAN_SPEED_LPF_COEF) + delta_yaw * SCAN_SPEED_LPF_COEF;
        prdctx->position[2] = euler[2];
    }

    prdctx->last_linesens = prdctx->linesens;
    if(!enable_ext_linesens){
        input = 0; 
        for(i = 0; i < LINESENS_NUM; i++){
            input = input << 1;
            input = input | (1 & (!digitalRead(LINESENS_GPIO_PIN[i])));
        }
        prdctx->linesens = input;
    }else prdctx->linesens = ext_linesens;

    for(i = 0; i < MOTOR_NUM; i++){
        input = digitalRead(prdctx->posctx[i].rot_encoder_gpio);
        if(prdctx->posctx[i].last_input != input){
            prdctx->posctx[i].last_input = input; 
            if(prdctx->posctx[i].voltage >= 0.0F) prdctx->posctx[i].count++;
            else prdctx->posctx[i].count--;
            if(prdctx->move_remaining > 0.0F && prdctx->move_remaining != FLT_MAX) prdctx->move_remaining -= (WHEEL_CIRCUMFERENCE / PULSE_PER_REVOLUTION);
        }

        prdctx->posctx[i].speed = 
            (prdctx->posctx[i].count - prdctx->posctx[i].log[(prdctx->posctx[i].period - 1) % LOG_SIZE].count) / PULSE_PER_REVOLUTION / TICK_SEC * 2.0F * M_PI;
    }   

    prdctx->speed = WHEEL_RADIUS / 2.0F * (prdctx->posctx[0].speed + prdctx->posctx[1].speed); 
    //prdctx->rotspeed = WHEEL_RADIUS / WHEEL_TREAD * (prdctx->posctx[0].speed - prdctx->posctx[1].speed); 
    if(fabs(prdctx->target_voltage) > EPS){
        prdctx->position[0] = prdctx->position[0] + prdctx->speed * cos(DEG2RAD(prdctx->position[2])) * TICK_SEC;
        prdctx->position[1] = prdctx->position[1] + prdctx->speed * sin(DEG2RAD(prdctx->position[2])) * TICK_SEC;
    }
    if(prdctx->state == STATE_REMOTE_SCAN){
        if(prdctx->scan_stage == 1){
            prdctx->skip_count = 0;
            prdctx->scanbuf.odom[0] = prdctx->position[0];
            prdctx->scanbuf.odom[1] = prdctx->position[1];
            prdctx->scanbuf.odom[2] = prdctx->position[2];

            prdctx->scan_angle = 0.0F;
            prdctx->scan_stage = 2;
        }
        else prdctx->scan_angle += delta_yaw;

        cmd_turn_impl(0.5F); 
        
        push_event_log("stage = %d, angle = %f, %f, dy = %f, %f, %d" , prdctx->scan_stage, 
                prdctx->scan_angle, 
                prdctx->position[2], 
                delta_yaw,
                prdctx->delta_yaw_lpf,
                prdctx->skip_count 
                );
        prdctx->scan_angle_prev = prdctx->scan_angle;

        if(prdctx->scan_periods++ >= SCAN_BUFSIZE) prdctx->scan_stage = -1;
        
        if(prdctx->scan_stage == 2){
            if(prdctx->scan_angle >= 165.0F){
                prdctx->scanbuf.start_angle = prdctx->scan_angle;
                prdctx->scanbuf.range[prdctx->scanbuf.num++] = prdctx->tof;
                //push_event_log("scan: num = %d, stage = %d, angle = %f, range = %d", prdctx->scanbuf.num - 1, prdctx->scan_stage, angle, prdctx->scanbuf.range[prdctx->scanbuf.num - 1]);
                prdctx->scan_stage = 3;
                prdctx->scan_periods = 0;
            }
        }else if(prdctx->scan_stage == 3){
            prdctx->scanbuf.range[prdctx->scanbuf.num++] = prdctx->tof;
            //push_event_log("scan: num = %d, stage = %d, angle = %f, range = %d", prdctx->scanbuf.num - 1, prdctx->scan_stage, angle, prdctx->scanbuf.range[prdctx->scanbuf.num - 1]);
            if(prdctx->scan_angle >= 555.0F) prdctx->scan_stage = 4;
        }else if(prdctx->scan_stage == 4){
            prdctx->scanbuf.end_angle = prdctx->scan_angle;
            prdctx->scanbuf.range[prdctx->scanbuf.num++] = prdctx->tof;
            //push_event_log("scan: num = %d, stage = %d, angle = %f, range = %d", prdctx->scanbuf.num - 1, prdctx->scan_stage, angle, prdctx->scanbuf.range[prdctx->scanbuf.num - 1]);
            prdctx->scan_stage = 5;
            prdctx->scan_periods = 0;
            prdctx->scanbuf.scan = true;
        }else if(prdctx->scan_stage == 5){
            if(prdctx->scan_angle >= 700.0F){
                prdctx->scan_stage = 6;
                prdctx->state = STATE_REMOTE_STOP;
            }
        }else{
            prdctx->scan_stage = -1;
            prdctx->state = STATE_REMOTE_STOP;
        }
    }else if(prdctx->state < STATE_LINE_IDLE){
        //push_event_log("state = %d, voltage = %f", prdctx->state, prdctx->target_voltage);
        if(prdctx->move_remaining > STOP_START * 2.0F) prdctx->state = STATE_REMOTE_DRIVE;
        if(prdctx->state == STATE_REMOTE_DRIVE && fabs(prdctx->target_voltage) < EPS) prdctx->state = STATE_REMOTE_STOP;
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
        if(prdctx->state == STATE_REMOTE_DRIVE || prdctx->state == STATE_REMOTE_SCAN) prdctx->posctx[i].voltage = prdctx->target_voltage; 
        else if(prdctx->state == STATE_REMOTE_DECEL) prdctx->posctx[i].voltage = DECEL_VOLTAGE;
        else if(prdctx->state == STATE_REMOTE_STOP || prdctx->state == STATE_REMOTE_IDLE){
            prdctx->posctx[i].voltage = 0.0F;
        }
        else if(prdctx->state == STATE_LINE_IDLE) prdctx->posctx[i].voltage = 0.0F;
        else if(prdctx->state == STATE_LINE_DRIVE) prdctx->posctx[i].voltage = prdctx->target_voltage;
        else if(prdctx->state == STATE_LINE_STOP){
            prdctx->posctx[i].voltage = 0.0F;
            turn_at_offset(prdctx, 0.0F);
        }

        v = prdctx->posctx[i].voltage; 
        if(v >= EPS){
            v += prdctx->posctx[i].voltage_offset;
        }else if(v <= -1.0F * EPS){
            v -= prdctx->posctx[i].voltage_offset;
        }else if(prdctx->posctx[i].voltage_offset > EPS){
            v = v + DECEL_VOLTAGE;
            v += prdctx->posctx[i].voltage_offset;
        }else if(prdctx->posctx[i].voltage_offset < -1.0F * EPS){
            v = v - DECEL_VOLTAGE;
            v += prdctx->posctx[i].voltage_offset;
        }

        if(fabs(v) < DECEL_VOLTAGE) v = 0.0F;
        v *= DIRECTION_CORRECTION[i];
        drv8830_move(&prdctx->posctx[i].conn, v);

        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].state = prdctx->state;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].voltage = prdctx->posctx[i].voltage;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].count = prdctx->posctx[i].count;
        prdctx->posctx[i].log[prdctx->posctx[i].period % LOG_SIZE].speed = prdctx->posctx[i].speed;

        prdctx->posctx[i].period++;
    }

    if(prdctx->scanbuf.scan){
        evrbcar_udp_send_scan_data(&scan_udpctx, &prdctx->scanbuf, prdctx->scanbuf.num);
    }else if(prdctx->state != STATE_REMOTE_SCAN && prdctx->count++ % 64 == 0){
        prdctx->scanbuf.odom[0] = prdctx->position[0];
        prdctx->scanbuf.odom[1] = prdctx->position[1];
        prdctx->scanbuf.odom[2] = prdctx->position[2];

        evrbcar_udp_send_scan_data(&scan_udpctx, &prdctx->scanbuf, 0);
        //push_event_log("odom = (%f, %f, %f), tof = %d"
        //        , prdctx->position[0], prdctx->position[1], prdctx->position[2], prdctx->tof);
    }
     
    pthread_mutex_unlock(&prdctx->mutex);

    return (bool)finalize;
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
            cmd_move_to(2.0F * v * 10.0F);
        }
        else if(0 == strncmp("move_at", cmd, 6)) {
            cmd_move_at(v);
        }
        else if(0 == strncmp("line_trace", cmd, 10)) {
            cmd_line_trace(v, -1);
        }
        else if(0 == strncmp("turn", cmd, 4)) {
            cmd_turn(v);
        }
        break;
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

    cmd_stop();
}

static void signal_handler(int signum) {
    (void)signum;
    finalize = true;
}

extern char *optarg;
extern int optind, opterr, optopt;
int main(int argc, char *argv[]){
    int opt;
    struct timespec interval = { interval.tv_sec = 0, interval.tv_nsec = TICK_NS};
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t prdth;
    evdsptc_context_t udpth;
    evdsptc_event_t prdev;
    evdsptc_event_t udpev;
    int i,j;
    struct mg_context *mgctx;
    bool dump = false;
    const char *server_address = "";
    unsigned short port = EVRBCAR_UDP_PORT;
    struct sockaddr servSockAddr;
    int server_sock;
    int ioctlval = 0;
  
    init_event_log();
    evrbcar_imu_init(&imuctx, I2C_DEVNAME, BNO055_ADDRESS_A);
    evrbcar_tof_init(&tofctx, I2C_DEVNAME, 0x29);

    const char *options[] = { 
        "document_root", "./htdocs",
        "request_timeout_ms", "10000",
        "websocket_timeout_ms", "15000",
        "num_threads", "1",
        0
    };
    mg_init_library(0);
    mgctx = mg_start(NULL, 0, options);
   
    if(wiringPiSetupGpio() == -1) return 1; 
    
    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
    evdsptc_setmutexattrinitializer(&mutexattr);

    prdctx.initial = true;
    prdctx.max = TICK_NS;
    prdctx.min = TICK_NS;
    prdctx.linesens = 7;
    prdctx.target_voltage = 0.0F;
    prdctx.speed = 0.0F;
    prdctx.rotspeed = 0.0F;
    prdctx.position[0] = 0.0F;
    prdctx.position[1] = 0.0F;
    prdctx.position[2] = 0.0F;
    prdctx.last_linesens = 7;
    prdctx.scan_stage = 0;

    while ((opt = getopt(argc, argv, "lds:")) != -1) {
        switch (opt) {
        case 'l':
            push_event_log("line tracing mode");
            cmd_line_trace(1.0F, -1);
            break;
        case 'd':
            dump = true; 
            break;
        case 's':
            push_event_log("laser scan mode");
            evrbcar_udp_init(&scan_udpctx, optarg, SCAN_UDP_PORT);
            break;
        default:
            break;
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
    ioctlval = 1;
    ioctl(server_sock, FIONBIO, &ioctlval);
    sockaddr_init(server_address, port, &servSockAddr);
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

    evdsptc_event_init(&prdev, periodic_routine, (void*)&prdctx, false, NULL);
    evdsptc_post(&prdth, &prdev);

    evdsptc_event_init(&udpev, udp_routine, (void*)server_sock, false, NULL);
    evdsptc_post(&udpth, &udpev);

    mg_set_request_handler(mgctx, "/evrbcar", HttpHandler, NULL);
    mg_set_websocket_handler(mgctx,
            "/websocket",
            WebSocketConnectHandler,
            WebSocketReadyHandler,
            WebsocketDataHandler,
            WebSocketCloseHandler,
            0);
    
    evdsptc_event_waitdone(&prdev);
    evdsptc_destroy(&prdth, true);

    pthread_cancel(evdsptc_getthreads(&udpth)[0]);
    evdsptc_destroy(&udpth, true);
    
    evrbcar_imu_destroy(&imuctx);
    evrbcar_tof_destroy(&tofctx);
    
    usleep(500 * 1000);
    destroy_event_log();

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
