#ifndef __EVRBCAR_H__
#define __EVRBCAR_H__

#include <stdbool.h>
#include <string.h>
#include <math.h>

#define CLIENT_UDP_PORT (65000)
#define EVRBCAR_UDP_PORT (65001)
#define SCAN_UDP_PORT (65002)
#define DEG2RAD(deg) (deg /180.0F * M_PI)
#define RAD2DEG(rad) (rad /M_PI * 180.0F)
#define SCAN_BUFSIZE (256)

typedef enum evrbcar_cmd_mode{
    EVRBCAR_CMD_STOP = 0,
    EVRBCAR_CMD_MOVE_TO,
    EVRBCAR_CMD_MOVE_AT,
    EVRBCAR_CMD_TURN,
    EVRBCAR_CMD_LINE_TRACE,
    EVRBCAR_CMD_EXT_LINE_TRACE,
    EVRBCAR_CMD_MOVE_TURN,
    EVRBCAR_CMD_CONNECT = 129,
    EVRBCAR_CMD_SCAN,
} t_evrbcar_cmd_mode;

typedef struct evrbcar_cmd_request {
    t_evrbcar_cmd_mode mode;
    int ivalue[2];
    float fvalue[2];
} t_evrbcar_cmd_request;

typedef struct evrbcar_cmd_response {
    t_evrbcar_cmd_mode mode;
    int ivalue[2];
    float fvalue[2];
} t_evrbcar_cmd_response;

typedef struct scan_data {
    float odom[3];
    bool scan;
    float start_angle;
    float end_angle;
    short num;
    short range[SCAN_BUFSIZE];
} t_scan_data;

#ifndef __JOYSTICK__

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

typedef struct evrbcar_udp_context {
    int sock;
    struct sockaddr sockaddr;
} t_evrbcar_udp_context;

int evrbcar_udp_init(t_evrbcar_udp_context *udpctx, const char *address, unsigned short port);
int evrbcar_udp_cmd_line_trace(t_evrbcar_udp_context *udpctx, float level);
int evrbcar_udp_cmd_ext_line_trace(t_evrbcar_udp_context *udpctx, float level, int linesens);
int evrbcar_udp_send_scan_data(t_evrbcar_udp_context *udpctx, t_scan_data* scan_data, short num);

inline static void sockaddr_init (const char *address, unsigned short port, struct sockaddr *sockaddr) {

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

#endif
#endif
