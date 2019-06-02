#ifndef __EVRBCAR_H__
#define __EVRBCAR_H__

#define CLIENT_UDP_PORT (65000)
#define EVRBCAR_UDP_PORT (65001)

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

#ifndef __JOYSTICK__

#include <sys/socket.h>

typedef struct evrbcar_udp_context {
    int sock;
    struct sockaddr sockaddr;
} t_evrbcar_udp_context;

int evrbcar_udp_init(t_evrbcar_udp_context *udpctx, const char *address, unsigned short port);
int evrbcar_udp_cmd_line_trace(t_evrbcar_udp_context *udpctx, float level);
int evrbcar_udp_cmd_ext_line_trace(t_evrbcar_udp_context *udpctx, float level, int linesens);

#endif
#endif
