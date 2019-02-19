#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "evrbcar.h"

static void alarm_handler (int ignored) {
    (void)ignored;
}

int evrbcar_udp_init(t_evrbcar_udp_context *udpctx, const char *address, unsigned short port) {
    struct sockaddr_in sockaddr_in;
    struct sigaction action;

    sockaddr_in.sin_family = AF_INET;

    if (inet_aton(address, &sockaddr_in.sin_addr) == 0) {
        if (strcmp(address, "") == 0 ) {
            sockaddr_in.sin_addr.s_addr = htonl(INADDR_ANY);
        } else return -1;
    }

    if (port == 0) return -1;
    sockaddr_in.sin_port = htons(port);

    udpctx->sockaddr = *((struct sockaddr *)&sockaddr_in);

    udpctx->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udpctx->sock < 0) return -1; 

    action.sa_handler = alarm_handler;
    if (sigfillset(&(action.sa_mask)) < 0) return -1;
    action.sa_flags = 0;

    if (sigaction(SIGALRM, &action, NULL) < 0) return -1;

    return 0;
}

int evrbcar_udp_cmd_line_trace(t_evrbcar_udp_context *udpctx, float level){
    int sendSize;
    t_evrbcar_cmd_request req;
    int size = sizeof(t_evrbcar_cmd_request);

    req.mode = EVRBCAR_CMD_LINE_TRACE;
    req.value[0] = level;
    sendSize = sendto(udpctx->sock, &req, size, 0, &udpctx->sockaddr, sizeof(udpctx->sockaddr));
    if (sendSize != size) return -1;
    return sendSize;
}


