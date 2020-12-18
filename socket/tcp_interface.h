#ifndef _TCP_INTERFACE_H_
#define _TCP_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define TCP_CLIENT_NUM_MAX 10  //最大允许TCP_CLIENT_NUM_MAX个客户端连接
#define TCP_SERVER_NUM_MAX 10  //最大允许TCP_SERVER_NUM_MAX个服务端连接

#define TCP_CLIENT_TO_SERVER -1

int32_t tcp_server_init(uint32_t port,
                        void (*tcp_receive)(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len),
                        void (*index_info)(int32_t index_s, int32_t index_c, int32_t fd, bool sta));
int32_t tcp_client_init(const uint8_t *addr, uint32_t port,
                        void (*tcp_client_recv)(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len));
uint32_t tcp_send_data(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len);

#ifdef __cplusplus
}
#endif

#endif  //_TCP_INTERFACE_H_