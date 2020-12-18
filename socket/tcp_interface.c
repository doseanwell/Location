/****************************************************************************
 * 文件名: tcp_interface.c
 * 内容简述：tcp通信接口封装，支持服务端、客户端创建，服务端创建支持单进程同时多创建
 * 文件历史：
 *      版本号             日期              作者说明
 *      V1.0.0         2020-12-16            sean
 ****************************************************************************/
#include <errno.h>
#include <fcntl.h>
#include <linux/unistd.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "tcp_interface.h"

typedef struct {
    int32_t init;  // tcp_type初始化标志

    int32_t client_count;                                       //连接的客户端数量
    int32_t fd_client[TCP_SERVER_NUM_MAX][TCP_CLIENT_NUM_MAX];  //每个连接的客户端设备符记录
    uint8_t addr_client[TCP_CLIENT_NUM_MAX][100];               //每个连接的客户端地址记录

    int32_t            server_count;                                        //连接的服务端数量
    int32_t            epoll_fd[TCP_SERVER_NUM_MAX];                        //每个服务端设备符记录
    int32_t            sock_listen[TCP_SERVER_NUM_MAX];                     //每个服务端监听记录
    struct epoll_event event_list[TCP_SERVER_NUM_MAX][TCP_CLIENT_NUM_MAX];  //每个服务端事件链表记录
    int32_t            tid[TCP_SERVER_NUM_MAX];                             //每个服务端线程id记录

    void (*index_info)(int32_t index_s, int32_t index_c, int32_t fd,
                       bool sta);  //获取index回调函数，用于主线程控制新连接的客服端
    void (*tcp_receive)(int32_t index_s, int32_t index_c, const uint8_t *dat,
                        int32_t len);  //每个服务端接收回调函数，由主程序提供具体实现
} tcp_server_type;

typedef struct {
    int32_t init;  // tcp_type初始化标志

    int32_t client_count;                        //连接的客户端数量
    bool    connect_status[TCP_CLIENT_NUM_MAX];  //每个连接的客户端连接状态
    bool    run_flag[TCP_CLIENT_NUM_MAX];        //每个连接的客户端运行标志，用于控制线程
    int32_t fd[TCP_CLIENT_NUM_MAX];              //每个连接的客户端设备符记录
    uint8_t addr[TCP_CLIENT_NUM_MAX][100];       //每个连接的服务端地址记录
    int32_t tid[TCP_SERVER_NUM_MAX];             //每个服务端线程id记录

    void (*tcp_client_recv)(int32_t index_s, int32_t index_c, const uint8_t *dat,
                            int32_t len);  //每个服务端接收回调函数，由主程序提供具体实现
} tcp_client_type;

#define RECV_LEN 10240

tcp_server_type tcp_server_info = { 0 };
tcp_client_type tcp_client_info = { 0 };
/*******************************************************************************
 * 名称: setnonblocking
 * 功能: 设置sock为非阻塞方式
 * 形参: sock_fd：sock句柄号
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
void setnonblocking(int32_t sock_fd)
{
    int32_t opts;
    opts = fcntl(sock_fd, F_GETFL);
    if (opts < 0) {
        perror("fcntl(sock,GETFL)");
        return;
    }
    opts = opts | O_NONBLOCK;
    if (fcntl(sock_fd, F_SETFL, opts) < 0) {
        perror("fcntl(sock,SETFL,opts)");
        return;
    }
}
/*******************************************************************************
 * 名称: setblocking
 * 功能: 设置sock为阻塞方式
 * 形参: sock_fd：sock句柄号
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
void setblocking(int32_t sock_fd)
{
    int32_t opts;
    opts = fcntl(sock_fd, F_GETFL);
    if (opts < 0) {
        perror("fcntl(sock,GETFL)");
        exit(1);
    }

    opts &= ~O_NONBLOCK;
    if (fcntl(sock_fd, F_SETFL, opts) < 0) {
        perror("fcntl(sock,SETFL,opts)");
        exit(1);
    }
}

/*******************************************************************************
 * 名称: set_socketport_mode
 * 功能: 设置套接字的端口模式，即是否允许重用，以及是否立即关闭
 * 形参: sock_fd：sock句柄号
 *      opt：0 禁止端口重用 1 允许端口重用
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
void set_socketport_mode(int32_t sock_fd, int32_t opt)
{
    setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // broke pipe 回避处理
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGPIPE, &sa, NULL) < 0)
        printf("cannot ignore SIGPIPE");
}
/*******************************************************************************
 * 名称: epoll_event_del
 * 功能: epoll事件删除sock_fd的检测
 * 形参: index_s：服务端在tcp_server_info保存的相关数据偏移index
 *      sockid：fd句柄号
 *      ee：epoll_event结构体指针
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
void epoll_event_del(int32_t index_s, int32_t sock_fd, struct epoll_event *ee)
{
    int32_t i;
    for (i = 0; i < TCP_CLIENT_NUM_MAX; i++)  //用于轮寻当前断开连接的fd，并关闭它，且设置为-1作为断开的标志
    {
        if (sock_fd == tcp_server_info.fd_client[index_s][i]) {
            tcp_server_info.fd_client[index_s][i] = -1;
            if (tcp_server_info.index_info) {
                tcp_server_info.index_info(index_s, i, sock_fd,
                                           false);  //调用回调函数，主函数处理数据，这里是处理断开连接的客户端
            }
            epoll_ctl(tcp_server_info.epoll_fd[index_s], EPOLL_CTL_DEL, sock_fd, ee);  //删除对应的fd句柄
            break;
        }
    }
}
/*******************************************************************************
 * 名称: accept_conn
 * 功能: 客户端连接状态的设置
 * 形参: index_s：服务端在tcp_server_info保存的相关数据偏移index
 *      srv_fd：监听的SOCKET号
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
void accept_connect(int32_t index_s, int32_t srv_fd)
{
    int32_t fd;
    int32_t i, index_c = 0;

    struct sockaddr_in sin;
    socklen_t          len = sizeof(struct sockaddr_in);

    bzero(&sin, len);

    fd = accept(srv_fd, (struct sockaddr *)&sin, &len);

    for (i = 0; i < TCP_CLIENT_NUM_MAX; i++) {  // 检测是否有被释放的MSG，若有就使用释放的空间
        if (tcp_server_info.fd_client[index_s][i] == -1) {
            index_c = i;
            printf("msg_server_init i is:%d \n", i);
            break;
        }
    }

    if (fd < 0) {
        printf("bad accept\n");
        return;
    }
    else {
        printf("Accept connection: %d\n", fd);
    }

    if (i >= TCP_CLIENT_NUM_MAX) {  // 判断是否有释放的空间记录
        send(fd, "ERROR NO SPACE\r\n", strlen("ERROR NO SPACE\r\n"), 0);
        close(fd);
        printf("ERROR NO SPACE\r\n");
        // tcp_server_info.fd[tcp_server_info.index] = fd;
        // tcp_server_info.index++;
        // return tcp_server_info.index - 1;
    }
    else {
        tcp_server_info.fd_client[index_s][index_c] = fd;
    }

    if (tcp_server_info.index_info) {
        tcp_server_info.index_info(index_s, index_c, fd, true);  //调用回调函数，主函数处理数据
    }

    setnonblocking(fd);  //设置fd为非阻塞方式

    //将新建立的连接添加到EPOLL的监听中
    struct epoll_event new_event;
    new_event.data.fd = fd;
    new_event.events  = EPOLLIN | EPOLLET;

    epoll_ctl(tcp_server_info.epoll_fd[index_s], EPOLL_CTL_ADD, fd, &new_event);

    send(fd, "ICY 200 OK\r\n", strlen("ICY 200 OK\r\n"), 0);
}
/*******************************************************************************
 * 名称: recv_data
 * 功能: TCP服务端接收数据处理
 * 形参: index_s：服务端在tcp_server_info保存的相关数据偏移index
        events：epoll_event结构体指针
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
static void tcp_recv_data(int32_t index_s, struct epoll_event *events)
{
    int32_t ret;
    int32_t recv_Len = 0;
    int32_t fd       = events->data.fd;
    int32_t i;
    uint8_t s_buf[RECV_LEN];  //接收缓存

    if (fd < 0) {
        return;
    }

    while (1) {
        // recv数据
        ret = recv(fd, (void *)s_buf, RECV_LEN, 0);
        if (ret == 0) {
            // 这里表示对端的socket已正常关闭.发送过FIN了
            printf("unconnect %d\r\n", fd);
            epoll_event_del(index_s, fd, events);
            break;
        }
        else if (ret < 0) {
            if (errno == EAGAIN) {
                // 由于是非阻塞的模式,所以当errno为EAGAIN时,表示当前缓冲区已无数据可读
                // 在这里就当作是该次事件已处理处.
                // printf("Data rev Over!\r\n");
                break;
            }
            else if (errno == ECONNRESET) {
                // 对方发送了RST
                epoll_event_del(index_s, fd, events);
                break;
            }
            else if (errno == EINTR) {
                // 被信号中断
                continue;
            }
            else {
                //其他不可弥补的错误
                epoll_event_del(index_s, fd, events);
                break;
            }
        }

        for (i = 0; i < TCP_CLIENT_NUM_MAX; i++)  //用于轮寻当前断开连接的fd，并关闭它，且设置为-1作为断开的标志
        {
            if (fd == tcp_server_info.fd_client[index_s][i]) {
                if (tcp_server_info.tcp_receive) {
                    tcp_server_info.tcp_receive(index_s, i, s_buf, ret);  //调用回调函数，主函数处理数据
                }

                break;
            }
        }

        //数据接受正常
        // printf("ret is:%d\r\n", ret);
        /*if(ret <
        MAX_LINE)//如果使用阻塞方式需要这个判断，否则接收完后就会停在recv一直等待
        {
            recv_Len = 0;
            break;
        }*/
        // if (ret >= MAX_LINE) {
        //     one_pack_size += MAX_LINE;
        //     continue;
        // }
        // else  //数据接收完毕
        // {
        // }
    }
}
/*******************************************************************************
 * 名称: tcp_send_data
 * 功能: TCP服务端发送函数
 * 形参: index_s：服务端在tcp_server_info保存的相关数据偏移index
 *      index_c：服务端在tcp_server_info保存的相关数据偏移index
        dat：数据指针
        len：数据长度
 * 返回: 无
 * 说明: index_s为-1时表示是客户端向服务器发送数据，反之
 ******************************************************************************/
uint32_t tcp_send_data(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len)
{
    if (index_s >= 0) {  //判断是否为服务器向客户端发送数据
        return (uint32_t)send(tcp_server_info.fd_client[index_s][index_c], dat, len, 0);
    }
    else {
        return (uint32_t)send(tcp_client_info.fd[index_c], dat, len, 0);
    }
}
/*******************************************************************************
 * 名称: tcp_server_thread
 * 功能: 服务端运行线程
 * 形参: arg：服务端在tcp_server_info保存的相关数据偏移index
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
static void *tcp_server_thread(void *arg)
{
    int32_t ret, n;
    int32_t index_s = *(int32_t *)arg;
    // printf("*index is:%d\n", index);
    // printf("fd is:%d\n", tcp_server_info.fd[index]);
    *(int32_t *)arg = -1;

    while (1) {
        // timeout = 3000;
        // 3. epoll_wait
        ret = epoll_wait(tcp_server_info.epoll_fd[index_s], tcp_server_info.event_list[index_s], TCP_CLIENT_NUM_MAX, 5);

        if (ret < 0) {
            printf("epoll error\n");
            break;
        }
        else if (ret == 0) {
            // printf("timeout ...\n");
        }

        //直接获取了事件数量,给出了活动的流,这里是和poll区别的关键
        for (n = 0; n < ret; n++) {
            //错误退出
            if ((tcp_server_info.event_list[index_s][n].events & EPOLLERR)
                || (tcp_server_info.event_list[index_s][n].events & EPOLLHUP)) {
                if (tcp_server_info.event_list[index_s][n].events & EPOLLERR)
                    printf("epoll EPOLLERR\n");
                else
                    printf("epoll EPOLLHUP\n");
                // printf ("eventList[n].data.fd0 is:%d\n", eventList[n].data.fd);
                epoll_event_del(index_s, tcp_server_info.event_list[index_s][n].data.fd,
                                &tcp_server_info.event_list[index_s][n]);
                // printf ("eventList[n].data.fd is:%d\n", eventList[n].data.fd);
                // return -1;
            }

            if (tcp_server_info.event_list[index_s][n].data.fd == tcp_server_info.sock_listen[index_s]) {
                accept_connect(index_s, tcp_server_info.sock_listen[index_s]);
            }
            else if (tcp_server_info.event_list[index_s][n].events & EPOLLIN) {
                tcp_recv_data(index_s, &tcp_server_info.event_list[index_s][n]);
            }
            else if (tcp_server_info.event_list[index_s][n].events & EPOLLOUT) {
                // send_data(index_s, &tcp_server_info.event_list[index_s][n]);
            }
        }
    }
    // pthread_join(tcp_server_info.tid[index], NULL);  //等待线程结束
    pthread_exit(NULL);  //退出线程
}
/*******************************************************************************
 * 名称: create_tcp_server_thread
 * 功能: 服务端线程创建
 * 形参: index_s：服务端在tcp_server_info保存的相关数据偏移index
 * 返回: -1 服务端初始化失败 0 服务线程创建成功
 * 说明: 无
 ******************************************************************************/
static int32_t create_tcp_server_thread(int32_t index_s)
{
    pthread_t tid;
    int32_t   index_tmp = index_s;
    if (pthread_create(&tid, NULL, tcp_server_thread, &index_tmp) != 0) {
        printf("Create tcp server thread error!\r\n");
        return -1;
    }
    else {
        printf("Create tcp server thread Success!\r\n");
    }
    while (index_tmp != -1) {
        usleep(10000);
    }  //等待线程启动完成
    tcp_server_info.tid[index_s] = tid;
    return 0;
}
/*******************************************************************************
 * 名称: tcp_server_init
 * 功能: msg服务端初始化
 * 形参: port：服务绑定端口
 *      tcp_receive：tcp消息接收回调函数
 *      index_info：tcp客户端连接消息接收回调函数
 *                  index_s：服务端index偏移
 *                  index_c：客户端index偏移，受index_s控制
 * 返回: -1 服务端初始化失败 其他 服务端初始化成功后tcp_server_info保存的相关服务端数据index偏移
 * 说明: tcp_receive，必须设置，否则无法接收数据;index_info，可以不设置，配置NULL即可
 *      tcp_receive和index_info配置只在最后一次tcp_server_init调用时配置，前面设置NULL
 ******************************************************************************/
int32_t tcp_server_init(uint32_t port,
                        void (*tcp_receive)(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len),
                        void (*index_info)(int32_t index_s, int32_t index_c, int32_t fd, bool sta))
{
    int32_t            fd;
    int32_t            i = 0, index_s = 0;
    struct sockaddr_in server_addr;
    struct epoll_event event;
    //初始化fd
    if (tcp_server_info.init <= 0) {
        memset(tcp_server_info.fd_client, -1, sizeof(tcp_server_info.fd_client));
        memset(tcp_server_info.epoll_fd, -1, sizeof(tcp_server_info.epoll_fd));
        tcp_server_info.init = 1;
    }

    if (tcp_server_info.server_count >= TCP_SERVER_NUM_MAX) {  // 判断服务器启动总量是否超过最大限制
        fprintf(stderr, "tcp reaches the maximum value.");
        return -1;
    }

    for (i = 0; i < TCP_SERVER_NUM_MAX; i++) {  // 检测是否有被释放的tcp，若有就使用释放的空间
        if (tcp_server_info.epoll_fd[i] == -1) {
            index_s = i;
            printf("tcp_server_init i is:%d \n", i);
            break;
        }
    }

    if (i >= TCP_SERVER_NUM_MAX) {  // 判断是否有释放的空间记录
        fprintf(stderr, "tcp reaches the maximum value.");
        return -1;
    }

    // socket
    if ((tcp_server_info.sock_listen[index_s] = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("socket error\n");
        exit(1);
    }

    set_socketport_mode(tcp_server_info.sock_listen[index_s], 1);

    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // bind 绑定sockListen和server_addr
    if (bind(tcp_server_info.sock_listen[index_s], (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("bind error\n");
        exit(1);
    }

    // listen 监听
    if (listen(tcp_server_info.sock_listen[index_s], 5) < 0) {
        printf("listen error\n");
        exit(1);
    }

    // epoll 初始化
    tcp_server_info.epoll_fd[index_s] = epoll_create(TCP_CLIENT_NUM_MAX);
    event.events                      = EPOLLIN | EPOLLET;
    event.data.fd                     = tcp_server_info.sock_listen[index_s];

    tcp_server_info.server_count++;  //服务端总数量加1
    // epoll_ctrl
    if (epoll_ctl(tcp_server_info.epoll_fd[index_s], EPOLL_CTL_ADD, tcp_server_info.sock_listen[index_s], &event) < 0) {
        printf("epoll add fail : fd = %d\n", tcp_server_info.sock_listen[index_s]);
        exit(1);
    }

    tcp_server_info.tcp_receive = (void *)tcp_receive;
    tcp_server_info.index_info  = (void *)index_info;
    // printf("New process:  PID: %d,TID: %u.\n", getpid(), pthread_self());  // why pthread_self
    // printf("New process:  PID: %d,TID: %u.\n", getpid(), tid_s);           // why pthread_self
    create_tcp_server_thread(index_s);

    return index_s;
}
/*******************************************************************************
 * 名称: tcp_server_deinit
 * 功能: tcp服务端关闭注销
 * 形参: index_s：服务端在tcp_server_info保存的相关数据偏移index
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
void tcp_server_deinit(int index_s)
{
    tcp_server_info.epoll_fd[index_s] = -1;
    tcp_server_info.server_count--;
}
/*******************************************************************************
 * 名称: tcp_receive_register
 * 功能: msg消息接收注册
 * 形参: index：接收的参数，值为服务端在tcp_server_info保存的相关数据偏移index
 *      tcp_receive：tcp消息接收回调函数
 * 返回: 0 成功
 * 说明: 本函数只在为服务端时使用，注意有回调函数tcp_receive，必须设置，否则无法接收数据
 ******************************************************************************/
int32_t tcp_receive_register(void (*tcp_receive)(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len))
{
    tcp_server_info.tcp_receive = (void *)tcp_receive;
    return 0;
}
/*******************************************************************************
 * 名称: tcp_index_register
 * 功能: msg消息接收注册
 * 形参: index_info：tcp客户端连接消息接收回调函数
 *                  index_s：服务端index偏移
 *                  index_c：客户端index偏移，受index_s控制
 * 返回: 0 成功
 * 说明: 本函数只在为服务端时使用，注意有回调函数index_info，可以不设置
 ******************************************************************************/
int32_t tcp_index_register(void (*index_info)(int32_t index_s, int32_t index_c))
{
    tcp_server_info.index_info = (void *)index_info;
    return 0;
}
/*******************************************************************************
 * 名称: tcp_client_thread
 * 功能: 客户端运行线程
 * 形参: arg：服务端在tcp_client_info保存的相关数据偏移index
 * 返回: 无
 * 说明: 无
 ******************************************************************************/
static void *tcp_client_thread(void *arg)
{
    int32_t ret;
    int32_t index_c         = *(int32_t *)arg;
    uint8_t c_buf[RECV_LEN] = { 0 };
    // printf("*index is:%d\n", index);
    // printf("fd is:%d\n", tcp_server_info.fd[index]);
    *(int32_t *)arg = -1;

    while (tcp_client_info.run_flag[index_c]) {
        ret = recv(tcp_client_info.fd[index_c], c_buf, RECV_LEN, 0);
        if (ret < 0) {
            if (errno == EAGAIN) {
                // 由于是非阻塞的模式,所以当errno为EAGAIN时,表示当前缓冲区已无数据可读
                // 在这里就当作是该次事件已处理处,设置接收完数据的参数
                // DEBUG_INFO("Data recv Over!\r\n");
                // DEBUG_INFO("\033[1m\033[41;33m head is:%d   \033[0m \n", tcp_client->client_recv->head);

                // if (time_count++ > 20)  //当运行２０次后还是没有读到数据，那么设置data_falg标志为ｆａｌｓｅ
                // {
                //     time_count = 0;
                //     data_flag  = false;
                // }
                usleep(100000);  //没有数据延时100ms
                continue;
            }
            else if (errno == ECONNRESET) {
                // 对方发送了RST
                //              close_and_disable(fd, events, tcp_server);
                printf("PID %d Client reset cmd ! connect close\r\n", getpid());
                tcp_client_info.connect_status[index_c] = false;
                break;
            }
            else if (errno == EINTR) {
                printf("Signal is interrupt!\r\n");
                // 被信号中断
                continue;
            }
            else {
                //其他不可弥补的错误
                printf("PID %d Client recv err ! connect close\r\n", getpid());
                tcp_client_info.connect_status[index_c] = false;
                break;
            }
        }
        else if (ret == 0)  //到这里说明读取错误
        {
            printf("PID %d Client recv err ! connect close\r\n", getpid());
            tcp_client_info.connect_status[index_c] = false;
            break;
        }
        else {
            if (tcp_client_info.tcp_client_recv) {
                tcp_client_info.tcp_client_recv(TCP_CLIENT_TO_SERVER, index_c, c_buf, ret);
            }
            usleep(1000);  //有数据延时1ms
        }
    }

    // pthread_join(tcp_server_info.tid[index], NULL);  //等待线程结束
    pthread_exit(NULL);  //退出线程
}
/*******************************************************************************
 * 名称: tcp_client_init
 * 功能: TCP客户端初始化，创建客户端线程
 * 形参: tcp_client：Tcp_Client_Type结构体指针
 * 返回: 0：初始化成功  -1：初始化失败
 * 说明: tcp_client_recv配置只在最后一次tcp_client_init调用时配置，前面设置NULL
 ******************************************************************************/
int32_t tcp_client_init(const uint8_t *addr, uint32_t port,
                        void (*tcp_client_recv)(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len))
{
    pthread_t          tid_c;
    int32_t            fd;
    int32_t            i = 0, index_c = 0;
    int32_t            index_tmp;
    struct sockaddr_in server_addr;

    fd_set         rset, wset;
    struct timeval tval;

    //初始化fd
    if (tcp_client_info.init <= 0) {
        memset(tcp_client_info.fd, -1, sizeof(tcp_client_info.fd));
        tcp_client_info.init = 1;
    }

    if (tcp_client_info.client_count >= TCP_CLIENT_NUM_MAX) {  // 判断服务器启动总量是否超过最大限制
        fprintf(stderr, "tcp reaches the maximum value.");
        return -1;
    }

    for (i = 0; i < TCP_CLIENT_NUM_MAX; i++) {  // 检测是否有被释放的tcp，若有就使用释放的空间
        if (tcp_client_info.fd[i] == -1) {
            index_c = i;
            printf("tcp_client_init index_c is:%d \n", index_c);
            break;
        }
    }

    if (i >= TCP_CLIENT_NUM_MAX) {  // 判断是否有释放的空间记录
        fprintf(stderr, "tcp reaches the maximum value.");
        return -1;
    }

    // socket
    if ((tcp_client_info.fd[index_c] = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("socket error\n");
        exit(1);
    }

    //  setblocking(tcp_client->sock_cli_fd);//设置为阻塞模式
    setnonblocking(tcp_client_info.fd[index_c]);

    // 忽略pipe信号
    set_socketport_mode(tcp_client_info.fd[index_c], 1);

    /// s
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port   = htons(port);  ///服务器端口
    // server_addr.sin_addr.s_addr = inet_addr(addr);  ///服务器ip
    if (inet_pton(AF_INET, addr, &server_addr.sin_addr) <= 0) {
        printf("inet_pton error for %s\n", addr);
        exit(1);
    }

    ///连接服务器，成功返回0，错误返回-1
    while (connect(tcp_client_info.fd[index_c], (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        if (errno != EINPROGRESS) {
            perror("Connect error");
            exit(1);
        }
        usleep(100000);
    }
    tcp_client_info.client_count++;
    tcp_client_info.connect_status[index_c] = true;  //设置连接状态
    tcp_client_info.run_flag[index_c]       = true;  //设置运行标志
    tcp_client_info.tcp_client_recv         = (void *)tcp_client_recv;

    index_tmp = index_c;
    if (pthread_create(&tid_c, NULL, tcp_client_thread, (void *)&index_tmp) != 0) {
        printf("Create tcp_client thread error!\r\n");
        return -1;
    }
    else {
        printf("Create tcp_client thread Success!\r\n");
    }
    while (index_tmp != -1) {
        usleep(10000);
    }  //等待线程启动完成
    tcp_client_info.tid[index_c] = tid_c;
    return index_c;
}
