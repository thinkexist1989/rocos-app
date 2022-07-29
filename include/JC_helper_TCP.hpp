#pragma once

/**
 * @file TCP_server.cpp
 * @author JC
 * @brief 这是一个TCP服务器例子，基于线程编程，启动后会等待客户端连接，每收到一帧数据就返回数据“i am server at xxx”
 * @brief 支持重连
 * @version 0.1
 * @date 2021-11-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <atomic>
#include <boost/thread.hpp>
#include <errno.h>
#include <iostream>
#include <netinet/in.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Log.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace JC_helper
{

    class TCP_server
    {
    private:
        static int listenfd;    //监听的句柄、
        static int connfd;      //客户端的句柄
        static bool flag_init;  //初始化成功标志

        struct sockaddr_in servaddr;  // IP地址

    public:
        std::atomic< bool > flag_receive{ false };
        std::vector< char > send_buf;  //发送buffer
        std::vector< char > receive_buff;

        TCP_server( int max_num = 2048 )
        {
            receive_buff.resize( max_num );
            send_buf.resize( max_num );
        }
        ~TCP_server( )
        {
            close( connfd );
            close( listenfd );
        }

        int init( int port = 12345 )
        {
            flag_init = false;

            memset( &servaddr, 0, sizeof( servaddr ) );
            servaddr.sin_family      = AF_INET;
            servaddr.sin_addr.s_addr = htonl( INADDR_ANY );  //监听全部ip
            servaddr.sin_port        = htons( port );        //监听端口号

            if ( ( listenfd = socket( AF_INET, SOCK_STREAM, 0 ) ) == -1 )
            {
                PLOG_ERROR.printf( "create socket error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }

            int on = 1;
            if ( setsockopt( listenfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof( on ) ) < 0 )
            {
                PLOG_ERROR.printf( "setsockopt error: %s(errno: %d)\n", strerror( errno ), errno );
            return -1;
            }

            if ( bind( listenfd, ( struct sockaddr* )&servaddr, sizeof( servaddr ) ) == -1 )
            {
                PLOG_ERROR.printf( "bind socket error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }

            if ( listen( listenfd, 10 ) == -1 )
            {
                PLOG_ERROR.printf( "listen socket error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }

            if ( signal( SIGINT, my_close ) == SIG_ERR )
            {
                PLOG_ERROR << "TCP 线程绑定ctrl+c信号 失败";
                return -1;
            }

            PLOG_INFO << "TCP init success";
            flag_init = true;
            return 0;
        }

        int RunServer( )
        {
            while ( flag_init )  //!不断连接新client
            {
                PLOG_INFO.printf( "======waiting for client's request======" );

                if ( ( connfd = accept( listenfd, ( struct sockaddr* )NULL, NULL ) ) == -1 )
                {
                    PLOG_ERROR.printf( "accept socket error: %s(errno: %d)", strerror( errno ), errno );
                    return -1;
                }

                //!接入一个客户端
                PLOG_INFO << "client 连接成功,id = " << connfd;

                while ( flag_init )  //不断接收client的消息，直至client主动断开连接
                {
                    //** 不断接受数据 **//

                    int recv_num = recv( connfd, &receive_buff[ 0 ], receive_buff.size( ), 0 );  //接受多少个字符,client断开连接时，立刻返回recv_num=0

                    if ( recv_num > 0 )
                    {
                        receive_buff[ recv_num ] = '\0';
                        flag_receive             = true;

                        // sprintf( &send_buf[ 0 ], "i am server at %d times", count++ );               //反馈消息给客户端
                        // int send_len = send( connfd, &send_buf[ 0 ], strlen( &send_buf[ 0 ] ), 0 );  //反馈消息给客户端
                    }
                    else
                    {
                        PLOG_ERROR << "client 断开连接";
                        close( connfd );
                        break;
                    }

                    //**-------------------------------**//
                }
            }

            close( connfd );
            return 0;
        }

        static void my_close( int sig )
        {
            if ( sig == SIGINT )
            {
                // ctrl+c退出时执行的代码
                PLOG_INFO << "按下ctrl+c,清除TCP资源,程序退出";
                close( connfd );
                close( listenfd );
                flag_init = false;
                exit( 0 );
            }
        }
    };

    inline int TCP_server::listenfd   = 0;
    inline int TCP_server::connfd     = 0;
    inline bool TCP_server::flag_init = false;

}  // namespace JC_helper

#if 0

int main( int argc, char** argv )
{
    plog::ColorConsoleAppender< plog::TxtFormatter > consoleAppender;
    plog::init( plog::debug, &consoleAppender );  // Initialize the logger.

    TCP_server my_server;
    my_server.init( );
    boost::thread( &TCP_server::RunServer, &my_server ).detach( );  //开启服务器

    while ( 1 )
    {
        sleep( 1 );
        if ( my_server.flag_receive )
        {
            PLOG_DEBUG << "Received=" << &my_server.receive_buff[ 0 ];
            my_server.flag_receive = false;
        }
    }
}

#endif