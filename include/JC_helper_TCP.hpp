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

#include <boost/thread.hpp>
#include <errno.h>
#include <iostream>
#include <netinet/in.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Log.h>
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
        int listenfd, connfd;         //监听的句柄、客户端的句柄
        struct sockaddr_in servaddr;  // IP地址
        int count = 0;                //统计发送的次数

    public:
        bool flag_receive{ false };
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

        int init( int port = 6000 )
        {
            memset( &servaddr, 0, sizeof( servaddr ) );
            servaddr.sin_family      = AF_INET;
            servaddr.sin_addr.s_addr = htonl( INADDR_ANY );  //服务器ip就是自己
            servaddr.sin_port        = htons( port );        //监听端口号

            if ( ( listenfd = socket( AF_INET, SOCK_STREAM, 0 ) ) == -1 )
            {
                PLOG_ERROR.printf( "create socket error: %s(errno: %d)\n", strerror( errno ), errno );
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
            PLOG_INFO << "TCP init success";
            return 0;
        }

        int RunServer( )
        {
            while ( 1 )  //!不断连接新client
            {
                PLOG_INFO.printf( "======waiting for client's request======\n" );

                if ( ( connfd = accept( listenfd, ( struct sockaddr* )NULL, NULL ) ) == -1 )
                {
                    PLOG_ERROR.printf( "accept socket error: %s(errno: %d)", strerror( errno ), errno );
                    return -1;
                }

                //!接入一个客户端

                while ( 1 )  //不断接收client的消息，直至client主动断开连接
                {
                    //** 不断接受数据 **//

                    int recv_num = recv( connfd, &receive_buff[ 0 ], receive_buff.size( ), 0 );  //接受多少个字符,client断开连接时，立刻返回recv_num=0

                    if ( recv_num > 0 )
                    {
                        receive_buff[ recv_num ] = '\0';
                        flag_receive             = true;

                        sprintf( &send_buf[ 0 ], "i am server at %d times", count++ );               //反馈消息给客户端
                        int send_len = send( connfd, &send_buf[ 0 ], strlen( &send_buf[ 0 ] ), 0 );  //反馈消息给客户端
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
            close( listenfd );
            return 0;
        }
    };
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