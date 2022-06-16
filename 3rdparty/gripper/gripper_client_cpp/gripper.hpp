#ifndef GRIPPER_HPP
#define GRIPPER_HPP

#include "plog/Log.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>


namespace JC_helper
{
    class gripper
    {
        int my_server;
public:
        ~gripper( )
        {
            close( my_server );
        }

        int init( )
        {
            struct sockaddr_in servaddr;
            if ( ( my_server = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
            {
                printf( "create socket error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }
            memset( &servaddr, 0, sizeof( servaddr ) );  //清空
            servaddr.sin_family = AF_INET;
            servaddr.sin_port   = htons( 5000 );
            if ( inet_pton( AF_INET, "127.0.0.1", &servaddr.sin_addr ) <= 0 )
            {
                printf( "inet_pton error for 127.0.0.1:5000\n" );
                return -1;
            }

            if ( connect( my_server, ( struct sockaddr* )&servaddr, sizeof( servaddr ) ) < 0 )
            {
                printf( "connect error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }
            return 0;
        }

        // inline int get_client_handle( )
        // {
        //     int my_server;
        //     struct sockaddr_in servaddr;
        //     if ( ( my_server = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
        //     {
        //         printf( "create socket error: %s(errno: %d)\n", strerror( errno ), errno );
        //         return -1;
        //     }
        //     memset( &servaddr, 0, sizeof( servaddr ) );  //清空
        //     servaddr.sin_family = AF_INET;
        //     servaddr.sin_port   = htons( 5000 );
        //     if ( inet_pton( AF_INET, "127.0.0.1", &servaddr.sin_addr ) <= 0 )
        //     {
        //         printf( "inet_pton error for 127.0.0.1:5000\n" );
        //         return -1;
        //     }

        //     if ( connect( my_server, ( struct sockaddr* )&servaddr, sizeof( servaddr ) ) < 0 )
        //     {
        //         printf( "connect error: %s(errno: %d)\n", strerror( errno ), errno );
        //         return -1;
        //     }
        //     return my_server;
        // }

        int send_command( const std::string& sendline )
        {
            if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
            {
                printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }
            sleep(1);
            return 0;
        }

        // int main( int argc, char** argv )
        // {
        //     int my_server = get_client_connnect( );
        //     std::string sendline;

        //     for ( int i{ 0 }; i < 1; i++ )
        //     {
        //         sleep( 3 );
        //         sendline = std::string( "150#100#100" );
        //         if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
        //         {
        //             printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
        //             return 0;
        //         }

        //         sleep( 3 );
        //         sendline = std::string( "200#200#200" );
        //         if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
        //         {
        //             printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
        //             return 0;
        //         }
        //     }

        //     sleep( 3 );
        //     sendline = std::string( "-1#-1#-1" );  //关闭远程TCP
        //     if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
        //     {
        //         printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
        //         return 0;
        //     }

        //     close( my_server );
        //     return 0;
        // }
    };  // namespace gripper
}  // namespace JC_helper

#endif