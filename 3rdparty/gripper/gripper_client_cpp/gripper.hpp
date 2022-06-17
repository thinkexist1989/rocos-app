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
                PLOG_ERROR.printf( "create socket error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }
            memset( &servaddr, 0, sizeof( servaddr ) );  //清空
            servaddr.sin_family = AF_INET;
            servaddr.sin_port   = htons( 5000 );
            if ( inet_pton( AF_INET, "127.0.0.1", &servaddr.sin_addr ) <= 0 )
            {
                PLOG_ERROR.printf( "inet_pton error for 127.0.0.1:5000\n" );
                return -1;
            }

            if ( connect( my_server, ( struct sockaddr* )&servaddr, sizeof( servaddr ) ) < 0 )
            {
                PLOG_ERROR.printf( "connect error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }
            return 0;
        }

    

        int send_command( const std::string& sendline )
        {
            if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
            {
                PLOG_ERROR.printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
                return -1;
            }
            sleep(1);
            return 0;
        }

 
    };  // namespace gripper
}  // namespace JC_helper

#endif