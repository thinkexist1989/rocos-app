#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

int get_client_connnect( )
{
    int my_server;
    struct sockaddr_in servaddr;
    if ( ( my_server = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
    {
        printf( "create socket error: %s(errno: %d)\n", strerror( errno ), errno );
        return 0;
    }
    memset( &servaddr, 0, sizeof( servaddr ) );  //清空
    servaddr.sin_family = AF_INET;
    servaddr.sin_port   = htons( 5000 );
    if ( inet_pton( AF_INET, "127.0.0.1", &servaddr.sin_addr ) <= 0 )
    {
        printf( "inet_pton error for 127.0.0.1:5000\n" );
        return 0;
    }

    if ( connect( my_server, ( struct sockaddr* )&servaddr, sizeof( servaddr ) ) < 0 )
    {
        printf( "connect error: %s(errno: %d)\n", strerror( errno ), errno );
        return 0;
    }
    return my_server;
}

int main( int argc, char** argv )
{
    int my_server = get_client_connnect( );
    std::string sendline;


    // for ( int i{ 0 }; i < 1; i++ )
    // {
    //     sleep( 3 );
    //     sendline = std::string( "150#100#100" );
    //     if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
    //     {
    //         printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
    //         return 0;
    //     }

    //     sleep( 3 );
    //     sendline = std::string( "200#200#200" );
    //     if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
    //     {
    //         printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
    //         return 0;
    //     }
    // }



    // sleep( 3 ); 
    sendline = std::string( "150#80#80" );
    if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
    {
        printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
        return 0;
    }

    sleep( 1 );
    sendline = std::string( "210#80#80" );
    if ( send( my_server, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
    {
        printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
        return 0;
    }

    close( my_server );
    return 0;
}
