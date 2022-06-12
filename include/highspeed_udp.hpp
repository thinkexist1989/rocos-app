#ifndef HIGHSPEED_UDP
#    define HIGHSPEED_UDP

#    ifdef _WIN32
#        define WIN32_LEAN_AND_MEAN
#        include <windows.h>
#        include <winsock2.h>
#        pragma comment( lib, "Ws2_32.lib" )
typedef SOCKET SOCKET_HANDLE;
#    else
#        include <arpa/inet.h>
#        include <netdb.h>
#        include <sys/socket.h>
#        include <unistd.h>
typedef int SOCKET_HANDLE;
#    endif

#    include <stdio.h>
#    include <stdlib.h>
#    include <string.h>
#    include <sys/types.h>

#    define FT_PORT  49152       /* FT_PORT the Ethernet DAQ always uses */
#    define FT_SAMPLE_COUNT 10  /* 10 incoming samples */
#    define FT_SPEED 10         /* 1000 / FT_SPEED = FT_SPEED in Hz */
#    define FT_FILTER 4         /* 0 = No FT_FILTER; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz */
#    define FT_BIASING_ON 0xFF  /* Biasing on */
#    define FT_BIASING_OFF 0x00 /* Biasing off */

#    define COMMAND_START 0x0002  /* Command for start streaming */
#    define COMMAND_STOP 0x0000   /* Command for stop streaming */
#    define COMMAND_BIAS 0x0042   /* Command for toggle biasing */
#    define COMMAND_FILTER 0x0081 /* Command for setting FT_FILTER */
#    define COMMAND_SPEED 0x0082  /* Command for setting FT_SPEED */

#    define UNIT 1  // 0 - Dimensionless  | 1 - Newton/Newton-meter

#    if UNIT == 1
#        define FORCE_DIV 10000.0    // Default divide value
#        define TORQUE_DIV 100000.0  // Default divide value
#    else
#        define FORCE_DIV 1.0
#        define TORQUE_DIV 1.0
#    endif

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct ResponseStruct
{
    unsigned int sequenceNumber;
    unsigned int sampleCounter;
    unsigned int status;
    int32 fx;
    int32 fy;
    int32 fz;
    int32 tx;
    int32 ty;
    int32 tz;
} Response;

/* Sleep ms milliseconds */
static void MySleep( unsigned long ms )
{
#    ifdef _WIN32
    Sleep( ms );
#    else
    usleep( ms * 1000 );
#    endif
}

static int Connect( SOCKET_HANDLE* handle, const char* ipAddress, uint16 PORT )
{
    struct sockaddr_in addr;
    struct hostent* he;
    int err;
#    ifdef _WIN32
    WSADATA wsaData;
    WORD wVersionRequested;
    wVersionRequested = MAKEWORD( 2, 2 );
    WSAStartup( wVersionRequested, &wsaData );
    if ( GetLastError( ) != 0 )
    {
        return -1;
    }
#    endif

    *handle = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
#    ifdef _WIN32
    if ( *handle == INVALID_SOCKET )
    {
#    else
    if ( *handle == -1 )
    {
#    endif
        fprintf( stderr, "Socket could not be opened.\n" );
        return -2;
    }
    he = gethostbyname( ipAddress );
    memcpy( &addr.sin_addr, he->h_addr_list[ 0 ], he->h_length );
    addr.sin_family = AF_INET;
    addr.sin_port   = htons( FT_PORT );

    err = connect( *handle, ( struct sockaddr* )&addr, sizeof( addr ) );
    if ( err < 0 )
    {
        return -3;
    }
    return 0;
}

static void UdpClose( SOCKET_HANDLE* handle )
{
#    ifdef _WIN32
    closesocket( *handle );
    WSACleanup( );
#    else
    close( *handle );
#    endif
}

static void SendCommand( SOCKET_HANDLE* socket, uint16 command, uint32 data )
{
    unsigned char request[ 8 ];
    *( uint16* )&request[ 0 ] = htons( 0x1234 );
    *( uint16* )&request[ 2 ] = htons( command );
    *( uint32* )&request[ 4 ] = htonl( data );
    send( *socket, ( const char* )request, 8, 0 );
    MySleep( 5 );  // Wait a little just to make sure that the command has been processed by Ethernet DAQ
}

static Response Receive( SOCKET_HANDLE* socket )
{
    unsigned char inBuffer[ 36 ];
    Response response;
    unsigned int uItems = 0;
    recv( *socket, ( char* )inBuffer, 36, 0 );
    response.sequenceNumber = ntohl( *( uint32* )&inBuffer[ 0 ] );
    response.sampleCounter  = ntohl( *( uint32* )&inBuffer[ 4 ] );
    response.status         = ntohl( *( uint32* )&inBuffer[ 8 ] );
    response.fx             = ( ntohl( *( int32* )&inBuffer[ 12 + ( uItems++ ) * 4 ] ) );
    response.fy             = ( ntohl( *( int32* )&inBuffer[ 12 + ( uItems++ ) * 4 ] ) );
    response.fz             = ( ntohl( *( int32* )&inBuffer[ 12 + ( uItems++ ) * 4 ] ) );
    response.tx             = ( ntohl( *( int32* )&inBuffer[ 12 + ( uItems++ ) * 4 ] ) );
    response.ty             = ( ntohl( *( int32* )&inBuffer[ 12 + ( uItems++ ) * 4 ] ) );
    response.tz             = ( ntohl( *( int32* )&inBuffer[ 12 + ( uItems++ ) * 4 ] ) );

    return response;
}

static Response GetReal( Response r )
{
    Response real;

    real.fx = r.fx / FORCE_DIV;
    real.fy = r.fy / FORCE_DIV;
    real.fz = r.fz / FORCE_DIV;
    real.tx = r.tx / TORQUE_DIV;
    real.ty = r.ty / TORQUE_DIV;
    real.tz = r.tz / TORQUE_DIV;

    return real;
}

static void ShowResponse( Response r )
{
    double fx = r.fx / FORCE_DIV;
    double fy = r.fy / FORCE_DIV;
    double fz = r.fz / FORCE_DIV;
    double tx = r.tx / TORQUE_DIV;
    double ty = r.ty / TORQUE_DIV;
    double tz = r.tz / TORQUE_DIV;
#    if UNIT == 1
    fprintf( stdout, "S:%u SN: %u SC: %u Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz );
#    else
    fprintf( stdout, "S:%u SN: %u SC: %u Fx: %.2f Fy: %.2f Fz: %.2f Tx: %.2f Ty: %.2f Tz: %.2f\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz );
#    endif
    fflush( stdout );
}

#endif

// int main( int argc, char** argv )
// {
//     Response r;
//     unsigned int i;
//     SOCKET_HANDLE socketHandle; /* Handle to UDP socket used to communicate with Ethernet DAQ. */

//     if ( Connect( &socketHandle, "192.168.1.105", FT_PORT ) != 0 )
//     {
//         fprintf( stderr, "Could not connect to device..." );
//         return -1;
//     }
//     SendCommand( &socketHandle, COMMAND_SPEED, FT_SPEED );
//     SendCommand( &socketHandle, COMMAND_FILTER, FT_FILTER );
//     SendCommand( &socketHandle, COMMAND_BIAS, FT_BIASING_OFF );
//     // SendCommand( &socketHandle, COMMAND_START, FT_SAMPLE_COUNT );

//     for ( int j{ 0 }; j < 1000; j++ )
//     {
//         SendCommand( &socketHandle, COMMAND_START, FT_SAMPLE_COUNT );
//         if ( j == 10 )
//         {
//             SendCommand( &socketHandle, COMMAND_BIAS, FT_BIASING_ON );
//         }
//         for ( i = 0; i < FT_SAMPLE_COUNT; ++i )
//         {
//             r = Receive( &socketHandle );
//             ShowResponse( r );
//         }
//     }
//     // Close(&socketHandle);
//     return 0;
// }
