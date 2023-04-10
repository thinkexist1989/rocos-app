#pragma one 


#include <arpa/inet.h>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <net/if.h>
#include <netinet/in.h>
#include <openssl/des.h>
#include <openssl/err.h>
#include <openssl/md5.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>
#include <openssl/sha.h>
#include "plog/Appenders/ColorConsoleAppender.h"
#include "plog/Appenders/ConsoleAppender.h"
#include "plog/Appenders/RollingFileAppender.h"
#include "plog/Formatters/CsvFormatter.h"
#include "plog/Formatters/TxtFormatter.h"
#include "plog/Init.h"
#include "plog/Log.h"
#include <set>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "JC_helper_kinematics.hpp"

namespace JC_helper
{
    /**
     * @brief 
     * 
     * @return int 
     */
    int authentication( );
}

