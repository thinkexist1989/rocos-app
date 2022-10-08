#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <stddef.h>


namespace constants
{
    inline constexpr size_t _robot_num  { 7 }; // 机械臂关节,需要c++17
    inline constexpr size_t _slave_num  { 9 }; // 主站全部关节

    // ... other related constants
}
#endif