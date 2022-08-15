# time:2022年07月28日16:13:15记录，调试日记

1. 离线规划类型：
    1.1. moveL
    1.2. moveC
    1.3. MoveJ
    1.4. MultiMoveL

2. 在线规划类型：
    2.1. Dragging[关节空间]
    2.2. Dragging[笛卡尔空间]
    2.3. admittance_teaching导纳示教
    2.4. admittance_link导纳直线

3. 注意：
    3.1. 离线规划全部[moveL、moveC、MoveJ、MultiMoveL]内置速度检查[取值都来自urdf的max_vel_]，不使用check_vel_acc()
    3.2. 在线规划情况复杂且危险，为了统一接口而设计了check_vel_acc()
    3.3. 目前admittance_link、admittance_teaching、Dragging(笛卡尔空间)都使用了check_vel_acc()，并且最大速度和最大加速度看情况设置[比如Dragging(笛卡尔空间)的最大加速度检查我就设置为了2]
    3.4. Dragging(关节空间)没有速度保护，只有命令位置保护，伺服位置保护，但是我认为足够安全了


# time:2022年08月15日14:43:54 更新日记
1. MoveJ包含:
    1. 命令位置保护:CheckBeforeMove()
    2. 速度检查
    3. 不包含伺服位置检查，因为这由算法dobuleS保证，我认为这足够安全

2. moveL、moveC、MultiMoveL包含：
    1. 伺服位置检查,由逆解算法提供
    2. 速度检查

3. admittance_link、admittance_teaching、Dragging(笛卡尔空间)包含：
    1. 速度和加速度保护，由check_vel_acc()提供，会触发紧急关节空间停止Joint_stop()
    2. 位置伺服保护：safety_servo()

4. Dragging(关节空间)包含：
    1. 命令位置保护（）
    2. 伺服位置保护
    3. 速度保护由OTG算法提供







