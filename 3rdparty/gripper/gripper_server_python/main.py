#!/usr/bin/env python3
import sys
import socket
import comModbusRtu
import baseRobotiq2FGripper
import time
import _thread
import atexit 



gripper = baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
command = [100, 100, 100]  # 位置、速度、力
flag_turnoff = False
my_server = 0

@atexit.register 

def clean(): 
    global my_server
    my_server.close()  
    time.sleep(0.6)

def tcpServer(var):
    global my_server
    global flag_turnoff

    host = "127.0.0.1"
    port = 5000
    s = socket.socket()
    s.bind((host, port))
    s.listen(10)  # 只能同时连接一个

    while ~flag_turnoff:
        my_server, address = s.accept()

        while True:
            data = my_server.recv(1024)  # 接收buffer的大小
            data = str(data)

            data = data[2: len(data)-1]
            print(data)

            res = data.split('#')
            count = 0

            if(data!=""):
                for i in res:
                    command[count] = int(i)
                    count = count+1

                # if  int(res[0])== -1 and int(res[1])== -1  and int(res[2])== -1:
                #     print("break")
                #     flag_turnoff =True
                #     break
                # else:
                    gripper.my_refresh_Command(1, 0, 1, command[0], command[1], command[2])
                    # time.sleep(0.6)
            else:
                break
                
    my_server.close()


def mainLoop(device):
    global flag_turnoff
    gripper.client = comModbusRtu.communication()
    gripper.client.connectToDevice(device)
    gripper.my_refresh_Command(0, 0, 0, 0, 0, 0) #爪子初始化

    while ~flag_turnoff:

        # status = gripper.getStatus()
        # print(status)
        gripper.sendCommand()
    


if __name__ == '__main__':
    print("gripper 启动")
    _thread.start_new_thread(tcpServer, ("TCP",))
    mainLoop("/dev/serial_gripper")
    print("gripper 结束")
    flag_turnoff= True
    my_server.close()
    my_server.clean()
    time.sleep(0.6)
