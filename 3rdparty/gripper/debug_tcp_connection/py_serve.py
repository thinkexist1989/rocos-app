# 这里python作为服务器, 等待stoke每次的访问
import socket
import time


def tcpServer():
    host = "127.0.0.1"
    port = 5000
    s = socket.socket()
    s.bind((host, port))
    s.listen(1)  # 只能同时连接一个
    my_server, address = s.accept()
    print("connection from ", str(address))
    while True:
        data = my_server.recv(1024)  # 接收buffer的大小
        if not data:
            break
        print("from connected user :", (data))
        data = str(data).upper()
        print("sending data :", data)
        my_server.send(data.encode())
        time.sleep(2)
    my_server.close()


if __name__ == '__main__':
    tcpServer()
