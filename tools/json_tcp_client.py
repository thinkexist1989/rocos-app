#!/usr/bin/env python3
"""只接收的 TCP 客户端：实时显示服务端内容，默认连接 192.168.10.52:6666。"""
import argparse
import codecs
import logging
import socket
import sys


def receive_loop(connection):
    """直接显示 UTF-8 字节流，不等待完整 JSON 或换行；增量解码支持分片中文。"""
    decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
    while True:
        try:
            chunk = connection.recv(65536)
        except socket.timeout:
            continue  # 服务端暂时没有数据时，保持连接等待。
        text = decoder.decode(chunk, final=not chunk)
        sys.stdout.write(text)
        sys.stdout.flush()
        if not chunk:
            logging.info("服务端已关闭连接")
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.10.52")
    parser.add_argument("--port", type=int, default=6666)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port 必须为 1～65535")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        with socket.create_connection((args.host, args.port), timeout=3) as connection:
            connection.settimeout(1.0)
            logging.info("已连接 %s:%s，只接收并显示服务端内容，Ctrl+C 退出", args.host, args.port)
            receive_loop(connection)
    except KeyboardInterrupt:
        logging.info("已退出")
    except OSError as error:
        logging.error("连接或接收失败：%s", error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
