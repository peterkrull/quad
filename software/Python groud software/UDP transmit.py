import time
import socket
import threading

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
server.settimeout(0.2)

ip = "<broadcast>"
port = 51000
server.bind((ip,port))

def udp_heartbeat():
    while True:
        server.sendto("\n".encode(),("192.168.1.135",port))
        time.sleep(0.2)

def udp_sender():
    while True:
        message = input()
        print("sending :",message)
        for i in range(10):
            server.sendto((message+"\n").encode(),("192.168.1.135",port))
            time.sleep(0.05)

t_udp_heartbeat = threading.Thread(None,udp_heartbeat)
t_udp_heartbeat.start()

t_udp_sender = threading.Thread(None,udp_sender)
t_udp_sender.start()

