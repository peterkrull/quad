import time
import socket
import threading

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
server.settimeout(0.2)

ip = "<broadcast>"
port = 51000
server.bind((ip,port))

receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receiver.settimeout(0.2)

ip2 = "0.0.0.0"
port2 = 50000
receiver.bind((ip2,port2))

def udp_heartbeat():
    while True:
        server.sendto("\n".encode(),("192.168.1.162",port))
        time.sleep(0.2)

message = ""
def udp_sender():
    global message
    while True:
        message = input()

        # print("send :",message)
        for i in range(10):
            server.sendto((message+"\n").encode(),("192.168.1.162",port))
            time.sleep(0.05)

prev_time = 0
def udp_receiver():
    global prev_time,message
    while True:
        try:
            data, address = receiver.recvfrom(1024)
            if time.time() > prev_time + 0.01:
                prev_time = time.time()
                if data.decode() == message:
                    print("ECHO MATCH")
                else:
                    print("echo :",  data.decode())
        except Exception as e:
            # print(f"Timeout {e}")
            time.sleep(0.04)
        

t_udp_heartbeat = threading.Thread(None,udp_heartbeat)
t_udp_heartbeat.start()

t_udp_receiver = threading.Thread(None,udp_receiver)
t_udp_receiver.start()

t_udp_sender = threading.Thread(None,udp_sender)
t_udp_sender.start()

