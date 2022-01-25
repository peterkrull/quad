import time
import socket

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
server.settimeout(0.2)

ip = "<broadcast>"
port = 51000
server.bind((ip,port))

while True:
    message = input()
    print("sending : ",message)
    server.sendto((message+"\n").encode(),("192.168.1.135",port))
    