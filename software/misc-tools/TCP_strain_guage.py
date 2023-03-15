#!/usr/bin/env python

import socket
import csv
import time

TCP_IP = '192.168.1.199'
TCP_PORT = 2984
BUFFER_SIZE = 1024

file_name = "thrust_response.csv"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
#s.send(MESSAGE)

all_data = []
start_time = time.time()
curr_time = str(int(start_time))

while 1:
    try:
        timestamp = time.time()-start_time
        data = s.recv(BUFFER_SIZE).decode().split("\n")
        values = []
        for entry in range(len(data)-1):
            x = data[entry].split(",")
            values.append(float(x[1]))

        print("received data:",timestamp,values[0])

        for i in range(len(values)):
            all_data.append([timestamp+(i*0.02),round(values[i],4)])

    except KeyboardInterrupt:
        s.close()
        break

with open(curr_time + "_" + file_name,'w',newline="") as f:
    print("Saving data..")
    writer = csv.writer(f)
    for data in all_data:
        writer.writerow(data)
    print("All data saved.")