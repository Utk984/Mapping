import math
import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import sys
import tty
import termios
import os
import heapq
from numpy import Inf

boundary = 2.7
per = 5
pocc = 0.8
pfree = 0.1
prior = 0
log = np.zeros((per,per))
space = np.arange(1, per*per+1).reshape(per, per)
arduino_port = '/dev/ttyUSB0'
baud_rate = 9600
turn_delay = 0.3
move_delay = 3
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)

def prob(position,orien,measurement):
    print(f'\nGrid: \n{space}')
    grid = boundary/per
    print(grid)
    sign = 1
    if orien in ['U','L']:
        sign = -1

    see = []
    index = np.where(space == position)
    row, col = index[0][0], index[1][0]
    while True:
        print(measurement)
        measurement = round(measurement-grid,3)
        if orien=='R' or orien=='L':
            col = col + sign
        else:
            row = row + sign
        if measurement < 0 or row == per or col == per or row < 0 or col < 0:
            break
        see.append([row,col])

    for i in see:
        calc(i,pfree)
    if row >= 0 and row < per and col >= 0 and col < per:
        calc([row,col],pocc)
    print(f'\nLog Prob: \n{log}')

def calc(index,p):
    log[index[0],index[1]] = log[index[0],index[1]] + math.log(p/(1-p)) + prior

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def res():
    pro = (1-1/(1+np.exp(log)))
    print(f'\nFinal Probability: \n{pro}')
    image = np.where(prob > 0.7, 1, np.where(prob < 0.3, 0, 0))
    return image

def to_1d(row, col, cols):
    return row * cols + col

def Dijkstra(graph, start, end):
    rows, cols = len(graph), len(graph[0])
    l = rows * cols

    def to_2d(index):
        return divmod(index, cols)

    dist = [Inf for _ in range(l)]
    dist[start] = 0
    vis = [False for _ in range(l)]
    parent = [-1 for _ in range(l)]
    pqueue = [(0, start)]

    while len(pqueue) > 0:
        _, u = heapq.heappop(pqueue)
        if vis[u]:
            continue
        vis[u] = True
        current_row, current_col = to_2d(u)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_row, new_col = current_row + dr, current_col + dc
            if 0 <= new_row < rows and 0 <= new_col < cols:
                v = to_1d(new_row, new_col, cols)
                if graph[new_row][new_col] == 1:
                    continue
                if dist[u] + 1 < dist[v]:
                    dist[v] = dist[u] + 1
                    parent[v] = u
                    heapq.heappush(pqueue, (dist[v], v))
    path = []
    current = end
    while current != -1:
        path.append(current)
        current = parent[current]
    path.reverse()

    return path

def rotate(K,T):
    arduino.write(K.encode())
    time.sleep(turn_delay)
    arduino.write('S'.encode())

def turn(one,two,ori):
    obs = ['U','R','D','L']
    if (ori=='U' and two==one+1) or (ori=='D' and two==one-1) or (ori=='R' and two==one+5) or (ori=='L' and two==one-5):
        rotate('D')
        return obs[(obs.index(ori)+1)%4]
    elif (ori=='U' and two==one-1) or (ori=='D' and two==one+1) or (ori=='R' and two==one-5) or (ori=='L' and two==one+5):
        rotate('A')
        return obs[(obs.index(ori)-1)%4]

def move(s,a,e,p):
    ori = a
    for i in range(0,len(p)-1):
        ori = turn(p[i],p[i+1],ori)
        arduino.write('W'.encode())
        time.sleep(move_delay)
        arduino.write('S'.encode())

try:
    while True:
        print("\nPress 1 to Move\nPress 2 to Take Reading\nPress 3 to Enter State\n")
        key = get_key()
        measurment = 0.0
        print(key)
        arduino.write(key.encode())
        if key.upper() == 'Q':
            break
        if key=='T':
            time.sleep(10)
            measurement = arduino.readline().decode('utf-8').rstrip()
            if measurement=="":
                measurement=400.0
            print(measurement)
            position = int(input("\nEnter position: "))
            orien = input("Enter Action: ")
            prob(position,orien,float(measurement)/100)
    grid = res()

    s = int(input("\nEnter start position: "))
    a = input("\nEnter orientation: ")
    e = int(input("\nEnter end position: "))
    index1 = np.where(space == s)
    start_box = (index[0][0], index[1][0])
    index2 = np.where(space == e)
    end_box = (index[0][0], index[1][0])

    start_index = to_1d(*start_box, len(grid[0]))
    end_index = to_1d(*end_box, len(grid[0]))

    path = Dijkstra(grid, start_index, end_index)
    for i in range(0,len(path)):
        path[i]=path[i]+1
    print("\nShortest path:", path)
    move(s,a,e,path)

finally:
    arduino.close()
