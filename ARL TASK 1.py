import matplotlib.pyplot as plt
import cv2
import numpy as np
import random
from math import atan2, cos, sin
import math
import copy
import time
class vertice:
    x = -1
    y = -1
    cost = 0
    #ntype = 2 #0 - start 1 - end 2 - intermediate
    def __init__(self, x, y,parent, cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost

def black(b, g, r):
    return (b < 10) * (g < 10) * (r < 10)

def blue(b, g, r):
    return (b > 200) * (g < 100) * (r < 100)

def red(b, g, r):
    return (b < 100) * (g < 100) * (r > 200)

def green(b, g, r):
    return (b < 100) * (g > 200) * (r < 100)

def white(b, g, r):
    return (b > 200 ) * (g > 200) * (r > 200)

class RRT:
    
    def __init__(self, img, startp, endp, bias = 0, manr = 100, expandDis = 0.5):
        self.b, self.g, self.r = cv2.split(img)
        self.black = black(self.b, self.g, self.r)
        self.blue = blue(self.b, self.g, self.r)
        self.green = green(self.b, self.g, self.r)
        self.img = img
        self.xdim = img.shape[1]-1
        self.ydim = img.shape[0]-1
        self.root = vertice(startp[0], startp[1], -1, 0)
        self.startp = startp
        self.endp = endp
        self.bias = bias
        self.manr = manr
        self.expandDis = expandDis
        self.vertices = []
        self.vertices.append(self.root)
    
    def collision_check(self, p):
        
        if self.blue[p[1]][p[0]] or self.black[p[1]][p[0]]:
            return False
        return True

    def dist(self, p1, p2):
        return abs(p1[0] - p2[0] ) + abs(p1[1] - p2[1])

    def step(self, p1, p2):
        if self.dist(p1,p2) <= self.manr:
            
            return p2
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        p =  np.array(p1)
        st = (p2[1] - p1[1])/(abs(self.dist(p2, p1))+1)
        ct = (p2[0] - p1[0])/(abs(self.dist(p2, p1))+1)
        p = p.astype('float64')
        while self.dist(p, p1) < self.manr:
            if not self.collision_check([int(round(p[0])), int(round(p[1]))]) or self.green[int(round(p[0])), int(round(p[1]))]:
                return int(round(p[0])), int(round(p[1]))
            p += np.array([ct, st])
        return int(round(p[0])), int(round(p[1]))

    def get_nearest_vertice_index(self, p):
        dlist = [abs(pt.x - p[0])+ abs(pt.y - p[1]) for pt in self.vertices]
        minind = dlist.index(min(dlist))
        return minind
    
    def get_random_point(self):
        while True:
            if random.random() < self.bias:
                return endp
            p = int(random.random()*self.xdim), int(random.random()*self.ydim)
            #if self.collision_check(p):
            return p
    def draw(self):
        img2 = self.img.copy()
        for i in range(0, len(self.vertices)):
            if self.vertices[i].parent != -1:
                cv2.line(img2,(self.vertices[i].x, self.vertices[i].y),(self.vertices[self.vertices[i].parent].x,self.vertices[self.vertices[i].parent].y),(255,0,0),5)
            cv2.circle(img2,(self.vertices[i].x,self.vertices[i].y), 20, (0,255,255), -1)
        return img2
    def draw_path(self, img2):
        cv = self.path_vertice
        while cv.parent != -1:
            cv2.line(img2,(cv.x, cv.y),(self.vertices[cv.parent].x,self.vertices[cv.parent].y),(255,0,0),5)
            cv = self.vertices[cv.parent]
    def get_path(self):
        path = []
        path.append(self.path_vertice)
        cv = self.path_vertice

        while cv.parent != -1:
            path.append(self.vertices[cv.parent])
            cv = self.vertices[cv.parent]
        path.reverse()
        path2 = []
        for u, v in zip(path[:-1], path[1:]):
            theta = atan2(v.y - u.y, v.x - u.x)
            p = [u.x, u.y]
            f = [v.x, v.y]
            path2.append(p)
            p = np.array([u.x, u.y])
            st = (f[1] - p[1])/(abs(self.dist(f, p))+1)
            ct = (f[0] - p[0])/(abs(self.dist(f, p))+1)
            p = p.astype('float64')
            while int(round(p[0])) != f[0] or int(round(p[1])) != f[1]:
                p += np.array([ct, st])
                path2.append([int(round(p[0])), int(round(p[1]))])
        path3 = [path2[0]]
        for i in range(1, len(path2)):
            if path3[len(path3)-1] != path2[i]:
                path3.append(path2[i])
        return path3
    
    def check_draw_path(self, fr2, path):
        for k in path:
            if not self.collision_check(k):
                return False
            fr2[k[1], k[0]] = np.array([120,120,120])
        return True
    
    def check_path(self, path):
        for k in path:
            if not self.collision_check(k):
                return False
        return True

    def change_frame(self, img):
        self.b, self.g, self.r = cv2.split(img)
        self.black = black(self.b, self.g, self.r)
        self.blue = blue(self.b, self.g, self.r)
        self.green = green(self.b, self.g, self.r)
        self.img = img
        self.xdim = img.shape[1]-1
        self.ydim = img.shape[0]-1
    
    def regenerate_path(self, startp):
        self.vertices = []
        self.root = vertice(startp[0], startp[1], -1, 0)
        self.startp = startp
        self.vertices.append(self.root)
        self.rrt_init()

    
    def rrt_init(self ):
        while True:
            rnd = self.get_random_point()
            n = self.get_nearest_vertice_index(rnd)
            nv = self.vertices[n]
            pt = self.step([nv.x, nv.y], rnd)
            newv = vertice(pt[0], pt[1],  n, 0)
            if self.collision_check(pt):
                self.vertices.append(newv)
                if self.green[newv.y, newv.x]:
                    self.path_vertice = newv
                    print("Found Path!")
                    return
            continue
            

    
cv2.namedWindow('image')
cap = cv2.VideoCapture('dynamic_obstacles.mp4')
i = 0
j = 0
path = 0
pp = 0
rrt = 0
speed = 1
patho = 0
treedraw = False

print("Input speed in pixel per frame: ")
speed = int(input())
print("Input bias for rrt targeting: ")
bias = float(input())
print("Input maxd for rrt targeting: ")
maxd = int(input())
while(cap.isOpened()):
    ret, frame = cap.read()
    fr2 = frame[20:1060,440:1470]
    if i == 0:
        fr3 = fr2.copy()
        b, g, r = cv2.split(fr3)
        b = b/255
        g = g/255
        r = r/255
        endp = [np.argmax((g > 0.8) - (g*b*r)) % fr2.shape[1], np.argmax((g > 0.8) - (g*b*r)) // fr2.shape[1]]
        startp = [np.argmax((r > 0.8) - (g*b*r)) % fr2.shape[1], np.argmax((r > 0.8) - (g*b*r)) // fr2.shape[1]]
        print(startp, endp)
        rrt=RRT(fr3, startp, endp, bias, manr=maxd)
        rrt.rrt_init()
        path = rrt.get_path()
        fr2 = rrt.draw()
        treedraw = True
    else:
        rrt.change_frame(fr2.copy())
        if rrt.check_draw_path(fr2, path):
            pp = path[0]
        else:
            
            print("Creating new path!")
            rrt.regenerate_path(pp)
            path = rrt.get_path()
            pp = path[0]
            fr2 = rrt.draw()
            treedraw = True

        cv2.circle(fr2,(path[0][0],path[0][1]), 5, (0,125,255), -1)
        if len(path) > speed:
            path = path[speed:]
        else:
            path = path[1:]
    i+=1
    
    cv2.imshow('image',fr2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if treedraw:
        time.sleep(3.1)
        treedraw = False


cap.release()
cv2.destroyAllWindows()