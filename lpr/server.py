#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from socket import *
from time import ctime
from hyperlpr import LPR
import cv2
import os

HOST = "localhost"
PORT = 3000
BUFSIZ = 1024
ADDR = (HOST, PORT)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(1)

PR = LPR(os.path.join(os.path.split(os.path.realpath(__file__))[0],"models"))

while True:
    print "waiting for connection..."
    tcpCliSock, addr = tcpSerSock.accept()
    print "...connected from: ", addr

    while True:
        data = tcpCliSock.recv(BUFSIZ)
        if not data:
            print "exit"
            break
        elif data == "1":
            img = cv2.imread("../build/ROI.png")
            res = PR.plateRecognition(img, True)
            print "[%s] %s\0" % (ctime(), res[0][0].encode("utf-8"))
            tcpCliSock.send('[%s] %s\0' % (ctime(), res[0][0].encode("utf-8")))

tcpSerSock.close()
