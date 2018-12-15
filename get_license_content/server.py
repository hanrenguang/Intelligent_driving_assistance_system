#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from socket import *
from time import ctime
from hyperlpr import *
import cv2

HOST = "localhost"
PORT = 3000
BUFSIZ = 4096
ADDR = (HOST, PORT)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(2)

while True:
    print "waiting for connection..."
    tcpCliSock, addr = tcpSerSock.accept()
    print "...connected from: ", addr

    while True:
        data = tcpCliSock.recv(BUFSIZ)
        if not data:
            break
        img = cv2.imread(data)
        res = HyperLPR_PlateRecogntion(img)
        print "res: %s" % (res[0][0].encode("utf-8"))
        tcpCliSock.send('[%s] %s' % (ctime(), res[0][0].encode("utf-8")))

tcpSerSock.close()