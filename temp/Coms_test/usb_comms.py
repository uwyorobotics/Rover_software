#shebang

'''this is a simple module to use opencv to read in an image, then compress it, then send it
record the size of the frame. 

-Wifi Halo currently sends at about 500kb/s
jpeg_10 compression gets down to 10kB - 80kb per frame
-thats 6 frames per second
'''

import cv2
import sys
import time
import socket
import pickle
import struct

loops_per_second = 10
time_per_loop = 1/loops_per_second #1 1/(frames/second)  =seconds/frame

def JPEG_compression(image, quality=90):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),quality]
    _,encode_img = cv2.imencode('.jpg',image,encode_param)
    im_size = sys.getsizeof(encode_img)
    decoded_image = cv2.imdecode(encode_img, cv2.IMREAD_COLOR)
    return im_size,decoded_image

def JPEG_return_compressed(image, quality=90):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),quality]
    _,encode_img = cv2.imencode('.jpg',image,encode_param)
    im_size = sys.getsizeof(encode_img)
    #decoded_image = cv2.imdecode(encode_img, cv2.IMREAD_COLOR)
    return im_size,encode_img

#init the socket
send_portal = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host = '169.254.188.191'
port = 1234

send_portal.connect((host,port))

#init the video capture
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print('no camera, exiting')
    exit()

while True:
    start = time.perf_counter()
    ret,frame = camera.read()

    if not ret:
        print('camera stream broken, exiting')
        break
    #no compression: size = 921744; ~1MB
    print(f'size: {sys.getsizeof(frame)}')
    cv2.imshow('raw frame',frame)
    #JPEG compression gets us down to 75836 bytes, ~76KB
    jsize, jpg_frame = JPEG_compression(frame)
    print(f'jpeg size: {jsize}')
    cv2.imshow('JPeg with size 90',jpg_frame)
    #setting the quality down to 10 gets us down to 12727 bytes, ~13KB
    j10size, jpg_frame_10 = JPEG_compression(frame,10)
    print(f'jpeg_10 size: {j10size}')
    cv2.imshow('JPEG size 10',jpg_frame_10)
    j10_compressed_size,jpeg_10_compressed = JPEG_return_compressed(frame, 10)


    data = pickle.dump(jpeg_10_compressed)
    message = struct.pack("Q",len(data)) + data
    send_portal.sendall(message)


    #this line shows the image
    #cv2.imshow('frame',jpg_frame)
    if cv2.waitKey(1) == ord('q'):
        break
    end = time.perf_counter()
    print(f'frames per second {loops_per_second}')
    print(f'time Elapsed: {end-start}')
    print(f'time_to_wait: {time_per_loop - (end-start)}')
    if time_per_loop > (end-start):
        time.sleep(time_per_loop - (end-start))
camera.release()
cv2.destroyAllwindows()
send_portal.close


