# https://opencv-tutorial.readthedocs.io/en/latest/yolo/yolo.html
# https://docs.opencv.org/master/d6/d0f/group__dnn.html
# https://docs.opencv.org/3.4/db/d30/classcv_1_1dnn_1_1Net.html
# https://github.com/opencv/opencv/blob/master/samples/dnn/object_detection.py
import cv2 as cv
import numpy as np
import time

# # Load an image

# frame = cv.imread("qr1.png")
# frame = cv.resize(frame,(1024,720))

threshold = 0.6  
# maxWidth = 4096; maxHeight = 2304
# # imgHeight, imgWidth = frame.shape[:2]
# hScale = 1; wScale = 1
thickness = 1

# if imgHeight > maxHeight:
#     hScale = imgHeight / maxHeight
#     thickness = 6

# if imgWidth > maxWidth:
#     wScale = imgWidth / maxWidth
#     thickness = 6

# # Load class names and YOLOv3-tiny model
classes = open('qrcode_yolo3/qrcode.names').read().strip().split('\n')
# net = cv.dnn.readNetFromDarknet('qrcode-yolov3-tiny.cfg', 'qrcode-yolov3-tiny.weights')
# net = cv.dnn.readNetFromDarknet('qrcode-yolov3.cfg', 'qrcode-yolov3_last.weights')
# net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
# net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU) # DNN_TARGET_OPENCL DNN_TARGET_CPU DNN_TARGET_CUDA

# start_time = time.monotonic()
# # Convert frame to blob
# blob = cv.dnn.blobFromImage(frame, 1/255, (416, 416), swapRB=True, crop=False)
# elapsed_ms = (time.monotonic() - start_time) * 1000
# print('blobFromImage in %.1fms' % (elapsed_ms))

def postprocess(frame, outs):
    frameHeight, frameWidth = frame.shape[:2]
    cropped_image = frame.copy()

    classIds = []
    confidences = []
    boxes = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > threshold:
                x, y, width, height = detection[:4] * np.array([frameWidth, frameHeight, frameWidth, frameHeight])
                left = int(x - width / 2)
                top = int(y - height / 2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, int(width), int(height)])

    indices = cv.dnn.NMSBoxes(boxes, confidences, threshold, threshold - 0.1)
    score = 0.0
    left = None
    top = None
    width = None
    height = None
    for i in indices:
        if confidences[i[0]] > score:
            score = confidences[i[0]]
            i = i[0]
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            cropped_image = frame[top:top + height, left:left + width]
            # cv.imshow('cropped', cropped_image)
            # cv.imwrite('cropped.jpg', cropped_image)

            # Draw bounding box for objects
            cv.rectangle(frame, (left, top), (left + width, top + height), (0, 0, 255), thickness)

            # Draw class name and confidence
            label = '%s:%.2f' % (classes[classIds[i]], confidences[i])
            print(label)
            cv.putText(frame, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
    return cropped_image ,(left, top, width, height)


def get_qrcode_range(frame):
    # Load an image

    # Load class names and YOLOv3-tiny model
    # net = cv.dnn.readNetFromDarknet('qrcode-yolov3-tiny.cfg', 'qrcode-yolov3-tiny.weights')
    net = cv.dnn.readNetFromDarknet('qrcode_yolo3/qrcode-yolov3.cfg', 'qrcode_yolo3/qrcode-yolov3_last.weights')
    net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
    # net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU) # DNN_TARGET_OPENCL DNN_TARGET_CPU DNN_TARGET_CUDA

    start_time = time.monotonic()
    # Convert frame to blob
    blob = cv.dnn.blobFromImage(frame, 1/255, (416, 416), swapRB=True, crop=False)
    # elapsed_ms = (time.monotonic() - start_time) * 1000
    # print('blobFromImage in %.1fms' % (elapsed_ms))
    # Determine the output layer
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    net.setInput(blob)
    outs = net.forward(ln)

    cropped_image ,pos = postprocess(frame, outs)
    elapsed_ms = (time.monotonic() - start_time) * 1000
    print('blobFromImage in %.1fms' % (elapsed_ms))
    #if type(cropped_image) =='NoneType':
    # if cropped_image is None:
    #     cropped_image = frame.copy()

    # cv.imshow('QR Detection', frame)
    # cv.waitKey()
    return cropped_image, frame, pos
