import cv2
import numpy
import sys

if __name__ == '__main__':
    
    tracker = cv2.TrackerMIL_create()
    video = cv2.VideoCapture(1)

    if not video.isOpened():
        print "Cloud not open video"
        sys.exit()

    ok, frame = video.read()
    if not ok:
        print "Cannot read video file"

    bbox = (287, 23, 86, 320)
    ok = tracker.init(frame, bbox)

    while True:
        ok ,frame = video.read()
        if not ok:
            break
        ok, bbox = tracker.update(frame)

        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0,0,255))
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        cv2.imshow("Tracking", frame)

        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
# W = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
# H = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
# count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
# fps = cap.get(cv2.CAP_PROP_FPS)

# while True:
#     ret, frame = cap.read()
# 
#     if not ret:
#         print "Cannot not open video"
#         sys.exit()
# 
#     bbox = (287, 23, 86, 320)
# 
#     ret = tracker.init(frame, bbox)
# 
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(1) == 27:
#         break
#     # print "width:"+str(W)+"height:"+str(H)
#     # print "count"+str(count)+"fps"+str(fps)
#     
# cap.release()
# cv2.destroyAllWindows()
