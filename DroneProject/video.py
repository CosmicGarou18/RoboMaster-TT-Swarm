from djitellopy import Tello
import cv2

tello = Tello("10.65.164.76")


#fix for colours
frame = frame_read.frame
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
cv2.imshow("Tello", frame)


tello.connect()
tello.streamon()

frame_read = tello.get_frame_read()

while True:
    frame = frame_read.frame

    if frame is None:
        continue

    original = frame.copy()

    converted = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    cv2.imshow("Original", original)
    cv2.imshow("Converted", converted)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
tello.streamoff()