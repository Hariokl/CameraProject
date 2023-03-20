def faceDetect():
    global angle, count, face_in_center_count
    with mp_face_detection.FaceDetection(min_detection_confidence=0.7,
                                         model_selection=0) as face_detection:
        while True:
            if count <= 0:
                frame = getFrame()

                frame = cv.flip(frame, 1)

                frame.flags.writeable = False
                frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                results = face_detection.process(frame)

                frame.flags.writeable = True
                frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

                angle, count = checkIfToMove(results, angle, count)

            k = cv.waitKey(1)
            count -= 1
            if k == ord("q"):
                print("Quit")
                break
            # sleep(0.2)


def getFrame():
    data = requests.get("http://raspberrypi:8080/&action=snapshot").content
    fet = np.frombuffer(data, dtype=np.uint8)
    return cv.imdecode(fet, cv.IMREAD_UNCHANGED)


def checkIfToMove(results_of_detection, angle, count):
    if results_of_detection.detections and count <= 0:
        relative_bb = results_of_detection.detections[0].location_data.relative_bounding_box
        cur_pos = round((2 * relative_bb.xmin + relative_bb.width) / 2, 4)

        if cur_pos <= 1 / 3 or cur_pos >= 2 / 3:
            angle -= 16
            if cur_pos >= 2 / 3:
                angle += 32
            angle = normalizeAngle(angle)
            setAngle(angle)
            count = 5

    return angle, count


def setAngle(angle):
    duty = angle / 18 + 2

    pwm.ChangeDutyCycle(duty)
    GPIO.output(17, True)
    sleep(0.3)
    pwm.ChangeDutyCycle(0)


def normalizeAngle(angle):
    return max(min(angle, 180), 0)


# useless
def setCameraToMove(move_bool):
    GPIO.output(17, move_bool)


def main():
    setup()
    faceDetect()
    end_of_life()


def setup():
    global pwm, mp_drawing, mp_face_detection, angle, max_angle, count, face_in_center_count

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    # values
    mp_face_detection = mp.solutions.face_detection
    mp_drawing = mp.solutions.drawing_utils

    # freq = 25
    pwm = GPIO.PWM(17, 50)

    # funcs to start
    # pwm.ChangeFrequency(freq) 
    pwm.start(0)
    GPIO.output(17, True)

    setAngle(angle)


def end_of_life():
    GPIO.output(17, False)
    pwm.ChangeDutyCycle(0)
    pwm.stop()
    GPIO.cleanup()
    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    import cv2 as cv
    import mediapipe as mp
    import RPi.GPIO as GPIO
    import settings
    from time import sleep, perf_counter
    import requests
    import numpy as np

    settings.variables_init()
    from settings import *

    main()
