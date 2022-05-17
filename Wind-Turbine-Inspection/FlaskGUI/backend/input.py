import time
from abc import ABC
from threading import Thread

import cv2


class RenderInput(ABC):
    def __init__(self, render=True, verbose=True):
        self.render = render
        self.verbose = verbose

    def getInput(self):
        pass


# ASYNC is not working in debug mode.
class RenderVideoInput(RenderInput):
    def __init__(self, camera_index=0, sleep_time=0, async_stream=False, render=True, verbose=True):
        super().__init__(render, verbose)
        self.camera = cv2.VideoCapture(camera_index)
        self.sleep_time = sleep_time
        self.async_stream = async_stream
        if self.async_stream:
            self.frame_queue = []
            self.thread = Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()

    def genFrame(self):
        if self.async_stream:
            return self.getAsyncFrame()
        else:
            return self.getInput()

    def getInput(self):
        while True:
            if not self.render:
                time.sleep(1)
                continue
            success, frame = self.camera.read()  # read the camera frame
            if not success:
                break
            else:
                frame = cv2.flip(frame, 1)
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result

    # def timer_test(self):
    #     while True:
    #         print("Test loop")
    #         if not self.render:
    #             time.sleep(5)
    #             continue
    #         time.sleep(5)
    #         if self.__camera.isOpened():
    #             success, frame = self.__camera.read()  # read the camera frame
    #             if success:
    #                 self.__frame_queue.append(frame)
    #                 print(f"THREAD:: len of queue is {len(self.__frame_queue)}")

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.camera.isOpened():
                if self.verbose:
                    print("THREAD UPDATE")
                (self.status, self.frame) = self.camera.read()
                self.frame_queue.append(self.frame)
            time.sleep(self.sleep_time)

    def getAsyncFrame(self):
        while True:
            if self.verbose:
                print(f"MAIN:: len of queue is {len(self.frame_queue)}")
                # print(f"MAIN:: Frame value {self.frame is None}")
            if len(self.frame_queue) == 0 or not self.render:
                time.sleep(1)
                continue
            if self.verbose:
                print(f"MAIN:: FOUND ONE")
            frame = self.frame_queue.pop()
            frame = cv2.flip(frame, 1)
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
