import cv2
import time

try:
    from .camera_calibrator import OpenCVCalibration, ChessboardInfo
except ImportError:
    from camera_calibrator import OpenCVCalibration, ChessboardInfo

if __name__ == '__main__':
    capture = cv2.VideoCapture(0)
    boards = []
    boards.append(ChessboardInfo(8,6,0.0245))
    calibrator = OpenCVCalibration(capture, boards)
    while True:
        time.sleep(1)
