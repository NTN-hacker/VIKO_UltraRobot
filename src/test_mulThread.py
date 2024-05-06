import sys
import threading

sys.path.append("E:\\Project\\Robot-6DOF\\VIKO_UltraRobot")
from layout import app_robot as app


class TestThread:
    def __init__(self, name):
        self.name = name

    def run(self):
        print("Starting " + self.name)
        print("Exiting " + self.name)


if __name__ == "__main__":
    app.main()
