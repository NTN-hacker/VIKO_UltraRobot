import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QMessageBox
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QIcon
import threading
import time
import os
# sys.path.append("D:\\Quan\\roboDK\\Vision-Machine-collab-Nhan\\VIKO_UltraRobot")
sys.path.append(
    "E:\\Project\\Robot-6DOF\\VIKO_UltraRobot"
)

from src import motion_limit as ML
# from src import robotic_modify as RM

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Ohlabs Robot Control")
        self.setGeometry(100, 100, 300, 200)
        self.setStyleSheet("background-color: grey;")  # Set background color

        self.label = QLabel("Program is Stopped", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setGeometry(50, 50, 200, 30)

        # Scale the icon
        self.iconOn = QIcon("layout\\png\\on.png")
        self.iconOff = QIcon("layout\\png\\off.png")

        self.button = QPushButton("Start Program", self)
        self.button.setGeometry(100, 100, 100, 50)
        self.button.clicked.connect(self.toggle_program)
        self.setWindowIcon(QIcon("layout\\png\\icon.png"))
        self.program_running = False
        self.program_thread = None

    def toggle_program(self):
        if not self.program_running:
            self.start_program()
            self.label.setText("Program is Running")
            self.button.setText("On")
            self.button.setIcon(self.iconOn)
        else:
            self.stop_program()
            self.label.setText("Program is Stopped")
            self.button.setText("Off")
            self.button.setIcon(self.iconOff)

    def start_program(self):
        """
        Starts the program in a separate thread.

        This function sets the program_running flag to True,
        creates a new thread and starts the run_program method.
        """
        self.program_running = True
        self.program_thread = threading.Thread(target=self.run_program)
        # Start the thread
        self.program_thread.start()

    def run_program(self):
        if self.program_running is True:
            print("Program is running...")
            # RM.mainRob()
            obj = ML.TestThread("test")
            obj.run()
            time.sleep(1)

    def stop_program(self):
        self.program_running = False
        if self.program_thread and self.program_thread.is_alive():
            self.program_thread.join()
        reply = QMessageBox()
        reply.setText('Are you sure you want to exit the program?')
        reply.setStandardButtons(QMessageBox.StandardButton.Yes | 
                     QMessageBox.StandardButton.No)
        
        x = reply.exec()
        
        

        if x == QMessageBox.standardButtons.Yes:
            # Perform any necessary cleanup operations here
            # ...

            # Exit the Python program
            sys.exit()

    def closeEvent(self, event):
        """Override the close event to stop the program thread before closing the window."""
        self.stop_program()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


