import sys
from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel
from PyQt6.QtCore import Qt
import threading
import time


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Program Controller")
        self.setGeometry(100, 100, 300, 200)

        self.label = QLabel("Program is Stopped", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setGeometry(50, 50, 200, 30)

        self.button = QPushButton("Start Program", self)
        self.button.setGeometry(100, 100, 100, 50)
        self.button.clicked.connect(self.toggle_program)

        self.program_running = False
        self.program_thread = None

    def toggle_program(self):
        if not self.program_running:
            self.start_program()
            self.label.setText("Program is Running")
            self.button.setText("Stop Program")
        else:
            self.stop_program()
            self.label.setText("Program is Stopped")
            self.button.setText("Start Program")

    def start_program(self):
        self.program_running = True
        self.program_thread = threading.Thread(target=self.run_program)
        self.program_thread.start()

    def run_program(self):
        while self.program_running:
            print("Program is running...")
            time.sleep(1)

    def stop_program(self):
        self.program_running = False
        if self.program_thread and self.program_thread.is_alive():
            self.program_thread.join()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
