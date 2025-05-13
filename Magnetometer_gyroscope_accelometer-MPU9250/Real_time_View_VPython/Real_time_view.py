import sys
import serial
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# Set correct serial port and baud rate
SERIAL_PORT = "COM3"  # Change to match your system
BAUD_RATE = 115200

class IMUReader:
    """Reads roll, pitch, and yaw data from MPU9250 sensor."""
    def __init__(self, port, baud_rate):
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.ser = None

    def get_data(self):
        """Gets real-time IMU rotation data (roll, pitch, yaw)."""
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode().strip()
                values = list(map(float, line.split(",")))  # Expecting "roll, pitch, yaw"
                return np.deg2rad(values)  # Convert degrees to radians
            except ValueError:
                return [0, 0, 0]
        return [0, 0, 0]

class OpenGLWidget(QOpenGLWidget):
    """PyQt Widget with OpenGL rendering."""
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.imu_reader = IMUReader(SERIAL_PORT, BAUD_RATE)
        self.rotation = [0, 0, 0]
        
        # Timer to update rotation every 50ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateRotation)
        self.timer.start(50)

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 1, 50)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -5)
        
        # Apply IMU-based rotation
        glRotatef(np.rad2deg(self.rotation[2]), 0, 0, 1)  # Yaw
        glRotatef(np.rad2deg(self.rotation[1]), 0, 1, 0)  # Pitch
        glRotatef(np.rad2deg(self.rotation[0]), 1, 0, 0)  # Roll

        # Draw a 3D cube
        self.drawCube()

    def updateRotation(self):
        """Updates rotation based on MPU9250 data."""
        self.rotation = self.imu_reader.get_data()
        self.update()

    def drawCube(self):
        """Draws a 3D cube with OpenGL."""
        vertices = [
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1,  1), (1, -1,  1), (1, 1,  1), (-1, 1,  1)
        ]
        edges = [
            (0,1), (1,2), (2,3), (3,0),
            (4,5), (5,6), (6,7), (7,4),
            (0,4), (1,5), (2,6), (3,7)
        ]

        glBegin(GL_LINES)
        glColor3f(1.0, 1.0, 1.0)  # White
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()

class MainWindow(QMainWindow):
    """Main application window."""
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("MPU9250 3D Cube Visualization")
        self.setGeometry(100, 100, 800, 600)
        self.opengl_widget = OpenGLWidget(self)
        self.setCentralWidget(self.opengl_widget)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())