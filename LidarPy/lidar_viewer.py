import zmq
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPainter, QColor
from PyQt5.QtCore import Qt, QTimer
import configparser
import numpy as np
import struct

class LidarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []
        self.setMinimumSize(800, 800)
        self.scale = 100  # 1 meter = 100 pixels
        self.max_distance = 3.0  # 3 meters max for colormap

    def update_points(self, points):
        self.points = points
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Dark background
        painter.fillRect(self.rect(), QColor(30, 30, 30))

        # Translate to center
        painter.translate(self.width() / 2, self.height() / 2)

        # Draw grid
        painter.setPen(QColor(100, 100, 100))
        for i in range(-3, 4):
            painter.drawLine(int(i * self.scale), int(-self.height() / 2), int(i * self.scale), int(self.height() / 2))
            painter.drawLine(int(-self.width() / 2), int(i * self.scale), int(self.width() / 2), int(i * self.scale))

        # Draw points with heatmap
        for x, y in self.points:
            distance = np.sqrt(x**2 + y**2)
            # Blue (closer) to red (farther)
            blue = int(255 * (1 - min(distance / self.max_distance, 1)))
            red = int(255 * min(distance / self.max_distance, 1))
            painter.setPen(QColor(red, 0, blue))
            painter.setBrush(QColor(red, 0, blue))
            painter.drawEllipse(int(x * self.scale), int(-y * self.scale), 5, 5)

class LidarViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR Point Cloud Viewer")
        
        # Load configuration
        config = configparser.ConfigParser()
        config.read('config.ini')
        self.ip = config.get('DEFAULT', 'zmq_ip', fallback='192.168.1.10')
        self.port = config.getint('DEFAULT', 'zmq_port', fallback=5000)

        # Set up ZeroMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{self.ip}:{self.port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.RCVHWM, 1)  # Keep only latest message
        self.socket.setsockopt(zmq.RCVBUF, 131072)  # 128KB receive buffer
        self.socket.setsockopt(zmq.CONFLATE, 1)  # Drop old messages
        self.socket.setsockopt(zmq.IMMEDIATE, 1)  # Don't buffer if not ready

        # Set up UI
        self.central_widget = LidarWidget()
        self.setCentralWidget(self.central_widget)
        
        # Dark theme
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e1e; }
            QWidget { background-color: #1e1e1e; color: #ffffff; }
        """)

        # Start receiving data
        self.receive_data()

    def receive_data(self):
        try:
            # Receive binary message
            start_time = time.time()
            binary_data = self.socket.recv(zmq.DONTWAIT)
            # Unpack point count (first 4 bytes)
            if len(binary_data) < 4:
                return
            point_count = struct.unpack('I', binary_data[:4])[0]
            # Unpack points (8 bytes each: 2 floats)
            points = []
            if len(binary_data) >= 4 + point_count * 8:
                points_array = np.frombuffer(binary_data[4:], dtype=np.float32)
                points = [(points_array[i], points_array[i+1]) for i in range(0, len(points_array), 2)]
            self.central_widget.update_points(points)
            # Log timing
            duration = (time.time() - start_time) * 1000
            print(f"Receive and process time: {duration:.2f} ms")
        except zmq.Again:
            pass
        except struct.error as e:
            print(f"Error unpacking data: {e}")
        QTimer.singleShot(10, self.receive_data)  # Reduced to 10ms for faster polling

if __name__ == '__main__':
    import time
    app = QApplication(sys.argv)
    viewer = LidarViewer()
    viewer.show()
    sys.exit(app.exec_())