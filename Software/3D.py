import sys
import random
from PyQt5 import QtWidgets
from PyQt5.QtGui import QVector3D
from pyqtgraph.Qt import QtCore
import pyqtgraph.opengl as gl

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # Configurar ventana principal
        self.setWindowTitle('3D CubeSat 3U Visualizer')
        self.setGeometry(0, 0, 800, 600)

        # Configurar ventana de gráficos 3D
        self.gl_widget = gl.GLViewWidget(self)
        self.setCentralWidget(self.gl_widget)

      
        # Configurar CubeSat 3U
        self.cubeSat = gl.GLBoxItem(size=QVector3D(1,1,3))  # Dimensiones 1x1x3 para representar un CubeSat 3U
        self.gl_widget.addItem(self.cubeSat)
        
       
        # Configurar temporizador
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(200)  # Actualizar cada 50 ms

    def update(self):
        # Generar datos aleatorios para roll, pitch y heading
        roll, pitch, heading = random.randint(0, 1), random.randint(0, 1), random.randint(0, 360)
        
        # Actualizar la rotación del CubeSat
        self.cubeSat.rotate(roll, 1, 0, 0)  # Rotate around x-axis
        self.cubeSat.rotate(pitch, 0, 1, 0)  # Rotate around y-axis
        self.cubeSat.rotate(heading, 0, 0, 1)  # Rotate around z-axis
   

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
