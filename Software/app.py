import sys
import random
import numpy as np
import pyqtgraph as pg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Ventana de aplicación con gráfico y botones")
        self.setGeometry(100, 100, 800, 600)

        # Layout principal de la ventana
        layout = QVBoxLayout()
        
        # Agregar gráfico al layout
        graph_container = QWidget()
        graph_layout = QVBoxLayout(graph_container)
  
        # Botones
        button1 = QPushButton("Botón 1")
        button2 = QPushButton("Botón 2")
        button3 = QPushButton("Botón 3")
        button4 = QPushButton("Botón 4")

        # Conexión de los botones a sus respectivas funciones
        button1.clicked.connect(self.plot_random_data)
        button2.clicked.connect(self.clear_plot)
        button3.clicked.connect(self.close)
        button4.clicked.connect(self.print_hello)

      
         # Gráfico
        graph = pg.GraphicsLayoutWidget()


        # # Crear las gráficas
        # p1 = self.win.addPlot(title="Ángulo")
        # p1.setYRange(-180, 180)
        # p1.setLabel('left', 'Ángulo (°)')
        # p1.setLabel('bottom', 'Tiempo (s)')
        # p1.addLegend()
        # curve_roll = p1.plot(pen='r', name='Roll')
        # curve_pitch = p1.plot(pen='g', name='Pitch')

        layout.addWidget(graph)
        layout.addWidget(graph_container)
        layout.addWidget(self.win)

        # p2 = self.win.addPlot(title="Presión")
        # p2.setLabel('left', 'Presión (hPa)')
        # p2.setLabel('bottom', 'Tiempo (s)')
        # curve_pressure = p2.plot(pen='r')

        # p3 = self.win.addPlot(title="Temperatura")
        # p3.setLabel('left', 'Temperatura (°C)')
        # p3.setLabel('bottom', 'Tiempo (s)')
        # curve_temperature = p3.plot(pen='b')

        # p4 = self.win.addPlot(title="Altitud Corregida")
        # p4.setLabel('left', 'Altitud Corregida (meters)')
        # p4.setLabel('bottom', 'Tiempo (s)')
        # curve_altitude = p4.plot(pen='m')

        # p5 = self.win.addPlot(title="Heading")
        # p5.setYRange(0, 360)
        # p5.setLabel('left', 'Heading (°)')
        # p5.setLabel('bottom', 'Tiempo (s)')
        # curve_heading = p5.plot(pen='b')

        # Layout
        

        # Agregar botones al layout
        layout.addWidget(button1)
        layout.addWidget(button2)
        layout.addWidget(button3)
        layout.addWidget(button4)

        # Widget principal
        main_widget = QWidget()
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

    def plot_random_data(self):
        # Generar datos aleatorios
        data = np.random.rand(100)

        # Limpiar gráfico anterior
        self.figure.clear()

        # Agregar nuevos datos al gráfico
        ax = self.figure.add_subplot(111)
        ax.plot(data, "-")

        # Actualizar el gráfico en el lienzo
        self.canvas.draw()

    def clear_plot(self):
        # Limpiar gráfico
        self.figure.clear()
        self.canvas.draw()

    def print_hello(self):
        print("¡Hola!")

# Configuración de la aplicación
app = QApplication(sys.argv)
window = MainWindow()
window.show()

# Ejecutar la aplicación
sys.exit(app.exec_())
