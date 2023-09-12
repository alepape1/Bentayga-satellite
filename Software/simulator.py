import sys
from PyQt5 import QtWidgets
from PyQt5.QtGui import QVector3D
from PyQt5.QtWidgets import QSplitter, QVBoxLayout, QLabel, QWidget
from pyqtgraph.Qt import QtCore
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np
import random

MAX_DATA_POINTS = 3600
simulation = True

class SensorData:

    def __init__(self):
        self.time = []
        self.roll = []
        self.pitch = []
        self.heading = []
        self.temperature = []
        self.humidity = []
        self.pressure = []
        self.GpsAltitude = []
        self.latitude = []
        self.longitude = []
        self.speed = []
        self.numSatellites = []
        self.intensity = []
        self.snr = []

    def add_data_point(self, time, roll, pitch, heading, temperature, humidity, pressure, GpsAltitude, latitude, longitude, speed, numSatellites, intensity, snr):
        
        if len(self.time) >= MAX_DATA_POINTS:
            self.time.pop(0)
            self.roll.pop(0)
            self.pitch.pop(0)
            self.heading.pop(0)   
            self.temperature.pop(0)
            self.humidity.pop(0)
            self.pressure.pop(0)
            self.GpsAltitude.pop(0)
            self.latitude.pop(0)
            self.longitude.pop(0)
            self.speed.pop(0)
            self.numSatellites.pop(0)
            self.intensity.pop(0)
            self.snr.pop(0)
        
        self.time.append(time)
        self.roll.append(roll)
        self.pitch.append(pitch)
        self.heading.append(heading)
        self.temperature.append(temperature)
        self.humidity.append(humidity)
        self.pressure.append(pressure)
        self.GpsAltitude.append(GpsAltitude)
        self.latitude.append(latitude)
        self.longitude.append(longitude)
        self.speed.append(speed)
        self.numSatellites.append(numSatellites)
        self.intensity.append(intensity)
        self.snr.append(snr)


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        
        super(MainWindow, self).__init__(parent)

        # Configurar ventana principal
        self.setWindowTitle('Bentayga Launch App & 3D CubeSat 3U Visualizer')
        self.setGeometry(0, 0, 800, 600)

        # Configurar ventana de gráficos 2D
        self.win = pg.GraphicsLayoutWidget(self)

        # Configurar ventana de gráficos 3D
        self.gl_widget = gl.GLViewWidget(self)

        # Configurar QSplitter
        splitter = QSplitter(QtCore.Qt.Vertical)
        splitter.addWidget(self.gl_widget)
        splitter.addWidget(self.win)
        self.setCentralWidget(splitter)

        # Configurar CubeSat 3U
        self.cubeSat = gl.GLBoxItem(size=QVector3D(1, 1, 3), color=(255, 255, 0, 255))  # Dimensiones 1x1x3 para representar un CubeSat 3U
        self.gl_widget.addItem(self.cubeSat)

        # Agregar el objeto de líneas al widget de gráficos 3D
        self.gl_widget.addItem(self.cubeSat)

        # Configurar las gráficas
        p1 = self.win.addPlot(row=0, col=0, title="Ángulo")
        p1.setYRange(-180, 180)
        p1.setLabel('left', 'Ángulo (°)')
        p1.setLabel('bottom', 'Tiempo (s)')
        p1.addLegend()
        self.curve_roll = p1.plot(pen='r', name='Roll')
        self.curve_pitch = p1.plot(pen='g', name='Pitch')

        # Presion
        p2 = self.win.addPlot(row=1, col=0, title="Presión")
        p2.setLabel('left', 'Presión (hPa)')
        p2.setLabel('bottom', 'Tiempo (s)')
        self.curve_pressure = p2.plot(pen='r')

        # Temperatura externa
        p3 = self.win.addPlot(row=0, col=1, title="Temperatura")
        p3.setLabel('left', 'Temperatura (°C)')
        p3.setLabel('bottom', 'Tiempo (s)')
        self.curve_temperature = p3.plot(pen='b')
        
        # Altitud GPS
        p4 = self.win.addPlot(row=1, col=1, title="Altitud GPS")
        p4.setLabel('left', 'Altitud GPS (meters)')
        p4.setLabel('bottom', 'Tiempo (s)')
        self.curve_GpsAltitude = p4.plot(pen='m')

        # Intesidad de la señal Lora
        p5 = self.win.addPlot(row=2, col=0, title="Nivel de Intensidad de Señal")
        p5.setLabel('left', 'Nivel de Intensidad (dBm)')
        p5.setLabel('bottom', 'Tiempo (s)')
        self.curve_intensity = p5.plot(pen='b')

        # Relacion señal a ruido (SNR)
        p6 = self.win.addPlot(row=2, col=1, title="Relación Señal-Ruido (SNR)")
        p6.setLabel('left', 'SNR (dB)')
        p6.setLabel('bottom', 'Tiempo (s)')
        self.curve_snr = p6.plot(pen='g')

        # Configurar temporizador
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # Actualizar cada 10 ms

        # Abrir conexión al puerto serie
        
        if simulation is not True:
          self.ser = serial.Serial('COM14', 115200)

        # Crear objeto para almacenar los datos del sensor
        self.sensor_data = SensorData()

        # Crear widget para las etiquetas de roll, pitch y heading
        label_widget = QWidget()
        label_layout = QVBoxLayout()
        label_widget.setLayout(label_layout)

        # Crear las etiquetas de roll, pitch y heading
        self.label_roll = QLabel("Roll: ")
        self.label_pitch = QLabel("Pitch: ")
        self.label_heading = QLabel("Heading: ")

        # Agregar las etiquetas al layout
        label_layout.addWidget(self.label_roll)
        label_layout.addWidget(self.label_pitch)
        label_layout.addWidget(self.label_heading)

        # Agregar el widget al layout principal
        layout = QVBoxLayout()
        layout.addWidget(splitter)
        layout.addWidget(label_widget)

        # Crear un widget principal y establecer el layout
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def update(self):
        # Generar datos aleatorios simulados para las gráficas
        simulated_time = len(self.sensor_data.time) + 1
        simulated_roll = random.uniform(-180, 180)
        simulated_pitch = random.uniform(-180, 180)
        simulated_heading = random.uniform(0, 360)
        simulated_temperature = random.uniform(-40, 40)
        simulated_pressure = random.uniform(900, 1100)
        simulated_GpsAltitude = random.uniform(0, 5000)
        simulated_intensity = random.uniform(-100, -30)
        simulated_snr = random.uniform(0, 20)

        # Agregar datos simulados a la instancia de SensorData
        self.sensor_data.add_data_point(simulated_time, simulated_roll, simulated_pitch,
                                    simulated_heading, simulated_temperature,
                                    simulated_pressure, simulated_GpsAltitude,
                                    0, 0, 0, 0, 0, simulated_intensity, simulated_snr)

        # Verificar que hay datos en las listas antes de intentar acceder a ellos
        if len(self.sensor_data.pitch) > 0:

        # Actualizar la rotación del CubeSat
        self.cubeSat.resetTransform()
        self.cubeSat.rotate(self.sensor_data.pitch[-1], 0, 1, 0)  # Rotate around y-axis
        self.cubeSat.rotate(self.sensor_data.heading[-1], 0, 0, 1)  # Rotate around z-axis
        self.cubeSat.rotate(self.sensor_data.roll[-1], 1, 0, 0)  # Rotate around x-axis

        # Actualizar las etiquetas de roll, pitch y heading
        self.label_roll.setText("Roll: " + str(self.sensor_data.roll[-1]))
        self.label_pitch.setText("Pitch: " + str(self.sensor_data.pitch[-1]))
        self.label_heading.setText("Heading: " + str(self.sensor_data.heading[-1]))

        # Actualizar las gráficas
        self.curve_roll.setData(self.sensor_data.time, self.sensor_data.roll)
        self.curve_pitch.setData(self.sensor_data.time, self.sensor_data.pitch)
        self.curve_pressure.setData(self.sensor_data.time, self.sensor_data.pressure)
        self.curve_temperature.setData(self.sensor_data.time, self.sensor_data.temperature)
        self.curve_GpsAltitude.setData(self.sensor_data.time, self.sensor_data.GpsAltitude)
        self.curve_intensity.setData(self.sensor_data.time, self.sensor_data.intensity)
        self.curve_snr.setData(self.sensor_data.time, self.sensor_data.snr)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
