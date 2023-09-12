import sys
import random
import serial
from PyQt5 import QtWidgets
from PyQt5.QtGui import QVector3D
from PyQt5.QtWidgets import QSplitter
from pyqtgraph.Qt import QtCore
import pyqtgraph.opengl as gl
import pyqtgraph as pg


# Crear listas para almacenar los valores de los datos
# Formato del frame recivido : roll,pitch,heading,temperature,humidity,pressure,GpsAltitude,latitude,longitude,speed,numSatellites
time_data = []
roll_data = []
pitch_data = []
heading_data = []
temperature_data = []
humidity_data = []
pressure_data = []
GpsAltitude_data = []
latitude_data = []
longitude_data = []
speed_data = []
numSatellites_data = []
intensity_data = []
snr_data = []

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
        splitter = QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self.win)
        splitter.addWidget(self.gl_widget)
        self.setCentralWidget(splitter)

        # Configurar CubeSat 3U
        self.cubeSat = gl.GLBoxItem(size=QVector3D(1,1,3))  # Dimensiones 1x1x3 para representar un CubeSat 3U
        self.gl_widget.addItem(self.cubeSat)

        # Configurar las gráficas
        p1 = self.win.addPlot(row=0, col=0, title="Ángulo")
        p1.setYRange(-180, 180)
        p1.setLabel('left', 'Ángulo (°)')
        p1.setLabel('bottom', 'Tiempo (s)')
        p1.addLegend()
        self.curve_roll = p1.plot(pen='r', name='Roll')
        self.curve_pitch = p1.plot(pen='g', name='Pitch')

        # ... Resto de tu configuración de gráficos 2D ...
        p2 = self.win.addPlot(row=1, col=0, title="Presión")
        p2.setLabel('left', 'Presión (hPa)')
        p2.setLabel('bottom', 'Tiempo (s)')
        self.curve_pressure = p2.plot(pen='r')

        p3 = self.win.addPlot(row=0, col=1, title="Temperatura")
        p3.setLabel('left', 'Temperatura (°C)')
        p3.setLabel('bottom', 'Tiempo (s)')
        self.curve_temperature = p3.plot(pen='b')

        p4 = self.win.addPlot(row=1, col=1, title="Altitud Corregida")
        p4.setLabel('left', 'Altitud Barómetrica (meters)')
        p4.setLabel('bottom', 'Tiempo (s)')
        self.curve_GpsAltitude = p4.plot(pen='m')

        p5 = self.win.addPlot(row=2, col=0, title="Nivel de Intensidad de Señal")
        p5.setLabel('left', 'Nivel de Intensidad (dBm)')
        p5.setLabel('bottom', 'Tiempo (s)')
        self.curve_intensity = p5.plot(pen='b')

        p6 = self.win.addPlot(row=2, col=1, title="Relación Señal-Ruido (SNR)")
        p6.setLabel('left', 'SNR (dB)')
        p6.setLabel('bottom', 'Tiempo (s)')
        self.curve_snr = p6.plot(pen='g')

        # Configurar temporizador
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(200)  # Actualizar cada 50 ms

        # Abrir conexión al puerto serie
        self.ser = serial.Serial('COM14', 115200)

    def update(self):

        global time_data, roll_data, pitch_data,  heading_data, temperature_data, humidity_data, pressure_data,  GpsAltitude_data, intensity_data, snr_data 
        
        if self.ser.in_waiting > 0:

            # Leer una línea completa de datos del puerto serie
            line = self.ser.readline().decode().strip()
            # Separar los valores de la línea utilizando comas
            values = line.split(',')
            # Comprobar si la línea contiene el número correcto de valores
            if len(values) == 13:
                # Imprimir la línea de datos en la consola
                print(line)
                # Convertir los valores a números y agregarlos a las listas de datos
                time_data.append(len(time_data) + 1)
                roll_data.append(float(values[0]))
                pitch_data.append(float(values[1]))
                heading_data.append(float(values[2]))
                temperature_data.append(float(values[3]))
                humidity_data.append(float(values[4]))
                pressure_data.append(float(values[5]))
                GpsAltitude_data.append(float(values[6]))
                latitude_data.append(float(values[7]))
                longitude_data.append(float(values[8]))
                speed_data.append(float(values[9]))
                numSatellites_data.append(int(values[10]))
                intensity_data.append(float(values[11]))
                snr_data.append(float(values[12]))
                


        # Actualizar la rotación del CubeSat
        self.cubeSat.rotate(roll_data[-1], 1, 0, 0)  # Rotate around x-axis
        self.cubeSat.rotate(pitch_data[-1], 0, 1, 0)  # Rotate around y-axis
        self.cubeSat.rotate(heading_data[-1], 0, 0, 1)  # Rotate around z-axis

        # actualización de gráficas 2D  
        self.curve_roll.setData(time_data, roll_data)
        self.curve_pitch.setData(time_data, pitch_data)
        self.curve_pressure.setData(time_data, pressure_data)
        self.curve_temperature.setData(time_data, temperature_data)
        self.curve_GpsAltitude.setData(time_data, GpsAltitude_data)
        self.curve_intensity.setData(time_data, intensity_data)
        self.curve_snr.setData(time_data, snr_data)
        # self.curve_heading.setData(time_data, heading_data)
        
        # Configuración del temporizador para actualizar los gráficos
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(10)  # Actualizar cada 10 ms  

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    
    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
