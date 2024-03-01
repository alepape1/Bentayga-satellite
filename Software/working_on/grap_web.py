import serial
import pyqtgraph as pg
from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QWidget , QVBoxLayout, QPushButton
import sys

is_acquiring = False

def start_acquisition():
    global is_acquiring
    is_acquiring = True

def stop_acquisition():
    global is_acquiring
    is_acquiring = False


# Configuración de la aplicación
app = QApplication(sys.argv)
window = QWidget()
window.setWindowTitle('Datos del Arduino')

# Crear botones
start_button = QPushButton('Start')
stop_button = QPushButton('Stop')

# Conectar botones a funciones
start_button.clicked.connect(start_acquisition)
stop_button.clicked.connect(stop_acquisition)

# Añadir botones al layout principal
layout = QVBoxLayout()
layout.addWidget(start_button)
layout.addWidget(stop_button)

# Crear las gráficas
view = pg.GraphicsView()
layout.addWidget(view)

layout.setStretch(1, 1)  # Stretch the graphics area

# Set the layout for the main window
window.setLayout(layout)

# Create a GraphicsLayoutWidget
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Datos del Arduino')
pg.setConfigOptions(antialias=True)

# Crear las gráficas en el GraphicsLayoutWidget
p1 = win.addPlot(title="Ángulo")
p1.setYRange(-180, 180)
p1.setLabel('left', 'Ángulo (°)')
p1.setLabel('bottom', 'Tiempo (s)')
p1.addLegend()
curve_roll = p1.plot(pen='r', name='Roll')
curve_pitch = p1.plot(pen='g', name='Pitch')
curve_heading = p1.plot(pen='b', name='Heading')

p2 = win.addPlot(title="Presión")
p2.setLabel('left', 'Presión (hPa)')
p2.setLabel('bottom', 'Tiempo (s)')
curve_pressure = p2.plot(pen='r')

p3 = win.addPlot(title="Temperatura")
p3.setLabel('left', 'Temperatura (°C)')
p3.setLabel('bottom', 'Tiempo (s)')
curve_temperature = p3.plot(pen='b')

# Abrir conexión al puerto serie
ser = serial.Serial('COM9', 115200)

# Crear listas para almacenar los valores de los datos
time_data = []
roll_data = []
pitch_data = []
heading_data = []
pressure_data = []
temperature_data = []

# Leer datos del puerto serie y graficarlos en tiempo real
def update():
    global time_data, roll_data, pitch_data, heading_data, pressure_data, temperature_data
    if is_acquiring and ser.in_waiting > 0:
        # Leer una línea completa de datos del puerto serie
        line = ser.readline().decode().strip()
        # Separar los valores de la línea utilizando comas
        values = line.split(',')
        # Comprobar si la línea contiene el número correcto de valores
        if len(values) == 10:
            # Imprimir la línea de datos en la consola
            print(line)
            # Convertir los valores a números y agregarlos a las listas de datos
            time_data.append(len(time_data) + 1)
            roll_data.append(float(values[0]))
            pitch_data.append(float(values[1]))
            heading_data.append(float(values[2]))
            pressure_data.append(float(values[4]))
            temperature_data.append(float(values[3]))
            # Actualizar las gráficas con los nuevos valores
            curve_roll.setData(time_data, roll_data)
            curve_pitch.setData(time_data, pitch_data)
            curve_heading.setData(time_data, heading_data)
            curve_pressure.setData(time_data, pressure_data)
            curve_temperature.setData(time_data, temperature_data)
            app.processEvents()

# Configuración del temporizador para actualizar los gráficos
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)  # Actualizar cada 10 ms

# Mostrar la ventana principal de la aplicación
win.show()
app.exec_()
