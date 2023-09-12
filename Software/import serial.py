import serial
import pyqtgraph as pg
from PyQt5 import QtGui, QtCore



# Configuración de la aplicación
app = QtGui.QGuiApplication([])
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Bentayga Launch App')
pg.setConfigOptions(antialias=True)

# Crear las gráficas
p1 = win.addPlot(row=0, col=0, title="Ángulo")
p1.setYRange(-180, 180)
p1.setLabel('left', 'Ángulo (°)')
p1.setLabel('bottom', 'Tiempo (s)')
p1.addLegend()
curve_roll = p1.plot(pen='r', name='Roll')
curve_pitch = p1.plot(pen='g', name='Pitch')

p2 = win.addPlot(row=1, col=0, title="Presión")
p2.setLabel('left', 'Presión (hPa)')
p2.setLabel('bottom', 'Tiempo (s)')
curve_pressure = p2.plot(pen='r')

p3 = win.addPlot(row=0, col=1, title="Temperatura")
p3.setLabel('left', 'Temperatura (°C)')
p3.setLabel('bottom', 'Tiempo (s)')
curve_temperature = p3.plot(pen='b')

p4 = win.addPlot(row=1, col=1, title="Altitud Corregida")
p4.setLabel('left', 'Altitud Barómetrica (meters)')
p4.setLabel('bottom', 'Tiempo (s)')
curve_altitude = p4.plot(pen='m')

# Abrir conexión al puerto serie
ser = serial.Serial('COM9', 115200)

# Crear listas para almacenar los valores de los datos
time_data = []
roll_data = []
pitch_data = []
pressure_data = []
temperature_data = []
altitude_data = []

# Leer datos del puerto serie y graficarlos en tiempo real
def update():
    global time_data, roll_data, pitch_data, pressure_data, temperature_data, altitude_data
    if ser.in_waiting > 0:
        # Leer una línea completa de datos del puerto serie
        line = ser.readline().decode().strip()
        # Separar los valores de la línea utilizando comas
        values = line.split(',')
        # Comprobar si la línea contiene el número correcto de valores
        if len(values) == 10:
            # Convertir los valores a números y agregarlos a las listas de datos
            time_data.append(len(time_data) + 1)
            roll_data.append(float(values[0]))
            pitch_data.append(float(values[1]))
            pressure_data.append(float(values[4]))
            temperature_data.append(float(values[3]))
            altitude_data.append(float(values[5]))
            # Actualizar las gráficas con los nuevos valores
            curve_roll.setData(time_data, roll_data)
            curve_pitch.setData(time_data, pitch_data)
            curve_pressure.setData(time_data, pressure_data)
            curve_temperature.setData(time_data, temperature_data)
            curve_altitude.setData(time_data, altitude_data)
            QtGui.QGuiApplication.processEvents()

# Configuración del temporizador para actualizar los gráficos
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)  # Actualizar cada 10 ms

# Mostrar la ventana principal de la aplicación
win.show()
app.exec_()
