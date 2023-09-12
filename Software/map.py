import serial
import pyqtgraph as pg
from PyQt5 import QtGui, QtCore, QtWebEngineWidgets
import folium


# Configuración de la aplicación
app = QtGui.QGuiApplication([])
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Bentayga Launch App')
pg.setConfigOptions(antialias=True)

# Crear las gráficas
# Tus gráficas aquí...

# Agregar un widget de mapa a la aplicación
m = folium.Map(location=[45.5236, -122.6750])  # Usa la ubicación que prefieras
map_widget = QtWebEngineWidgets.QWebEngineView()
map_widget.setHtml(m._repr_html_())

# Abrir conexión al puerto serie
ser = serial.Serial('COM9', 115200)

# Crear listas para almacenar los valores de los datos
# Tus listas de datos aquí...

# Agregar listas para almacenar latitud y longitud
latitude_data = []
longitude_data = []

# Leer datos del puerto serie y graficarlos en tiempo real
def update():
    global time_data, roll_data, pitch_data, pressure_data, temperature_data, altitude_data, heading_data, latitude_data, longitude_data
    if ser.in_waiting > 0:
        # Leer una línea completa de datos del puerto serie
        line = ser.readline().decode().strip()
        # Separar los valores de la línea utilizando comas
        values = line.split(',')
        # Comprobar si la línea contiene el número correcto de valores
        if len(values) == 12:  # Asegúrate de que el número correcto de valores esté siendo leído
            # Convertir los valores a números y agregarlos a las listas de datos
            # Tus datos aquí...

            # Agregar latitud y longitud a sus respectivas listas
            latitude_data.append(float(values[-4]))
            longitude_data.append(float(values[-3]))

            # Actualizar las gráficas con los nuevos valores
            # Tus actualizaciones de gráficas aquí...

            # Actualizar el mapa con la nueva ubicación
            folium.Marker(location=[latitude_data[-1], longitude_data[-1]], popup='Posición actual').add_to(m)
            map_widget.setHtml(m._repr_html_())

            QtGui.QGuiApplication.processEvents()

# Configuración del temporizador para actualizar los gráficos
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)  # Actualizar cada 10 ms

# Mostrar la ventana principal de la aplicación
win.show()
map_widget.show()  # Muestra el widget de mapa
app.exec_()
