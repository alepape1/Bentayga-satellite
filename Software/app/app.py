import sys
import serial
from PyQt5 import QtWidgets
from PyQt5.QtGui import QVector3D
from PyQt5.QtWidgets import QGroupBox, QSplitter,QVBoxLayout, QHBoxLayout, QLabel, QWidget
from pyqtgraph.Qt import QtCore
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np
import csv
import os

MAX_DATA_POINTS = 3600
SERIAL_PORT = "COM25"

#Definir la clase SensorData
class SensorData:
    def __init__(self):

        self.time = []
        self.roll = []
        self.pitch = []
        self.heading = []
        self.batt_temp_left = []
        self.batt_temp_right = []
        self.batt_temp_down = []
        self.batt_temp_box = []
        self.batt_temp_cubesat = []
        self.temperature = []
        self.pressure = []
        self.humidity = []
        self.GpsAltitude = []
        self.latitude = []
        self.longitude = []
        self.speed = []
        self.numSatellites = []
        self.year = []
        self.month = []
        self.day = []
        self.hour = []
        self.minute =[]
        self.second = []
        self.camera_info = []
        self.camera_flag = []
        self.intensity = []
        self.snr = []

    def add_data_point(self, time, roll, pitch, heading, batt_temp_left, batt_temp_right, batt_temp_down, batt_temp_box,
                       batt_temp_cubesat, temperature, pressure, humidity, GpsAltitude, latitude, longitude, speed,
                       numSatellites, year, month, day, hour, minute, second, camera_info, camera_flag, intensity, snr):
        
        MAX_DATA_POINTS = 10000  # Define your maximum data points
        
        if len(self.time) >= MAX_DATA_POINTS:
            
            self.time.pop(0)
            self.roll.pop(0)
            self.pitch.pop(0)
            self.heading.pop(0)   
            self.batt_temp_left.pop(0)
            self.batt_temp_right.pop(0)
            self.batt_temp_down.pop(0)
            self.batt_temp_box.pop(0)
            self.batt_temp_cubesat.pop(0)
            self.temperature.pop(0)
            self.humidity.pop(0)
            self.pressure.pop(0)
            self.GpsAltitude.pop(0)
            self.latitude.pop(0)
            self.longitude.pop(0)
            self.speed.pop(0)
            self.numSatellites.pop(0)
            self.year.pop(0)
            self.month.pop(0)
            self.day.pop(0)
            self.hour.pop(0)
            self.minute.pop(0)
            self.second.pop(0)
            self.camera_info.pop(0)
            self.camera_flag.pop(0)
            self.intensity.pop(0)
            self.snr.pop(0)
        
        self.time.append(time)
        self.roll.append(roll)
        self.pitch.append(pitch)
        self.heading.append(heading)
        self.batt_temp_left.append(batt_temp_left)
        self.batt_temp_right.append(batt_temp_right)
        self.batt_temp_down.append(batt_temp_down)
        self.batt_temp_box.append(batt_temp_box)
        self.batt_temp_cubesat.append(batt_temp_cubesat)
        self.temperature.append(temperature)
        self.humidity.append(humidity)
        self.pressure.append(pressure)
        self.GpsAltitude.append(GpsAltitude)
        self.latitude.append(latitude)
        self.longitude.append(longitude)
        self.speed.append(speed)
        self.numSatellites.append(numSatellites)
        self.year.append(year)
        self.month.append(month)
        self.day.append(day)
        self.hour.append(hour)
        self.minute.append(minute)
        self.second.append(second)
        self.camera_info.append(camera_info)
        self.camera_flag.append(camera_flag)
        self.intensity.append(intensity)
        self.snr.append(snr)

    def print_sensor_data(self):
        attributes = dir(self)
        data_string = ""
        for attribute in attributes:
            if not attribute.startswith("__") and isinstance(getattr(self, attribute), list):
                values = getattr(self, attribute)
                if values:
                    data_string += f"{attribute}: {values[-1]}   "  # Separador entre atributos

        print(data_string)

#Definir la clase MainWindow para mostrar la app
class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # Configurar ventana principal
        self.setWindowTitle('Bentayga Launch App & 3D CubeSat 3U Visualizer')
        self.setGeometry(0, 0, 1080, 720)

        # Configurar ventana de gráficos 2D
        self.win = pg.GraphicsLayoutWidget(self)

        # Directorio donde se guardará el archivo CSV
        self.directory = "log"

        # Asegúrate de que la carpeta "log" exista o créala si no existe
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        # Crear el archivo CSV al iniciar la aplicación con el datetime
        current_datetime = QtCore.QDateTime.currentDateTime()
        formatted_datetime = current_datetime.toString("yyyy-MM-dd_hh-mm-ss")
      
        # Combina la ruta de la carpeta "log" con el nombre del archivo CSV
        filename = os.path.join(self.directory, f"{formatted_datetime}.csv")
        self.csv_file = open(filename, "w")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time", "Roll", "Pitch", "Heading", "Batt Temp Left", "Batt Temp Right",
                            "Batt Temp Down", "Batt Temp Box", "Batt Temp CubeSat", "Temperature",
                            "Pressure", "Humidity", "GpsAltitude", "Latitude", "Longitude", "Speed",
                            "NumSatellites", "Year", "Month", "Day", "Hour", "Minute", "Second",
                            "Camera Info","Camera Flag", "Intensity", "SNR"])  # Header


        # Configurar QSplitter
        splitter = QSplitter(QtCore.Qt.Vertical)

        # Configurar un layout para las gráficas en la tercera columna
        graph_layout = QVBoxLayout()
        splitter.addWidget(self.win)
        graph_layout.addWidget(self.win)

        # Establecer el peso de las filas y columnas en el QSplitter
        splitter.setSizes([int(self.height() * 0.3), int(self.height() * 0.3)])  # Ajusta la proporción según tu preferencia

        # Agregar las gráficas a la ventana
        self.setCentralWidget(splitter)

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
        p3 = self.win.addPlot(row=0, col=1, title="Temperatura externa")
        p3.setLabel('left', 'Temperatura (°C)')
        p3.setLabel('bottom', 'Tiempo (s)')
        self.curve_temperature = p3.plot(pen='b')
        
        # Altitud GPS
        p4 = self.win.addPlot(row=1, col=1, title="Altitud GPS")
        p4.setLabel('left', 'Altitud GPS (metros)')
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

        # Temperatura de baterias
        p7 = self.win.addPlot(row=0, col=2, title="Temperaturas Baterias")
        p7.setLabel('left', 'Temperatura (°C)')
        p7.setLabel('bottom', 'Tiempo (s)')
        p7.addLegend(offset=(1, 5))  # Agregar la leyenda a la derecha de la gráfica

        # Temperatura media baterias
        p8 = self.win.addPlot(row=1, col=2, title="Velocidad GPS")
        p8.setLabel('left', 'm/sg')
        p8.setLabel('bottom', 'Tiempo (s)')
        p8.addLegend(offset=(1, 5))  # Agregar la leyenda a la derecha de la gráfica

        # Camera Capturing
        p9 = self.win.addPlot(row=2, col=2, title="Camera capture")
        p9.setLabel('left', 'Capturing')
        p9.setLabel('bottom', 'Tiempo (s)')
        p9.addLegend(offset=(1, 5))  # Agregar la leyenda a la derecha de la gráfica

        # Crear las curvas de los graficos
        self.curve_batt_temp_left = p7.plot(pen='r', name='Batt Temp Left')
        self.curve_batt_temp_right = p7.plot(pen='g', name='Batt Temp Right')
        self.curve_batt_temp_down = p7.plot(pen='b', name='Batt Temp Down')
        self.curve_batt_temp_box = p7.plot(pen='m', name='Batt Temp Box')
        self.curve_batt_temp_cubesat = p7.plot(pen='c', name='Batt Temp CubeSat')
        self.curve_GPS_speed = p8.plot(pen='g', name='Velocidad GPS')
        self.curve_camera_capture = p9.plot(pen='r', name='Capturas camara hiperespectral')


        # Configurar temporizador
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # Actualizar cada 10 ms

        # Abrir conexión al puerto serie
        self.ser = serial.Serial(SERIAL_PORT, 115200)

        # Crear objeto para almacenar los datos del sensor
        self.sensor_data = SensorData()

        # Crear widget para las etiquetas de roll, pitch y heading
        label_widget = QWidget()
        label_layout = QHBoxLayout()
        label_widget.setLayout(label_layout)

        # Crear los grupos de layaout para Roll, Pitch, heading y posicion GPS
        roll_pitch_heading_groupbox = QGroupBox("Roll, Pitch, Heading y posicion GPS")
        roll_pitch_heading_layout = QHBoxLayout()
        roll_pitch_heading_groupbox.setLayout(roll_pitch_heading_layout)

        # Crear las etiquetas de roll, pitch, heading, latitud y longitud GPS
        self.label_roll = QLabel("Roll: ")
        self.label_pitch = QLabel("Pitch: ")
        self.label_heading = QLabel("Heading: ")
        self.label_latitude = QLabel("latitud GPS: ")
        self.label_longitude = QLabel("longitud GPS: ")


        # Agregar etiquetas al layout del grupo
        roll_pitch_heading_layout.addWidget(self.label_roll)
        roll_pitch_heading_layout.addWidget(self.label_pitch)
        roll_pitch_heading_layout.addWidget(self.label_heading)
        roll_pitch_heading_layout.addWidget(self.label_heading)
        roll_pitch_heading_layout.addWidget(self.label_latitude)
        roll_pitch_heading_layout.addWidget(self.label_longitude)

        # Camera Capturing y otras variables
        camera_groupbox = QGroupBox("Camera")
        camera_layout = QHBoxLayout()
        camera_groupbox.setLayout(camera_layout)
        
        self.label_camera_capture = QLabel("Camera Capture: ")
        # Agregar etiqueta al layout del grupo
        camera_layout.addWidget(self.label_camera_capture)

        # Agregar los grupos de etiquetas al diseño principal del layaout
        label_layout.addWidget(roll_pitch_heading_groupbox)
        label_layout.addWidget(camera_groupbox)


        # Agregar el widget al layout principal
        layout = QVBoxLayout()
        layout.addWidget(splitter)
        layout.addWidget(label_widget)

        # Crear un widget principal y establecer el layout
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def update(self):

        if self.ser.in_waiting > 0:
            # Leer una línea completa de datos del puerto serie
            line = self.ser.readline().decode().strip()
            print(line)
            # Separar los valores de la línea utilizando comas
            values = line.split(',')

            # Print the last values of sensor_data
            self.sensor_data.print_sensor_data()

            # Comprobar si la línea contiene el número correcto de valores

            if len(values) >= 26:

                 # Dentro del método update()
                self.sensor_data.add_data_point(len(self.sensor_data.time) + 1,
                                                    float(values[0]),
                                                    float(values[1]),
                                                    float(values[2]), 
                                                    float(values[3]), 
                                                    float(values[4]), 
                                                    float(values[5]), 
                                                    float(values[6]), 
                                                    float(values[7]), 
                                                    float(values[8]), 
                                                    float(values[9]),
                                                    float(values[10]), 
                                                    float(values[11]), 
                                                    float(values[12]), 
                                                    float(values[13]),
                                                    float(values[14]),     
                                                    int(values[15]),
                                                    int(values[16]), 
                                                    int(values[17]), 
                                                    int(values[18]), 
                                                    int(values[19]), 
                                                    int(values[20]),
                                                    int(values[21]),
                                                    int(values[22]),
                                                    float(values[23]),  
                                                    float(values[24]),   
                                                    float(values[25])  
                                                                        
                                                    )
                           
                # Obtener el último valor de cada atributo
                time = self.sensor_data.time[-1]
                roll = self.sensor_data.roll[-1]
                pitch = self.sensor_data.pitch[-1]
                heading = self.sensor_data.heading[-1]
                batt_temp_left = self.sensor_data.batt_temp_left[-1]
                batt_temp_right = self.sensor_data.batt_temp_right[-1]
                batt_temp_down = self.sensor_data.batt_temp_down[-1]
                batt_temp_box = self.sensor_data.batt_temp_box[-1]
                batt_temp_cubesat = self.sensor_data.batt_temp_cubesat[-1]
                temperature = self.sensor_data.temperature[-1]
                pressure = self.sensor_data.pressure[-1]
                humidity = self.sensor_data.humidity[-1]
                GpsAltitude = self.sensor_data.GpsAltitude[-1]
                latitude = self.sensor_data.latitude[-1]
                longitude = self.sensor_data.longitude[-1]
                speed = self.sensor_data.speed[-1]
                numSatellites = self.sensor_data.numSatellites[-1]
                year = self.sensor_data.year[-1]
                month = self.sensor_data.month[-1]
                day = self.sensor_data.day[-1]
                hour = self.sensor_data.hour[-1]
                minute = self.sensor_data.minute[-1]
                second = self.sensor_data.second[-1]
                camera_info = self.sensor_data.camera_info[-1]
                camera_flag = self.sensor_data.camera_flag[-1]
                intensity = self.sensor_data.intensity[-1]
                snr = self.sensor_data.snr[-1]

                # Escribe los valores en el archivo CSV
                self.csv_writer.writerow([time, roll, pitch, heading, batt_temp_left, batt_temp_right, batt_temp_down,
                                        batt_temp_box, batt_temp_cubesat, temperature, pressure, humidity,
                                        GpsAltitude, latitude, longitude, speed, numSatellites, year, month, day,
                                        hour, minute, second, camera_info, camera_flag, intensity, snr])
                self.csv_file.flush()  # Asegura que los datos se escriban en el archivo inmediatamente
                print("Data written to CSV file")


        # Actualizar las etiquetas de roll, pitch y heading
        self.label_roll.setText("Roll: " + str(self.sensor_data.roll[-1]))
        self.label_pitch.setText("Pitch: " + str(self.sensor_data.pitch[-1]))
        self.label_heading.setText("Heading: " + str(self.sensor_data.heading[-1]))
        self.label_latitude.setText("GPS latitude: " + str(self.sensor_data.latitude[-1]))
        self.label_longitude.setText("GPS longitude: " + str(self.sensor_data.longitude[-1]))   

        if self.sensor_data.camera_flag[-1] == 1:
            capture = 'YES'
        else:
            capture = 'NO'

        self.label_camera_capture.setText("Captura camara: " + capture)

        # Actualizar las gráficas
        self.curve_roll.setData(self.sensor_data.time, self.sensor_data.roll)
        self.curve_pitch.setData(self.sensor_data.time, self.sensor_data.pitch)
        self.curve_pressure.setData(self.sensor_data.time, self.sensor_data.pressure)
        self.curve_temperature.setData(self.sensor_data.time, self.sensor_data.temperature)
        self.curve_batt_temp_box.setData(self.sensor_data.time, self.sensor_data.batt_temp_box)
        self.curve_batt_temp_left.setData(self.sensor_data.time, self.sensor_data.batt_temp_left)
        self.curve_batt_temp_right.setData(self.sensor_data.time, self.sensor_data.batt_temp_right)
        self.curve_batt_temp_down.setData(self.sensor_data.time, self.sensor_data.batt_temp_down)
        self.curve_batt_temp_cubesat.setData(self.sensor_data.time, self.sensor_data.batt_temp_cubesat)
        self.curve_GpsAltitude.setData(self.sensor_data.time, self.sensor_data.GpsAltitude)
        self.curve_camera_capture.setData(self.sensor_data.time, self.sensor_data.camera_flag)
        self.curve_GPS_speed.setData(self.sensor_data.time, self.sensor_data.speed)
        self.curve_intensity.setData(self.sensor_data.time, self.sensor_data.intensity)
        self.curve_snr.setData(self.sensor_data.time, self.sensor_data.snr)
         
    def closeEvent(self, event):
        # Cerrar la conexión del puerto serie
        self.ser.close()
        self.csv_file.close()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
