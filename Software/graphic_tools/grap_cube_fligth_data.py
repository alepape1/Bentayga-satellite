import matplotlib.pyplot as plt
import datetime
import csv

import csv
import datetime

def read_sensor_data(file_name, num_lines=None):
    # Variables para almacenar los datos
    time = []
    roll = []
    pitch = []
    heading = []
    batt_temp_left = []
    batt_temp_right = []
    batt_temp_down = []
    batt_temp_box = []
    batt_temp_cubesat = []
    temperature = []
    pressure = []
    humidity = []
    gps_altitude = []
    latitude = []
    longitude = []
    speed = []
    num_satellites = []

    # Variable para el tiempo transcurrido en segundos
    time_elapsed_seconds = None
    sample_count = 0
    
    # Abrir el archivo CSV
    with open(file_name, "r") as file:
        # Crear un lector CSV
        csv_reader = csv.reader(file)
        # Leer la primera línea para verificar las columnas
        first_line = next(csv_reader)

        for _ in range(1):
            next(csv_reader)

        expected_columns = [
            "Time", "Roll", "Pitch", "Heading", "Batt Temp Left", "Batt Temp Right", "Batt Temp Down", "Batt Temp Box",
            "Batt Temp CubeSat", "Temperature", "Pressure", "Humidity", "GpsAltitude", "Latitude", "Longitude", "Speed",
            "NumSatellites", "Year", "Month", "Day", "Hour", "Minute", "Second", "Camera Info", "Intensity", "SNR"
        ]

        # Verificar si las columnas en el archivo coinciden con las esperadas
        if first_line != expected_columns:
            raise ValueError("Las columnas en el archivo CSV no coinciden con las esperadas.")

        # Leer cada línea del archivo
        for i, line in enumerate(file):
            # Verificar si se alcanzó el número máximo de líneas
            if num_lines is not None and i >= num_lines:
                break

            # Saltar líneas en blanco
            if line.strip() == "":
                continue

            # Dividir la línea en pares clave-valor
            data = line.strip().split(",")
            # Incrementar el contador de muestras
            sample_count += 1

            if i == 0:  # Procesar la primera línea para obtener la fecha y hora inicial
                year = int(data[17])
                month = int(data[18])
                day = int(data[19])
                hour = int(data[20])
                minute = int(data[21])
                second = int(data[22])
                time_start = datetime.datetime(year, month, day, hour, minute, second)
            
            # Calcular el tiempo transcurrido en segundos desde el inicio
            if i == num_lines-2:  # Procesar la ultima linea
                year = int(data[17])
                month = int(data[18])
                day = int(data[19])
                hour = int(data[20])
                minute = int(data[21])
                second = int(data[22])
                time_val = datetime.datetime(year, month, day, hour, minute, second)
                time_elapsed_seconds = (time_val - time_start).total_seconds()
                # Formato de la cadena de fecha y hora
                date_format = "%Y-%m-%d %H:%M:%S"
                print("The plot duration is from " + str(time_val) + " until " + str(time_val) + " and the duration is " + str(time_elapsed_seconds))

            # Extraer los valores de cada par clave-valor y convertirlos a float
            roll_val = float(data[1])
            pitch_val = float(data[2])
            heading_val = float(data[3])
            batt_temp_left_val = float(data[4])
            batt_temp_right_val = float(data[5])
            batt_temp_down_val = float(data[6])
            batt_temp_box_val = float(data[7])
            batt_temp_cubesat_val = float(data[8])
            temperature_val = float(data[9])
            pressure_val = float(data[10])
            humidity_val = float(data[11])
            gps_altitude_val = float(data[12])
            latitude_val = float(data[13])
            longitude_val = float(data[14])
            speed_val = float(data[15])
            num_satellites_val = float(data[0])

            # Agregar los valores a las listas correspondientes
            time.append(sample_count)
            roll.append(roll_val)
            pitch.append(pitch_val)
            heading.append(heading_val)
            batt_temp_left.append(batt_temp_left_val)
            batt_temp_right.append(batt_temp_right_val)
            batt_temp_down.append(batt_temp_down_val)
            batt_temp_box.append(batt_temp_box_val)
            batt_temp_cubesat.append(batt_temp_cubesat_val)
            temperature.append(temperature_val)
            pressure.append(pressure_val)
            humidity.append(humidity_val)
            gps_altitude.append(gps_altitude_val)
            latitude.append(latitude_val)
            longitude.append(longitude_val)
            speed.append(speed_val)
            num_satellites.append(num_satellites_val)

    return time, roll, pitch, heading, batt_temp_left, batt_temp_right, batt_temp_down, batt_temp_box, batt_temp_cubesat, temperature, pressure, humidity, gps_altitude, latitude, longitude, speed, num_satellites



def plot_sensor_data(time, roll, pitch, heading, batt_temp_left, batt_temp_right, batt_temp_down, batt_temp_box, batt_temp_cubesat, temperature, humidity, pressure, gps_altitude, latitude, longitude, speed, num_satellites):
    # Crear subplots para cada conjunto de datos
    fig, axs = plt.subplots(4, figsize=(12, 8))
    fig.suptitle('Sensor Data', fontsize=16)

    # Gráfico para Roll
    axs[0].plot(time, roll, label='Roll', color='b')
    axs[0].set_xlabel('Time')
    axs[0].set_ylabel('Roll', color='b')
    axs[0].tick_params(axis='y', labelcolor='b')

    # Establecer límites en el eje y para Roll
    axs[0].set_ylim(-180, 180)

    # Gráfico para Temperaturas de la batería
    axs[1].plot(time, batt_temp_left, label='Batt Temp Left', color='g')
    axs[1].plot(time, batt_temp_right, label='Batt Temp Right', color='r')
    axs[1].set_ylabel('Battery Temperatures', color='k')
    axs[1].tick_params(axis='y', labelcolor='k')

    # Establecer límites en el eje y para Temperaturas de la batería
    axs[1].set_ylim(0, 50)

    # Gráfico para Humedad
    axs[2].plot(time, humidity, label='Humidity', color='purple')
    axs[2].set_xlabel('Time')
    axs[2].set_ylabel('Humidity', color='purple')
    axs[2].tick_params(axis='y', labelcolor='purple')

    # Establecer límites en el eje y para Humedad
    axs[2].set_ylim(0, 100)

    # Gráfico para Presión y Altura GPS
    axs[3].plot(time, pressure, label='Pressure', color='orange')
    axs[3].plot(time, gps_altitude, label='GPS Altitude', color='brown')
    axs[3].set_xlabel('Time')
    axs[3].set_ylabel('Pressure, GPS Altitude', color='brown')
    axs[3].tick_params(axis='y', labelcolor='brown')

    # Establecer límites en el eje y para Presión y Altura GPS
    axs[3].set_ylim(0, 1500)  # Cambia estos valores según tus datos

    # Configurar leyendas para cada subplot
    for ax in axs:
        ax.legend(loc='upper left')

    # Ajustar los espacios entre subplots
    plt.tight_layout()

    # Mostrar los gráficos
    plt.show()

# Nombre del archivo
file_name = r"log\2023-09-07_00-16-04.csv"

# Líneas a leer (opcional, establecer en None para leer todo el archivo)
num_lines_to_read = 5000000

# Leer los datos del archivo
time, roll, pitch, heading, batt_temp_left, batt_temp_right, batt_temp_down, batt_temp_box, batt_temp_cubesat, temperature, humidity, pressure, gps_altitude, latitude, longitude, speed, num_satellites = read_sensor_data(file_name, num_lines_to_read)

# Graficar los datos
plot_sensor_data(time, roll, pitch, heading, batt_temp_left, batt_temp_right, batt_temp_down, batt_temp_box, batt_temp_cubesat, temperature, humidity, pressure, gps_altitude, latitude, longitude, speed, num_satellites)
