import matplotlib.pyplot as plt

def read_sensor_data(file_name, num_lines=None):
    # Variables para almacenar los datos
    roll = []
    pitch = []
    heading = []
    batt_temp = []
    temperature = []
    humidity = []
    pressure = []
    gps_altitude = []
    latitude = []
    longitude = []
    speed = []
    num_satellites = []

    # Abrir el archivo
    with open(file_name, "r") as file:
        # Leer cada línea del archivo
        for i, line in enumerate(file):
            # Verificar si se alcanzó el número máximo de líneas
            if num_lines is not None and i >= num_lines:
                break

            # Dividir la línea en pares clave-valor
            data = line.strip().split(",")

            # Extraer los valores de cada par clave-valor
            roll_val = float(data[0].split(":")[1])
            pitch_val = float(data[1].split(":")[1])
            heading_val = float(data[2].split(":")[1])
            batt_temp_val = float(data[3].split(":")[1])
            temperature_val = float(data[4].split(":")[1])
            humidity_val = float(data[5].split(":")[1])
            pressure_val = float(data[6].split(":")[1])
            gps_altitude_val = float(data[7].split(":")[1])
            latitude_val = float(data[8].split(":")[1])
            longitude_val = float(data[9].split(":")[1])
            speed_val = float(data[10].split(":")[1])
            num_satellites_val = float(data[11].split(":")[1])

            # Agregar los valores a las listas correspondientes
            roll.append(roll_val)
            pitch.append(pitch_val)
            heading.append(heading_val)
            batt_temp.append(batt_temp_val)
            temperature.append(temperature_val)
            humidity.append(humidity_val)
            pressure.append(pressure_val)
            gps_altitude.append(gps_altitude_val)
            latitude.append(latitude_val)
            longitude.append(longitude_val)
            speed.append(speed_val)
            num_satellites.append(num_satellites_val)

    return roll, pitch, heading, batt_temp, temperature, humidity, pressure, gps_altitude, latitude, longitude, speed, num_satellites

def plot_sensor_data(roll, pitch, heading, batt_temp, temperature, humidity, pressure, gps_altitude, latitude, longitude, speed, num_satellites):
    # Crear los gráficos en la misma ventana
    fig, axes = plt.subplots(3, 3, figsize=(12, 8))
    fig.suptitle('Bentayga Sensor Data Freeze test', fontsize=16)

    # Gráfico para Roll
    axes[0, 0].plot(roll)
    axes[0, 0].set_title('Roll')

    # Gráfico para Pitch
    axes[0, 1].plot(pitch)
    axes[0, 1].set_title('Pitch')

    # Gráfico para Heading
    axes[0, 2].plot(heading)
    axes[0, 2].set_title('Heading')

    # Gráfico para Batt. Temp
    axes[1, 0].plot(batt_temp)
    axes[1, 0].set_title('External Temp')

    # Gráfico para Temperature
    axes[1, 1].plot(temperature)
    axes[1, 1].set_title('Battery Temperature')

    # Gráfico para Humidity
    axes[1, 2].plot(humidity)
    axes[1, 2].set_title('Humidity')

    # Gráfico para Pressure
    axes[2, 0].plot(pressure)
    axes[2, 0].set_title('Pressure')

    # Gráfico para GpsAltitude
    axes[2, 1].plot(gps_altitude)
    axes[2, 1].set_title('GpsAltitude')


    # Gráfico para Speed
    axes[2, 2].plot(speed)
    axes[2, 2].set_title('Speed')


    # Ajustar los espacios entre los subplots
    plt.tight_layout()

    # Mostrar los gráficos
    plt.show()

# Nombre del archivo
file_name = "CONGE.TXT"

# Líneas a leer (opcional, establecer en None para leer todo el archivo)
num_lines_to_read = None

# Leer los datos del archivo
roll, pitch, heading, batt_temp, temperature, humidity, pressure, gps_altitude, latitude, longitude, speed, num_satellites = read_sensor_data(file_name, num_lines_to_read)

# Graficar los datos
plot_sensor_data(roll, pitch, heading, batt_temp, temperature, humidity, pressure, gps_altitude, latitude, longitude, speed, num_satellites)
