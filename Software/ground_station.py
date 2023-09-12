import serial
import matplotlib.pyplot as plt
import time

# Función para medir la tasa de bits en kbps
def calculate_bitrate(bytes_sent, start_time, end_time):
    # Convertir tamaño de bytes a kilobits
    kbits_sent = bytes_sent * 8 / 1000
    
    # Calcular la duración de la transmisión en segundos
    duration = end_time - start_time
    
    # Calcular la tasa de bits en kbps
    bitrate = kbits_sent / duration
    
    return bitrate

# Configuración de las gráficas
plt.ion() # Modo interactivo
fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(10,8))
fig.suptitle('Datos del Arduino')
plt.subplots_adjust(hspace=0.5)
ax[0].set_xlabel('Tiempo (s)')
ax[0].set_ylabel('Ángulo (°)')
ax[0].set_ylim([-180, 180])
ax[0].legend(['Roll', 'Pitch', 'Heading'], loc='upper right')
ax[1].set_xlabel('Tiempo (s)')
ax[1].set_ylabel('Presión (hPa)')
ax[1].legend(['Presión'], loc='upper right')
ax[2].set_xlabel('Tiempo (s)')
ax[2].set_ylabel('Temperatura (°C)')
ax[2].legend(['Temperatura'], loc='upper right')

# Abrir conexión al puerto serie
ser = serial.Serial('COM9', 115200)

# Abrir archivo para escritura
filename = 'data1.txt'
f = open(filename, 'w')

# Escribir la cabecera en la primera línea del archivo
header = 'Roll,Pitch,Heading,Temperature,Pressure,Sea Level Pressure,Altitude,Corrected Altitude,Packet RSSI,Packet SNR'
f.write(header + '\n')

# Crear listas para almacenar los valores de los datos
time_data = []
roll_data = []
pitch_data = []
heading_data = []
pressure_data = []
temperature_data = []

# Inicializar variables para la medición de la tasa de bits
bytes_sent = 0
start_time = time.time()

# Leer datos del puerto serie, guardarlos en el archivo y graficarlos en tiempo real
while True:
    if ser.in_waiting > 0:
        # Leer una línea completa de datos del puerto serie
        line = ser.readline().decode().strip()
        # Separar los valores de la línea utilizando comas
        values = line.split(',')
        # Comprobar si la línea contiene el número correcto de valores
        if len(values) == 10:
            # Imprimir la línea de datos en la consola
            print(line)
            # Escribir la línea de datos en el archivo
            f.write(line + '\n')
            # Incrementar el tamaño de los datos enviados
            bytes_sent += len(line.encode())
            # Forzar la escritura del buffer al archivo
            f.flush()
            # Convertir los valores a números y agregarlos a las listas de datos
            time_data.append(len(time_data) + 1)
            roll_data.append(float(values[0]))
            pitch_data.append(float(values[1]))
            heading_data.append(float(values[2]))
            pressure_data.append(float(values[4]))
            temperature_data.append(float(values[3]))
            # Calcular la tasa de bits de la transmisión
            end_time = time.time()
            bitrate = calculate_bitrate(bytes_sent, start_time, end_time)
            print("Tasa de bits: {:.2f} kbps".format(bitrate))
            # Actualizar las gráficas con los nuevos valores
            # ax[0].plot(time_data, roll_data, 'r-', label='Roll')
            # ax[0].plot(time_data, pitch_data, 'g-', label='Pitch')
            # ax[0].plot(time_data, heading_data, 'b-', label='Heading')
            # ax[1].plot(time_data, pressure_data, 'r-', label='Presión')
            # ax[2].plot(time_data, temperature_data, 'b-', label='Temperatura')
            # plt.draw()
            plt.pause(0.01)

# Cerrar archivo y conexión al puerto serie al finalizar
f.close()
ser.close()
