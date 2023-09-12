import matplotlib.pyplot as plt
import pandas as pd

start_row = 3

# Cargar el archivo CSV en un DataFrame
csv_file = r'log\2023-09-11_23-58-14.csv'  # Reemplaza con la ruta de tu archivo CSV

# Define los nombres de las columnas en el orden correcto
column_names = ["Time", "Roll", "Pitch", "Heading", "Batt Temp Left", "Batt Temp Right",
                "Batt Temp Down", "Batt Temp Box", "Batt Temp CubeSat", "Temperature", "Pressure",
                "Humidity", "GpsAltitude", "Latitude", "Longitude", "Speed", "NumSatellites",
                "Year", "Month", "Day", "Hour", "Minute", "Second", "Camera Info", "Intensity", "SNR"]


data = pd.read_csv(csv_file, header= 1,  skiprows=range(1, start_row), nrows=5000 , sep=',', names = column_names )
print(data.columns)
print(data['Time'])
print(data)

# Crear una figura con subplots
fig, axs = plt.subplots(5, 1, figsize=(10, 18))

# Graficar Roll, Pitch y Heading
axs[0].plot(data['Time'], label='Time')
axs[0].plot(data['Roll'], label='Roll')
axs[0].plot(data['Pitch'], label='Pitch')
axs[0].plot(data['Heading'], label='Heading')
axs[0].set_ylabel('Ángulo (°)')
axs[0].set_xlabel('Tiempo (s)')
axs[0].legend()

# Graficar Temperatura
axs[1].plot(data['Time'], data['Batt Temp CubeSat'], label='Batt Temp CubeSat')
axs[1].plot(data['Time'], data['Batt Temp Down'], label='Batt Down')
axs[1].plot(data['Time'], data['Batt Temp Left'], label='Batt Left')
axs[1].plot(data['Time'], data['Batt Temp Right'], label='Batt Right')
axs[1].plot(data['Time'], data['Batt Temp Box'], label='Batt Box')
axs[1].set_ylabel('Temperatura (°C)')
axs[1].set_xlabel('Tiempo (s)')
axs[1].legend()

# Graficar Presión
axs[2].plot(data['Time'], data['Pressure'], label='Pressure')
axs[2].set_ylabel('External Pressure')
axs[2].set_xlabel('Tiempo (s)')
axs[2].legend()

# Graficar Humedad
axs[3].plot(data['Time'], data['Humidity'], label='Humidity')
axs[3].set_ylabel('Humedad')
axs[3].set_xlabel('Tiempo (s)')
axs[3].legend()

# Graficar Altitud GPS y Velocidad
axs[4].plot(data['Time'], data['GpsAltitude'], label='GpsAltitude')
axs[4].plot(data['Time'], data['Speed'], label='Speed')
axs[4].set_ylabel('Valor')
axs[4].set_xlabel('Tiempo (s)')
axs[4].legend()

# Ajustar el diseño de los subplots
plt.tight_layout()

# Mostrar la gráfica
plt.show()
