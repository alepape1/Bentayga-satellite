import matplotlib.pyplot as plt
import pandas as pd


start_row = 12212026

# Cargar el archivo CSV en un DataFrame
csv_file = r'log\2023-09-07_00-16-04.csv'  # Reemplaza con la ruta de tu archivo CSV
data = pd.read_csv(csv_file, skiprows=range(1, start_row), nrows=5000 )
print(data)

# Cambiar los nombres de las columnas en el DataFrame
# data.rename(columns={'Time': 'Roll', 'Roll': 'Pitch', 'Pitch': 'Heading', 'Heading': 'Time'}, inplace=True)
print(data.columns)

# Crear una figura con subplots
fig, axs = plt.subplots(5, 1, figsize=(10, 18))

# Graficar Roll, Pitch y Heading
axs[0].plot(data['Roll'], label='Roll')
axs[0].plot(data['Pitch'], label='Pitch')
axs[0].plot(data['Time'], label='Heading')
axs[0].set_ylabel('Ángulo (°)')
axs[0].set_xlabel('Tiempo (s)')
axs[0].legend()

# Graficar Temperatura
# axs[1].plot(data['Batt Temp CubeSat'], label='Batt Temp CubeSat')
axs[1].plot(data['Heading'], label='Batt Down')
axs[1].plot(data['Batt Temp Left'], label='Batt Left')
axs[1].plot(data['Batt Temp Right'], label='Batt Right')
axs[1].plot(data['Batt Temp Box'], label='Batt Box')
axs[1].set_ylabel('Temperatura (°C)')
axs[1].set_xlabel('Tiempo (s)')
axs[1].legend()

# Graficar Presión
axs[2].plot(data['Temperature'], label='Temperature')
axs[2].set_ylabel('External Pressure')
axs[2].set_xlabel('Tiempo (s)')
axs[2].legend()

# Graficar Humedad
axs[3].plot( data['Temperature'], label='Pressure')
axs[3].set_ylabel('Pressure (hPa)')
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
