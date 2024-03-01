import matplotlib.pyplot as plt
import pandas as pd
import tkinter as tk
from tkinter import filedialog
import matplotlib.pyplot as plt
import pandas as pd
import os

# Obtener el directorio actual como punto de partida
current_directory = os.getcwd()


# Función para cargar y graficar el archivo CSV
def cargar_y_graficar():
    
    file_path = filedialog.askopenfilename(filetypes=[("CSV Files", "*.csv")], initialdir = current_directory)
    
    if file_path:

        start_row = 0

        # Cargar el archivo CSV en un DataFrame
        csv_file = file_path  # Reemplaza con la ruta de tu archivo CSV
        data = pd.read_csv(csv_file, skiprows=range(1, start_row), nrows=5000 )
        print(data['Time'])

        # Cambiar los nombres de las columnas en el DataFrame
        print(data.columns)
        print(data)

        # Crear la primera ventana de gráficos con 2x2 subplots
        fig1, axs1 = plt.subplots(2, 2, figsize=(12, 8))
        fig1.suptitle('Flight Actitude and GPS track', fontsize=16)

        # Graficar Roll, Pitch y Heading en la misma gráfica
        axs1[0, 0].plot(data['Time'], data['Roll'], label='Roll')
        axs1[0, 0].plot(data['Time'], data['Pitch'], label='Pitch')
        axs1[0, 0].plot(data['Time'], data['Heading'], label='Heading')
        axs1[0, 0].set_ylabel('Ángulo (°)')
        axs1[0, 0].set_xlabel('Tiempo (s)')
        axs1[0, 0].legend()
        axs1[0, 0].set_ylim(-180.0, 180.0)  # Cambia estos valores según tus datos

        axs1[0, 1].plot(data['Time'], data['Latitude'], label='Latitude')
        axs1[0, 1].plot(data['Time'], data['Longitude'], label='Longitude')
        axs1[0, 1].set_ylabel('Valor')
        axs1[0, 1].set_xlabel('Tiempo (s)')
        axs1[0, 1].legend()

        axs1[1, 1].plot(data['Time'], data['Speed'], label='Speed')
        axs1[1, 1].set_ylabel('Valor')
        axs1[1, 1].set_xlabel('Tiempo (s)')
        axs1[1, 1].legend()

        # Graficar Altitud GPS y Velocidad en la segunda ventana
        axs1[1, 0].plot(data['Time'], data['GpsAltitude'], label='GpsAltitude')
        axs1[1, 0].set_ylabel('Valor')
        axs1[1, 0].set_xlabel('Tiempo (s)')
        axs1[1, 0].legend()

        # Crear la segunda ventana de gráficos con 2x2 subplots
        fig2, axs2 = plt.subplots(2, 2, figsize=(12, 8))
        fig2.suptitle('Meteorological data and Batt temp', fontsize=16)
        # Graficar Temperatura en la segunda ventana
        axs2[0, 0].plot(data["Time"], data['Temperature'], label='Temperature')
        axs2[0, 0].set_ylabel('Temperatura (°C)')
        axs2[0, 0].set_xlabel('Tiempo (s)')
        axs2[0, 0].legend()

        # Graficar Presion en la segunda ventana
        axs2[0, 1].plot(data["Time"], data['Pressure'], label='Pressure')
        axs2[0, 1].set_ylabel('External Pressure (hPa)')
        axs2[0, 1].set_xlabel('Tiempo (s)')
        axs2[0, 1].legend()
        axs2[0, 1].set_ylim(0, 1200)  # Cambia estos valores según tus datos

        # Graficar Humedad en la segunda ventana
        axs2[1, 0].plot(data["Time"], data['Humidity'], label='Humidity')
        axs2[1, 0].set_ylabel('Humidity (%)')
        axs2[1, 0].set_xlabel('Tiempo (s)')
        axs2[1, 0].legend()


        # Graficar Temperatura de Baterias en la primera ventana
        axs2[1, 1].plot(data["Time"], data['Batt Temp Down'], label='Batt Down')
        axs2[1, 1].plot(data["Time"], data['Batt Temp Left'], label='Batt Left')
        axs2[1, 1].plot(data["Time"], data['Batt Temp Right'], label='Batt Right')
        axs2[1, 1].plot(data["Time"], data['Batt Temp Box'], label='Batt Box')
        axs2[1, 1].set_ylabel('Temperatura (°C)')
        axs2[1, 1].set_xlabel('Tiempo (s)')
        axs2[1, 1].legend()

        # Crear la tercera ventana de gráficos con 1x2 subplots
        fig3, axs3 = plt.subplots(1, 2, figsize=(12, 8))
        fig3.suptitle('Reception Signal RF', fontsize=16)

        # Graficar Presion en la segunda ventana
        axs3[0].plot(data["Time"], data['Intensity'], label='Intensity', color='orange')
        axs3[0].set_ylabel('Signal Intensity (dBm)')
        axs3[0].set_xlabel('Tiempo (s)')
        axs3[0].legend()
 

        # Graficar Humedad en la segunda ventana
        axs3[1].plot(data["Time"], data['SNR'], label='SNR', color='purple')
        axs3[1].set_ylabel('SNR ratio')
        axs3[1].set_xlabel('Tiempo (s)')
        axs3[1].legend()

        # Ajustar el diseño de los subplots
        plt.tight_layout()

        # Mostrar la gráfica
        plt.show()

# Crear la ventana principal
root = tk.Tk()
root.title("Selección de archivo CSV")

# Botón para cargar y graficar el archivo CSV
cargar_button = tk.Button(root, text="Cargar Archivo CSV", command=cargar_y_graficar)
cargar_button.pack()

# Iniciar la aplicación
root.mainloop()