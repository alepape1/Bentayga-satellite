
#  Aplicación de Visualización de datos del Bentayga

Este manual te guiará a través de los pasos necesarios para ejecutar la aplicación de visualización de datos del CubeSat en tu sistema. La aplicación está escrita en Python y utiliza algunas bibliotecas específicas, por lo que debes asegurarte de que todo esté configurado correctamente.

## Requisitos Previos

Asegúrate de tener lo siguiente instalado en tu sistema:

1. **Python:** La aplicación está escrita en Python, así que necesitas tener Python instalado. Puedes descargar Python desde [python.org](https://www.python.org/downloads/) y seguir las instrucciones de instalación para tu sistema operativo.

2. **PyQt5:** La aplicación utiliza PyQt5 para la interfaz gráfica. Puedes instalarlo utilizando pip:

```sh
pip install PyQt5
```

3. **PyQtGraph:** PyQtGraph se utiliza para crear gráficos en tiempo real en la aplicación. Instálalo con pip:

```sh
pip install pyqtgraph
```
4. **NumPy:**  se utiliza para manipulación de matrices. Si aún no lo tienes instalado, puedes hacerlo con pip:

```sh
pip install numpy
```
## Ejecución de la Aplicación
Una vez que hayas configurado los requisitos previos, sigue estos pasos para ejecutar la aplicación:

1. **Clona el Repositorio:**

Clona el repositorio de la aplicación desde GitHub o descárgalo como un archivo ZIP y descomprímelo en tu sistema si aun no lo tienes descargado.

```sh
git clone https://github.com/alepape1/Bentayga-satellite.git
```

2. **Navega al Directorio de la Aplicación:**

Abre una terminal y navega al directorio donde se encuentra la aplicación. Por ejemplo:
   

3. **Ejecuta la Aplicación:**

Ejecuta la aplicación utilizando Python 3.7 o superior, tendrás que tener todas las dependencias anteriores instaladas:

```sh
python app.py
```


4. **Interactúa con la Aplicación:**

La aplicación se abrirá y mostrará gráficos en tiempo real de los datos del CubeSat.
Puedes ver la información actualizada en los gráficos y las etiquetas de la interfaz.
Cerrar la Aplicación:

Para cerrar la aplicación, simplemente cierra la ventana de la aplicación.
Registro de Datos
La aplicación registrará los datos en un archivo CSV en un directorio llamado "log" dentro del directorio de la aplicación. Los archivos CSV se nombrarán con la fecha y hora en que se inició la aplicación.

**Notas Importantes:**
Asegúrate de que la configuración del puerto COM (por ejemplo, COM25) y la velocidad de baudios (115200) en el código de la aplicación coincidan con la configuración de tu CubeSat.
Si encuentras problemas al ejecutar la aplicación, verifica que todos los paquetes necesarios estén instalados correctamente y que el archivo de código fuente de la aplicación tenga el nombre correcto.
