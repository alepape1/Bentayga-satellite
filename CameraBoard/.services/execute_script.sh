#!/bin/bash
echo "Starting camera capturing process."

BASE_DIR=/home/bentayga/bentayga
BACKUP_DIR=/media/bentayga/BACKUP
export TERM=xterm-256color

source $BASE_DIR/venv/bin/activate

export PATH_TO_SENSORS=$BASE_DIR/sensors
SENSORS_LOG=$PATH_TO_SENSORS/sensors_detection.log

python3 $PATH_TO_SENSORS/setup_system.py --log-out $SENSORS_LOG --backup_path $BACKUP_DIR
sensor_result=$(echo $?)
OUTPUTS_DIR=$BASE_DIR/outputs

if [ $sensor_result == 0 ]; then
    # PARAMETROS: 1 (ganancia - dejarlo asi) 30000000 (tiempo de exposicion en ns) 10 (tiempo en segundos entre capturas) -i 1 (interfaz i2c - depende de la JETSON) 
	python3.8 $BASE_DIR/src/main.py 1 30000000 10 -i 1 --data_capturing_path $OUTPUTS_DIR --backup_path $BACKUP_DIR/outputs
	#python3.8 $BASE_DIR/src/test_main.py 1 30000000 10 -i 1 --data_capturing_path $OUTPUTS_DIR --backup_path $BACKUP_DIR/outputs
fi

echo "Script finished"
exit

