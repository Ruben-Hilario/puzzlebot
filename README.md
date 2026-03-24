1. Build the docker
	(Cambiar el remote username en Dockerfile y devcontainer.json)

2. Setup de lidar
	chmod +x ./clone.sh
	./clone.sh

4. Probar el lidar con rviz
	ros2 launch rplidar_ros view_rplidar_a1_launch.py serial_port:='dev/ttyUSB1'
	(Revisar puerto al que está conectado 'ls /dev | grep tty', puede aparecer como USB o ACM)

