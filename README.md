# simple_navigation_goals

## Instalar dependencias y clonar el proyecto
```
sudo apt install ros-melodic-navigation
mkdir -p ~/bot_ws/src
cd ~/bot_ws/src
git clone git@github.com:mjd-x/simple_navigation_goals.git
cd ..
catkin_make install
source devel/setup.bash
```

## Correr el nodo

Recorrer cualquier secuencia de puntos especificada en movebase_seq.launch:

```
roslaunch simple_navigation_goals movebase_seq.launch
```

o

Recorrer un cuadrado del largo especificado en movebase_square.launch:

```
roslaunch simple_navigation_goals movebase_square.launch
```

[Video demo v0.1](https://www.youtube.com/watch?v=j8ioTDPSb-U)

## Correr todo el stack en el robot:

Iniciar el stack de navegacion:

```
roslaunch rtabmap_robot2_fie navigating_imu.launch
```

Iniciar el nodo como se describe en "Correr el nodo"

Visualizacion con rviz:

```
rosrun rviz rviz -d mapping.rviz
```