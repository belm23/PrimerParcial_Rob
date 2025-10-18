# PrimerParcial_Rob

Primeramente, ingresar al workspace (1er_Parcial) y usar colcon build, luego usar source install/local_setup.bash para preparar los 5 entornos.

Para el ejercicio 2, se necesitara de 5 pestañas de la terminal para comprobar los 3 sensores, el promedio de los mismos y el suscriptor final usando los siguientes comandos en cada terminal:

ros2 run my_package pub1

ros2 run my_package pub2

ros2 run my_package pub3

ros2 run my_package filter

ros2 run my_package finish
