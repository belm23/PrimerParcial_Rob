# Primer Parcial Robótica
Visualización y Control Cinemático Inverso de Dedos
Implementacion de la visualización y control de cinemática inversa para los dedos índice y pulgar utilizando ROS2 y RViz.
# Objetivos
Implementar la visualización y control de cinemática inversa para el dedo índice.
Implementar la visualización y control de cinemática inversa para el dedo pulgar.

# Configuración del Modelo

Para ejecutar los ejercicios, se debe definir en view_robot.launch.py el modelo por defecto:
default_model_path = PulgarExam.urdf → Para el pulgar
default_model_path = DedoIndice.urdf → Para el dedo índice

# Visualización en RViz

Abrir una terminal y ejecutar los siguientes comandos:

    colcon build
    source install/setup.bash
    ros2 launch robot_description view_robot.launch.py
Esto abrirá RViz, donde se visualizará el URDF correspondiente al dedo seleccionado.

# Control de Cinemática Inversa

Abrir otra terminal para ejecutar la cinemática inversa. Dependiendo del dedo que se quiera controlar:

# Para el pulgar
    colcon build
    source install/setup.bash
    ros2 run visual_pubsub inverse_kinematics

# Para el dedo índice
    colcon build
    source install/setup.bash
    ros2 run visual_pubsub inverse_k
