# Sistema de Teleoperación usando RV con ROS2

<p align="center">
  <img src="assets/teleoperacion.jpeg" width="700">
</p>


Sistema ROS 2 desarrollado para un proyecto de **teleoperación robótica mediante realidad virtual**, integrando percepción 6-DoF, planificación de movimientos y control de un manipulador desde Unity.

---
## Tabla de Contenidos

- [Características del sistema](#características-del-sistema)
- [Paquetes oficiales](#paquetes-oficiales)
- [Contribuciones del proyecto](#contribuciones-del-proyecto)
---

## Características del sistema

- Comunicación bidireccional entre ROS 2 y Unity mediante ROS-TCP.
- Integración con el manipulador xArm Lite 6, tanto en modo simulado como en robot real.
- Publicación y corrección de poses 6-DoF provenientes de sistemas de visión.
- Generación y gestión de transformaciones TF dinámicas y estáticas.
- Control independiente del gripper, desacoplado del control del brazo.
- Compatibilidad con el framework de planificación MoveIt 2.
- Arquitectura modular y extensible, orientada a la integración de nuevos sensores y dispositivos.


---

## Paquetes oficiales

### ROS-TCP-Endpoint
Paquete oficial utilizado como puente de comunicación entre Unity y ROS 2.  
Permite la transmisión de poses, estados y comandos desde el entorno de realidad virtual hacia el sistema ROS.

- Repositorio oficial: https://github.com/Unity-Technologies/ROS-TCP-Endpoint
- Función principal: comunicación ROS 2 ↔ Unity
- Estado en el proyecto: integrado sin modificaciones internas

---

### xarm_ros2
Paquete oficial desarrollado por UFactory para el control del manipulador xArm Lite 6.

Incluye los elementos necesarios para la operación del robot, tanto en simulación como en robot real.

- Repositorio oficial: https://github.com/xArm-Developer/xarm_ros2
- Componentes principales:
  - Descripción del robot (URDF)
  - Integración con ros2_control
  - Configuración para MoveIt 2
- Estado en el proyecto: utilizado como base del sistema, sin modificaciones estructurales


## Contribuciones del proyecto

### foundation_pose_tf
Paquete desarrollado en este proyecto para el procesamiento y transformación de poses 6-DoF provenientes de sistemas de visión.

Su función principal es adaptar las poses detectadas a marcos de referencia compatibles con ROS y con los módulos de planificación de movimiento.

Funciones principales:
- Conversión de mensajes **PoseStamped** a transformaciones TF válidas.
- Corrección y alineación de marcos de referencia.
- Publicación de marcadores de visualización para RViz.
- Integración directa con el pipeline de planificación de agarre (grasp planning).

Ejemplo de ejecución:
```
ros2 run foundation_pose_tf pose_to_tf
```
### gripper_description
Paquete desarrollado en este proyecto que contiene la descripción del efector final utilizada por el sistema.

Incluye:
- Modelos de malla (meshes) de la base del gripper.
- Modelos de malla del dedo izquierdo y del dedo derecho.
- Configuración cinemática del gripper integrada al URDF base del robot.

Este paquete permite visualizar el gripper correctamente ensamblado junto al manipulador en RViz, facilitando la simulación, la validación del modelo completo del sistema robótico y su uso en planificación de movimiento.

---

### gripper_control
Paquete desarrollado en este proyecto encargado de la lógica de control del gripper.

Permite la apertura y el cierre del gripper de manera coherente en distintos entornos de ejecución.

Funciones principales:
- Control del gripper en simulación mediante RViz.
- Control del gripper en el robot real.
- Control del gripper desde Unity mediante mensajes JSON enviados a través del servidor web proporcionado por el fabricante del robot.
- Sincronización del estado del gripper entre ROS, Unity y el sistema físico.

Este paquete desacopla el control del gripper del control del brazo, facilitando su integración en flujos de Pick and Place y en esquemas de teleoperación.
