# DecMani - Detección y Manipulación de Objetos de menaje

![Imagen o Logo del Proyecto](https://github.com/jtaboadab/decmani/blob/main/images/Escena.jpg)

> El objetivo principal de la detección y manipulación de objetos de menaje es dotar a los sistemas robóticos de la capacidad de interactuar de manera inteligente con objetos domésticos, facilitando tareas como la organización, limpieza, y asistencia en entornos hogareños. Este campo encuentra aplicaciones prácticas en la automatización de tareas domésticas y en la mejora de la accesibilidad para personas con necesidades especiales.
>
> Al incorporar algoritmos avanzados de visión por computadora y técnicas de manipulación robótica, los sistemas diseñados para la detección y manipulación de objetos de menaje pueden ofrecer soluciones eficientes y versátiles para mejorar la calidad de vida en el hogar, brindando un enfoque innovador hacia la automatización de actividades diarias.

---

## Contenido

1. [Introducción](#introducción)
2. [Dispositivos utilizados](#dispositivos-utilizados)
3. [Instalación](#instalación)
4. [Resultados](#resultados)
5. [Estructura del Proyecto](#estructura-del-proyecto)
6. [Licencia](#licencia)
7. [Contribuciones](#contribuciones)
   
---

## Introducción

### Descripción:

La Detección y Manipulación de Objetos de Menaje mediante [ROS2](https://www.ros.org/) y [Detectron2](https://github.com/facebookresearch/detectron2/) representa un enfoque avanzado para la automatización de tareas en entornos domésticos. Este proyecto se centra en la integración de dos tecnologías líderes: ROS2, un marco de trabajo de robótica de código abierto, y Detectron2, una biblioteca de detección de objetos basada en aprendizaje profundo.

### Contexto:

En un contexto donde la robótica y la inteligencia artificial se fusionan para mejorar la interacción humano-máquina, la detección y manipulación de objetos de menaje adquiere especial relevancia. La capacidad de un robot para identificar y manipular objetos comunes en el hogar presenta oportunidades significativas para la asistencia en la vida diaria y la simplificación de tareas domésticas.

ROS2, como marco de trabajo, proporciona la estructura necesaria para el desarrollo de sistemas robóticos complejos. Su capacidad para facilitar la comunicación entre módulos, manejar el control de hardware y permitir la expansión modular lo convierte en una elección lógica para proyectos de este tipo. Detectron2, por otro lado, aporta poderosas capacidades de detección de objetos basadas en modelos de aprendizaje profundo, lo que permite una identificación precisa y eficiente de objetos en imágenes o videos.

### Objetivos:

   - Detección Precisa: Implementar algoritmos de detección de objetos utilizando Detectron2 para lograr una identificación precisa y robusta de objetos de menaje en entornos domésticos.

   - Integración con ROS2: Integrar los resultados de la detección de objetos en el marco de trabajo ROS2 para facilitar la coordinación y control de sistemas robóticos.

   - Manipulación Robótica: Desarrollar capacidades de manipulación robótica que permitan al sistema interactuar físicamente con los objetos identificados, utilizando información proporcionada por la detección.

   - Eficiencia y Tiempo Real: Optimizar el rendimiento del sistema para garantizar una ejecución eficiente y en tiempo real, crucial para la aplicabilidad práctica en entornos dinámicos.

   - Escalabilidad y Modularidad: Diseñar el sistema de manera que sea escalable y modular, permitiendo futuras expansiones y adaptaciones a diferentes entornos y requisitos específicos.

Este proyecto tiene como objetivo no solo demostrar la viabilidad técnica de la detección y manipulación de objetos de menaje, sino también sentar las bases para soluciones robóticas más avanzadas y accesibles en el ámbito doméstico.

---

## Dispositivos utilizados

**Cámara RGB-D Orbbec Atra Pro Plus**

![Astra Pro Plus](https://github.com/jtaboadab/decmani/blob/main/images/AstraProPlus.jpg)

**Brazo Robótico Interbotix WidowX 250**

![Interbotix Widowx 250](https://github.com/jtaboadab/decmani/blob/main/images/wx250.png)

**Marcador ArUco**

![Marcador ArUco](https://github.com/jtaboadab/decmani/blob/main/images/ArUco.PNG)

---

## Instalación

Mira las [instrucciones de instalación](https://github.com/jtaboadab/decmani/blob/main/INSTALL.md).

---

## Resultados

**¡Clicar la imagen para ir al vídeo!**

Detección y manipulación de varios vasos:

[![Detección y manipulación de varios vasos](https://img.youtube.com/vi/qSP8cxH0kIA/0.jpg)](https://www.youtube.com/watch?v=qSP8cxH0kIA)

Comprobación de la calibración cámara-brazo:

[![Calibración cámara-brazo](https://img.youtube.com/vi/SP1khlVxjg4/0.jpg)](https://www.youtube.com/watch?v=SP1khlVxjg4)

Vista desde el ordenador:

[![Vista desde el ordenador](https://img.youtube.com/vi/OjQWqC-PPnk/0.jpg)](https://www.youtube.com/watch?v=OjQWqC-PPnk)

---

## Estructura del Proyecto

### Carpetas

- **Images/**: Contiene las imágenes del proyecto.
- **arm_robot_py/**: Contiene el nodo que controla el brozo robótico.
- **calibration_cpp/**: Contiene el nodo de calibración cámara-brazo.
- **camera_py/**: Contiene un nodo para guardar imágenes.
- **coord_trans_py/**: Contiene el nodo para calcular las coordenadas 3D del objeto con respecto el brazo robótico.
- **detectron_py/** Contiene el nodo de detección de objetos y el nodo del cálculo de la posición del objeto con respecto a la cámara.
- **dynamixel-workbench/**, **interbotix_ros_core/**, **interbotix_ros_manipulators/**, **interbotix_ros_toolboxes/**, **interbotix_xs_driver/**: Son los nodos proporcionados por el fabricante para poder usar el brazo robótico.
- **msg_srv_creator/**: Contiene los archivos de definición de mensajes de ROS.
- **ros2_astra_camera/**: Es el nodo prporcionado por el fabricante para poder usar la cámara RGB-D.


### Scripts

- **arm_robot_py/**
  - arm_robot_py/
    - `arm_robot_py.py`
- **calibration_cpp/**
  - src/
    - `calibration_cpp.cpp`
- **camera_py/**
  - camera_py/
    - `camera_py`
- **coord_trans_py/**
  - coord_trans_py/
    - `coord_trans_py.py`
- **detectron2_py/**
  - detectron2_py/
    - `detectron2_py.py`
    - `bbox_py.py`

### Otros Archivos

- `.gitignore`: Lista de archivos y carpetas que deben ignorarse al realizar seguimiento con Git.
- `INSTALL.md`: Tutorial de instalación.
- `LICENSE.md`: Licencia del proyecto.
- `README.md`: Documentación principal del proyecto.

---

## Licencia

DecMani se publica bajo la licencia [Apache 2.0 license](https://github.com/jtaboadab/decmani/blob/main/LICENSE.md).

---

## Contribuciones

¡Gracias por considerar contribuir a DecMani! Este proyecto se beneficia de la colaboración y apreciamos las contribuciones de la comunidad.
