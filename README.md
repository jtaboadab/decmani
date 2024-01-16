# DecMani - Detección y Manipulación de Objetos de menaje

![Imagen o Logo del Proyecto](https://github.com/jtaboadab/decmani/blob/main/Images/Escena.jpg)

> El objetivo principal de la detección y manipulación de objetos de menaje es dotar a los sistemas robóticos de la capacidad de interactuar de manera inteligente con objetos domésticos, facilitando tareas como la organización, limpieza, y asistencia en entornos hogareños. Este campo encuentra aplicaciones prácticas en la automatización de tareas domésticas y en la mejora de la accesibilidad para personas con necesidades especiales.
>
> Al incorporar algoritmos avanzados de visión por computadora y técnicas de manipulación robótica, los sistemas diseñados para la detección y manipulación de objetos de menaje pueden ofrecer soluciones eficientes y versátiles para mejorar la calidad de vida en el hogar, brindando un enfoque innovador hacia la automatización de actividades diarias.

---

## Contenido

1. [Introducción](#introducción)
2. [Instalación](#instalación)
3. [Resultados](#resultados)
4. [Estructura del Proyecto](#estructura-del-proyecto)
5. [Licencia](#licencia)
6. [Contribuciones](#contribuciones)
   
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

## Instalación

Mira las [instrucciones de instalación](https://github.com/jtaboadab/decmani/blob/main/INSTALL.md).

---

## Resultados

[![Detección y manipulación de varios vasos](https://img.youtube.com/vi/qSP8cxH0kIA/0.jpg)](https://www.youtube.com/watch?v=qSP8cxH0kIA)

---

## Estructura del Proyecto

### Archivos Principales

- `detectron2_node.py`: Archivo principal que contiene la implementación del nodo ROS2 con Detectron2.
- `requirements.txt`: Lista de dependencias del proyecto.

### Carpetas

- **src/**: Contiene el código fuente del proyecto.
  - `__init__.py`: Archivo de inicialización para tratar la carpeta como un paquete Python.
  - `detectron2_node.py`: Implementación del nodo de ROS2 con Detectron2.
  - `utils/`: Carpeta que contiene módulos o scripts utilitarios.
    - `helper_functions.py`: Funciones auxiliares para el procesamiento de imágenes u otras tareas.
- **msg_srv_creator/**: Contiene los archivos de definición de mensajes de ROS (si aplica).
  - `Mask.msg`: Definición del mensaje de máscara.
- **docs/**: Documentación adicional si es necesario.
- **tests/**: Archivos de prueba si es relevante.

### Configuración y Scripts

- **config/**: Archivos de configuración o modelos preentrenados para Detectron2.
  - `mask_rcnn_config.yaml`: Configuración del modelo Mask R-CNN.
- **launch/**: Archivos de lanzamiento de ROS2 si es aplicable.
  - `detectron2_node_launch.py`: Archivo de lanzamiento para el nodo Detectron2.
- `setup.py`: Archivo de configuración para la instalación del proyecto.

### Otros Archivos

- `.gitignore`: Lista de archivos y carpetas que deben ignorarse al realizar seguimiento con Git.
- `LICENSE`: Licencia del proyecto.
- `README.md`: Documentación principal del proyecto.

---

## Licencia

DecMani se publica bajo la licencia [Apache 2.0 license](https://github.com/jtaboadab/decmani/blob/main/LICENSE.md).

---

## Contribuciones

¡Gracias por considerar contribuir a DecMani! Este proyecto se beneficia de la colaboración y apreciamos las contribuciones de la comunidad.
