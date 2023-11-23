# DecMani - Detección y Manipulación de Objetos de menaje

![Imagen o Logo del Proyecto](url_de_la_imagen)

> El objetivo principal de la detección y manipulación de objetos de menaje es dotar a los sistemas robóticos de la capacidad de interactuar de manera inteligente con objetos domésticos, facilitando tareas como la organización, limpieza, y asistencia en entornos hogareños. Este campo encuentra aplicaciones prácticas en la automatización de tareas domésticas y en la mejora de la accesibilidad para personas con necesidades especiales.
>
> Al incorporar algoritmos avanzados de visión por computadora y técnicas de manipulación robótica, los sistemas diseñados para la detección y manipulación de objetos de menaje pueden ofrecer soluciones eficientes y versátiles para mejorar la calidad de vida en el hogar, brindando un enfoque innovador hacia la automatización de actividades diarias.
---
## Contenido

1. [Introducción](#introducción)
2. [Instalación](#instalación)
3. [Uso](#uso)
4. [Estructura del Proyecto](#estructura-del-proyecto)
5. [Contribuciones](#contribuciones)
6. [Licencia](#licencia)
---
## Introducción

### Descripción:

La Detección y Manipulación de Objetos de Menaje mediante ROS2 y Detectron2 representa un enfoque avanzado para la automatización de tareas en entornos domésticos. Este proyecto se centra en la integración de dos tecnologías líderes: ROS2, un marco de trabajo de robótica de código abierto, y Detectron2, una biblioteca de detección de objetos basada en aprendizaje profundo.

### Contexto:

En un contexto donde la robótica y la inteligencia artificial se fusionan para mejorar la interacción humano-máquina, la detección y manipulación de objetos de menaje adquiere especial relevancia. La capacidad de un robot para identificar y manipular objetos comunes en el hogar presenta oportunidades significativas para la asistencia en la vida diaria y la simplificación de tareas domésticas.

ROS2, como marco de trabajo, proporciona la estructura necesaria para el desarrollo de sistemas robóticos complejos. Su capacidad para facilitar la comunicación entre módulos, manejar el control de hardware y permitir la expansión modular lo convierte en una elección lógica para proyectos de este tipo. Detectron2, por otro lado, aporta poderosas capacidades de detección de objetos basadas en modelos de aprendizaje profundo, lo que permite una identificación precisa y eficiente de objetos en imágenes o videos.

### Objetivos:

Detección Precisa: Implementar algoritmos de detección de objetos utilizando Detectron2 para lograr una identificación precisa y robusta de objetos de menaje en entornos domésticos.

Integración con ROS2: Integrar los resultados de la detección de objetos en el marco de trabajo ROS2 para facilitar la coordinación y control de sistemas robóticos.

Manipulación Robótica: Desarrollar capacidades de manipulación robótica que permitan al sistema interactuar físicamente con los objetos identificados, utilizando información proporcionada por la detección.

Eficiencia y Tiempo Real: Optimizar el rendimiento del sistema para garantizar una ejecución eficiente y en tiempo real, crucial para la aplicabilidad práctica en entornos dinámicos.

Escalabilidad y Modularidad: Diseñar el sistema de manera que sea escalable y modular, permitiendo futuras expansiones y adaptaciones a diferentes entornos y requisitos específicos.

Este proyecto tiene como objetivo no solo demostrar la viabilidad técnica de la detección y manipulación de objetos de menaje, sino también sentar las bases para soluciones robóticas más avanzadas y accesibles en el ámbito doméstico.
---
## Instalación

> Instrucciones paso a paso sobre cómo instalar y configurar el proyecto.

### Instalación ROS2 Humble

Este proyecto ha sido realizado en el SO Ubuntu 22.04.

Para la instalación de ROS2 Humble en el SO mencionado debe seguir los pasos de la página oficial: [Tutorial Instalación ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Para configurar el entorno es recomendable seguir el siguiente tutorial: [Tutorial Beginners: CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)

### Creación del workspace

Antes de realizar la clonación del repositorio es necesario crear un workspace:
```
mkdir -p ~/decmani_ws/src
cd ~/decmani_ws/src
```

Clonamos el repositorio:
```
git clone https://github.com/jtaboadab/decmani.git
```

Volvemos a la raiz del workspace y realizamos colcon build:
```
cd ~/decmani_ws
```
```
colcon build
```
### Creación de un entorno virtual para instalar la librerías necesarias
---
## Uso
---
## Estructura del Proyecto
---
## Contribuciones
---
## Licencia
