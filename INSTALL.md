# Instalación

> Instrucciones paso a paso sobre cómo instalar y configurar el proyecto.
> 
---

## Instalación ROS2 Humble

Este proyecto ha sido realizado en el S.O. Ubuntu 22.04.

Para la instalación de ROS2 Humble en el S.O. mencionado debe seguir los pasos de la página oficial: [Tutorial Instalación ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Para configurar el entorno es recomendable seguir el siguiente tutorial: [Tutorial Beginners: CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)

---

## Creación del workspace

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

---

## Creación de un entorno virtual

Abre tu terminal y ejecuta los siguientes comandos para crear y activar un entorno virtual:

Instalación de virtualenv
```
pip install virtualenv
```
Creación del entorno virtual
```
python -m venv myenv
```
Activación del entorno virtual
```
source myenv/bin/activate
```

---

## Instalación de las librerías necesarias

- Linux con Python ≥ 3.7
  
- PyTorch ≥ 1.8 y torchvision que coincidan con la instalación de PyTorch. Instálalos juntos en  [pytorch.org](https://pytorch.org/) para asegurarte
  
- gcc & g++ ≥ 5.4 son requeridos
  
- OpenCV
  ```
  pip install opencv-python
  ```
  
- CvBridge
  ```
  pip install cvbridge3
  ```
  
- NumPy
  ```
  pip install numpy
  ```
- Detectron2
  Build Detectron2 from Source
  ```
  python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'
  # (add --user if you don't have permission)

  # Or, to install it from a local clone:
  git clone https://github.com/facebookresearch/detectron2.git
  python -m pip install -e detectron2
  ```
