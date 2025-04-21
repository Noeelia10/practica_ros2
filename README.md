# Práctica ROS2 – Arquitectura de Software 🤖

Este repositorio contiene los ejercicios prácticos del 1 al 8 desarrollados en ROS2, como parte del curso de Arquitectura de Software para Robots. Cada ejercicio implementa un aspecto fundamental de ROS2 como publicación/suscripción, servicios, acciones, uso de interfaces personalizadas y control del Turtlebot3 simulado.

---

## 📁 Contenido del repositorio

### 🔹 `ejercicio1`  
- Publicador simple que envía mensajes.

### 🔹 `ejercicio2`  
- Nodo suscriptor que recibe mensajes.

### 🔹 `ejercicio3`  
- Comunicación entre nodos publicador y suscriptor.

### 🔹 `ejercicio4`  
- Uso de **servicios**.  
- **Depende del paquete `interfaces_pkg`** donde se define el archivo `.srv` correspondiente.

### 🔹 `ejercicio5_action`  
- Implementación de **acciones**: control de movimiento lineal o rotacional del robot.
- **Depende del paquete `acciones_pkg`**, que contiene el archivo de acción `MoverTurtle.action`.

### 🔹 `ejercicio6`  
- Acción para mover el robot simulando un **triángulo**.
- También **usa el archivo `MoverTriangulo.action`** definido en el paquete `acciones_pkg`.

### 🔹 `ejercicio7`  
- Acción para dibujar un **polígono con N lados**.  
- A partir de 11 lados o más, realiza un **movimiento circular**.
- **Depende del archivo `MoverPoligono.action`** definido en `acciones_pkg`.

### 🔹 `ejercicio8`  
- Nodo de **seguridad con LIDAR** que evita colisiones durante el movimiento lineal.  
- Se suscribe a `/scan` y a `/cmd_vel`, y evita publicar comandos si hay peligro cercano.

---

## Paquetes auxiliares

### 🔸 `acciones_pkg`
Contiene los archivos `.action` que usan los ejercicios 5, 6 y 7:
- `MoverTurtle.action`
- `MoverTriangulo.action`
- `MoverPoligono.action`

### 🔸 `interfaces_pkg`
Define el archivo `.srv` que usa el ejercicio 4:
- `Mover.srv` o el correspondiente según la práctica.
