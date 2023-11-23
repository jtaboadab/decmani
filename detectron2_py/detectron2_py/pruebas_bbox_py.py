import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calcular_bounding_box_3d(centroide, dimensiones):
    
    ancho, alto, profundidad = dimensiones
    x, y, z = centroide

    # Calcular esquinas del bounding box
    esquina_frontal_inferior_izquierda = [x - ancho / 2, y - alto / 2, z]
    esquina_frontal_superior_izquierda = [x - ancho / 2, y + alto / 2, z]
    esquina_frontal_superior_derecha = [x + ancho / 2, y + alto / 2, z]
    esquina_frontal_inferior_derecha = [x + ancho / 2, y - alto / 2, z]

    esquina_trasera_inferior_izquierda = [x - ancho / 2, y - alto / 2, z + profundidad]
    esquina_trasera_superior_izquierda = [x - ancho / 2, y + alto / 2, z + profundidad]
    esquina_trasera_superior_derecha = [x + ancho / 2, y + alto / 2, z + profundidad]
    esquina_trasera_inferior_derecha = [x + ancho / 2, y - alto / 2, z + profundidad]

    bounding_box_3d = [
        
        esquina_frontal_inferior_izquierda,
        esquina_frontal_superior_izquierda,
        esquina_frontal_superior_derecha,
        esquina_frontal_inferior_derecha,
        esquina_trasera_inferior_izquierda,
        esquina_trasera_superior_izquierda,
        esquina_trasera_superior_derecha,
        esquina_trasera_inferior_derecha
    ]

    return bounding_box_3d

# Ejemplo de uso
centroide = [0.3, 0.0475, 0.5]  # Coordenadas (x, y, z) del centroide
dimensiones = [0.12, 0.095, 0.08]  # Dimensiones (ancho, alto, profundidad) del objeto

bounding_box_3d = calcular_bounding_box_3d(centroide, dimensiones)

# Visualización del bounding box 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Dibujar las líneas del bounding box
for i in range(4):
    
    ax.plot([bounding_box_3d[i][0], bounding_box_3d[(i + 1) % 4][0]],
            [bounding_box_3d[i][1], bounding_box_3d[(i + 1) % 4][1]],
            [bounding_box_3d[i][2], bounding_box_3d[(i + 1) % 4][2]], 'b-')

ax.plot([bounding_box_3d[i][0] for i in range(4)],
        [bounding_box_3d[i][1] for i in range(4)],
        [bounding_box_3d[i][2] for i in range(4)], 'b-')

for i in range(4):
    
    ax.plot([bounding_box_3d[i + 4][0], bounding_box_3d[(i + 1) % 4 + 4][0]],
            [bounding_box_3d[i + 4][1], bounding_box_3d[(i + 1) % 4 + 4][1]],
            [bounding_box_3d[i + 4][2], bounding_box_3d[(i + 1) % 4 + 4][2]], 'b-')

ax.plot([bounding_box_3d[i + 4][0] for i in range(4)],
        [bounding_box_3d[i + 4][1] for i in range(4)],
        [bounding_box_3d[i + 4][2] for i in range(4)], 'b-')

plt.show()