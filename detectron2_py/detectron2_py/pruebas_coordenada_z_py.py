import cv2
import numpy as np
import matplotlib.pyplot as plt
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

## Detectron2
cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
predictor = DefaultPredictor(cfg)

im_color = cv2.imread('/home/tfg/dectmani_ws/src/detectron2_py/detectron2_py/color.jpg')
im_depth = cv2.imread('/home/tfg/dectmani_ws/src/detectron2_py/detectron2_py/normalized_depth.jpg')
profundidad = cv2.imread('/home/tfg/dectmani_ws/src/detectron2_py/detectron2_py/depth.jpg')
        
outputs = predictor(im_color)

# Inicializamos un vector para almacenar la posición en la que se encuentran los objetos deseados
vector = []

# Mediante el bucle almacenamos la posición de aquellos objetos que sean de la clase 41
for iter in range(len(outputs['instances'])):
    
    if outputs['instances'].pred_classes[iter].item() == 41:
        
        vector.append(iter)

# Creamos el tensor de los objetos deseados mediante el vector
masks = outputs['instances'].pred_masks[vector]

# Obtenemos la parte del array del tensor
masks = masks.detach().to('cpu').numpy()

# Inicializamos una lista para guardar las máscaras de las imágenes en forma binarizada
mask_images = []

for k in range(len(masks)): 
     
    x, y = masks[k].shape  
    mask_image = np.zeros((x, y), dtype=np.uint8) 

    for i in range(x):
        
        for j in range(y):
            
            if masks[k][i][j]:
                
                mask_image[i][j] = 255

## Realizamos es escalado
# Dimensiones originales
original_height, original_width = im_color.shape[:2]

# Factor de escala
scale_factor = 0.925

# Calcular el nuevo tamaño manteniendo las dimensiones originales
new_height = int(original_height * scale_factor)
new_width = int(original_width * scale_factor)

# Calcular los píxeles de relleno
padding_top = (original_height - new_height) // 2
padding_bottom = original_height - new_height - padding_top
padding_left = (original_width - new_width) // 2
padding_right = original_width - new_width - padding_left

# Escalar la imagen
scaled_image = cv2.resize(im_depth, (new_width, new_height))

# Añadir píxeles de relleno
scaled_image = cv2.copyMakeBorder(scaled_image, padding_top, padding_bottom, padding_left, padding_right, cv2.BORDER_CONSTANT, value=0)

# Valores extrínsecos que relacionan ambas cámaras
M=np.array([[1, 0, -25.463199615478516], [0, 1, -0.20235499739646912]])
P=np.array([[0.9999510049819946, 0.009837210178375244, 0],[-0.009841140359640121, 0.9999340176582336, 0], [0, 0, 1]])

# Aplicar la transformación
scaled_image_a = cv2.warpAffine(scaled_image, M, (im_color.shape[1], im_color.shape[0]))
scaled_image_p = cv2.warpPerspective(scaled_image_a, P, (im_color.shape[1], im_color.shape[0]))

## Visualizamos resultados
# Crear subplots
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

# Mostrar la primera imagen en el primer subplot
axes[0].imshow(im_color)
axes[0].set_title('Im Color')
axes[0].axis('off')

# Mostrar la segunda imagen en el segundo subplot
axes[1].imshow(im_depth)
axes[1].set_title('Im Profundidad')
axes[1].axis('off')

# Superponer las imágenes en el tercer subplot
axes[2].imshow(scaled_image_p)
axes[2].set_title('Im Prof Escalada')
axes[2].axis('off')

plt.suptitle('Comparativa Imágenes')

# Ajustar el espacio entre subplots
plt.tight_layout()

# Mostrar los subplots
plt.show()

# Factor de transparencia (ajusta según sea necesario)
alpha = 0.5

# Superponer las imágenes
im_superposed_image = cv2.addWeighted(im_color, 1 - alpha, im_depth, alpha, 0)
im_superposed_image_p = cv2.addWeighted(im_color, 1 - alpha, scaled_image_p, alpha, 0)

## Visualizamos resultados
# Crear subplots
fig, axes = plt.subplots(1, 2, figsize=(15, 5))

# Mostrar la primera imagen en el primer subplot
axes[0].imshow(im_superposed_image)
axes[0].set_title('Im Prof Superpuesta')
axes[0].axis('off')

# Mostrar la segunda imagen en el segundo subplot
axes[1].imshow(im_superposed_image_p)
axes[1].set_title('Im Prof Esc Superpuesta')
axes[1].axis('off')

plt.suptitle('Comparativa Imágenes Profundidad')

# Ajustar el espacio entre subplots
plt.tight_layout()

# Mostrar los subplots
plt.show()

imagen_enmascarada_color = cv2.bitwise_and(im_color, im_color, mask=mask_image)
imagen_enmascarada_depth = cv2.bitwise_and(im_depth, im_depth, mask=mask_image)
imagen_enmascarada_scaled = cv2.bitwise_and(scaled_image_p, scaled_image_p, mask=mask_image)

## Visualizamos resultados
# Crear subplots
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

# Mostrar la primera imagen en el primer subplot
axes[0].imshow(imagen_enmascarada_color)
axes[0].set_title('Im Enm Color')
axes[0].axis('off')

# Mostrar la segunda imagen en el segundo subplot
axes[1].imshow(imagen_enmascarada_depth)
axes[1].set_title('Im Enm Profundidad')
axes[1].axis('off')

# Superponer las imágenes en el tercer subplot
axes[2].imshow(imagen_enmascarada_scaled)
axes[2].set_title('Im Enm Prof Escalada')
axes[2].axis('off')

plt.suptitle('Comparativa Imágenes Enmascaradas')

# Ajustar el espacio entre subplots
plt.tight_layout()

# Mostrar los subplots
plt.show()

# Obtenemos la máscara en formato imagen y debemos convertir los números a un formato determinado
imagen_gris = cv2.cvtColor(imagen_enmascarada_scaled, cv2.COLOR_BGR2GRAY)
umbral=150
imagen_gris[imagen_gris >= umbral] = 0
imagen_gris[(imagen_gris < umbral) & (imagen_gris > 0)] = 255
kernel = np.ones((3, 3), np.uint8)
imagen_gris = cv2.erode(imagen_gris, kernel, iterations=10)
imagen_gris = cv2.dilate(imagen_gris, kernel, iterations=10)

cv2.imshow('Punto central objeto', imagen_gris)
cv2.waitKey(0)
cv2.destroyAllWindows()

#v=cv2.convertScaleAbs(imagen_enmascarada_scaled)

# Obtenemos el contorno de la máscara
contornos,hierarchy = cv2.findContours(imagen_gris, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
print(contornos)

cnt=contornos[0]

# Calculamos el momento para posteriormente utilizarlo en el cálculo de centroide, área y perímetro
M=cv2.moments(cnt)

# Calculamos centroide
cX=int(M['m10']/M['m00'])
cY=int(M['m01']/M['m00'])

punto_central_objeto = [cX, cY, int(np.mean(profundidad[cY][cX]))]
print(punto_central_objeto)

cv2.circle(scaled_image_p, (cX, cY), 1, (0, 0, 255), -1)
cv2.imshow('Punto central objeto', imagen_gris)
cv2.waitKey(0)
cv2.destroyAllWindows()