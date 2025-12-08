# RoboArm 3D - Controlador de Brazo Robotico

Sistema de visualizacion y control 3D interactivo para un brazo robotico de 3 grados de libertad (DOF) con motores NEMA 17.

## Caracteristicas

### Visualizacion 3D
- Renderizado en tiempo real con Three.js
- Carga del modelo STL del brazo robotico
- Multiples vistas de camara (frontal, superior, isometrica)
- Controles de orbita para rotacion y zoom
- Grilla y ejes de referencia configurables

### Control de Articulaciones
- **Base (θ₁)**: Rotacion -180° a +180°
- **Hombro (θ₂)**: Rotacion -90° a +90°
- **Codo (θ₃)**: Rotacion -135° a +135°

### Cinematica
- **Cinematica Directa**: Calcula la posicion del efector final a partir de los angulos articulares
- **Cinematica Inversa**: Calcula los angulos necesarios para alcanzar una posicion objetivo
- **Parametros DH**: Visualizacion de parametros Denavit-Hartenberg
- **Jacobiano**: Calculo de velocidades articulares

### Configuracion de Motores
- Soporte para motores NEMA 17
- Configuracion de pasos por revolucion
- Microstepping (1, 1/2, 1/4, 1/8, 1/16, 1/32)
- Calculo de pasos de motor para movimientos

### Interfaz de Usuario
- Diseno moderno y profesional
- Tema oscuro
- Controles deslizantes para cada articulacion
- Botones de acceso rapido a angulos comunes
- Posiciones predefinidas (Home, Pick, Place)
- Notificaciones toast
- Display de posicion en tiempo real

## Estructura del Proyecto

```
3DModelRobot/
├── index.html          # Pagina principal
├── assembly.stl        # Modelo 3D del brazo robotico
├── css/
│   └── styles.css      # Estilos de la interfaz
├── js/
│   ├── main.js         # Aplicacion principal Three.js
│   ├── kinematics.js   # Modulo de cinematica
│   └── ui.js           # Controlador de interfaz
└── README.md
```

## Uso

1. Abrir `index.html` en un navegador web moderno
2. Usar los controles deslizantes para mover las articulaciones
3. Alternar entre vistas en el panel de control
4. Usar cinematica inversa para mover a posiciones especificas

### Controles de Teclado/Raton
- **Click + Arrastrar**: Rotar vista
- **Scroll**: Zoom
- **Click derecho + Arrastrar**: Desplazar vista

## Parametros del Robot

| Parametro | Valor | Descripcion |
|-----------|-------|-------------|
| L1 | 50mm | Altura de la base |
| L2 | 100mm | Longitud del brazo superior |
| L3 | 80mm | Longitud del antebrazo |

## Formulas de Cinematica

### Cinematica Directa (DH)
La posicion del efector final se calcula usando matrices de transformacion Denavit-Hartenberg:

```
T = T01 * T12 * T23
```

Donde cada matriz Tij representa la transformacion entre articulaciones.

### Cinematica Inversa
Para un brazo de 3 DOF, se utilizan soluciones geometricas:

1. **θ₁** = atan2(y, x)
2. **θ₃** = acos((L2² + L3² - D²) / (2·L2·L3))
3. **θ₂** = atan2(z-L1, r) + atan2(L3·sin(θ₃), L2 + L3·cos(θ₃))

## Tecnologias Utilizadas

- **Three.js**: Renderizado 3D WebGL
- **STLLoader**: Carga de modelos STL
- **OrbitControls**: Controles de camara
- **CSS3**: Animaciones y estilos modernos
- **JavaScript ES6+**: Logica de aplicacion

## Compatibilidad

- Chrome 80+
- Firefox 75+
- Safari 13+
- Edge 80+

## Licencia

MIT License
