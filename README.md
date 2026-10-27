# RoboArm 3D - Controlador de Brazo Robotico

Sistema de visualizacion y control 3D interactivo para un brazo robotico de 3 grados de libertad (DOF) con motores NEMA 17.

**Demo en vivo:** https://antoniosanchezcaba.github.io/3DModelRobot/

## Caracteristicas

### Visualizacion 3D
- Renderizado en tiempo real con Three.js (r128)
- Modelo 3D del brazo generado a partir de las longitudes L1, L2 y L3
- Multiples vistas de camara (frontal, superior, isometrica)
- Controles de orbita para rotacion y zoom
- Grilla, ejes de referencia y trayectoria del efector configurables

### Control de Articulaciones
- **Base (θ₁)**: Rotacion -180° a +180°
- **Hombro (θ₂)**: Rotacion -90° a +90°
- **Codo (θ₃)**: Rotacion -135° a +135°
- Deslizadores, campos numericos y botones de angulos rapidos
- **Arrastre directo**: arrastra la esfera verde (efector final) o la azul (codo) con el raton o con el dedo
  - Amarillo: el brazo sigue al cursor
  - Rojo: el brazo esta en el limite (alcance, limites articulares o suelo) y sigue al cursor hasta donde puede

### Restricciones fisicas
- Ninguna parte del brazo puede quedar por debajo del suelo (Z = 0)
- Los movimientos animados (presets, posiciones guardadas, IK) siguen un camino que nunca atraviesa el suelo: si es necesario, el hombro se eleva primero
- Los limites articulares se respetan en todos los modos de control

### Posiciones Guardadas
- Posiciones predefinidas: Home, Pick y Place
- **Guardar** la posicion actual con un nombre propio (se guarda en el navegador y sobrevive a recargar la pagina)
- Mover a una posicion guardada con un clic, renombrarla o eliminarla
- Guardar con un nombre existente reemplaza esa posicion (se pide confirmacion)
- **Exportar / Importar** las posiciones como archivo JSON (incluye los pasos de cada motor)

### Cinematica
- **Cinematica Directa**: posicion del efector final a partir de los angulos articulares
- **Cinematica Inversa**: angulos necesarios para alcanzar una posicion objetivo (hasta 4 soluciones; se elige la valida mas cercana a la pose actual)
- **Parametros DH**: tabla Denavit-Hartenberg del brazo

### Motores NEMA 17
- Pasos por revolucion y microstepping (1, 1/2, 1/4, 1/8, 1/16, 1/32)
- Posicion de cada motor en pasos desde Home, en tiempo real

## Estructura del Proyecto

```
3DModelRobot/
├── index.html              # Pagina principal
├── css/
│   └── styles.css          # Estilos de la interfaz
├── js/
│   ├── kinematics.js       # Cinematica directa/inversa, suelo, planificacion de trayectorias
│   ├── positions.js        # Posiciones guardadas con nombre (localStorage, import/export)
│   ├── ui.js               # Controlador de interfaz
│   └── main.js             # Escena Three.js, modelo del brazo y arrastre
├── tests/
│   ├── unit/               # Pruebas de cinematica y posiciones (node --test)
│   └── e2e/                # Pruebas en navegador (Playwright)
├── server.py               # Servidor local de desarrollo
└── README.md
```

## Uso

1. Abrir la demo en vivo, o ejecutar `python3 server.py` y abrir http://localhost:8000
2. Mover las articulaciones con los deslizadores o arrastrando las esferas del brazo
3. Guardar posiciones con **Guardar** y volver a ellas desde la lista
4. Usar la pestaña Cinematica para calcular y aplicar la cinematica inversa

### Controles de Teclado/Raton
- **Arrastrar una esfera**: mover el brazo
- **Click + Arrastrar** (fuera del brazo): rotar vista
- **Scroll**: Zoom
- **Click derecho + Arrastrar**: Desplazar vista

## Pruebas

```bash
npm test            # pruebas unitarias (Node 18+, sin dependencias)
npm run test:e2e    # pruebas en navegador (requiere: npm install && npx playwright install chromium)
```

## Parametros del Robot

| Parametro | Valor | Descripcion |
|-----------|-------|-------------|
| L1 | 50mm | Altura del hombro sobre la base |
| L2 | 100mm | Longitud del brazo superior |
| L3 | 80mm | Longitud del antebrazo |

## Formulas de Cinematica

Convenciones: θ₂ se mide desde la horizontal (+ hacia arriba); θ₃ es relativo al brazo superior (0 = brazo estirado, + = el antebrazo se pliega hacia arriba).

### Cinematica Directa
```
r = L2·cos(θ₂) + L3·cos(θ₂ + θ₃)
x = r·cos(θ₁)
y = r·sin(θ₁)
z = L1 + L2·sin(θ₂) + L3·sin(θ₂ + θ₃)
```

### Cinematica Inversa
Con r = √(x² + y²) y D = √(r² + (z - L1)²):

1. **θ₁** = atan2(y, x)  (o θ₁ + 180° para alcanzar por detras de la base)
2. **θ₃** = ± acos((D² - L2² - L3²) / (2·L2·L3))
3. **θ₂** = atan2(z - L1, r) - atan2(L3·sin(θ₃), L2 + L3·cos(θ₃))

Se descartan las soluciones fuera de los limites articulares o con el codo o el efector bajo el suelo, y se elige la mas cercana a la pose actual.

## Despliegue

La aplicacion es estatica (sin compilacion) y se publica con GitHub Pages directamente desde la rama. Al publicar una nueva version, actualiza el parametro `?v=` de los archivos CSS/JS en `index.html` para que los navegadores no usen copias antiguas en cache.

## Tecnologias Utilizadas

- **Three.js**: Renderizado 3D WebGL
- **OrbitControls**: Controles de camara
- **CSS3**: Animaciones y estilos modernos
- **JavaScript ES6+**: Logica de aplicacion

## Compatibilidad

- Chrome 80+
- Firefox 75+
- Safari 13.1+
- Edge 80+

## Licencia

MIT License
