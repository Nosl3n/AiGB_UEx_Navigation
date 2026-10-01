# Bitácora

Diario de la sesión de trabajo (30-09-2026 → 01-10-2026) que llevó el stack del Shadow a un Husky agrícola.
Va en orden cronológico. En cada etapa: qué se pidió, qué se hizo, qué falló, cómo se arregló y qué se midió.
El resumen por archivos está en [`cambios.md`](cambios.md); los límites en
[`cosas_a_tener_en_cuenta.md`](cosas_a_tener_en_cuenta.md).

---

## 1. Viabilidad y plan

- **Petición**: pasar el repo a robótica agrícola con un Husky A300 en Webots, cultivos a su escala, ZED y LiDAR del
  Shadow, localización GPS y, después, navegación autónoma y reconocimiento de personas y tareas.
- **Análisis**: el bridge y los drivers son casi genéricos (buscan los dispositivos por nombre). Webots R2025a no
  trae el Husky: mejor un PROTO propio que convertir el URDF. La pila cognitiva está pensada para interiores
  (marco `room`).
- **Resultado**: [`plan.md`](plan.md) con fases F0–F7.

## 2. F1–F2: el Husky en Webots y su bridge

- **PROTO `HuskyA300`** con medidas del A300 (radio 0,1651 m, vía 0,5708 m, batalla 0,512 m), 4 motores, helios,
  ZED, IMU, brújula y GPS.
- **Bridge**: nombre del robot por configuración, cinemática skid-steer y servidor GPS nuevo (WGS84 → UTM).
- **Problemas de prueba resueltos**:
  - `pkill -f` mataba su propia shell y dejaba Webots zombi → los procesos se paran por PID registrado;
  - un script llamado `inspect.py` tapaba el módulo estándar de Python ("agent already connected");
  - la coma decimal del locale español rompía `printf` → `LC_ALL=C`.
- **Mando Xbox**: no respondía porque apuntaba al puerto 1237 de IceStorm en vez del 9999. Arreglado; el giro va al
  stick derecho (eje 3), que permite avanzar recto con el izquierdo.
- **Odometría skid-steer**: el arrastre lateral de las ruedas hace que la rotación por ruedas sea mayor que la
  real (medido: ×1,70 en pivote, ×2,27 en arco). Se usa `TrackScale 1.875`, como Clearpath.

## 3. F2 (cont.): el Husky en el grafo DSR

- `husky.json` con el árbol de frames (husky → body → helios/zed/imu/gps), malla OBJ y configuración de
  `robot_concept`. Lanzadores `sub_husky.toml` y `cognitive_husky.toml` con rutas dentro de este repo.

## 4. F4: `openfield_concept`, localización por GPS

- **Petición**: un agente que sustituya a `room_concept`, alimentado por GPS y trabajando junto con la odometría
  de ruedas y la IMU. Se eligió un **EKF**; polígono del campo en metros locales o en puntos GPS.
- **Hecho**: EKF SE(2) con predicción por ruedas + giróscopo y corrección por GPS (con brazo de palanca de la
  antena) + yaw absoluto de la IMU. Ruido de proceso que crece con la velocidad. Publica el nodo `field`.
- **Decisión**: el marco del mundo se llama `field`, no `room`. Se registró el tipo `field` en cortex (aprobado
  por el usuario) y se creó `common/world_frame` para que todos los agentes encuentren `field` o, si no, `room`.
- **Medido**: error de posición **2,2 cm p50** frente a la verdad de la simulación.
- **Arreglos**:
  - la IMU dejaba de llegar al cambiar el descriptor del plano de medios → se re-suscribe si calla 1 s;
  - el error se evaluaba contra la verdad de otro instante → ahora se compara en el sello de tiempo de la medida.

## 5. F3: hileras de palmeras

- `YoungPalm.proto` (palmera joven procedural) y `gen_palm_rows.py`: **5 hileras × 10 palmeras**, separación
  calculada desde el ancho del robot (pasillo libre de 1,3 m para 0,69 m de robot).

## 6. F5: navegación autónoma (controller + residual)

- controller y residual pasan a usar el marco `field`. El objetivo se manda con `send_goal.py` (la arista `target`
  no existe en cortex; se usa `goto_action`).
- **Auto-retornos del LiDAR** (puntos del propio robot tomados como obstáculo):
  - **causa principal**: `voxel_downsample` de la ZED en residual sumaba sobre memoria sin inicializar y creaba
    puntos a 30 m y de 10³¹ m;
  - el auto-filtro era un disco: ahora tiene la **forma real del robot** (caja orientada) y la malla del Husky en
    `lidar3d_dds`.
- **Ruido del LiDAR mal entendido**: en Webots `noise` es relativo al alcance máximo. El `0.005` heredado era
  σ = 15 cm en el Husky. Ahora el PROTO se configura en metros (2 cm, como un RS-Helios real); igual con la IMU.
- **Filtrado de campo**: residual solo marca cosas con muchos puntos y altura (umbral de suelo 0,25 m, extensión
  vertical mínima 0,30 m, grupos mínimos), y `CeilZ` a la altura del robot.
- **Resultado**: el robot cruza el pasillo de palmeras hasta el objetivo de forma autónoma.

## 7. CPU y memoria del driver del LiDAR

- **Diagnóstico** (instantáneas de hilos con gdb): 72 hilos (OpenMP 31 + TBB/Embree 31). La mitad del tiempo
  estaba en espera activa de OpenMP entre escaneos.
- **Arreglo**: `Threads.Count = 4` (OpenMP + Embree) y `OMP_WAIT_POLICY=PASSIVE` en el lanzador.
- **Medido**: hilos 72 → 23; **CPU ~1200 % → 17 %** a la misma frecuencia (17 Hz); RSS estable en ~97 MB.

## 8. F4 (cont.): ventana de localización

- Ventana Qt de `openfield_concept`: mapa con trayectorias (azul con GPS, rojo sin GPS), elipse 2σ, gráfica de
  error frente a la verdad y casillas para apagar el GPS o el yaw de la IMU.
- **Demostrado**: sin GPS el error crece por navegación a estima (hasta 4 m con el robot atascado) y vuelve a
  centímetros al encender el GPS. Hallazgo: el σ del filtro es demasiado optimista cuando las ruedas patinan.
- **Arreglo posterior**: "no se localiza" → la IMU había dejado de llegar; resuelto con la re-suscripción.

## 9. F6: personas (retina + human_concept)

- **retina**: modelos YOLO26 de segmentación y pose (ONNX + CUDA) copiados al repo (sin versionar);
  configuración del Husky solo con la ZED y clases del campo.
- **human_concept no creaba ninguna persona**: el fitter estaba declarado pero nunca se construía. Arreglado.
- **Identidades**: los ids de esqueleto de retina son índices por frame; dos personas se intercambiaban y el
  ajuste quedaba entre ambas. Ahora se asocian por cercanía en el marco del mundo.
- **Demasiadas personas** (35 nodos para 2 trabajadores): el contador de ausencia solo avanzaba con datos nuevos,
  así que una persona que dejaba de verse no se borraba nunca. Primero: vida de 2 s sin verla.
- **El LiDAR como "segunda mirada"** (lo que en el Shadow hacía la cámara 360):
  - existencia por evidencia (cámara + rayos del LiDAR en una banda de torso por encima de las palmeras);
  - seguimiento con LiDAR cuando la cámara la pierde, que mantiene el id;
  - ajustes medidos en prueba: banda subida a 1,20 m (las hojas creaban un fantasma), caja tallada donde el
    LiDAR ve el torso (la cámara sitúa a la persona 0,15–0,3 m desviada a 12 m), 1 punto basta (entre 11 y 13 m
    solo llegan ~2 puntos por el hueco entre anillos), esqueletos duplicados descartados.
- **Resultado**: con el robot de espaldas 20 s, la persona conserva **el mismo id durante los 80 s** de prueba y el
  LiDAR la sigue caminando. Antes: un id nuevo cada pocos segundos.

## 10. F3 (cont.): suelo de campo

- `FieldTerrain.proto` y `husky_field.wbt`: camellones de 0,20 m bajo las hileras, pasillos, ondulación, terrones
  y textura de tierra marrón. Autotest correcto; patina más en los giros y se sube al camellón si sale del pasillo.

## 11. Documentación

- `teoria_navegacion.md`: navegación LiDAR en campo abierto (alineación con la gravedad, segmentación de suelo,
  mapas de elevación, transitabilidad, hileras de cultivo).
- `cosas_a_tener_en_cuenta.md`, `cambios.md`, esta bitácora y el `README.md` renovado.

---

## Qué mejoró, en números

| | Antes | Después |
|---|---|---|
| Localización | sin localización en exterior | 2,2 cm p50 con GPS |
| CPU del driver LiDAR | ~1200 % | 17 % |
| Personas creadas | 0 (fitter sin construir) → 35 nodos para 2 personas | 1 identidad estable por persona |
| Persona fuera de la cámara | se perdía o se duplicaba | el LiDAR la sigue y conserva el id |
| Puntos del propio robot | obstáculos fantasma | filtrados con la forma real del robot |

## Pendiente (resumen; detalle en `plan.md`)

- Nacimiento de personas más robusto (`instance_tracker`): siguen apareciendo nodos falsos de 2 s durante los giros.
- Persona como obstáculo normal (huella física) y residual cediendo sus puntos.
- Suelo no plano: estimar el suelo local (residual) y la inclinación del robot (EKF), necesario para
  `husky_field.wbt`.
- Mapa local robocéntrico, capas de coste continuas, covarianza de odometría con patinaje.
- F7: reconocimiento de tareas agrícolas.
