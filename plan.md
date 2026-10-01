# Plan: de robótica social a robótica agrícola (Husky A300 en Webots)

Objetivo: montar un robot agrícola (Husky A300) en un entorno agrícola en Webots, con su bridge
a RoboComp, localización por GPS y, después, navegación autónoma y reconocimiento de humanos y de
sus tareas agrícolas. Se reutilizan los sensores del Shadow: la cámara ZED y el LiDAR `helios`.

## Viabilidad por partes

| Parte | Viabilidad | Por qué |
|---|---|---|
| **Husky A300 en Webots** | ✅ Media-alta | Webots R2025a **no trae el Husky** (en `projects/robots/clearpath` solo están heron, moose y pr2). Hay dos caminos: pasar el URDF oficial de Clearpath (paquete `clearpath_common`, que usa xacro) a Webots con `urdf2webots`, o escribir un PROTO propio con primitivas: chasis, 4 ruedas y mástil de sensores. Se elige el **PROTO propio**, con las medidas del A300 y, si se quiere, la malla del URDF solo para verse. Da más control sobre las físicas y evita depender de ROS 2. |
| **Bridge** | ✅ Alta | En `webots-bridge/src/specificworker.cpp` casi todo es genérico: busca los dispositivos por nombre (`helios`, `zed`, `zed-ranger`, `accelerometer`, `gyro`). Si el PROTO del Husky usa **esos mismos nombres**, los drivers (`lidar3d_dds`, `zed_camera`, `imu_dds`) funcionan sin cambios. Faltan tres cosas: (1) el nombre `DEF "shadow"` está escrito a mano en el código y hay que pasarlo a la configuración; (2) una cinemática **skid-steer** de 4 motores, que es casi igual que la `Differential` que ya existe; (3) el **GPS**. |
| **Sensores del Shadow** | ✅ Alta | Se pueden copiar la ZED (`zed` + `zed-ranger`) y el LiDAR `helios` directamente del `Shadow.proto`. Un aviso: el helios tiene 82 capas, 180° de campo vertical y 100 m de alcance, y en un mundo exterior grande pesa mucho. Conviene bajarlo a unas 32 capas y unos 30 m, montado en un mástil. |
| **Entorno agrícola** | ⚠️ Hay que rehacerlo | `WORLD/worlds/Farm.wbt` usa ahora `UnevenTerrain 200×200×10`, es decir, **10 m de desnivel**, algo imposible de navegar. Además solo tiene palmeras sueltas. Hay que generar el campo con un script paramétrico. |
| **GPS** | ✅ Alta | Webots tiene el dispositivo `GPS` (con ruido) y `WorldInfo.gpsCoordinateSystem "WGS84"`/`gpsReference`, y RoboComp ya tiene `GPS.idsl`. Además, el diseño de `active_inference/field_concept.md` ya resuelve cómo encajar la pose en el grafo DSR para que el controller y el residual funcionen **sin tocar su código**. Solo cambia una cosa: la fuente debe ser el GPS simulado junto con la IMU y la odometría, no la GT con ruido añadido. |
| **Navegación autónoma** | ⚠️ Media | El `grid_planner` toma el polígono de la habitación como límite y el residual detecta obstáculos. Entre hileras funciona si las hileras se declaran en el grafo como obstáculos conocidos. Si solo se detectan con el LiDAR, el follaje genera ruido. |
| **Humanos y sus tareas** | ⚠️ Baja-media | `human_concept` espera esqueletos BODY_18, que en el robot real salen del ZED SDK, y ese SDK no existe en simulación. Hace falta otra fuente de esqueletos: un estimador de pose ONNX sobre la imagen de la ZED, o la GT del supervisor. El reconocimiento de **tareas** (cosechar, cargar, podar…) no existe todavía. Además, los humanos de Webots solo caminan, así que para tareas agrícolas hacen falta animaciones propias (`skin_animated_humans` con BVH). |

## Tamaño de los cultivos según el robot

Las medidas del Husky A300 son aproximadas y hay que confirmarlas con el URDF de Clearpath: unos
0,99 × 0,70 m, vía de unos 0,55 m y ruedas de unos 330 mm de diámetro. Con eso, el punto de partida:

- **Separación entre hileras:** 1,2 m. Deja al robot unos 0,70 m y unos 0,25 m de margen por lado.
- **Ancho de la planta:** 0,4 m como máximo.
- **Altura de la planta:** entre 0,3 y 0,8 m. Una fila de 0,6 m queda por debajo del LiDAR del
  mástil y dentro del campo de la ZED.
- **Cabeceras para girar:** al menos 3 m.
- **Terreno:** llano o con ondulación suave, por debajo de 0,3 m.
- **Campo inicial:** 30 × 50 m. El planner construye la rejilla sobre el rectángulo que rodea el
  polígono, así que un campo enorme no conviene.

Todo esto debe quedar como parámetros del generador del mundo.

## Plan de trabajo (en orden)

### F0. Preparación
1. Dejar el stack Shadow actual compilando y arrancando como referencia. El README exige rutas
   `~/robocomp/components/...`.
2. Crear un paquete `webots-agri/` (protos, mundos, controladores) y mover ahí `WORLD/`.

### F1. Robot Husky A300
3. Hacer el `HuskyA300.proto`: chasis, 4 ruedas skid-steer (`wheel_fl/fr/rl/rr` con sensores de
   posición), masas e inercias realistas y fricción de rueda sobre tierra.
4. Montar los sensores con los nombres del Shadow: `helios` en el mástil (unos 0,8–1,0 m), `zed` y
   `zed-ranger` delante (unos 0,6 m, algo inclinada hacia abajo), `accelerometer` y `gyro`. Añadir
   `gps` e `InertialUnit` o `Compass` para el rumbo.
5. Crear un mundo de prueba llano con el Husky para validar teleoperación y físicas.

### F2. Bridge del Husky
6. Parametrizar el bridge actual (o copiarlo como `webots-husky-bridge`): `Robot.Def` configurable
   y `Base.Kinematics = "skid"` con 4 motores (izquierdos y derechos agrupados), más radio y vía
   propios.
7. Añadir la interfaz GPS. Implementar `GPS.idsl` (lat/lon/alt, UTM y x/y locales) y regenerar con
   `robocompdsl`. **Ojo:** `generated/main.cpp` y `doorcontrolI.*` tienen cambios locales que hay
   que volver a aplicar después.
8. Crear `sub_husky.toml` (bridge, helios, zed, imu, joystick) y los ficheros de calibración de
   montaje (`mount_calib_Husky.txt`, `husky.json` para `robot_concept`).
9. Validar que lidar, cámara, IMU, odometría y GPS llegan a DDS/Ice y que el joystick mueve el
   robot.

### F3. Entorno agrícola
10. Hacer un `Crop.proto` o reutilizar las plantas de Webots: una planta con varias variantes de
    altura y aspecto y colisión simplificada, para no hundir el rendimiento.
11. Escribir un generador Python `gen_field.py` que produzca `Field.wbt` a partir de hileras,
    separación, largo, cabeceras, huecos, ruido de posición y tipo de cultivo.
12. Configurar `WorldInfo`: `gpsCoordinateSystem "WGS84"`, `gpsReference` (por ejemplo, cerca de
    la UEx en Badajoz: unos 38,88 N y 6,97 O) y `northDirection`. Usar terreno suave, caminos,
    lindes y cielo exterior.
13. Medir el rendimiento: el tiempo real del simulador debe mantenerse por encima de 0,5x con
    todos los sensores activos.

### F4. Localización por GPS
14. Implementar `field_concept` siguiendo `active_inference/field_concept.md`, con una fuente
    distinta: GPS (1–10 Hz, con ruido) + IMU/brújula (θ) + odometría, fusionados en un EKF
    sencillo que convierte WGS84 a ENU local.
15. Publicar en el DSR el nodo `room` (polígono igual al del campo) y la arista RT `robot→room`
    con una covarianza honesta.
16. Cambiar `required_agent_names` en el controller y el residual para usar `field_concept`.
17. Validar comparando con la GT. En este caso sí es una evaluación real, porque la pose ya no
    sale de la GT.

### F5. Navegación autónoma
18. Crear un `crop_row_concept` que publique las hileras en el DSR como obstáculos conocidos, bien
    desde el mapa del generador o bien detectándolas con el LiDAR. Así el residual no las confunde
    con obstáculos nuevos.
19. Crear misiones de campo: waypoints GPS y recorrido en boustrophedon (ir y volver fila por
    fila, girando en las cabeceras).
20. Revisar el controller: límites de velocidad del Husky, cómo ejecuta una base skid-steer las
    órdenes laterales y la afordancia `afford_room`, que `field_concept.md` marca como pendiente
    de comprobar.

### F6. Humanos en el campo
21. Crear trabajadores agrícolas animados (`skin_animated_humans` con BVH de agacharse/cosechar,
    cargar caja, caminar, estar de pie) usando la convención `HUMAN_*` que ya lee el bridge.
22. Crear una fuente de esqueletos para `human_concept`: primero la GT del supervisor, para depurar
    el pipeline; después, un estimador de pose ONNX sobre la ZED más la profundidad, para que sea
    realista.

### F7. Reconocimiento de tareas agrícolas
23. Generar un dataset en simulación con secuencias de esqueleto etiquetadas por tarea (el
    generador lo produce gratis).
24. Hacer un clasificador temporal (LSTM/TCN; revisar `active_inference/ltsm_agent/`) que escriba
    en el DSR la tarea del humano y alimente la navegación: no entrar en una hilera ocupada o
    acercarse a un trabajador que está cargando.

## Riesgos principales

- La pila `active_inference` está muy orientada a interiores: paredes, muebles, `room_concept`.
  Los conceptos de cocina no sirven en el campo; hay que decidir qué agentes se lanzan en
  `cognitive.toml`.
- El rendimiento en exteriores, por el LiDAR denso y cientos de plantas con colisión.
- En simulación no hay ZED SDK, así que el seguimiento de humanos exige un estimador propio.
- `robocompdsl` pisa `generated/`, así que hay que guardar los cambios locales antes de regenerar.

## Estado

| Fase | Estado |
|---|---|
| F0 | pendiente (`WORLD/` sigue en su sitio; `webots-agri/` ya creado) |
| F1 | ✅ hecho (falta confirmar medidas con el URDF del A300) |
| F2 | ✅ hecho (incluido `husky.json` y `robot_concept` para el Husky) |
| F3 | pendiente |
| F4 | ✅ `openfield_concept` funcionando (falta afinar covarianza y conectar controller/residual en F5) |
| F5–F7 | pendiente |

### F1 — hecho (2026-09-30)
- `webots-agri/protos/HuskyA300.proto`: skid-steer de 4 ruedas (`front_left`, `rear_left`,
  `front_right`, `rear_right`), mismas convenciones que el Shadow (+Y adelante, +X derecha, eje de
  rueda -X, origen en el suelo). Sensores: `helios` (50 capas / 110°, 30 m, mástil a 0,95 m), `zed`
  + `zed-ranger` (0,55 m, 15° hacia abajo), `accelerometer`, `gyro`, `inertial unit`, `compass`,
  `gps` (ruido correlacionado configurable: `gpsAccuracy`, `gpsNoiseCorrelation`).
- `webots-agri/worlds/husky_test.wbt`: banco de pruebas llano, `gpsCoordinateSystem "WGS84"`,
  referencia cerca de la UEx (38.8794, -6.9707), norte = +Y.
- `webots-agri/controllers/husky_selftest/`: prueba automática (avance, pivote, arco; odometría vs
  GT; todos los sensores). Resultado: avance exacto; el skid-steer **pierde ~41 % del giro** por
  arrastre (pivote: factor 1,70; arco a 0,6 m/s: 2,27).
- Hallazgo: Webots convierte local→WGS84 sumando el desplazamiento en **coordenadas UTM** de la
  referencia. Por eso el marco local correcto es `UTM(lat,lon) − UTM(ref)` (error < 1 mm); una
  proyección tangente simple da 12 cm de error a 5 m. Importante para F4.

### F2 — hecho (2026-09-30)
- Bridge (`webots-bridge`):
  - `Robot.Def` configurable (antes `"shadow"` fijo).
  - `Base.Kinematics = "skid"` + `Base.TrackScale` (multiplicador de vía efectiva, como el
    `wheel_separation_multiplier` de Clearpath), aplicado en IK y en la odometría de ruedas.
  - Interfaz **GPS** (`RoboCompGPS`, puerto 10009): `getPos` (m, x = este, y = norte, = marco del
    mundo Webots), `getData` (lat/lon/alt), `getUTMData`, `resetPos`. Añadida a mano en
    `generated/` como `DoorControl` y al `.cdsl`; es opcional (sin `Endpoints.GPS` no se sirve).
  - IMU: orientación desde el `InertialUnit` (con ruido y varianza nominal) y campo magnético desde
    el `Compass` cuando existen; el Shadow sigue igual.
  - `etc/config_husky`.
- Drivers: `lidar3d_dds/etc/config_helios_husky_webots.toml` (self-filter con mesh
  `robots/HuskyA300/husky_a300.stl`, generado por `webots-agri/tools/gen_husky_selffilter_stl.py`;
  `mesh_filter.cpp` ahora acepta un mesh único para cualquier robot), `zed_camera/etc/configHuskyWebot.toml`.
- `active_inference/utils/sub_husky.toml`: todas las rutas `cwd` apuntan a **este repo**
  (`~/robocomp/AiGB_UEx_Navigation/...`), no a `~/robocomp/components` como `sub_shadow.toml`.
  Drivers compilados en su sitio (`lidar3d_dds`, `imu_dds`, `zed_camera`, `python_xbox_controller`);
  `robocomp-robolab/.gitignore` ignora sus `bin/`, `build/` y `.ice` generados.
- `zed_camera`: el ZED SDK pasa a ser opcional. Sin SDK compila en modo **solo simulación**
  (`Config.Simulated = true`); con SDK, igual que antes. En esta máquina no hay SDK ni CUDA.
- Prueba de extremo a extremo (Webots + IceStorm + bridge + cliente Ice): GPS, IMU, brújula y base
  responden; 0,5 m/s × 4 s → 1,83 m; 0,5 rad/s × 4 s → 2,18 rad (sobregiro del 9 % con
  `TrackScale = 1.875`, coherente con el 1,70 medido).

### Pendiente de F1/F2
- Confirmar medidas y masa del A300 con `clearpath_common` (a300.urdf.xacro) y ajustar los campos
  del PROTO y del STL de self-filter (el script tiene las mismas constantes).
- Medir el ruido de odometría del Husky parado (`SensorNoise.*Floor` viene del P3Bot).
- En la prueba del helios, el driver bajó a ~2 Hz tras unos segundos: revisar si es la hibernación
  del bridge o el ritmo de la simulación con `--no-rendering`.
- Límites de velocidad del joystick (`config_shadow`) frente a los 2 m/s del Husky.

### F2 (cont.) — el Husky en el DSR (2026-09-30)
- `active_inference/robot_concept/husky.json`: `root → husky (robot) → body → helios / zed / imu / gps`
  (tipo `gps`, ya registrado en cortex) + `mind`; montajes = campos del PROTO.
- `robot_concept/etc/config_husky.toml` + `etc/base_husky.toml` (capacidad de la base: diferencial,
  1 m/s, 1,5 rad/s, R y vía del Husky) + `meshes/husky_a300.obj` (generado por
  `webots-agri/tools/gen_husky_selffilter_stl.py`, el mismo que el STL del self-filter).
- El helios lee su montaje de `husky.json` (`mount_file`). Corregido el valor de reserva `ry = π`
  heredado del Shadow, que habría invertido la nube.
- Probado: ZED y helios ~17 Hz, IMU ~121 Hz, odometría ~11 Hz; cuerpo medido del mesh 0,685 × 0,99 m.

### F4 — `openfield_concept` (2026-09-30)
- Agente DSR nuevo (`active_inference/openfield_concept`, **id 25**; el 18 y el 19 estaban en uso por
  viewer3d y ltsm_agent sin figurar en la tabla de IDs de `CONCEPT_AGENT_RECIPE.md`: añadidos).
- EKF SE(2) `[x, y, θ]` (`src/openfield_ekf.{h,cpp}`, solo Eigen, con `self_test()`):
  predicción con avance de ruedas + giro del **giroscopio** (las ruedas del skid-steer sobreestiman el
  giro); corrección con GPS (antena con brazo de palanca) y yaw absoluto de la IMU. Ruido de proceso
  como densidad que crece con la velocidad, sin umbrales.
- Publica el mismo contrato que `room_concept` (`src/openfield_scene_graph.cpp`, portado de
  `RoomSceneGraph::write_robot_room_rt`): nodo `room` con el polígono del campo + arista RT
  `robot→room` con covarianza en los índices 0/1/5 y twist del hijo.
- Límite del campo en config: metros locales (`Field.PolygonX/Y`) o coordenadas GPS
  (`Field.PolygonLat/Lon`, convertidas a UTM como el bridge).
- Protocolo de presencia compartido (`common/concept_presence`); entrada principal = IMU; apagado
  limpio (borra su nodo y el `room`).
- `utils/cognitive_husky.toml`: `robot_concept` + `openfield_concept`.
- **Resultado** (husky_test.wbt, recta 0,6 m/s + pivote 0,5 rad/s + arco, error frente a la GT al
  instante de la GT): posición p50 2,2 cm / p95 5,0 cm / máx 6,8 cm; rumbo p50 0,07° / máx 0,19°.
  50 Hz, 1–2 % de CPU.

### Pendiente de F4
- La covarianza publicada es ~2× optimista en posición (σ ≈ 1 cm frente a error p50 2,2 cm): el ruido
  del GPS de Webots está correlacionado (`noiseCorrelation 0.9`) y el EKF lo trata como blanco.
  Opciones: subir `Ekf.GpsSigmaM` o modelar el sesgo del GPS como estado (Gauss-Markov).
- El interfaz `RoboCompGPS` no lleva marca de tiempo: cada fix se aplica al recibirlo (≤ 50 ms de
  sondeo + 100 ms de periodo del receptor). Suficiente a 1 m/s con RTK; revisar si se sube la velocidad.
- Conectar `controller` y `residual_concept` (`required_agent_names` → `openfield_concept`), F5.
