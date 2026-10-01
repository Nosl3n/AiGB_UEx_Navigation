# Cosas a tener en cuenta

Restricciones, supuestos y límites conocidos del proyecto. Cada punto dice **qué** es, **por qué** importa y,
cuando aplica, **dónde** está. Mantener al día: un supuesto que se olvida es un fallo que vuelve.

Complementa a `plan.md` (qué se ha hecho y qué falta) y a `teoria_navegacion.md` (fundamento teórico).

---

## 1. Reglas de trabajo

- **Solo se modifica este repositorio** (`AiGB_UEx_Navigation`). Cualquier cambio fuera (p. ej. `~/robocomp/cortex`)
  se pregunta antes. Excepción ya aprobada: el tipo de nodo `field` en cortex
  (`dsr_node_type.h`, `REGISTER_NODE_TYPE(field)`); la reinstalación la hace el usuario con sudo.
- **Los commits los hace el usuario**, nunca el asistente.
- **Respuestas en español.** Comentarios e identificadores del código pueden seguir en inglés (estilo existente).
- **Los `.md` nuevos van en la raíz** de `AiGB_UEx_Navigation`, no en subcarpetas.
- **Antes de lanzar una prueba, comprobar que el stack del usuario no está corriendo** (dos instancias
  comparten el grafo DSR, los puertos Ice y los dominios DDS) y avisar.
- **Parar agentes con SIGINT/SIGTERM, nunca `kill -9`**: los agentes borran sus nodos del grafo al salir; con -9
  quedan nodos huérfanos.
- **Compilar con `make -j8` como máximo**: la máquina no tiene swap y `-j32` dispara el OOM-killer, que mata
  otra cosa (p. ej. el navegador), no el compilador.
- **Locale `es_ES`**: la coma decimal rompe `strtof`/`atof`/`printf` en shell. En C++ leer con `std::from_chars`;
  en scripts de shell usar `LC_ALL=C`.

## 2. Webots y simulación

- **Webots R2025a.** Los PROTO de Webots (Pedestrian, Soil, fondos…) se descargan de GitHub al cargar el mundo:
  sin red la primera carga falla.
- **Plantillas JS de PROTO**: `wbrandom` hay que importarlo (`import * as wbrandom from 'wbrandom.js';`).
  `IndexedFaceSet` no tiene campo `solid`: para que una hoja se vea por ambas caras hay que duplicar la cara con
  el orden invertido.
- **Ruido de sensores en Webots es relativo**:
  - `Lidar.noise` es relativo a `maxRange` → `heliosNoiseM / heliosMaxRange` (HuskyA300.proto).
  - `InertialUnit.noise` es relativo a π/2 → `imuNoiseRad / (π/2)`.
- **GPS en WGS84**: Webots pasa local → lat/lon vía UTM de `gpsReference`; local = UTM(lat,lon) − UTM(ref).
  `northDirection 0 1 0` ⇒ +Y = norte, +X = este (ENU).
- **Humanos (Pedestrian)** no tienen bounding object: el robot los atraviesa físicamente. Son solo para percepción.
  Caminan una trayectoria fija en bucle; en `husky_field.wbt` sus pies no siguen el relieve (altura fija).
- **Palmeras (YoungPalm)**: tronco 0,7 m, copa hasta ~0,95 m, radio ~0,6 m. Solo el tronco tiene colisión; el robot
  puede rozar las hojas. Hileras a 2,5 m ⇒ pasillo libre 1,3 m para un robot de 0,69 m.
- **El objetivo de prueba no debe estar en la trayectoria de los trabajadores** (p. ej. (1,25, 9,5) está en la de
  HUMAN_2): el robot se queda bloqueado esperando.

## 3. Mundo `husky_field.wbt` (suelo de campo)

- Es `husky_test.wbt` (mismas hileras, GPS, robot y trabajadores) sobre `protos/FieldTerrain.proto`:
  camellones de 0,20 m bajo cada hilera, pasillos y cabeceras a z ≈ 0, ondulación ±4 cm (8–15 m) y terrones ±1,2 cm.
- **z = 0 es el nivel del pasillo**, no "el suelo" en general. Varias piezas suponen suelo plano a z = 0
  (ver §5 y §6); en los pasillos sigue siendo casi cierto, sobre los camellones no.
- Las palmeras se colocan a z = 0,14 (`gen_palm_rows.py --z 0.14`): hundidas unos cm en el camellón, porque la
  altura exacta del terreno en cada punto (ondulación + terrones) la calcula el PROTO y el script no la conoce.
- Un giro o un arco fuera del pasillo sube el robot al camellón (autotest: alabeo 20°, z 0,22 m). El robot debe
  ir por los pasillos.
- El patinaje en giro sobre tierra rugosa es mayor que en plano (autotest: pivote 64° reales frente a 115° de
  ruedas): la odometría de ruedas pierde aún más en giros.

## 4. Localización (`openfield_concept`, ID 25)

- **EKF SE(2)** [x, y, θ]: predice con ruedas + giróscopo y corrige con el GPS (brazo de palanca (0, −0,25)) y el
  yaw absoluto de la IMU. **No estima z, alabeo ni cabeceo**: el nodo `field` es un plano. En terreno con relieve
  la pose 3D del robot (y de sus sensores) es incorrecta en z y en inclinación.
- **El σ es demasiado optimista cuando las ruedas patinan** (medido: 4 m de error con el robot atascado sin GPS).
  Falta covarianza de odometría que crezca con el deslizamiento.
- Sin GPS el error crece (navegación a estima); con GPS ~2 cm p50 en simulación (el GPS simulado es casi ideal).
- **Latencia del GPS** no modelada (sin sello de tiempo de la medida).
- El tipo de nodo `field` debe existir en cortex; si no, el agente no puede crear el marco del mundo.
- `Field.Polygon` (x ±8, y ±13) delimita la zona de trabajo y la rejilla del planificador.

## 5. LiDAR (helios) y `residual_concept`

- **Self-filter**: la malla del Husky (`HuskyA300/husky_a300.stl`) elimina los retornos sobre el propio robot. Si
  cambia la geometría del robot hay que regenerarla (`webots-agri/tools/gen_husky_selffilter_stl.py`).
- **`lidar3d_dds` consume mucha CPU** si OpenMP hace espera activa: lanzar con `OMP_WAIT_POLICY=PASSIVE` y
  `[Threads] Count = 4` (de ~1200 % a ~17 %).
- **Umbrales de suelo pensados para suelo plano**: `FloorZ0 = 0,25`, `MinVerticalExtentM = 0,30`, `CeilZ = 1,10`.
  Con camellones de 0,20 m + ondulación + cabeceo del robot, el suelo puede superar 0,25 m a distancia y aparecer
  como obstáculo (a 10 m, 1° de cabeceo = 0,17 m de error en z). Pendiente: suelo local estimado (Patchwork++ o
  mapa de elevación, ver `teoria_navegacion.md`).
- **Celda fantasma persistente** conocida en la rejilla de residual.
- **Anillos del helios**: 50 capas en 110° ⇒ 2,2° entre anillos (0,5 m de separación vertical a 13 m). Un objeto
  estrecho a 11–13 m puede recibir solo 0–2 puntos.

## 6. Personas (`retina` + `human_concept`)

- **retina**: YOLO26-seg/pose en ONNX con CUDA (TensorRT no instalado). Los modelos están en `retina/models/yolo26`
  (en `.gitignore`: no se versionan).
- **Los ids de esqueleto de retina son índices por frame**, no identidades: `human_concept` los asocia por
  vecino más próximo en el marco del mundo.
- **Existencia por evidencia** (`human_lidar_presence`): cámara a favor; LiDAR a favor o en contra según retornos y
  rayos que atraviesan una caja de torso. Se borra con P(existe) < 0,2, o tras 2 s sin evidencia de ningún sensor.
- **Banda de torso del LiDAR: z 1,20–1,85 m del marco del mundo.** Supuestos:
  - suelo a z ≈ 0 (en `husky_field` la persona en el pasillo cumple; sobre un camellón estaría 0,2 m más alta);
  - la banda está por encima de las copas (0,95 m): con 1,05 m las hojas creaban fantasmas;
  - **un trabajador agachado queda por debajo**: el LiDAR no opina y solo actúan la cámara y los 2 s;
  - cualquier retorno en la banda dentro de la puerta de una persona se toma como esa persona (campo abierto).
- **Error de posición de la cámara ∝ distancia²** (`CameraDepthSigmaK = 0,002`: 0,29 m a 12 m).
- **Durante un giro** la cámara proyecta mal los esqueletos (latencia pose ↔ imagen): nacen personas falsas de
  1–2 ajustes que mueren en 2 s. Pendiente: nacimiento con `common/instance_tracker`.
- **El ajuste del esqueleto deriva a distancia** (1–4 m en pruebas anteriores).
- **El controller trata a la persona con un disco especial** (`PersonRadiusM 0,8`), no como al resto de obstáculos.
  Pendiente: huella física (`width_m`/`depth_m`) publicada por human_concept y que residual ceda esos puntos.
- `human_concept` no sigue el contrato común de los agentes de concepto (`CONCEPT_AGENT_LIFECYCLE.md`): sin
  `instance_tracker` ni `ai_belief`.

## 7. Navegación (`controller`)

- Planificador de rejilla binaria sobre el bbox del polígono del campo; obstáculos = nodos `object`/`obstacle`
  + celdas de residual + discos de personas. **Sin costes continuos** (pendiente: capas gaussianas).
- **Rejilla en coordenadas del mundo**, no centrada en el robot (pendiente: mapa local robocéntrico).
- `driving_enabled_` exige Run/clic (`Controller.ArmOnStart false`).
- La arista `target` no está registrada en cortex: el objetivo usa `goto_action` (`send_goal.py`, agente 98).
- **Skid-steer a baja velocidad de giro se atasca** (fricción lateral): giros muy lentos no arrancan.
- No se tiene en cuenta la pendiente ni la transitabilidad del suelo (camellón = no transitable solo si residual
  lo ve como obstáculo).

## 8. Infraestructura

- IDs de agente únicos en el grafo (colisión = SIGSEGV). Registro en `active_inference/CONCEPT_AGENT_RECIPE.md`;
  usados en el proyecto: 25 openfield_concept, 98 send_goal; siguiente libre 26+.
- IceStorm (rcnode) en el puerto 9999; el mando Xbox publica ahí (avance eje 1, giro eje 3 = stick derecho).
- DSR en el dominio DDS 0; plano de medios (LiDAR/IMU/ZED, zero-copy) en el dominio 7.
- La ZED funciona sin el SDK de ZED (`HAVE_ZED_SDK` opcional), con la cámara de Webots.
- `robot_concept` no aborta limpio con SIGINT en algunos casos (pendiente).
