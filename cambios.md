# Cambios respecto al repo original (Shadow)

**Punto de partida:** la instantánea del stack del robot social **Shadow** (commit `98ac65f`, "Copy"). Es un robot
de interiores que se localiza contra las paredes de una habitación (`room_concept`) y usa una cámara 360 Ricoh y
dos LiDAR (helios + bpearl).

**Qué es ahora:** un **Husky A300 agrícola** en campo abierto, localizado por GPS. Este documento resume qué se
añadió y qué se modificó. El cómo y el porqué de cada paso está en [`bitacora.md`](bitacora.md); los límites en
[`cosas_a_tener_en_cuenta.md`](cosas_a_tener_en_cuenta.md).

**Compatibilidad:** todo lo del Shadow sigue en su sitio. Los cambios en código compartido buscan primero el marco
`field` y, si no existe, `room`, así que en interiores el comportamiento es el de antes.

---

## 1. Resumen

| Área | Shadow (original) | Husky agrícola (ahora) |
|---|---|---|
| Robot | Shadow, base omnidireccional/diferencial | Husky A300, 4 ruedas **skid-steer** (PROTO propio) |
| Mundo | Piso interior (`webots-shadow`) | Campo con hileras de palmeras, camellones y trabajadores (`webots-agri`) |
| Sensores | helios + bpearl + ZED + Ricoh 360 + IMU | helios + ZED + IMU + brújula + **GPS** |
| Localización | `room_concept` (paredes de la habitación) | **`openfield_concept`** (EKF GPS + IMU + ruedas) |
| Marco del mundo | nodo `room` | nodo **`field`** (ENU), nuevo tipo en cortex |
| Personas | `human_concept` (no llegaba a crear personas) | Esqueleto + identidad estable + **seguimiento con LiDAR** |
| Navegación | controller sobre la habitación | controller sobre el polígono del campo |

## 2. Añadido (nuevo)

### `webots-agri/` — todo nuevo
- `protos/HuskyA300.proto`: chasis, 4 ruedas skid-steer, mástil con `helios` (50 capas, 110°, 30 m), `zed` +
  `zed-ranger` (inclinada 15°), IMU, brújula, acelerómetro, giróscopo y GPS. Ruidos expresados en unidades físicas
  (m, rad) y convertidos a las unidades relativas de Webots.
- `protos/YoungPalm.proto`: palmera joven procedural (tronco 0,7 m, copa ~0,6 m).
- `protos/FieldTerrain.proto`: suelo labrado con camellones, pasillos, ondulación y terrones.
- `worlds/husky_test.wbt` (suelo plano) y `worlds/husky_field.wbt` (suelo de campo): 5×10 palmeras, 2 trabajadores
  (Pedestrian), GPS en WGS84 con referencia en Badajoz.
- `controllers/husky_selftest/`: autotest de dispositivos, odometría, GPS y capturas.
- `tools/gen_palm_rows.py` (hileras dimensionadas para el robot), `tools/gen_husky_selffilter_stl.py`
  (malla del robot para el auto-filtro del LiDAR y para el grafo).

### `active_inference/openfield_concept/` — agente nuevo (ID 25)
Sustituye a `room_concept` en exteriores.
- **EKF SE(2)**: predice con ruedas + giróscopo; corrige con el GPS (con brazo de palanca de la antena) y con el
  yaw absoluto de la IMU.
- Crea el nodo `field` y publica la arista RT robot → `field` con covarianza y velocidad.
- **Ventana Qt**: mapa, error frente a la verdad de la simulación, y casillas para apagar el GPS o el yaw de la IMU.
- `geo_utm.h` (WGS84 ↔ UTM), interfaz `GPS.ice`, polígono del campo en metros locales o en coordenadas GPS.

### `active_inference/common/world_frame/world_frame.h` — nuevo
`frame_node`, `frame_nodes`, `frame_name` y `FrameCache`: devuelven `field` si existe y, si no, `room`. Lo usan
controller, residual, retina y human_concept en lugar del `"room"` escrito a mano.

### `active_inference/human_concept/src/human_lidar_presence.{h,cpp}` — nuevo
El LiDAR como "segunda mirada": existencia de cada persona por evidencia y seguimiento cuando la cámara la pierde.

### Configuraciones del Husky
- `robot_concept/husky.json` (árbol de frames: husky → body → helios/zed/imu/gps), `etc/config_husky.toml`,
  `etc/base_husky.toml`, `etc/mount_calib_husky.txt`, `meshes/husky_a300.obj`.
- `controller/etc/config_husky.toml`, `residual_concept/etc/config_husky.toml`, `retina/etc/config_husky.toml`.
- `utils/sub_husky.toml` (capa sensoriomotora), `utils/cognitive_husky.toml` (capa cognitiva),
  `utils/send_goal.py` (objetivo de navegación por línea de comandos, agente 98).
- `webots-bridge/etc/config_husky`, `lidar3d_dds/etc/config_helios_husky_webots.toml`,
  `lidar3d_dds/robots/HuskyA300/husky_a300.stl`, `zed_camera/etc/configHuskyWebot.toml`,
  `python_xbox_controller/etc/config_husky`.

### Fuera del repo (aprobado)
- **cortex**: `REGISTER_NODE_TYPE(field)` en `dsr_node_type.h`, reinstalado. Sin esto `openfield_concept` no puede
  crear el marco del mundo.

## 3. Modificado

### `webots-bridge/`
- Nombre del robot configurable (`Robot.Def`) en vez de `"shadow"` escrito en el código.
- Cinemática **skid-steer** (`Base.Kinematics "skid"`) con factor de pista (`Base.TrackScale 1.875`, como el
  `wheel_separation_multiplier` de Clearpath).
- **Interfaz GPS** nueva (`generated/gpsI.{h,cpp}`, `Endpoints.GPS`, puerto 10009) con conversión UTM.
- IMU: orientación desde el InertialUnit y magnetómetro desde la brújula.

### `robocomp-robolab/`
- `lidar3d_dds`: el filtro de malla acepta la malla de cualquier robot (antes solo la del Shadow); nueva clave
  `Threads.Count` (OpenMP + Embree). CPU de ~1200 % a 17 %.
- `zed_camera`: el SDK de ZED pasa a ser opcional (`HAVE_ZED_SDK`), para usar solo la cámara de Webots.
- `.gitignore` para `bin/`, `build/` e `*.ice` generados.

### `active_inference/`
- **controller**: marco del mundo vía `world_frame`; `field` no cuenta como obstáculo; disco de seguridad para
  cada persona (`PersonRadiusM`); opción `Controller.ArmOnStart`; el objetivo usa la arista `goto_action`.
- **residual_concept**: marco del mundo vía `world_frame`; corregido un bug en `voxel_downsample` de la ZED (suma
  sobre memoria sin inicializar); auto-filtro con la **forma real del robot** (caja orientada) en vez de un disco;
  parámetros de campo (suelo, altura máxima, tamaño mínimo de grupo).
- **retina**: búsquedas de `"room"` sustituidas por `world_frame`; config del Husky solo con la ZED, clases
  relevantes en el campo y pose humana activada.
- **human_concept**:
  - el fitter **nunca se construía**: no se creaba ninguna persona;
  - identidades estables (asociación en el marco del mundo; los ids de retina son índices por frame);
  - duplicados descartados;
  - borrado por evidencia de existencia (cámara + LiDAR) en vez de un contador que no avanzaba;
  - seguimiento con LiDAR cuando la cámara deja de ver a la persona.
- **common**: `lidar_ingestor` y `nbv/graph_obstacles.h` usan el marco del mundo en vez de `"room"`.
- `CONCEPT_AGENT_RECIPE.md`: registro de IDs de agente actualizado (25, 98).

## 4. Sin cambios
`webots-shadow/`, `room_concept`, los demás agentes de objetos (mesa, silla, botella, puerta, nevera, …) y los
lanzadores del Shadow (`sub_shadow.toml`, `cognitive.toml`).

## 5. Diferencias a revisar antes de subir
- `active_inference/controller/etc/missions.toml`: tiene un punto añadido en tiempo de ejecución, fuera del piso
  (`63.73, 121.09`), en la misión "complete tour". Conviene restaurarlo:
  `git checkout 98ac65f -- active_inference/controller/etc/missions.toml`.
- `residual_concept/etc/residual_room_poly.csv` y `openfield_concept/tmp/openfield_pose.csv`: los reescriben
  los agentes al ejecutarse; no son cambios de diseño.
- `python_xbox_controller/etc/config_shadow`: el puerto de IceStorm es 1237, frente a 9999 en el original. El Husky
  usa `config_husky` (9999).
