# AiGB_UEx_Navigation — robótica agrícola con el Husky A300

Adaptación del stack cognitivo `active_inference` (RoboComp + grafo compartido DSR), que se escribió para el
robot social **Shadow** en interiores, a un **robot agrícola**: un Clearpath **Husky A300** simulado en Webots,
en un campo con hileras de palmeras y trabajadores, que se localiza por **GPS**, navega de forma autónoma por los
pasillos y detecta y sigue a las **personas**.

Estado: localización GPS (~2 cm), navegación autónoma por los pasillos y detección/seguimiento de personas
funcionando en simulación. Reconocimiento de tareas agrícolas: pendiente. Detalle en [`plan.md`](plan.md).

## Documentos

| Archivo | Qué contiene |
|---|---|
| [`plan.md`](plan.md) | Plan por fases (F0–F7), estado de cada una y pendientes |
| [`cambios.md`](cambios.md) | Qué cambió respecto al repo original del Shadow, archivo por archivo |
| [`bitacora.md`](bitacora.md) | Diario de trabajo: qué se hizo, qué se arregló y qué se midió |
| [`cosas_a_tener_en_cuenta.md`](cosas_a_tener_en_cuenta.md) | Restricciones, supuestos y límites conocidos — **leer antes de tocar nada** |
| [`teoria_navegacion.md`](teoria_navegacion.md) | Fundamento teórico: navegación LiDAR en campo abierto |
| `active_inference/CLAUDE.md`, `ARCHITECTURE.md`, … | Documentación original del stack (reglas de agentes DSR) |

## Qué hay en cada carpeta

```
AiGB_UEx_Navigation/
├── webots-agri/            NUEVO — todo lo agrícola de Webots
│   ├── protos/             HuskyA300 (robot + sensores), YoungPalm (palmera), FieldTerrain (suelo labrado)
│   ├── worlds/             husky_test.wbt (suelo plano) · husky_field.wbt (camellones, suelo de campo)
│   ├── controllers/        husky_selftest (autotest de sensores y conducción)
│   └── tools/              gen_palm_rows.py (hileras) · gen_husky_selffilter_stl.py (malla de auto-filtro)
├── webots-bridge/          Puente Webots → RoboComp (añadido: skid-steer, GPS, robot configurable)
├── robocomp-robolab/       Drivers: lidar3d_dds (helios), zed_camera, imu_dds, mando Xbox
├── active_inference/       Agentes cognitivos DSR
│   ├── robot_concept/      Cuerpo del robot en el grafo (husky.json)
│   ├── openfield_concept/  NUEVO — localización GPS + IMU + ruedas (EKF), sustituye a room_concept
│   ├── residual_concept/   Ocupación de lo que ningún concepto explica (palmeras, obstáculos)
│   ├── controller/         Planificador y control de la base
│   ├── retina/             YOLO (segmentación + pose) sobre la ZED
│   ├── human_concept/      Personas: esqueleto + seguimiento con LiDAR
│   ├── common/             Módulos compartidos (world_frame nuevo: `field` fuera, `room` dentro)
│   └── utils/              Lanzadores: sub_husky.toml, cognitive_husky.toml, send_goal.py
├── webots-shadow/          Mundo original del Shadow (sin cambios)
└── WORLD/                  Mundos y controladores de ejemplo (Farm, bioloid)
```

## Requisitos (no incluidos en el repo)

- **RoboComp** en `~/robocomp` (`robocomp_core`, `robocomp_interfaces`, `robocomp_tools`) y **cortex** (librería
  DSR) instalada en `/usr/local`, **con el tipo de nodo `field` registrado** (`REGISTER_NODE_TYPE(field)` en
  `core/include/dsr/core/types/type_checking/dsr_node_type.h`, y reinstalar).
- **Webots R2025a** en `/usr/local/webots`. Necesita red la primera vez (descarga PROTOs de GitHub).
- **Modelos ONNX de retina** en `active_inference/retina/models/yolo26/` (`yolo26x-seg.onnx`,
  `yolo26l-pose.onnx`): no se versionan (`*.onnx` está ignorado).
- ONNX Runtime con CUDA, Embree/TBB, Qt6, Eigen, FastDDS (los del stack original).

El repo debe estar en **`~/robocomp/AiGB_UEx_Navigation`**: los lanzadores usan esa ruta.

## Compilar

Cada componente se compila en su carpeta. **Máximo `-j8`** (la máquina no tiene swap y más hilos agotan la memoria):

```bash
cmake -B build && make -C build -j8
```

Componentes usados: `webots-bridge`, `robocomp-robolab/components/hardware/{laser/lidar3d_dds,camera/zed_camera,imu/imu_dds}`,
`active_inference/{robot_concept,openfield_concept,residual_concept,controller,retina,human_concept}`.

## Lanzar

```bash
# 0. IceStorm (rcnode) en el puerto 9999, como en el stack original.
# 1. Webots: abrir webots-agri/worlds/husky_field.wbt (o husky_test.wbt).
cd ~/robocomp/AiGB_UEx_Navigation/active_inference/utils
python3 subcognitive.py sub_husky.toml        # capa sensoriomotora: bridge, helios, IMU, ZED, mando
python3 cognitive.py cognitive_husky.toml     # capa cognitiva (otra terminal)
```

- **Mando Xbox**: avance con el stick izquierdo (eje 1), giro con el derecho (eje 3).
- **Objetivo de navegación**: clic en la vista 2D del controller (pulsar Run), o
  `python3 send_goal.py X Y` (coordenadas del campo, ENU: x este, y norte).
- **Ventana de localización** de `openfield_concept`: mapa, error frente a la verdad de la simulación y
  casillas para apagar el GPS o el yaw de la IMU.

## El robot y el campo

- **Husky A300**: 4 ruedas skid-steer, LiDAR `helios` (50 capas, 110° vertical, 30 m), cámara estéreo **ZED**,
  IMU, brújula y **GPS** (WGS84, referencia cerca del campus de la UEx en Badajoz).
- **Campo**: 5 hileras × 10 palmeras jóvenes, pasillos de 2,5 m. En `husky_field.wbt` cada hilera va sobre un
  camellón de 0,20 m, con suelo de tierra rugoso. Dos trabajadores caminan por el campo.
- **Marco del mundo**: nodo `field` del grafo (ENU), creado por `openfield_concept`. Los agentes buscan `field`
  y, si no existe, `room`: el mismo código sigue sirviendo en interiores.

## Origen

Instantánea del 18-09-2026 de `robocomp/active_inference` (`main` @ `5dc985e`), `robocomp/webots-bridge`
(`main` @ `486e233`), `robocomp/webots-shadow` (`main` @ `64a153d`) y parte de `robocomp/robocomp-robolab`
(`master` @ `1cc4825`), con las modificaciones locales que necesitaban para arrancar. Qué se ha cambiado
desde entonces: [`cambios.md`](cambios.md).
