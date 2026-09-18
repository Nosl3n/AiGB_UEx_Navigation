# AiGB_UEx_Navigation

Instantánea (18-09-2026) del stack `active_inference` sobre el robot Shadow en Webots, **con las
modificaciones locales incluidas**. Las versiones limpias de los repos originales no arrancan tal cual,
así que cada carpeta es el commit indicado **más** los cambios sin commitear que había en ese momento.
No se incluye el historial de git, ni `build/`, ni `bin/`, ni lo que ignora cada `.gitignore`.

| Carpeta | Repo original | Rama @ commit | Cambios locales incluidos |
|---|---|---|---|
| `active_inference/` | https://github.com/robocomp/active_inference | `main` @ `5dc985e` | `cognitive.toml`, `sub.toml`, `utils/cognitive.toml`, `utils/sub_shadow.toml`, `utils/README.md` (nuevo), `controller/src/lidar_buffer_types.h`, `retina/tools/expose_semantic_logits.py`, `robot_concept/etc/config_shadow.toml`, `robot_concept/shadow.json`, `robot_concept/etc/mount_calib_Shadow.txt` (nuevo), `room_concept/etc/{camera_calib_Shadow_ricoh,camera_calib_Shadow_zed,image_edge_replay_ricoh,image_edge_replay_zed,last_robot_pose}.txt` |
| `webots-bridge/` | https://github.com/robocomp/webots-bridge | `main` @ `486e233` | `etc/config.toml`, `generated/CMakeLists.txt`, `generated/main.cpp`, `generated/doorcontrolI.{cpp,h}` (nuevos) |
| `webots-shadow/` | https://github.com/robocomp/webots-shadow | `main` @ `64a153d` | `worlds/piso/piso.wbt`, `worlds/piso/.piso.jpg` (nuevo) |
| `robocomp-robolab/` | https://github.com/robocomp/robocomp-robolab | `master` @ `1cc4825` | Solo los drivers usados: `ricoh_omni_dds`, `lidar3d_dds`, `imu_dds`, `zed_camera`, `python_xbox_controller`, `SVD48VBase` |

Excluido a propósito: `active_inference/room_concept/tmp/` (volcados de depuración, uno de ellos de 105 MB,
por encima del límite de GitHub).

## Qué NO está aquí (hay que instalarlo aparte)

- **Instalación de RoboComp**, que clona automáticamente en `~/robocomp`:
  `robocomp_core` @ `e78962d`, `robocomp_interfaces` @ `68fb477`, `robocomp_tools` @ `c40cfbf6a` y
  `cortex` (`development` @ `533c58e`, la librería DSR que se instala en `/usr/local/lib`).
- **Webots R2025a** (`/usr/local/webots`).
- **Modelos ONNX de retina** (`active_inference/retina/models/…`): no están versionados.

## Dónde va cada carpeta

Los `.toml` y algunas configuraciones usan rutas absolutas, así que hay que respetar esta estructura:

```
~/robocomp/components/active_inference
~/robocomp/components/webots-bridge
~/robocomp/components/webots-shadow
~/robocomp/components/robocomp-robolab/components/hardware/...
```

## Lanzar

```bash
cd ~/robocomp/components/active_inference/utils
python3 subcognitive.py sub_shadow.toml     # capa sensorimotora (Webots, bridge, drivers)
python3 cognitive.py cognitive.toml         # capa cognitiva (agentes DSR), en otra terminal
```
