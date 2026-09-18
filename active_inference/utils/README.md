# utils — cómo lanzar la pila

Hay **dos lanzadores** y **varios TOML** para cada uno. Esta página dice cuál usar en cada caso.
Detalles del monitor web y su arquitectura: [netmon/README.md](netmon/README.md).

## TL;DR — simulación de Shadow en Webots (lo que funciona hoy en esta máquina)

```bash
cd ~/robocomp/components/active_inference/utils

# Terminal 1 — capa sensorimotora (Webots + bridge + sensores)
python3 subcognitive.py sub_shadow.toml

# Terminal 2 — capa cognitiva (agentes DSR), cuando la tabla de la terminal 1 esté en verde
python3 cognitive.py cognitive.toml
```

- Monitor web: <http://127.0.0.1:8080>
- Webots debe tener abierto **`webots-shadow/worlds/piso/piso.wbt`** (robot `ShadowDiff`). El
  lanzador arranca Webots si no está corriendo, pero abre **el último mundo usado**: compruébalo.
- Se lanza **desde `utils/`**: los dos scripts importan el paquete `netmon/` de esta carpeta.

## Los dos lanzadores

| Script | Capa | Qué arranca | TOML por defecto |
|---|---|---|---|
| `subcognitive.py` | sensorimotora | Webots (si hace falta), rcnode/icebox, bridge, cámaras, LiDAR, IMU, joystick | `sub.toml` |
| `cognitive.py` | cognitiva | agentes DSR: `robot_concept`, `room_concept`, `retina`, `*_concept`, `controller` | `cognitive.toml` |

Siempre **primero `subcognitive.py`, luego `cognitive.py`**. Los agentes no arrancan Webots y
necesitan los sensores ya publicando. El orden *dentro* de `cognitive.toml` da igual: cada agente
espera a sus `required_agent_names`.

Los dos comparten el monitor web: el primero que arranca lo sirve y el otro solo registra su estado.

> ⚠ **No pases el TOML del otro lanzador.** `subcognitive.py cognitive.toml` o
> `cognitive.py sub_shadow.toml` arrancan igual, pero en la capa equivocada. Por ejemplo,
> `cognitive.py` nunca arranca Webots.

## Qué TOML usar

### `subcognitive.py`

| TOML | Escenario | Estado en esta máquina (2026-09-17) |
|---|---|---|
| **`sub_shadow.toml`** | **Shadow en Webots** (`piso.wbt`) | ✅ **Usar este.** Todo presente. |
| `sub.toml` | Shadow en Webots, **versión antigua** (Lidar3D, RicohOmni, RGBD_360, zed_component) | ❌ Obsoleto: faltan 6 binarios. Lo sustituye `sub_shadow.toml`. |
| `sub_real.toml` | Robot Shadow **real** (sin Webots, `[general] start_webots = false`) | ⚠ Falta el binario `SVD48VBase` y los configs `config_shadow_real.toml`, `config_helios_flip.toml`, `config_bpearl_shadow.toml`. |
| `sub_p3bot.toml` | P3Bot en Webots | ❌ No existe `webots-p3bot/components/p3bot-bridge`, y falta `zed_camera/etc/config_webots.toml`. |

### `cognitive.py`

| TOML | Escenario | Estado en esta máquina (2026-09-17) |
|---|---|---|
| **`cognitive.toml`** | **Simulación** (va con `sub_shadow.toml`) | ✅ **Usar este.** |
| `cognitive_real.toml` | Robot real, mínimo (robot + room + imu_fusion) | ❌ `room_concept/etc/config_apartamento.toml` ya no existe (ahora es `config.toml`), y no existe `robocomp-shadow/agents/imu_fusion`. |
| `cognitive_real_completo.toml` | Robot real, completo (+ retina, residual, table, chair…) | ❌ Mismos fallos que `cognitive_real.toml`, y además falta el binario de `residual_concept`. |

Para volver a comprobar las tablas (binario y config de cada entrada):

```bash
python3 - <<'E'
import toml, os, glob
for f in sorted(glob.glob('*.toml')):
    for c in toml.load(f).get('components', []):
        cwd = os.path.expanduser(c.get('cwd', '')); cmd = c.get('cmd', '').replace('vglrun ', '').split()
        miss = [x for x, p in (('bin', cmd[0]), ('cfg', cmd[1] if len(cmd) > 1 else None))
                if p and not os.path.exists(os.path.join(cwd, p))]
        if miss: print(f, c['name'], miss)
E
```

> Los `cognitive.toml` y `sub.toml` de la **raíz** de `active_inference/` son copias antiguas: no
> los uses con estos scripts.

## Configs a vigilar (ya causaron problemas)

- **Bridge de Webots con `etc/config`, no `etc/config.toml`.** Solo `etc/config` trae
  `Base.Kinematics = "differential"`. Con `config.toml` el bridge busca las ruedas mecanum
  (`wheel1..4`), no encuentra motores en `ShadowDiff` y **el robot no se mueve**, aunque el
  controller mande velocidades: acaba en bucles `SPINNING` / `ESCAPE`. Al arrancar, el log del
  bridge debe decir `Base: differential (2 driven wheels, R=0.1 m, half-track=0.259 m)`.
- **`robot_concept/etc/config_shadow.toml` → `base_config_file`** debe ser una ruta que exista
  en *esta* máquina (`/home/nosl3n/...`). Si no existe, `robot_concept.err` dice
  `could not read base config` y el controller usa sus constantes por defecto.
  `config_p3bot.toml` todavía apunta a `/home/pbustos/`.

## Logs

`tmux` no está instalado, así que cada componente escribe en:

```
~/.local/logs/<name>.out     # stdout
~/.local/logs/<name>.err     # stderr
```

`<name>` es el campo `name` del TOML (`bridge`, `controller`, `robot_concept`…). **Se
sobrescriben en cada arranque**: copia el log **después** de parar el componente y **antes** de
relanzar.

La consola de Webots no va a ningún log (el lanzador la manda a `/dev/null`); se ve en la propia
ventana de Webots.

Con `tmux` instalado, cada componente corre en su propia sesión y el monitor web muestra su
terminal.

## Parar

- **Ctrl+C en cada terminal**: primero `cognitive.py`, luego `subcognitive.py`. El lanzador
  manda SIGTERM a cada componente, los agentes borran sus nodos del grafo DSR y, si alguno no
  termina en 5 s, lo mata con SIGKILL.
- **Nunca `kill -9`** a un agente: sus nodos se quedan colgados en el grafo compartido (ver
  `../CLAUDE.md`, «Stopping an agent»).
- Relanzar con procesos viejos todavía vivos también los mata con **SIGKILL** (`_remove_existing`).
  Para antes con Ctrl+C y relanza después.
- Webots no se cierra al parar `subcognitive.py`: se reutiliza en el siguiente arranque.

## Flags útiles

| Flag | Script | Efecto |
|---|---|---|
| `--no-webots` | `subcognitive.py` | No arrancar Webots (lo ignora si el TOML tiene `[general] start_webots`) |
| `--no-rcnode` | `subcognitive.py` | No comprobar/arrancar rcnode (icebox) |
| `--no-battery` | `subcognitive.py` | Sin monitor de batería Victron (`/dev/ttyUSB0`) |
| `--no-web` | ambos | No servir el monitor web |
| `--web-port N` | ambos | Puerto web (el mismo en los dos; por defecto 8080) |
| `--no-bw` | ambos | Sin captura de ancho de banda ICE |

## Otras cosas de esta carpeta

| Ruta | Qué es |
|---|---|
| `netmon/` | Paquete común de los dos lanzadores: monitor web, registro en `/tmp/robocomp_netmon/`, puente de estadísticas DDS |
| `dds_preflight/` | Comprueba que el descubrimiento DDS por memoria compartida funciona (dominio 7). Los lanzadores lo ejecutan solos al arrancar. Si falla, casi siempre quedan segmentos viejos: para todo y `rm -f /dev/shm/fastdds_*`. Síntoma sin él: el plano de medios a 0.0 Hz mientras Ice funciona. |
| `pano_grab/` | Diagnóstico: guarda UN frame de la 360 (`rc/ricoh/rgb`) en PNG — `pano_grab [out.png] [topic] [domain]` |
