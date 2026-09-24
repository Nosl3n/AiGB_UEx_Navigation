# field_concept — agente de posición tipo GPS para espacio abierto (Webots)

Documento de diseño. Analiza cómo funciona `room_concept`, qué papel cumple en la navegación del robot
y cómo construir `field_concept`: un agente equivalente para espacio abierto que publica la pose del
robot a partir de la pose real de la simulación con ruido añadido, simulando un GPS.

> Estado: **propuesta**. Todavía no hay código.

---

## 1. Qué hace `room_concept`

Tiene dos partes bien diferenciadas.

### 1.1 El localizador (lo que `field_concept` sustituye)

- **Entradas:** la franja alta del LiDAR (nodo `helios`), la odometría y los comandos de velocidad
  del nodo `robot`, y la IMU.
- **Referencia:** un polígono de la habitación cargado de un SVG (`RoomLayoutSvg` en
  `[Scenario.*]`, por ejemplo `p3bot_layout.svg`), o estimado por el propio agente (modo
  "wall-slam").
- **Estimación:** la pose del robot `[x, y, θ]` en el marco de la habitación. Optimiza una ventana
  deslizante que combina:
  - el ajuste de los puntos del LiDAR a la distancia a las paredes (SDF),
  - las esquinas detectadas,
  - la odometría preintegrada como prior de movimiento.
- **Arranque:** carga la pose guardada o calcula el centro de la nube de puntos, hace una búsqueda
  global en rejilla y resuelve la ambigüedad θ / θ+π.
- **Código:** `room_concept.cpp` (~8000 líneas), `room_model.cpp`, `corner_detector.cpp`,
  `wall_map.cpp`. Hay más detalle en `room_concept/ROOM_CONCEPT.md`.

### 1.2 El publicador en el DSR (el "contrato" que `field_concept` debe cumplir)

Lo implementan `room_scene_graph.cpp` y `pose_publisher.cpp`:

| Qué escribe | Detalles |
|---|---|
| **Nodo `room`** (tipo `room`, nombre `"room"`) | Atributos `delimiting_polygon_x/y` (contorno), `room_height` y `room_height_sigma`. **Solo se crea cuando la localización es estable** (`STABLE_FRAMES_REQUIRED` frames seguidos con covarianza baja, `room_scene_graph.cpp:243`). Antes de eso no se publica ninguna pose. |
| **Arista RT `robot → room`** | El padre es el **robot**, que es la raíz del árbol RT, y el hijo es la **room**. Por tanto guarda **T_robot←room** (la inversa de la pose del robot). Se escribe con `rt_api_->insert_or_assign_edge_RT(parent, child, RTBlock{...}, timestamp_ms)` (`room_scene_graph.cpp:680`). Es un buffer circular con marca de tiempo, así que los consumidores pueden interpolar al instante de su escaneo. |
| Covarianza de la pose | Es 6×6 SE3 fila-mayor `[x, y, z, rx, ry, rz]`. Las componentes SE(2) van en los índices **0, 1 y 5**. El controller lee el yaw de la posición (5,5). Antes de publicarla se transforma con el jacobiano de la inversión. |
| Twist | Es la velocidad del **hijo** (la room) en sus propios ejes, calculada con la adjunta SE(2) (`room_scene_graph.cpp:600-615`), y va con su propia covarianza. |
| Atributos de velocidad antiguos | `rt_translation_velocity = [adv, side, 0]` y `rt_rotation_euler_xyz_velocity = [0, 0, rot]`. El controller todavía los usa como verificación independiente. |
| Nodos `wall_i` y `floor` | Cuelgan de la room. El controller no los trata como obstáculos. |
| Afordancias | `afford_room` (exploración epistémica) y `afford_calib` (desactivada). |
| Presencia | Nodo latido `room_concept 5`, más la sección `[Owns]` para limpiar al salir. |

Hay dos ritmos de publicación:

- **Pose corregida**, a la frecuencia del LiDAR (~20 Hz).
- **Pose predicha**, a la frecuencia de la IMU, entre correcciones.

---

## 2. Por qué los demás agentes dependen de él

**La clave es que el nodo `room` es el marco "mundo" de todo el sistema.** Los agentes expresan sus
datos en `"room"`.

### 2.1 Controller

1. **Presencia.** `controller/etc/config.toml:49` tiene
   `required_agent_names = ["robot_concept", "room_concept"]`. El controller no pasa al estado
   Operating hasta que `room_concept` emite latido.
2. **Nodo room.** `ControllerWorldModel::refresh_graph_state()` busca un nodo de tipo `room` y otro de
   tipo `robot`. Si falta alguno, `ready()` devuelve falso.
3. **Pose.** La obtiene con
   `get_transformation_matrix(room, robot, ts, "RT", Interpolated)`
   (`controller_world_model.cpp:108`), es decir, recorriendo la arista que publica `room_concept`.
   También lee la covarianza, por ejemplo para limitar la rotación.
4. **Planificador.** `grid_planner.cpp:134` marca como ocupada toda celda que quede **fuera** del
   `delimiting_polygon`. Las paredes no se representan aparte: el polígono ya las hace. Si más del
   95 % de la rejilla queda fuera, descarta el polígono y entra en modo degradado.

### 2.2 Residual

1. **Presencia.** También exige `room_concept` (`residual_concept/etc/config.toml:33`).
2. **LiDAR.** Lo transforma al marco `"room"` con `reader_->poll("room", ...)`, pasando por la arista
   robot→room.
3. **Obstáculos.** Crea nodos `obstacle` llamados `residual_N` con una arista RT **room → residual_N**
   (`residual_scene_graph.cpp:74`). Así los obstáculos no modelados se añaden al grafo.
4. El controller los lee y marca sus celdas como ocupadas (`grid_planner.cpp:173`).
5. Lee el polígono de la room y las transformaciones room→objeto de los demás conceptos (mesas,
   sillas…) para descontar lo que ya está explicado.

### 2.3 Consecuencia

**El ruido de la pose publicada se propaga a todo lo que se proyecta en el marco room.** Si la pose
salta, los obstáculos del residual se emborronan o aparecen obstáculos fantasma, y el planificador
reacciona a ellos.

---

## 3. Viabilidad de `field_concept`

**Es viable y bastante más sencillo que `room_concept`.** Además, la pose real ya está en el grafo:
`robot_concept` escribe en el nodo `robot` los atributos

- `robot_gt_x`
- `robot_gt_y`
- `robot_gt_angle`
- `robot_gt_timestamp`

(`robot_concept/src/specificworker.cpp:2207`). Solo lo hace cuando el productor indica que es una
simulación, y los valores salen del supervisor de Webots a través del bridge
(`robotNode->getPosition()/getOrientation()`). Por tanto **`field_concept` no necesita ningún proxy
Ice**: le basta con leer el grafo.

Como alternativa, se puede usar `Webots2Robocomp::getObjectPose(DEF)` (proxy `Webots2Robocomp`, que
ya existe en la configuración de `room_concept`). No hace falta.

---

## 4. Diseño propuesto

### 4.1 Estructura

- Generarlo con `robocompdsl` usando `options dsr`, igual que `room_concept.cdsl`. No necesita
  interfaces adicionales.
- Reutilizar `common/agent_presence_monitor` y la misma estructura `[Presence]` / `[Owns]`:

```toml
[Agent]
id   = <id libre>
name = "field_concept"

[Presence]
required_agent_names = ["robot_concept"]

[Owns]
nodes    = ["field_concept <id>"]
subtrees = ["room"]      # el agente sí es dueño de la room en este caso
```

### 4.2 Arranque: crear el nodo `room`

Debe tener el **mismo tipo y el mismo nombre `"room"`**, porque residual usa `"room"` literal en el
código.

- `delimiting_polygon_x/y`: un rectángulo, definido en la configuración, que delimite la zona útil
  del mundo abierto.
  - Debe tener al menos 3 vértices.
  - El tamaño tiene que ser razonable, porque la rejilla del planner se construye sobre su bounding
    box: un campo de 500 m generaría una rejilla enorme.
- `room_height`: un valor alto. Antes hay que revisar qué agentes lo usan para recortar el LiDAR.
- Opcionalmente, un nodo `floor` para el viewer3d. Sin nodos `wall_i`.

### 4.3 Bucle: leer la pose, añadir ruido y publicar

Se ejecuta, por ejemplo, cada vez que cambia `robot_gt_timestamp`.

**a) Leer la pose GT y convertirla al marco room.** Hay que tener cuidado con las convenciones:

- `room_concept/src/ground_truth_log.cpp` indica que el signo de `robot_gt_angle` está **invertido**
  respecto a la convención del sistema. Allí lo niegan localmente y comprueban el signo con
  estadística (`gt_convention_report`).
- El cuerpo del robot usa **+Y hacia delante** y **+X lateral** (ver `../FRAMES.md`).
- Hay que decidir el origen del marco room: el origen de Webots o la pose inicial del robot.
- Hay que comprobar las unidades (metros).

**b) Añadir ruido.** Opciones, de más simple a más realista:

1. Gaussiano blanco en x, y y θ. Es lo más fácil, pero produce saltos entre muestras que afectan al
   residual y a cualquier velocidad derivada de la pose.
2. **Gauss-Markov / paseo aleatorio correlacionado** (recomendado): un sesgo que deriva lentamente
   más un pequeño ruido blanco. Se parece más a un GPS real.
3. Publicar a baja frecuencia (1-10 Hz, como un GPS) y extrapolar con la odometría entre muestras
   mediante un filtro complementario o un EKF sencillo. Es lo mismo que hace `room_concept` con su
   pose predicha.

> Un GPS real no da orientación. Lo realista sería obtener θ de la IMU o de una brújula con ruido.
> Para empezar se puede tomar θ de la GT con ruido.

**c) Escribir la arista RT `robot → room`** exactamente como `RoomSceneGraph::dsr_update_pose()`
(`room_concept/src/room_scene_graph.cpp`). Lo más seguro es copiar esa función, porque sus
comentarios documentan varios errores que ya salieron caros: el orden adv/side, la adjunta y el
slot del yaw.

- Traslación y rotación: la **inversa** de la pose (T_robot←room).
- Covarianza: la del ruido inyectado, transformada con el jacobiano de la inversión y colocada en los
  índices 0, 1 y 5 de la 6×6.
- Twist: el del hijo, calculado con la adjunta SE(2) a partir de la velocidad medida del robot, con
  su covarianza.
- Atributos de velocidad antiguos, escritos **antes** del bloque RT, porque el orden importa (ver el
  comentario "ORDER IS LOAD-BEARING").
- `timestamp_ms` coherente con el reloj de la simulación, porque el controller interpola al instante
  del LiDAR.

**d) Publicar una covarianza honesta.** Si se declara σ = 1 cm pero se inyectan 20 cm, el residual y
el controller confiarán más de la cuenta en la pose.

### 4.4 Enganche con el resto del sistema

- **Opción recomendada: cambiar solo configuración.** En controller y residual, poner
  `required_agent_names = ["robot_concept", "field_concept"]`. El código no cambia, porque ambos
  buscan el nodo por tipo `room`.
- Opción alternativa: llamar al agente `room_concept`. Evita tocar las configuraciones, pero es
  confuso y conviene evitarlo.

### 4.5 Qué no tiene que hacer

- Grid search, SDF, detección de esquinas ni calibración.
- La afordancia `afford_room`. En la configuración del controller, `room_concept` aparece como agente
  requerido y las misiones llegan por otras afordancias. **Hay que comprobar** si el flujo de
  exploración del controller espera `afford_room`.

---

## 5. Riesgos y puntos a vigilar

- **Obstáculos.** En espacio abierto no hay paredes: el único límite es el rectángulo del polígono, y
  la navegación depende de que el residual detecte bien los obstáculos.
- **Priors de contención.** Agentes como `chair_concept` o `cabinet_concept` usan el polígono como
  prior de "estar dentro de la habitación". Con un rectángulo grande ese prior se vuelve permisivo.
- **Ruido alto.** Hace que los obstáculos del residual vibren. Por eso se recomienda ruido
  correlacionado o la fusión con odometría antes que ruido blanco puro.
- **Uso de la GT.** `robot_concept` advierte que la GT es solo para validación. Aquí se usa a
  propósito como sensor simulado, así que no sirve para evaluar la precisión de la localización:
  el error medido será simplemente el ruido que se haya inyectado.
- **Latencia.** La GT se muestrea en el bridge. Hay que usar `robot_gt_timestamp` como marca de la
  arista, no la hora de lectura.

---

## 6. Resumen

`field_concept` sustituye el localizador de `room_concept` por la GT de Webots con ruido y mantiene
el mismo contrato en el DSR:

- nodo `room` con `delimiting_polygon`,
- arista RT `robot → room` con marca de tiempo, covarianza y twist,
- presencia como agente requerido.

Así, controller y residual funcionan sin cambiar su código, solo su `required_agent_names`. El
trabajo real está en respetar el formato de la arista RT y las convenciones de marcos y signos.
