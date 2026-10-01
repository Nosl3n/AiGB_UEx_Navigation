# Teoría: LiDAR para navegación en campo abierto agrícola

Resumen de la literatura sobre cómo usar el LiDAR para navegar en entornos agrícolas de campo abierto,
y cómo encaja cada técnica en este proyecto (Husky A300 + `openfield_concept` + `residual_concept` +
`controller`).

> **Aviso sobre las referencias.** Las citas están hechas de memoria, sin consultar las fuentes en el
> momento de redactar. Verifica título, autores y año antes de usarlas en un trabajo o artículo.

---

## 0. El problema

En interiores el suelo es un plano y el robot está siempre horizontal, así que separar "suelo" de
"obstáculo" es fácil: todo lo que esté por encima de una banda fija sobre el plano es un obstáculo. En
un campo agrícola eso falla por tres razones:

1. **El suelo no es uniforme.** Hay surcos, pendientes, baches y montículos, y su altura varía de un
   metro a otro. Un plano global no lo describe.
2. **El robot no está siempre horizontal.** Al inclinarse, el LiDAR inclina con él toda la nube. El
   suelo de delante aparece "levantado" y se confunde con obstáculos, o se pierden obstáculos bajos.
3. **El LiDAR siempre golpea algo.** Hierba, hojas, rastrojo o polvo dan retornos que no son
   obstáculos sólidos. Si se marcan todos como ocupados, el robot no se mueve.

Las secciones siguientes son las respuestas que da la literatura, en el orden en que se aplicarían.

---

## 1. Compensar la inclinación del robot

**Idea.** Antes de decidir qué es suelo, se gira cada nube al **marco de la gravedad** con el roll y el
pitch de la IMU. Si el robot va en movimiento, además se corrige la distorsión del barrido (*de-skew*):
cada punto se refiere a la pose del robot en el instante en que se midió, no al del final del barrido.

**Referencias.**
- Los sistemas LiDAR-inercial lo integran como parte de la odometría; por ejemplo, **LIO-SAM**
  (Shan et al., 2020).

**En este proyecto.**
- La IMU ya publica roll, pitch y yaw (`InertialUnit` del PROTO → bridge → `imu_dds` → media plane).
- `openfield_concept` hoy solo estima `[x, y, θ]`; el roll y el pitch están disponibles pero no se
  aplican a la nube. Este sería el primer paso.

---

## 2. Suelo local en lugar de un plano global

**Idea.** En vez de ajustar un único plano a todo el suelo, se divide el entorno en regiones y se
estima un suelo local en cada una. Así se siguen las pendientes y el terreno irregular.

**Referencias.**
- **Himmelsbach et al., 2010** — segmentación por sectores radiales con ajuste de rectas por sector.
- **Patchwork** (Lim et al., RA-L 2021) y **Patchwork++** (Lee et al., IROS 2022) — un plano por celda de
  un modelo de zonas concéntricas alrededor del sensor; robusto a pendientes y terreno irregular. Es la
  referencia práctica actual.
- **TRAVEL** (Oh et al., RA-L 2022) — segmenta a la vez el suelo transitable y los objetos, pensado
  para terreno exterior.
- **CSF, Cloth Simulation Filter** (Zhang et al., 2016) — extrae el suelo desnudo bajo vegetación
  simulando una tela que cae sobre la nube invertida.

**En este proyecto.**
- `residual_concept` ajusta hoy **un único plano de suelo** por barrido. Su referencia era el bpearl
  del Shadow; el Husky no lo tiene, así que el plano se queda en z = 0. En `husky_test.wbt` (suelo
  llano) funciona; en `Farm.wbt` (terreno irregular) no.
- Sustituirlo por una segmentación local tipo Patchwork++ es el cambio de más impacto para terreno
  irregular.

---

## 3. Mapas de elevación: las "superficies gaussianas"

**Idea.** Cada celda del mapa guarda la **altura del terreno como una gaussiana** (media y varianza).
Cada nueva medida se fusiona con un filtro de Kalman, y la incertidumbre de la pose del robot se
propaga a la varianza del mapa: si el robot no sabe bien dónde está, el mapa refleja esa duda.

**Referencias.**
- **Fankhauser et al., "Probabilistic terrain mapping for mobile robots with uncertain localization"**
  (RA-L 2018), con las librerías `elevation_mapping` y `grid_map`.
- Versiones continuas del mapa:
  - **Gaussian Process terrain modeling** (Vasudevan et al., JFR 2009).
  - **Inferencia bayesiana con kernels, BGK** (Shan, Wang y Englot, 2018).
  - **Hilbert maps** (Ramos y Ott), que ya propone el documento del repo `RESIDUAL_BELIEF_FIELD.md`.

**En este proyecto.**
- Encaja muy bien: `openfield_concept` ya publica la **σ de la pose** (covarianza en la arista
  `husky → field`), que es exactamente lo que este modelo necesita propagar al mapa.
- La rejilla de `residual_concept` ya es un posterior probabilístico por celda (Bernoulli en
  log-odds), pero de ocupación, no de altura. Un mapa de elevación sería una capa nueva o una
  ampliación de esa rejilla.

---

## 4. Transitabilidad en lugar de "ocupado / libre"

**Idea.** Del mapa de elevación se calculan, por celda, la **pendiente**, la **rugosidad** y el
**escalón** respecto a las vecinas. Con eso se obtiene un **coste continuo de transitabilidad** que el
planificador minimiza, en lugar de un mapa binario.

**Referencias.**
- **Wermelinger et al., 2016** — planificación sobre mapas de transitabilidad.
- **Driving on point clouds** (Krüsi et al., JFR 2017).
- **STEP** (Fan et al., 2021) — transitabilidad consciente del riesgo.

**En este proyecto.**
- El planificador del `controller` (`grid_planner.cpp`) ya tiene un coste blando: penaliza pasar cerca
  de obstáculos con una rampa lineal hasta 0,9 m (`ClearanceWeight`, `ClearancePref`).
- Un coste de transitabilidad continuo es la versión fundamentada de "poner gaussianas en el mapa de
  coste", y respeta la regla del repo de **no usar umbrales** (`active_inference/CLAUDE.md`): el
  riesgo crece de forma continua con la pendiente o la rugosidad, sin cortes duros.
- **Condición de seguridad** (de `RESIDUAL_BELIEF_FIELD.md`): esta capa debe ir **encima** de la
  ocupación dura, nunca sustituirla. Todo retorno real debe subir el riesgo en algún sitio, y lo
  "nunca visto" no puede leerse como libre.

---

## 5. Vegetación frente a obstáculo sólido

**Idea.** Se clasifica la geometría local de los puntos con un análisis de componentes principales de
su vecindad:

| Clase | Forma local | Ejemplo |
|---|---|---|
| Dispersión | puntos repartidos en volumen | hierba, hojas |
| Lineal | puntos alineados | troncos, ramas, postes |
| Superficie | puntos en un plano | suelo, rocas |

Con eso, las hojas cuestan poco y los troncos bloquean.

**Referencias.**
- **Lalonde et al., "Natural terrain classification using 3-D ladar data for ground robot mobility"**
  (JFR 2006).
- Variantes modernas añaden la intensidad del retorno, el multi-eco o el aprendizaje automático.

**En este proyecto.**
- Es justo el caso de las palmeras de `husky_test.wbt`: hoy residual marca las hojas como obstáculo,
  así que el robot ve pasillos de 1,3 m (entre copas) en lugar de 2,5 m (entre troncos).

---

## 6. Específico de cultivos en hileras

**Idea común.** Usar la **estructura de las hileras** como información previa (líneas paralelas a
distancia conocida) para localizar y guiar al robot dentro del cultivo, y el **GPS** para el
posicionamiento global entre parcelas.

**Referencias.**
- **Hiremath et al., 2014** — navegación en maíz con un modelo del láser y filtro de partículas.
- **Malavazi et al., 2018** — navegación con solo LiDAR para un robot agrícola.
- **Higuti et al., JFR 2019** (TerraSentia, navegación bajo el dosel) y **Velasquez et al., 2021** —
  seguimiento de hileras con fusión de sensores.
- **Winterhalter et al., RA-L 2018 / JFR 2021** — detección de hileras y localización "más allá de
  seguir la hilera".
- **Bergerman et al., 2015** — robots en frutales.
- **Gasparino et al., WayFAST 2022** — transitabilidad aprendida de forma auto-supervisada.
- **Datasets:** FieldSAFE (Kragh et al., 2017) y los de Bosch/BoniRob.

**En este proyecto.**
- Sería un futuro `crop_row_concept` (tarea 18 de `plan.md`): publicaría las hileras en el DSR como
  conocimiento del entorno, de modo que residual dejaría de tratarlas como "desconocidas".

---

## 7. Hoja de ruta propuesta para este proyecto

Por orden de impacto:

| # | Paso | Dónde | Resuelve |
|---|---|---|---|
| 1 | Girar las nubes al marco de la gravedad con la IMU (roll, pitch) | `residual_concept` (o el driver del LiDAR) | robot inclinado |
| 2 | Segmentación local del suelo tipo Patchwork++ en lugar del plano único | `residual_concept` | suelo irregular |
| 3 | Mapa de elevación con varianza por celda, alimentado con la σ de `openfield_concept` | capa nueva o ampliación de la rejilla de residual | "superficies gaussianas" |
| 4 | Coste de transitabilidad continuo (pendiente, rugosidad, escalón) para el planificador | `controller` (`grid_planner.cpp`) | coste blando fundamentado |
| 5 | Clasificar vegetación frente a sólido | `residual_concept` | hojas que bloquean pasillos |
| 6 | Concepto de hilera de cultivo con la estructura de las hileras | `crop_row_concept` (nuevo) | guiado dentro del cultivo |

**Estado actual** (2026-10-01): nada de esto está implementado. Hoy el Husky navega en
`husky_test.wbt` (suelo llano) con el plano de suelo en z = 0, la rejilla de ocupación de residual y
el coste de holgura lineal del planificador. Ver `plan.md`, sección F5.
