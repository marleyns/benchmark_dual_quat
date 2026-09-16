# benchmark_dual_quat
Repositorio que contiene los scripts para el @benchmark de matrices homogéneas y cuaterniones duales en FK  para el cobot UR10.
# Scripts de Cinemática Directa y Cuaterniones Duales – UR10

Repositorio complementario del estudio sobre la equivalencia entre matrices de transformación homogénea y cuaterniones duales unitarios aplicados a la cinemática directa del manipulador UR10.

## Scripts incluidos

| Archivo                    | Lenguaje | Descripción breve                          |
|---------------------------|----------|--------------------------------------------|
| `download-FK.py`          | Python   | Extracción de datos de cinemática directa desde RoboDK |
| `dualquat_composicion.jl` | Julia    | Construcción y manipulación de cuaterniones duales     |

---

### 1. Script Python (`download-FK.py`)

Para la obtención de los datos de la cinemática directa (FK) del UR10 se empleó el API de Python de RoboDK. El marco de referencia utilizado es “UR10 Base”. Mediante la función `PoseWrt()` se recuperó la pose relativa entre la base del robot y la brida. Paralelamente, se extrajeron los ángulos articulares con `robot.Joints()` (convertidos a radianes) para construir las seis matrices de transformación homogénea locales. Estas matrices se contrastaron con el resultado de `robot.SolveFK()` para validar la coherencia de la cadena cinemática. Dado que la estación no cuenta con herramienta (TCP) conectada, las poses corresponden a la brida del manipulador. Finalmente, con `pose_2_quaternion()` se obtuvo el cuaternión de rotación `[qw, qx, qy, qz]` y el vector de traslación `[x, y, z]`, datos indispensables para la posterior construcción del cuaternión dual unitario en Julia.

**Requisitos:**
- RoboDK instalado y en ejecución
- API de Python de RoboDK configurada
- Estación con el robot UR10 cargado

**Ejecución:**
```bash
python download-FK.py


### 2. Script Julia (`dualquat_composicion.jl`)

La implementación se realizó en el REPL de Julia, previa carga de las librerías `Quaternions.jl` y `ForwardDiff.jl`. Se definieron las funciones `const DualQuaternion{T}`, `motodual_from_quat(q_r, t)`, `motodual(T)` y `dualmoto(H)`. 

La estructura `DualQuaternion{T}` representa un cuaternión dual cuyas cuatro componentes (una escalar y tres imaginarias) son números duales del tipo `ForwardDiff.Dual{Nothing, T, 1}`. Cada número dual almacena un valor primal y una parte tangencial, de modo que la regla del producto propaga automáticamente la información de traslación. 

La función `motodual_from_quat(q_r, t)` construye el cuaternión dual directamente a partir del cuaternión de rotación y el vector de traslación obtenidos desde RoboDK, sin necesidad de pasar por una matriz homogénea 4×4. Las funciones `motodual(T)` y `dualmoto(H)` establecen el homomorfismo entre ambas representaciones, permitiendo convertir matrices homogéneas en cuaterniones duales y viceversa. Estas tres últimas funciones fueron desarrolladas por el autor como complemento metodológico del estudio.

#### Benchmarks de rendimiento

Con el fin de comparar el costo computacional de ambas representaciones, se definieron los siguientes benchmarks utilizando la librería `BenchmarkTools.jl`:

- **Composición de matrices homogéneas:**
  ```julia
  @benchmark($T_1 * $T_2 * $T_3 * $T_4 * $T_5 * $T_6)
