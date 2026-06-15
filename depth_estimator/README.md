# depth_estimator — ROS2 Humble

Nó wrapper que recebe imagens de câmera e publica estimativa de profundidade monocular usando **Depth Anything V2** (HuggingFace).

---

## Arquitetura

```
/camera/image_raw  (sensor_msgs/Image)
        │
        ▼
 DepthEstimatorNode
        │
        ├──▶  /depth/image_raw     (sensor_msgs/Image, 32FC1)   ← depth em ponto flutuante
        ├──▶  /depth/image_visual  (sensor_msgs/Image, rgb8)    ← colormap INFERNO
        └──▶  /depth/camera_info   (sensor_msgs/CameraInfo)     ← repasse do camera_info
```

---

## Dependências Python

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install transformers accelerate pillow opencv-python-headless
```

> Para CPU somente: `pip install torch torchvision` (sem o `--index-url`).

---

## Build

```bash
# Na raiz do workspace ROS2
cp -r depth_estimator/ src/
colcon build --packages-select depth_estimator
source install/setup.bash
```

---

## Execução

### Forma direta
```bash
ros2 run depth_estimator depth_estimator_node \
  --ros-args \
  -p input_topic:=/camera/image_raw \
  -p device:=cuda \
  -p publish_visual:=true
```

### Via launch file
```bash
ros2 launch depth_estimator depth_estimator.launch.py
```

### Com argumentos customizados
```bash
ros2 launch depth_estimator depth_estimator.launch.py \
  device:=cpu \
  model_name:=depth-anything/Depth-Anything-V2-Large-hf \
  input_topic:=/drone/front_camera/image_raw \
  max_depth:=20.0
```

### Via arquivo de configuração
```bash
ros2 run depth_estimator depth_estimator_node \
  --ros-args --params-file config/depth_estimator.yaml
```

---

## Parâmetros

| Parâmetro           | Tipo   | Padrão                                           | Descrição                          |
|---------------------|--------|--------------------------------------------------|------------------------------------|
| `model_name`        | string | `depth-anything/Depth-Anything-V2-Small-hf`      | ID HuggingFace ou caminho local    |
| `device`            | string | `cuda`                                           | `cuda` ou `cpu`                    |
| `input_topic`       | string | `/camera/image_raw`                              | Tópico de imagem de entrada        |
| `camera_info_topic` | string | `/camera/camera_info`                            | Tópico CameraInfo (opcional)       |
| `output_topic`      | string | `/depth/image_raw`                               | Depth em 32FC1                     |
| `visual_topic`      | string | `/depth/image_visual`                            | Visualização colorida em rgb8      |
| `publish_visual`    | bool   | `True`                                           | Habilita publicação visual         |
| `queue_size`        | int    | `5`                                              | Tamanho das filas de mensagens     |
| `min_depth`         | float  | `0.1`                                            | Profundidade mínima visual (m)     |
| `max_depth`         | float  | `10.0`                                           | Profundidade máxima visual (m)     |

---

## Checkpoints disponíveis

| Model ID (HuggingFace)                          | Velocidade  | Precisão |
|-------------------------------------------------|-------------|----------|
| `depth-anything/Depth-Anything-V2-Small-hf`     | ~25 ms/GPU  | boa      |
| `depth-anything/Depth-Anything-V2-Base-hf`      | ~45 ms/GPU  | melhor   |
| `depth-anything/Depth-Anything-V2-Large-hf`     | ~90 ms/GPU  | melhor   |

> O modelo **Small** é o recomendado para uso em tempo real (≥ 30 FPS @ RTX 3060+).

---

## Verificação rápida

```bash
# Verificar tópicos publicados
ros2 topic list | grep depth

# Conferir tipo e frequência
ros2 topic info /depth/image_raw
ros2 topic hz   /depth/image_raw

# Visualizar no RViz2
rviz2  # Add → By topic → /depth/image_visual → Image
       # Add → By topic → /depth/image_raw    → Image (depth mode)
```

---

## Estrutura do pacote

```
depth_estimator/
├── depth_estimator/
│   ├── __init__.py
│   └── depth_estimator_node.py   ← nó principal
├── launch/
│   └── depth_estimator.launch.py
├── config/
│   └── depth_estimator.yaml
├── package.xml
├── setup.py
└── README.md
```

---

## Extensão / Troca de modelo

Para usar outro backbone (ex.: MiDaS, ZoeDepth, ONNX custom), substitua apenas a classe `DepthBackend` em `depth_estimator_node.py`. A interface esperada é:

```python
class DepthBackend:
    def __init__(self, model_name: str, device: str): ...
    def predict(self, bgr_image: np.ndarray) -> np.ndarray:
        """Retorna depth map float32 (H x W)."""
        ...
```
