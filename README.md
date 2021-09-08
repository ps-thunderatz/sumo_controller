# 🤼 Sumo Controller 🤖

Esse é um exemplo de um projeto simples de ROS feito em Python, que tem a função de controlar um robô de sumô.

## Índice<!-- omit in toc -->

- [🎈 Introdução](#-introdução)
- [📂 Arquivos no projeto](#-arquivos-no-projeto)
- [🔨 Como executar](#-como-executar)
- [📚 Como utilizar as bibliotecas](#-como-utilizar-as-bibliotecas)
  - [🔦 Sensores de distância](#-sensores-de-distância)
  - [➖ Sensores de linha](#-sensores-de-linha)
  - [⚖️ IMU](#%EF%B8%8F-imu)
  - [🚦 Estado da Partida](#-estado-da-partida)
  - [🏎️ Motores](#️-motores)
- [📖 Estratégias](#-estratégias)

## 🎈 Introdução

A partir desse repositório que você construirá o código de controle do seu primeiro robô de sumô simulado! Lembre-se de baixar (ou clonar) no mesmo workspace catkin que está o repositório da [simulação do Gazebo](https://github.com/ps-thunderatz/sumo_simulation).

## 📂 Arquivos no projeto

- **src/**
  - **sumo_controller_node.py** - Arquivo onde a lógica de controle do robô será implementada. É nesse arquivo que você e seu grupo deverão escrever o código do robô.
  - **example_node.py** - Arquivo com um exemplo de node que pode ser usado para testes mais simples.
  - **utils/** - Pasta com bibliotecas para auxiliar no desenvolvimento do projeto.
- **launch/** - Pasta com os arquivos que executam o projeto.
- **config/line_sensor_param.yaml** - Arquivo para calibração do sensor de linha.
- **CMakeLists.txt** e **package.xml** - Arquivos de configuração do pacote ROS. **Não modifique!**

## 🔨 Como executar

Antes de executar o código de controle, é preciso que a [simulação do Gazebo](https://github.com/ps-thunderatz/sumo_simulation) esteja rodando. Depois disso, basta executar o comando:

```bash
roslaunch sumo_controller sumo_controller.launch
```

A cor padrão do time é `"blue"`, mas é possível alterá-la para `"red"` com o parâmetro `team`.

```bash
roslaunch sumo_controller sumo_controller.launch team:=red
```

Na pasta source também existe um node de exemplo `example_node.py`, que precisa ter os nomes dos tópicos corrigidos para funcionar. Para executar o seu node de controle contra o exemplo, execute o comando:

```bash
roslaunch sumo_controller example.launch
```

## 📚 Como utilizar as bibliotecas

Dentro da pasta **src/utils/**, existem alguns módulos de Python para facilitar o desenvolvimento do código de controle do robô. A seguir, você encontrará uma breve descrição de como usar cada um deles.

### 🔦 Sensores de distância

Para utilizar a biblioteca dos sensores de distância, primeiro faça o import da classe `DistanceSensor`, disponível no módulo `utils.distance_sensor`.

```python
from utils.distance_sensor import DistanceSensor
```

Em seguida, crie uma variável do tipo `DistanceSensor`, especificando o tópico do sensor.

```python
my_distance_sensor = DistanceSensor("topico/do/sensor/de/distancia")
```

Para ler o último valor de distância obtido pelo sensor, utilize o método `get_range()`.

```python
range_reading = my_distance_sensor.get_range()
```

Para obter os valores mínimos e máximos da leitura do sensor de distância, utilize a função `get_limits()`.

```python
min_range, max_range = my_distance_sensor.get_limits()
```

Dica: O robô possui 3 sensores de distância, então se quiser aproveitar o máximo das informações disponíveis, você precisará criar mais de um objeto `DistanceSensor`.

### ➖ Sensores de linha

Para utilizar a biblioteca dos sensores de linha, primeiro faça o import da classe `LineSensor`, disponível no módulo `utils.line_sensor`.

```python
from utils.line_sensor import LineSensor
```

Em seguida, crie uma variável do tipo `LineSensor`, especificando o tópico do sensor.

```python
my_line_sensor = LineSensor("topico/do/sensor/de/linha")
```

Para descobrir se o sensor está lendo uma linha branca, utilize o método `is_on_line()`.

```python
if my_line_sensor.is_on_line():
    # ...
```

Dica: O robô possui 2 sensores de linha, então se quiser aproveitar o máximo das informações disponíveis, você precisará criar mais de um objeto `LineSensor`.

### ⚖️ IMU

Para utilizar a biblioteca dos sensores de linha, primeiro faça o import da classe `ImuSensor`, disponível no módulo `utils.imu_sensor`.

```python
from utils.imu_sensor import ImuSensor
```

Em seguida, crie uma variável do tipo `ImuSensor`, especificando o tópico do sensor.

```python
my_imu_sensor = ImuSensor("topico/do/sensor/imu")
```

Para ler o último valor de velocidade angular obtido pelo sensor, utilize o método `get_angular_velocity()`.

```python
angular_velocity = my_imu_sensor.get_angular_velocity()
```

Esse valor é do tipo [Vector3](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html), e possuí componentes x, y e z. No caso do sumô, a mais importante é a componente z, que representa rotação no plano do dojô. Ela pode ser acessada através de `angular_velocity.z`.


Para ler o último valor de aceleração linear obtido pelo sensor, utilize o método `get_linear_acceleration()`.

```python
linear_acceleration = my_imu_sensor.get_linear_acceleration()
```

Esse valor também é do tipo [Vector3](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html) e portanto, é possível acessar seus atributos da mesma forma.

Dica: Esses valores de velocidade angular e aceleração linear podem ser acumulados ao longo do tempo para se obter outras informações, como a orientação, velocidade e posição do robô no dojô.

### 🚦 Estado da Partida

Para utilizar a biblioteca de leitura do estado da partida, primeiro faça o import da classe `MatchState`, disponível no módulo `utils.match_state`.

```python
from utils.match_state import MatchState
```

Em seguida, crie uma variável do tipo `MatchState`, especificando o tópico do estado da partida.

```python
match_state = MatchState("topico/do/estado/da/partida")
```

Para saber se a partida iniciou, utilize o método `started()`.

```python
if match_state.started():
    # ...
```

### 🏎️ Motores

Para utilizar a biblioteca dos sensores de distância, primeiro faça o import da classe `Motors`, disponível no módulo `utils.motors`.

```python
from utils.motors import Motors
```

Em seguida, crie uma variável do tipo `Motor`, especificando o tópico de cada motor.

```python
motors = Motors("topico/do/motor/esquerdo", "topico/do/motor/direito")
```

Por fim, os comandos para os motores podem ser enviados por meio da função `drive()`, que recebe como parâmetro dois números inteiros de **-100** (força total de ré) até **100** (força total para frente).

```python
motors.drive(80, 80)
```

## 📖 Estratégias

No seu código poderão existir diversos comportamentos diferentes para o robô, como:
- Atacar pela esquerda
- Atacar pela direita
- Aguardar ataque

E muitos outros possíveis! De forma que a estratégia que o robô irá seguir será definida somente na hora de executar o código, através da adição de um argumento extra no roslaunch do seu node de controle.

```bash
roslaunch sumo_controller sumo_controller.launch strategy:=some_strategy
```

Ou no roslaunch do seu node contra o exemplo.

```bash
roslaunch sumo_controller example.launch strategy:=some_strategy
```

Esse valor pode ter qualquer nome que você definir, e para obtê-lo no código, utilize o método `rospy.get_param()`.

```python
strategy = rospy.get_param("strategy")
```
