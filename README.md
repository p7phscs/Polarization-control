# Polarization-control
 
Este repositório contém o código para controle de polarização utilizando **Raspberry Pi**, **ADS1115** para leitura de detectores e **MPC320** para controle dos pedais de polarização. O código oferece uma série de funcionalidades, como aquisição de dados em tempo real, controle de movimento dos pedais com compensação, e cálculos de parâmetros de Stokes (S1, S2).

## Índice

1. [Requisitos](#requisitos)
2. [Como Usar](#como-usar)
   - [Comando `serie`](#comando-serie)
   - [Cimando `serie compensation`](#comando-serie_compensation)
   - [Comando `power_impactdet`](#comando-power_impactdet)
   - [Comando `power_impact_geral`](#comando-power_impact_geral)
   - [Comando `move`](#comando-move)
3. [Funções Internas](#funções-internas)
   - [Função `acquisition_thread_func`](#função-acquisition_thread_func)
   - [Função `control_thread_func`](#função-control_thread_func)
   - [Função `compute_stokes_from_vals`](#função-compute_stokes_from_vals)
   - [Função `compute_control_score`](#função-compute_control_score)
4. [Instalação e Dependências](#instalação-e-dependências)
5. [Licença](#licença)

---

## Requisitos

- **Raspberry Pi** com I2C habilitado.
- **ADS1115** (ou similar) para leitura de sinais de tensão.
- **MPC320** da Thorlabs para controle dos pedais de polarização.
- Bibliotecas Python:
  - `qmi` para controle do MPC320.
  - `ADS1x15-ADC` para comunicação com o ADS1115.
  - `numpy` para manipulação de arrays e cálculos.
  - `matplotlib` para gráficos (opcional).

Instale as dependências necessárias com o seguinte comando:

```bash
pip install ADS1x15-ADC qmi numpy matplotlib
```
## Comando `serie`

Descrição detalhada sobre como usar o comando `serie`.

### Parâmetros:
- `--duration`: Tempo da execução da série temporal em segundos.
- `--compensation`: Ativa a compensação do pedal.
- `--minimize`: Define qual detector minimizar, ex: `h`, `v`, `d`, `a`.
- `--active_paddles`: Define quais pedais podem atuar, ex: `1 2 3`.
por definição, caso não solicitado o comando acima, todos os pedais atuarão. 
## Comando `serie compensation`

Descrição detalhada sobre o comando `serie` com compensação.

### Parâmetros:
- `--compensation`: Ativa a compensação do pedal.
- `--minimize`: Define qual detector minimizar.
- `--settle_s`: Define o tempo de espera da medição para compensação.

## Comando `power_impact_geral`

Este comando realiza uma varredura geral para calcular o impacto de todos os detectores (H, V, D, A, S1, S2).

### Parâmetros:
- `--step`: Define o passo em graus para a varredura.
- `--dwell`: Tempo de espera após cada movimento.
- `--save_csv`: Salva os resultados da varredura em um arquivo CSV.

## Comando `move`

Este comando move um pedal específico do MPC320 para uma posição definida em graus.

### Parâmetros:
- `--paddle`: Define qual pedal mover (1, 2 ou 3).
- `--pos`: Define a posição de destino em graus.

