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
