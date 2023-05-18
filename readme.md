# Reator CSRT - PID Nível e Temperatura

Este reator foi desenvolvido para estudos de controle PID. O equipamento é composto por sensores para o controle de nível e temperatura, os quais controlam a entrada e saída de fluidos em diferentes temperaturas.

O equipamento está pronto para uso, bastando alimentar o painel principal com uma fonte de 110VCA e conectar o cabo USB de interface em um carregador comum de 5VDC ou diretamente em um computador, onde é possível visualizar as informações.

## Instalação

### Baixando e instalando o Arduino

Neste projeto utilizamos a IDE Legacy (1.8.X). Mais informações sobre a instalação podem ser encontradas [aqui](https://www.arduino.cc/en/software)

### Arquivos do projeto

Este projeto consiste em dois módulos ESP posicionados no reator e em um painel de força. Cada um dos módulos possui um programa específico, mas para a usabilidade normal, basta baixar o código do "esp_painel".


## Uso

Após instalar o Arduino e baixar o arquivo do programa "esp_painel", é possível alterar os setpoints e as constantes dos PIDs.

### Uso através do protocolo MQTT

Após se conectar à ao broker mqtt (broker.hivemq.com). O reator pode ser controlado através dos tópicos apontados na tabela abaixo.

|                 **Tópico**                |                                             **Descrição**                                            |
|:-----------------------------------------:|:----------------------------------------------------------------------------------------------------:|
| UTFPR/REATOR_PID/SP_ALTURA                | Define o valor de referência (setpoint) para a altura no reator.                                     |
| UTFPR/REATOR_PID/OFFSET_ALTURA            | Define o desvio (offset) para a altura no reator.                                                    |
| UTFPR/REATOR_PID/PWM_BOMBA_06_NIVEL       | Controla a potência do sinal PWM para a bomba 06, com o objetivo de regular o nível do reator.       |
| UTFPR/REATOR_PID/SP_TEMPERATURA           | Define o valor de referência (setpoint) para a temperatura no reator.                                |
| UTFPR/REATOR_PID/PWM_BOMBA_09_TEMPERATURA | Controla a potência do sinal PWM para a bomba 09, com o objetivo de regular a temperatura do reator. |
| UTFPR/REATOR_PID/RPM_MOTOR_PASSO          | Define a velocidade de rotação do motor de passo.                                                    |
| UTFPR/REATOR_PID/KP_NIVEL                 | Define o coeficiente proporcional (KP) para o controle de nível do reator.                           |
| UTFPR/REATOR_PID/KI_NIVEL                 | Define o coeficiente integral (KI) para o controle de nível do reator.                               |
| UTFPR/REATOR_PID/KD_NIVEL                 | Define o coeficiente derivativo (KD) para o controle de nível do reator.                             |
| UTFPR/REATOR_PID/KP_TEMPERATURA           | Define o coeficiente proporcional (KP) para o controle de temperatura do reator.                     |
| UTFPR/REATOR_PID/KI_TEMPERATURA           | Define o coeficiente integral (KI) para o controle de temperatura do reator.                         |
| UTFPR/REATOR_PID/KD_TEMPERATURA           | Define o coeficiente derivativo (KD) para o controle de temperatura do reator.                       |
| UTFPR/REATOR_PID/MODO_CONTROLE            | Define o modo de controle do reator (por exemplo, PID, ON-OFF, etc.).                                |

As informações provenientes do reator também podem ser acessadas através dos tópicos a seguir, bastando apenas se cadastrar nos tópicos para acessar.

|                  **Tópico**                  |                                        **Descrição**                                        |
|:--------------------------------------------:|:-------------------------------------------------------------------------------------------:|
| UTFPR/REATOR_PID/ALTURA                      | Valor atual da altura no reator.                                                            |
| UTFPR/REATOR_PID/SP_ALTURA_VA                | Valor atual do setpoint para a altura no reator.                                            |
| UTFPR/REATOR_PID/PWM_BOMBA_06_NIVEL_VA       | Valor atual do sinal PWM para a bomba 06, utilizado para controlar o nível do reator.       |
| UTFPR/REATOR_PID/TEMPERATURA                 | Valor atual da temperatura no reator.                                                       |
| UTFPR/REATOR_PID/SP_TEMPERATURA_VA           | Valor atual do setpoint para a temperatura no reator.                                       |
| UTFPR/REATOR_PID/PWM_BOMBA_09_TEMPERATURA_VA | Valor atual do sinal PWM para a bomba 09, utilizado para controlar a temperatura do reator. |
| UTFPR/REATOR_PID/NIVEL_MIN                   | Valor mínimo para o nível do reator.                                                        |
| UTFPR/REATOR_PID/NIVEL_MAX                   | Valor máximo para o nível do reator.                                                        |
| UTFPR/REATOR_PID/RPM_MOTOR_PASSO_VA          | Valor atual das rotações por minuto (RPM) do motor de passo.                                |
| UTFPR/REATOR_PID/KP_NIVEL_VA                 | Valor atual do coeficiente proporcional (KP) para o controle de nível do reator.            |
| UTFPR/REATOR_PID/KI_NIVEL_VA                 | Valor atual do coeficiente integral (KI) para o controle de nível do reator.                |
| UTFPR/REATOR_PID/KD_NIVEL_VA                 | Valor atual do coeficiente derivativo (KD) para o controle de nível do reator.              |
| UTFPR/REATOR_PID/KP_TEMPERATURA_VA           | Valor atual do coeficiente proporcional (KP) para o controle de temperatura do reator.      |
| UTFPR/REATOR_PID/KI_TEMPERATURA_VA           | Valor atual do coeficiente integral (KI) para o controle de temperatura do reator.          |
| UTFPR/REATOR_PID/KD_TEMPERATURA_VA           | Valor atual do coeficiente derivativo (KD) para o controle de temperatura do reator.        |

## Contribuições

Este projeto contou com a contribuição de vários alunos do curso de Engenharia Química da UTFPR - Ponta Grossa. Uma lista será feita e apresentada aqui.

## Licença

[MIT](https://choosealicense.com/licenses/mit/)