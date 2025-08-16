# 🔒 Projeto Tranca Eletrônica RFID (ESP32)

Este projeto consiste em uma tranca eletrônica com autenticação via **tags/cartões RFID**, utilizando uma **ESP32**, um **leitor RFID MFRC522** e uma **fechadura eletrônica 12V**.

A lógica é simples: ao aproximar uma tag RFID do leitor, o sistema verifica se a tag é válida através de um broker se comunicando com uma API externa.  
- Se for **válida**, a fechadura é destrancada.  
- Se **inválida ou não cadastrada**, o acesso é negado.

> ⚠️ **Este projeto funciona em conjunto com a API:** [Projeto Tranca API](<https://github.com/MatheusH2021/API-TRANCA.git>)  
> Para funcionamento completo, clone e configure o repositório da API também!

---

## 🚀 Tecnologias Utilizadas

- ESP-IDF (framework oficial da Espressif)
- Linguagem C
- Protocolo MQTT 5

---

## 🔌 Componentes Utilizados

- ESP32 WROOM-32
- [Leitor RFID + Tag e Cartão (MFRC522)](https://www.eletrogate.com/kit-modulo-rfid-mfrc522-1356-mhz)
- [Buzzer Ativo 5V](https://www.eletrogate.com/buzzer-ativo-5v)
- LEDs difusos 3mm (Azul, Verde, Vermelho) + resistores
- [Relé 1 Canal 5V](https://www.eletrogate.com/modulo-rele-1-canal-5v)
- Jumpers macho-fêmea
- Conversor de tensão
- [Display LCD 16x2 com I2C](https://www.eletrogate.com/display-lcd-16x2-i2c-backlight-azul)
- [Fechadura Eletrônica 12V](https://www.eletrogate.com/fechadura-eletronica-mk-202-12v)
- Fonte 12V 10A
- [Protoboard 830 pontos](https://www.eletrogate.com/protoboard-830-pontos)

---

## ⚙️ Conexões dos Componentes com a ESP32

### 📟 Leitor RFID MFRC522
- SDA (CS) → GPIO 5  
- SCK → GPIO 18  
- MOSI → GPIO 23  
- MISO → GPIO 19  
- RST → GPIO 4  
- GND → GND Direito  
- 3V3 → 3V3

### 🔔 Buzzer Ativo
- Positivo → GPIO 15  
- Negativo → GND Esquerdo

### 💡 LEDs
- Azul → GPIO 14  
- Verde → GPIO 13  
- Vermelho → GPIO 25

### 📺 Display LCD 16x2 (I2C)
- SDA → GPIO 21  
- SCL → GPIO 22

### 🔌 Relé 1 canal 5v
- VIN → GPIO 32

## 🔋 Alimentação Elétrica do Sistema

Neste projeto, a alimentação é feita com uma **fonte de 12V 10A**, que supre toda a corrente necessária para a tranca e demais componentes.

### 🔧 Funcionamento da Distribuição de Energia:

- A **fonte 12V** é conectada diretamente a:
  - Um **conversor de tensão (step-down)**, que **reduz de 12V para 5V**.
  - A saída de 5V do conversor é usada para alimentar:
    - ESP32  
    - Leitor RFID  
    - Relé  
    - Display LCD  
    - LEDs  
    - Buzzer  

- A **fechadura eletrônica 12V** recebe os **12V diretamente da fonte**, mas **só é energizada quando o relé é acionado**.

- Também é utilizado um **diodo 1N5408** entre a **saída de 12V do relé e a tranca eletrônica**, com o objetivo de:
  - **Impedir o retorno de corrente** para os outros componentes do circuito após a tranca ser desativada.
  - **Proteger o circuito** contra picos ou retorno indesejado de tensão, aumentando a segurança da alimentação.

> ⚠️ O relé atua como uma chave controlada pela ESP32, quando o acesso é autorizado, ele fecha o circuito e permite que os 12V cheguem à tranca.

> 💡Também foi utilizado uma protoboard de 830 pinos, facilitando a conexão dos componentes entre si e a alimentação elétrica.
---

### 🧭 Esquema Resumido

```
[Fonte 12V] ─┬─> [Conversor Step-down] ──> [ESP32 + Componentes (5V)]
             │
             └─> [COM do Relé] ──> [Diodo 1N5408] ──> [Tranca 12V]
```
Essa abordagem garante que todos os componentes sejam alimentados corretamente e com segurança, sem sobrecarregar a ESP32 ou correr risco de retorno de corrente.

## ⚙️ Esquema ilustrativo do projeto
---

<img width="900" height="500" alt="Captura de tela 2025-08-12 102354" src="https://github.com/user-attachments/assets/842efbcc-3c60-41ab-91b5-00ac60d8ea87" />

> 💡 A linha de energia da protoboard do lado de baixo é alimentada com 12v, e a linha de cima é alimentada pelo coversor MP1584 com 5v

> 💡 No esquema a fechadura 12v não está presente, mas ela também faz parte do projeto, conectada o GND a linha inferior negativa da protoboard e o positivo conectado a saida NC do relé. A entrada COM do relé também recebe os 12v.

> 💡 Leitor rfid MFRC522 e conversor MP1584 presentes de forma ilustrativa, pois a plataforma não possuia os componentes expecíficos para montagem do esquema.
---

## 📡 Configuração do Broker MQTT (Mosquitto)

Para este projeto, foi utilizado um **broker MQTT local**, utilizando o [Mosquitto](https://mosquitto.org/). Se você deseja seguir esse mesmo modelo, siga os passos abaixo:

### 🧩 Instalação do Mosquitto

1. **Baixe e instale o Mosquitto:**

   - Acesse: https://mosquitto.org/download/
   - Baixe a versão adequada para o seu sistema operacional.
   - Instale normalmente (em Windows, marque a opção para instalar como serviço, se desejar).

2. **Configure o broker:**

   - Após a instalação, localize o arquivo `mosquitto.conf` na **pasta raiz de instalação** (exemplo: `C:\Program Files\Mosquitto\`).

   - Edite o arquivo `mosquitto.conf` e adicione as seguintes configurações ao final:

     ```conf
     listener 1883
     allow_anonymous true
     ```

   - Salve o arquivo após a edição.

### ▶️ Iniciando o Broker

Para iniciar o Mosquitto com o arquivo de configuração personalizado, utilize o seguinte comando no terminal ou prompt de comando:

```bash
mosquitto -c "C:\caminho\para\mosquitto.conf" -v
```

> 🔁 O parâmetro `-v` ativa o modo verboso, permitindo ver as mensagens trocadas no broker (útil para testes e depuração).

---

### ⚠️ Considerações Importantes

- A **ESP32**, a **API da tranca** e o **broker MQTT** **devem estar na mesma rede local**, pois trata-se de um broker local sem exposição externa.
- Tanto a **tranca eletrônica (ESP32)** quanto a **API** precisam se conectar a esse mesmo broker para que a comunicação funcione corretamente.
- Pode ser necessário **permitir a porta `1883` no firewall** do seu sistema para que as conexões MQTT funcionem corretamente.

> 💡 Se desejar, você pode usar um broker MQTT público ou hospedado na nuvem, como o HiveMQ, EMQX ou Mosquitto hospedado remotamente. Nesse caso, lembre-se de configurar as credenciais e URLs nos arquivos do projeto.

---

## 🛠️ Como Rodar o Projeto

1. **Clone ou baixe o repositório:**

```bash
git clone https://github.com/MatheusH2021/tranca-eletronica-rfid.git
```

2. **Configure o ambiente ESP-IDF:**

- Instale o ESP-IDF na sua máquina (recomendado utilizar a extensão do VSCode).
- Siga os passos oficiais: [Instalação do ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)

3. **Compile o projeto:**

```bash
idf.py build
```

4. **Configure a rede Wi-Fi:**

- Acesse o arquivo `sdkconfig` e edite as seguintes variáveis:

```text
CONFIG_EXAMPLE_WIFI_SSID="nome_da_sua_rede_wifi"
CONFIG_EXAMPLE_WIFI_PASSWORD="senha_da_sua_rede"
```

> 💡 Alternativamente, você pode usar o menuconfig do ESP-IDF para configurar:

```bash
idf.py menuconfig
```
- Vá em `Example Configuration` e insira sua SSID e senha.

5. **Configure o broker MQTT:**

- No arquivo `main/app_main.c`, nas primeiras linhas, localize e edite a variável:

```c
#define MQTT_BROKER_ADDR "mqtt://<SEU_BROKER>"
```

> Substitua `<SEU_BROKER>` pelo endereço do seu broker MQTT (deve ser o mesmo usado na API).

6. **Conecte a ESP32 ao computador e execute os comandos:**

```bash
idf.py build         # Compila o projeto
idf.py flash         # Realiza o upload do firmware para a ESP32
idf.py monitor       # Inicia o monitor serial para visualizar os logs
```

> Se estiver usando o VSCode com ESP-IDF:
- Clique nos ícones "Build", "Flash" e "Monitor" na barra inferior da extensão.

---

## 🧪 Como Testar o Projeto

Após seguir os passos acima, com a API da tranca **ativa** e o **broker MQTT funcionando**, a ESP32 irá:

1. Exibir no LCD:
   - "Conectando ao Wi-Fi"
   - "Conectando ao broker MQTT"
   - "Aproxime Tag ou Cartao RFID"

2. Ao aproximar uma tag/crachá:
   - LED azul pisca 2x
   - Se **cadastrada e ativa**:
     - LED verde pisca 2x
     - Buzzer emite 1 bip
     - Relé é ativado → tranca liberada
     - LCD: `UID: [UID]` e `Acesso Permitido`

   - Se **não cadastrada ou inativa**:
     - LED vermelho pisca 2x
     - Buzzer emite 2 bips
     - Tranca permanece fechada
     - LCD: `UID: [UID]` e `Acesso Negado`

   - Se a leitura for muito rápida:
     - LED vermelho pisca 2x
     - Buzzer emite 3 bips
     - LCD: `Mantenha próximo por mais tempo`

---

## 🔄 Funcionamento Interno

1. ESP32 lê a tag RFID.
2. Publica o UID no tópico MQTT: `esp32/tranca/autenticar`.
3. A API responde no tópico `esp32/tranca/desbloquear`:
   - `"1"` → Acesso liberado.
   - `"0"` → Acesso negado.
4. A ESP32 aciona os atuadores e atualiza o display LCD conforme a resposta.

---
