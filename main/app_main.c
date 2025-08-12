/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_lcd_io_i2c.h"
#include <stdbool.h>

#define MISO_PIN_NUM 19
#define MOSI_PIN_NUM 23
#define SCK_PIN_NUM 18
#define CS_PIN_NUM 5

#define BUZZER_PIN_NUM 15
#define RELE_PIN_NUM  32

#define GREEN_LED_PIN_NUM 13
#define RED_LED_PIN_NUM 25
#define BLUE_LED_PIN_NUM 14

#define I2C_BUS_PORT I2C_NUM_0
#define I2C_SDA_PIN_NUM  21
#define I2C_SCL_PIN_NUM  22

#define I2C_ADDR_LCD 0x27

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RS        0x01

#define CommandReg 0x01

static const char *TAG = "MFRC522";
spi_device_handle_t spi;

#define MQTT_BROKER_ADDR "mqtt://<SEU BROKER MQTT AQUI>" // Se for local seu broker será: mqtt://SeuIp:1883, se for online será a url de destino
#define MQTT_BROKER_TOPIC_AUT "esp32/tranca/autenticar"
#define MQTT_BROKER_TOPIC_UNLOCK "esp32/tranca/desbloquear"
#define MQTT_BROKER_TOPIC "esp32/tranca/geral"
#define LED_PIN_NUM 27

void blink_led(gpio_num_t pin, int time_ms) {
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
    gpio_set_level(pin, 0);
} 

void buzzer_bip(int times) {

    for (size_t i = 0; i < times; i++)
    {
        gpio_set_level(BUZZER_PIN_NUM, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(BUZZER_PIN_NUM, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

void activate_rele() {
    gpio_set_level(RELE_PIN_NUM, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(RELE_PIN_NUM, 0);
}

static esp_err_t lcd_write_4bits(uint8_t data) {
    uint8_t data_en = data | LCD_ENABLE | LCD_BACKLIGHT;
    uint8_t data_no_en = (data & ~LCD_ENABLE) | LCD_BACKLIGHT;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDR_LCD << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data_en, true);
    i2c_master_write_byte(cmd, data_no_en, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_BUS_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd_write_4bits erro: %d", ret);
    }
    return ret;
}

static esp_err_t lcd_send_command(uint8_t cmd) {
    esp_err_t ret;
    ret = lcd_write_4bits(cmd & 0xF0);
    if (ret != ESP_OK) return ret;
    ret = lcd_write_4bits((cmd << 4) & 0xF0);
    return ret;
}

static esp_err_t lcd_send_data(uint8_t data) {
    esp_err_t ret;
    uint8_t high = (data & 0xF0) | LCD_RS;
    uint8_t low = ((data << 4) & 0xF0) | LCD_RS;

    ret = lcd_write_4bits(high);
    if (ret != ESP_OK) return ret;
    ret = lcd_write_4bits(low);
    return ret;
}

void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(50));

    lcd_write_4bits(0x30);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_4bits(0x30);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_4bits(0x30);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_4bits(0x20);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_send_command(0x28); // 4 bits, 2 linhas, 5x8 dots
    lcd_send_command(0x0C); // display on, cursor off, blink off
    lcd_send_command(0x01); // clear display
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_command(0x06); // entry mode set

    ESP_LOGI(TAG, "LCD inicializado");
}

void lcd_write_string(const char *str) {
    while (*str) {
        lcd_send_data((uint8_t)(*str));
        str++;
    }
}

void lcd_set_cursor(uint8_t line, uint8_t column) {
    uint8_t pos = (line == 0) ? 0x00 + column : 0x40 + column;
    lcd_send_command(0x80 | pos);
}

void lcd_clear_lines() {
    lcd_set_cursor(0, 0);          
    lcd_write_string("                ");
    lcd_set_cursor(1, 0);
    lcd_write_string("                ");
}

// --- Final código LCD ---

uint8_t mfrc522_read(uint8_t reg) {
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = ((reg << 1) & 0x7E) | 0x80;
    tx_data[1] = 0x00;

    spi_transaction_t t = {
        .length = 8 * 2,     
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);

    return rx_data[1];
}

void mfrc522_write(uint8_t reg, uint8_t value) {
    uint8_t tx_data[2];

    // Bit 7 = 0 para escrita, bit 6–1 = endereço, bit 0 = 0
    tx_data[0] = ((reg << 1) & 0x7E);  // bit 7 = 0 -> escrita
    tx_data[1] = value;               // valor que queremos escrever

    spi_transaction_t t = {
        .length = 8 * 2,              // 2 bytes (em bits)
        .tx_buffer = tx_data,
        .rx_buffer = NULL,           // Não queremos receber nada
    };

    esp_err_t ret = spi_device_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);
}

void mfrc522_antenna_on() {
    uint8_t val = mfrc522_read(0x14);  // TxControlReg
    if (!(val & 0x03)) {
        mfrc522_write(0x14, val | 0x03); // Liga os bits 1 e 0
    }
}

void mfrc522_init() {
    // Soft reset
    mfrc522_write(CommandReg, 0x0F); // CommandReg = SoftReset
    vTaskDelay(pdMS_TO_TICKS(50));

    // Timer
    mfrc522_write(0x2A, 0x8D); // TModeReg
    mfrc522_write(0x2B, 0x3E); // TPrescalerReg
    mfrc522_write(0x2C, 0x00); // TReloadRegH
    mfrc522_write(0x2D, 0x1E); // TReloadRegL

    // Transmissão
    mfrc522_write(0x15, 0x40); // TxASKReg
    mfrc522_write(0x11, 0x3D); // ModeReg

    // Antena
    mfrc522_antenna_on();
}

bool mfrc522_request() {
    //Ajustando o BitFrameingReg para 7 bits
    mfrc522_write(0x0D, 0x07);

    //Limpando o FIFO
    mfrc522_write(0x0A, 0x80);

    //Escrevendo Comando no FIFO
    mfrc522_write(0x09, 0x26);

    // Limpa todos os flags de interrupção anteriores
    mfrc522_write(0x04, 0x7F); // ComIrqReg: clear all IRQ flags

    //Instruindo o MFRC522 a transmitir dados
    mfrc522_write(0x01, 0x0C);

    //Disparando o envio
    mfrc522_write(0x0D, 0x87);
    vTaskDelay(pdMS_TO_TICKS(15)); 

    // Esperar até o comando terminar ou dar timeout (max ~25ms)
    uint8_t irq;
    int tries = 25;
    do {
        vTaskDelay(pdMS_TO_TICKS(1)); // aguarda 1ms
        irq = mfrc522_read(0x04);     // ComIrqReg
        tries--;
    } while (!(irq & 0x30) && tries > 0);

    if (!(irq & 0x30)) {
        // ESP_LOGW(TAG, "Timeout na REQA.");
        return false;
    }

    // Verifica erro
    uint8_t error = mfrc522_read(0x06); // ErrorReg
    if (error & 0x1B) { // bits de erro
        ESP_LOGW(TAG, "Erro na comunicação REQA: 0x%02X", error);
        return false;
    }

    //Se a resposta do FIFO for menor que 2 bytes retorna falso
    uint8_t fifo_level = mfrc522_read(0x0A);
    if (fifo_level != 2) return false;

    //Se não lê a resposta
    uint8_t tag_type[2];
    tag_type[0] = mfrc522_read(0x09);
    tag_type[1] = mfrc522_read(0x09);

    //Printa o tipo da tag
    ESP_LOGI(TAG, "Tag detectada: Tipo = %02X %02X", tag_type[0], tag_type[1]);
    return true;

}

bool mfrc522_anticollision(uint8_t *uid, bool *statusMessage) {
    blink_led(BLUE_LED_PIN_NUM, 200);
    lcd_set_cursor(0, 0); // Move cursor para column 0 da linha desejada
    lcd_write_string("Scanneando.....");
    lcd_set_cursor(1, 0); // Move cursor para column 1 da linha desejada
    lcd_write_string("                ");
    vTaskDelay(pdMS_TO_TICKS(250));

    // Ajusta BitFramingReg: 8 bits válidos no último byte
    mfrc522_write(0x0D, 0x00);  // BitFramingReg = 0

    // Limpa FIFO
    mfrc522_write(0x0A, 0x80);  // FIFOLevelReg: Flush FIFO

    // Envia os dois comandos: anticollision nível 1
    mfrc522_write(0x09, 0x93);  // FIFODataReg
    mfrc522_write(0x09, 0x20);  // 0x20 = anticollision

    // Inicia comando Transceive
    mfrc522_write(0x01, 0x0C);  // CommandReg = Transceive

    // Seta o bit StartSend (bit 7) no BitFramingReg
    mfrc522_write(0x0D, 0x80);  // StartSend + 8 bits válidos
    vTaskDelay(pdMS_TO_TICKS(10));

    // Aguarda resposta com timeout
    int i = 25;
    uint8_t irq;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        irq = mfrc522_read(0x04);  // ComIrqReg
        i--;
    } while (!(irq & 0x30) && i > 0);

    if (!(irq & 0x30)) {
        ESP_LOGW(TAG, "Timeout na anticollision.");
        return false;
    }

    // Verifica erro
    uint8_t error = mfrc522_read(0x06);  // ErrorReg
    if (error & 0x1B) {
        ESP_LOGW(TAG, "Erro na anticollision: 0x%02X", error);
        return false;
    }

    
    // Verifica se tem 5 bytes no FIFO
    uint8_t fifo_level = mfrc522_read(0x0A);
    // ESP_LOGW(TAG, "FIFOLevel = %d", fifo_level);
    if (fifo_level != 5) {
        ESP_LOGW(TAG, "Dados insuficientes no FIFO: %d bytes", fifo_level);
        lcd_clear_lines();

        lcd_set_cursor(0, 0);
        lcd_write_string("Mantenha proximo");
        lcd_set_cursor(1, 0);
        lcd_write_string("por mais tempo...");

        buzzer_bip(3);

        blink_led(RED_LED_PIN_NUM, 100);
        vTaskDelay(pdMS_TO_TICKS(100));
        blink_led(RED_LED_PIN_NUM, 100);

        *statusMessage = false;

        vTaskDelay(pdMS_TO_TICKS(1000));

        return false;
    }

    // Le os 5 bytes: 4 bytes do UID e 1 byte do checksum
    uint8_t checksum = 0;
    for (int j = 0; j < 5; j++) {
        uid[j] = mfrc522_read(0x09);
        if (j < 4) checksum ^= uid[j];  // calcula checksum
    }

    // Verifica se checksum bate
    if (checksum != uid[4]) {
        ESP_LOGW(TAG, "Checksum inválido no UID.");
        return false;
    }

    blink_led(BLUE_LED_PIN_NUM, 200);

    return true;
}

void display_main_text(bool *statusMessage) {
    if (!*statusMessage) {
        lcd_clear_lines();

        lcd_set_cursor(0, 0);
        lcd_write_string("Aproxime Tag ou");
        lcd_set_cursor(1, 0);
        lcd_write_string("Cartao RFID");

        *statusMessage = true;
    } 
    
}

esp_mqtt_client_handle_t mqtt_client = NULL;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "user"},
        {"p", "password"}
    };

#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

static esp_mqtt5_disconnect_property_config_t disconnect_property = {
    .session_expiry_interval = 60,
    .disconnect_reason = 0,
};

static void print_user_property(mqtt5_user_property_handle_t user_property)
{
    if (user_property) {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count) {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK) {
                for (int i = 0; i < count; i ++) {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        const char *json_msg_conect = "{\"dispositivo\": \"esp32\", \"status\": \"conectado\"}";
        // Publica uma simples mensagem no broker em formato JSON de conexão ao broker (Informa que a esp32 foi conectada)
        msg_id = esp_mqtt_client_publish(client, MQTT_BROKER_TOPIC, json_msg_conect, 0, 1, 0);
        ESP_LOGI(TAG, "Mensagem de conexão enviada com sucesso, msg_id=%d", msg_id);

        // Inscreve a esp32 no topico "/ifpe/ads/embarcados/controle_diferenciado"
        msg_id = esp_mqtt_client_subscribe(client, MQTT_BROKER_TOPIC, 1);
        msg_id = esp_mqtt_client_subscribe(client, MQTT_BROKER_TOPIC_AUT, 1);
        msg_id = esp_mqtt_client_subscribe(client, MQTT_BROKER_TOPIC_UNLOCK, 1);
        ESP_LOGI(TAG, "Esp32 Inscrito no topico %s", MQTT_BROKER_TOPIC);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);

        // Gera a mensagem de inscrição ao tópico esolhido
        char json_msg_sub[256];  // Buffer para armazenar a string final
        snprintf(json_msg_sub, sizeof(json_msg_sub), "{\"dispositivo\": \"esp32\", \"mensagem\": \"inscrito em um topico\"}");

        // Publica uma mensagem no broker em formato JSON informando que a esp32 está inscrita no tópico
        msg_id = esp_mqtt_client_publish(client, MQTT_BROKER_TOPIC, json_msg_sub, 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_user_property(&disconnect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_disconnect_property(client, &disconnect_property);
        esp_mqtt5_client_delete_user_property(disconnect_property.user_property);
        disconnect_property.user_property = NULL;
        esp_mqtt_client_disconnect(client);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // Aqui realizamos a logica de gerenciamento da Fecharura
        // quando recebe 1- Destranca a fechadura
        // quando recebe 0- Significa que a tag lida ou não está cadastrada ou está desativada

        // Verifica se o topico recebido é igual ao tópico definido
        if (event->topic_len == strlen(MQTT_BROKER_TOPIC_UNLOCK) &&
            memcmp(event->topic, MQTT_BROKER_TOPIC_UNLOCK, event->topic_len) == 0) {
            
            // Loga o tópico e a mensagem recebida
            ESP_LOGI(TAG, "Mensagem recebida do tópico %.*s: %.*s",
                    event->topic_len, event->topic,
                    event->data_len, event->data);

            // Verifica se há pelo menos um byte de dado antes de acessar event->data[0]
            if (event->data_len > 0) {
                // Pega o primeiro caractere da mensagem (usado como comando)
                char cmd = event->data[0];

                // Analisa o comando usando switch-case 
                switch (cmd) {
                    case '1':
                        // Neste case, caso o comando seja 1- Destranca a fechadura
                        blink_led(GREEN_LED_PIN_NUM, 100);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        blink_led(GREEN_LED_PIN_NUM, 100);
                        buzzer_bip(1);
                        lcd_set_cursor(1,0);
                        lcd_write_string("Acesso Liberado ");
                        activate_rele();
                        break;
                    case '0':
                        blink_led(RED_LED_PIN_NUM, 100);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        blink_led(RED_LED_PIN_NUM, 100);
                        buzzer_bip(2);

                        lcd_set_cursor(1,0);
                        lcd_write_string("Acesso Negado   ");
                        break;
                    default:
                        // Caso o comando seja diferente de 1 ou 0, loga apenas "comando desconhecido"
                        ESP_LOGW(TAG, "Comando desconhecido: %c", cmd);
                        break;
                }
            } else {
                // Caso não tenha byte, Loga a mensagem de dados vazios
                ESP_LOGW(TAG, "Dados vazios recebidos no tópico.");
            }

        } else {
            ESP_LOGI(TAG, "Mensagem recebida de tópico não monitorado: %.*s",
                    event->topic_len, event->topic);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt5_app_start(void)
{
    // Configura as opções do protocolo MQTT
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = MQTT_BROKER_TOPIC,
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = MQTT_BROKER_ADDR, // Broker usado para conexão
        .session.protocol_ver = MQTT_PROTOCOL_V_5, // Versão do protocolo MQTT usado
        .session.last_will.topic = MQTT_BROKER_TOPIC, // Tópico a ser publicado a mensagem de desconexão inesperada
        .session.last_will.msg = "offline",         // Mensagem a ser publicada caso haja desconexão inesperada
        .session.last_will.msg_len = strlen("offline"), // Tamanho da mensagem
        .session.last_will.qos = 1,
    };

    lcd_set_cursor(0, 0);
    lcd_write_string("Conectando ao ");
    lcd_set_cursor(1, 0);
    lcd_write_string("Broker MQTT");
    vTaskDelay(pdMS_TO_TICKS(1000));

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt5_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt5_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    mqtt_client = esp_mqtt_client_init(&mqtt5_cfg);

    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(mqtt_client, &connect_property);

    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);

    esp_mqtt_client_start(mqtt_client);
}

void handle_send(const char* uid) {
    char json_msg_sub[256];
    snprintf(json_msg_sub, sizeof(json_msg_sub), "{\"uid\": \"%s\"}", uid);

    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_BROKER_TOPIC_AUT, json_msg_sub, 0, 1, 0);
    ESP_LOGI(TAG, "Mensagem de conexão enviada com sucesso, msg_id=%d", msg_id);
}

void app_main(void)
{
    // Configura GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_PIN_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << RELE_PIN_NUM);
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << GREEN_LED_PIN_NUM);
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << RED_LED_PIN_NUM);
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << BLUE_LED_PIN_NUM);
    gpio_config(&io_conf);

    // Inicializa barramento SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = MISO_PIN_NUM,
        .mosi_io_num = MOSI_PIN_NUM,
        .sclk_io_num = SCK_PIN_NUM,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = CS_PIN_NUM,
        .queue_size = 1,
    };

    // Inicializa barramento I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN_NUM,
        .scl_io_num = I2C_SCL_PIN_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_BUS_PORT, &conf);
    i2c_driver_install(I2C_BUS_PORT, conf.mode, 0, 0, 0);

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);    

    // Inicializa LCD
    lcd_init();

    // Inicializa MFRC522
    mfrc522_init(); 
    ESP_LOGI(TAG, "MFRC522 inicializado. Aguardando estabilizacao...");
    vTaskDelay(pdMS_TO_TICKS(300));
    bool statusMessage = false;

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    lcd_set_cursor(0, 0);
    lcd_write_string("Conectando a ");
    lcd_set_cursor(1, 0);
    lcd_write_string("Rede Wifi...");

    ESP_ERROR_CHECK(example_connect());

    mqtt5_app_start();

    while (1) {
        uint8_t uid[5];

        display_main_text(&statusMessage);

        if (mfrc522_request()) {
            if (mfrc522_anticollision(uid, &statusMessage)) {

                ESP_LOGI(TAG, "UID: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);

                char str_uid[17];
                snprintf(str_uid, sizeof(str_uid), "UID:%02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);

                char send_uid[9];
                snprintf(send_uid, sizeof(send_uid), "%02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3]);
                
                lcd_clear_lines();
                lcd_set_cursor(0,0);
                lcd_write_string(str_uid);

                handle_send(send_uid);
                vTaskDelay(pdMS_TO_TICKS(1000));
                statusMessage = false;
            } else {
                ESP_LOGW(TAG, "Falha na anticollision");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
