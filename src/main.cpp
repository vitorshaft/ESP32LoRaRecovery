#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Configurações de Hardware
#define GPS_UART_NUM UART_NUM_2  // UART2 (GPIO16=RX, GPIO17=TX)
#define LORA_FREQUENCY 915E6
#define LORA_SS_PIN 5
#define LORA_RST_PIN 14
#define LORA_DIO0_PIN 2

// Endereços LoRa
const uint8_t DESTINATION_ADDRESS = 0x0A;
const uint8_t LOCAL_ADDRESS = 0x0F;

// Estrutura para dados GPS
typedef struct {
    double latitude;
    double longitude;
    uint32_t timestamp;
    uint8_t satellites;
    float hdop;
} GpsData_t;

// Fila e semáforos
QueueHandle_t gpsDataQueue;
SemaphoreHandle_t loraMutex;

// Objetos globais
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // UART2

// Protótipos de tarefas
void TaskGPSReader(void *pvParameters);
void TaskLoRaSender(void *pvParameters);
void TaskSystemMonitor(void *pvParameters);

// Protótipos de funções auxiliares
uint16_t calculateCRC16(const uint8_t *data, size_t length);
void sendLoRaPacket(const GpsData_t *data);
void loraInit();

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

    // Inicializa LoRa com pinos específicos para ESP32
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed!");
        while (1);
    }
    
    LoRa.setSyncWord(0xF3);
    LoRa.enableCrc();

    // Cria recursos do FreeRTOS
    gpsDataQueue = xQueueCreate(5, sizeof(GpsData_t));
    loraMutex = xSemaphoreCreateMutex();
    
    if (gpsDataQueue == NULL || loraMutex == NULL) {
        Serial.println("RTOS resource creation failed!");
        while (1);
    }

    // Cria tarefas no núcleo apropriado
    xTaskCreatePinnedToCore(
        TaskGPSReader,     // Função da tarefa
        "GPS Reader",      // Nome da tarefa
        4096,             // Tamanho da stack (maior no ESP32)
        NULL,             // Parâmetros
        3,                // Prioridade
        NULL,             // Handle da tarefa
        0                 // Núcleo (0 ou 1)
    );

    xTaskCreatePinnedToCore(
        TaskLoRaSender,
        "LoRa Sender",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        TaskSystemMonitor,
        "System Monitor",
        2048,
        NULL,
        1,
        NULL,
        1
    );

    // Não chamamos vTaskStartScheduler() - o ESP32 já inicia automaticamente
}

void loop() {
    // Não usado quando FreeRTOS está ativo
    vTaskDelete(NULL); // Deleta a tarefa loop padrão
}

// Tarefa: Leitura contínua do GPS
void TaskGPSReader(void *pvParameters) {
    GpsData_t newData;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid() && gps.location.isUpdated()) {
                    newData.latitude = gps.location.lat();
                    newData.longitude = gps.location.lng();
                    newData.timestamp = millis();
                    newData.satellites = gps.satellites.value();
                    newData.hdop = gps.hdop.value() / 100.0;
                    
                    if (xQueueSend(gpsDataQueue, &newData, 0) != pdTRUE) {
                        Serial.println("GPS queue full!");
                    }
                }
            }
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
    }
}

// Tarefa: Transmissão LoRa
void TaskLoRaSender(void *pvParameters) {
    GpsData_t txData;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        if (xQueueReceive(gpsDataQueue, &txData, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                sendLoRaPacket(&txData);
                xSemaphoreGive(loraMutex);
            }
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5000));
    }
}

// Tarefa: Monitoramento do sistema
void TaskSystemMonitor(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    UBaseType_t uxHighWaterMark;
    
    for (;;) {
        Serial.printf("Heap free: %d bytes\n", esp_get_free_heap_size());
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        Serial.printf("Monitor stack: %d\n", uxHighWaterMark);
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10000));
    }
}

// Função: Envia pacote LoRa com CRC
void sendLoRaPacket(const GpsData_t *data) {
    uint8_t buffer[24]; // Buffer maior para mais campos
    uint8_t *ptr = buffer;
    
    // Cabeçalho do pacote
    *ptr++ = DESTINATION_ADDRESS;
    *ptr++ = LOCAL_ADDRESS;
    
    // Timestamp (4 bytes)
    *ptr++ = (data->timestamp >> 24) & 0xFF;
    *ptr++ = (data->timestamp >> 16) & 0xFF;
    *ptr++ = (data->timestamp >> 8) & 0xFF;
    *ptr++ = data->timestamp & 0xFF;
    
    // Latitude (4 bytes)
    int32_t lat = (int32_t)(data->latitude * 1e6);
    *ptr++ = (lat >> 24) & 0xFF;
    *ptr++ = (lat >> 16) & 0xFF;
    *ptr++ = (lat >> 8) & 0xFF;
    *ptr++ = lat & 0xFF;
    
    // Longitude (4 bytes)
    int32_t lng = (int32_t)(data->longitude * 1e6);
    *ptr++ = (lng >> 24) & 0xFF;
    *ptr++ = (lng >> 16) & 0xFF;
    *ptr++ = (lng >> 8) & 0xFF;
    *ptr++ = lng & 0xFF;
    
    // Satélites (1 byte)
    *ptr++ = data->satellites;
    
    // HDOP (2 bytes)
    uint16_t hdop = (uint16_t)(data->hdop * 100);
    *ptr++ = (hdop >> 8) & 0xFF;
    *ptr++ = hdop & 0xFF;
    
    // Calcula CRC (2 bytes)
    uint16_t crc = calculateCRC16(buffer, ptr - buffer);
    *ptr++ = (crc >> 8) & 0xFF;
    *ptr++ = crc & 0xFF;
    
    // Envia pacote
    LoRa.beginPacket();
    LoRa.write(buffer, ptr - buffer);
    LoRa.endPacket();
    
    // Log para debug
    Serial.printf("Sent packet - Lat: %.6f Lng: %.6f Sats: %d HDOP: %.2f CRC: 0x%04X\n",
                 data->latitude, data->longitude, data->satellites, data->hdop, crc);
}

// Função: Calcula CRC-16 (Modbus)
uint16_t calculateCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}