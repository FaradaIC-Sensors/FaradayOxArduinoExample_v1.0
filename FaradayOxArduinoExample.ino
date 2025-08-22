// FaradaIC
// FaradayOx sensor Example
// Register Map Version >= 1.0
// Firmware Version >= 1.0


#include "faradaic_registers.h"
#include <stdbool.h>
#include <stdint.h>

#define UART_BUFFER_SIZE 64

typedef struct {
  uint8_t status;
  float concentration;
  float temperature;
  float humidity;
} MeasurementResult_t;

static uint32_t build_frame(uint8_t *buffer, 
  uint32_t buffer_size, 
  uint8_t operation, 
  uint16_t data_len, 
  uint16_t address);
  
static bool read_exact(uint8_t *buffer, 
  uint32_t buffer_size, 
  uint32_t expected_len, 
  uint32_t timeout_ms);

static bool check_ack_ready(uint8_t *buffer, 
  uint32_t buffer_size, 
  bool accept_ready);

static bool wait_ack(uint8_t *buffer,
  uint32_t buffer_size);

static bool read_address_data_and_suffix(uint8_t *buffer, uint32_t buffer_size);

bool faradayox_ping(void);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  if (faradayox_ping()) {
    Serial.println("PING OK");
  } else {
    Serial.println("PING FAIL");
  }

  MeasurementResult_t measurement = {0};
  if (faradayox_measure(&measurement)) {
    Serial.println("MEASUREMENT OK");
    Serial.print("Conc:");
    Serial.print(measurement.concentration);
    Serial.print("; Temp: ");
    Serial.print(measurement.temperature);
    Serial.print("; Humidity: ");
    Serial.print(measurement.humidity);
    Serial.print("\r\n");
  } else {
    Serial.println("MEASUREMENT FAIL");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

bool faradayox_measure(MeasurementResult_t *measurement) {
  if (!measurement) {
    return false;
  }

  uint8_t buffer[UART_BUFFER_SIZE];
  if (!faradayox_ping()) {
    return false;
  }
  buffer[FRAME_DATA_POS] = REG_CONTROL_START_MEASUREMENT;
  // Start the measurement
  uint32_t len = build_frame(buffer, UART_BUFFER_SIZE, OPERATION_WRITE, 1, REG_CONTROL);
  if (len > 0) {
    Serial1.write(buffer, len);
    Serial1.flush();
  }
  // Read the ACK
  if (!check_ack_ready(buffer, UART_BUFFER_SIZE, false)) {
    return false;
  }

  // Wait for the module to perform the measurement
  delay(200);
  if (!faradayox_ping()) {
    return false;
  }
  // Read Status, Concentration, Temperature, Humidity
  len = build_frame(buffer, UART_BUFFER_SIZE, OPERATION_READ, 14, REG_STATUS);
  if (len > 0) {
    Serial1.write(buffer, len);
    Serial1.flush();
  }

  if (!wait_ack(buffer, UART_BUFFER_SIZE)) {
    return false;
  }
  if (!read_address_data_and_suffix(buffer, UART_BUFFER_SIZE)) {
    return false;
  }
  // At this point buffer contatins valid measurements data
  measurement->status = buffer[FRAME_DATA_POS + REG_STATUS - REG_STATUS];
  // If bad status
  if (!(buffer[FRAME_DATA_POS] == REG_STATUS_MEASUREMENT_FINISHED)) {
    return false;
  }

  measurement->concentration = *(float *)(buffer + FRAME_DATA_POS + REG_CONCENTRATION_LLSB - REG_STATUS);
  measurement->temperature = *(float *)(buffer + FRAME_DATA_POS + REG_TEMPERATURE_LLSB - REG_STATUS);
  measurement->humidity = *(float *)(buffer + FRAME_DATA_POS + REG_HUMIDITY_LLSB - REG_STATUS);
  return true;
}

// Data should be already placed in the buffer before calling this function
// Return 0 if something is wrong. Otherwise returns frame size
static uint32_t build_frame(uint8_t *buffer, 
  uint32_t buffer_size, 
  uint8_t operation, 
  uint16_t data_len, 
  uint16_t address) {
  
  uint32_t frame_size = FRAME_PROTOCOL_OVERHEAD;

  // If operation WRITE, increase frame size to fit passed data
  if (operation == OPERATION_WRITE) {
    frame_size += data_len;
  }

  // Can data fit into the buffer and if operation is allowed
  if ((buffer_size < frame_size) ||
    (!(operation == OPERATION_READ || operation == OPERATION_WRITE))) {
    return 0;
  }

  buffer[FRAME_STX_POS] = STX;
  buffer[FRAME_OP_POS] = operation;
  buffer[FRAME_ADDR_LSB_POS] = address & 0xFF;
  buffer[FRAME_ADDR_MSB_POS] = (address >> 8) & 0xFF;
  buffer[FRAME_LEN_LSB_POS] = data_len & 0xFF;
  buffer[FRAME_LEN_MSB_POS] = (data_len >> 8) & 0xFF;
  // CRC is skipped for now, just 0xFFFF

  uint16_t crc16 = 0xFFFF;
  buffer[frame_size - FRAME_CRC_LSB_END_POS] = crc16 & 0xFF;
  buffer[frame_size - FRAME_CRC_MSB_END_POS] = (crc16 >> 8) & 0xFF;
  buffer[frame_size - FRAME_ETX_END_POS] = ETX;
  return frame_size;
}

static bool read_exact(uint8_t *buffer, 
  uint32_t buffer_size, uint32_t expected_length, uint32_t timeout_ms) {
  uint32_t start = millis();
  uint32_t got = 0;
  bool receiving = true;
  while (receiving)
  {
    // Timeout
    if (millis() - start > timeout_ms)
    {
      receiving = false;
    }

    if (Serial1.available())
    {
      // Buffer overflow
      if (got >= buffer_size) {
        receiving = false;
        break;
      }

      int incoming_byte = Serial1.read();
      if (incoming_byte >= 0) {
        buffer[got++] = incoming_byte;
      }
      // Finished
      if (got == expected_length) {
        receiving = false;
        break;
      }
    }
  }
  return got == expected_length;
}

static bool faradayox_ping(void) {
  uint8_t buffer[UART_BUFFER_SIZE];
  // For ping read 0 bytes from 0 address
  uint32_t len = build_frame(buffer, UART_BUFFER_SIZE, OPERATION_READ, 0, 0);
  if (len > 0) {
    Serial1.write(buffer, len);
    Serial1.flush();
  }
  // Wait for the module to wake up
  return check_ack_ready(buffer, UART_BUFFER_SIZE, true);
}

static bool check_ack_ready(uint8_t *buffer, 
  uint32_t buffer_size, 
  bool accept_ready) {
  if (!buffer) {
    return false;
  }

  if (!read_exact(buffer, buffer_size, FRAME_RESPONSE_HEADER_SIZE, 25) || 
      buffer[FRAME_STX_POS] != STX) {
    // Not a valid frame
    return false;
  }

  if (!((accept_ready && buffer[FRAME_RESPONSE_STATUS_POS] == PROTOCOL_READY_BYTE) || 
    buffer[FRAME_RESPONSE_STATUS_POS] == PROTOCOL_ACK_BYTE)) {
    // Nack
    return false;
  }

    // Wait for the remaining bytes
  if (!read_exact(buffer + FRAME_RESPONSE_HEADER_SIZE, 
    buffer_size - FRAME_RESPONSE_HEADER_SIZE, 
    PROTOCOL_SHORT_RESPONSE_SIZE - FRAME_RESPONSE_HEADER_SIZE, 25)) {
    // Not a valid frame
    return false;
  }
  // Check CRC
  uint16_t crc16 = 0xFFFF;
  if (!((buffer[PROTOCOL_SHORT_RESPONSE_SIZE - FRAME_CRC_LSB_END_POS] == (crc16 >> 8) & 0xFF) && 
      (buffer[PROTOCOL_SHORT_RESPONSE_SIZE - FRAME_CRC_MSB_END_POS] == (crc16 >> 8) & 0xFF))) {
    return false;
  }
  // Check ETX
  if (buffer[PROTOCOL_SHORT_RESPONSE_SIZE - FRAME_ETX_END_POS] != ETX) {
    return false;
  }
  return true;
}

static bool wait_ack(uint8_t *buffer,
  uint32_t buffer_size) {
     if (!buffer) {
    return false;
  }

  if (!read_exact(buffer, buffer_size, FRAME_RESPONSE_HEADER_SIZE, 50) || 
      buffer[FRAME_STX_POS] != STX) {
    // Not a valid frame
    return false;
  }
  
  if (!(buffer[FRAME_RESPONSE_STATUS_POS] == PROTOCOL_ACK_BYTE)) {
    // Nack
    return false;
  }
  return true; 
}

static bool read_address_data_and_suffix(uint8_t *buffer, uint32_t buffer_size) {
  // Read Address and Length
  if (!read_exact(buffer + FRAME_RESPONSE_HEADER_SIZE, UART_BUFFER_SIZE - FRAME_RESPONSE_HEADER_SIZE, 4, 25)) {
    return false;
  }
  uint16_t data_length = buffer[FRAME_LEN_LSB_POS] | (((uint16_t)buffer[FRAME_LEN_MSB_POS]) << 8);

  // Read Data and frame ending
  // 1 symbol 100uS at 115200
  if (!read_exact(buffer + FRAME_DATA_POS, UART_BUFFER_SIZE - FRAME_DATA_POS, data_length + FRAME_PROTOCOL_SUFFIX_SIZE, 10 + (data_length >> 3))) {
    return false;
  }
  // Check CRC
  uint16_t crc16 = 0xFFFF;
  if (!((buffer[data_length + FRAME_PROTOCOL_OVERHEAD - FRAME_CRC_LSB_END_POS] == (crc16 >> 8) & 0xFF) && 
      (buffer[data_length + FRAME_PROTOCOL_OVERHEAD - FRAME_CRC_MSB_END_POS] == (crc16 >> 8) & 0xFF))) {
    return false;
  }
  // Check ETX
  if (buffer[data_length + FRAME_PROTOCOL_OVERHEAD - FRAME_ETX_END_POS] != ETX) {
    return false;
  }
  return true;
} 
