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

static bool faradayox_ping(void);

static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

static const uint16_t crc16_tab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B,
    0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738,
    0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
    0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
    0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB,
    0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
    0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D,
    0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
    0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  if (faradayox_ping()) {
    Serial.println("PING OK");
  } else {
    Serial.println("PING FAIL");
    return;
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
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

static bool faradayox_measure(MeasurementResult_t *measurement) {
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
  if (!(buffer[FRAME_DATA_POS] & REG_STATUS_MEASUREMENT_FINISHED)) {
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
  // CRC computed from OP, ADDR, LEN, DATA
  uint16_t crc16 = crc16_ccitt_false(buffer + FRAME_OP_POS, frame_size - FRAME_CRC_LSB_END_POS - FRAME_OP_POS);
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
  uint16_t crc16 = crc16_ccitt_false(buffer + FRAME_OP_POS, PROTOCOL_SHORT_RESPONSE_SIZE - FRAME_OP_POS - FRAME_CRC_LSB_END_POS);
  if (!((buffer[PROTOCOL_SHORT_RESPONSE_SIZE - FRAME_CRC_LSB_END_POS] == (crc16 & 0xFF)) && 
      (buffer[PROTOCOL_SHORT_RESPONSE_SIZE - FRAME_CRC_MSB_END_POS] == ((crc16 >> 8) & 0xFF)))) {
    Serial.println(4);
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
  uint16_t crc16 = crc16_ccitt_false(buffer + FRAME_OP_POS, data_length + FRAME_PROTOCOL_OVERHEAD - FRAME_OP_POS - FRAME_CRC_LSB_END_POS);
  if (!((buffer[data_length + FRAME_PROTOCOL_OVERHEAD - FRAME_CRC_LSB_END_POS] == (crc16 & 0xFF)) && 
      (buffer[data_length + FRAME_PROTOCOL_OVERHEAD - FRAME_CRC_MSB_END_POS] == ((crc16 >> 8) & 0xFF)))) {
    return false;
  }
  // Check ETX
  if (buffer[data_length + FRAME_PROTOCOL_OVERHEAD - FRAME_ETX_END_POS] != ETX) {
    return false;
  }
  return true;
}

static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFFU;
  while (len--)
  {
    uint8_t idx = (uint8_t) ((crc >> 8) ^ *data++);
    crc = (uint16_t) ((crc << 8) ^ crc16_tab[idx]);
  }
  return crc;
}
