#include "vesc_uart.h"
#include <string.h>

static UART_HandleTypeDef *vesc_huart;
static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];
static uint16_t rx_old_pos = 0;

void VESC_UART_Init(UART_HandleTypeDef *huart) {
    vesc_huart = huart;
    HAL_UART_Receive_DMA(vesc_huart, rx_buffer, sizeof(rx_buffer));
}

static uint16_t crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint16_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void send_packet(uint8_t *payload, uint16_t len) {
    uint16_t crc = crc16(payload, len);
    uint16_t pos = 0;

    if (len <= 255) {
        tx_buffer[pos++] = VESC_PACKET_START_SMALL;
        tx_buffer[pos++] = (uint8_t)len;
    } else {
        tx_buffer[pos++] = VESC_PACKET_START_LARGE;
        tx_buffer[pos++] = (uint8_t)(len >> 8);
        tx_buffer[pos++] = (uint8_t)(len & 0xFF);
    }

    memcpy(&tx_buffer[pos], payload, len);
    pos += len;

    tx_buffer[pos++] = (uint8_t)(crc >> 8);
    tx_buffer[pos++] = (uint8_t)(crc & 0xFF);
    tx_buffer[pos++] = VESC_PACKET_END;

    HAL_UART_Transmit(vesc_huart, tx_buffer, pos, 100);
}

static void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number & 0xFF;
}

static void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number & 0xFF;
}

static void handle_packet(uint8_t *payload, uint16_t len, MotorStatePublic_t *motorState) {
    if (len == 0) return;

    VESC_COMM_ID comm_id = (VESC_COMM_ID)payload[0];
    uint8_t response[128];
    int32_t r_pos = 0;

    switch (comm_id) {
        case COMM_FW_VERSION:
            response[r_pos++] = COMM_FW_VERSION;
            response[r_pos++] = 5; // FW Major
            response[r_pos++] = 02; // FW Minor
            response[r_pos++] = 0; // FW Patch
            memcpy(&response[r_pos], "EBiCS", 5);
            r_pos += 5;
            // UUID (12 bytes)
            memset(&response[r_pos], 0, 12);
            r_pos += 12;
            response[r_pos++] = 0; // Paired
            send_packet(response, r_pos);
            break;

        case COMM_GET_VALUES:
            response[r_pos++] = COMM_GET_VALUES;
            buffer_append_int16(response, 250, &r_pos); // Temp FET (10x)
            buffer_append_int16(response, 250, &r_pos); // Temp Motor (10x)
            buffer_append_int32(response, (int32_t)(motorState->debug[1] / 10), &r_pos); // Avg Motor Current (100x)
            buffer_append_int32(response, (int32_t)(motorState->debug[0] / 10), &r_pos); // Avg Input Current (100x)
            buffer_append_int32(response, 0, &r_pos); // Avg Id (100x)
            buffer_append_int32(response, 0, &r_pos); // Avg Iq (100x)
            buffer_append_int16(response, 0, &r_pos); // Duty Cycle Now (1000x)
            buffer_append_int32(response, (int32_t)motorState->speed * 10, &r_pos); // RPM (1x)
            buffer_append_int16(response, (int16_t)(motorState->battery_voltage / 100), &r_pos); // Input Voltage (10x)
            buffer_append_int32(response, 0, &r_pos); // Amp Hours (10000x)
            buffer_append_int32(response, 0, &r_pos); // Amp Hours Charged (10000x)
            buffer_append_int32(response, 0, &r_pos); // Watt Hours (10000x)
            buffer_append_int32(response, 0, &r_pos); // Watt Hours Charged (10000x)
            buffer_append_int32(response, 0, &r_pos); // Tachometer (1x)
            buffer_append_int32(response, 0, &r_pos); // Tachometer Abs (1x)
            response[r_pos++] = (uint8_t)motorState->error_state; // Fault Code
            buffer_append_int32(response, 0, &r_pos); // PID Pos (1000000x)
            response[r_pos++] = 1; // Controller ID

            send_packet(response, r_pos);
            break;

        case COMM_SET_CURRENT:
            if (len >= 5) {
                // VESC usually sends current as float32
                float current_f;
                uint32_t current_u = (payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4];
                memcpy(&current_f, &current_u, 4);
                motorState->i_q_setpoint_target = (int16_t)(current_f * 1000.0f); // Convert to mA
            }
            break;

        case COMM_ALIVE:
            // Send back COMM_ALIVE or just reset watchdog
            break;

        default:
            break;
    }
}

void VESC_UART_Process(MotorStatePublic_t *motorState) {
    if (!vesc_huart || !vesc_huart->hdmarx) return;
    uint16_t curr_pos = sizeof(rx_buffer) - vesc_huart->hdmarx->Instance->CNDTR;

    while (rx_old_pos != curr_pos) {
        if (rx_buffer[rx_old_pos] == VESC_PACKET_START_SMALL || rx_buffer[rx_old_pos] == VESC_PACKET_START_LARGE) {
            uint16_t start_pos = rx_old_pos;
            uint16_t packet_len = 0;
            uint16_t header_len = 0;

            if (rx_buffer[start_pos] == VESC_PACKET_START_SMALL) {
                header_len = 2;
                packet_len = rx_buffer[(start_pos + 1) % sizeof(rx_buffer)];
            } else {
                header_len = 3;
                packet_len = (rx_buffer[(start_pos + 1) % sizeof(rx_buffer)] << 8) | rx_buffer[(start_pos + 2) % sizeof(rx_buffer)];
            }

            // Check if we have enough data for header + CRC + end
            uint16_t total_len = header_len + packet_len + 3;
            if (total_len > sizeof(rx_buffer)) {
                // Invalid length
                rx_old_pos = (rx_old_pos + 1) % sizeof(rx_buffer);
                continue;
            }

            uint16_t available = (curr_pos >= start_pos) ? (curr_pos - start_pos) : (sizeof(rx_buffer) - start_pos + curr_pos);

            if (available >= total_len) {
                uint8_t packet[256];
                for (uint16_t i = 0; i < total_len; i++) {
                    packet[i] = rx_buffer[(start_pos + i) % sizeof(rx_buffer)];
                }

                if (packet[total_len - 1] == VESC_PACKET_END) {
                    uint16_t crc_received = (packet[total_len - 3] << 8) | packet[total_len - 2];
                    uint16_t crc_calc = crc16(&packet[header_len], packet_len);

                    if (crc_calc == crc_received) {
                        handle_packet(&packet[header_len], packet_len, motorState);
                        rx_old_pos = (start_pos + total_len) % sizeof(rx_buffer);
                        continue;
                    }
                }
            } else {
                // Not enough data yet
                break;
            }
        }
        rx_old_pos = (rx_old_pos + 1) % sizeof(rx_buffer);
    }
}
