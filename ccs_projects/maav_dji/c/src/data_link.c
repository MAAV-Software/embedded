#include "../include/data_link.h"
#include <stdlib.h>

static inline void data_link_branchless_assemble_byte(uint8_t* pkt, size_t* pkt_i,
	uint8_t data_byte);

static inline uint8_t data_link_branchless_decode_byte(uint8_t* pkt, size_t* pkt_i);

void data_link_assemble_packet(uint8_t* msg, uint8_t* pkt, uint16_t size) {
	size_t msg_i = 0, pkt_i = 0;
	pkt[pkt_i++] = DATA_FRAME_START_DELIMITER;

	data_link_branchless_assemble_byte(pkt, &pkt_i, size >> 8);
	pkt_i++;
	data_link_branchless_assemble_byte(pkt, &pkt_i, size & 0xFF);
	pkt_i++;

	while (msg_i < size) {
		data_link_branchless_assemble_byte(pkt, &pkt_i, msg[msg_i++]);
		pkt_i++;
	}

	// TODO: calculate checksum
	uint8_t checksum = 0;
	data_link_branchless_assemble_byte(pkt, &pkt_i, checksum);
}

bool data_link_decode_packet(uint8_t* msg, uint8_t* pkt, uint8_t* size) {
	size_t msg_i = 0, pkt_i = 0;
	if (pkt[pkt_i++] != DATA_FRAME_START_DELIMITER) {
		return false;
	}

	uint8_t size_upper = data_link_branchless_decode_byte(pkt, &pkt_i);
	pkt_i++;
	uint8_t size_lower = data_link_branchless_decode_byte(pkt, &pkt_i);
	pkt_i++;
	*size = (uint16_t) (size_upper << 8) | size_lower;

	while (msg_i < *size) {
		msg[msg_i++] = data_link_branchless_decode_byte(pkt, &pkt_i);
		pkt_i++;
	}

	// TODO: check checksum
	uint8_t checksum = 0;
	uint8_t pkt_checksum = data_link_branchless_decode_byte(pkt, &pkt_i);
	if (checksum != pkt_checksum) {
		return false;
	}

	return true;
}

static inline void data_link_branchless_assemble_byte(uint8_t* pkt, size_t* pkt_i,
	uint8_t data_byte) {
	uint8_t is_special_char = (data_byte == DATA_FRAME_START_DELIMITER) +
	(data_byte == DATA_FRAME_ESCAPE_CHAR);
	uint8_t xor_val = is_special_char << 5;
	pkt[*pkt_i] = DATA_FRAME_ESCAPE_CHAR;
	pkt[*pkt_i + is_special_char] = data_byte ^ xor_val;
	*pkt_i += is_special_char;
}

static inline uint8_t data_link_branchless_decode_byte(uint8_t* pkt, size_t* pkt_i) {
	uint8_t is_delimited = (pkt[*pkt_i] == DATA_FRAME_ESCAPE_CHAR);
	uint8_t xor_val = is_delimited << 5;
	*pkt_i += is_delimited;
	return (pkt[*pkt_i] ^ xor_val);
}
