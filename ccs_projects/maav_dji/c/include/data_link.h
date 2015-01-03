#ifndef DATA_LINK_LAYER_H
#define DATA_LINK_LAYER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_FRAME_START_DELIMITER 0x7E
#define DATA_FRAME_ESCAPE_CHAR 0x7D
#define DATA_FRAME_XOR 0x20

/*
A Data link layer packet will look at like
<start delimiter> - 1 byte
<length of message> - 2 to 4 bytes (depending if they have to be escaped) (big endian)
<msg> - msg with escaped characters
<checksum> - 1 to 2 bytes (depending if it needs to be escaped)

checksum is the sum of all the data bytes (unescaped), take only the least significant byte, and subtract from 0xFF.
NOTE: probably should include the length bytes in the checksum
*/

// states for data_frame_push_byte
typedef enum data_frame_state { READY, LEN_1, LEN_2, LEN_1_ESCP, LEN_2_ESCP, READ, READ_ESCP, CHECKSUM, CHECKSUM_ESCP, DONE, START_ERR, CHECKSUM_ERR, DATA_ERR } data_frame_state_t;

extern char* data_frame_state_names[];

// structure to hold a data frame while it's being decoded
typedef struct data_frame {
	uint8_t* buffer;
	uint16_t size; // keeps track of length of message
	uint16_t index;
	uint8_t checksum;
	data_frame_state_t state; // state determing next byte read in
} data_frame_t;

/**
 * @brief creates a data frame that can hold a message of max size
 * @param size max size this data frame can hold
 * @return ptr to created data frame. You are responsible for calling data_frame_destroy() on this
 */
data_frame_t* data_frame_create(uint16_t size);

/**
 * @brief frees all data for a data frame
 */
void data_frame_destroy(data_frame_t* frame);

/**
 * @brief pushes a byte into the data frame.
 * @details used for decoding a bytes stream one at a time. frame will keep a state
 * of where it is in the decode process.
 * the frame is done if the state is DONE or if there is an error
 * START_ERR is the start delimiter is missing
 * CHECKSUM_ERR is the checksum is wrong
 * DATA_ERR is when the data contains an illegal character (TODO)
 * @param frameframe to push byte into
 * @param byte byte to push into frame
 * @return true if byte was succesfully processed
 */
bool data_frame_push_byte(data_frame_t* frame, uint8_t byte);

/**
 * @brief assembles a packet in the data link layer
 * @details will add the message delimiters, escape the
 * necessary characters, and calculate the checksum
 * @param msg message to be encapsulated
 * @param pkt array to put assembled packet in (size should be 2 * sizeof(msg) + 3) to be on the safe side
 * @param size size of msg
 * @return size of pkt
 */
uint16_t data_link_assemble_packet(uint8_t* msg, uint8_t* pkt, uint16_t size);

/**
 * @brief decodes a recieved packet
 * @details will take the pkt, strip away the delimiters
 * unescape all escaped characters, and check that the checksum is right
 * @param msg array to put decoded msg in (must be big enough)
 * @param pkt data packet to be decoded
 * @param size place to put the size of the msg
 * @return true if successfully decoded
 */
bool data_link_decode_packet(uint8_t* msg, uint8_t* pkt, uint16_t* size);

#ifdef __cplusplus
}
#endif

#endif /* DATA_LINK_LAYER_H */
