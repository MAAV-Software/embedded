#ifndef DATA_LINK_LAYER_H
#define DATA_LINK_LAYER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_FRAME_START_DELIMITER 0x7E
#define DATA_FRAME_ESCAPE_CHAR 0x7D

/*
A Data link layer packet will look at like
<start delimiter> - 1 byte
<length of message> - 2 to 4 bytes (depending if they have to be escaped)
<msg> - msg with escaped characters
<checksum> - 1 to 2 bytes (depending if it needs to be escaped)
*/


/**
 * @brief assembles a packet in the data link layer
 * @details will add the message delimiters, escape the
 * necessary characters, and calculate the checksum
 * @param msg message to be encapsulated
 * @param pkt array to put assembled packet in (size should be 2 * sizeof(msg) + 3) to be on the safe side
 * @param size size of msg
 */
void data_link_assemble_packet(uint8_t* msg, uint8_t* pkt, uint16_t size);

/**
 * @brief decodes a recieved packet
 * @details will take the pkt, strip away the delimiters
 * unescape all escaped characters, and check that the checksum is right
 * @param msg array to put decoded msg in (must be big enough)
 * @param pkt data packet to be decoded
 * @param size place to put the size of the msg
 * @return true if successfully decoded
 */
bool data_link_decode_packet(uint8_t* msg, uint8_t* pkt, uint8_t* size);

#ifdef __cplusplus
}
#endif

#endif /* DATA_LINK_LAYER_H */
