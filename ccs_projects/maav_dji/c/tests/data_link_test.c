#include "include/data_link.h"
#include "include/test_definitions.h"
#include <stdio.h>
#include <stdint.h>

// intended to be run on a computer

#define buffer_size 4

void printBuffer(uint8_t* buff, size_t size) {
	int i;
	for (i = 0; i < size; ++i) {
		printf("0x%x\t", buff[i]);
	}
	printf("\n\r");
}

int data_link_test() {
	uint8_t msg_buff[buffer_size] = { 0x20, 0x7D, 0x10, 0x01 };
	uint8_t pkt_buff[buffer_size * 2 + 3];

	uint16_t pkt_size = data_link_assemble_packet(msg_buff, pkt_buff, buffer_size);

	printBuffer(msg_buff, buffer_size);
	printBuffer(pkt_buff, pkt_size);

	uint16_t decoded_size;
	if (data_link_decode_packet(msg_buff, pkt_buff, &decoded_size)) {
		printBuffer(msg_buff, decoded_size);
	} else {
		printf("could not decode\n\r");
	}

	printf("pkt_size: %d\n\r", pkt_size);

	data_frame_t* frame = data_frame_create(buffer_size);
	int i;
	for (i = 0; i < pkt_size; ++i) {
		printf("%s\n\r", data_frame_state_names[frame->state]);
		printf("0x%x\n\r", pkt_buff[i]);
		data_frame_push_byte(frame, pkt_buff[i]);
	}
	printf("%s\n\r", data_frame_state_names[frame->state]);
	printf("frame_index: %d\n\r", frame->index);
	printBuffer(frame->buffer, frame->size);

	data_frame_destroy(frame);
}