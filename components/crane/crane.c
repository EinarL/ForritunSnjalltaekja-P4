#include "crane.h"

#include <string.h>

#include <esp_log.h>

#include <lownet.h>
#include <utility.h>
#include <serial_io.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define CRANE_PROTO 0x05

#define TAG "crane"

void crane_connect(uint8_t id);
void crane_disconnect();
int  crane_action(uint8_t action); // returns zero if ACK is received
void crane_test(uint8_t id);
void crane_receive(const lownet_frame_t* frame);
void crane_send(uint8_t destination, const crane_packet_t* packet);

// state of a single flow
static struct
{
	uint16_t seq;
	uint8_t crane;
	QueueHandle_t acks;
	enum
		{
			ST_DISCONNECTED,
			ST_HANDSHAKE,
			ST_CONNECTED,
		} state;
} state;

int crane_init(void)
{
	if (lownet_register_protocol(CRANE_PROTO, crane_receive) != 0)
		{
			ESP_LOGE(TAG, "Failed to register crane protocol");
			return 1;
		}

	state.seq = 0;
	state.crane = 0;
	state.state = ST_DISCONNECTED;
	state.acks = xQueueCreate(8, sizeof(uint16_t));
	return 0;
}

void crane_command(char* args)
{
	if (!args)
		{
			serial_write_line("Missing argument COMMAND");
			return;
		}
	char* saveptr;

	char* command = strtok_r(args, " ", &saveptr);
	if (!command)
		{
			serial_write_line("Missing argument COMMAND");
			return;
		}

	if (strcmp(command, "open") == 0) 
		{
			char* id = strtok_r(NULL, " ", &saveptr);
			if (!id)
				{
					serial_write_line("Missing argument ID");
					return;
				}
			uint8_t dest = hex_to_dec(id + 2); // id needs to be 0x...
			crane_connect(dest);
		}
	else if (strcmp(command, "close") == 0)
		{
			crane_disconnect();
		}
	else if (strcmp(command, "test") == 0)
		{
			char* id = strtok_r(NULL, " ", &saveptr);
			if (!id)
				{
					serial_write_line("Missing argument ID");
					return;
				}
			uint8_t dest = hex_to_dec(id + 2); // id needs to be 0x...
			crane_test(dest);
		}
	else
		{
			uint8_t action;
			switch (command[0])
				{
					// ------------------------------------------------
					// Milestone II, Task 2: implement commands to CLI
				case 'f':  // FORWARD, call crane_action with appropriate action!
					action = CRANE_FWD;
					break;
				case 'b': // BACKWARD
					action = CRANE_REV;
					break;
				case 's': // STOP
					action = CRANE_STOP;
					break;
				case 'u': // UP
					action = CRANE_UP;
					break;
				case 'd': // DOWN
					action = CRANE_DOWN;
					break;
				case '1': // LIGHT_ON
					action = CRANE_LIGHT_ON;
					break;
				case '0': // LIGHT_OFF
					action = CRANE_LIGHT_OFF;
					break;
				// repeat the above for all commands!
				// ------------------------------------------------
				default:
					ESP_LOGI(TAG, "Invalid crane command");
					return;
				}
			int ret = crane_action(action);
			printf("returned %d\n", ret);
		}
}

void crane_recv_connect(const crane_packet_t* packet)
{
	ESP_LOGI(TAG, "Received CONNECT packet");
	if (state.state != ST_HANDSHAKE)
		return;

	ESP_LOGI(TAG, "packet flags: %02x", packet->flags);

	// ------------------------------------------------
	// Milestone I: check that both SYN and ACK flags are set?
	// - if not, return
	if ((packet->flags & 0b00000011) != 0b00000011){
		return;
	}

	uint32_t challenge_bits = packet->d.conn.challenge;

	conn_t con;
	{
		con.challenge = challenge_bits ^ 0xFFFFFFFF;
	}

	// Milestone I: respond with appropriate (ack) packet + challenge! (cf. Task 2)
	crane_packet_t outpkt;
	{
		outpkt.type = packet->type;
		outpkt.flags = CRANE_ACK;
		outpkt.seq = 0;

		outpkt.d.conn = con;
	}
	crane_send(state.crane, &outpkt);
	// ------------------------------------------------
}

void crane_recv_close(const crane_packet_t* packet)
{
	ESP_LOGI(TAG, "Closing connection");
	state.seq = 0;
	state.state = ST_DISCONNECTED;
	state.crane = 0;
}

void crane_recv_status(const crane_packet_t* packet)
{
	char buffer[200];

	if ( packet->flags & CRANE_NAK )	// crane missed some packet
		{
			// Not in use yet -- anywhere, so you can ignore
			ESP_LOGI(TAG, "Received status packet with NAK -- not in use yet" );
			return;
		}
	else // push the cumulative ack to ACK-queue
	{
		xQueueSend( state.acks, &packet->seq, 0 ); //xQueueSend( ack_queue, packet->seq, 0 );
	}
		

	snprintf(buffer, sizeof buffer,
					 "backlog: %d\n"
					 "time: %d\n"
					 "light: %s\n",
					 packet->d.status.backlog,
					 packet->d.status.time_left,
					 packet->d.status.light ? "on" : "off");
	serial_write_line(buffer);
}

void crane_receive(const lownet_frame_t* frame)
{
	crane_packet_t packet;
	memcpy(&packet, frame->payload, sizeof packet);
	ESP_LOGI(TAG, "Received packet frame from %02x, type: %d", frame->source, packet.type);
	switch (packet.type)
		{
		case CRANE_CONNECT:
			crane_recv_connect(&packet);
			break;
		case CRANE_STATUS:
			crane_recv_status(&packet);
			break;
		case CRANE_ACTION:
			break;
		case CRANE_CLOSE:
			crane_recv_close(&packet);
		}
}

/*
 * This function starts the connection establishment
 * procedure by sending a SYN packet to the given node.
 */
void crane_connect(uint8_t id)
{
	if (state.state != ST_DISCONNECTED)
		return;

	crane_packet_t packet;
	packet.type = CRANE_CONNECT;
	packet.flags = CRANE_SYN;
	packet.seq = state.seq;

	state.crane = id;
	state.state = ST_HANDSHAKE;
	++state.seq;

	crane_send(id, &packet);
}

void crane_disconnect(void)
{
	crane_packet_t packet;

	// ------------------------------------------------
	// Milestone I, Task 3: construct a CLOSE packet and
	// send it to the (right) crane!
	{
		packet.type = CRANE_CLOSE;
		packet.flags = CRANE_ACK;
		packet.seq = state.seq;

		packet.d.close = 0;
	}
	// ------------------------------------------------
	// Then we update our state
	state.state = ST_DISCONNECTED;
	state.seq = 0;

	crane_send(state.crane, &packet);
}


/*
 *	Subroutine for crane_action: read ACKs from crane, blocks for some time
 */
uint16_t read_acks(void)
{
	uint16_t seq, x;

	// Wait for an ack up to 5 seconds
	if ( xQueueReceive(state.acks, &seq, 5000/portTICK_PERIOD_MS) != pdTRUE )
		seq = 0; // set seq to 0 if no acks received (otherwise if acks were recieved then the xQueueReceive function sets seq to be the sequence number received)
	// read any other acks if in the queue
	while ( xQueueReceive(state.acks, &x, 0) == pdTRUE )
		seq = seq >= x ? seq : x;
	return seq;
}

/*
 *	This can block for a while if no immediate ACK
 */
int crane_action(uint8_t action)
{
	crane_packet_t packet;

	packet.type = CRANE_ACTION;
	packet.seq = state.seq;
	packet.d.action.cmd = action;

	crane_send(state.crane, &packet);

	// ------------------------------------------------
	// Milestone II, Task 1: run the following up to five times
	// Your code goes here
	{	
		uint16_t seq;
		uint8_t count = 0;
		while(count < 5){
			seq = read_acks();	 // seq is the cumulative ack from crane, or zero if none
			count++;

			if (seq > state.seq){ // If seq > state.seq, report an error and close the connection, return -2
				ESP_LOGE(TAG, "Error: seq is larger than state.seq, seq: %d, state.seq: %d", seq, state.seq);
				return -2;
			}
			else if (seq == state.seq){ // - Else if seq == state.seq, we are good, increment state.seq by one (why?!) and return 0
				state.seq++;
				return 0;
			}
			// - Otherwise retransmit
		}
	}
	// ------------------------------------------------

	// No ack received, disconnect
	ESP_LOGI(TAG, "Received no ack from node=0x%02x", state.crane );
	crane_disconnect();
	return -1;
}


// ------------------------------------------------
//
// Milestone III: run the test pattern
//
// 1. establish connection with TEST flag
// 2. run the test pattern according to the specs
// 3. close the connection
//
// Hint: you can work it all out here slowly, or be a wizard
//       and launch a separate task for this!
//
void crane_test(uint8_t id)
{
}
// ------------------------------------------------


void crane_send(uint8_t id, const crane_packet_t* packet)
{
	lownet_frame_t frame;
	frame.destination = id;
	frame.protocol = CRANE_PROTO;
	frame.length = sizeof *packet;
	memcpy(frame.payload, packet, sizeof *packet);

	lownet_send(&frame);
}
