// CSTDLIB includes.
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <esp_log.h>
#include <esp_random.h>

// LowNet includes.
#include "lownet.h"

#include "serial_io.h"
#include "utility.h"
#include "cli.h"
#include "crypt.h"

#include "chat.h"
#include "ping.h"
#include "cli.h"
#include "command.h"

#include "crane.h"

#define TAG "main"

// Usage: help_command(NULL)
// Pre:   None, this command takes no arguments.
// Post:  A list of available commands has been written to the serial port.
void help_command(char*);

const command_t commands[] = {
	{"shout",   "/shout MSG                   Broadcast a message.", shout_command},
	{"tell",    "/tell ID MSG or @ID MSG      Send a message to a specific node", tell_command},
	{"ping",    "/ping ID                     Check if a node is online", ping_command},
	{"date",    "/date                        Print the current time", date_command},
	{"setkey",  "/setkey [KEY|0|1]            Set the encryption key to use.  If no key is provided encryption is disabled", crypt_setkey_command},
	{"id",      "/id                          Print your ID", id_command},
	{"testenc", "/testenc [STR]               Run STR through a encrypt/decrypt cycle to verify that encryption works", crypt_test_command},
	{"crane",   "/crane COMMAND               /crane help for details", crane_command},
	{"help",    "/help                        Print this help", help_command}
};

const size_t NUM_COMMANDS = sizeof commands / sizeof(command_t);
#define FIND_COMMAND(_command) (find_command(_command, commands, NUM_COMMANDS))

// Usage: help_command(NULL)
// Pre:   None, this command takes no arguments.
// Post:  A list of available commands has been written to the serial port.
void help_command(char*)
{
	/*
		Loop Invariant:
		0 <= i < NUM_COMMANDS
		forall x | 0 <= x < i : commands[x] has been written to the serial port
	 */
	for (size_t i = 0; i < NUM_COMMANDS; ++i)
		serial_write_line(commands[i].description);
	serial_write_line("Any input not preceded by a '/' or '@' will be treated as a broadcast message.");
}

void app_main(void)
{
	char msg_in[MSG_BUFFER_LENGTH];
	char msg_out[MSG_BUFFER_LENGTH];

	// Initialize the serial services.
	init_serial_service();

	// Initialize the LowNet services.
	lownet_init(crypt_encrypt, crypt_decrypt);
	chat_init();
	ping_init();
	

	// Initialize the command module
	command_init();
	crane_init();

		// Dummy implementation -- this isn't true network time!  Following 2
	//	lines are not needed when an actual source of network time is present.
	lownet_time_t init_time = {1, 0};
	lownet_set_time(&init_time);

	serial_write_line("> ");

	while (true)
		{
			memset(msg_in, 0, MSG_BUFFER_LENGTH);
			memset(msg_out, 0, MSG_BUFFER_LENGTH);
			
			if (!serial_read_line(msg_in)) {
				
				// Quick & dirty input parse.
				if (msg_in[0] == 0) continue;
				if (msg_in[0] == '/')
					{
						char* name = strtok(msg_in + 1, " ");
						if (!name)
							continue;
						command_fun_t command = FIND_COMMAND(name);
						if (!command)
							{
								char buffer[17 + strlen(name) + 1];
								sprintf(buffer, "Invalid command: %s", name);
								serial_write_line(buffer);
								continue;
							}
						char* args = strtok(NULL, "\n");
						command(args);
					}
				else if (msg_in[0] == '@')
					{
						FIND_COMMAND("tell")(msg_in + 1);
					}
				else
					{
						// Default, chat broadcast message.
						FIND_COMMAND("shout")(msg_in);
					}
				serial_write_line("> ");
			}
		}
}
// lorem ipsum
