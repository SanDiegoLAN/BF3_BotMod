// In-game text chat for BotMod through the server's own RCON port: reads player chat (for !commands) and posts
// admin messages. Runs on a background thread; the game thread only polls / queues.
#pragma once

typedef void (*ChatLogFn)(const char *fmt, ...);

// Reads the RCON address and password from the server's command line (-RemoteAdminPort, +admin.password).
void chat_start(ChatLogFn log);
void chat_stop(void);
// Next chat line that starts with '!' (game thread). Returns 0 when none is waiting.
int chat_poll(char *who, int whoLen, char *text, int textLen);
// Queues a message for everyone (split to the 128-character chat limit).
void chat_say(const char *fmt, ...);
void chat_say_team(int team, const char *fmt, ...);
// Queues any RCON command; words separated by tabs, e.g. "admin.movePlayer\t<name>\t1\t2\tfalse".
void chat_rcon(const char *fmt, ...);
// The server's current player limit (vars.maxPlayers via RCON serverInfo, refreshed every 30 s); 0 = not known yet.
int chat_max_players(void);
// The server's game mode (serverInfo, e.g. ConquestLarge0, TankSuperiority0), read again after every level load; "" = not yet.
void chat_game_mode(char *out, int n);
// How many server.onLevelLoaded / server.onRoundOver events RCON has delivered so far (level reloads of the same map
// are invisible otherwise).
void chat_level_events(long *loads, long *roundsOver);
// The winning team of the last server.onRoundOver (-1: none yet). Co-op: what a mission pass / fail reports.
long chat_last_round_winner(void);
// Switches the server to map `code` (e.g. MP_017) right away, via the rotation; announces it in chat.
void chat_map(const char *code, const char *who);
// Says this server's map rotation in chat (the !map list).
void chat_map_list(void);
