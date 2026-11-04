// RCON chat bridge: a worker thread keeps a Frostbite RCON connection to this server, enables events, collects
// player.onChat lines starting with '!' and sends queued admin.say messages.
#include <winsock2.h>
#include <windows.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "chat.h"

#define QUEUE_LEN  256
#define LINE_LEN   256

typedef struct ChatLine { char who[64]; char text[LINE_LEN]; } ChatLine;

static ChatLogFn g_log;
static CRITICAL_SECTION g_lock;
static ChatLine g_in[QUEUE_LEN];
static int g_inHead, g_inTail;
static char g_out[QUEUE_LEN][LINE_LEN];
static int g_outHead, g_outTail;
static HANDLE g_thread;
static volatile LONG g_stop;
static SOCKET g_sock = INVALID_SOCKET;
static char g_host[64] = "127.0.0.1", g_password[128];
static int g_port = 47900;
static unsigned g_seq;

static void read_command_line(void)
{
    const char *cl = GetCommandLineA(), *p;
    if ((p = strstr(cl, "-RemoteAdminPort ")) != NULL) {
        char addr[96] = "";
        sscanf(p + 17, "%95s", addr);
        char *colon = strrchr(addr, ':');
        if (colon) { *colon = 0; g_port = atoi(colon + 1); if (addr[0]) snprintf(g_host, sizeof g_host, "%s", addr); }
        else g_port = atoi(addr);
    }
    if ((p = strstr(cl, "+admin.password ")) != NULL) {
        p += 16;
        if (*p == '"') { p++; const char *e = strchr(p, '"'); snprintf(g_password, sizeof g_password, "%.*s", e ? (int)(e - p) : 127, p); }
        else sscanf(p, "%127s", g_password);
    }
    // The instance's Admin\Startup.txt runs after the command line: its admin.password is the one RCON really takes
    // (server 2's command line said QNWSxJYl, its Startup.txt 8zrdbcGi - the login failed silently, so no level / round
    // events reached BotMod, a round restart went unnoticed and a bot spawned with the old round's freed kit crashed it)
    if ((p = strstr(cl, "-serverInstancePath ")) != NULL) {
        char dir[MAX_PATH] = "", path[MAX_PATH + 32], line[256];
        p += 20;
        if (*p == '"') { p++; const char *e = strchr(p, '"'); snprintf(dir, sizeof dir, "%.*s", e ? (int)(e - p) : MAX_PATH - 1, p); }
        else sscanf(p, "%259s", dir);
        snprintf(path, sizeof path, "%s\\Admin\\Startup.txt", dir);
        FILE *f = fopen(path, "r");
        if (f) {
            while (fgets(line, sizeof line, f)) {
                const char *q = line;
                while (*q == ' ' || *q == '\t') q++;
                if (strncmp(q, "admin.password ", 15)) continue;
                q += 15;
                while (*q == ' ') q++;
                if (*q == '"') { q++; const char *e = strchr(q, '"'); if (e) snprintf(g_password, sizeof g_password, "%.*s", (int)(e - q), q); }
                else { char w[128] = ""; if (sscanf(q, "%127s", w) == 1) snprintf(g_password, sizeof g_password, "%s", w); }
            }
            fclose(f);
        }
    }
}

static int send_words(unsigned seq, int n, const char **words)
{
    char buf[1024];
    unsigned size = 12;
    for (int i = 0; i < n; i++) size += 4 + (unsigned)strlen(words[i]) + 1;
    if (size > sizeof buf) return 0;
    memcpy(buf, &seq, 4);
    memcpy(buf + 4, &size, 4);
    memcpy(buf + 8, &n, 4);
    unsigned o = 12;
    for (int i = 0; i < n; i++) {
        unsigned len = (unsigned)strlen(words[i]);
        memcpy(buf + o, &len, 4);
        memcpy(buf + o + 4, words[i], len + 1);
        o += 4 + len + 1;
    }
    return send(g_sock, buf, (int)size, 0) == (int)size;
}

static int recv_all(char *buf, int n)
{
    int got = 0;
    while (got < n && !g_stop) {
        int r = recv(g_sock, buf + got, n - got, 0);
        if (r == SOCKET_ERROR && WSAGetLastError() == WSAETIMEDOUT) { if (got == 0) return -1; continue; }
        if (r <= 0) return 0;
        got += r;
    }
    return !g_stop;
}

// Reads one packet: 1 ok, -1 timeout before any byte, 0 connection lost. words point into buf.
static int recv_packet(unsigned *seq, int *nw, char **words, int maxWords, char *buf, int bufLen)
{
    unsigned hdr[3];
    int r = recv_all((char *)hdr, 12);
    if (r <= 0) return r;
    unsigned size = hdr[1];
    if (size < 12 || size - 12 > (unsigned)bufLen) return 0;
    if (recv_all(buf, (int)(size - 12)) <= 0) return 0;
    *seq = hdr[0];
    *nw = 0;
    unsigned o = 0;
    for (unsigned i = 0; i < hdr[2] && o + 4 <= size - 12; i++) {
        unsigned len;
        memcpy(&len, buf + o, 4);
        if (o + 4 + len + 1 > size - 12) break;
        if (*nw < maxWords) words[(*nw)++] = buf + o + 4;
        buf[o + 4 + len] = 0;
        o += 4 + len + 1;
    }
    return 1;
}

static int request(int n, const char **words)
{
    unsigned seq = ++g_seq & 0x3fffffff;
    return send_words(seq, n, words);
}

static int connect_rcon(void)
{
    g_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_sock == INVALID_SOCKET) return 0;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((u_short)g_port);
    a.sin_addr.s_addr = inet_addr(g_host);
    if (connect(g_sock, (struct sockaddr *)&a, sizeof a) != 0) { closesocket(g_sock); g_sock = INVALID_SOCKET; return 0; }
    DWORD tmo = 200;
    setsockopt(g_sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof tmo);
    const char *login[2] = {"login.plainText", g_password};
    const char *events[2] = {"admin.eventsEnabled", "true"};
    return request(2, login) && request(2, events);
}

// An event from the server (sent unasked): acknowledge it, and queue "!" chat lines for the game thread.
static volatile LONG g_levelLoads, g_roundsOver, g_lastWinner = -1;
void chat_level_events(long *loads, long *roundsOver) { *loads = g_levelLoads; *roundsOver = g_roundsOver; }
long chat_last_round_winner(void) { return g_lastWinner; }

static void handle_event(unsigned seq, int nw, char **w)
{
    const char *ok[1] = {"OK"};
    send_words(seq | 0x40000000 | 0x80000000, 1, ok);
    if (nw >= 1 && !strcmp(w[0], "server.onLevelLoaded")) InterlockedIncrement(&g_levelLoads);
    if (nw >= 1 && !strncmp(w[0], "server.onRoundOver", 18)) {   // (2026-10-28 co-op: logged in full - what a mission pass / fail sends)
        char all[256]; int k = 0;
        for (int i = 0; i < nw && k < (int)sizeof all - 40; i++) k += snprintf(all + k, sizeof all - k, "%s%.32s", i ? " " : "", w[i]);
        if (g_log) g_log("chat: event %s", all);
        if (!strcmp(w[0], "server.onRoundOver")) {
            if (nw >= 2) InterlockedExchange(&g_lastWinner, atol(w[1]));
            InterlockedIncrement(&g_roundsOver);
        }
    }
    if (nw >= 3 && !strcmp(w[0], "player.onChat") && w[2][0] == '!' && strcmp(w[1], "Server")) {
        EnterCriticalSection(&g_lock);
        int next = (g_inHead + 1) % QUEUE_LEN;
        if (next != g_inTail) {
            snprintf(g_in[g_inHead].who, sizeof g_in[g_inHead].who, "%s", w[1]);
            snprintf(g_in[g_inHead].text, sizeof g_in[g_inHead].text, "%s", w[2]);
            g_inHead = next;
        }
        LeaveCriticalSection(&g_lock);
    }
}

// Sends a request and waits (up to ~3 s) for its response; events arriving meanwhile are handled.
// Returns the number of response words (pointing into buf), 0 on failure.
static int rcon_call(int n, const char **words, char **out, int maxOut, char *buf, int bufLen)
{
    unsigned seq = ++g_seq & 0x3fffffff;
    if (!send_words(seq, n, words)) return 0;
    for (int waited = 0; waited < 15; ) {
        unsigned rs;
        int nw;
        int r = recv_packet(&rs, &nw, out, maxOut, buf, bufLen);
        if (r == -1) { waited++; continue; }
        if (r == 0) return 0;
        if (!(rs & 0x40000000)) { handle_event(rs, nw, out); continue; }
        if ((rs & 0x3fffffff) == seq) return nw;
    }
    return 0;
}

// Friendly names of the map codes (for the !map list).
static const char *map_friendly(const char *code)
{
    static const char *const t[][2] = {
        {"MP_001", "bazaar"}, {"MP_003", "teheran"}, {"MP_007", "caspian"}, {"MP_011", "seine"}, {"MP_012", "firestorm"},
        {"MP_013", "damavand"}, {"MP_017", "canals"}, {"MP_018", "kharg"}, {"MP_Subway", "metro"}, {"XP1_001", "karkand"},
        {"XP1_002", "oman"}, {"XP1_003", "sharqi"}, {"XP1_004", "wake"}, {"XP2_Factory", "scrapmetal"}, {"XP2_Office", "operation925"},
        {"XP2_Palace", "donya"}, {"XP2_Skybar", "ziba"}, {"XP3_Desert", "bandar"}, {"XP3_Alborz", "alborz"}, {"XP3_Shield", "shield"},
        {"XP3_Valley", "valley"}, {"XP4_Quake", "markaz"}, {"XP4_FD", "talah"}, {"XP4_Rubble", "epicenter"}, {"XP4_Parl", "azadi"},
        {"XP5_001", "riverside"}, {"XP5_002", "nebandan"}, {"XP5_003", "kiasar"}, {"XP5_004", "sabalan"}};
    for (int k = 0; k < (int)(sizeof t / sizeof t[0]); k++) if (!_stricmp(t[k][0], code)) return t[k][1];
    return code;
}
// !map <code>: only a map in THIS server's rotation (the user, 2026-10-13: each server keeps its own playlist and game
// modes) - the rotation entry is made next and started, in the mode the rotation has for it.
static void do_map(const char *code, const char *who)
{
    static char buf[65536];
    char *w[1024];
    if (!_stricmp(code, "restart")) {   // !map restart: the current round over again (the user, 2026-10-16)
        char say[200];
        snprintf(say, sizeof say, "[BotMod] %s is restarting the round", who);
        InterlockedIncrement(&g_roundsOver);   // (bots hold their spawns now: the round is torn down - spawning into it, then again into the
                                               //  reload with the old one's objects, crashed server 3 on a double restart, 2026-10-22)
        const char *s[3] = {"admin.say", say, "all"};
        rcon_call(3, s, w, 1024, buf, sizeof buf);
        Sleep(3000);
        const char *rr[1] = {"mapList.restartRound"};
        int rn = rcon_call(1, rr, w, 1024, buf, sizeof buf);
        g_log("chat: %s restarted the round (%s)", who, rn > 0 ? w[0] : "no answer");
        return;
    }
    const char *list[1] = {"mapList.list"};
    int n = rcon_call(1, list, w, 1024, buf, sizeof buf);
    int idx = -1, count = 0, per = 3;
    char mode[64] = "";
    if (n >= 3 && !strcmp(w[0], "OK")) {
        count = atoi(w[1]);
        per = atoi(w[2]);
        if (per < 2) per = 3;
        for (int k = 0; k < count && 3 + k * per + 1 < n; k++)
            if (!_stricmp(w[3 + k * per], code)) { idx = k; snprintf(mode, sizeof mode, "%s", w[3 + k * per + 1]); break; }
    }
    if (idx < 0) { chat_say("[BotMod] %s: %s is not in this server's map list - type !map to see it", who, map_friendly(code)); return; }
    char say[200], idxs[16];
    snprintf(idxs, sizeof idxs, "%d", idx);
    const char *next[2] = {"mapList.setNextMapIndex", idxs};
    n = rcon_call(2, next, w, 1024, buf, sizeof buf);
    if (n < 1 || strcmp(w[0], "OK")) { chat_say("[BotMod] %s: could not select %s (%s)", who, code, n ? w[0] : "no answer"); return; }
    snprintf(say, sizeof say, "[BotMod] %s is changing the map to %s (%s)", who, map_friendly(code), mode);
    const char *s[3] = {"admin.say", say, "all"};
    rcon_call(3, s, w, 1024, buf, sizeof buf);
    Sleep(3000);   // let players read it: the map change clears the chat box (this thread only; the game keeps running)
    const char *run[1] = {"mapList.runNextRound"};
    rcon_call(1, run, w, 1024, buf, sizeof buf);
    g_log("chat: %s changed the map to %s (rotation index %d, %s)", who, code, idx, mode);
}
// !map with no name: this server's rotation, a few names per chat line
static void do_map_list(void)
{
    static char buf[65536];
    char *w[1024];
    const char *list[1] = {"mapList.list"};
    int n = rcon_call(1, list, w, 1024, buf, sizeof buf);
    if (n < 3 || strcmp(w[0], "OK")) { chat_say("[BotMod] cannot read this server's map list"); return; }
    int count = atoi(w[1]), per = atoi(w[2]);
    if (per < 2) per = 3;
    // just the names, once each, packed into as few chat lines as fit (the user: long lists scrolled off the screen; the
    // server already decides the mode)
    char line[128];
    const char *done[64];
    int nd = 0, m = snprintf(line, sizeof line, "[BotMod] maps:");
    for (int k = 0; k < count && 3 + k * per < n && nd < 64; k++) {
        const char *nm = map_friendly(w[3 + k * per]);
        int dup = 0;
        for (int q = 0; q < nd; q++) if (!strcmp(done[q], nm)) dup = 1;
        if (dup) continue;
        done[nd++] = nm;
        if (m + 1 + (int)strlen(nm) > 120) { chat_say("%s", line); m = snprintf(line, sizeof line, "[BotMod]"); }
        m += snprintf(line + m, sizeof line - m, " %s", nm);
    }
    chat_say("%s", line);
}

static volatile int g_maxPlayers;
int chat_max_players(void) { return g_maxPlayers; }
static char g_gameMode[64];          // serverInfo word 4, e.g. ConquestLarge0, TankSuperiority0, AirSuperiority0
static volatile long g_modeLoads = -1;   // the level-load count it was read at (re-read after each load)
void chat_game_mode(char *out, int n) { snprintf(out, n, "%s", g_gameMode); }
// serverInfo: OK, name, players, maxPlayers, ... - asked now and then so admin changes to vars.maxPlayers are picked up
static void poll_server_info(void)
{
    static DWORD lastTick;
    DWORD now = GetTickCount();
    if (g_maxPlayers && now - lastTick < 30000 && g_modeLoads == g_levelLoads) return;   // (at once after a level load: the mode)
    lastTick = now;
    static char buf[4096];
    char *w[16];
    const char *info[1] = {"serverInfo"};
    int n = rcon_call(1, info, w, 16, buf, sizeof buf);
    if (n >= 4 && !strcmp(w[0], "OK")) {
        int m = atoi(w[3]);
        if (m > 0 && m != g_maxPlayers) { g_log("chat: server allows %d players", m); g_maxPlayers = m; }
        if (n >= 5 && strcmp(w[4], g_gameMode)) { g_log("chat: game mode %s", w[4]); snprintf(g_gameMode, sizeof g_gameMode, "%s", w[4]); }
        g_modeLoads = g_levelLoads;
    }
}

static DWORD WINAPI chat_worker(LPVOID unused)
{
    (void)unused;
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    static char buf[16384];
    int warned = 0;
    while (!g_stop) {
        if (g_sock == INVALID_SOCKET) {
            if (!connect_rcon()) {
                if (!warned++) g_log("chat: cannot reach RCON at %s:%d yet - retrying", g_host, g_port);
                for (int t = 0; t < 30 && !g_stop; t++) Sleep(100);
                continue;
            }
            g_log("chat: connected to RCON %s:%d", g_host, g_port);
            warned = 0;
        }
        poll_server_info();
        // outgoing messages
        for (;;) {
            char msg[LINE_LEN];
            EnterCriticalSection(&g_lock);
            int have = g_outHead != g_outTail;
            if (have) { memcpy(msg, g_out[g_outTail], LINE_LEN); g_outTail = (g_outTail + 1) % QUEUE_LEN; }
            LeaveCriticalSection(&g_lock);
            if (!have) break;
            if (msg[0] == '\x01') {   // "\x01map\t<code>\t<who>": needs replies, done here
                char *code = strchr(msg, '\t'), *who = code ? strchr(code + 1, '\t') : NULL;
                if (!strncmp(msg + 1, "maplist", 7)) { do_map_list(); continue; }
                if (code && who) { *who = 0; do_map(code + 1, who + 1); }
                continue;
            }
            const char *words[12];
            int n = 0;
            for (char *s = msg; s && n < 12; ) {   // queued commands are tab-separated words
                words[n++] = s;
                char *tab = strchr(s, '\t');
                if (tab) { *tab = 0; s = tab + 1; } else s = NULL;
            }
            request(n, words);
        }
        // incoming packets (responses and events)
        unsigned seq;
        int nw;
        char *w[8];
        int r = recv_packet(&seq, &nw, w, 8, buf, sizeof buf);
        if (r == -1) continue;
        if (r == 0) {
            if (!g_stop) g_log("chat: RCON connection lost - reconnecting");
            closesocket(g_sock);
            g_sock = INVALID_SOCKET;
            continue;
        }
        if (seq & 0x40000000) continue;   // response to one of our requests
        handle_event(seq, nw, w);
    }
    if (g_sock != INVALID_SOCKET) { closesocket(g_sock); g_sock = INVALID_SOCKET; }
    WSACleanup();
    return 0;
}

void chat_start(ChatLogFn log)
{
    g_log = log;
    InitializeCriticalSection(&g_lock);
    read_command_line();
    if (!g_password[0]) { g_log("chat: no +admin.password on the server command line - chat disabled"); return; }
    g_stop = 0;
    g_thread = CreateThread(NULL, 0, chat_worker, NULL, 0, NULL);
}

void chat_stop(void)
{
    if (!g_thread) return;
    g_stop = 1;
    WaitForSingleObject(g_thread, 5000);
    CloseHandle(g_thread);
    g_thread = NULL;
    DeleteCriticalSection(&g_lock);
}

int chat_poll(char *who, int whoLen, char *text, int textLen)
{
    if (!g_thread) return 0;
    int have = 0;
    EnterCriticalSection(&g_lock);
    if (g_inHead != g_inTail) {
        snprintf(who, whoLen, "%s", g_in[g_inTail].who);
        snprintf(text, textLen, "%s", g_in[g_inTail].text);
        g_inTail = (g_inTail + 1) % QUEUE_LEN;
        have = 1;
    }
    LeaveCriticalSection(&g_lock);
    return have;
}

void chat_say_team(int team, const char *fmt, ...)   // admin.say to one team only
{
    if (!g_thread) return;
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    for (char *t = msg; *t; t++) if (*t == '\t') *t = ' ';
    EnterCriticalSection(&g_lock);
    int next = (g_outHead + 1) % QUEUE_LEN;
    if (next != g_outTail) { snprintf(g_out[g_outHead], LINE_LEN, "admin.say\t%.120s\tteam\t%d", msg, team); g_outHead = next; }
    LeaveCriticalSection(&g_lock);
}
void chat_say(const char *fmt, ...)
{
    if (!g_thread) return;
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    for (char *t = msg; *t; t++) if (*t == '\t') *t = ' ';
    EnterCriticalSection(&g_lock);
    for (const char *p = msg; *p; ) {   // chat lines are limited to 128 characters
        int next = (g_outHead + 1) % QUEUE_LEN;
        if (next == g_outTail) break;
        snprintf(g_out[g_outHead], LINE_LEN, "admin.say\t%.120s\tall", p);
        g_outHead = next;
        p += strlen(p) > 120 ? 120 : strlen(p);
    }
    LeaveCriticalSection(&g_lock);
}

void chat_map(const char *code, const char *who)
{
    chat_rcon("\x01map\t%s\t%s", code, who);
}

void chat_map_list(void) { chat_rcon("maplist"); }

void chat_rcon(const char *fmt, ...)
{
    if (!g_thread) return;
    char cmd[LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(cmd, sizeof cmd, fmt, ap);
    va_end(ap);
    EnterCriticalSection(&g_lock);
    int next = (g_outHead + 1) % QUEUE_LEN;
    if (next != g_outTail) { memcpy(g_out[g_outHead], cmd, LINE_LEN); g_outHead = next; }
    LeaveCriticalSection(&g_lock);
}
