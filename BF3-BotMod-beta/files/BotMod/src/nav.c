// Automatic navigation grid: every cell is probed with downward rays that peel off up to NAV_LAYERS stacked
// surfaces (ground height + slope + headroom), every neighbour pair with two horizontal rays (walls/fences) and a
// step-height check. Paths are planned with A* over (cell, layer) nodes and string-pulled.
#include <windows.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "nav.h"

#define PROBE_TOP        1200.0f  // start height of ground probes
#define PROBE_BOTTOM     -200.0f
#define PEEL_OFFSET      0.25f    // restart the next downward probe this far below a found surface
#define MIN_NORMAL_Y     0.70f    // ~45 degrees steepest walkable slope
#define MIN_HEADROOM     1.8f     // standing room needed under the surface above
#define MAX_STEP_UP      0.9f     // vertical step a soldier can walk up between adjacent cells
#define MAX_DROP         4.0f     // ledge a soldier may walk off (one-way link down), e.g. hangar deck -> sea
#define MAX_STAIR_UP     2.6f     // neighbours this far apart connect if the ground between them is a staircase (1.6: Kharg's 2.4 m flight
                                  //  up to an M-COM landing linked one way only - nobody routed up to it, 2026-10-23)
#define MAX_STAIR_STEP   0.7f     // largest single rise between the 0.5 m samples of a staircase
#define KNEE_HEIGHT      0.6f     // clearance rays between cells
#define CHEST_HEIGHT     1.4f
#define NAV_FILE_MAGIC   0x3556414e  // "NAV5" (NAV4 + water flags, fence crossings, top 2 + bottom 4 surfaces)
#define NAV_FILE_MAGIC4  0x3456414e  // "NAV4" (still loads: no fences / water)
#define FENCE_COST       40.0f       // m of walking a knifed fence crossing is worth: go round short fences, cut long ones
#define WATER_COST_MUL   4.0f        // swimming
#define L                NAV_LAYERS

static const int kDir[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
// ---- Search state, one set per thread (2026-10-25: vehicle route searches run on a worker thread - 140-476 ms each on the game thread
// with 50 bots, a hitch every 20 s): the game thread's and the route worker's. Everything a search reads or writes besides the grid
// itself lives here; the setters below set the calling thread's own.
#define NAV_MAX_BLOCKERS 32
#define NAV_MAX_OUT 256
typedef struct NavCtx {
    const unsigned char *pathMask;   // vehicles: only these nodes (NULL = soldiers)
    int longLegs;                    // boats: straight lines across open water
    int partialOk, partialUsed;
    float blk[NAV_MAX_BLOCKERS][4];
    int blkN, blkOff;
    float blkBox[4];
    float vehHalf, hMul;
    char lastError[128];
    short outLink[NAV_MAX_OUT];
    int rescuing;
    int bufN, heapCap;               // A* buffers (per grid size)
    float *gs;
    int *from;
    unsigned *stamp, curStamp;
    void *heap;
    int *nodesOut, nodesMax;         // set: the search hands back its raw node chain (no string pulling)
} NavCtx;
static NavCtx g_navCtx[2] = {{.vehHalf = 1.5f, .hMul = 1.0f}, {.vehHalf = 1.5f, .hMul = 1.0f}};
static volatile unsigned g_navWorkerTid;
void nav_set_worker_thread(unsigned tid) { g_navWorkerTid = tid; }
static inline NavCtx *nav_ctx(void)
{
    unsigned tid;
    __asm__("movl %%fs:0x24, %0" : "=r"(tid));   // (the TEB's thread id - no call)
    return tid == g_navWorkerTid ? &g_navCtx[1] : &g_navCtx[0];
}
#define g_pathMask    (nav_ctx()->pathMask)
#define g_longLegs    (nav_ctx()->longLegs)
#define g_partialOk   (nav_ctx()->partialOk)
#define g_partialUsed (nav_ctx()->partialUsed)
#define g_blk         (nav_ctx()->blk)
#define g_blkN        (nav_ctx()->blkN)
#define g_blkOff      (nav_ctx()->blkOff)
#define g_blkBox      (nav_ctx()->blkBox)
#define g_vehHalf     (nav_ctx()->vehHalf)
#define g_hMul        (nav_ctx()->hMul)
#define g_lastError   (nav_ctx()->lastError)
#define g_outLink     (nav_ctx()->outLink)
void nav_set_long_legs(int on) { g_longLegs = on; }
// A* expansion cap override (2026-10-29 co-op: one 550 m route to a single objective needs more than 60000; 0 = the defaults)
static int s_maxExpanded;
void nav_set_max_expanded(int n) { s_maxExpanded = n; }
const unsigned char *nav_path_mask(void) { return g_pathMask; }
int nav_long_legs(void) { return g_longLegs; }
void nav_vehicle_state(float *half, float *hmul) { *half = g_vehHalf; *hmul = g_hMul; }
void nav_set_vehicle_state(float half, float hmul) { g_vehHalf = half; g_hMul = hmul; }
int nav_get_blockers(float *xyzr, int max) { int n = g_blkN < max ? g_blkN : max; if (n > 0) memcpy(xyzr, g_blk, (size_t)n * sizeof g_blk[0]); return n; }
void nav_set_last_error(const char *s) { snprintf(g_lastError, sizeof g_lastError, "%s", s); }
// Partial routes (boats, 2026-10-19: Noshahr's flag 0 sits behind a quay wall the water map counted as shallows - every boat
// route to it searched the whole sea and failed): when the goal cannot be reached, the route to the explored node nearest it.
void nav_set_partial(int on) { g_partialOk = on; g_partialUsed = 0; }
int nav_partial_used(void) { return g_partialUsed; }
int nav_partial_ok(void) { return g_partialOk; }
void nav_set_partial_used(int used) { g_partialUsed = used; }
void nav_set_path_mask(const unsigned char *mask) { g_pathMask = mask; }
// (g_blkBox: all of them, x0, z0, x1, z1 - most nodes are nowhere near one: no per-blocker test for them)
void nav_set_blockers(const float *xyzr, int n)
{
    if (n > NAV_MAX_BLOCKERS) n = NAV_MAX_BLOCKERS;
    g_blkN = xyzr && n > 0 ? n : 0;
    if (g_blkN) memcpy(g_blk, xyzr, (size_t)g_blkN * sizeof g_blk[0]);
    g_blkBox[0] = g_blkBox[1] = 1e30f; g_blkBox[2] = g_blkBox[3] = -1e30f;
    for (int k = 0; k < g_blkN; k++) {
        float r = g_blk[k][3];
        if (g_blk[k][0] - r < g_blkBox[0]) g_blkBox[0] = g_blk[k][0] - r;
        if (g_blk[k][2] - r < g_blkBox[1]) g_blkBox[1] = g_blk[k][2] - r;
        if (g_blk[k][0] + r > g_blkBox[2]) g_blkBox[2] = g_blk[k][0] + r;
        if (g_blk[k][2] + r > g_blkBox[3]) g_blkBox[3] = g_blk[k][2] + r;
    }
}
static int node_blocked(const NavGrid *g, int node)   // a vehicle stands on it (routes on foot / swimming and vehicle routes)
{
    if (!g_blkN || g_blkOff) return 0;
    int cl = node / NAV_LAYERS;
    float x = g->x0 + (cl % g->w) * g->cell, z = g->z0 + (cl / g->w) * g->cell, y = g->height[node];
    if (x < g_blkBox[0] || x > g_blkBox[2] || z < g_blkBox[1] || z > g_blkBox[3]) return 0;
    for (int k = 0; k < g_blkN; k++) {
        float dx = x - g_blk[k][0], dz = z - g_blk[k][2];
        if (dx * dx + dz * dz < g_blk[k][3] * g_blk[k][3] && fabsf(y - g_blk[k][1]) < 4.0f) return 1;
    }
    return 0;
}
// g_vehHalf - vehicles: half the width kept clear on a straight short cut; g_hMul - the taxi map's dear off-street classes made A*
// search 60000 nodes on Karkand: greedier there
void nav_set_vehicle_half(float h) { g_vehHalf = h > 0 ? h : 1.5f; g_hMul = h > 0 ? 2.0f : 1.0f; }
static const unsigned char *g_treeDistSet;
static const NavGrid *g_treeGrid;   // the grid the table belongs to (a table from another map is never read)
static int g_treeW, g_treeH;
void nav_set_tree_dist(const NavGrid *g, const unsigned char *dist) { g_treeGrid = g; g_treeDistSet = dist; g_treeW = g ? g->w : 0; g_treeH = g ? g->h : 0; }
#define TREE_DIST(g) ((g) == g_treeGrid && (g)->w == g_treeW && (g)->h == g_treeH ? g_treeDistSet : NULL)
#define TREE_NEAR_DM 12   // soldiers: within 1.2 m of a trunk
static const unsigned char *g_roadMaskSet;   // soldiers: road classes (cheaper along roads)
static const NavGrid *g_roadGrid;
static int g_roadW, g_roadH;
void nav_set_road_mask(const NavGrid *g, const unsigned char *mask) { g_roadGrid = g; g_roadMaskSet = mask; g_roadW = g ? g->w : 0; g_roadH = g ? g->h : 0; }
#define ROAD_MASK(g) ((g) == g_roadGrid && (g)->w == g_roadW && (g)->h == g_roadH ? g_roadMaskSet : NULL)

static void *nav_alloc(size_t n) { return HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, n); }
static unsigned g_gridSerial;
static NavGrid *new_grid(void)
{
    NavGrid *g = nav_alloc(sizeof *g);
    if (!g_gridSerial) g_gridSerial = GetTickCount() << 8;   // a hot reload restarts the count while the old grid lives on
    if (g) g->serial = ++g_gridSerial;
    return g;
}
static void nav_release(void *p) { if (p) HeapFree(GetProcessHeap(), 0, p); }
void nav_release_search_buffers(void)   // the A* buffers of both threads (a core unload: they leaked ~16 bytes a node each reload)
{
    for (int k = 0; k < 2; k++) {
        NavCtx *X = &g_navCtx[k];
        nav_release(X->gs); nav_release(X->from); nav_release(X->stamp); nav_release(X->heap);
        X->gs = NULL; X->from = NULL; X->stamp = NULL; X->heap = NULL; X->bufN = 0;
    }
}

static void alloc_arrays(NavGrid *g)
{
    size_t n = (size_t)g->w * g->h * L;
    g->height = nav_alloc(n * sizeof(float));
    g->flags = nav_alloc(n);
    g->edges = nav_alloc(n);
    g->fences = nav_alloc(n);
}

NavGrid *nav_create(const char *level, float x0, float z0, float x1, float z1, float cell)
{
    NavGrid *g = new_grid();
    strncpy(g->level, level, sizeof g->level - 1);
    g->cell = cell;
    g->x0 = (x0 < x1 ? x0 : x1) + cell * 0.5f;
    g->z0 = (z0 < z1 ? z0 : z1) + cell * 0.5f;
    g->w = (int)(fabsf(x1 - x0) / cell);
    g->h = (int)(fabsf(z1 - z0) / cell);
    alloc_arrays(g);
    return g;
}

static void nav_blocks_forget(const NavGrid *g);
static unsigned char *g_stairOk;          // bit d: the link in straight direction d is a real staircase up (re-checked)
static const NavGrid *g_stairOkGrid;
static int g_stairOkW, g_stairOkH;
static void (*g_beforeFree)(const NavGrid *);
void nav_set_free_hook(void (*f)(const NavGrid *)) { g_beforeFree = f; }
void nav_free(NavGrid *g)
{
    if (g && g_beforeFree) g_beforeFree(g);   // (the route worker may be searching it)
    if (!g) return;
    if (g_stairOkGrid == g) { nav_release(g_stairOk); g_stairOk = NULL; g_stairOkGrid = NULL; }
    nav_blocks_forget(g);   // its stuck-spot blocks die with it (a new map's grid can land at the same address)
    nav_release(g->height);
    nav_release(g->flags);
    nav_release(g->edges);
    nav_release(g->fences);
    nav_release(g);
}

static double now_ms(void)
{
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1000.0 / (double)f.QuadPart;
}

static void cell_pos(const NavGrid *g, int cx, int cz, float *x, float *z)
{
    *x = g->x0 + cx * g->cell;
    *z = g->z0 + cz * g->cell;
}

int nav_cell_index(const NavGrid *g, float x, float z)
{
    int cx = (int)floorf((x - g->x0) / g->cell + 0.5f);
    int cz = (int)floorf((z - g->z0) / g->cell + 0.5f);
    if (cx < 0 || cz < 0 || cx >= g->w || cz >= g->h) return -1;
    return cz * g->w + cx;
}

// Peels surfaces top-down (up to MAX_PEELS rays) and keeps the first L *walkable* ones: flat enough and with headroom
// under the surface above. Non-walkable surfaces (vehicle wrecks, railings, roof edges) still limit headroom but do not
// use up a layer, so the ground under an overpass next to a wreck is still found.
#define MAX_PEELS 16
#define MAX_SURFACES 16
static NavRayFn g_rayNoTerrain;   // same ray ignoring terrain: tells terrain hits apart (nothing walkable lies under terrain)
void nav_set_terrain_test(NavRayFn rayIgnoringTerrain) { g_rayNoTerrain = rayIgnoringTerrain; }
static NavHitClassFn g_classify;
void nav_set_hit_classifier(NavHitClassFn fn) { g_classify = fn; }

// A cell with more walkable surfaces than layers (a bridge over a road over a basement) keeps the top 2 and the
// bottom L-2: the deck you walk on and the ground under it matter more than the middle storeys. Water ends the probe:
// nothing under its surface is walked on.
static int probe_ground(NavGrid *g, NavRayFn ray, int cell)
{
    float x, z, p[3], n[3], top = PROBE_TOP, above = 1e30f, hs[MAX_SURFACES];
    unsigned char wet[MAX_SURFACES];
    cell_pos(g, cell % g->w, cell / g->w, &x, &z);
    int rays = 0, ns = 0;
    for (int peel = 0; peel < MAX_PEELS && ns < MAX_SURFACES; peel++) {
        float a[3] = {x, top, z}, b[3] = {x, PROBE_BOTTOM, z};
        rays++;
        if (!ray(a, b, p, n)) break;
        int water = g_classify && g_classify() == NAV_HIT_WATER;
        if (water || (n[1] >= MIN_NORMAL_Y && above - p[1] >= MIN_HEADROOM)) { hs[ns] = p[1]; wet[ns] = (unsigned char)water; ns++; }
        if (water) break;
        above = p[1];
        top = p[1] - PEEL_OFFSET;
        // Terrain is a one-sided heightfield: peeling on would find the sea plane that continues under the land.
        if (g_rayNoTerrain) {
            float q[3];
            rays++;
            if (!g_rayNoTerrain(a, b, q, NULL) || q[1] < p[1] - 0.05f) break;
        }
    }
    for (int k = 0, l = 0; k < ns && l < L; k++) {
        if (ns > L && k >= 2 && k < ns - (L - 2)) continue;   // a middle storey
        int node = cell * L + l++;
        g->height[node] = hs[k];
        g->flags[node] |= NAV_WALKABLE | (wet[k] ? NAV_WATER : 0);
    }
    return rays;
}

// Layer of neighbour cell `nc` that node (c, l) connects to: the walkable one within a step, closest in height.
static int neighbour_layer(const NavGrid *g, int c, int l, int nc, int diagonal)
{
    float h = g->height[c * L + l], best = MAX_STEP_UP * (diagonal ? 1.41f : 1.0f);
    int bl = -1;
    for (int k = 0; k < L; k++) {
        int nn = nc * L + k;
        if (!(g->flags[nn] & NAV_WALKABLE)) continue;
        float dh = fabsf(g->height[nn] - h);
        if (dh <= best) { best = dh; bl = k; }
    }
    return bl;
}

// Layer of neighbour cell `nc` that is up a staircase from node (c, l): the closest walkable surface between
// MAX_STEP_UP and MAX_STAIR_UP above it.
static int stair_layer(const NavGrid *g, int c, int l, int nc)
{
    float h = g->height[c * L + l], best = MAX_STAIR_UP;
    int bl = -1;
    for (int k = 0; k < L; k++) {
        int nn = nc * L + k;
        if (!(g->flags[nn] & NAV_WALKABLE)) continue;
        float dh = g->height[nn] - h;
        if (dh > MAX_STEP_UP && dh <= best) { best = dh; bl = k; }
    }
    return bl;
}

// Ground between (ax, ha, az) and (bx, hb, bz) rises in steps a soldier walks up: samples at 1/4, 1/2, 3/4, rising
// gently - and at least two of them part-way up (a wall face shows only the bottom and the top: gabions, kerbs).
static int stairs_between(NavRayFn ray, float ax, float ha, float az, float bx, float hb, float bz)
{
    float prev = ha;
    int between = 0;
    for (int k = 1; k <= 4; k++) {
        float t = k * 0.25f, h;
        if (k < 4) {
            float x = ax + (bx - ax) * t, z = az + (bz - az) * t, p[3], n[3];
            float a[3] = {x, hb + 1.0f, z}, b[3] = {x, ha - 1.0f, z};
            if (!ray(a, b, p, n) || n[1] < MIN_NORMAL_Y) return 0;
            h = p[1];
            if (h > ha + 0.15f && h < hb - 0.15f) between++;
        } else h = hb;
        if (h - prev > MAX_STAIR_STEP || h < prev - 0.3f) return 0;
        prev = h;
    }
    return between >= 2;
}

// Knee and chest rays between two ground points shifted sideways by `off` (m): clear?
static int pair_clear(NavRayFn ray, float ax, float ha, float az, float bx, float hb, float bz, float off)
{
    float dx = bx - ax, dz = bz - az, len = sqrtf(dx * dx + dz * dz);
    float px = -dz / len * off, pz = dx / len * off;
    const float heights[2] = {KNEE_HEIGHT, CHEST_HEIGHT};
    for (int k = 0; k < 2; k++) {
        float a[3] = {ax + px, ha + heights[k], az + pz}, b[3] = {bx + px, hb + heights[k], bz + pz};
        if (ray(a, b, NULL, NULL)) return 0;
    }
    return 1;
}
static const float kDoorOffsets[4] = {0.5f, -0.5f, 0.9f, -0.9f};
// A doorway is one a BODY fits through: the line at the offset and 0.35 m either side of it clear (one line slipped past the
// edge of a 2 m door - Noshahr's RU base, 2026-10-26: the cells above and below a one-cell door both got "doorway" links, a dozen
// bots walked into the wall beside it at every round start)
#define DOOR_HALF_BODY 0.35f
static int door_gap_ok(NavRayFn ray, float ax, float ha, float az, float bx, float hb, float bz, float off)
{
    if (!pair_clear(ray, ax, ha, az, bx, hb, bz, off) || !pair_clear(ray, ax, ha, az, bx, hb, bz, off - DOOR_HALF_BODY) ||
        !pair_clear(ray, ax, ha, az, bx, hb, bz, off + DOOR_HALF_BODY)) return 0;
    // ...and the step SIDEWAYS onto that line, at both ends (a line 0.9 m over ran on the far side of the pocket's own wall - a
    // "doorway" south out of a corner nobody can walk; Noshahr 2026-10-26)
    float dx = bx - ax, dz = bz - az, len = sqrtf(dx * dx + dz * dz);
    if (len < 0.01f) return 1;
    float side = off + (off > 0 ? DOOR_HALF_BODY : -DOOR_HALF_BODY);
    float px = -dz / len * side, pz = dx / len * side;
    const float heights[2] = {KNEE_HEIGHT, CHEST_HEIGHT};
    for (int k = 0; k < 2; k++) {
        float a0[3] = {ax, ha + heights[k], az}, a1[3] = {ax + px, ha + heights[k], az + pz};
        float b0[3] = {bx, hb + heights[k], bz}, b1[3] = {bx + px, hb + heights[k], bz + pz};
        if (ray(a0, a1, NULL, NULL) || ray(b0, b1, NULL, NULL)) return 0;
    }
    return 1;
}

// Layer of neighbour cell `nc` that node (c, l) can drop down to when there is no step: the highest walkable surface
// between MAX_STEP_UP and MAX_DROP below it.
static int drop_layer(const NavGrid *g, int c, int l, int nc)
{
    float h = g->height[c * L + l], bestH = -1e30f;
    int bl = -1;
    for (int k = 0; k < L; k++) {
        int nn = nc * L + k;
        if (!(g->flags[nn] & NAV_WALKABLE)) continue;
        float dh = h - g->height[nn];
        if (dh > MAX_STEP_UP && dh <= MAX_DROP && g->height[nn] > bestH) { bestH = g->height[nn]; bl = k; }
    }
    return bl;
}

// One-way drop from node (c, l) into cell nc: walk horizontally off the ledge (knee/chest rays at the upper height).
static int probe_drop(NavGrid *g, NavRayFn ray, int c, int l, int nc, int d)
{
    int node = c * L + l;
    if (neighbour_layer(g, c, l, nc, d & 1) >= 0 || drop_layer(g, c, l, nc) < 0) return 0;
    float ax, az, bx, bz, h = g->height[node];
    cell_pos(g, c % g->w, c / g->w, &ax, &az);
    cell_pos(g, nc % g->w, nc / g->w, &bx, &bz);
    const float heights[2] = {KNEE_HEIGHT, CHEST_HEIGHT};
    for (int k = 0; k < 2; k++) {
        float a[3] = {ax, h + heights[k], az}, b[3] = {bx, h + heights[k], bz};
        if (ray(a, b, NULL, NULL)) return k + 1;
    }
    g->edges[node] |= (unsigned char)(1 << d);
    return 2;
}

// Probes the edge from cell towards direction d (0..3) for every walkable layer; sets bits on both nodes.
static int probe_edge(NavGrid *g, NavRayFn ray, int cell, int d)
{
    int cx = cell % g->w, cz = cell / g->w;
    int nx = cx + kDir[d][0], nz = cz + kDir[d][1];
    if (nx < 0 || nz < 0 || nx >= g->w || nz >= g->h) return 0;
    int nc = nz * g->w + nx, rays = 0;
    float ax, az, bx, bz;
    cell_pos(g, cx, cz, &ax, &az);
    cell_pos(g, nx, nz, &bx, &bz);
    for (int l = 0; l < L; l++) {
        int node = cell * L + l;
        if (!(g->flags[node] & NAV_WALKABLE)) continue;
        int nl = neighbour_layer(g, cell, l, nc, d & 1), stairs = 0;
        if (nl < 0 && !(d & 1)) {   // up a staircase? (straight directions)
            int sl = stair_layer(g, cell, l, nc);
            if (sl >= 0) {
                rays += 3;
                if (stairs_between(ray, ax, g->height[node], az, bx, g->height[nc * L + sl], bz)) { nl = sl; stairs = 1; }
            }
        }
        if (nl < 0) { rays += probe_drop(g, ray, cell, l, nc, d); continue; }
        float ha = g->height[node], hb = g->height[nc * L + nl];
        int blocked = 0, fence = 1;
        const float heights[3] = {KNEE_HEIGHT, CHEST_HEIGHT, 1.0f};
        for (int k = 0; k < 2; k++) {
            float a[3] = {ax, ha + heights[k], az}, b[3] = {bx, hb + heights[k], bz};
            rays++;
            if (ray(a, b, NULL, NULL)) {
                blocked = 1;
                if (k == 1 && !(g_classify && g_classify() == NAV_HIT_FENCE)) fence = 0;   // chest height: the fence itself
                if (k == 0 && !(g_classify && g_classify() == NAV_HIT_FENCE)) {   // knee: a low base is fine, a wall is not
                    float a1[3] = {ax, ha + heights[2], az}, b1[3] = {bx, hb + heights[2], bz};
                    rays++;
                    if (ray(a1, b1, NULL, NULL) && !(g_classify && g_classify() == NAV_HIT_FENCE)) fence = 0;
                }
            }
            if (blocked && !fence) break;
        }
        if (blocked && !(d & 1)) {   // a doorway beside the centre line? (straight directions)
            for (int k = 0; k < 4 && blocked; k++) {
                rays += 6;
                if (door_gap_ok(ray, ax, ha, az, bx, hb, bz, kDoorOffsets[k])) {
                    blocked = 0;
                    g->fences[node] |= (unsigned char)(1 << (d + 1));
                    g->fences[nc * L + nl] |= (unsigned char)(1 << (d + 5));
                }
            }
        }
        if (blocked) {
            if (fence && !(d & 1) && g_classify) {   // straight crossings only
                g->fences[node] |= (unsigned char)(1 << d);
                g->fences[nc * L + nl] |= (unsigned char)(1 << (d + 4));
            }
            if (stairs) continue;   // (no drop probe: a blocked staircase is just blocked)
            continue;
        }
        g->edges[node] |= (unsigned char)(1 << d);
        g->edges[nc * L + nl] |= (unsigned char)(1 << (d + 4));
    }
    if (!(d & 1))   // a staircase rising from the neighbour back into this cell (only ever probed from the top before)
        for (int l2 = 0; l2 < L; l2++) {
            int lower = nc * L + l2;
            if (!(g->flags[lower] & NAV_WALKABLE) || neighbour_layer(g, nc, l2, cell, 0) >= 0) continue;
            int sl = stair_layer(g, nc, l2, cell);
            if (sl < 0) continue;
            float hl = g->height[lower], hu = g->height[cell * L + sl];
            rays += 5;
            if (!stairs_between(ray, bx, hl, bz, ax, hu, az)) continue;
            int clear = 1;
            const float hs[2] = {KNEE_HEIGHT, CHEST_HEIGHT};
            for (int k = 0; k < 2 && clear; k++) {
                float a1[3] = {bx, hl + hs[k], bz}, b1[3] = {ax, hu + hs[k], az};
                if (ray(a1, b1, NULL, NULL)) clear = 0;
            }
            if (!clear) continue;
            g->edges[lower] |= (unsigned char)(1 << (d + 4));
            g->edges[cell * L + sl] |= (unsigned char)(1 << d);
        }
    for (int l = 0; l < L; l++)   // drops the other way (neighbour is the higher side)
        if (g->flags[nc * L + l] & NAV_WALKABLE) rays += probe_drop(g, ray, nc, l, cell, d + 4);
    return rays;
}

int nav_build_step(NavGrid *g, NavRayFn ray, int rayBudget, double maxMs)
{
    double t0 = now_ms();
    int n = g->w * g->h, used = 0, steps = 0;
    while (g->phase < 2 && used < rayBudget) {
        if (g->phase == 0) {
            used += probe_ground(g, ray, g->cursor++);
            if (g->cursor >= n) { g->phase = 1; g->cursor = 0; }
        } else {
            used += probe_edge(g, ray, g->cursor >> 2, g->cursor & 3);
            if (++g->cursor >= n * 4) { g->phase = 2; g->cursor = 0; }
        }
        if ((++steps & 63) == 0 && now_ms() - t0 > maxMs) break;
    }
    g->raysCast += used;
    g->buildMs += now_ms() - t0;
    return g->phase == 2;
}

typedef struct NavFileHeader { unsigned magic; char level[32]; float x0, z0, cell; int w, h, layers; } NavFileHeader;

static void blocks_restore_into(const NavGrid *g, unsigned char *edges);
int nav_save(const NavGrid *g, const char *path)
{
    // The links as built + repaired - WITHOUT the temporary stuck-bot blocks (nav_block_towards: 3 minutes each). Saving
    // g->edges as it stood baked every block active at that moment into the map's file for good: doorways bots got stuck
    // in became walls, and rooms / whole quarters were cut off (Seine Crossing, 2026-10-14).
    size_t n = (size_t)g->w * g->h * L;
    unsigned char *edges = HeapAlloc(GetProcessHeap(), 0, n);
    if (!edges) return 0;
    memcpy(edges, g->edges, n);
    blocks_restore_into(g, edges);
    FILE *f = fopen(path, "wb");
    if (!f) { HeapFree(GetProcessHeap(), 0, edges); return 0; }
    NavFileHeader hd = {NAV_FILE_MAGIC, {0}, g->x0, g->z0, g->cell, g->w, g->h, L};
    memcpy(hd.level, g->level, sizeof hd.level);
    int ok = fwrite(&hd, sizeof hd, 1, f) == 1 && fwrite(g->height, sizeof(float), n, f) == n &&
             fwrite(g->flags, 1, n, f) == n && fwrite(edges, 1, n, f) == n && fwrite(g->fences, 1, n, f) == n;
    fclose(f);
    HeapFree(GetProcessHeap(), 0, edges);
    return ok;
}

NavGrid *nav_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    NavFileHeader hd;
    NavGrid *g = NULL;
    if (fread(&hd, sizeof hd, 1, f) == 1 && (hd.magic == NAV_FILE_MAGIC || hd.magic == NAV_FILE_MAGIC4) && hd.layers == L && hd.w > 0 && hd.h > 0 &&
        hd.w * hd.h < 16000000) {
        g = new_grid();
        memcpy(g->level, hd.level, sizeof g->level);
        g->x0 = hd.x0; g->z0 = hd.z0; g->cell = hd.cell; g->w = hd.w; g->h = hd.h; g->phase = 2;
        alloc_arrays(g);
        size_t n = (size_t)g->w * g->h * L;
        if (fread(g->height, sizeof(float), n, f) != n || fread(g->flags, 1, n, f) != n || fread(g->edges, 1, n, f) != n ||
            (hd.magic == NAV_FILE_MAGIC && fread(g->fences, 1, n, f) != n)) {
            nav_free(g);
            g = NULL;
        }
    }
    fclose(f);
    return g;
}

// ---- A* over (cell, layer) nodes ----
typedef struct HeapItem { float f; int node; } HeapItem;

static void heap_push(HeapItem *h, int *n, float f, int node)
{
    int i = (*n)++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (h[p].f <= f) break;
        h[i] = h[p];
        i = p;
    }
    h[i].f = f;
    h[i].node = node;
}

static HeapItem heap_pop(HeapItem *h, int *n)
{
    HeapItem top = h[0], last = h[--(*n)];
    int i = 0;
    for (;;) {
        int c = 2 * i + 1;
        if (c >= *n) break;
        if (c + 1 < *n && h[c + 1].f < h[c].f) c++;
        if (last.f <= h[c].f) break;
        h[i] = h[c];
        i = c;
    }
    h[i] = last;
    return top;
}

// Node reached from `node` in direction d, or -1.
static int step_node(const NavGrid *g, int node, int d)
{
    if (!(g->edges[node] & (1 << d))) return -1;
    int c = node / L, l = node % L;
    int nc = (c / g->w + kDir[d][1]) * g->w + c % g->w + kDir[d][0];
    int stairOk = g_stairOk && g_stairOkGrid == g && g_stairOkW == g->w && g_stairOkH == g->h && (g_stairOk[node] & (1 << d));
    int nl = stairOk ? stair_layer(g, c, l, nc) : -1;   // a link known to be a staircase up: the step up, not the floor under it (Kharg,
    if (nl < 0) nl = neighbour_layer(g, c, l, nc, d & 1);   //  2026-10-23: a flight over walkable ground led to that ground - never up)
    // A cell can hold both a surface ~1 m up and one below: the next step of a staircase plus the floor under it (Seine),
    // or an escalator's handrail top plus its steps going down (Metro). The load-time stair re-check records which links
    // are real staircases: those go up, everything else is the drop. (Up first sent bots onto Metro's handrails - they
    // could ride the escalators up but never find the way back down.)
    if (nl < 0) {
        int stairUp = g_stairOk && g_stairOkGrid == g && g_stairOkW == g->w && g_stairOkH == g->h && (g_stairOk[node] & (1 << d));
        if (stairUp) nl = stair_layer(g, c, l, nc);
        if (nl < 0) nl = drop_layer(g, c, l, nc);    // one-way link off a ledge (or down a staircase / escalator)
        if (nl < 0) nl = stair_layer(g, c, l, nc);   // up a staircase (no drop there)
    }
    return nl < 0 ? -1 : nc * L + nl;
}
int nav_step_node(const NavGrid *g, int node, int d) { return step_node(g, node, d); }
static int nearest_node(const NavGrid *g, float x, float y, float z);
int nav_nearest_node(const NavGrid *g, float x, float y, float z) { return nearest_node(g, x, y, z); }


// Connected walkable node near (x, y, z): nearest cell within a few cells, layer closest to y (NAN = lowest).
static int nearest_node(const NavGrid *g, float x, float y, float z)
{
    int c = nav_cell_index(g, x, z);
    if (c < 0) return -1;
    int cx = c % g->w, cz = c / g->w;
    for (int r = 0; r <= 4; r++) {
        int best = -1;
        float bestD = 1e30f;
        for (int dz = -r; dz <= r; dz++)
            for (int dx = -r; dx <= r; dx++) {
                if (abs(dx) != r && abs(dz) != r) continue;   // ring only
                int x2 = cx + dx, z2 = cz + dz;
                if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h) continue;
                for (int l = 0; l < L; l++) {
                    int node = (z2 * g->w + x2) * L + l;
                    if (!(g->flags[node] & NAV_WALKABLE) || !g->edges[node]) continue;
                    if (g_pathMask && !g_pathMask[node]) continue;
                    float d = isnan(y) ? -g->height[node] : fabsf(g->height[node] - y);
                    if (!isnan(y) && d > 3.0f) continue;   // wrong level (e.g. an overpass above the bot)
                    if (d < bestD) { bestD = d; best = node; }
                }
            }
        if (best >= 0) return best;
    }
    return -1;
}

// True if walking straight from node a to cell of node b stays on connected nodes (for string pulling).
static int straight_walkable(const NavGrid *g, int a, int b)
{
    int bc = b / L;
    int x = (a / L) % g->w, z = (a / L) / g->w, x1 = bc % g->w, z1 = bc / g->w, node = a;
    int dx = abs(x1 - x), dz = abs(z1 - z), sx = x < x1 ? 1 : -1, sz = z < z1 ? 1 : -1, err = dx - dz;
    while (x != x1 || z != z1) {
        int e2 = 2 * err, mx = 0, mz = 0;
        if (e2 > -dz) { err -= dz; mx = sx; }
        if (e2 < dx) { err += dx; mz = sz; }
        int d;
        for (d = 0; d < 8; d++) if (kDir[d][0] == mx && kDir[d][1] == mz) break;
        node = step_node(g, node, d);
        if (node < 0 || (g_pathMask && !g_pathMask[node])) return 0;
        if (node_blocked(g, node)) return 0;   // (no short cut through a parked vehicle)
        if (g_pathMask && g_pathMask[node] == 5 && node != b) return 0;   // no short cut past a tree trunk
        if (!g_pathMask && TREE_DIST(g) && TREE_DIST(g)[node] <= TREE_NEAR_DM && node != b) return 0;
        x += mx;
        z += mz;
    }
    return node == b;
}

// A soldier is ~0.8 m wide: test the centre line and lines offset BODY_HALF_WIDTH to each side, at knee and chest height.
#define BODY_HALF_WIDTH 0.6f   // (a little more than a body: straightened legs keep off walls and corners)
int nav_segment_clear(NavRayFn ray, const float *a, const float *b)
{
    float dx = b[0] - a[0], dz = b[2] - a[2], len = sqrtf(dx * dx + dz * dz);
    if (len < 0.01f) return 1;
    float half = g_pathMask ? g_vehHalf : BODY_HALF_WIDTH;   // vehicles: a car's width (poles and trees between grid points)
    float sx = -dz / len * half, sz = dx / len * half;
    const float heights[3] = {KNEE_HEIGHT, CHEST_HEIGHT, 0.35f};   // (vehicles also low: gabion corners, kerbs)
    const float side[7] = {0, 1, -1, 0.33f, -0.33f, 0.67f, -0.67f};   // (vehicles: 7 lines ~0.5 m apart - trees, poles)
    for (int k = 0; k < (g_pathMask ? 3 : 2); k++)
        for (int s = 0; s < (g_pathMask ? 7 : 3); s++) {
            float p[3] = {a[0] + sx * side[s], a[1] + heights[k], a[2] + sz * side[s]};
            float q[3] = {b[0] + sx * side[s], b[1] + heights[k], b[2] + sz * side[s]};
            if (ray(p, q, NULL, NULL)) return 0;
        }
    return 1;
}

static void node_pos(const NavGrid *g, int node, float *out)
{
    cell_pos(g, (node / L) % g->w, (node / L) / g->w, &out[0], &out[2]);
    out[1] = g->height[node];
}

// Walkable ground near (x, y, z) (same cell rules as path planning). Returns 1 and the ground point in out.
int nav_snap(const NavGrid *g, float x, float y, float z, float *out)
{
    int node = nearest_node(g, x, y, z);
    if (node < 0) return 0;
    node_pos(g, node, out);
    return 1;
}

// Links blocked where a bot got stuck come back after NAV_BLOCK_MS: a block is a guess, and on Seine one on the only
// staircase up from the river made the whole lower quay a trap for the rest of the round.
#define NAV_BLOCK_MS   (3 * 60 * 1000.0)
#define MAX_NAV_BLOCKS 1024
static struct { const NavGrid *g; int w, h, node, nb, d; double until; } g_blocks[MAX_NAV_BLOCKS];
static int g_blockNext;
static void nav_blocks_forget(const NavGrid *g)
{
    for (int k = 0; k < MAX_NAV_BLOCKS; k++) if (g_blocks[k].g == g) g_blocks[k].g = NULL;
}
// A record still describes this grid: same grid, same size, nodes inside it (an old map's records once wrote past
// the end of a new grid allocated at the old address - 6 crashes on the AM1 test box, 1 s after "level ready").
static int block_ok(const NavGrid *g, int k)
{
    int n = g->w * g->h * L;
    return g_blocks[k].g == g && g_blocks[k].w == g->w && g_blocks[k].h == g->h && g_blocks[k].node >= 0 && g_blocks[k].node < n &&
           g_blocks[k].nb >= 0 && g_blocks[k].nb < n;
}
// The links the active temporary blocks took out, put back into a copy of the link array (for saving).
static void blocks_restore_into(const NavGrid *g, unsigned char *edges)
{
    for (int k = 0; k < MAX_NAV_BLOCKS; k++) {
        if (!g_blocks[k].g || g_blocks[k].g != g || !block_ok(g, k)) continue;
        edges[g_blocks[k].node] |= (unsigned char)(1 << g_blocks[k].d);
        edges[g_blocks[k].nb] |= (unsigned char)(1 << ((g_blocks[k].d + 4) & 7));
    }
}
int nav_unblock_expired(NavGrid *g, int all)
{
    double t = now_ms();
    int n = 0;
    for (int k = 0; k < MAX_NAV_BLOCKS; k++) {
        if (!g_blocks[k].g || g_blocks[k].g != g) continue;
        if (!block_ok(g, k)) { g_blocks[k].g = NULL; continue; }
        if (!all && t < g_blocks[k].until) continue;
        g->edges[g_blocks[k].node] |= (unsigned char)(1 << g_blocks[k].d);
        g->edges[g_blocks[k].nb] |= (unsigned char)(1 << ((g_blocks[k].d + 4) & 7));
        g_blocks[k].g = NULL;
        n++;
    }
    return n;
}
int nav_block_towards(NavGrid *g, float x, float y, float z, float tx, float tz)
{
    int node = nearest_node(g, x, y, z), removed = 0;
    if (node < 0) return 0;
    float dx = tx - x, dz = tz - z, len = sqrtf(dx * dx + dz * dz);
    if (len < 0.01f) return 0;
    for (int d = 0; d < 8; d++) {
        float ddx = (float)kDir[d][0], ddz = (float)kDir[d][1], dl = sqrtf(ddx * ddx + ddz * ddz);
        if ((ddx * dx + ddz * dz) / (dl * len) < 0.5f) continue;   // only directions within +-60 deg
        int nb = step_node(g, node, d);
        if (nb < 0) continue;
        if (!(g->edges[node] & (1 << d))) continue;
        g->edges[node] &= (unsigned char)~(1 << d);
        g->edges[nb] &= (unsigned char)~(1 << ((d + 4) & 7));
        int k = g_blockNext++ % MAX_NAV_BLOCKS;
        if (g_blocks[k].g == g && block_ok(g, k)) {   // (the oldest record is overwritten: put its link back first)
            g->edges[g_blocks[k].node] |= (unsigned char)(1 << g_blocks[k].d);
            g->edges[g_blocks[k].nb] |= (unsigned char)(1 << ((g_blocks[k].d + 4) & 7));
        }
        g_blocks[k].g = g; g_blocks[k].w = g->w; g_blocks[k].h = g->h; g_blocks[k].node = node; g_blocks[k].nb = nb; g_blocks[k].d = d; g_blocks[k].until = now_ms() + NAV_BLOCK_MS;
        removed++;
    }
    return removed;
}

// ---- Local repair: fine slope check of links near a spot where bots get stuck ----
#define FINE_STEP        0.5f     // ground sample spacing along a link
#define MAX_FINE_RISE    0.5f     // per FINE_STEP (45 deg): steeper than this a soldier cannot walk up (stairs and escalators stay under it)
static int link_too_steep(const NavGrid *g, NavRayFn ray, int node, int nb)
{
    float a[3], b[3];
    node_pos(g, node, a);
    node_pos(g, nb, b);
    if (b[1] - a[1] < 0.3f) return 0;   // level or downhill: fine
    float dx = b[0] - a[0], dz = b[2] - a[2], len = sqrtf(dx * dx + dz * dz);
    int steps = (int)(len / FINE_STEP + 0.5f);
    if (steps < 2) steps = 2;
    float prev = a[1], top = b[1] + 1.0f, bottom = a[1] - 1.0f;
    for (int k = 1; k <= steps; k++) {
        float t = (float)k / steps, from[3] = {a[0] + dx * t, top, a[2] + dz * t}, to[3] = {from[0], bottom, from[2]}, hit[3];
        if (!ray(from, to, hit, NULL)) continue;
        if (hit[1] - prev > MAX_FINE_RISE * (len / steps) / FINE_STEP) return 1;
        prev = hit[1];
    }
    return 0;
}

int nav_repair_around(NavGrid *g, NavRayFn ray, float x, float y, float z, float radius, float dirX, float dirZ)
{
    float dlen = sqrtf(dirX * dirX + dirZ * dirZ);
    int c = nav_cell_index(g, x, z), removed = 0;
    if (c < 0) return 0;
    int cx = c % g->w, cz = c / g->w, r = (int)(radius / g->cell + 0.5f);
    for (int zz = cz - r; zz <= cz + r; zz++)
        for (int xx = cx - r; xx <= cx + r; xx++) {
            if (xx < 0 || zz < 0 || xx >= g->w || zz >= g->h) continue;
            for (int l = 0; l < L; l++) {
                int node = (zz * g->w + xx) * L + l;
                if (!(g->flags[node] & NAV_WALKABLE) || fabsf(g->height[node] - y) > 4.0f) continue;
                for (int d = 0; d < 8; d++) {
                    if (dlen > 0.01f) {   // only links going the way the bot was trying to go (+-60 deg)
                        float ddx = (float)kDir[d][0], ddz = (float)kDir[d][1];
                        if ((ddx * dirX + ddz * dirZ) / (sqrtf(ddx * ddx + ddz * ddz) * dlen) < 0.5f) continue;
                    }
                    int nb = step_node(g, node, d);
                    if (nb < 0 || !link_too_steep(g, ray, node, nb)) continue;
                    g->edges[node] &= (unsigned char)~(1 << d);   // one way: walking down that bank is still fine
                    removed++;
                }
            }
        }
    return removed;
}

// ---- special links (ladders) ----
#define NAV_MAX_LINKS 256
typedef struct NavLink { const NavGrid *g; int from, to; float cost; float a[3], b[3]; } NavLink;
static NavLink g_links[NAV_MAX_LINKS];
static int g_linkCount;
void nav_clear_links(void) { g_linkCount = 0; }
static int nearest_node(const NavGrid *g, float x, float y, float z);
int nav_add_link(const NavGrid *g, const float *a, const float *b, float cost)
{
    if (!g || g_linkCount >= NAV_MAX_LINKS) return -1;
    int from = nearest_node(g, a[0], a[1], a[2]), to = nearest_node(g, b[0], b[1], b[2]);
    if (from < 0 || to < 0 || from == to) return -1;
    NavLink *k = &g_links[g_linkCount];
    k->g = g; k->from = from; k->to = to; k->cost = cost;
    memcpy(k->a, a, 12); memcpy(k->b, b, 12);
    return g_linkCount++;
}
void nav_disable_link(int id) { if (id >= 0 && id < g_linkCount) g_links[id].from = -1; }
int nav_path_link(int k) { return k >= 0 && k < NAV_MAX_OUT ? g_outLink[k] : -1; }
static int link_between(const NavGrid *g, int a, int b)
{
    for (int k = 0; k < g_linkCount; k++) if (g_links[k].g == g && g_links[k].from == a && g_links[k].to == b) return k;
    return -1;
}
const char *nav_last_error(void) { return g_lastError; }

int nav_reprobe_area(NavGrid *g, NavRayFn ray, float x0, float z0, float x1, float z1, float fromY)
{
    int c0 = nav_cell_index(g, x0, z0), c1 = nav_cell_index(g, x1, z1), added = 0;
    if (c0 < 0 || c1 < 0) return 0;
    int ax = c0 % g->w, az = c0 / g->w, bx = c1 % g->w, bz = c1 / g->w;
    if (ax > bx) { int t = ax; ax = bx; bx = t; }
    if (az > bz) { int t = az; az = bz; bz = t; }
    for (int cz = az; cz <= bz; cz++)
        for (int cx = ax; cx <= bx; cx++) {
            int cell = cz * g->w + cx;
            float x, z, hit[3];
            cell_pos(g, cx, cz, &x, &z);
            float a[3] = {x, fromY, z}, b[3] = {x, fromY - 30.0f, z};
            if (!ray(a, b, hit, NULL)) continue;
            int have = 0, freeL = -1;
            for (int l = 0; l < L; l++) {
                int node = cell * L + l;
                if (g->flags[node] & NAV_WALKABLE) { if (fabsf(g->height[node] - hit[1]) < 0.4f) have = 1; }
                else if (freeL < 0) freeL = l;
            }
            if (have) continue;
            if (freeL < 0) {   // every slot used by structure above (bridge girders, stairwells): give up the least connected
                int best = -1, bestEdges = 99;
                for (int l = 1; l < L; l++) {   // never the topmost surface
                    int ne = 0;
                    for (int d = 0; d < 8; d++) if (g->edges[cell * L + l] & (1 << d)) ne++;
                    if (ne < bestEdges || (ne == bestEdges && g->height[cell * L + l] < g->height[cell * L + best])) { bestEdges = ne; best = l; }
                }
                if (best < 0) continue;
                freeL = best;
            }
            int node = cell * L + freeL;
            g->height[node] = hit[1];
            g->flags[node] |= NAV_WALKABLE;
            g->edges[node] = 0;
            added++;
        }
    // re-link the area and its border (both directions for every pair)
    for (int cz = az - 1; cz <= bz + 1; cz++)
        for (int cx = ax - 1; cx <= bx + 1; cx++) {
            if (cx < 0 || cz < 0 || cx >= g->w || cz >= g->h) continue;
            for (int d = 0; d < 8; d++) probe_edge(g, ray, cz * g->w + cx, d);
        }
    return added;
}

unsigned char *nav_drive_mask(const NavGrid *g, int *count)
{
    int n = g->w * g->h * L, ok = 0;
    unsigned char *m = nav_alloc((size_t)n);
    if (!m) return NULL;
    for (int node = 0; node < n; node++) {
        if (!(g->flags[node] & NAV_WALKABLE) || (g->flags[node] & NAV_WATER) || g->edges[node] != 0xff) continue;
        // under cover (a floor / roof less than 6 m above - layers run top-down): a building, not a road
        int l = node % L, covered = 0;
        for (int k = 0; k < l && !covered; k++) {
            int up = node - l + k;
            if ((g->flags[up] & NAV_WALKABLE) && g->height[up] - g->height[node] < 6.0f) covered = 1;
        }
        if (covered) continue;
        int good = 1, nbs[8];
        for (int d = 0; d < 8 && good; d++) {
            int nb = step_node(g, node, d);
            nbs[d] = nb;
            if (nb < 0 || (g->flags[nb] & NAV_WATER) || fabsf(g->height[nb] - g->height[node]) > 0.7f * (d & 1 ? 1.41f : 1.0f)) good = 0;
        }
        if (!good) continue;
        // bumpiness: how far the node sits off the straight line through its opposite neighbours (a road - flat or
        // up a hill - is ~0; rough ground is not)
        float rough = 0, h = g->height[node];
        for (int d = 0; d < 4; d++) {
            float r = fabsf(2.0f * h - g->height[nbs[d]] - g->height[nbs[d + 4]]);
            if (r > rough) rough = r;
        }
        float slope = 0;   // steepest rise to a straight neighbour (m per cell)
        for (int d = 0; d < 8; d += 2) { float s = fabsf(g->height[nbs[d]] - h); if (s > slope) slope = s; }
        int cls = rough < 0.08f ? 1 : rough < 0.2f ? 2 : rough < 0.4f ? 3 : 4;   // cost classes: x0.5, x1, x1.6, x2.5
        if (slope > 0.25f && cls < 4) cls++;   // climbing (over 12%): roads mostly run flat - a smooth hillside is not a road
        if (slope > 0.45f && cls < 4) cls++;
        m[node] = (unsigned char)cls;
        ok++;
    }
    // next to the edge of the drivable area (a wall, a building, a drop): the dearest class - routes keep to the middle
    // of open ground (vehicles are 5 m long and turn wide) but can still squeeze through a gate when there is no other way
    unsigned char *e = nav_alloc((size_t)n);
    if (e) {
        memcpy(e, m, (size_t)n);
        for (int node = 0; node < n; node++) {
            if (!m[node]) continue;
            for (int d = 0; d < 8; d++) { int nb = step_node(g, node, d); if (nb < 0 || !m[nb]) { e[node] = 4; break; } }
        }
        for (int node = 0; node < n; node++) {   // the next ring in: dearer too (4 m from a wall where there is room)
            if (!m[node] || e[node] == 4) continue;
            for (int d = 0; d < 8; d++) { int nb = step_node(g, node, d); if (nb >= 0 && e[nb] == 4 && m[nb]) { if (e[node] < 3) e[node] = 3; break; } }
        }
        nav_release(m);
        m = e;
    }
    if (count) *count = ok;
    return m;
}

// Tank mask (2026-10-13): steep mountain maps (Alborz) leave the drive mask in islands - a looser one for tracked
// vehicles: walkable, dry, not under cover, neighbours within 1.4 m per cell (steeper than the drive mask's 0.7, no
// full set of walking links needed). Kept well off rocks / walls / other drops: nodes within ~6 m of the area's edge
// are the dearest class, the next ~4 m dear - routes run through the middle of open ground (the infantry route
// hugged the rocks and tanks stuck on their edges).
// Clear-of mask (2026-10-13, the Pilots' taxis): the nodes of src of class 1..maxClass with drivable ground (any class
// but a tree's, within 2 m of the height) all round them out to radius - a car's centre that far from anything that
// is not road (a wall, a parked truck, a drop). Taxis keep to the roads, 3 m clear of objects (the user).
unsigned char *nav_mask_clear_of(const NavGrid *g, const unsigned char *src, int maxClass, float radius, int *count)
{
    int n = g->w * g->h * L, ok = 0, r = (int)(radius / g->cell + 0.999f);
    unsigned char *m = nav_alloc((size_t)n);
    if (!m || !src) return m;
    for (int node = 0; node < n; node++) {
        int c = src[node];
        if (!c || c > maxClass) continue;
        int cell = node / L, cx = cell % g->w, cz = cell / g->w, good = 1;
        float h = g->height[node];
        for (int dz = -r; dz <= r && good; dz++)
            for (int dx = -r; dx <= r && good; dx++) {
                if ((dx * dx + dz * dz) * g->cell * g->cell > radius * radius) continue;
                int x2 = cx + dx, z2 = cz + dz, found = 0;
                if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h) { good = 0; break; }
                for (int l = 0; l < L && !found; l++) {
                    int nb = (z2 * g->w + x2) * L + l;
                    if (src[nb] && src[nb] != 5 && fabsf(g->height[nb] - h) < 2.0f) found = 1;
                }
                if (!found) good = 0;
            }
        if (good) { m[node] = (unsigned char)c; ok++; }
    }
    if (count) *count = ok;
    return m;
}

int g_lastStreetNodes;   // street nodes the last nav_mask_streets found
// Streets (the user's rule, 2026-10-13): a strip of ground that is the same all across a fixed width and at least
// twice as long as it is wide. "The same": one ground material (mats, per node - Wake Island's dirt roads are a 400c
// strip between 4013 verges, while its beaches are a wide 400c expanse) and no kerb (a height kink over 0.1 m across a
// node - checked off terrain only: dirt roads are bumpy). In one of 4 directions the strip is 2..8 cells (4-16 m)
// across and 2x that along. Street nodes away from the strip's edge class 1 (cheapest), at its edge 2; anything else
// in the mask 4 (dear: pavements, squares, beaches, fields).
void nav_mask_streets(const NavGrid *g, unsigned char *mask, const unsigned short *mats)
{
    int n = g->w * g->h * L, qh = 0, qt = 0, nst = 0;
    unsigned char *dist = nav_alloc((size_t)n), *kerb = nav_alloc((size_t)n);
    int *q = nav_alloc((size_t)n * sizeof(int));
    if (!dist || !q || !kerb) { nav_release(dist); nav_release(q); nav_release(kerb); return; }
    for (int node = 0; node < n; node++) {
        if (!mask[node]) continue;
        int nbs[8];
        for (int d = 0; d < 8; d++) nbs[d] = step_node(g, node, d);
        int terrain = mats && (mats[node] & 0xff00) == 0x4000;
        for (int d = 0; d < 4 && !terrain; d++)
            if (nbs[d] >= 0 && nbs[d + 4] >= 0 && fabsf(2.0f * g->height[node] - g->height[nbs[d]] - g->height[nbs[d + 4]]) > 0.1f) kerb[node] = 1;
    }
    // (mask value 7 on the way in: a prop beside it - an edge for the distances, never a street, the dearest class out)
    #define SAME(a, b) (mask[b] && mask[b] != 7 && !kerb[b] && (!mats || mats[b] == mats[a]))
    for (int node = 0; node < n; node++) {
        if (!mask[node]) continue;
        dist[node] = 255;
        int edge = kerb[node];
        for (int d = 0; d < 8 && !edge; d++) { int nb = step_node(g, node, d); if (nb < 0 || !SAME(node, nb)) edge = 1; }
        if (edge) { dist[node] = 0; q[qt++] = node; }
    }
    while (qh < qt) {
        int node = q[qh++];
        for (int d = 0; d < 8; d++) {
            int nb = step_node(g, node, d);
            if (nb < 0 || !mask[nb] || dist[nb] <= dist[node] + 1) continue;
            dist[nb] = dist[node] + 1;
            q[qt++] = nb;
        }
    }
    // distance (cells) from the mask's own edge - a wall, a bank, a parked truck: 3 m off objects where there is room, the
    // middle of a narrow gap where there is not (the user, 2026-10-13: a dirt road between two obstacles on Firestorm)
    unsigned char *de = nav_alloc((size_t)n);
    if (de) {
        qh = qt = 0;
        for (int node = 0; node < n; node++) {
            if (!mask[node]) continue;
            de[node] = 255;
            int edge = mask[node] == 7;
            for (int d = 0; d < 8 && !edge; d++) { int nb = step_node(g, node, d); if (nb < 0 || !mask[nb] || mask[nb] == 5) edge = 1; }
            if (edge) { de[node] = 0; q[qt++] = node; }
        }
        while (qh < qt) {
            int node = q[qh++];
            for (int d = 0; d < 8; d++) {
                int nb = step_node(g, node, d);
                if (nb < 0 || !mask[nb] || de[nb] <= de[node] + 1) continue;
                de[nb] = de[node] + 1;
                q[qt++] = nb;
            }
        }
    }
    for (int node = 0; node < n; node++) {
        if (!mask[node] || mask[node] == 5) continue;
        if (mask[node] == 7) { mask[node] = 6; continue; }
        int street = 0, l = node % L, roofed = 0;
        for (int k = 0; k < l && !roofed; k++) if ((g->flags[node - l + k] & NAV_WALKABLE) && g->height[node - l + k] - g->height[node] < 25.0f) roofed = 1;
        if (!kerb[node] && !roofed)   // (under a roof - a hangar floor - is no street)
            for (int d = 0; d < 4 && !street; d++) {
                int across = 1, along = 1;
                for (int sgn = 0; sgn < 2 && across <= 8; sgn++) {   // across: d+2 and d+6
                    int c = node, dd = (d + 2 + sgn * 4) & 7;
                    for (int k = 0; k < 9; k++) { int c2 = step_node(g, c, dd); if (c2 < 0 || !SAME(node, c2)) break; c = c2; across++; }
                }
                if (across < 2 || across > 8) continue;
                int need = 2 * across > 12 ? 2 * across : 12;   // and 24 m at least (mottled ground paint made short "streets")
                for (int sgn = 0; sgn < 2 && along < need; sgn++) {   // along: d and d+4
                    int c = node, dd = (d + sgn * 4) & 7;
                    for (int k = 0; k < need; k++) { int c2 = step_node(g, c, dd); if (c2 < 0 || !SAME(node, c2)) break; c = c2; along++; }
                }
                if (along >= need) street = 1;
            }
        int e = de ? de[node] : 2;
        mask[node] = street ? (e >= 2 && dist[node] >= 1 ? 1 : e >= 1 ? 2 : 3) : (e >= 1 ? 4 : 6);   // 6: off-street against an edge (dearest)
        nst += street;
    }
    #undef SAME
    g_lastStreetNodes = nst;
    nav_release(dist); nav_release(q); nav_release(kerb); nav_release(de);
}

unsigned char *nav_tank_mask(const NavGrid *g, int *count)
{
    int n = g->w * g->h * L, ok = 0;
    unsigned char *m = nav_alloc((size_t)n);
    if (!m) return NULL;
    for (int node = 0; node < n; node++) {
        if (!(g->flags[node] & NAV_WALKABLE) || (g->flags[node] & NAV_WATER)) continue;
        int l = node % L, covered = 0;
        for (int k = 0; k < l && !covered; k++) {
            int up = node - l + k;
            if ((g->flags[up] & NAV_WALKABLE) && g->height[up] - g->height[node] < 6.0f) covered = 1;
        }
        if (covered) continue;
        int good = 1;
        for (int d = 0; d < 8 && good; d++) {
            int nb = step_node(g, node, d);
            if (nb < 0 || (g->flags[nb] & NAV_WATER) || fabsf(g->height[nb] - g->height[node]) > 1.4f * g->cell / 2.0f * (d & 1 ? 1.41f : 1.0f)) good = 0;
        }
        if (!good) continue;
        m[node] = 2;
        ok++;
    }
    int ringHard = (int)(6.0f / g->cell + 0.99f), ringSoft = ringHard + (int)(4.0f / g->cell + 0.99f);
    unsigned char *dist = nav_alloc((size_t)n);   // rings from the edge (0 = not in the mask)
    if (dist) {
        for (int node = 0; node < n; node++) {
            if (!m[node]) continue;
            for (int d = 0; d < 8; d++) { int nb = step_node(g, node, d); if (nb < 0 || !m[nb]) { dist[node] = 1; break; } }
        }
        for (int r = 1; r < ringSoft; r++)
            for (int node = 0; node < n; node++) {
                if (dist[node] != r) continue;
                for (int d = 0; d < 8; d++) { int nb = step_node(g, node, d); if (nb >= 0 && m[nb] && !dist[nb]) dist[nb] = (unsigned char)(r + 1); }
            }
        for (int node = 0; node < n; node++) {
            if (!m[node] || !dist[node]) continue;
            m[node] = dist[node] <= ringHard ? 4 : 3;
        }
        nav_release(dist);
    }
    if (count) *count = ok;
    return m;
}

void nav_node_pos(const NavGrid *g, int node, float *out) { node_pos(g, node, out); }

static const NavGrid *g_stairGrid;
static int g_stairCursor, g_stairRemoved, g_stairRestored, g_railCursor, g_railsRemoved, g_stairsAdded;
int nav_stairs_added(void) { return g_stairsAdded; }
int nav_stairs_restored(void) { return g_stairRestored; }
int nav_rails_removed(void) { return g_railsRemoved; }
static int g_railPassOn = 0;
// Handrail / balustrade / wall tops: a cell's single ground sample can land on one and make it "floor" (Metro: bots walked
// onto escalator and staircase rails and bashed into walls). Real floor is there half a metre to each side; a rail
// falls away on at least two sides - such nodes stop being walkable.
#define RAIL_PROBE      0.5f
#define RAIL_FALL       0.6f
static int rail_top(const NavGrid *g, NavRayFn ray, int node, int *used)
{
    float p[3];
    node_pos(g, node, p);
    static const float off[4][2] = {{RAIL_PROBE, 0}, {-RAIL_PROBE, 0}, {0, RAIL_PROBE}, {0, -RAIL_PROBE}};
    int falls = 0;
    for (int k = 0; k < 4 && falls < 2; k++) {
        float a[3] = {p[0] + off[k][0], p[1] + 0.5f, p[2] + off[k][1]}, b[3] = {a[0], p[1] - 1.5f, a[2]}, hit[3];
        (*used)++;
        if (!ray(a, b, hit, NULL) || hit[1] < p[1] - RAIL_FALL) falls++;
    }
    return falls >= 2;
}
static const NavGrid *g_hullGrid;
static int g_hullCursor, g_hullCut, g_hullRestored;
int nav_hull_restored(void) { return g_hullRestored; }
int nav_fix_hull_step(NavGrid *g, NavRayFn ray, int rayBudget, int *cut)
{
    int n = g->w * g->h * L, used = 0;
    if (g_hullGrid != g) { g_hullGrid = g; g_hullCursor = 0; g_hullCut = 0; g_hullRestored = 0; }
    for (; g_hullCursor < n && used < rayBudget; g_hullCursor++) {
        int node = g_hullCursor, c = node / L;
        if (!(g->flags[node] & NAV_WATER) || !(g->flags[node] & NAV_WALKABLE)) continue;
        float h = g->height[node];
        int over = 0;   // (something above it in the same column: under a ship / pier / bridge)
        for (int l2 = 0; l2 < L && !over; l2++) {
            int o = c * L + l2;
            if (o != node && (g->flags[o] & NAV_WALKABLE) && g->height[o] > h + 2.0f) over = 1;
        }
        if (!over) continue;
        int l = node % L;
        for (int d = 0; d < 8; d++) {   // every water neighbour, linked or not: re-checked - cut through solid, linked again when clear (a cut made
            int nx2 = c % g->w + kDir[d][0], nz2 = c / g->w + kDir[d][1];   //  through a parked boat - Kharg's well, 2026-10-23 - comes back)
            if (nx2 < 0 || nz2 < 0 || nx2 >= g->w || nz2 >= g->h) continue;
            int nc = nz2 * g->w + nx2, nl = neighbour_layer(g, c, l, nc, d & 1);
            if (nl < 0) continue;
            int nb = nc * L + nl;
            if (!(g->flags[nb] & NAV_WATER) || !(g->flags[nb] & NAV_WALKABLE) || fabsf(g->height[nb] - h) > 1.0f) continue;
            float a[3], b[3];
            node_pos(g, node, a); node_pos(g, nb, b);
            a[1] = h + 0.5f; b[1] = g->height[nb] + 0.5f;
            used++;
            int back = (d + 4) & 7, linked = (g->edges[node] >> d) & 1;
            if (ray(a, b, NULL, NULL)) {
                if (linked) { g->edges[node] &= (unsigned char)~(1 << d); g->edges[nb] &= (unsigned char)~(1 << back); g_hullCut++; }
            } else if (!linked) { g->edges[node] |= (unsigned char)(1 << d); g->edges[nb] |= (unsigned char)(1 << back); g_hullRestored++; }
        }
    }
    if (cut) *cut = g_hullCut;
    return g_hullCursor >= n;
}

// A link forced both ways between the nodes nearest a and b (neighbouring cells, straight): a staircase the checks miss (BotMod_navlinks.txt).
// 1 done, 0 not neighbours / no nodes.
// Doorway links of a grid built before door_gap_ok: re-tested a slice at a time (rayBudget rays a call); a link whose centre line is
// blocked and no body-wide gap is found is removed both ways. Returns 1 when the whole grid is done; *cut counts removals.
// Remove-only re-test of the links round a stuck bot: every same-level link in the box whose centre line is blocked and (a straight one)
// has no body-wide gap beside it is cut, both ways. Never adds a link (the build's probe, used here first, re-created a false 2.6 m
// "staircase" over a wall stub the stair re-check had just removed - Noshahr, 2026-10-26). Returns the links cut.
int nav_cut_blocked_around(NavGrid *g, NavRayFn ray, float x0, float z0, float x1, float z1)
{
    int c0 = nav_cell_index(g, x0, z0), c1 = nav_cell_index(g, x1, z1), cut = 0;
    if (c0 < 0 || c1 < 0) return 0;
    int ax = c0 % g->w, az = c0 / g->w, bx = c1 % g->w, bz = c1 / g->w;
    if (ax > bx) { int t = ax; ax = bx; bx = t; }
    if (az > bz) { int t = az; az = bz; bz = t; }
    for (int cz = az; cz <= bz; cz++)
        for (int cx = ax; cx <= bx; cx++)
            for (int l = 0; l < L; l++) {
                int node = (cz * g->w + cx) * L + l;
                if (!(g->flags[node] & NAV_WALKABLE)) continue;
                for (int d = 0; d < 8; d++) {
                    if (!(g->edges[node] & (1 << d))) continue;
                    int nb = step_node(g, node, d);
                    if (nb < 0 || fabsf(g->height[nb] - g->height[node]) > MAX_STEP_UP) continue;   // (climbs / drops: the stair checks' business)
                    float pa[3], pb[3];
                    node_pos(g, node, pa); node_pos(g, nb, pb);
                    if (pair_clear(ray, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], 0.0f)) continue;
                    int ok = 0;
                    for (int k = 0; k < 4 && !ok && !(d & 1); k++) if (door_gap_ok(ray, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], kDoorOffsets[k])) ok = 1;
                    if (ok) continue;
                    g->edges[node] &= (unsigned char)~(1 << d);
                    g->edges[nb] &= (unsigned char)~(1 << ((d + 4) & 7));
                    if (!(d & 1)) { g->fences[node] &= (unsigned char)~(1 << (d + 1)); g->fences[nb] &= (unsigned char)~(1 << ((d + 5) & 7)); }
                    cut++;
                }
            }
    return cut;
}
int nav_fix_doors_step(NavGrid *g, NavRayFn ray, int rayBudget, int *cut)
{
    static const NavGrid *forG;
    static int cursor;
    if (forG != g) { forG = g; cursor = 0; }
    int n = g->w * g->h * L, rays = 0;
    for (; cursor < n && rays < rayBudget; cursor++) {
        int node = cursor;
        if (!(g->flags[node] & NAV_WALKABLE) || !g->fences[node]) continue;
        for (int d = 0; d < 8; d += 2) {
            if (!(g->fences[node] & (1 << (d + 1)))) continue;   // a doorway link this way...
            int blk = -1;                                        // ...open, or taken out for now by a stuck-bot block (it would come back
            for (int k = 0; k < MAX_NAV_BLOCKS && blk < 0; k++)  //  when the block ends / with every save: a phantom kept returning, Noshahr)
                if (g_blocks[k].g == g && block_ok(g, k) && ((g_blocks[k].node == node && g_blocks[k].d == d) || (g_blocks[k].nb == node && ((g_blocks[k].d + 4) & 7) == d))) blk = k;
            if (!(g->edges[node] & (1 << d)) && blk < 0) continue;
            int had = g->edges[node] & (1 << d);
            if (!had) g->edges[node] |= (unsigned char)(1 << d);   // (step_node needs the link to find the neighbour)
            int nb = step_node(g, node, d);
            if (!had) g->edges[node] &= (unsigned char)~(1 << d);
            if (nb < 0) continue;
            float pa[3], pb[3];
            node_pos(g, node, pa); node_pos(g, nb, pb);
            rays += 2;
            if (pair_clear(ray, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], 0.0f)) continue;   // (the centre line is clear: an ordinary link)
            int ok = 0;
            for (int k = 0; k < 4 && !ok; k++) { rays += 6; if (door_gap_ok(ray, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], kDoorOffsets[k])) ok = 1; }
            if (ok) continue;
            if (blk >= 0) g_blocks[blk].g = NULL;   // (its block too: nothing puts it back)
            int back = (d + 4) & 7;
            g->edges[node] &= (unsigned char)~(1 << d);
            g->fences[node] &= (unsigned char)~(1 << (d + 1));
            g->edges[nb] &= (unsigned char)~(1 << back);
            g->fences[nb] &= (unsigned char)~(1 << ((d + 5) & 7));
            (*cut)++;
        }
    }
    return cursor >= n;
}

int nav_force_link(NavGrid *g, const float *a, const float *b)
{
    int na = nearest_node(g, a[0], a[1], a[2]), nb = nearest_node(g, b[0], b[1], b[2]);
    if (na < 0 || nb < 0) return 0;
    int ca = na / L, cb = nb / L, dx = cb % g->w - ca % g->w, dz = cb / g->w - ca / g->w, d = -1;
    for (int k = 0; k < 8; k += 2) if (kDir[k][0] == dx && kDir[k][1] == dz) d = k;
    if (d < 0) return 0;
    int back = (d + 4) & 7;
    g->edges[na] |= (unsigned char)(1 << d);
    g->edges[nb] |= (unsigned char)(1 << back);
    if (g_stairOk && g_stairOkGrid == g) {   // (the higher one is reached as a staircase up from the lower)
        if (g->height[nb] > g->height[na]) g_stairOk[na] |= (unsigned char)(1 << d);
        else g_stairOk[nb] |= (unsigned char)(1 << back);
    }
    return 1;
}

int nav_fix_stairs_step(NavGrid *g, NavRayFn ray, int rayBudget, int *removed)
{
    int n = g->w * g->h * L, used = 0;
    if (g_stairGrid != g || g_stairOkGrid != g) {
        g_stairGrid = g; g_stairCursor = 0; g_stairRemoved = 0; g_stairRestored = 0; g_railCursor = 0; g_railsRemoved = 0; g_stairsAdded = 0;
        nav_release(g_stairOk);
        g_stairOk = nav_alloc((size_t)n);   // (zeroed: nothing known to be a staircase until checked)
        g_stairOkGrid = g; g_stairOkW = g->w; g_stairOkH = g->h;
    }
    if (g_stairCursor < 0 && g_railCursor < 0) { if (removed) *removed = g_stairRemoved; return 1; }
    for (; g_stairCursor >= 0 && g_stairCursor < n && used < rayBudget; g_stairCursor++) {
        int node = g_stairCursor, c = node / L, l = node % L;
        if (!(g->flags[node] & NAV_WALKABLE)) continue;
        for (int d = 0; d < 8; d += 2) {   // straight directions (staircase links are straight)
            if (!(g->edges[node] & (1 << d))) {
                int nx2 = c % g->w + kDir[d][0], nz2 = c / g->w + kDir[d][1];
                if (nx2 < 0 || nz2 < 0 || nx2 >= g->w || nz2 >= g->h) continue;
                int nc2 = nz2 * g->w + nx2;
                {   // a staircase up that the build never linked (it rises towards -x / -z: only ever seen from the top)
                    int sl2 = neighbour_layer(g, c, l, nc2, 0) < 0 ? stair_layer(g, c, l, nc2) : -1;
                    if (sl2 >= 0) {
                        float a2[3], b2[3];
                        node_pos(g, node, a2); node_pos(g, nc2 * L + sl2, b2);
                        used += 5;
                        int ok = stairs_between(ray, a2[0], a2[1], a2[2], b2[0], b2[1], b2[2]);
                        const float hs2[2] = {KNEE_HEIGHT, CHEST_HEIGHT};
                        for (int k = 0; k < 2 && ok; k++) {
                            float p1[3] = {a2[0], a2[1] + hs2[k], a2[2]}, p2[3] = {b2[0], b2[1] + hs2[k], b2[2]};
                            if (ray(p1, p2, NULL, NULL)) ok = 0;
                        }
                        if (ok) {
                            g->edges[node] |= (unsigned char)(1 << d);
                            g->edges[nc2 * L + sl2] |= (unsigned char)(1 << ((d + 4) & 7));
                            if (g_stairOk) g_stairOk[node] |= (unsigned char)(1 << d);
                            g_stairsAdded++;
                            continue;
                        }
                    }
                }
                // a drop this check used to delete (and save): a staircase-height surface next door failed the stair
                // test, so the link went - though it was the drop down (Metro's escalator tops). Back if the way is clear.
                if (neighbour_layer(g, c, l, nc2, 0) >= 0 || stair_layer(g, c, l, nc2) < 0 || drop_layer(g, c, l, nc2) < 0) continue;
                float ax, az, bx, bz, h = g->height[node];
                cell_pos(g, c % g->w, c / g->w, &ax, &az);
                cell_pos(g, nx2, nz2, &bx, &bz);
                const float hs[2] = {KNEE_HEIGHT, CHEST_HEIGHT};
                int clear = 1;
                for (int k = 0; k < 2 && clear; k++) {
                    float a1[3] = {ax, h + hs[k], az}, b1[3] = {bx, h + hs[k], bz};
                    used++;
                    if (ray(a1, b1, NULL, NULL)) clear = 0;
                }
                if (clear) { g->edges[node] |= (unsigned char)(1 << d); g_stairRestored++; }
                continue;
            }
            int nx = c % g->w + kDir[d][0], nz = c / g->w + kDir[d][1];
            if (nx < 0 || nz < 0 || nx >= g->w || nz >= g->h) continue;
            int nc = nz * g->w + nx;
            if (neighbour_layer(g, c, l, nc, 0) >= 0) continue;   // an ordinary step
            int sl = stair_layer(g, c, l, nc);
            if (sl < 0) continue;
            float a[3], b[3];
            node_pos(g, node, a); node_pos(g, nc * L + sl, b);
            used += 3;
            if (stairs_between(ray, a[0], a[1], a[2], b[0], b[1], b[2])) { if (g_stairOk) g_stairOk[node] |= (unsigned char)(1 << d); }
            else if (drop_layer(g, c, l, nc) < 0) { g->edges[node] &= (unsigned char)~(1 << d); g_stairRemoved++; }   // (else: the drop)
        }
    }
    if (g_stairCursor >= n) g_stairCursor = -1;
    if (g_stairCursor >= 0) return 0;
    // then the rail pass (off: it also took floor edges and corners - 25,000 of Metro's nodes cut off)
    if (!g_railPassOn) g_railCursor = -1;
    for (; g_railCursor >= 0 && g_railCursor < n && used < rayBudget; g_railCursor++) {
        int node = g_railCursor;
        if (!(g->flags[node] & NAV_WALKABLE) || (g->flags[node] & NAV_WATER) || !g->edges[node]) continue;
        if (rail_top(g, ray, node, &used)) { g->flags[node] &= (unsigned char)~NAV_WALKABLE; g->edges[node] = 0; g_railsRemoved++; }
    }
    if (g_railCursor >= n) g_railCursor = -1;
    if (g_railCursor >= 0) return 0;
    if (removed) *removed = g_stairRemoved;
    return 1;
}

unsigned char *nav_boat_mask(const NavGrid *g, int *count)
{
    int n = g->w * g->h * L, ok = 0;
    unsigned char *m = nav_alloc((size_t)n);
    if (!m) return NULL;
    for (int node = 0; node < n; node++) {
        if (!(g->flags[node] & NAV_WATER) || g->edges[node] != 0xff) continue;
        int open = 1, good = 1;
        for (int d = 0; d < 8 && good; d++) {
            int nb = step_node(g, node, d);
            if (nb < 0) good = 0;
            else if (!(g->flags[nb] & NAV_WATER)) open = 0;
        }
        if (!good) continue;
        m[node] = open ? 1 : 4;   // right against land: the shallows - dearest
        ok++;
    }
    // distance from land in cells (breadth-first from the shore): within 3 cells (6 m) the dearest class, within 10
    // (20 m) the middle one - boats keep 20 m off land and obstacles and only come in to land
    int *dist = nav_alloc((size_t)n * sizeof(int)), *queue = nav_alloc((size_t)n * sizeof(int));
    if (dist && queue) {
        int head = 0, tail = 0;
        for (int node = 0; node < n; node++) {
            dist[node] = m[node] == 4 ? 0 : -1;
            if (m[node] == 4) queue[tail++] = node;
        }
        while (head < tail) {
            int u = queue[head++];
            if (dist[u] >= 10) continue;
            for (int d = 0; d < 8; d++) {
                int v = step_node(g, u, d);
                if (v < 0 || !m[v] || dist[v] >= 0) continue;
                dist[v] = dist[u] + 1;
                queue[tail++] = v;
            }
        }
        for (int node = 0; node < n; node++)
            if (m[node] && dist[node] >= 0) m[node] = dist[node] <= 3 ? 4 : 3;   // (beyond 10 cells: open water, class 1)
    }
    nav_release(dist); nav_release(queue);
    if (count) *count = ok;
    return m;
}

int nav_mask_nearest_in(const NavGrid *g, const unsigned char *mask, const int *labels, int label, float x, float z, float maxDist, float *out)
{
    int c = nav_cell_index(g, x, z);
    if (c < 0 || !labels || !label) return 0;
    int cx = c % g->w, cz = c / g->w, rmax = (int)(maxDist / g->cell);
    for (int r = 0; r <= rmax; r++) {
        int best = -1;
        float bestD = 1e30f;
        for (int dz = -r; dz <= r; dz++)
            for (int dx = -r; dx <= r; dx++) {
                if (abs(dx) != r && abs(dz) != r) continue;
                int x2 = cx + dx, z2 = cz + dz;
                if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h) continue;
                for (int l = 0; l < L; l++) {
                    int node = (z2 * g->w + x2) * L + l;
                    if (!mask[node] || labels[node] != label) continue;
                    float d = (float)(dx * dx + dz * dz);
                    if (d < bestD) { bestD = d; best = node; }
                }
            }
        if (best >= 0) { node_pos(g, best, out); return 1; }
    }
    return 0;
}

int nav_mask_nearest(const NavGrid *g, const unsigned char *mask, float x, float z, float maxDist, float *out)
{
    int c = nav_cell_index(g, x, z);
    if (c < 0) return 0;
    int cx = c % g->w, cz = c / g->w, rmax = (int)(maxDist / g->cell);
    for (int r = 0; r <= rmax; r++) {
        int best = -1;
        float bestD = 1e30f;
        for (int dz = -r; dz <= r; dz++)
            for (int dx = -r; dx <= r; dx++) {
                if (abs(dx) != r && abs(dz) != r) continue;   // ring only
                int x2 = cx + dx, z2 = cz + dz;
                if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h) continue;
                for (int l = 0; l < L; l++) {
                    int node = (z2 * g->w + x2) * L + l;
                    if (!mask[node]) continue;
                    float d = (float)(dx * dx + dz * dz);
                    if (d < bestD) { bestD = d; best = node; }
                }
            }
        if (best >= 0) { node_pos(g, best, out); return 1; }
    }
    return 0;
}

int nav_landings(const NavGrid *g, const unsigned char *mask, float sx, float sz, float gx, float gz, float walkW,
                 float *water, float *land, int max)
{
    int n = g->w * g->h * L, cnt = 0;
    float score[16];
    if (max > 16) max = 16;
    for (int node = 0; node < n; node++) {
        if (mask[node] != 3) continue;   // beach: water with land next to it
        float p[3];
        node_pos(g, node, p);
        float s = sqrtf((p[0] - sx) * (p[0] - sx) + (p[2] - sz) * (p[2] - sz)) + walkW * sqrtf((p[0] - gx) * (p[0] - gx) + (p[2] - gz) * (p[2] - gz));
        if (cnt == max && s >= score[cnt - 1]) continue;
        int dry = -1;
        for (int d = 0; d < 8 && dry < 0; d++) {
            int nb = step_node(g, node, d);
            if (nb >= 0 && !(g->flags[nb] & NAV_WATER)) dry = nb;
        }
        if (dry < 0) continue;
        int k = cnt < max ? cnt++ : max - 1;   // insert, sorted
        while (k > 0 && score[k - 1] > s) { score[k] = score[k - 1]; memcpy(&water[k * 3], &water[(k - 1) * 3], 12); memcpy(&land[k * 3], &land[(k - 1) * 3], 12); k--; }
        score[k] = s;
        memcpy(&water[k * 3], p, 12);
        node_pos(g, dry, &land[k * 3]);
    }
    return cnt;
}

int nav_land_around(const NavGrid *g, const unsigned char *mask, float x, float z, float radius, float *mx, float *mz, float *nearest)
{
    int c = nav_cell_index(g, x, z), cnt = 0;
    *nearest = 1e30f;
    if (c < 0) return 0;
    int cx = c % g->w, cz = c / g->w, r = (int)(radius / g->cell);
    float sx = 0, sz = 0;
    for (int dz = -r; dz <= r; dz++)
        for (int dx = -r; dx <= r; dx++) {
            int x2 = cx + dx, z2 = cz + dz;
            if (dx * dx + dz * dz > r * r) continue;
            int land = 1;
            if (x2 >= 0 && z2 >= 0 && x2 < g->w && z2 < g->h)
                for (int l = 0; l < L && land; l++) if (mask[(z2 * g->w + x2) * L + l]) land = 0;
            if (!land) continue;
            float px, pz;
            cell_pos(g, x2, z2, &px, &pz);
            float d = sqrtf((px - x) * (px - x) + (pz - z) * (pz - z));
            if (d < *nearest) *nearest = d;
            sx += px; sz += pz; cnt++;
        }
    if (cnt) { *mx = sx / cnt; *mz = sz / cnt; }
    return cnt;
}

void nav_mask_strip(const NavGrid *g, unsigned char *mask, unsigned char cls, const float *keep, int nkeep, float keepR)
{
    int n = g->w * g->h * L;
    for (int node = 0; node < n; node++) {
        if (mask[node] != cls) continue;
        float p[3];
        node_pos(g, node, p);
        int kept = 0;
        for (int k = 0; k < nkeep && !kept; k++) {
            float dx = p[0] - keep[k * 2], dz = p[2] - keep[k * 2 + 1];
            if (dx * dx + dz * dz < keepR * keepR) kept = 1;
        }
        if (!kept) mask[node] = 0;
    }
}

int nav_open_water_near(const NavGrid *g, const unsigned char *mask, float x, float z, float maxDist, float *out)
{
    int c = nav_cell_index(g, x, z);
    if (c < 0) return 0;
    int cx = c % g->w, cz = c / g->w, rmax = (int)(maxDist / g->cell);
    for (int r = 0; r <= rmax; r++)
        for (int dz = -r; dz <= r; dz++)
            for (int dx = -r; dx <= r; dx++) {
                if (abs(dx) != r && abs(dz) != r) continue;
                int x2 = cx + dx, z2 = cz + dz;
                if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h) continue;
                for (int l = 0; l < L; l++) { int node = (z2 * g->w + x2) * L + l; if (mask[node] == 1) { node_pos(g, node, out); return 1; } }
            }
    return 0;
}

int *nav_land_labels(const NavGrid *g)
{
    int n = g->w * g->h * L, next = 0;
    int *lab = nav_alloc((size_t)n * sizeof(int)), *queue = nav_alloc((size_t)n * sizeof(int));
    if (!lab || !queue) { nav_release(lab); nav_release(queue); return NULL; }
    for (int s = 0; s < n; s++) {
        if (lab[s] || !(g->flags[s] & NAV_WALKABLE) || (g->flags[s] & NAV_WATER) || !g->edges[s]) continue;
        int head = 0, tail = 0;
        lab[s] = ++next;
        queue[tail++] = s;
        while (head < tail) {
            int u = queue[head++];
            for (int d = 0; d < 8; d++) {
                int v = step_node(g, u, d);
                if (v < 0 || lab[v] || (g->flags[v] & NAV_WATER)) continue;
                lab[v] = next;
                queue[tail++] = v;
            }
        }
    }
    nav_release(queue);
    return lab;
}

int *nav_mask_labels(const NavGrid *g, const unsigned char *mask)
{
    int n = g->w * g->h * L, next = 0;
    int *lab = nav_alloc((size_t)n * sizeof(int)), *queue = nav_alloc((size_t)n * sizeof(int));
    if (!lab || !queue || !mask) { nav_release(lab); nav_release(queue); return NULL; }
    for (int s = 0; s < n; s++) {
        if (lab[s] || !mask[s]) continue;
        int head = 0, tail = 0;
        lab[s] = ++next;
        queue[tail++] = s;
        while (head < tail) {
            int u = queue[head++];
            for (int d = 0; d < 8; d++) {
                int v = step_node(g, u, d);
                if (v < 0 || lab[v] || !mask[v]) continue;
                lab[v] = next;
                queue[tail++] = v;
            }
        }
    }
    nav_release(queue);
    return lab;
}

int nav_label_at(const NavGrid *g, const int *labels, float x, float y, float z)
{
    int c = nav_cell_index(g, x, z);
    if (c < 0) return 0;
    int cx = c % g->w, cz = c / g->w, best = 0;
    float bestD = 1e30f;
    for (int r = 0; r <= 6 && !best; r++)
        for (int dz = -r; dz <= r; dz++)
            for (int dx = -r; dx <= r; dx++) {
                int x2 = cx + dx, z2 = cz + dz;
                if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h) continue;
                for (int l = 0; l < L; l++) {
                    int node = (z2 * g->w + x2) * L + l;
                    if (!labels[node]) continue;
                    float d = fabsf(g->height[node] - y);
                    if (d < bestD) { bestD = d; best = labels[node]; }
                }
            }
    return best;
}

int nav_landing_on(const NavGrid *g, const unsigned char *mask, const int *labels, int label, float sx, float sz, float gx, float gz,
                   float walkW, float minStart, float *water)
{
    int n = g->w * g->h * L, found = 0;
    float best = 1e30f;
    for (int node = 0; node < n; node++) {
        if (!mask[node]) continue;   // (a beach: next to dry land of the landmass - checked below)
        float p[3];
        node_pos(g, node, p);
        float ds = sqrtf((p[0] - sx) * (p[0] - sx) + (p[2] - sz) * (p[2] - sz));
        if (ds < minStart) continue;
        float s = ds + walkW * sqrtf((p[0] - gx) * (p[0] - gx) + (p[2] - gz) * (p[2] - gz));
        if (s >= best) continue;
        int ok = 0;
        for (int d = 0; d < 8 && !ok; d++) { int nb = step_node(g, node, d); if (nb >= 0 && labels[nb] == label) ok = 1; }
        if (!ok) continue;
        best = s; memcpy(water, p, 12); found = 1;
    }
    return found;
}

int nav_mask_block(const NavGrid *g, unsigned char *mask, float x, float z, float radius)
{
    int r = (int)(radius / g->cell) + 1, c = nav_cell_index(g, x, z), n = 0;
    if (c < 0) return 0;
    int cx = c % g->w, cz = c / g->w;
    for (int dz = -r; dz <= r; dz++)
        for (int dx = -r; dx <= r; dx++) {
            int x2 = cx + dx, z2 = cz + dz;
            if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h || (dx * dx + dz * dz) * g->cell * g->cell > radius * radius) continue;
            for (int l = 0; l < L; l++) { int node = (z2 * g->w + x2) * L + l; if (mask[node]) { mask[node] = 0; n++; } }
        }
    return n;
}

int nav_mask_restore(const NavGrid *g, unsigned char *mask, const unsigned char *base, float x, float z, float radius)
{
    int r = (int)(radius / g->cell) + 1, c = nav_cell_index(g, x, z), n = 0;
    if (c < 0 || !base) return 0;
    int cx = c % g->w, cz = c / g->w;
    for (int dz = -r; dz <= r; dz++)
        for (int dx = -r; dx <= r; dx++) {
            int x2 = cx + dx, z2 = cz + dz;
            if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h || (dx * dx + dz * dz) * g->cell * g->cell > radius * radius) continue;
            for (int l = 0; l < L; l++) { int node = (z2 * g->w + x2) * L + l; if (!mask[node] && base[node]) { mask[node] = base[node]; n++; } }
        }
    return n;
}

void nav_debug_edge(const NavGrid *g, NavRayFn ray, float x, float y, float z, int d, char *out, int outSize)
{
    int node = nearest_node(g, x, y, z), k = 0;
    if (node < 0) { snprintf(out, outSize, "no node"); return; }
    int c = node / L, l = node % L, cx = c % g->w, cz = c / g->w, nx = cx + kDir[d][0], nz = cz + kDir[d][1];
    if (nx < 0 || nz < 0 || nx >= g->w || nz >= g->h) { snprintf(out, outSize, "off grid"); return; }
    int nc = nz * g->w + nx;
    float ax, az, bx, bz, ha = g->height[node];
    cell_pos(g, cx, cz, &ax, &az);
    cell_pos(g, nx, nz, &bx, &bz);
    int nl = neighbour_layer(g, c, l, nc, d & 1), sl = stair_layer(g, c, l, nc), dl = drop_layer(g, c, l, nc);
    k += snprintf(out + k, outSize - k, "node (%.1f,%.2f,%.1f) edges %02x bit%d=%d | next cell layers:", ax, ha, az, g->edges[node], d,
                  (g->edges[node] >> d) & 1);
    for (int q = 0; q < L; q++) if (g->flags[nc * L + q] & NAV_WALKABLE) k += snprintf(out + k, outSize - k, " %.2f", g->height[nc * L + q]);
    k += snprintf(out + k, outSize - k, " | step %d stair %d drop %d", nl, sl, dl);
    int tl = nl >= 0 ? nl : sl;
    if (tl >= 0) {
        float hb = g->height[nc * L + tl];
        k += snprintf(out + k, outSize - k, " | samples:");
        for (int s = 1; s <= 3; s++) {
            float t = s * 0.25f, sx = ax + (bx - ax) * t, sz = az + (bz - az) * t, p[3], n[3];
            float a[3] = {sx, hb + 1.0f, sz}, b[3] = {sx, ha - 1.0f, sz};
            if (ray(a, b, p, n)) k += snprintf(out + k, outSize - k, " %.2f(n%.2f)", p[1], n[1]); else k += snprintf(out + k, outSize - k, " miss");
        }
        k += snprintf(out + k, outSize - k, " stairs_between=%d", stairs_between(ray, ax, ha, az, bx, hb, bz));
        const float hs[2] = {KNEE_HEIGHT, CHEST_HEIGHT};
        for (int q = 0; q < 2; q++) {
            float a[3] = {ax, ha + hs[q], az}, b[3] = {bx, hb + hs[q], bz}, p[3];
            int hit = ray(a, b, p, NULL);
            k += snprintf(out + k, outSize - k, " %s %s", q ? "chest" : "knee", hit ? "BLOCKED" : "clear");
            if (hit) k += snprintf(out + k, outSize - k, "@(%.1f,%.2f,%.1f)", p[0], p[1], p[2]);
        }
    }
}

int nav_floors_at(const NavGrid *g, float x, float y, float z, float *heights, int max)
{
    int c = nav_cell_index(g, x, z), n = 0;
    if (c < 0) return 0;
    for (int l = 0; l < L && n < max; l++) {
        int node = c * L + l;
        if (!(g->flags[node] & NAV_WALKABLE) || !g->edges[node]) continue;
        float h = g->height[node];
        int k = n++;   // insert, nearest to y first
        while (k > 0 && fabsf(heights[k - 1] - y) > fabsf(h - y)) { heights[k] = heights[k - 1]; k--; }
        heights[k] = h;
    }
    return n;
}

int nav_is_water(const NavGrid *g, float x, float y, float z)
{
    int c = nav_cell_index(g, x, z);
    if (c < 0) return 0;
    for (int l = 0; l < L; l++) {
        int node = c * L + l;
        if ((g->flags[node] & NAV_WATER) && fabsf(g->height[node] - y) < 2.0f) return 1;
    }
    return 0;
}

int nav_reach(const NavGrid *g, float x, float y, float z, float *nodePos)
{
    int start = nearest_node(g, x, y, z);
    if (start < 0) return 0;
    node_pos(g, start, nodePos);
    int n = g->w * g->h * L, count = 0, head = 0, tail = 0;
    unsigned char *seen = nav_alloc(n);
    int *queue = nav_alloc((size_t)n * sizeof(int));
    seen[start] = 1; queue[tail++] = start;
    while (head < tail) {
        int u = queue[head++];
        count++;
        for (int d = 0; d < 8; d++) {
            int v = step_node(g, u, d);
            if (v >= 0 && !seen[v]) { seen[v] = 1; queue[tail++] = v; }
        }
    }
    nav_release(seen); nav_release(queue);
    return count;
}

// String pulling over a node chain (from the bot's actual position: the farthest node reachable in a straight line each time).
// The game thread only - its rays are the engine's.
int nav_smooth_path(const NavGrid *g, NavRayFn ray, float sx, float sy, float sz, const int *nodes, int len, float *out, int maxPts)
{
    int count = 0;
    if (len <= 0) return 0;
    {
        // String pulling from the bot's actual position: take the farthest node reachable in a straight line.
        float from[3] = {sx, isnan(sy) ? g->height[nodes[0]] : sy, sz};
        int anchor = -1;
        while (anchor < len - 1 && count < maxPts) {
            int next = anchor + 1, limit = len - 1, link = -1;
            if (anchor >= 0) link = link_between(g, nodes[anchor], nodes[anchor + 1]);
            if (link < 0)   // the next link start is as far as a straight run may go
                for (int j = anchor + 1; j < len - 1; j++) if (link_between(g, nodes[j], nodes[j + 1]) >= 0) { limit = j; break; }
            if (link < 0)
                for (int j = limit; j > next; j--) {
                    if (j - anchor > (g_pathMask && !g_longLegs ? 8 : g_longLegs ? 250 : 64)) continue;   // cars: short legs on the road; boats: as long as the water allows
                    if (anchor >= 0 && !straight_walkable(g, nodes[anchor], nodes[j])) continue;
                    float to[3];
                    node_pos(g, nodes[j], to);
                    if (ray && !nav_segment_clear(ray, from, to)) continue;
                    next = j;
                    break;
                }
            if (link < 0 && next == anchor + 1 && anchor >= 0 && count < maxPts - 1) {   // through a doorway: via its gap
                int a = nodes[anchor], b = nodes[next], ca = a / L, cb = b / L;
                int ddx = cb % g->w - ca % g->w, ddz = cb / g->w - ca / g->w, dd = -1;
                for (int k = 0; k < 8; k += 2) if (kDir[k][0] == ddx && kDir[k][1] == ddz) dd = k;
                if (dd >= 0 && (g->fences[a] & (1 << (dd + 1)))) {
                    float pa[3], pb[3], off = 0;
                    node_pos(g, a, pa); node_pos(g, b, pb);
                    for (int k = 0; k < 4 && ray; k++)
                        if (door_gap_ok(ray, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], kDoorOffsets[k])) { off = kDoorOffsets[k]; break; }
                    float dx = pb[0] - pa[0], dz = pb[2] - pa[2], ln = sqrtf(dx * dx + dz * dz);
                    float *o = &out[count * 3];
                    o[0] = (pa[0] + pb[0]) * 0.5f - dz / ln * off;
                    o[1] = pa[1] > pb[1] ? pa[1] : pb[1];
                    o[2] = (pa[2] + pb[2]) * 0.5f + dx / ln * off;
                    if (count < NAV_MAX_OUT) g_outLink[count] = -1;
                    memcpy(from, o, sizeof from);
                    count++;
                }
            }
            int startsLink = next < len - 1 && link_between(g, nodes[next], nodes[next + 1]) >= 0;
            if (link >= 0) memcpy(&out[count * 3], g_links[link].b, 12);   // over the ladder: its exit spot
            else if (startsLink) memcpy(&out[count * 3], g_links[link_between(g, nodes[next], nodes[next + 1])].a, 12);   // its foot
            else node_pos(g, nodes[next], &out[count * 3]);
            if (count < NAV_MAX_OUT) g_outLink[count] = (short)link;
            memcpy(from, &out[count * 3], sizeof from);
            count++;
            anchor = next;
        }
    }
    return count;
}
static int nav_find_path_once(const NavGrid *g, NavRayFn ray, float sx, float sy, float sz, float tx, float ty, float tz, float *out, int maxPts);
int nav_find_path(const NavGrid *g, NavRayFn ray, float sx, float sy, float sz, float tx, float ty, float tz, float *out, int maxPts)
{
    int n = nav_find_path_once(g, ray, sx, sy, sz, tx, ty, tz, out, maxPts);
    if (!n && g_blkN && !strstr(g_lastError, "exhausted")) {   // boxed in by vehicles: the route as if they were not there (not after a search
        // that used its whole budget - that one is too far / unreachable either way, and running it twice made 500 ms plans, 2026-10-23)
        g_blkOff = 1;
        n = nav_find_path_once(g, ray, sx, sy, sz, tx, ty, tz, out, maxPts);
        g_blkOff = 0;
    }
    return n;
}
// The A* alone (the route worker): the node chain from start to goal into nodes[], its length (0: none - nav_last_error says why).
// No rays: safe off the game thread, given a grid nobody frees meanwhile and this thread's own mask / blockers.
int nav_find_nodes(const NavGrid *g, float sx, float sy, float sz, float tx, float ty, float tz, int *nodes, int maxNodes)
{
    NavCtx *X = nav_ctx();
    float dummy[3];
    X->nodesOut = nodes; X->nodesMax = maxNodes;
    int n = nav_find_path(g, NULL, sx, sy, sz, tx, ty, tz, dummy, 1);
    X->nodesOut = NULL;
    return n;
}
static int nav_find_path_once(const NavGrid *g, NavRayFn ray, float sx, float sy, float sz, float tx, float ty, float tz, float *out, int maxPts)
{
    int start = nearest_node(g, sx, sy, sz), goal = nearest_node(g, tx, ty, tz);
    if (goal < 0 && !isnan(ty)) goal = nearest_node(g, tx, NAN, tz);
    if (start < 0 || goal < 0) {
        snprintf(g_lastError, sizeof g_lastError, "no %s node", start < 0 ? "start" : "goal");
        return 0;
    }
    // Search buffers are allocated once per grid size and reused; a per-search stamp marks which entries are valid,
    // so nothing has to be cleared (a plan used to allocate and clear ~20 MB).
    // Weighted A*: leaning on the heuristic expands far fewer nodes for near-shortest routes (CPU on small LAN boxes).
    #define ASTAR_HEURISTIC_WEIGHT 1.5f
    #define ASTAR_MAX_EXPANDED     150000
    int n = g->w * g->h * L;
    NavCtx *X = nav_ctx();   // (this thread's own buffers)
    if (X->bufN != n) {
        nav_release(X->gs); nav_release(X->from); nav_release(X->stamp); nav_release(X->heap);
        X->gs = nav_alloc(n * sizeof(float));
        X->from = nav_alloc(n * sizeof(int));
        X->stamp = nav_alloc(n * sizeof(unsigned));
        X->heapCap = n + 1024;
        X->heap = nav_alloc(X->heapCap * sizeof(HeapItem));
        X->bufN = n;
        X->curStamp = 0;
        if (!X->gs || !X->from || !X->stamp || !X->heap) {   // (out of address space - a 32-bit server near 3 GB: the search wrote through a NULL
            nav_release(X->gs); nav_release(X->from); nav_release(X->stamp); nav_release(X->heap);   //  open list and server 3 went down, 2026-10-27)
            X->gs = NULL; X->from = NULL; X->stamp = NULL; X->heap = NULL; X->bufN = 0;
            snprintf(g_lastError, sizeof g_lastError, "out of memory for the search buffers");
            return 0;
        }
    }
    float *gs = X->gs;
    int *from = X->from, heapCap = X->heapCap;
    unsigned *stamp = X->stamp;
    HeapItem *heap = X->heap;
    const unsigned char *pathMask = X->pathMask;   // (the hot loop reads its own copy)
    X->curStamp += 2;   // stamp = curStamp: open (gs/from valid); curStamp + 1: closed
    unsigned curStamp = X->curStamp;
    int hn = 0, found = 0, count = 0, closedCount = 0;
    #define SEEN(x)   (stamp[x] >= curStamp)
    #define CLOSED(x) (stamp[x] == curStamp + 1)
    int gx = (goal / L) % g->w, gz = (goal / L) / g->w;
    // the estimate assumes the cheapest ground there is (roads), or the search never looks sideways for them
    const unsigned char *g_roadMask = ROAD_MASK(g), *g_treeDist = TREE_DIST(g);   // (only this grid's tables)
    // vehicles: the full weight too (0.5 kept it exact for the cheap paved roads - and searched half of a big map: a
    // Damavand re-route froze the server 1.5 s); a road-hugging route a little less than perfect plans in a fraction
    float hw = ASTAR_HEURISTIC_WEIGHT * (g_pathMask ? g_hMul : g_roadMask ? 0.6f : 1.0f);
    int maxExpanded = s_maxExpanded > 0 ? s_maxExpanded : g_longLegs ? 120000 : 60000;   // (an unreachable goal used to search a whole big map - ~100 ms - and bots retried every
                                                     //  second; water legs: open sea round an island is wide - Kharg's ship to its north shore, 2026-10-22)
    gs[start] = 0; from[start] = -1; stamp[start] = curStamp;
    heap_push(heap, &hn, 0, start);
    int nearNode = -1;   // (partial routes: the closed node nearest the goal)
    float nearH = 1e30f;
    while (hn > 0) {
        int cur = heap_pop(heap, &hn).node;
        if (CLOSED(cur)) continue;
        if (cur == goal) { found = 1; break; }
        stamp[cur] = curStamp + 1;
        if (g_partialOk) {
            int qx = abs(gx - (cur / L) % g->w), qz = abs(gz - (cur / L) / g->w);
            float qh = (float)(qx > qz ? qx + 0.4142f * qz : qz + 0.4142f * qx);
            if (qh < nearH) { nearH = qh; nearNode = cur; }
        }
        if (++closedCount > maxExpanded) break;   // unreachable goals would otherwise search the whole grid
        for (int d = 0; d < 8; d++) {
            int nb = step_node(g, cur, d);
            if (nb < 0 || (pathMask && !pathMask[nb]) || node_blocked(g, nb)) continue;
            // no slicing through a solid corner: a diagonal needs at least one adjacent straight move (requiring both
            // cut MP_Subway in two - its halves join only through diagonal steps)
            if ((d & 1) && step_node(g, cur, (d + 7) & 7) < 0 && step_node(g, cur, (d + 1) & 7) < 0) continue;
            float climb = g->height[nb] - g->height[cur];
            float step = ((d & 1) ? 1.4142f : 1.0f) * g->cell;
            static const float kDriveCost[8] = {1.0f, 0.5f, 1.0f, 1.6f, 2.5f, 12.0f, 4.0f, 1.0f};   // 5: within 2.5 m of a tree; 6: taxi map, off-street at an edge
            float mul = pathMask ? kDriveCost[pathMask[nb] & 7] : (g->flags[nb] & NAV_WATER) ? WATER_COST_MUL :
                        g_roadMask && g_roadMask[nb] == 1 ? 0.6f : g_roadMask && g_roadMask[nb] == 2 ? 0.85f : 1.0f;   // roads
            if (!pathMask && g_treeDist && g_treeDist[nb] <= TREE_NEAR_DM) mul *= 6.0f;   // round a tree, not into it
            if (!pathMask && g->edges[nb] != 0xff && !(g->flags[nb] & NAV_WATER)) mul *= 1.5f;   // beside a wall / edge: a cell off it where there is room
            float cost = gs[cur] + step * mul + (climb > 0 ? climb * 2.0f : 0);
            if (CLOSED(nb)) continue;
            if (!SEEN(nb) || cost < gs[nb]) {
                gs[nb] = cost;
                from[nb] = cur;
                stamp[nb] = curStamp;
                int hx = abs(gx - (nb / L) % g->w), hz = abs(gz - (nb / L) / g->w);
                float heur = (float)(hx > hz ? hx + 0.4142f * hz : hz + 0.4142f * hx) * g->cell;
                if (hn < heapCap) heap_push(heap, &hn, cost + heur * hw, nb);
            }
        }
        for (int d = 0; d < 8 && !g_pathMask; d += 2) {   // knifeable fences (even bits): crossable at a price
            if (!(g->fences[cur] & (1 << d))) continue;
            int c = cur / L, l = cur % L, nc = (c / g->w + kDir[d][1]) * g->w + c % g->w + kDir[d][0];
            int nl = neighbour_layer(g, c, l, nc, 0);
            if (nl < 0) continue;
            int nb = nc * L + nl;
            float cost = gs[cur] + g->cell + FENCE_COST;
            if (CLOSED(nb)) continue;
            if (!SEEN(nb) || cost < gs[nb]) {
                gs[nb] = cost;
                from[nb] = cur;
                stamp[nb] = curStamp;
                int hx = abs(gx - (nb / L) % g->w), hz = abs(gz - (nb / L) / g->w);
                float heur = (float)(hx > hz ? hx + 0.4142f * hz : hz + 0.4142f * hx) * g->cell;
                if (hn < heapCap) heap_push(heap, &hn, cost + heur * hw, nb);
            }
        }
        for (int k = 0; k < g_linkCount && !g_pathMask; k++) {   // ladders
            if (g_links[k].g != g || g_links[k].from != cur) continue;
            int nb = g_links[k].to;
            float cost = gs[cur] + g_links[k].cost;
            if (CLOSED(nb)) continue;
            if (!SEEN(nb) || cost < gs[nb]) {
                gs[nb] = cost;
                from[nb] = cur;
                stamp[nb] = curStamp;
                int hx = abs(gx - (nb / L) % g->w), hz = abs(gz - (nb / L) / g->w);
                float heur = (float)(hx > hz ? hx + 0.4142f * hz : hz + 0.4142f * hx) * g->cell;
                if (hn < heapCap) heap_push(heap, &hn, cost + heur * hw, nb);
            }
        }
    }
    snprintf(g_lastError, sizeof g_lastError, "%s after %d nodes (start h%.1f, goal h%.1f)", found ? "ok" : "A* exhausted",
             closedCount, g->height[start], g->height[goal]);
    if (!found && g_partialOk && nearNode >= 0 && nearNode != start) { goal = nearNode; found = 1; g_partialUsed = 1; }
    // Boxed in (the 2 m grid sees a tight spot as enclosed): plan from a nearby open node and walk to it first.
    if (!found && closedCount < 64 && !X->rescuing && maxPts > 1 && !g_pathMask) {
        int cx = (start / L) % g->w, cz = (start / L) / g->w, tries = 0;
        unsigned pocket = curStamp + 1;   // stamp of the nodes the failed search closed (nested searches use newer stamps)
        for (int r = 1; r <= 3 && tries < 3; r++)
            for (int dz = -r; dz <= r; dz++)
                for (int dx = -r; dx <= r; dx++) {
                    if (abs(dx) != r && abs(dz) != r) continue;
                    int x2 = cx + dx, z2 = cz + dz;
                    if (x2 < 0 || z2 < 0 || x2 >= g->w || z2 >= g->h) continue;
                    for (int l = 0; l < L; l++) {
                        int node = (z2 * g->w + x2) * L + l;
                        if (!(g->flags[node] & NAV_WALKABLE) || !g->edges[node] || stamp[node] == pocket) continue;
                        if (fabsf(g->height[node] - g->height[start]) > 1.5f || tries >= 3) continue;
                        float p[3], s[3] = {sx, sy, sz};
                        node_pos(g, node, p);
                        if (ray && !nav_segment_clear(ray, s, p)) continue;   // must be walkable to in a straight line
                        tries++;
                        X->rescuing = 1;
                        int n = nav_find_path(g, ray, p[0], p[1], p[2], tx, ty, tz, out + 3, maxPts - 1);
                        X->rescuing = 0;
                        if (n > 0) {
                            memcpy(out, p, sizeof p);
                            memmove(g_outLink + 1, g_outLink, sizeof g_outLink[0] * (size_t)(n < NAV_MAX_OUT - 1 ? n : NAV_MAX_OUT - 1));
                            g_outLink[0] = -1;
                            return n + 1;
                        }
                    }
                }
    }
    if (found) {
        int len = 0;
        for (int c = goal; c >= 0; c = from[c]) len++;
        int *nodes = nav_alloc(len * sizeof(int));
        int k = len;
        for (int c = goal; c >= 0; c = from[c]) nodes[--k] = c;
        if (X->nodesOut) {   // the route worker: the raw chain - the game thread pulls the strings (its rays are the engine's)
            int m = len < X->nodesMax ? len : X->nodesMax;
            memcpy(X->nodesOut, nodes, (size_t)m * sizeof(int));
            nav_release(nodes);
            return m;
        }
        count = nav_smooth_path(g, ray, sx, sy, sz, nodes, len, out, maxPts);
        nav_release(nodes);
    }
    #undef SEEN
    #undef CLOSED
    return count;
}
