// Automatic navigation grid for BotMod: built from physics raycasts (no hand-placed markers), A* path planning.
// Each 2D cell holds up to NAV_LAYERS stacked surfaces (e.g. bridge deck, road under it, basement), so bots can
// walk under overpasses and through ground floors.
#pragma once

typedef int (*NavRayFn)(const float *from, const float *to, float *hitPos, float *hitNormal);

#define NAV_LAYERS   6   // carriers / multi-storey buildings stack many floors
#define NAV_WALKABLE 0x01
#define NAV_WATER    0x02   // water surface: swimming (slow, a last resort for the planner)

// What the last ray hit, for the builder (set by the host): water surface / a fence a knife gets through.
#define NAV_HIT_WATER 1
#define NAV_HIT_FENCE 2
typedef int (*NavHitClassFn)(void);
void nav_set_hit_classifier(NavHitClassFn fn);

typedef struct NavGrid {
    char level[32];
    float x0, z0, cell;        // world position of cell (0,0) centre, cell size (m)
    int w, h;                  // cells along x, z
    // per node (cell * NAV_LAYERS + layer, layer 0 = topmost surface)
    float *height;
    unsigned char *flags;      // NAV_* bits
    unsigned char *edges;      // bit d = can walk to the matching layer of the neighbour in direction d
    unsigned char *fences;     // straight directions d (even): bit d = only a knifeable fence is in the way;
                               // bit d+1 = the way is a doorway off the cell centre line (walk through its gap)
    // incremental build state
    int phase;                 // 0 = ground probes, 1 = edge probes, 2 = done
    int cursor;
    unsigned raysCast;
    double buildMs;
    unsigned serial;           // unique per grid made this run - caches key on it, not the address (a new map's grid
                               // can be allocated where the old one was)
} NavGrid;

NavGrid *nav_create(const char *level, float x0, float z0, float x1, float z1, float cell);
void nav_free(NavGrid *g);
// Runs up to rayBudget raycasts / maxMs milliseconds of building. Returns 1 once the grid is complete.
// Optional: raycast that ignores terrain, used to stop probing below the terrain surface.
void nav_set_terrain_test(NavRayFn rayIgnoringTerrain);
int nav_build_step(NavGrid *g, NavRayFn ray, int rayBudget, double maxMs);
int nav_save(const NavGrid *g, const char *path);
NavGrid *nav_load(const char *path);
// Plans a path between world positions (y picks the level at start/goal; pass NAN for "lowest walkable").
// Writes up to maxPts smoothed waypoints (x, y, z triples). Returns the count, 0 if unreachable.
// If ray is non-NULL, every straight shortcut is also verified with knee/chest-height physics rays.
const char *nav_last_error(void);
// Diagnostics: node chosen for (x, y, z) and how many nodes are reachable from it (0 = no node).
int nav_reach(const NavGrid *g, float x, float y, float z, float *nodePos);
// Re-probes the ground of the cells in [x0..x1] x [z0..z1] with rays starting at fromY (below a bridge deck or roof
// whose layers used up the cell's surfaces), adds the ground it finds and re-links the area. Returns nodes added.
int nav_reprobe_area(NavGrid *g, NavRayFn ray, float x0, float z0, float x1, float z1, float fromY);   // why the last nav_find_path failed
int nav_find_path(const NavGrid *g, NavRayFn ray, float sx, float sy, float sz, float tx, float ty, float tz, float *out, int maxPts);
// Route worker (2026-10-25): the A* alone, off the game thread, and the string pulling of its node chain on the game thread.
int nav_find_nodes(const NavGrid *g, float sx, float sy, float sz, float tx, float ty, float tz, int *nodes, int maxNodes);
int nav_smooth_path(const NavGrid *g, NavRayFn ray, float sx, float sy, float sz, const int *nodes, int len, float *out, int maxPts);
void nav_set_worker_thread(unsigned tid);   // that thread searches with its own mask / blockers / buffers
void nav_set_free_hook(void (*f)(const NavGrid *));   // called before a grid is freed
const unsigned char *nav_path_mask(void);
int nav_long_legs(void);
void nav_vehicle_state(float *half, float *hmul);
void nav_set_vehicle_state(float half, float hmul);
int nav_get_blockers(float *xyzr, int max);
void nav_set_last_error(const char *s);
// Removes the links out of the node at (x, y, z) that head towards (tx, tz) (+-60 deg); used when a bot gets stuck.
int nav_block_towards(NavGrid *g, float x, float y, float z, float tx, float tz);
// Puts back links blocked by nav_block_towards once their 3 minutes are up (all = every one). Returns how many.
int nav_unblock_expired(NavGrid *g, int all);
// Re-checks the upward links of every node within `radius` of (x, y, z) with a fine ground sample (every 0.5 m) and
// removes those that climb steeper than a soldier can walk (short steep banks the 2 m build probe stepped over).
// Returns the number of links removed.
int nav_repair_around(NavGrid *g, NavRayFn ray, float x, float y, float z, float radius, float dirX, float dirZ);   // dir 0,0 = all directions
int nav_cell_index(const NavGrid *g, float x, float z);
// Water surface at (x, y, z)? (the node nearest there is flagged NAV_WATER)
int nav_is_water(const NavGrid *g, float x, float y, float z);
// Vehicles: a mask of the nodes a car can drive (dry, gently sloped, open on all 8 sides at a similar height - a
// corridor of 2 cells or more). nav_set_path_mask(mask) makes nav_find_path stay on it (no ladders, fences, doorways);
// NULL = soldiers again. The mask is the caller's (free with HeapFree on the process heap).
unsigned char *nav_drive_mask(const NavGrid *g, int *count);
// Looser mask for tanks on steep maps (steeper ground, no full link set), kept well off edges. Classes 2-4.
unsigned char *nav_tank_mask(const NavGrid *g, int *count);
// Strikes the drivable nodes within `radius` of (x, z) off the mask (a vehicle got stuck there). Returns how many.
// Puts back the nodes nav_mask_block cleared around (x, z), from base (a copy taken before). Returns how many.
int nav_mask_restore(const NavGrid *g, unsigned char *mask, const unsigned char *base, float x, float z, float radius);
// Walkable floor heights at (x, z), nearest to y first (several when floors stack: a walkway over a platform). Count.
// Diagnostics: re-runs the build's checks for the link from the node nearest (x, y, z) in direction d (0..7) and
// describes them in out.
void nav_debug_edge(const NavGrid *g, NavRayFn ray, float x, float y, float z, int d, char *out, int outSize);
int nav_floors_at(const NavGrid *g, float x, float y, float z, float *heights, int max);
int nav_mask_block(const NavGrid *g, unsigned char *mask, float x, float z, float radius);
void nav_node_pos(const NavGrid *g, int node, float *out);   // world position of a node
// Re-checks the grid's staircase links (rises of 0.9-1.6 m) with the strict rule - real stairs show intermediate heights,
// a wall face does not - a few thousand rays per call. Returns 1 when the pass is complete (*removed = links dropped).
int nav_fix_stairs_step(NavGrid *g, NavRayFn ray, int rayBudget, int *removed);
// Water under something (a ship's hull and decks, a pier): a link from such a water node to its neighbour is cut if a ray just over the
// water hits between them - a hull wall at the waterline, not open water under a bridge. Incremental: 1 when done (*cut: links cut).
int nav_fix_hull_step(NavGrid *g, NavRayFn ray, int rayBudget, int *cut);
int nav_cut_blocked_around(NavGrid *g, NavRayFn ray, float x0, float z0, float x1, float z1);   // remove-only re-test round a stuck bot
void nav_release_search_buffers(void);   // (core unload)
int nav_fix_doors_step(NavGrid *g, NavRayFn ray, int rayBudget, int *cut);   // doorway links a body does not fit through: removed
int nav_force_link(NavGrid *g, const float *a, const float *b);   // a staircase link both ways between neighbouring cells (manual)
int nav_hull_restored(void);   // links the last nav_fix_hull_step pass put back (clear now)
void nav_set_path_mask(const unsigned char *mask);
// n circles {x, y, z, r} whose nodes are off limits (the next plans, on foot or driving, until cleared) - other vehicles standing in the way,
// routed round like any wall. No route round them: nav_find_path plans without them. NULL / 0 clears.
void nav_set_blockers(const float *xyzr, int n);
// Per node: distance to the nearest tree trunk in decimetres (255 = none within 2.5 m), or NULL. Soldiers' paths keep
// off nodes within 1.2 m of one (dear, and no straight short cut past it).
void nav_set_tree_dist(const NavGrid *g, const unsigned char *dist);
// The node a step in direction d (0..7) leads to, if the grid connects them (-1 otherwise).
int nav_step_node(const NavGrid *g, int node, int d);
int nav_nearest_node(const NavGrid *g, float x, float y, float z);
// Drop links the last stair re-check put back (ones an older version deleted as "false staircases").
int nav_stairs_restored(void);
// Nodes the last re-check found to be handrail / wall tops (no floor half a metre to two sides) and made unwalkable.
int nav_rails_removed(void);
// Staircase-up links the last re-check added (ones the build missed: flights rising towards -x / -z).
int nav_stairs_added(void);
// Long straight legs on the path mask (open water: a boat plots a direct line), instead of short road-hugging ones.
void nav_set_long_legs(int on);
void nav_set_max_expanded(int n);   // A* expansion cap (0 = defaults: 60000, long legs 120000)
void nav_set_partial(int on);   // routes that cannot reach the goal: to the nearest point they can (boats)
int nav_partial_used(void);
int nav_partial_ok(void);
void nav_set_partial_used(int used);   // (the route worker hands it back)
// Boats: water nodes connected all round (class 1 open water, 3 next to land - the beach).
unsigned char *nav_boat_mask(const NavGrid *g, int *count);
// The node of `mask` nearest (x, z) within maxDist (any height): 1 with its position.
int nav_mask_nearest(const NavGrid *g, const unsigned char *mask, float x, float z, float maxDist, float *out);
// Boat landing: beach nodes of a boat mask (class 3), best first by boat distance from (sx, sz) + walkW x the walk to
// (gx, gz). Writes up to max positions (x, y, z triples) and the dry land node next to each. Returns the count.
int nav_landings(const NavGrid *g, const unsigned char *mask, float sx, float sz, float gx, float gz, float walkW,
                 float *water, float *land, int max);
// Landmasses: every dry walkable node labelled by the piece of land it is on (flood fill that stops at water; 0 = none).
// Caller frees with HeapFree. nav_landing_on() is nav_landings() restricted to beaches of landmass `label`, at least
// minStart from (sx, sz).
int *nav_land_labels(const NavGrid *g);
// Connected regions of a mask (e.g. the drive mask: the land a vehicle can drive between), labels 1.. per node, 0 off the mask.
// HeapFree the result.
int *nav_mask_labels(const NavGrid *g, const unsigned char *mask);
// The mask node nearest (x, z) within maxDist that is in the connected piece `label` (labels from nav_mask_labels)
int nav_mask_nearest_in(const NavGrid *g, const unsigned char *mask, const int *labels, int label, float x, float z, float maxDist, float *out);
// Open water (boat mask class 1) nearest (x, z) within maxDist: 1 with its position.
int nav_open_water_near(const NavGrid *g, const unsigned char *mask, float x, float z, float maxDist, float *out);
// Land (any cell with no node of the boat mask: shore, rocks, piers) within radius of (x, z): its mean position in
// *mx, *mz and the distance to the nearest of it in *nearest. Returns the number of land cells (0 = open water).
int nav_land_around(const NavGrid *g, const unsigned char *mask, float x, float z, float radius, float *mx, float *mz, float *nearest);
// Clears every node of class cls from mask except within keepR of one of the nkeep points (x, z pairs).
void nav_mask_strip(const NavGrid *g, unsigned char *mask, unsigned char cls, const float *keep, int nkeep, float keepR);
int nav_label_at(const NavGrid *g, const int *labels, float x, float y, float z);
int nav_landing_on(const NavGrid *g, const unsigned char *mask, const int *labels, int label, float sx, float sz, float gx, float gz,
                   float walkW, float minStart, float *water);
// Soldiers too prefer roads: with a drive mask set here, paved nodes (class 1) cost x0.6 and smooth ones (2) x0.85.
void nav_set_road_mask(const NavGrid *g, const unsigned char *mask);
// Special one-way links (ladders): from the node at a to the node at b, with an extra cost. Paths through one keep a
// and b as exact waypoints; nav_path_link(k) names the link whose far end is waypoint k of the last nav_find_path
// (-1 for ordinary waypoints). Links belong to one grid and are not saved with it.
void nav_clear_links(void);
int nav_add_link(const NavGrid *g, const float *a, const float *b, float cost);   // link id, -1 if a or b has no node
void nav_disable_link(int id);
int nav_path_link(int k);
// Walkable ground point near (x, y, z) within the planner's vertical tolerance; 0 if none.
int nav_snap(const NavGrid *g, float x, float y, float z, float *out);
// Body-width clearance test (3 parallel rays at knee and chest height) between two ground points.
int nav_segment_clear(NavRayFn ray, const float *a, const float *b);
void nav_set_vehicle_half(float h);
void nav_mask_streets(const NavGrid *g, unsigned char *mask, const unsigned short *mats);   // street classes (the user's strip rule; mats may be NULL)
unsigned char *nav_mask_clear_of(const NavGrid *g, const unsigned char *src, int maxClass, float radius, int *count);
