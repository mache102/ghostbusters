// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/collision.h"
#include "box2d/id.h"
#include "box2d/math_types.h"
#include "topo_ghosts.h"

#include <stdbool.h>

// Ghost collision challenge scenarios. Each scenario builds a small world around a single
// "player" body and measures how it moves across seams, so the same setup can be visualized in
// the samples app ("Ghost Collisions" category) and swept headlessly by the ghost_probe tool.
//
// The player starts resting on (or flying toward) a surface whose ideal response is known: it
// should keep its tangential speed and stay at a constant height. Deviations are recorded as
// ghost collision symptoms: snags (sudden tangential speed loss), hops/kicks (motion along the
// surface normal), sinking, and falling through.
//
// Multi-body scenarios (Shape Parade) measure several players at once, see GhostRun::players.

#ifdef __cplusplus
extern "C"
{
#endif

#define GHOST_MAX_VARIANTS 8

#define GHOST_MITIGATE( m ) ( 1u << ( m ) )
// Semrau is offered everywhere for comparison, even where it is a no-op (circles: 1-point contacts)
#define GHOST_MITIGATIONS_BASIC                                                                                                  \
	( GHOST_MITIGATE( ghostMitigation_none ) | GHOST_MITIGATE( ghostMitigation_semrau ) |                                        \
	  GHOST_MITIGATE( ghostMitigation_topological ) )
#define GHOST_MITIGATIONS_SEMRAU ( GHOST_MITIGATIONS_BASIC | GHOST_MITIGATE( ghostMitigation_semrau ) )
#define GHOST_MITIGATIONS_ALL ( GHOST_MITIGATIONS_SEMRAU | GHOST_MITIGATE( ghostMitigation_semrauRecycling ) )
// Scenarios with a polygon player also offer the rounded polygon comparison
#define GHOST_ROUNDED GHOST_MITIGATE( ghostMitigation_rounded )

// Default GhostParams::roundness: convex radius = 8% of the polygon width
#define GHOST_DEFAULT_ROUNDNESS 0.08f
#define GHOST_MAX_SWEEP 5
#define GHOST_MAX_AUX_BODIES 4
#define GHOST_MAX_SURFACE_POINTS 128
#define GHOST_MAX_PLAYERS 8

// Pre-solve based mitigations, applied to the player only.
typedef enum GhostMitigation
{
	// No filtering
	ghostMitigation_none = 0,

	// Semrau shallow face-contact culling plus horizontal CCD culling against floor tiles, the
	// same as the upstream Character/Ghost Culling sample. Contact recycling is disabled.
	ghostMitigation_semrau,

	// Same filter but with contact recycling left on (the v3.2 default)
	ghostMitigation_semrauRecycling,

	// Per-contact topological ghosts (shared/topo_ghosts.c). Contact recycling is disabled.
	ghostMitigation_topological,

	// No callbacks: every polygon player shape gets a large convex radius (GhostParams::roundness
	// times its width) with its faces kept in place, so only the corners are rounded (see
	// GhostRoundPolygon). Circles and capsules are already round: same as none for them.
	ghostMitigation_rounded,

	ghostMitigation_count
} GhostMitigation;

// Which parameter a scenario sweeps in the headless probe
typedef enum GhostSweepField
{
	ghostSweep_speed = 0,
	ghostSweep_gravityScale,
	ghostSweep_knob,
} GhostSweepField;

typedef struct GhostParams
{
	int variant;
	GhostMitigation mitigation;
	float speed;
	float gravityScale;
	float knob;

	// ghostMitigation_rounded only: convex radius as a fraction of the polygon width (its minimum
	// width across its faces, e.g. the side of a box). 0 leaves the polygons sharp.
	float roundness;
} GhostParams;

typedef struct GhostMetrics
{
	int steps;
	int snagCount;	 // steps where tangential speed dropped by more than 10% while in the measured region
	int rejectedCount; // contacts culled by the mitigation callbacks
	float minSpeedRatio; // min tangential speed / launch speed
	float maxHop;		 // max height above the resting height along the surface normal
	float maxSink;		 // max depth below the resting height
	float maxKick;		 // max velocity along the surface normal
	float progress;		 // distance travelled along the surface tangent
	float idealProgress; // launch speed * elapsed time, for time loss
	bool finished;		 // reached the end of the path
	bool fellThrough;	 // sank below the fall threshold

	// Scenarios with GhostScenario::measureAirborne only (single player): steps in the measured
	// region where no contact point of the player is within GHOST_AIR_GAP of a surface, and the
	// number of grounded -> airborne transitions (lift-offs)
	int airborneSteps;
	int liftOffCount;
} GhostMetrics;

// Contact point separation above which a player counts as airborne (see GhostMetrics::airborneSteps)
#define GHOST_AIR_GAP 0.01f

typedef struct GhostRun GhostRun;

// One measured body of a multi-body scenario (GhostRun::playerCount > 0). Each player is protected
// by the run's mitigation through its own state and measured separately with the same frame.
typedef struct GhostPlayer
{
	b2BodyId bodyId;
	b2ShapeId shapeId;
	const char* label; // shape name

	// Launched (enabled, velocity set) before run step launchStep. Players never collide with each
	// other (shared negative group index); staggered launches keep them from loading the same dynamic
	// surface pieces at once.
	int launchStep;
	b2Vec2 launchPosition;
	b2Vec2 launchVelocity;
	float launchAngularVelocity;
	bool launched;

	// Kinetic energy factor for curved frame expected speeds: 1 + I / (m r^2) with r = restHeight
	// for rolling shapes, 1 for driven sliding shapes
	float energyFactor;

	// Sliding shapes are driven along the surface tangent by the force that cancels their sliding
	// friction under the gravity load (a platformer drive), 0 for rolling shapes
	float driveForce;
	float restHeight; // radius / inradius: center height when lying on a flat surface
	float lastTangentSpeed;
	bool airborne; // after the last step (measureAirborne scenarios)

	// Topological mitigation state: player 0 uses the run's embedded context, others are heap allocated
	TopoGhostContext* topo;

	GhostMetrics metrics;
} GhostPlayer;

// Measurement frame at a point of a curved surface, see GhostRun::localFrame
typedef struct GhostLocalFrame
{
	b2Vec2 tangent;		 // local surface direction of travel
	b2Vec2 normal;		 // local outward surface normal
	float progress;		 // distance travelled along the path (e.g. arc length)
	float height;		 // signed distance of the player above the surface along normal
	float expectedSpeed; // ideal tangential speed here (e.g. from energy conservation), <= 0: launch speed
} GhostLocalFrame;

struct GhostRun
{
	b2WorldId worldId;
	b2BodyId playerId;
	b2ShapeId playerShapeId;
	int scenarioIndex;
	GhostParams params;

	// Measurement frame
	b2Vec2 origin;
	b2Vec2 tangent;
	b2Vec2 normal;
	float restHeight;	 // player height along the normal when resting on the surface
	float measureStart; // progress where metrics start (skip launch ramps)
	float pathLength;	 // progress that counts as finished
	float fallDepth;	 // sink beyond this counts as falling through
	float lastTangentSpeed;

	// Optional curved measurement frame. When set, GhostRun_Update ignores origin/tangent/normal
	// and measures against the local frame at the player position instead: hop/sink = height -
	// restHeight, kick = velocity along the local normal, progress = frame progress. Speeds are
	// normalized by the frame's expected speed (gravity changes the speed on a curved path):
	// minSpeedRatio = min tangential speed / expected speed, snags = drops of that ratio by more
	// than 10% in one step, and idealProgress integrates the expected speed over time.
	GhostLocalFrame ( *localFrame )( const GhostRun* run, b2Vec2 position );

	// Surface polyline (left to right) for GhostPolylineFrame, in world coordinates
	b2Vec2 surfacePoints[GHOST_MAX_SURFACE_POINTS];
	int surfacePointCount;

	// Optional scenario driven bodies (kinematic platforms) and the time at the start of the
	// current step, steps * 1/60, for deterministic motion
	b2BodyId auxBodyIds[GHOST_MAX_AUX_BODIES];
	b2Vec2 auxOrigins[GHOST_MAX_AUX_BODIES];
	int auxBodyCount;
	float auxPhase;
	float time;

	// State for ghostMitigation_topological
	TopoGhostContext topo;

	// Single player scenarios: the player's metrics. Multi-body scenarios: the worst player (max kick,
	// hop and sink, min speed ratio, the progress / ideal progress of the slowest player, snags and
	// culls summed, fell if any fell, finished only when all finished, steps = run steps).
	GhostMetrics metrics;

	// Multi-body scenarios only (playerCount is 0 otherwise). Player 0 is playerId / playerShapeId /
	// topo. Players are measured by their support point, the lowest point of the shape along the
	// surface normal, instead of their center: hop / sink = gap / penetration between that point and
	// the surface, kick = the smaller of that point's and the center's velocity along the normal (the
	// whole body moves away from the surface). Rolling, sliding, tumbling and rocking across polyline
	// vertices read (near) zero. Progress and speed (snags, minV) are measured at the center.
	GhostPlayer players[GHOST_MAX_PLAYERS];
	int playerCount;
	int framePlayer; // player being measured, read by localFrame callbacks
	int stepCount;	 // world steps since the run was created
	bool airborne;	 // player 0 was airborne after the last step (measureAirborne scenarios)
};

typedef struct GhostScenario
{
	const char* name;
	const char* summary;
	const char* knobName;
	const char* variantNames[GHOST_MAX_VARIANTS];
	int variantCount;

	// Bit mask of GHOST_MITIGATE( ghostMitigation_* ) that make sense for this scenario
	unsigned int mitigationMask;

	float speed;
	float gravityScale;
	float knob;

	GhostSweepField sweepField;
	float sweep[GHOST_MAX_SWEEP];
	int sweepCount;

	// Hard step cap for the headless probe
	int maxSteps;

	void ( *create )( GhostRun* run );

	// Optional, called before each world step (drives kinematic bodies). May be NULL.
	void ( *step )( GhostRun* run );

	// Variant the sample opens with: the one that shows the clearest difference between mitigations
	int defaultVariant;

	// Variants (bit mask) where the correct outcome is being blocked (a real wall or step): finishing
	// the path means the player passed through real geometry
	unsigned int blockedVariants;

	// The outcome is not captured by the metrics, judge it with --trace or in the sample
	bool judgeManually;

	// Optional (may be NULL): the smooth reference variant with the same surface shape for a variant,
	// or -1. The probe judges each player of a multi-body run against the same player in the
	// reference variant run without mitigation, so motion that the surface shape causes by itself
	// (rocking across polyline vertices) is not counted as a ghost symptom.
	int ( *referenceVariant )( int variant );

	// Track airborne steps and lift-offs (GhostMetrics::airborneSteps, liftOffCount). The probe adds
	// them as columns for this scenario and its verdict flags lift-off.
	bool measureAirborne;
} GhostScenario;

extern const GhostScenario g_ghostScenarios[];
extern const int g_ghostScenarioCount;

const char* GhostMitigationName( GhostMitigation mitigation );

// Rounded copy of a convex polygon with the same faces: every face plane moves inward by
// r = roundness * width (width = the polygon's minimum width across its faces) and the convex
// radius grows by r, so the faces end where they were and only the corners are rounded (radius r
// arcs). The core's inradius drops by r. Returns the polygon unchanged for roundness <= 0.
b2Polygon GhostRoundPolygon( const b2Polygon* polygon, float roundness );

// Default parameters for a scenario
GhostParams GhostDefaultParams( int scenarioIndex );

// Build the scenario in an empty world. Installs pre-solve callbacks that reference run, so run
// must stay at a fixed address while the world lives.
void GhostRun_Create( GhostRun* run, b2WorldId worldId, int scenarioIndex, GhostParams params );

// Release memory owned by the run (topological contexts of multi-body scenarios). Call before
// destroying the world or creating the run again. Safe on a zeroed or already destroyed run. The
// metrics (also per player) stay readable.
void GhostRun_Destroy( GhostRun* run );

// Call before each world step (drives scenario bodies, refreshes the topological ghost neighborhood)
void GhostRun_PreStep( GhostRun* run );

// Call after each world step to update the metrics
void GhostRun_Update( GhostRun* run );

// Local frame of the closest point on run->surfacePoints: tangent/normal of the closest segment
// (normal is the left perpendicular, +y for a left to right floor), progress = arc length from the
// first point. expectedSpeed is left at 0 (launch speed).
GhostLocalFrame GhostPolylineFrame( const GhostRun* run, b2Vec2 position );

#ifdef __cplusplus
}
#endif
