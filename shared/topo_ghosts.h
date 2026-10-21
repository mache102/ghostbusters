// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/collision.h"
#include "box2d/id.h"
#include "box2d/math_types.h"
#include "box2d/types.h"

#include <stdbool.h>
#include <stdint.h>

// Per-contact topological ghosts.
//
// Box2D chains suppress ghost collisions using ghost vertices that are baked into each chain
// segment. Baked ghosts cannot follow moving or dynamic bodies, cannot see other shapes, and
// cannot depend on which body is colliding. This module recomputes the local surface topology
// around a designated player body every step, at the current poses, and uses it for every contact
// of the player, following the localSeg rules from marblebuilds:
//
// 1. Interior cancellation: an edge that coincides with an antiparallel edge of a neighbor shape
//    is interior and is cancelled. If the player is fully embedded in one solid polygon, the
//    neighbor's face is kept as a wall so the player cannot phase on into the neighbor. If the
//    player's centroid is inside the touched solid, it is pushed out through that solid's surviving
//    outer faces only (never through an interior face into the neighbor).
// 2. Covering: an edge is cancelled for this player when the player rests on a surface (in front
//    of it, over its span, sunk less than penetrationTolerance) and the edge lies behind that
//    surface's line. It can only be reached through the support surface.
// 3. Ghost assignment: each surviving edge gets ghost vertices from the surviving edges that meet
//    it, by the smallest turn (the localSeg angle rule). A surface the player stands in front of
//    that hides the edge behind it takes priority (the marblebuilds covering rule, made player
//    relative).
//
// The topology is built once per step and per player shape in TopoGhost_PreStep, for the whole
// snapshotted neighborhood, in a frame with world axes and its origin at the player. The callbacks
// only look it up: pre-solve collides the touched shape's surviving edges against the player as
// chain segments with the cached ghosts and replaces the manifold with the result; pre-continuous
// judges time of impact events on the cached topology.
//
// Works with stock Box2D v3.2: the neighborhood (geometry and poses) is snapshotted before each
// step, because world queries are not allowed inside pre-solve, pre-solve sees the same poses as
// before the step, and the continuous phase runs while other threads write body poses. A touched
// shape missing from the step topology (it entered range after the snapshot, it is excluded by the
// player filter, or the player shape is beyond TOPO_MAX_PLAYER_SHAPES) falls back to building its
// local topology inside the callback.
//
// Requirements: contact recycling must be off for the player (a recycled contact skips pre-solve,
// TopoGhost_PreStep turns it off), and filterFcn must mirror any collision rules beyond filter bits
// (custom filter callback, joints with collideConnected = false, one-way platforms in pre-solve).

#ifdef __cplusplus
extern "C"
{
#endif

#define TOPO_MAX_NEIGHBORS 128

// Player shapes that get a once-per-step topology. Further shapes use the per-contact fallback.
#define TOPO_MAX_PLAYER_SHAPES 2

// Snapshot of a neighbor shape taken before the step
typedef struct TopoGhostShape
{
	b2ShapeId shapeId;
	b2ShapeType type;
	b2Filter filter;
	b2AABB bounds;
	b2WorldTransform transform;

	// Local geometry: the polygon, or the segment points in vertices[0] and vertices[1]
	b2Polygon geometry;
} TopoGhostShape;

// Return false to exclude a neighbor shape from the player's topology (per-entity rules such as
// pass-through blocks). Called from TopoGhost_PreStep, and from pre-solve for shapes missing from
// the step topology, so it must be thread-safe.
typedef bool TopoGhostFilterFcn( b2ShapeId playerShapeId, b2ShapeId neighborShapeId, void* context );

typedef struct TopoGhostConfig
{
	// Endpoints and lines closer than this are treated as coincident
	float linearTolerance;

	// Sine of the angle under which edges are treated as parallel
	float angularTolerance;

	// Extra margin around the player when gathering neighbors
	float queryMargin;

	// A face lying behind the surface the player rests on can only be reached by sinking into that
	// surface. Up to this depth such contacts are treated as ghosts.
	float penetrationTolerance;

	// World step, used to size the neighborhood query
	float timeStep;
} TopoGhostConfig;

typedef struct TopoGhostStats
{
	int unchanged; // contacts left alone (unsupported shapes)
	int replaced;  // manifolds rebuilt from topological chain segments
	int culled;	   // manifolds removed (all candidate edges interior or skipped)
	int kept;	   // centroid inside a solid, pushed out through the outer faces
	int vetoed;	   // continuous collision events vetoed on interior features
	int overflow;  // capacity limits hit (neighbors, edges or edge splits); topology may be incomplete
} TopoGhostStats;

// Internal topology types, stored in the context. Not part of the API.

#define TOPO_MAX_SHAPE_EDGES 8
#define TOPO_MAX_INTERVALS 6
#define TOPO_MAX_SUPPORTS 16
#define TOPO_MAX_STEP_EDGES ( TOPO_MAX_NEIGHBORS * TOPO_MAX_SHAPE_EDGES )
#define TOPO_MAX_STEP_PIECES ( 2 * TOPO_MAX_STEP_EDGES )
#define TOPO_ADJACENCY_WORDS ( ( TOPO_MAX_NEIGHBORS + 31 ) / 32 )

typedef struct TopoInterval
{
	float t0, t1;
} TopoInterval;

typedef struct TopoEdge
{
	b2Vec2 p1, p2;
	int owner;

	// Surviving portions of the edge after interior cancellation, as parameters along p1 -> p2
	TopoInterval intervals[TOPO_MAX_INTERVALS];
	int intervalCount;
} TopoEdge;

// A shape contributing edges. Its edges and its pieces are contiguous ranges.
typedef struct TopoOwner
{
	const TopoGhostShape* shape;
	b2Polygon polygon; // in the scene frame, valid when isSolid
	int firstEdge, edgeCount;
	int firstPiece, pieceCount;
	bool isSolid;
	bool centroidInside; // push out through the outer faces of this solid
	bool fullyInside;	 // embedded: the neighbor faces become walls against phasing
	bool truncated;		 // piece capacity hit, the pieces of this owner are incomplete
} TopoOwner;

// A surface line the player is resting on
typedef struct TopoSupport
{
	b2Vec2 point;
	b2Vec2 normal;
} TopoSupport;

// A surviving piece of an edge, used for ghost candidates and collision
typedef struct TopoPiece
{
	b2Vec2 a, b;
	b2Vec2 ghost1, ghost2; // ghost vertices with covered pieces excluded, when computed
	int edgeIndex;
	int slot; // interval index within the edge
} TopoPiece;

// Player shape geometry in a scene frame
typedef struct TopoPlayer
{
	b2Vec2 centroid;
	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	int pointCount;
	float radius;
} TopoPlayer;

typedef struct TopoScene
{
	TopoOwner* owners;
	TopoEdge* edges;
	TopoPiece* pieces;
	bool* covered;
	int ownerCapacity, edgeCapacity, pieceCapacity;
	int ownerCount, edgeCount, pieceCount;

	// Owner pairs whose bounds overlap: only these can share edges or vertices
	uint32_t adjacency[TOPO_MAX_NEIGHBORS][TOPO_ADJACENCY_WORDS];

	TopoPlayer player;
	TopoSupport supports[TOPO_MAX_SUPPORTS];
	int supportCount;
	float penetrationTolerance;

	// Capacity limit hit while building
	bool overflow;
} TopoScene;

// The topology of one player shape for the current step
typedef struct TopoStepScene
{
	b2ShapeId playerShapeId;
	b2WorldTransform frame; // world axes, origin at the player's pre-step position
	TopoScene scene;
	TopoOwner owners[TOPO_MAX_NEIGHBORS];
	TopoEdge edges[TOPO_MAX_STEP_EDGES];
	TopoPiece pieces[TOPO_MAX_STEP_PIECES];
	bool covered[TOPO_MAX_STEP_PIECES];
} TopoStepScene;

// Large (a few hundred KB). Must stay at a fixed address while the world lives.
typedef struct TopoGhostContext
{
	b2WorldId worldId;
	b2BodyId playerId;
	TopoGhostConfig config;

	TopoGhostFilterFcn* filterFcn;
	void* filterContext;

	// Neighborhood and player pose snapshotted before the step
	TopoGhostShape neighbors[TOPO_MAX_NEIGHBORS];
	int neighborCount;
	b2WorldTransform playerTransform;

	// Topology built once per step for each player shape, only read by the callbacks
	TopoStepScene stepScenes[TOPO_MAX_PLAYER_SHAPES];
	int stepSceneCount;

	// Diagnostic counters. Not synchronized; exact only with a single worker.
	TopoGhostStats stats;
} TopoGhostContext;

TopoGhostConfig TopoGhost_DefaultConfig( void );

void TopoGhost_Init( TopoGhostContext* context, b2WorldId worldId, b2BodyId playerId );

// Snapshot the shapes around the player and build the topology for the step. Call before every
// b2World_Step.
void TopoGhost_PreStep( TopoGhostContext* context );

// Pre-solve and pre-continuous callbacks. Pass the context as the callback context.
void TopoGhostPreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context );
bool TopoGhostPreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* context );

#ifdef __cplusplus
}
#endif
