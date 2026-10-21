// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/collision.h"
#include "box2d/id.h"
#include "box2d/math_types.h"
#include "box2d/types.h"

#include <stdbool.h>

// Per-contact topological ghosts.
//
// Box2D chains suppress ghost collisions using ghost vertices that are baked into each chain
// segment. Baked ghosts cannot follow moving or dynamic bodies, cannot see other shapes, and
// cannot depend on which body is colliding. This module recomputes the local surface topology for
// every contact of a designated player body, at the current poses, using the localSeg rules from
// marblebuilds:
//
// 1. Interior cancellation: an edge that coincides with an antiparallel edge of a neighbor shape
//    is interior and is cancelled. If the player is fully embedded in one solid polygon, the
//    neighbor's face is kept as a wall so the player cannot phase on into the neighbor. If the
//    player's centroid is inside the touched solid, it is pushed out through that solid's surviving
//    outer faces only (never through an interior face into the neighbor).
// 2. Covering: an edge of the touched shape is cancelled for this player when the player rests on
//    a surface (in front of it, over its span, sunk less than penetrationTolerance) and the edge
//    lies behind that surface's line. It can only be reached through the support surface.
// 3. Ghost assignment: each surviving edge of the touched shape gets ghost vertices from the
//    surviving edges that meet it, by the smallest turn (the localSeg angle rule). A surface the
//    player stands in front of that hides the edge behind it takes priority (the marblebuilds
//    covering rule, made player relative).
//
// The touched shape's surviving edges are then collided against the player as chain segments with
// those ghosts, and the result replaces the manifold in pre-solve.
//
// Works with stock Box2D v3.2: the neighborhood (geometry and poses) is snapshotted before each
// step, because world queries are not allowed inside pre-solve, pre-solve sees the same poses as
// before the step, and the continuous phase runs while other threads write body poses.
//
// Requirements: contact recycling must be off for the player (a recycled contact skips pre-solve,
// TopoGhost_PreStep turns it off), and filterFcn must mirror any collision rules beyond filter bits
// (custom filter callback, joints with collideConnected = false, one-way platforms in pre-solve).

#ifdef __cplusplus
extern "C"
{
#endif

#define TOPO_MAX_NEIGHBORS 128

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
// pass-through blocks). Called from pre-solve, so it must be thread-safe.
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

	// Diagnostic counters. Not synchronized; exact only with a single worker.
	TopoGhostStats stats;
} TopoGhostContext;

TopoGhostConfig TopoGhost_DefaultConfig( void );

void TopoGhost_Init( TopoGhostContext* context, b2WorldId worldId, b2BodyId playerId );

// Snapshot the shapes around the player. Call before every b2World_Step.
void TopoGhost_PreStep( TopoGhostContext* context );

// Pre-solve and pre-continuous callbacks. Pass the context as the callback context.
void TopoGhostPreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context );
bool TopoGhostPreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* context );

#ifdef __cplusplus
}
#endif
