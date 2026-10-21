// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

#include "ghost_scenarios.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

// Tags floor tiles so the CCD filter can recognize them, same as the upstream Ghost Culling sample
#define GHOST_FLOOR_MATERIAL 42
#define GHOST_TIME_STEP ( 1.0f / 60.0f )

typedef enum FloorKind
{
	floor_boxTiles,
	floor_segmentTiles,
	floor_customGhostSegments,
	floor_chain,
} FloorKind;

static b2Vec2 ToVec2( b2Pos p )
{
	return (b2Vec2){ (float)p.x, (float)p.y };
}

static b2Pos ToPos( b2Vec2 v )
{
	return (b2Pos){ v.x, v.y };
}

static b2ShapeDef FloorShapeDef( float friction )
{
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = friction;
	shapeDef.material.userMaterialId = GHOST_FLOOR_MATERIAL;
	return shapeDef;
}

static b2BodyId CreateGround( b2WorldId worldId, b2BodyType type )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = type;
	return b2CreateBody( worldId, &bodyDef );
}

// A loose chain segment with explicit ghosts (official v3.2 API)
static b2ShapeId AddChainSegment( b2BodyId bodyId, b2Vec2 ghost1, b2Vec2 p1, b2Vec2 p2, b2Vec2 ghost2, float friction )
{
	b2ShapeDef shapeDef = FloorShapeDef( friction );
	b2ChainSegment cs = { ghost1, { p1, p2 }, ghost2, B2_NULL_INDEX };
	return b2CreateChainSegmentShape( bodyId, &shapeDef, &cs );
}

static float TileTop( int i, float topY, float stepHeight )
{
	return ( i & 1 ) ? topY + stepHeight : topY;
}

// A flat floor of `count` tiles of `width` starting at x0. Odd tiles are raised by stepHeight.
// Segments run right to left so one-sided collision treats +y as the outside.
static void BuildFlatFloor( b2BodyId groundId, FloorKind kind, float x0, int count, float width, float topY, float stepHeight,
							float friction )
{
	if ( kind == floor_chain && stepHeight == 0.0f )
	{
		b2Vec2 points[512];
		B2_ASSERT( count + 1 <= 512 );
		for ( int i = 0; i <= count; ++i )
		{
			points[i] = (b2Vec2){ x0 + ( count - i ) * width, topY };
		}

		b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
		material.friction = friction;
		material.userMaterialId = GHOST_FLOOR_MATERIAL;

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = count + 1;
		chainDef.ghost1 = (b2Vec2){ x0 + ( count + 1 ) * width, topY };
		chainDef.ghost2 = (b2Vec2){ x0 - width, topY };
		chainDef.materials = &material;
		chainDef.materialCount = 1;
		b2CreateChain( groundId, &chainDef );
		return;
	}

	for ( int i = 0; i < count; ++i )
	{
		float xl = x0 + i * width;
		float xr = xl + width;
		float y = TileTop( i, topY, stepHeight );

		// Far endpoints of the neighboring tiles, extrapolated at the ends
		b2Vec2 ghost1 = { xr + width, i + 1 < count ? TileTop( i + 1, topY, stepHeight ) : y };
		b2Vec2 ghost2 = { xl - width, i > 0 ? TileTop( i - 1, topY, stepHeight ) : y };
		b2Vec2 p1 = { xr, y };
		b2Vec2 p2 = { xl, y };

		switch ( kind )
		{
			case floor_boxTiles:
			{
				b2ShapeDef shapeDef = FloorShapeDef( friction );
				b2Polygon box = b2MakeOffsetBox( 0.5f * width, 0.25f, (b2Vec2){ xl + 0.5f * width, y - 0.25f }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
			}
			break;

			case floor_segmentTiles:
			case floor_customGhostSegments:
			{
				b2ShapeDef shapeDef = FloorShapeDef( friction );
				b2Segment segment = { p1, p2 };
				b2ShapeId shapeId = b2CreateSegmentShape( groundId, &shapeDef, &segment );
				if ( kind == floor_customGhostSegments )
				{
					b2CustomShape_SetGhostVertices( shapeId, ghost1, ghost2 );
				}
			}
			break;

			case floor_chain:
				// Misaligned tiles as loose chain segments whose ghosts point at the neighbors
				AddChainSegment( groundId, ghost1, p1, p2, ghost2, friction );
				break;
		}
	}
}

// Pre-solve mitigation: Semrau's shallow face-contact cull, as in the upstream Ghost Culling sample.
// Applies to contacts of playerId, counts culled manifolds in rejectedCount.
static void SemrauPreSolve( b2BodyId playerId, int* rejectedCount, b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold )
{
	if ( manifold->pointCount != 2 )
	{
		return;
	}

	b2Vec2 normal = manifold->normal;
	b2BodyId bodyIdA = b2Shape_GetBody( shapeIdA );
	if ( B2_ID_EQUALS( bodyIdA, playerId ) == false )
	{
		b2BodyId bodyIdB = b2Shape_GetBody( shapeIdB );
		if ( B2_ID_EQUALS( bodyIdB, playerId ) == false )
		{
			return;
		}

		normal = b2Neg( normal );
	}

	b2Vec2 velocity = b2Body_GetLinearVelocity( playerId );
	if ( b2Dot( normal, velocity ) < 0.0f )
	{
		return;
	}

	float contactArea = b2Distance( manifold->points[0].anchorA, manifold->points[1].anchorA );
	if ( contactArea < 4.0f * B2_LINEAR_SLOP )
	{
		manifold->pointCount = 0;

		// Diagnostic only. The headless probe is single threaded.
		*rejectedCount += 1;
	}
}

// CCD mitigation: drop horizontal TOI hits of playerId against floor tiles
static bool SemrauPreContinuous( b2BodyId playerId, int* rejectedCount, b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Vec2 normal )
{
	if ( b2AbsFloat( normal.y ) > 0.1f )
	{
		return true;
	}

	b2ShapeId otherId;
	if ( B2_ID_EQUALS( b2Shape_GetBody( shapeIdA ), playerId ) )
	{
		otherId = shapeIdB;
	}
	else if ( B2_ID_EQUALS( b2Shape_GetBody( shapeIdB ), playerId ) )
	{
		otherId = shapeIdA;
	}
	else
	{
		return true;
	}

	if ( b2Shape_GetUserMaterial( otherId ) == GHOST_FLOOR_MATERIAL )
	{
		*rejectedCount += 1;
		return false;
	}

	return true;
}

static void GhostPreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context )
{
	GhostRun* run = context;
	SemrauPreSolve( run->playerId, &run->metrics.rejectedCount, shapeIdA, shapeIdB, manifold );
}

static bool GhostPreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* context )
{
	(void)point;
	GhostRun* run = context;
	return SemrauPreContinuous( run->playerId, &run->metrics.rejectedCount, shapeIdA, shapeIdB, normal );
}

// Multi-body scenarios: the player whose body is on either side of the contact, NULL if none.
// Players never collide with each other.
static GhostPlayer* FindContactPlayer( GhostRun* run, b2ShapeId shapeIdA, b2ShapeId shapeIdB )
{
	b2BodyId bodyIdA = b2Shape_GetBody( shapeIdA );
	b2BodyId bodyIdB = b2Shape_GetBody( shapeIdB );
	for ( int i = 0; i < run->playerCount; ++i )
	{
		GhostPlayer* player = run->players + i;
		if ( B2_ID_EQUALS( bodyIdA, player->bodyId ) || B2_ID_EQUALS( bodyIdB, player->bodyId ) )
		{
			return player;
		}
	}
	return NULL;
}

// Box2D has one pre-solve and one pre-continuous callback per world: these forward each contact to
// the mitigation state of the player involved
static void MultiPreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context )
{
	GhostRun* run = context;
	GhostPlayer* player = FindContactPlayer( run, shapeIdA, shapeIdB );
	if ( player == NULL )
	{
		return;
	}

	if ( run->params.mitigation == ghostMitigation_topological )
	{
		TopoGhostPreSolve( shapeIdA, shapeIdB, manifold, player->topo );
	}
	else
	{
		SemrauPreSolve( player->bodyId, &player->metrics.rejectedCount, shapeIdA, shapeIdB, manifold );
	}
}

static bool MultiPreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* context )
{
	GhostRun* run = context;
	GhostPlayer* player = FindContactPlayer( run, shapeIdA, shapeIdB );
	if ( player == NULL )
	{
		return true;
	}

	if ( run->params.mitigation == ghostMitigation_topological )
	{
		return TopoGhostPreContinuous( shapeIdA, shapeIdB, point, normal, player->topo );
	}

	return SemrauPreContinuous( player->bodyId, &player->metrics.rejectedCount, shapeIdA, shapeIdB, normal );
}

typedef struct PlayerDef
{
	b2Vec2 position;
	b2Vec2 velocity;
	float angularVelocity;
	float friction;
	float restitution;
	float gravityScale;
	bool lockRotation;
	bool isBullet;
} PlayerDef;

static PlayerDef DefaultPlayerDef( const GhostRun* run, b2Vec2 position )
{
	PlayerDef def = { 0 };
	def.position = position;
	def.velocity = (b2Vec2){ run->params.speed, 0.0f };
	def.gravityScale = run->params.gravityScale;
	def.lockRotation = true;
	return def;
}

static b2BodyId CreatePlayerBody( GhostRun* run, const PlayerDef* def, b2ShapeDef* shapeDef )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = ToPos( def->position );
	bodyDef.linearVelocity = def->velocity;
	bodyDef.angularVelocity = def->angularVelocity;
	bodyDef.gravityScale = def->gravityScale;
	bodyDef.motionLocks.angularZ = def->lockRotation;
	bodyDef.isBullet = def->isBullet;
	bodyDef.enableSleep = false;

	*shapeDef = b2DefaultShapeDef();
	shapeDef->density = 1.0f;
	shapeDef->material.friction = def->friction;
	shapeDef->material.restitution = def->restitution;

	if ( run->params.mitigation == ghostMitigation_topological )
	{
		// The topology is recomputed every step, so the manifold must not be recycled
		bodyDef.enableContactRecycling = false;
		shapeDef->enablePreSolveEvents = true;
		b2World_SetPreSolveCallback( run->worldId, TopoGhostPreSolve, TopoGhostPreContinuous, &run->topo );
	}
	else if ( run->params.mitigation != ghostMitigation_none )
	{
		// Pre-solve must see a fresh manifold at every tile boundary, unless we are testing the default
		bodyDef.enableContactRecycling = run->params.mitigation == ghostMitigation_semrauRecycling;
		shapeDef->enablePreSolveEvents = true;
		b2World_SetPreSolveCallback( run->worldId, GhostPreSolve, GhostPreContinuous, run );
	}

	run->playerId = b2CreateBody( run->worldId, &bodyDef );
	TopoGhost_Init( &run->topo, run->worldId, run->playerId );
	return run->playerId;
}

static void CreatePlayerPolygon( GhostRun* run, const PlayerDef* def, const b2Polygon* polygon )
{
	b2ShapeDef shapeDef;
	b2BodyId bodyId = CreatePlayerBody( run, def, &shapeDef );
	run->playerShapeId = b2CreatePolygonShape( bodyId, &shapeDef, polygon );
}

static void CreatePlayerCircle( GhostRun* run, const PlayerDef* def, float radius )
{
	b2ShapeDef shapeDef;
	b2BodyId bodyId = CreatePlayerBody( run, def, &shapeDef );
	b2Circle circle = { b2Vec2_zero, radius };
	run->playerShapeId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
}

static void CreatePlayerCapsule( GhostRun* run, const PlayerDef* def, const b2Capsule* capsule )
{
	b2ShapeDef shapeDef;
	b2BodyId bodyId = CreatePlayerBody( run, def, &shapeDef );
	run->playerShapeId = b2CreateCapsuleShape( bodyId, &shapeDef, capsule );
}

static void SetFlatFrame( GhostRun* run, float restHeight, float measureStart, float pathLength, float fallDepth )
{
	run->origin = b2Vec2_zero;
	run->tangent = (b2Vec2){ 1.0f, 0.0f };
	run->normal = (b2Vec2){ 0.0f, 1.0f };
	run->restHeight = restHeight;
	run->measureStart = measureStart;
	run->pathLength = pathLength;
	run->fallDepth = fallDepth;
}

// C1/C2/C10: a locked box sliding across a seamed floor under increasing load
static void CreateSlidingBox( GhostRun* run )
{
	static const FloorKind kinds[] = { floor_boxTiles, floor_segmentTiles, floor_customGhostSegments, floor_chain };
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kinds[run->params.variant], -2.0f, 24, 2.0f, 0.0f, 0.0f, 0.0f );

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 40.0f, 0.5f );
}

// C11: alternating tiles raised by a small step
static void CreateMisalignedTiles( GhostRun* run )
{
	static const FloorKind kinds[] = { floor_boxTiles, floor_customGhostSegments, floor_chain };
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kinds[run->params.variant], -2.0f, 24, 2.0f, 0.0f, run->params.knob, 0.0f );

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 40.0f, 0.5f );
}

// C4: a box held at a fixed tilt slides on one corner. Point-face contacts have one manifold
// point, so the Semrau filter (two-point face contacts only) never applies.
static void CreateTiltedBox( GhostRun* run )
{
	static const FloorKind kinds[] = { floor_boxTiles, floor_chain };
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kinds[run->params.variant], -2.0f, 24, 2.0f, 0.0f, 0.0f, 0.0f );

	const float h = 0.5f;
	float angle = run->params.knob * B2_PI / 180.0f;
	b2Rot q = b2MakeRot( angle );

	// Center height so the lowest corner touches the floor
	float restHeight = h * ( b2AbsFloat( q.s ) + q.c );
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, restHeight } );
	b2Polygon box = b2MakeOffsetBox( h, h, b2Vec2_zero, q );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, restHeight, 0.5f, 40.0f, 0.5f );
}

// C5: capsule (rounded) sliding across seams
static void CreateCapsuleSlide( GhostRun* run )
{
	static const FloorKind kinds[] = { floor_boxTiles, floor_segmentTiles, floor_chain };
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kinds[run->params.variant], -2.0f, 24, 2.0f, 0.0f, 0.0f, 0.0f );

	const float radius = 0.25f;
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, radius } );
	b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, radius };
	CreatePlayerCapsule( run, &def, &capsule );
	SetFlatFrame( run, radius, 0.5f, 40.0f, radius );
}

// C6: a thin plank pressed hard into the floor. One-sided chains drop the contact once the
// centroid is behind the segment, so the plank falls through instead of being pushed out.
static void CreateThinPlank( GhostRun* run )
{
	static const FloorKind kinds[] = { floor_chain, floor_boxTiles, floor_customGhostSegments };
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kinds[run->params.variant], -2.0f, 24, 2.0f, 0.0f, 0.0f, 0.0f );

	const float halfHeight = 0.04f;
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, halfHeight } );
	b2Polygon plank = b2MakeBox( 1.0f, halfHeight );
	CreatePlayerPolygon( run, &def, &plank );
	SetFlatFrame( run, halfHeight, 0.5f, 15.0f, 0.5f );
}

// C7: fast diagonal landing onto short segments. CCD runs against raw segments without ghosts,
// so seams cost time (hitching) rather than producing a wrong normal.
static void CreateFastLanding( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	switch ( run->params.variant )
	{
		case 0:
			BuildFlatFloor( groundId, floor_chain, -2.0f, 400, 0.5f, 0.0f, 0.0f, 0.0f );
			break;
		case 1:
			BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 400, 0.5f, 0.0f, 0.0f, 0.0f );
			break;
		default:
			// Reference: one long segment, no seams
			BuildFlatFloor( groundId, floor_chain, -2.0f, 1, 200.0f, 0.0f, 0.0f, 0.0f );
			break;
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 3.0f } );
	def.velocity = (b2Vec2){ run->params.speed, -15.0f };
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );

	// Landing takes about 0.17 s, start measuring after that
	// The floor is 200 m long
	SetFlatFrame( run, 0.5f, 0.3f * run->params.speed, b2MinFloat( 3.0f * run->params.speed, 180.0f ), 0.5f );
}

// C13: a kinematic tile flush with static terrain. No ghost information crosses bodies.
static void CreateKinematicSeam( GhostRun* run )
{
	FloorKind kind = run->params.variant == 0 ? floor_chain : floor_boxTiles;
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kind, -2.0f, 6, 2.0f, 0.0f, 0.0f, 0.0f );
	BuildFlatFloor( groundId, kind, 12.0f, 14, 2.0f, 0.0f, 0.0f, 0.0f );

	b2BodyId platformId = CreateGround( run->worldId, b2_kinematicBody );
	b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
	b2Polygon tile = b2MakeOffsetBox( 1.0f, 0.25f, (b2Vec2){ 11.0f, -0.25f }, b2Rot_identity );
	b2CreatePolygonShape( platformId, &shapeDef, &tile );

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 30.0f, 0.5f );
}

// C14: dynamic tiles resting side by side. Polygon vs polygon only, no ghost information.
static void CreateDynamicTiles( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	{
		b2ShapeDef shapeDef = FloorShapeDef( 0.6f );
		b2Polygon slab = b2MakeOffsetBox( 40.0f, 0.5f, (b2Vec2){ 20.0f, -1.0f }, b2Rot_identity );
		b2CreatePolygonShape( groundId, &shapeDef, &slab );
	}

	for ( int i = 0; i < 20; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ -1.0f + 2.0f * i, -0.25f };
		bodyDef.gravityScale = run->params.gravityScale;
		b2BodyId tileId = b2CreateBody( run->worldId, &bodyDef );

		b2ShapeDef shapeDef = FloorShapeDef( 0.6f );
		shapeDef.density = 1.0f;
		b2Polygon tile = b2MakeBox( 1.0f, 0.25f );
		b2CreatePolygonShape( tileId, &shapeDef, &tile );
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 30.0f, 0.5f );
}

// An open polyline given left to right, created as right to left loose chain segments (outside
// is +y). Ghosts at the ends extrapolate the end segments.
static void AddPolyline( b2BodyId bodyId, const b2Vec2* points, int count, float friction )
{
	b2Vec2 ghostLeft = b2Sub( b2MulSV( 2.0f, points[0] ), points[1] );
	b2Vec2 ghostRight = b2Sub( b2MulSV( 2.0f, points[count - 1] ), points[count - 2] );
	for ( int i = 0; i + 1 < count; ++i )
	{
		b2Vec2 ghost1 = i + 2 < count ? points[i + 2] : ghostRight;
		b2Vec2 ghost2 = i > 0 ? points[i - 1] : ghostLeft;
		AddChainSegment( bodyId, ghost1, points[i + 1], points[i], ghost2, friction );
	}
}

// marblebuilds one-way valve snag: a fast marble comes down a curve into a flat floor and sinks
// below the floor line for a few steps, because each manifold is computed once per step from the
// segment it was touching. It then passes over a one-sided valve segment with an open cell below
// it. The next solid tile's exposed side face has a legitimate convex corner whose vertex can
// catch the sunken marble. Variants 3 and 4 use a kinematic ground body like the game does.
static void CreateValveCorner( GhostRun* run )
{
	int variant = run->params.variant;
	bool kinematic = variant >= 3;
	int ghostMode = kinematic ? variant - 3 : variant;

	b2BodyId groundId = CreateGround( run->worldId, kinematic ? b2_kinematicBody : b2_staticBody );
	const float friction = 0.01f;

	// Curve flattening from slope -1 to -0.2, then the floor. The valve spans x in [2, 3].
	b2Vec2 surface[] = {
		{ -6.0f, 3.6f }, { -4.5f, 2.1f }, { -3.0f, 1.0f }, { -1.5f, 0.3f }, { 0.0f, 0.0f }, { 1.0f, 0.0f },
		{ 2.0f, 0.0f },	 { 3.0f, 0.0f },  { 4.0f, 0.0f },  { 5.0f, 0.0f },	{ 6.0f, 0.0f }, { 7.0f, 0.0f },
		{ 8.0f, 0.0f },	 { 9.0f, 0.0f },
	};
	AddPolyline( groundId, surface, (int)( sizeof( surface ) / sizeof( surface[0] ) ), friction );

	if ( ghostMode != 2 )
	{
		// Exposed left face of the first solid tile, facing the open cell under the valve. ghost2
		// continues along the tile bottom (convex). The local angle rule picks the tile's own top as
		// ghost1, a convex corner. The marblebuilds fix routes ghost1 through the valve's far endpoint.
		b2Vec2 ghost1 = ghostMode == 0 ? (b2Vec2){ 4.0f, 0.0f } : (b2Vec2){ 2.0f, 0.0f };
		AddChainSegment( groundId, ghost1, (b2Vec2){ 3.0f, 0.0f }, (b2Vec2){ 3.0f, -1.0f }, (b2Vec2){ 4.0f, -1.0f }, friction );
	}

	const float radius = 0.2f;
	b2Vec2 slopeDir = b2Normalize( b2Sub( surface[1], surface[0] ) );
	b2Vec2 slopeNormal = b2LeftPerp( slopeDir );
	// The knob shifts the start along the first segment, which changes where steps land relative
	// to the valley and so the sink depth
	b2Vec2 start = b2MulAdd( b2Lerp( surface[0], surface[1], run->params.knob ), radius + 0.01f, slopeNormal );

	PlayerDef def = DefaultPlayerDef( run, start );
	def.velocity = b2MulSV( run->params.speed, slopeDir );
	def.lockRotation = false;
	def.isBullet = true;
	def.friction = friction;
	def.restitution = 0.1f;
	CreatePlayerCircle( run, &def, radius );
	SetFlatFrame( run, radius, 0.0f, 8.0f, radius );
}

// marblebuilds filter/grate snag: every tile keeps all four faces (antiparallel interior faces
// at each seam), each tile a closed loop of loose chain segments.
static void CreateInteriorFaces( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	const float friction = 0.01f;
	const int tileCount = 20;
	const float x0 = -4.0f;

	if ( run->params.variant == 0 )
	{
		for ( int i = 0; i < tileCount; ++i )
		{
			float x = x0 + i;
			b2Vec2 v[4] = { { x, -1.0f }, { x + 1.0f, -1.0f }, { x + 1.0f, 0.0f }, { x, 0.0f } };
			for ( int j = 0; j < 4; ++j )
			{
				AddChainSegment( groundId, v[( j + 3 ) % 4], v[j], v[( j + 1 ) % 4], v[( j + 2 ) % 4], friction );
			}
		}
	}
	else
	{
		BuildFlatFloor( groundId, floor_chain, x0, tileCount, 1.0f, 0.0f, 0.0f, friction );
	}

	const float radius = 0.2f;
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ -3.5f, radius } );
	def.lockRotation = false;
	def.isBullet = true;
	def.friction = friction;
	def.restitution = 0.1f;
	CreatePlayerCircle( run, &def, radius );
	SetFlatFrame( run, radius, -3.0f, 14.0f, radius );
}

// A circle moving horizontally (no gravity) into the end of a platform, below its top line.
// Loose chain segments default their ghosts to their own endpoints, which degenerates the
// Voronoi tests at the segment ends. knob 0 approaches the p2 end, knob 1 the p1 end.
static void CreateDefaultGhosts( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	b2Vec2 p1 = { 4.0f, 0.0f };
	b2Vec2 p2 = { 0.0f, 0.0f };

	switch ( run->params.variant )
	{
		case 0:
		{
			b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
			b2Segment segment = { p1, p2 };
			b2CustomCreateChainSegmentShape( groundId, &shapeDef, &segment );
		}
		break;

		case 1:
			// Ghosts continue the line, as if the platform had collinear neighbors
			AddChainSegment( groundId, (b2Vec2){ 5.0f, 0.0f }, p1, p2, (b2Vec2){ -1.0f, 0.0f }, 0.0f );
			break;

		case 2:
			// Convex corners at both ends, the correct ghosts for an isolated platform top
			AddChainSegment( groundId, (b2Vec2){ 4.0f, -1.0f }, p1, p2, (b2Vec2){ 0.0f, -1.0f }, 0.0f );
			break;

		default:
		{
			b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
			b2Segment segment = { p1, p2 };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}
		break;
	}

	const float radius = 0.25f;
	const float height = 0.1f;
	bool fromRight = run->params.knob > 0.5f;
	b2Vec2 start = fromRight ? (b2Vec2){ 6.0f, height } : (b2Vec2){ -2.0f, height };

	PlayerDef def = DefaultPlayerDef( run, start );
	def.velocity = (b2Vec2){ fromRight ? -run->params.speed : run->params.speed, 0.0f };
	def.gravityScale = 0.0f;
	CreatePlayerCircle( run, &def, radius );

	SetFlatFrame( run, height, -10.0f, 7.0f, 10.0f );
	run->origin = start;
	run->origin.y = 0.0f;
	if ( fromRight )
	{
		run->tangent = (b2Vec2){ -1.0f, 0.0f };
	}
}

// A kinematic tile whose body origin is the tile center, so it rotates about its center. Its
// top is flush with y = 0.
static void AddKinematicTile( GhostRun* run, float centerX )
{
	B2_ASSERT( run->auxBodyCount < GHOST_MAX_AUX_BODIES );
	b2Vec2 center = { centerX, -0.25f };

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_kinematicBody;
	bodyDef.position = ToPos( center );
	b2BodyId bodyId = b2CreateBody( run->worldId, &bodyDef );

	b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
	b2Polygon tile = b2MakeBox( 1.0f, 0.25f );
	b2CreatePolygonShape( bodyId, &shapeDef, &tile );

	run->auxBodyIds[run->auxBodyCount] = bodyId;
	run->auxOrigins[run->auxBodyCount] = center;
	run->auxBodyCount += 1;
}

// Set the aux body velocities so they reach the target offset from their origins at the end of
// the coming step. Tracking the actual pose keeps the motion drift free.
static void DriveAuxBodies( GhostRun* run, b2Vec2 targetOffset, float targetAngle )
{
	for ( int i = 0; i < run->auxBodyCount; ++i )
	{
		b2BodyId bodyId = run->auxBodyIds[i];
		b2Vec2 target = b2Add( run->auxOrigins[i], targetOffset );
		b2Vec2 p = ToVec2( b2Body_GetPosition( bodyId ) );
		float angle = b2Rot_GetAngle( b2Body_GetRotation( bodyId ) );
		b2Body_SetLinearVelocity( bodyId, b2MulSV( 1.0f / GHOST_TIME_STEP, b2Sub( target, p ) ) );
		b2Body_SetAngularVelocity( bodyId, ( targetAngle - angle ) / GHOST_TIME_STEP );
	}
}

// Static floor with kinematic 2 m tiles centered in gaps of gapWidth, one every `spacing` meters.
// Whether a cross-body seam snags depends on where the player's leading edge lands relative to
// the tile edge at the start of a step (a side contact needs a gap below the speculative
// distance). A 1 m box launched from x = 0 at 10 m/s moves 1/6 m per step, so the tiles are
// offset by 2/3, 1/3 and 0 of that travel to sample several phases. The last tile edge (at rest)
// is reached exactly at a step boundary, the degenerate case of the Kinematic Seam scenario.
static void BuildKinematicCourse( GhostRun* run, FloorKind kind, float gapWidth, float spacing )
{
	const int tileCount = GHOST_MAX_AUX_BODIES - 1;
	const float margin = 0.5f * ( gapWidth - 2.0f );
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );

	float floorStart = -2.0f - margin;
	for ( int i = 0; i < tileCount; ++i )
	{
		float gapStart = 10.0f - margin + i * spacing + ( tileCount - 1 - i ) / 18.0f;
		int floorCount = (int)( 0.5f * ( gapStart - floorStart ) + 0.5f );
		BuildFlatFloor( groundId, kind, floorStart, floorCount, ( gapStart - floorStart ) / floorCount, 0.0f, 0.0f, 0.0f );
		AddKinematicTile( run, gapStart + 0.5f * gapWidth );
		floorStart = gapStart + gapWidth;
	}

	BuildFlatFloor( groundId, kind, floorStart, 10, 2.0f, 0.0f, 0.0f, 0.0f );
}

// Triangle wave in [-amplitude, amplitude] starting at 0 and moving positive, as a function of
// the distance travelled
static float TriangleWave( float distance, float amplitude )
{
	if ( amplitude <= 0.0f )
	{
		return 0.0f;
	}

	float period = 4.0f * amplitude;
	float u = fmodf( distance + amplitude, period );
	return u < 2.0f * amplitude ? u - amplitude : 3.0f * amplitude - u;
}

// Time when the leading edge of a player box of half width h launched from x = 0 reaches x
static float ArrivalTime( const GhostRun* run, float x, float h )
{
	return run->params.speed > 0.0f ? ( x - h ) / run->params.speed : 0.0f;
}

static void CreateBoxPlayer( GhostRun* run, float pathLength )
{
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, pathLength, 0.5f );
}

#define MOVING_PLATFORM_TRAVEL 0.25f

static void StepMovingPlatform( GhostRun* run )
{
	float x = TriangleWave( run->params.knob * ( run->time + GHOST_TIME_STEP ), MOVING_PLATFORM_TRAVEL );
	DriveAuxBodies( run, (b2Vec2){ x, 0.0f }, 0.0f );
}

// Platformer: flush kinematic tiles slide back and forth (knob = speed) inside gaps 0.5 m wider
// than the tiles. The open part of a gap is always narrower than the player.
// A rolling ball for the kinematic courses. Semrau never acts on it (1-point contacts).
static void CreateBallPlayer( GhostRun* run, float pathLength )
{
	const float radius = 0.25f;
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, radius } );
	def.lockRotation = false;
	def.friction = 0.6f;
	def.angularVelocity = -run->params.speed / radius;
	CreatePlayerCircle( run, &def, radius );
	SetFlatFrame( run, radius, 0.5f, pathLength, radius );
}

// Variants: 0 box on box tiles, 1 box on chain, 2 ball on box tiles, 3 ball on chain
static void CreateCoursePlayer( GhostRun* run, float pathLength )
{
	if ( run->params.variant >= 2 )
	{
		CreateBallPlayer( run, pathLength );
	}
	else
	{
		CreateBoxPlayer( run, pathLength );
	}
}

static void CreateMovingPlatform( GhostRun* run )
{
	FloorKind kind = ( run->params.variant & 1 ) == 0 ? floor_boxTiles : floor_chain;
	BuildKinematicCourse( run, kind, 2.0f + 2.0f * MOVING_PLATFORM_TRAVEL, 6.5f );
	CreateCoursePlayer( run, 30.0f );
}

#define ELEVATOR_PERIOD 0.8f

static void StepElevatorSeam( GhostRun* run )
{
	float omega = 2.0f * B2_PI / ELEVATOR_PERIOD;
	float y = run->params.knob * sinf( omega * ( run->time + GHOST_TIME_STEP - run->auxPhase ) );
	DriveAuxBodies( run, (b2Vec2){ 0.0f, y }, 0.0f );
}

// Platformer: flush kinematic tiles bobbing vertically around flush (knob = amplitude). The tiles
// are one period apart at 10 m/s and phased so each passes flush moving up as the player reaches
// it. Amplitudes above the linear slop are a real lip at some phases.
static void CreateElevatorSeam( GhostRun* run )
{
	FloorKind kind = run->params.variant == 0 ? floor_chain : floor_boxTiles;
	BuildKinematicCourse( run, kind, 2.0f, 10.0f * ELEVATOR_PERIOD );
	run->auxPhase = ArrivalTime( run, 10.0f, 0.5f );
	CreateBoxPlayer( run, 36.0f );
}

#define ROCKER_PERIOD 1.0f

static void StepRotatingPlatform( GhostRun* run )
{
	float omega = 2.0f * B2_PI / ROCKER_PERIOD;
	float amplitude = run->params.knob * B2_PI / 180.0f;
	float angle = -amplitude * sinf( omega * ( run->time + GHOST_TIME_STEP - run->auxPhase ) );
	DriveAuxBodies( run, b2Vec2_zero, angle );
}

// Platformer: flush kinematic tiles rocking about their centers (knob = amplitude in degrees),
// one period apart at 10 m/s and phased so each is flush with its near edge rising as the player
// reaches it
static void CreateRotatingPlatform( GhostRun* run )
{
	FloorKind kind = ( run->params.variant & 1 ) == 0 ? floor_boxTiles : floor_chain;
	BuildKinematicCourse( run, kind, 2.0f, 10.0f * ROCKER_PERIOD );
	run->auxPhase = ArrivalTime( run, 10.0f, 0.5f );
	CreateCoursePlayer( run, 40.0f );
}

// Platformer slope: a locked box slides down a 30 degree slope made of right triangle tiles
// (marblebuilds b23_planck_ramp style: hypotenuse on top, vertical side at each seam) or the
// same slope as a polyline.
#define SLOPE_ANGLE ( 30.0f * B2_PI / 180.0f )

// 30 degree slope from the origin down to the right, 30 m long: right triangle tiles (1 m) or a
// polyline through the same points
static void BuildSlope( GhostRun* run, bool triangleTiles, float friction )
{
	const int tileCount = 30;
	const float tileLength = 1.0f;
	b2Vec2 down = { cosf( SLOPE_ANGLE ), -sinf( SLOPE_ANGLE ) };

	b2Vec2 points[31];
	for ( int i = 0; i <= tileCount; ++i )
	{
		points[i] = b2MulSV( i * tileLength, down );
	}

	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	if ( triangleTiles )
	{
		b2ShapeDef shapeDef = FloorShapeDef( friction );
		for ( int i = 0; i < tileCount; ++i )
		{
			b2Vec2 v[3] = { points[i], points[i + 1], { points[i].x, points[i + 1].y } };
			b2Hull hull = b2ComputeHull( v, 3 );
			b2Polygon triangle = b2MakePolygon( &hull, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &triangle );
		}
	}
	else
	{
		AddPolyline( groundId, points, tileCount + 1, friction );
	}
}

static void CreateSlopeTiles( GhostRun* run )
{
	const float slope = SLOPE_ANGLE;
	b2Vec2 down = { cosf( slope ), -sinf( slope ) };
	b2Vec2 up = b2LeftPerp( down );

	// Variants: even = triangle tiles, odd = chain. 0/1 locked box, 2/3 rolling ball, 4/5 free box
	BuildSlope( run, ( run->params.variant & 1 ) == 0, 0.0f );

	int player = run->params.variant / 2;
	float h = player == 1 ? 0.3f : 0.5f;
	b2Vec2 start = b2MulAdd( b2MulSV( 1.0f, down ), h, up );
	PlayerDef def = DefaultPlayerDef( run, start );
	def.velocity = b2MulSV( run->params.speed, down );
	if ( player == 1 )
	{
		// Rolling ball: 1-point contacts, Semrau never acts on them
		def.lockRotation = false;
		def.friction = 0.6f;
		def.angularVelocity = -run->params.speed / h;
		CreatePlayerCircle( run, &def, h );
	}
	else
	{
		// Free box (player 2) slides with low friction and can be tipped over by a caught corner
		def.lockRotation = player == 0;
		def.friction = player == 2 ? 0.1f : 0.0f;
		b2Polygon box = b2MakeOffsetBox( h, h, b2Vec2_zero, b2MakeRot( -slope ) );
		CreatePlayerPolygon( run, &def, &box );
	}

	run->origin = b2Vec2_zero;
	run->tangent = down;
	run->normal = up;
	run->restHeight = h;
	run->measureStart = 1.5f;
	run->pathLength = 26.0f;
	run->fallDepth = 0.5f;
}

// Platformer character made of two boxes on one body: the seam between them slides over the
// floor seams.
static void CreateCompoundPlayer( GhostRun* run )
{
	FloorKind kind = run->params.variant == 0 ? floor_boxTiles : floor_chain;
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kind, -2.0f, 24, 2.0f, 0.0f, 0.0f, 0.0f );

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2ShapeDef shapeDef;
	b2BodyId bodyId = CreatePlayerBody( run, &def, &shapeDef );
	b2Polygon left = b2MakeOffsetBox( 0.5f, 0.5f, (b2Vec2){ -0.5f, 0.0f }, b2Rot_identity );
	b2Polygon right = b2MakeOffsetBox( 0.5f, 0.5f, (b2Vec2){ 0.5f, 0.0f }, b2Rot_identity );
	run->playerShapeId = b2CreatePolygonShape( bodyId, &shapeDef, &right );
	b2CreatePolygonShape( bodyId, &shapeDef, &left );
	SetFlatFrame( run, 0.5f, 0.5f, 40.0f, 0.5f );
}

// A rolling circle with friction crossing resting dynamic crates (polygon seams that carry no
// ghost information) or static box tiles
// Static slab with 20 resting dynamic crates (2 m x 0.5 m, tops at y = 0, from x = -2 to 38), or
// static box tiles
static void BuildCrateFloor( GhostRun* run, bool dynamicCrates, float friction )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	if ( dynamicCrates )
	{
		b2ShapeDef shapeDef = FloorShapeDef( friction );
		b2Polygon slab = b2MakeOffsetBox( 40.0f, 0.5f, (b2Vec2){ 20.0f, -1.0f }, b2Rot_identity );
		b2CreatePolygonShape( groundId, &shapeDef, &slab );

		for ( int i = 0; i < 20; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = (b2Pos){ -1.0f + 2.0f * i, -0.25f };
			bodyDef.gravityScale = run->params.gravityScale;
			b2BodyId crateId = b2CreateBody( run->worldId, &bodyDef );

			b2ShapeDef crateDef = FloorShapeDef( friction );
			crateDef.density = 1.0f;
			b2Polygon crate = b2MakeBox( 1.0f, 0.25f );
			b2CreatePolygonShape( crateId, &crateDef, &crate );
		}
	}
	else
	{
		BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 20, 2.0f, 0.0f, 0.0f, friction );
	}
}

static void CreateCrateFloorCircle( GhostRun* run )
{
	const float friction = 0.6f;
	BuildCrateFloor( run, run->params.variant == 0, friction );

	const float radius = 0.25f;
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, radius } );
	def.angularVelocity = -run->params.speed / radius;
	def.lockRotation = false;
	def.friction = friction;
	CreatePlayerCircle( run, &def, radius );
	SetFlatFrame( run, radius, 0.5f, 30.0f, radius );
}

// Interior face relevance: the player spawns embedded in the left of two tiles that share a seam
// at x = 0, centroid 0.1 m left of the seam and knob meters below the top. It should be pushed up
// and slide on along the top, not pushed through the seam or held inside.
static void CreateInteriorPushOut( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
	b2Polygon leftTile = b2MakeOffsetBox( 2.0f, 0.5f, (b2Vec2){ -2.0f, -0.5f }, b2Rot_identity );
	b2Polygon rightTile = b2MakeOffsetBox( 2.0f, 0.5f, (b2Vec2){ 2.0f, -0.5f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shapeDef, &leftTile );
	b2CreatePolygonShape( groundId, &shapeDef, &rightTile );

	const float h = 0.25f;
	b2Vec2 start = { -0.1f, -run->params.knob };
	PlayerDef def = DefaultPlayerDef( run, start );
	if ( run->params.variant == 0 )
	{
		CreatePlayerCircle( run, &def, h );
	}
	else
	{
		b2Polygon box = b2MakeBox( h, h );
		CreatePlayerPolygon( run, &def, &box );
	}

	// Measured from the start, so sink starts at the spawn depth (up to 0.55 m). Ending up below
	// the tile bottom (1 m thick) means it was pushed down through the tile. Progress is not a pass
	// criterion here: the neighbor tile's wall legitimately absorbs the rightward speed while the
	// player is embedded. Pass = pushed out and resting on top (see the final height in --trace).
	SetFlatFrame( run, h, -10.0f, 1.5f, 1.0f + h );
	run->origin = (b2Vec2){ start.x, 0.0f };
}

// ---------------------------------------------------------------------------------------------
// Adversarial scenarios: the correct outcome is often to be blocked or to land somewhere specific,
// so pathLength is placed beyond the obstacle (finishing means passing through). See each summary.

static void AddStaticBox( b2BodyId bodyId, float x0, float y0, float x1, float y1, float friction )
{
	b2ShapeDef shapeDef = FloorShapeDef( friction );
	b2Polygon box = b2MakeOffsetBox( 0.5f * ( x1 - x0 ), 0.5f * ( y1 - y0 ), (b2Vec2){ 0.5f * ( x0 + x1 ), 0.5f * ( y0 + y1 ) },
									 b2Rot_identity );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );
}

// A wall of nx by ny box tiles filling [x0, x1] x [y0, y1]
static void AddTiledBlock( b2BodyId bodyId, float x0, float y0, float x1, float y1, int nx, int ny, float friction )
{
	float w = ( x1 - x0 ) / nx;
	float h = ( y1 - y0 ) / ny;
	for ( int i = 0; i < nx; ++i )
	{
		for ( int j = 0; j < ny; ++j )
		{
			AddStaticBox( bodyId, x0 + i * w, y0 + j * h, x0 + ( i + 1 ) * w, y0 + ( j + 1 ) * h, friction );
		}
	}
}

// Two-sided segments between consecutive points
static void AddSegmentPath( b2BodyId bodyId, const b2Vec2* points, int count, float friction )
{
	b2ShapeDef shapeDef = FloorShapeDef( friction );
	for ( int i = 0; i + 1 < count; ++i )
	{
		b2Segment segment = { points[i], points[i + 1] };
		b2CreateSegmentShape( bodyId, &shapeDef, &segment );
	}
}

// Keep pressing the player along +x (or -x) like a character holding a direction
static void PushPlayer( GhostRun* run, b2Vec2 direction, float accel )
{
	float mass = b2Body_GetMass( run->playerId );
	b2Body_ApplyForceToCenter( run->playerId, b2MulSV( mass * accel, direction ), true );
}

static void StepPushRight( GhostRun* run )
{
	PushPlayer( run, (b2Vec2){ 1.0f, 0.0f }, 20.0f );
}

static void StepPushLeft( GhostRun* run )
{
	PushPlayer( run, (b2Vec2){ -1.0f, 0.0f }, 20.0f );
}

#define ADV_WALL_X 12.0f

// A1: a real 1 m wall at x = 12 rising from a tiled floor; the player is held pressing right.
// Correct: blocked with x = 11.5 for good (never finishes, never hops on top).
static void CreateTileWall( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	int variant = run->params.variant;
	float wx = ADV_WALL_X;
	switch ( variant )
	{
		case 0:
			BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 12, 2.0f, 0.0f, 0.0f, 0.0f );
			AddStaticBox( groundId, wx, 0.0f, wx + 1.0f, 1.0f, 0.0f );
			break;
		case 1:
			// Wall of 0.5 m tiles: seams on the face and inside
			BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 12, 2.0f, 0.0f, 0.0f, 0.0f );
			AddTiledBlock( groundId, wx, 0.0f, wx + 1.5f, 1.0f, 3, 4, 0.0f );
			break;
		case 2:
		{
			// One polyline that turns up into the wall (concave corner baked into the chain)
			b2Vec2 points[] = { { -2.0f, 0.0f }, { 4.0f, 0.0f }, { 8.0f, 0.0f }, { wx, 0.0f }, { wx, 0.5f }, { wx, 1.0f }, { wx + 4.0f, 1.0f } };
			AddPolyline( groundId, points, 7, 0.0f );
		}
		break;
		case 3:
			// Thin 0.1 m wall: tunneling bait
			BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 12, 2.0f, 0.0f, 0.0f, 0.0f );
			AddStaticBox( groundId, wx, 0.0f, wx + 0.1f, 1.0f, 0.0f );
			break;
		default:
		{
			// Wall face of two-sided segment tiles on a segment tile floor
			BuildFlatFloor( groundId, floor_segmentTiles, -2.0f, 12, 2.0f, 0.0f, 0.0f, 0.0f );
			b2Vec2 points[] = { { wx, 0.0f }, { wx, 0.5f }, { wx, 1.0f } };
			AddSegmentPath( groundId, points, 3, 0.0f );
		}
		break;
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, wx + 2.0f, 0.5f );
}

// A2: a real step up of knob meters at x = 12 built from tiles. A locked square box cannot climb
// a vertical riser. Correct: blocked at x = 11.5.
static void CreateTileStepUp( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	float wx = ADV_WALL_X;
	float h = run->params.knob;
	switch ( run->params.variant )
	{
		case 0:
			BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 7, 2.0f, 0.0f, 0.0f, 0.0f );
			// Raised tiles, same thickness, top at h
			for ( int i = 0; i < 6; ++i )
			{
				AddStaticBox( groundId, wx + 2.0f * i, h - 0.5f, wx + 2.0f * ( i + 1 ), h, 0.0f );
			}
			break;
		case 1:
			// Step built by stacking a thin tile of height h on the floor tiles
			BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 13, 2.0f, 0.0f, 0.0f, 0.0f );
			AddStaticBox( groundId, wx, 0.0f, wx + 12.0f, h, 0.0f );
			break;
		case 2:
		{
			b2Vec2 points[] = { { -2.0f, 0.0f }, { 4.0f, 0.0f }, { 8.0f, 0.0f }, { wx, 0.0f }, { wx, h }, { wx + 4.0f, h }, { wx + 12.0f, h } };
			AddPolyline( groundId, points, 7, 0.0f );
		}
		break;
		default:
		{
			BuildFlatFloor( groundId, floor_segmentTiles, -2.0f, 7, 2.0f, 0.0f, 0.0f, 0.0f );
			b2Vec2 points[] = { { wx, 0.0f }, { wx, h }, { wx + 2.0f, h }, { wx + 4.0f, h }, { wx + 12.0f, h } };
			AddSegmentPath( groundId, points, 5, 0.0f );
		}
		break;
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, wx + 2.0f, 0.5f );
}

// A3: floor ends at x = 10, a 3 m deep pit 4 m wide, the far wall at x = 14 back up to y = 0.
// Correct: the player falls into the pit and ends against the far wall or on the pit floor,
// x <= 13.5 (box) and y = -3 + half height. It must never finish (x >= 20).
static void CreateCliffPit( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	int variant = run->params.variant;
	bool tall = variant >= 2;
	bool chain = ( variant & 1 ) != 0;
	if ( chain )
	{
		b2Vec2 points[] = { { -2.0f, 0.0f }, { 4.0f, 0.0f }, { 10.0f, 0.0f }, { 10.0f, -1.5f }, { 10.0f, -3.0f }, { 12.0f, -3.0f },
							{ 14.0f, -3.0f }, { 14.0f, -1.5f }, { 14.0f, 0.0f }, { 20.0f, 0.0f }, { 30.0f, 0.0f } };
		AddPolyline( groundId, points, 11, 0.0f );
	}
	else
	{
		BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 6, 2.0f, 0.0f, 0.0f, 0.0f );
		// Cliff face and far wall made of stacked 0.5 m tiles, pit floor tiles
		AddTiledBlock( groundId, 8.0f, -3.5f, 10.0f, -0.5f, 1, 6, 0.0f );
		AddTiledBlock( groundId, 10.0f, -3.5f, 14.0f, -3.0f, 2, 1, 0.0f );
		AddTiledBlock( groundId, 14.0f, -3.5f, 16.0f, -0.5f, 1, 6, 0.0f );
		BuildFlatFloor( groundId, floor_boxTiles, 14.0f, 8, 2.0f, 0.0f, 0.0f, 0.0f );
	}

	float hy = tall ? 1.5f : 0.5f;
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, hy } );
	b2Polygon box = b2MakeBox( 0.5f, hy );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, hy, 0.5f, 20.0f, 100.0f );
}

// A4: a locked box pressed into a 10 m vertical wall (face at x = 1, outward +x) slides down it
// under gravity onto the pit floor at y = 0. The frame measures progress downward and the
// normal away from the wall. Correct: no snags at the 0.5 m wall seams, sink (penetration into
// the wall) ~0, hop (pushed off the wall) ~0, final rest at (1.5, 0.5).
static void CreateWallSlideDown( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	switch ( run->params.variant )
	{
		case 0:
			AddTiledBlock( groundId, 0.0f, 0.0f, 1.0f, 10.0f, 1, 20, 0.0f );
			AddTiledBlock( groundId, 1.0f, -0.5f, 9.0f, 0.0f, 4, 1, 0.0f );
			break;
		case 1:
		{
			// AddPolyline puts the solid side on the right of each point[i + 1] -> point[i]
			// segment: +x for the wall going down, +y for the floor
			b2Vec2 points[] = { { 1.0f, 10.0f }, { 1.0f, 7.5f }, { 1.0f, 5.0f }, { 1.0f, 2.5f }, { 1.0f, 0.0f }, { 9.0f, 0.0f } };
			AddPolyline( groundId, points, 6, 0.0f );
		}
		break;
		default:
		{
			b2Vec2 points[21];
			for ( int i = 0; i <= 20; ++i )
			{
				points[i] = (b2Vec2){ 1.0f, 10.0f - 0.5f * i };
			}
			AddSegmentPath( groundId, points, 21, 0.0f );
			b2Vec2 floorPoints[] = { { 1.0f, 0.0f }, { 3.0f, 0.0f }, { 5.0f, 0.0f }, { 9.0f, 0.0f } };
			AddSegmentPath( groundId, floorPoints, 4, 0.0f );
		}
		break;
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 1.5f, 9.0f } );
	def.velocity = (b2Vec2){ 0.0f, -run->params.speed };
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );

	run->origin = (b2Vec2){ 1.0f, 9.0f };
	run->tangent = (b2Vec2){ 0.0f, -1.0f };
	run->normal = (b2Vec2){ 1.0f, 0.0f };
	run->restHeight = 0.5f;
	run->measureStart = 0.3f;
	run->pathLength = 7.5f;
	run->fallDepth = 0.25f;
}

// A5: thin walls at x = 6 standing on box tiles: a two-sided segment and a one-sided chain wall
// (solid side facing -x), approached from the left (x = 0, moving +x) or from the right (x = 12,
// moving -x). Correct: blocked 0.5 m before the wall in every case except the one-sided chain
// approached from its back (variant 3), which is open by design and must pass like `none`.
static void CreateThinWalls( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, floor_boxTiles, -4.0f, 10, 2.0f, 0.0f, 0.0f, 0.0f );
	int variant = run->params.variant;
	bool fromRight = ( variant & 1 ) != 0;
	if ( variant < 2 )
	{
		b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
		b2Segment segment = { { 6.0f, 0.0f }, { 6.0f, 1.5f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}
	else
	{
		// Right of (6, 1.5) -> (6, 0) is -x
		AddChainSegment( groundId, (b2Vec2){ 6.0f, 2.5f }, (b2Vec2){ 6.0f, 1.5f }, (b2Vec2){ 6.0f, 0.0f }, (b2Vec2){ 6.0f, -1.0f },
						 0.0f );
	}

	b2Vec2 start = fromRight ? (b2Vec2){ 12.0f, 0.5f } : (b2Vec2){ 0.0f, 0.5f };
	PlayerDef def = DefaultPlayerDef( run, start );
	def.velocity = (b2Vec2){ fromRight ? -run->params.speed : run->params.speed, 0.0f };
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 9.0f, 0.5f );
	run->origin = (b2Vec2){ start.x, 0.0f };
	if ( fromRight )
	{
		run->tangent = (b2Vec2){ -1.0f, 0.0f };
	}
}

static void StepCrateByWall( GhostRun* run )
{
	if ( run->params.variant == 1 )
	{
		b2Body_SetLinearVelocity( run->auxBodyIds[0], (b2Vec2){ run->params.knob, 0.0f } );
	}
	else
	{
		PushPlayer( run, (b2Vec2){ 1.0f, 0.0f }, 20.0f );
	}
}

// A6: v0: the player stands on a dynamic 1 m crate resting flush against a static wall at x = 10
// and is held pressing right. v1: the player rides a kinematic platform (top y = 0, speed knob)
// that slides under a static wall whose bottom is flush with the platform top at x = 10 (the wall
// scrapes the player off). Correct: x stops at 9.5 and the player stays on top (no sink into the
// crate/platform, no pass through the wall).
static void CreateCrateByWall( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	const float friction = 0.6f;
	float topY;
	if ( run->params.variant == 0 )
	{
		BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 8, 2.0f, 0.0f, 0.0f, friction );
		AddStaticBox( groundId, 10.0f, 0.0f, 11.0f, 3.0f, friction );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 9.5f, 0.5f };
		b2BodyId crateId = b2CreateBody( run->worldId, &bodyDef );
		b2ShapeDef crateDef = FloorShapeDef( friction );
		crateDef.density = 1.0f;
		b2Polygon crate = b2MakeBox( 0.5f, 0.5f );
		b2CreatePolygonShape( crateId, &crateDef, &crate );
		topY = 1.0f;
	}
	else
	{
		AddStaticBox( groundId, 10.0f, 0.0f, 11.0f, 2.0f, friction );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = (b2Pos){ 6.0f, -0.25f };
		b2BodyId platformId = b2CreateBody( run->worldId, &bodyDef );
		b2ShapeDef shapeDef = FloorShapeDef( friction );
		b2Polygon tile = b2MakeBox( 3.0f, 0.25f );
		b2CreatePolygonShape( platformId, &shapeDef, &tile );
		run->auxBodyIds[0] = platformId;
		run->auxOrigins[0] = (b2Vec2){ 6.0f, -0.25f };
		run->auxBodyCount = 1;
		topY = 0.0f;
	}

	float startX = run->params.variant == 0 ? 9.3f : 7.0f;
	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ startX, topY + 0.5f } );
	def.velocity = (b2Vec2){ run->params.variant == 0 ? 1.0f : run->params.knob, 0.0f };
	def.friction = friction;
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	// Falling once the platform has passed under the wall is correct, so no fall threshold
	SetFlatFrame( run, topY + 0.5f, -10.0f, 10.5f, 100.0f );
}

static void StepElevatorFlush( GhostRun* run )
{
	// Rise (even variants) from top y = -1 to +1, or sink (odd variants) from 0 to -2, then stop
	bool rise = ( run->params.variant & 1 ) == 0;
	b2BodyId bodyId = run->auxBodyIds[0];
	float y = ToVec2( b2Body_GetPosition( bodyId ) ).y + 0.25f;
	float v = run->params.knob;
	if ( rise )
	{
		v = y < 1.0f ? b2MinFloat( v, ( 1.0f - y ) / GHOST_TIME_STEP ) : 0.0f;
	}
	else
	{
		v = y > -2.0f ? -b2MinFloat( v, ( y + 2.0f ) / GHOST_TIME_STEP ) : 0.0f;
	}
	b2Body_SetLinearVelocity( bodyId, (b2Vec2){ 0.0f, v } );
}

// A7: a 2 m kinematic elevator in a shaft at x in [4, 6] of a floor at y = 0, speed knob.
// Rise variants: the player stands still on the floor at x = 3.8 overhanging the shaft by 0.3 m;
// the elevator comes up from below and must lift it to y = 1.5 (no sink into the elevator top).
// Sink variants: the player stands at x = 4.2 with 0.3 m on the floor and its centroid over the
// flush elevator, which then drops away; it must stay on the floor edge at y = 0.5 (no sink into
// the floor). The frame is vertical: hop/sink are relative to y = 0.5.
static void CreateElevatorFlush( GhostRun* run )
{
	int variant = run->params.variant;
	bool rise = ( variant & 1 ) == 0;
	FloorKind kind = variant < 2 ? floor_boxTiles : floor_chain;
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, kind, -4.0f, 4, 2.0f, 0.0f, 0.0f, 0.0f );
	BuildFlatFloor( groundId, kind, 6.0f, 4, 2.0f, 0.0f, 0.0f, 0.0f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_kinematicBody;
	float top = rise ? -1.0f : 0.0f;
	bodyDef.position = (b2Pos){ 5.0f, top - 0.25f };
	b2BodyId platformId = b2CreateBody( run->worldId, &bodyDef );
	b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
	b2Polygon tile = b2MakeBox( 1.0f, 0.25f );
	b2CreatePolygonShape( platformId, &shapeDef, &tile );
	run->auxBodyIds[0] = platformId;
	run->auxOrigins[0] = (b2Vec2){ 5.0f, top - 0.25f };
	run->auxBodyCount = 1;

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ rise ? 3.8f : 4.2f, 0.5f } );
	def.velocity = b2Vec2_zero;
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, -10.0f, 100.0f, 0.25f );
}

// A8: tiled ceiling (bottom at y = 2, 2 m tiles, seams) over a box tile floor. v0/v1: gravity is
// inverted (gravity scale -g) so the player slides along the ceiling at 10 m/s (frame: normal
// -y, rest at y = 1.5). v2/v3: normal gravity, the player jumps from the floor with 12 m/s up
// into the ceiling and bounces along (rest height is the floor, hop must stay <= 1.0: above that
// the player went into the ceiling). Correct: no snags at ceiling seams, no pass through.
static void CreateTileCeiling( GhostRun* run )
{
	int variant = run->params.variant;
	bool chain = ( variant & 1 ) != 0;
	bool jump = variant >= 2;
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, floor_boxTiles, -2.0f, 24, 2.0f, 0.0f, 0.0f, 0.0f );
	if ( chain )
	{
		// Left to right: outside (right of travel) is -y
		b2Vec2 points[25];
		for ( int i = 0; i <= 24; ++i )
		{
			points[i] = (b2Vec2){ -2.0f + 2.0f * i, 2.0f };
		}
		for ( int i = 0; i < 24; ++i )
		{
			b2Vec2 ghost1 = i > 0 ? points[i - 1] : (b2Vec2){ -4.0f, 2.0f };
			b2Vec2 ghost2 = i + 2 <= 24 ? points[i + 2] : (b2Vec2){ 48.0f, 2.0f };
			AddChainSegment( groundId, ghost1, points[i], points[i + 1], ghost2, 0.0f );
		}
	}
	else
	{
		AddTiledBlock( groundId, -2.0f, 2.0f, 46.0f, 2.5f, 24, 1, 0.0f );
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, jump ? 0.5f : 1.5f } );
	if ( jump )
	{
		def.velocity = (b2Vec2){ run->params.speed, 12.0f };
		SetFlatFrame( run, 0.5f, 0.5f, 40.0f, 0.5f );
	}
	else
	{
		def.gravityScale = -run->params.gravityScale;
		SetFlatFrame( run, 0.5f, 0.5f, 40.0f, 0.5f );
		run->origin = (b2Vec2){ 0.0f, 2.0f };
		run->normal = (b2Vec2){ 0.0f, -1.0f };
	}
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
}

// A9: a vehicle (one body: chassis box plus two wheel circles 3 m apart, rotation free) drives
// over a convex crest at x = 10 onto a 20 degree downslope that ends at a flat floor 2 m lower.
// The body centroid is between the wheels, away from the shapes that touch the ground. Correct:
// the wheels follow the slope and the vehicle ends on the lower floor (y ~ -2 + 0.4); it must
// never sink below the surface (sink is measured against the lower floor line, so any sink is
// a fall through).
static void CreateVehicleCrest( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	float slope = 20.0f * B2_PI / 180.0f;
	float run2 = 2.0f / tanf( slope );
	b2Vec2 crest = { 10.0f, 0.0f };
	b2Vec2 foot = { 10.0f + run2, -2.0f };
	switch ( run->params.variant & 1 )
	{
		case 0:
		{
			b2Vec2 points[] = { { -4.0f, 0.0f }, { 2.0f, 0.0f }, { 6.0f, 0.0f }, crest, b2Lerp( crest, foot, 0.5f ), foot,
								{ foot.x + 4.0f, -2.0f }, { foot.x + 30.0f, -2.0f } };
			AddPolyline( groundId, points, 8, 0.0f );
		}
		break;
		default:
		{
			BuildFlatFloor( groundId, floor_boxTiles, -4.0f, 7, 2.0f, 0.0f, 0.0f, 0.0f );
			// Slope as triangle tiles on top of a block (vertical seams under the slope)
			b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
			int n = 3;
			for ( int i = 0; i < n; ++i )
			{
				b2Vec2 a = b2Lerp( crest, foot, (float)i / n );
				b2Vec2 b = b2Lerp( crest, foot, (float)( i + 1 ) / n );
				b2Vec2 v[4] = { a, b, { b.x, -2.5f }, { a.x, -2.5f } };
				b2Hull hull = b2ComputeHull( v, 4 );
				b2Polygon poly = b2MakePolygon( &hull, 0.0f );
				b2CreatePolygonShape( groundId, &shapeDef, &poly );
			}
			BuildFlatFloor( groundId, floor_boxTiles, foot.x, 16, 2.0f, -2.0f, 0.0f, 0.0f );
		}
		break;
	}

	const float radius = 0.4f;
	if ( run->params.variant >= 2 )
	{
		// Single convex shape: a free plank 3 m long (v2, v3) or a rotation locked 1 m box (v4,
		// v5), centroid inside the shape
		bool plankPlayer = run->params.variant < 4;
		float hy = plankPlayer ? 0.2f : 0.5f;
		PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, hy } );
		def.lockRotation = plankPlayer == false;
		b2Polygon plank = b2MakeBox( plankPlayer ? 1.5f : 0.5f, hy );
		CreatePlayerPolygon( run, &def, &plank );
		SetFlatFrame( run, hy, foot.x + 2.0f, foot.x + 12.0f, 0.15f );
		run->origin = (b2Vec2){ 0.0f, -2.0f };
		return;
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, radius } );
	def.lockRotation = false;
	def.friction = 0.0f;
	b2ShapeDef shapeDef;
	b2BodyId bodyId = CreatePlayerBody( run, &def, &shapeDef );
	b2Circle rear = { { -1.5f, 0.0f }, radius };
	b2Circle front = { { 1.5f, 0.0f }, radius };
	b2Polygon chassis = b2MakeOffsetBox( 2.0f, 0.3f, (b2Vec2){ 0.0f, 0.9f }, b2Rot_identity );
	run->playerShapeId = b2CreateCircleShape( bodyId, &shapeDef, &front );
	b2CreateCircleShape( bodyId, &shapeDef, &rear );
	shapeDef.density = 0.2f;
	b2CreatePolygonShape( bodyId, &shapeDef, &chassis );

	SetFlatFrame( run, radius, 0.5f, foot.x + 12.0f, 0.3f );
	run->origin = (b2Vec2){ 0.0f, -2.0f };
	// Wheel axle height above the lower floor at the end; the upper floor is 2 m higher
	run->restHeight = radius;
	run->measureStart = foot.x + 2.0f;
}

// A10: a staircase going down by knob meters per 2 m tile, under heavy gravity (sweep). Covered
// lower tiles lose their speculative contact while the player is still over the upper tile.
// Correct: no sink below each tile top (sink is measured against the ideal descending line, so
// compare none vs topological; fell = sank > 0.25 m).
static void CreateStepDown( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	float drop = run->params.knob;
	const int count = 24;
	for ( int i = 0; i < count; ++i )
	{
		float xl = -2.0f + 2.0f * i;
		float y = -drop * i;
		if ( run->params.variant == 0 )
		{
			AddStaticBox( groundId, xl, y - 0.5f, xl + 2.0f, y, 0.0f );
		}
		else
		{
			b2Vec2 ghost1 = { xl + 4.0f, y - drop };
			b2Vec2 ghost2 = { xl - 2.0f, y + drop };
			AddChainSegment( groundId, ghost1, (b2Vec2){ xl + 2.0f, y }, (b2Vec2){ xl, y }, ghost2, 0.0f );
		}
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 40.0f, 0.25f );
	// Tangent along the staircase average so hop/sink compare against it
	run->tangent = b2Normalize( (b2Vec2){ 2.0f, -drop } );
	run->normal = b2LeftPerp( run->tangent );
	run->origin = (b2Vec2){ -1.0f, 0.0f };
	run->restHeight = 0.5f * run->tangent.x;
}

// A11: floor tiles separated by small gaps (knob meters). The gap faces are real but cannot be
// reached by a 1 m box riding over a narrow gap. Correct: like a seam, no snag, no sink.
static void CreateSmallGaps( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	float gap = run->params.knob;
	float x = -2.0f;
	for ( int i = 0; i < 20; ++i )
	{
		if ( run->params.variant == 0 )
		{
			AddStaticBox( groundId, x, -0.5f, x + 2.0f, 0.0f, 0.0f );
		}
		else
		{
			b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
			b2Segment segment = { { x + 2.0f, 0.0f }, { x, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}
		x += 2.0f + gap;
	}

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ 0.0f, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 30.0f, 0.5f );
}

// A12: CCD runs its pre-continuous callback with the body already at the END of the step. A fast
// player is launched so that, unblocked, it would end the step resting against the BACK face of a
// thin wall at x = 12 (knob shifts that end pose, in meters). Correct: blocked at x = 11.5.
static void CreateCcdBackFace( GhostRun* run )
{
	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	BuildFlatFloor( groundId, run->params.variant < 2 ? floor_boxTiles : floor_chain, -4.0f, 12, 2.0f, 0.0f, 0.0f, 0.0f );
	float back;
	if ( ( run->params.variant & 1 ) == 0 )
	{
		AddStaticBox( groundId, 12.0f, 0.0f, 12.1f, 1.5f, 0.0f );
		back = 12.1f;
	}
	else
	{
		b2ShapeDef shapeDef = FloorShapeDef( 0.0f );
		b2Segment segment = { { 12.0f, 0.0f }, { 12.0f, 1.5f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
		back = 12.0f;
	}

	// Free flight end pose after n steps lands the left face on the back face
	float travel = run->params.speed * GHOST_TIME_STEP;
	float end = back + 0.5f + run->params.knob;
	int n = (int)( end / travel );
	float startX = end - n * travel;

	PlayerDef def = DefaultPlayerDef( run, (b2Vec2){ startX, 0.5f } );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	CreatePlayerPolygon( run, &def, &box );
	SetFlatFrame( run, 0.5f, 0.5f, 13.0f, 0.5f );
}

// Parabola Crates: a rolling circle crosses a band of crates lying on the curved floor
// y = x^2 / 8. The band between the floor and the floor raised by PARABOLA_THICKNESS (measured
// vertically) is sliced by slanted cuts of uneven spacing and skew into general convex
// quadrilaterals: two vertices on the floor curve, two on the top curve, straight edges. Adjacent
// crates share their cut edge exactly. The floor polyline uses the crate bottom vertices, so every
// crate rests flush on one floor segment.
#define PARABOLA_CRATE_COUNT 14
#define PARABOLA_THICKNESS 0.4f
#define PARABOLA_FRICTION 1.0f

static float ParabolaFloor( float x )
{
	return 0.125f * x * x;
}

// Curvature of the floor and of the crate tops (the floor shifted up)
static float ParabolaCurvature( float x )
{
	float slope = 0.25f * x;
	float s = sqrtf( 1.0f + slope * slope );
	return 0.25f / ( s * s * s );
}

// Measure against the crate top polyline. Expected speed: energy conservation for a rolling disk
// (kinetic energy 3/4 m v^2) relative to the launch point, which ignores the geometric loss at the
// polyline corners (compare against the smooth chain variant).
static GhostLocalFrame ParabolaFrame( const GhostRun* run, b2Vec2 position )
{
	GhostLocalFrame frame = GhostPolylineFrame( run, position );
	float g = -b2World_GetGravity( run->worldId ).y * run->params.gravityScale;
	float v0 = run->params.speed;
	float v2 = v0 * v0 + ( 4.0f / 3.0f ) * g * ( run->origin.y - position.y );
	frame.expectedSpeed = b2MaxFloat( sqrtf( b2MaxFloat( v2, 0.0f ) ), 0.5f );
	return frame;
}

// Builds the parabola floor, rim walls and the crate band (kind 0: dynamic crates, 1: static
// crates, 2: smooth reference polyline along the crate tops). Sets the run's surface polyline to
// the crate tops and returns the arc length from the start of the first crate top to the start of
// the last one, where a run is finished.
static float BuildParabolaCrates( GhostRun* run, int kind, float friction )
{
	// Relative crate widths and cut skews (top x - bottom x), deterministic and uneven
	static const float widths[PARABOLA_CRATE_COUNT] = { 1.0f, 1.3f, 0.8f, 1.15f, 0.9f, 1.25f, 0.85f,
														 1.1f, 1.35f, 0.95f, 1.2f, 0.8f, 1.05f, 1.3f };
	static const float skews[PARABOLA_CRATE_COUNT + 1] = { 0.06f,  -0.08f, 0.1f,  -0.05f, 0.09f, -0.1f, 0.04f, 0.12f,
														   -0.07f, 0.08f,  -0.09f, 0.05f, -0.06f, 0.1f,  -0.04f };
	const float x0 = -3.5f;
	const float span = 7.0f;

	float widthSum = 0.0f;
	for ( int i = 0; i < PARABOLA_CRATE_COUNT; ++i )
	{
		widthSum += widths[i];
	}

	b2Vec2 bottom[PARABOLA_CRATE_COUNT + 1];
	b2Vec2 top[PARABOLA_CRATE_COUNT + 1];
	float xb = x0;
	for ( int i = 0; i <= PARABOLA_CRATE_COUNT; ++i )
	{
		float xt = xb + skews[i];
		bottom[i] = (b2Vec2){ xb, ParabolaFloor( xb ) };
		top[i] = (b2Vec2){ xt, ParabolaFloor( xt ) + PARABOLA_THICKNESS };
		if ( i < PARABOLA_CRATE_COUNT )
		{
			xb += widths[i] * span / widthSum;
		}
	}

	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	AddPolyline( groundId, bottom, PARABOLA_CRATE_COUNT + 1, friction );

	// Rim walls clear of the crates keep the ball in the bowl after it finishes (sample only, the
	// probe stops at the path end before reaching them)
	{
		b2ShapeDef wallDef = b2DefaultShapeDef();
		b2Polygon left = b2MakeOffsetBox( 0.1f, 1.0f, (b2Vec2){ x0 - 0.2f, ParabolaFloor( x0 ) + 1.0f }, b2Rot_identity );
		b2Polygon right = b2MakeOffsetBox( 0.1f, 1.0f, (b2Vec2){ -x0 + 0.2f, ParabolaFloor( x0 ) + 1.0f }, b2Rot_identity );
		b2CreatePolygonShape( groundId, &wallDef, &left );
		b2CreatePolygonShape( groundId, &wallDef, &right );
	}

	if ( kind == 2 )
	{
		// Reference: the crate top surface as one chain-like polyline, no crates
		AddPolyline( groundId, top, PARABOLA_CRATE_COUNT + 1, friction );
	}
	else
	{
		for ( int i = 0; i < PARABOLA_CRATE_COUNT; ++i )
		{
			b2Vec2 quad[4] = { bottom[i], bottom[i + 1], top[i + 1], top[i] };
			b2Hull hull = b2ComputeHull( quad, 4 );
			B2_ASSERT( hull.count == 4 );

			b2ShapeDef shapeDef = FloorShapeDef( friction );
			if ( kind == 0 )
			{
				// Dynamic crate whose body origin is its first bottom vertex
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = ToPos( bottom[i] );
				bodyDef.gravityScale = run->params.gravityScale;
				b2BodyId crateId = b2CreateBody( run->worldId, &bodyDef );

				for ( int j = 0; j < 4; ++j )
				{
					quad[j] = b2Sub( quad[j], bottom[i] );
				}
				hull = b2ComputeHull( quad, 4 );
				shapeDef.density = 1.0f;
				b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
				b2CreatePolygonShape( crateId, &shapeDef, &polygon );
			}
			else
			{
				b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
				b2CreatePolygonShape( groundId, &shapeDef, &polygon );
			}
		}
	}

	B2_ASSERT( PARABOLA_CRATE_COUNT + 1 <= GHOST_MAX_SURFACE_POINTS );
	float arcLength = 0.0f;
	for ( int i = 0; i <= PARABOLA_CRATE_COUNT; ++i )
	{
		run->surfacePoints[i] = top[i];
		if ( 0 < i && i < PARABOLA_CRATE_COUNT )
		{
			arcLength += b2Distance( top[i - 1], top[i] );
		}
	}
	run->surfacePointCount = PARABOLA_CRATE_COUNT + 1;
	return arcLength;
}

static void CreateParabolaCrates( GhostRun* run )
{
	float arcLength = BuildParabolaCrates( run, run->params.variant, PARABOLA_FRICTION );
	const b2Vec2* top = run->surfacePoints;

	// Rolling ball launched down the local tangent of the first crate top
	const float radius = 0.25f;
	b2Vec2 tangent = b2Normalize( b2Sub( top[1], top[0] ) );
	b2Vec2 start = b2MulAdd( b2Lerp( top[0], top[1], 0.5f ), radius, b2LeftPerp( tangent ) );
	PlayerDef def = DefaultPlayerDef( run, start );
	def.velocity = b2MulSV( run->params.speed, tangent );
	def.angularVelocity = -run->params.speed / radius;
	def.lockRotation = false;
	def.friction = 0.6f;
	CreatePlayerCircle( run, &def, radius );

	// Finished at the start of the last crate top, on the far rim (x ~ 3)
	SetFlatFrame( run, radius, 0.5f, arcLength, radius );
	run->origin = start;
	run->localFrame = ParabolaFrame;
}

// Shape Parade: one body of each shape (circle, capsule, triangle, pentagon, hexagon, box) crosses
// the same surface, each protected by the selected mitigation and measured separately (see
// GhostRun::players). The bodies start at the same point, never collide with each other and are
// launched PARADE_LAUNCH_INTERVAL steps apart, so a dynamic surface carries one body at a time.
// A body is disabled once it finishes, so it cannot roll back into the path of later bodies.
// The circle rolls; the other shapes slide, driven by a force that cancels their sliding friction
// (StepShapeParade), so on a smooth surface they keep their speed like the circle. Players are
// measured at their lowest point (see GhostRun::players). The probe's --players mode judges each
// body against the same body on the smooth reference surface (ShapeParadeReference).
#define PARADE_FRICTION 0.6f

// Mixed with the body friction: sqrt( 0.6 * 0.4 ) = 0.49, below the tipping ratio of every parade
// polygon (half face / inradius = tan( pi / n ): hexagon 0.577), so driven polygons slide instead of
// tumbling on a smooth surface. Tumbling is real motion that reads as hops and kicks.
#define PARADE_SURFACE_FRICTION 0.4f
#define PARADE_LAUNCH_INTERVAL 30
#define PARADE_POLYGON_RADIUS 0.3f

typedef enum ParadeSurface
{
	parade_parabolaDynamic,
	parade_parabolaStatic,
	parade_parabolaChain,
	parade_slopeTiles,
	parade_slopeChain,
	parade_crateFloor,
	parade_flatChain,
} ParadeSurface;

// Support point of a shape: its farthest point along direction (pass -normal for the lowest point)
static b2Vec2 ShapeSupportPoint( b2ShapeId shapeId, b2Transform xf, b2Vec2 direction )
{
	switch ( b2Shape_GetType( shapeId ) )
	{
		case b2_circleShape:
		{
			b2Circle circle = b2Shape_GetCircle( shapeId );
			return b2MulAdd( b2TransformPoint( xf, circle.center ), circle.radius, direction );
		}

		case b2_capsuleShape:
		{
			b2Capsule capsule = b2Shape_GetCapsule( shapeId );
			b2Vec2 p1 = b2TransformPoint( xf, capsule.center1 );
			b2Vec2 p2 = b2TransformPoint( xf, capsule.center2 );
			b2Vec2 p = b2Dot( p1, direction ) >= b2Dot( p2, direction ) ? p1 : p2;
			return b2MulAdd( p, capsule.radius, direction );
		}

		case b2_polygonShape:
		{
			b2Polygon polygon = b2Shape_GetPolygon( shapeId );
			b2Vec2 best = b2TransformPoint( xf, polygon.vertices[0] );
			for ( int i = 1; i < polygon.count; ++i )
			{
				b2Vec2 p = b2TransformPoint( xf, polygon.vertices[i] );
				if ( b2Dot( p, direction ) > b2Dot( best, direction ) )
				{
					best = p;
				}
			}
			return b2MulAdd( best, polygon.radius, direction );
		}

		default:
			return xf.p;
	}
}

// Regular polygon with circumradius r lying on a face (bottom face parallel to the x axis)
static b2Polygon MakeRegularPolygon( int count, float r )
{
	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	for ( int i = 0; i < count; ++i )
	{
		float angle = -0.5f * B2_PI + B2_PI / count + 2.0f * B2_PI * i / count;
		points[i] = (b2Vec2){ r * cosf( angle ), r * sinf( angle ) };
	}
	b2Hull hull = b2ComputeHull( points, count );
	return b2MakePolygon( &hull, 0.0f );
}

// Adds a player lying on the surface at surfacePoint (tangent = direction of travel, the surface
// normal is its left perpendicular). restHeight is the center height when lying on a face (radius
// for round shapes, inradius for polygons).
static void AddParadePlayer( GhostRun* run, const char* label, b2ShapeType type, const void* geometry, float restHeight,
							 b2Vec2 surfacePoint, b2Vec2 tangent )
{
	B2_ASSERT( run->playerCount < GHOST_MAX_PLAYERS );
	int index = run->playerCount;
	GhostPlayer* player = run->players + index;
	run->playerCount += 1;

	player->label = label;
	player->restHeight = restHeight;
	player->launchStep = index * PARADE_LAUNCH_INTERVAL;
	player->launchPosition = b2MulAdd( surfacePoint, restHeight, b2LeftPerp( tangent ) );
	player->launchVelocity = b2MulSV( run->params.speed, tangent );

	// The circle starts rolling, the others start sliding (they may still tip over and tumble)
	player->launchAngularVelocity = type == b2_circleShape ? -run->params.speed / restHeight : 0.0f;

	bool mitigated = run->params.mitigation != ghostMitigation_none;
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = ToPos( player->launchPosition );
	bodyDef.rotation = b2MakeRot( b2Atan2( tangent.y, tangent.x ) );
	bodyDef.gravityScale = run->params.gravityScale;
	bodyDef.enableSleep = false;
	bodyDef.isEnabled = player->launchStep == 0;
	bodyDef.linearVelocity = player->launchVelocity;
	bodyDef.angularVelocity = player->launchAngularVelocity;
	if ( mitigated )
	{
		// Pre-solve must see a fresh manifold at every seam
		bodyDef.enableContactRecycling = false;
	}
	player->bodyId = b2CreateBody( run->worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = PARADE_FRICTION;
	shapeDef.filter.groupIndex = -1; // players never collide with each other
	shapeDef.enablePreSolveEvents = mitigated;
	switch ( type )
	{
		case b2_circleShape:
			player->shapeId = b2CreateCircleShape( player->bodyId, &shapeDef, geometry );
			break;
		case b2_capsuleShape:
			player->shapeId = b2CreateCapsuleShape( player->bodyId, &shapeDef, geometry );
			break;
		default:
			player->shapeId = b2CreatePolygonShape( player->bodyId, &shapeDef, geometry );
			break;
	}

	float mass = b2Body_GetMass( player->bodyId );
	if ( type == b2_circleShape )
	{
		player->energyFactor = 1.0f + b2Body_GetRotationalInertia( player->bodyId ) / ( mass * restHeight * restHeight );
	}
	else
	{
		// Same mixing rule as Box2D: sqrt( frictionA * frictionB )
		float friction = sqrtf( PARADE_FRICTION * PARADE_SURFACE_FRICTION );
		float gravity = b2Length( b2World_GetGravity( run->worldId ) ) * run->params.gravityScale;
		player->energyFactor = 1.0f;
		player->driveForce = friction * mass * gravity;
	}

	if ( index == 0 )
	{
		run->playerId = player->bodyId;
		run->playerShapeId = player->shapeId;
		player->topo = &run->topo;
		TopoGhost_Init( player->topo, run->worldId, player->bodyId );
	}
	else if ( run->params.mitigation == ghostMitigation_topological )
	{
		// Each player needs its own step topology. Freed in GhostRun_Destroy.
		player->topo = malloc( sizeof( TopoGhostContext ) );
		B2_ASSERT( player->topo != NULL );
		TopoGhost_Init( player->topo, run->worldId, player->bodyId );
	}
}

// The parade bodies, all starting at surfacePoint
static void AddParadePlayers( GhostRun* run, b2Vec2 surfacePoint, b2Vec2 tangent )
{
	const float r = PARADE_POLYGON_RADIUS;

	b2Circle circle = { b2Vec2_zero, 0.25f };
	AddParadePlayer( run, "circle", b2_circleShape, &circle, circle.radius, surfacePoint, tangent );

	b2Capsule capsule = { { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.15f };
	AddParadePlayer( run, "capsule", b2_capsuleShape, &capsule, capsule.radius, surfacePoint, tangent );

	b2Polygon triangle = MakeRegularPolygon( 3, r );
	AddParadePlayer( run, "triangle", b2_polygonShape, &triangle, r * cosf( B2_PI / 3.0f ), surfacePoint, tangent );

	b2Polygon pentagon = MakeRegularPolygon( 5, r );
	AddParadePlayer( run, "pentagon", b2_polygonShape, &pentagon, r * cosf( B2_PI / 5.0f ), surfacePoint, tangent );

	b2Polygon hexagon = MakeRegularPolygon( 6, r );
	AddParadePlayer( run, "hexagon", b2_polygonShape, &hexagon, r * cosf( B2_PI / 6.0f ), surfacePoint, tangent );

	b2Polygon box = b2MakeBox( 0.25f, 0.25f );
	AddParadePlayer( run, "box", b2_polygonShape, &box, 0.25f, surfacePoint, tangent );

	if ( run->params.mitigation != ghostMitigation_none )
	{
		b2World_SetPreSolveCallback( run->worldId, MultiPreSolve, MultiPreContinuous, run );
	}
}

// Parabola frame with the expected speed of the player being measured: energy conservation from
// its launch point with its own energy factor (rolling circle: 3/2; driven sliding shapes: 1, the
// drive cancels their friction). Flat faces still lose speed at every polyline vertex they rock
// across, so compare sliding shapes against the smooth chain variant.
static GhostLocalFrame ParadeParabolaFrame( const GhostRun* run, b2Vec2 position )
{
	GhostLocalFrame frame = GhostPolylineFrame( run, position );
	const GhostPlayer* player = run->players + run->framePlayer;
	float g = -b2World_GetGravity( run->worldId ).y * run->params.gravityScale;
	float v0 = run->params.speed;
	float v2 = v0 * v0 + 2.0f * g * ( player->launchPosition.y - position.y ) / player->energyFactor;
	frame.expectedSpeed = b2MaxFloat( sqrtf( b2MaxFloat( v2, 0.0f ) ), 0.5f );
	return frame;
}

static int ShapeParadeReference( int variant )
{
	switch ( (ParadeSurface)variant )
	{
		case parade_parabolaDynamic:
		case parade_parabolaStatic:
		case parade_parabolaChain:
			return parade_parabolaChain;
		case parade_slopeTiles:
		case parade_slopeChain:
			return parade_slopeChain;
		case parade_crateFloor:
		case parade_flatChain:
			return parade_flatChain;
		default:
			return -1;
	}
}

static void StepShapeParade( GhostRun* run );

static void CreateShapeParade( GhostRun* run )
{
	switch ( (ParadeSurface)run->params.variant )
	{
		case parade_parabolaDynamic:
		case parade_parabolaStatic:
		case parade_parabolaChain:
		{
			float arcLength = BuildParabolaCrates( run, run->params.variant - parade_parabolaDynamic, PARADE_SURFACE_FRICTION );
			const b2Vec2* top = run->surfacePoints;
			b2Vec2 tangent = b2Normalize( b2Sub( top[1], top[0] ) );
			AddParadePlayers( run, b2Lerp( top[0], top[1], 0.5f ), tangent );
			SetFlatFrame( run, 0.0f, 0.5f, arcLength, 0.25f );
			run->localFrame = ParadeParabolaFrame;
		}
		break;

		case parade_slopeTiles:
		case parade_slopeChain:
		{
			BuildSlope( run, run->params.variant == parade_slopeTiles, PARADE_SURFACE_FRICTION );
			b2Vec2 down = { cosf( SLOPE_ANGLE ), -sinf( SLOPE_ANGLE ) };
			AddParadePlayers( run, down, down );
			run->origin = b2Vec2_zero;
			run->tangent = down;
			run->normal = b2LeftPerp( down );
			run->restHeight = 0.0f;
			run->measureStart = 1.5f;
			run->pathLength = 26.0f;
			run->fallDepth = 0.25f;
		}
		break;

		default:
		{
			if ( run->params.variant == parade_crateFloor )
			{
				BuildCrateFloor( run, true, PARADE_SURFACE_FRICTION );
			}
			else
			{
				b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
				BuildFlatFloor( groundId, floor_chain, -2.0f, 20, 2.0f, 0.0f, 0.0f, PARADE_SURFACE_FRICTION );
			}
			AddParadePlayers( run, b2Vec2_zero, (b2Vec2){ 1.0f, 0.0f } );
			SetFlatFrame( run, 0.0f, 0.5f, 30.0f, 0.25f );
		}
		break;
	}
}

// Parabola Hills: Parabola Crates at 3x scale (the ball keeps its size) with static crates and a
// sine wave cut into the band top, so the ball rolls over gentle hills and through valleys. The
// band between the floor y = x^2 / 24 and HillsTop (floor + HILLS_THICKNESS + a cosine) is sliced
// by slanted cuts of uneven spacing into convex polygons: two vertices on the floor, the rest on
// the top curve. Each slice samples the top curve every HILLS_TOP_SPACING where the curve is
// concave down (crests); where it is concave up (valleys) the convex hull keeps one straight top
// edge per slice. Adjacent slices share their cut edge exactly. The ball must stay on the ground:
// the hill curvature is chosen so the smooth reference never lifts off (see HILLS_AMPLITUDE).
#define HILLS_SLICE_COUNT 20
#define HILLS_X0 -10.5f
#define HILLS_SPAN 21.0f
#define HILLS_THICKNESS 1.2f

// Lift-off on a crest needs v^2 * curvature > g * cos( slope ). Rolling down from the rim gives
// v^2 ~ 6 g at the bottom, so the net crest curvature A k^2 - 1/12 must stay well below 1/6.
// A k^2 = 0.177 here (net crest curvature 0.094), valleys are concave up by 0.26.
#define HILLS_AMPLITUDE 0.22f
#define HILLS_WAVELENGTH 7.0f
#define HILLS_TOP_SPACING 0.25f
#define HILLS_FRICTION 0.6f

// Finished this far (arc length) before the end of the far rim
#define HILLS_FINISH_MARGIN 3.0f

// Parade variants: launched at the polyline segment at this x, finished at the second x
#define HILLS_PARADE_START_X -7.0f
#define HILLS_PARADE_END_X 3.0f

typedef enum HillsVariant
{
	hills_staticSlices,
	hills_separateBodies,
	hills_smoothChain,
	hills_paradeSlices,
	hills_paradeChain,
} HillsVariant;

static float HillsFloor( float x )
{
	return x * x / 24.0f;
}

static float HillsTop( float x )
{
	return HillsFloor( x ) + HILLS_THICKNESS + HILLS_AMPLITUDE * cosf( 2.0f * B2_PI * x / HILLS_WAVELENGTH );
}

// Appends the top chain of a slice hull (the vertices that are not bottom corners) to the surface
// polyline, left to right. The first vertex is the previous slice's last one and is skipped.
static void AppendHullTop( GhostRun* run, const b2Hull* hull, b2Vec2 bottomLeft, b2Vec2 bottomRight, bool skipFirst )
{
	b2Vec2 top[B2_MAX_POLYGON_VERTICES];
	int count = 0;
	for ( int i = 0; i < hull->count; ++i )
	{
		b2Vec2 p = hull->points[i];
		if ( b2Distance( p, bottomLeft ) > FLT_EPSILON && b2Distance( p, bottomRight ) > FLT_EPSILON )
		{
			top[count++] = p;
		}
	}

	// Insertion sort by x (the top curve is a function of x)
	for ( int i = 1; i < count; ++i )
	{
		b2Vec2 p = top[i];
		int j = i - 1;
		while ( j >= 0 && top[j].x > p.x )
		{
			top[j + 1] = top[j];
			j -= 1;
		}
		top[j + 1] = p;
	}

	for ( int i = skipFirst ? 1 : 0; i < count; ++i )
	{
		B2_ASSERT( run->surfacePointCount < GHOST_MAX_SURFACE_POINTS );
		run->surfacePoints[run->surfacePointCount++] = top[i];
	}
}

// Every variant is judged against the smooth chain (same top polyline, no seams) with the same bodies
static int ParabolaHillsReference( int variant )
{
	return variant >= hills_paradeSlices ? hills_paradeChain : hills_smoothChain;
}

// Curvature of the top curve (positive: concave up), for the parade drive load
static float HillsCurvature( float x )
{
	float k = 2.0f * B2_PI / HILLS_WAVELENGTH;
	float slope = x / 12.0f - HILLS_AMPLITUDE * k * sinf( k * x );
	float second = 1.0f / 12.0f - HILLS_AMPLITUDE * k * k * cosf( k * x );
	float s = sqrtf( 1.0f + slope * slope );
	return second / ( s * s * s );
}

// Shape Parade frame (energy per player) over the hills, distinct so StepShapeParade uses HillsCurvature
static GhostLocalFrame ParadeHillsFrame( const GhostRun* run, b2Vec2 position )
{
	return ParadeParabolaFrame( run, position );
}

// Builds the floor, rim walls and the sliced band (or only its top as a polyline when smooth,
// separateBodies: one static body per slice) with the given surface friction. Sets the run's surface polyline to the band top and
// returns its arc length.
static float BuildParabolaHills( GhostRun* run, bool smooth, bool separateBodies, float friction )
{
	// Relative slice widths (0.7 to 1.5 m after scaling) and cut skews (top x - bottom x)
	static const float widths[HILLS_SLICE_COUNT] = { 1.0f, 1.4f, 0.75f, 1.15f, 0.85f, 1.3f, 0.7f, 1.05f, 1.45f, 0.9f,
													 1.2f, 0.8f, 1.35f, 0.95f, 1.1f, 0.72f, 1.25f, 0.88f, 1.4f, 1.0f };
	static const float skews[HILLS_SLICE_COUNT + 1] = { 0.18f, -0.24f, 0.3f,	-0.15f, 0.27f, -0.3f, 0.12f,
														0.36f, -0.21f, 0.24f,  -0.27f, 0.15f, -0.18f, 0.3f,
														-0.12f, 0.21f, -0.24f, 0.09f,  -0.3f, 0.18f,  -0.12f };
	float widthSum = 0.0f;
	for ( int i = 0; i < HILLS_SLICE_COUNT; ++i )
	{
		widthSum += widths[i];
	}

	b2Vec2 bottom[HILLS_SLICE_COUNT + 1];
	float topX[HILLS_SLICE_COUNT + 1];
	float xb = HILLS_X0;
	for ( int i = 0; i <= HILLS_SLICE_COUNT; ++i )
	{
		bottom[i] = (b2Vec2){ xb, HillsFloor( xb ) };
		topX[i] = xb + skews[i];
		if ( i < HILLS_SLICE_COUNT )
		{
			xb += widths[i] * HILLS_SPAN / widthSum;
		}
	}

	b2BodyId groundId = CreateGround( run->worldId, b2_staticBody );
	AddPolyline( groundId, bottom, HILLS_SLICE_COUNT + 1, friction );

	// Rim walls clear of the band keep the ball in the bowl (sample only, the probe stops at the path end)
	{
		b2ShapeDef wallDef = b2DefaultShapeDef();
		float y = HillsFloor( HILLS_X0 ) + 3.0f;
		b2Polygon left = b2MakeOffsetBox( 0.3f, 3.0f, (b2Vec2){ HILLS_X0 - 0.6f, y }, b2Rot_identity );
		b2Polygon right = b2MakeOffsetBox( 0.3f, 3.0f, (b2Vec2){ -HILLS_X0 + 0.6f, y }, b2Rot_identity );
		b2CreatePolygonShape( groundId, &wallDef, &left );
		b2CreatePolygonShape( groundId, &wallDef, &right );
	}

	// Slice polygons, also the source of the top polyline for every variant
	run->surfacePointCount = 0;
	for ( int i = 0; i < HILLS_SLICE_COUNT; ++i )
	{
		float x0 = topX[i];
		float x1 = topX[i + 1];
		int segments = (int)ceilf( ( x1 - x0 ) / HILLS_TOP_SPACING );
		segments = b2ClampInt( segments, 1, B2_MAX_POLYGON_VERTICES - 3 );

		b2Vec2 points[B2_MAX_POLYGON_VERTICES];
		int count = 0;
		points[count++] = bottom[i];
		points[count++] = bottom[i + 1];
		for ( int j = 0; j <= segments; ++j )
		{
			float x = x0 + ( x1 - x0 ) * j / segments;
			points[count++] = (b2Vec2){ x, HillsTop( x ) };
		}

		// The hull drops top samples in concave up (valley) parts and keeps the cut vertices
		b2Hull hull = b2ComputeHull( points, count );
		B2_ASSERT( hull.count >= 4 );
		AppendHullTop( run, &hull, bottom[i], bottom[i + 1], i > 0 );

		if ( smooth )
		{
			continue;
		}

		b2BodyId bodyId = groundId;
		if ( separateBodies )
		{
			bodyId = CreateGround( run->worldId, b2_staticBody );
		}

		b2ShapeDef shapeDef = FloorShapeDef( friction );
		b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
	}

	const b2Vec2* top = run->surfacePoints;
	int topCount = run->surfacePointCount;
	B2_ASSERT( top[0].x == topX[0] && top[topCount - 1].x == topX[HILLS_SLICE_COUNT] );
	if ( smooth )
	{
		// Reference: the same top surface as one chain-like polyline, no seams
		AddPolyline( groundId, top, topCount, friction );
	}

	float arcLength = 0.0f;
	for ( int i = 1; i < topCount; ++i )
	{
		arcLength += b2Distance( top[i - 1], top[i] );
	}
	return arcLength;
}

// Variants 0-2: one rolling ball. Variants 3-4: the Shape Parade bodies (see CreateShapeParade),
// each measured at its lowest point, launched PARADE_LAUNCH_INTERVAL steps apart.
static void CreateParabolaHills( GhostRun* run )
{
	const HillsVariant variant = (HillsVariant)run->params.variant;
	bool smooth = variant == hills_smoothChain || variant == hills_paradeChain;
	bool parade = variant >= hills_paradeSlices;
	float friction = parade ? PARADE_SURFACE_FRICTION : HILLS_FRICTION;
	float arcLength = BuildParabolaHills( run, smooth, variant == hills_separateBodies, friction );
	const b2Vec2* top = run->surfacePoints;

	if ( parade )
	{
		// Launched from the crest at x = -7, over the valley, the center hill and into the next
		// valley. From the rim the sliders reach ~8 m/s, tumble over the valley corners and lose too
		// much energy at them to climb the far side, even on the smooth chain.
		int i = 0;
		while ( i + 2 < run->surfacePointCount && top[i + 1].x < HILLS_PARADE_START_X )
		{
			i += 1;
		}
		b2Vec2 tangent = b2Normalize( b2Sub( top[i + 1], top[i] ) );
		b2Vec2 start = b2Lerp( top[i], top[i + 1], 0.5f );
		AddParadePlayers( run, start, tangent );
		GhostLocalFrame startFrame = GhostPolylineFrame( run, start );
		b2Vec2 end = { HILLS_PARADE_END_X, HillsTop( HILLS_PARADE_END_X ) };
		GhostLocalFrame endFrame = GhostPolylineFrame( run, end );
		SetFlatFrame( run, 0.0f, startFrame.progress + 1.5f, endFrame.progress, 0.25f );
		run->localFrame = ParadeHillsFrame;
		return;
	}

	// Rolling ball launched down the local tangent of the first top segment
	const float radius = 0.25f;
	b2Vec2 tangent = b2Normalize( b2Sub( top[1], top[0] ) );
	b2Vec2 start = b2MulAdd( b2Lerp( top[0], top[1], 0.5f ), radius, b2LeftPerp( tangent ) );
	PlayerDef def = DefaultPlayerDef( run, start );
	def.velocity = b2MulSV( run->params.speed, tangent );
	def.angularVelocity = -run->params.speed / radius;
	def.lockRotation = false;
	def.friction = 0.6f;
	CreatePlayerCircle( run, &def, radius );

	SetFlatFrame( run, radius, 1.5f, arcLength - HILLS_FINISH_MARGIN, radius );
	run->origin = start;
	run->localFrame = ParabolaFrame;
}

// clang-format off
const GhostScenario g_ghostScenarios[] = {
	{
		"Sliding Box",
		"Locked box sliding over a seamed floor; gravity scale loads the contact (C1, C2, C10)",
		"unused",
		{ "box tiles", "segment tiles", "custom ghost segments", "chain" }, 4,
		GHOST_MITIGATIONS_ALL, 10.0f, 1.0f, 0.0f,
		ghostSweep_gravityScale, { 1.0f, 10.0f, 40.0f, 100.0f }, 4,
		300, CreateSlidingBox, NULL,
		1, 0, false,
	},
	{
		"Misaligned Tiles",
		"Odd tiles raised by a small step (knob, meters) (C11)",
		"step height",
		{ "box tiles", "custom ghost segments", "loose chain segments" }, 3,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.001f,
		ghostSweep_knob, { 0.001f, 0.005f, 0.015f, 0.03f }, 4,
		300, CreateMisalignedTiles, NULL,
	},
	{
		"Tilted Box",
		"Locked box tilted by knob degrees, sliding on one corner: point-face contacts (C4)",
		"tilt (deg)",
		{ "box tiles", "chain" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 10.0f,
		ghostSweep_knob, { 2.0f, 10.0f, 30.0f }, 3,
		300, CreateTiltedBox, NULL,
	},
	{
		"Capsule",
		"Locked capsule sliding over seams (C5)",
		"unused",
		{ "box tiles", "segment tiles", "chain" }, 3,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_gravityScale, { 1.0f, 10.0f, 40.0f }, 3,
		300, CreateCapsuleSlide, NULL,
	},
	{
		"Thin Plank",
		"Thin plank under heavy forced gravity: deep penetration (C6)",
		"unused",
		{ "chain", "box tiles", "custom ghost segments" }, 3,
		GHOST_MITIGATIONS_BASIC, 5.0f, 2000.0f, 0.0f,
		ghostSweep_gravityScale, { 1000.0f, 2000.0f, 3000.0f, 4000.0f, 5000.0f }, 5,
		300, CreateThinPlank, NULL,
		1, 0, false,
	},
	{
		"Fast Landing",
		"Fast diagonal landing onto 0.5 m tiles: CCD time loss (C7)",
		"unused",
		{ "chain", "box tiles", "single segment" }, 3,
		GHOST_MITIGATIONS_SEMRAU, 40.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 20.0f, 40.0f, 80.0f }, 3,
		240, CreateFastLanding, NULL,
		1, 0, false,
	},
	{
		"Kinematic Seam",
		"Static floor with a flush kinematic tile: no cross-body ghosts (C13)",
		"unused",
		{ "chain + kinematic tile", "box tiles + kinematic tile" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_gravityScale, { 1.0f, 10.0f, 40.0f }, 3,
		300, CreateKinematicSeam, NULL,
	},
	{
		"Dynamic Tiles",
		"Box sliding across dynamic tiles resting side by side (C14)",
		"unused",
		{ "dynamic tiles" }, 1,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_gravityScale, { 1.0f, 10.0f, 40.0f }, 3,
		300, CreateDynamicTiles, NULL,
	},
	{
		"Valve Corner",
		"marblebuilds: fast marble sinks after a downslope and snags an exposed convex corner",
		"start phase",
		{ "local ghost rule", "ghost via valve", "solid under valve", "local ghost rule, kinematic", "ghost via valve, kinematic" }, 5,
		GHOST_MITIGATIONS_BASIC, 35.0f, 2.0f, 0.0f,
		ghostSweep_knob, { 0.0f, 0.2f, 0.4f, 0.6f, 0.8f }, 5,
		120, CreateValveCorner, NULL,
	},
	{
		"Interior Faces",
		"marblebuilds: tiles keep antiparallel interior faces (per-tile loops)",
		"unused",
		{ "per-tile loops", "merged surface" }, 2,
		GHOST_MITIGATIONS_BASIC, 25.0f, 2.0f, 0.0f,
		ghostSweep_speed, { 10.0f, 25.0f, 50.0f, 100.0f }, 4,
		180, CreateInteriorFaces, NULL,
	},
	{
		"Default Ghosts",
		"Circle hits a platform end below its top line; loose segment ghost defaults (knob: 0 = p2 end, 1 = p1 end)",
		"approach side",
		{ "ghosts = endpoints", "collinear ghosts", "convex corner ghosts", "plain segment" }, 4,
		GHOST_MITIGATIONS_BASIC, 5.0f, 0.0f, 0.0f,
		ghostSweep_knob, { 0.0f, 1.0f }, 2,
		120, CreateDefaultGhosts, NULL,
		0, 0, true,
	},
	{
		"Moving Platform",
		"Platformer: 3 flush kinematic tiles sliding back and forth in gaps (knob = platform speed)",
		"platform speed",
		{ "box tiles terrain", "chain terrain", "ball, box tiles terrain", "ball, chain terrain" }, 4,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 1.0f,
		ghostSweep_knob, { 0.0f, 1.0f, 3.0f }, 3,
		300, CreateMovingPlatform, StepMovingPlatform,
	},
	{
		"Elevator Seam",
		"Platformer: 3 kinematic tiles bobbing vertically around flush (knob = amplitude, m); above slop the lip is real",
		"amplitude",
		{ "chain + elevator tile", "box tiles + elevator tile" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.002f,
		ghostSweep_knob, { 0.0f, 0.002f, 0.01f, 0.03f }, 4,
		300, CreateElevatorSeam, StepElevatorSeam,
	},
	{
		"Rotating Platform",
		"Platformer: 3 kinematic tiles rocking about their centers (knob = amplitude, degrees)",
		"amplitude (deg)",
		{ "box tiles", "chain", "ball, box tiles", "ball, chain" }, 4,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.5f,
		ghostSweep_knob, { 0.0f, 0.5f, 2.0f }, 3,
		300, CreateRotatingPlatform, StepRotatingPlatform,
		3, 0, false,
	},
	{
		"Slope Tiles",
		"Platformer: locked box sliding down a 30 degree slope of triangle tiles vs a polyline",
		"unused",
		{ "box, triangle tiles", "box, chain", "ball, triangle tiles", "ball, chain", "free box, triangle tiles", "free box, chain" }, 6,
		GHOST_MITIGATIONS_SEMRAU, 5.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 0.0f, 5.0f, 10.0f }, 3,
		300, CreateSlopeTiles, NULL,
		4, 0, false,
	},
	{
		"Compound Player",
		"Player body made of two boxes side by side: internal seam on the sliding face",
		"unused",
		{ "box tiles", "chain" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_gravityScale, { 1.0f, 10.0f, 40.0f }, 3,
		300, CreateCompoundPlayer, NULL,
	},
	{
		"Crate Floor Circle",
		"Rolling circle with friction over resting dynamic crates vs static box tiles",
		"unused",
		{ "dynamic crates", "static box tiles" }, 2,
		GHOST_MITIGATIONS_BASIC, 10.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 5.0f, 10.0f, 20.0f }, 3,
		600, CreateCrateFloorCircle, NULL,
	},
	{
		"Interior Push-out",
		"Player spawned inside a tile next to a seam (knob = depth): must be pushed up, not through the seam",
		"depth",
		{ "circle", "box" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 1.0f, 1.0f, 0.15f,
		ghostSweep_knob, { 0.05f, 0.15f, 0.3f }, 3,
		240, CreateInteriorPushOut, NULL,
	},
	{
		"Tile Wall",
		"ADV: 1 m wall at x = 12 on a tiled floor, player held pressing right. Correct: blocked at x = 11.5, never finishes (v4 = segment wall)",
		"unused",
		{ "box tiles + wall box", "box tiles + tiled wall", "chain into wall", "thin 0.1 m wall", "segment tiles + segment wall" }, 5,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 10.0f, 40.0f, 80.0f }, 3,
		180, CreateTileWall, StepPushRight,
		0, 0x1f, false,
	},
	{
		"Tile Step Up",
		"ADV: real step up (knob, m) at x = 12. A locked square box cannot climb a riser. Correct: blocked at x = 11.5, never finishes",
		"step height",
		{ "raised box tiles", "stacked step tile", "chain", "segment tiles" }, 4,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.1f,
		ghostSweep_knob, { 0.05f, 0.1f, 0.3f }, 3,
		180, CreateTileStepUp, StepPushRight,
		0, 0xf, false,
	},
	{
		"Cliff Pit",
		"ADV: floor ends at x = 10, 3 m deep pit, far wall at x = 14. Correct: ends in the pit, x <= 13.5, y = -3 + half height, never finishes",
		"unused",
		{ "box tiles", "chain", "tall box, box tiles", "tall box, chain" }, 4,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 5.0f, 10.0f, 20.0f }, 3,
		240, CreateCliffPit, NULL,
		0, 0xf, false,
	},
	{
		"Wall Slide Down",
		"ADV: box pressed into a tiled vertical wall slides down it (progress = descent). Correct: no snags, sink (into wall) ~0, hop ~0",
		"unused",
		{ "box wall tiles", "chain wall", "segment wall tiles" }, 3,
		GHOST_MITIGATIONS_SEMRAU, 2.0f, 1.0f, 0.0f,
		ghostSweep_gravityScale, { 1.0f, 10.0f, 40.0f }, 3,
		240, CreateWallSlideDown, StepPushLeft,
		2, 0, false,
	},
	{
		"Thin Walls",
		"ADV: segment wall / one-sided chain wall at x = 6, from either side. Correct: blocked at 0.5 m from it (never finishes), except v3 (chain back side) passes",
		"unused",
		{ "segment from left", "segment from right", "chain front", "chain back (open)" }, 4,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 10.0f, 40.0f, 80.0f }, 3,
		120, CreateThinWalls, NULL,
		0, 0x7, false,
	},
	{
		"Crate By Wall",
		"ADV: v0 player on a dynamic crate flush with a wall, pushed right; v1 kinematic platform (knob m/s) slides under a wall. Correct: x stops at 9.5 on top",
		"platform speed",
		{ "dynamic crate", "kinematic platform under wall" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 2.0f,
		ghostSweep_knob, { 1.0f, 2.0f, 5.0f }, 3,
		240, CreateCrateByWall, StepCrateByWall,
		0, 0x3, false,
	},
	{
		"Elevator Flush",
		"ADV: elevator (knob m/s) rises under / sinks away from a player straddling its shaft. Correct: rise lifts to y = 1.5 with no sink; sink leaves it at y = 0.5",
		"elevator speed",
		{ "rise, box tiles", "sink, box tiles", "rise, chain", "sink, chain" }, 4,
		GHOST_MITIGATIONS_SEMRAU, 0.0f, 1.0f, 3.0f,
		ghostSweep_knob, { 1.0f, 3.0f, 6.0f, 12.0f }, 4,
		180, CreateElevatorFlush, StepElevatorFlush,
		0, 0, true,
	},
	{
		"Tile Ceiling",
		"ADV: tiled ceiling at y = 2. v0/v1 inverted gravity slide; v2/v3 jump into it. Correct: no snags, no pass through (jump: hop <= 1.0)",
		"unused",
		{ "box ceiling, inverted", "chain ceiling, inverted", "box ceiling, jump", "chain ceiling, jump" }, 4,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.0f,
		ghostSweep_gravityScale, { 1.0f, 10.0f, 40.0f }, 3,
		300, CreateTileCeiling, NULL,
		2, 0, true,
	},
	{
		"Vehicle Crest",
		"ADV: compound vehicle (two wheels + chassis), a free plank or a locked box over a crest onto a 20 deg downslope. Correct: finishes on the lower floor, never fell",
		"unused",
		{ "vehicle, chain", "vehicle, box + slope tiles", "plank, chain", "plank, box + slope tiles", "locked box, chain", "locked box, box + slope tiles" }, 6,
		GHOST_MITIGATIONS_SEMRAU, 8.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 3.0f, 8.0f, 15.0f }, 3,
		360, CreateVehicleCrest, NULL,
		1, 0, false,
	},
	{
		"Step Down",
		"ADV: staircase down by knob m per 2 m tile, gravity scale 40. Correct: no sink beyond none's",
		"drop",
		{ "box tiles", "chain" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 40.0f, 0.02f,
		ghostSweep_knob, { 0.01f, 0.02f, 0.05f, 0.2f }, 4,
		300, CreateStepDown, NULL,
	},
	{
		"Small Gaps",
		"ADV: floor tiles separated by gaps of knob m. Correct: no snag, no sink",
		"gap",
		{ "box tiles", "segment tiles" }, 2,
		GHOST_MITIGATIONS_SEMRAU, 10.0f, 1.0f, 0.05f,
		ghostSweep_knob, { 0.01f, 0.05f, 0.2f }, 3,
		300, CreateSmallGaps, NULL,
		1, 0, false,
	},
	{
		"CCD Back Face",
		"ADV: fast player whose unblocked end pose rests on the back of a thin wall (knob = end pose offset). Correct: blocked at x = 11.5, never finishes",
		"end offset",
		{ "thin box wall, box tiles", "segment wall, box tiles", "thin box wall, chain floor", "segment wall, chain floor" }, 4,
		GHOST_MITIGATIONS_BASIC, 80.0f, 1.0f, 0.0f,
		ghostSweep_knob, { -0.015f, 0.0f, 0.015f, 0.05f, 0.3f }, 5,
		120, CreateCcdBackFace, NULL,
		0, 0xf, false,
	},
	{
		"Parabola Crates",
		"Rolling circle over a band of slanted, unevenly cut quadrilateral crates on the curved floor y = x^2/8 (curved measurement frame)",
		"unused",
		{ "dynamic slices", "static slices", "smooth chain" }, 3,
		GHOST_MITIGATIONS_BASIC, 6.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 3.0f, 6.0f, 10.0f }, 3,
		600, CreateParabolaCrates, NULL,
		0, 0, false,
	},
	{
		"Shape Parade",
		"Circle, capsule, triangle, pentagon, hexagon and box, launched 0.5 s apart over the same surface and measured separately at their lowest point (gap, kick). Compare with the smooth chain variants",
		"unused",
		{ "parabola crates (dynamic)", "parabola crates (static)", "parabola smooth chain", "slope triangle tiles", "slope smooth chain", "dynamic crate floor", "flat smooth chain" }, 7,
		GHOST_MITIGATIONS_BASIC, 6.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 3.0f, 6.0f, 10.0f }, 3,
		900, CreateShapeParade, StepShapeParade,
		3, 0, false, ShapeParadeReference,
	},
	{
		"Parabola Hills",
		"Rolling circle over static slices of a 3x Parabola Crates band whose top is a gentle cosine (hills and valleys). Correct: never leaves the ground (air = airborne steps, lifts = lift-offs)",
		"unused",
		{ "static slices", "static slices, separate bodies", "smooth chain", "parade, static slices", "parade, smooth chain" }, 5,
		GHOST_MITIGATIONS_BASIC, 3.0f, 1.0f, 0.0f,
		ghostSweep_speed, { 1.0f, 3.0f, 5.0f }, 3,
		900, CreateParabolaHills, StepShapeParade,
		0, 0, false, ParabolaHillsReference, true,
	},
};
// clang-format on

const int g_ghostScenarioCount = (int)( sizeof( g_ghostScenarios ) / sizeof( g_ghostScenarios[0] ) );

const char* GhostMitigationName( GhostMitigation mitigation )
{
	switch ( mitigation )
	{
		case ghostMitigation_none:
			return "none";
		case ghostMitigation_semrau:
			return "semrau";
		case ghostMitigation_semrauRecycling:
			return "semrau + recycling";
		case ghostMitigation_topological:
			return "topological";
		default:
			return "?";
	}
}

GhostParams GhostDefaultParams( int scenarioIndex )
{
	const GhostScenario* scenario = g_ghostScenarios + scenarioIndex;
	GhostParams params = { 0 };
	params.variant = scenario->defaultVariant;
	params.mitigation = ghostMitigation_none;
	params.speed = scenario->speed;
	params.gravityScale = scenario->gravityScale;
	params.knob = scenario->knob;
	return params;
}

void GhostRun_Create( GhostRun* run, b2WorldId worldId, int scenarioIndex, GhostParams params )
{
	*run = (GhostRun){ 0 };
	run->worldId = worldId;
	run->scenarioIndex = scenarioIndex;
	run->params = params;
	run->metrics.minSpeedRatio = FLT_MAX;

	g_ghostScenarios[scenarioIndex].create( run );

	if ( run->playerCount > 0 )
	{
		// Players are tracked from their launch, see GhostRun_PreStep
		return;
	}

	b2Vec2 v = b2Body_GetLinearVelocity( run->playerId );
	if ( run->localFrame != NULL )
	{
		b2Vec2 p = ToVec2( b2Body_GetPosition( run->playerId ) );
		GhostLocalFrame frame = run->localFrame( run, p );
		float expected = frame.expectedSpeed > 0.0f ? frame.expectedSpeed : run->params.speed;
		run->lastTangentSpeed = expected > 0.0f ? b2Dot( v, frame.tangent ) * run->params.speed / expected : 0.0f;

		// Progress is absolute along the frame path, so the ideal starts where the player does
		run->metrics.idealProgress = frame.progress;
	}
	else
	{
		run->lastTangentSpeed = b2Dot( v, run->tangent );
	}
}

void GhostRun_Destroy( GhostRun* run )
{
	// Player 0 uses the embedded context
	for ( int i = 1; i < run->playerCount; ++i )
	{
		free( run->players[i].topo );
		run->players[i].topo = NULL;
	}
}

// Measurement frame at the player's center: tangent, normal, progress and the expected speed
typedef struct PlayerFrame
{
	b2Vec2 tangent;
	b2Vec2 normal;
	float progress;
	float expected;
} PlayerFrame;

static PlayerFrame GetPlayerFrame( GhostRun* run, int index, b2Vec2 center )
{
	PlayerFrame f;
	if ( run->localFrame != NULL )
	{
		run->framePlayer = index;
		GhostLocalFrame frame = run->localFrame( run, center );
		f.tangent = frame.tangent;
		f.normal = frame.normal;
		f.progress = frame.progress;
		f.expected = frame.expectedSpeed > 0.0f ? frame.expectedSpeed : run->params.speed;
	}
	else
	{
		f.tangent = run->tangent;
		f.normal = run->normal;
		f.progress = b2Dot( b2Sub( center, run->origin ), run->tangent );
		f.expected = run->params.speed;
	}
	return f;
}

// Drive the sliding parade players along the local surface tangent with the force that cancels
// their sliding friction under the gravity load at the local surface (see GhostPlayer::driveForce)
static void StepShapeParade( GhostRun* run )
{
	b2Vec2 gravity = b2MulSV( run->params.gravityScale, b2World_GetGravity( run->worldId ) );
	float g = b2Length( gravity );
	b2Vec2 up = b2MulSV( -1.0f / g, gravity );
	for ( int i = 0; i < run->playerCount; ++i )
	{
		GhostPlayer* player = run->players + i;
		if ( player->driveForce == 0.0f || player->launched == false || player->metrics.finished )
		{
			continue;
		}

		// Normal load per unit weight: gravity plus the centripetal load on the parabola
		b2Vec2 p = ToVec2( b2Body_GetPosition( player->bodyId ) );
		PlayerFrame f = GetPlayerFrame( run, i, p );
		float load = b2Dot( f.normal, up );
		if ( run->localFrame == ParadeParabolaFrame )
		{
			float vt = b2Dot( b2Body_GetLinearVelocity( player->bodyId ), f.tangent );
			load += vt * vt * ParabolaCurvature( p.x ) / g;
		}
		else if ( run->localFrame == ParadeHillsFrame )
		{
			float vt = b2Dot( b2Body_GetLinearVelocity( player->bodyId ), f.tangent );
			load += vt * vt * HillsCurvature( p.x ) / g;
		}
		load = b2MaxFloat( load, 0.0f );
		b2Body_ApplyForceToCenter( player->bodyId, b2MulSV( player->driveForce * load, f.tangent ), true );
	}
}

// Enable a multi-body player and start tracking it
static void LaunchPlayer( GhostRun* run, int index )
{
	GhostPlayer* player = run->players + index;
	if ( b2Body_IsEnabled( player->bodyId ) == false )
	{
		b2Body_Enable( player->bodyId );
	}
	b2Body_SetLinearVelocity( player->bodyId, player->launchVelocity );
	b2Body_SetAngularVelocity( player->bodyId, player->launchAngularVelocity );
	player->launched = true;

	GhostMetrics* m = &player->metrics;
	*m = (GhostMetrics){ 0 };
	m->minSpeedRatio = FLT_MAX;

	PlayerFrame f = GetPlayerFrame( run, index, player->launchPosition );
	player->lastTangentSpeed =
		f.expected > 0.0f ? b2Dot( player->launchVelocity, f.tangent ) * run->params.speed / f.expected : 0.0f;
	if ( run->localFrame != NULL )
	{
		// Progress is absolute along the frame path, so the ideal starts where the player does
		m->idealProgress = f.progress;
	}
}

void GhostRun_PreStep( GhostRun* run )
{
	run->time = run->metrics.steps * GHOST_TIME_STEP;

	const GhostScenario* scenario = g_ghostScenarios + run->scenarioIndex;
	if ( scenario->step != NULL )
	{
		scenario->step( run );
	}

	if ( run->playerCount > 0 )
	{
		for ( int i = 0; i < run->playerCount; ++i )
		{
			GhostPlayer* player = run->players + i;
			if ( player->launched == false && run->stepCount >= player->launchStep )
			{
				LaunchPlayer( run, i );
			}

			// Finished players are disabled
			bool active = player->launched && player->metrics.finished == false;
			if ( active && run->params.mitigation == ghostMitigation_topological )
			{
				TopoGhost_PreStep( player->topo );
			}
		}
		return;
	}

	if ( run->params.mitigation == ghostMitigation_topological )
	{
		TopoGhost_PreStep( &run->topo );
	}
}

GhostLocalFrame GhostPolylineFrame( const GhostRun* run, b2Vec2 position )
{
	GhostLocalFrame frame = { { 1.0f, 0.0f }, { 0.0f, 1.0f }, 0.0f, 0.0f, 0.0f };
	float bestDistance = FLT_MAX;
	float arcLength = 0.0f;
	for ( int i = 0; i + 1 < run->surfacePointCount; ++i )
	{
		b2Vec2 a = run->surfacePoints[i];
		b2Vec2 b = run->surfacePoints[i + 1];
		b2Vec2 e = b2Sub( b, a );
		float length = b2Length( e );
		if ( length < FLT_EPSILON )
		{
			continue;
		}

		b2Vec2 t = b2MulSV( 1.0f / length, e );
		float u = b2ClampFloat( b2Dot( b2Sub( position, a ), t ), 0.0f, length );
		b2Vec2 q = b2MulAdd( a, u, t );
		float distance = b2Distance( position, q );
		if ( distance < bestDistance )
		{
			b2Vec2 n = b2LeftPerp( t );
			bestDistance = distance;
			frame.tangent = t;
			frame.normal = n;
			frame.progress = arcLength + u;
			frame.height = b2Dot( b2Sub( position, q ), n ) < 0.0f ? -distance : distance;
		}

		arcLength += length;
	}

	return frame;
}

// True when no contact point of the body is within GHOST_AIR_GAP of a surface
static bool PlayerAirborne( b2BodyId bodyId )
{
	b2ContactData contacts[8];
	int contactCount = b2Body_GetContactData( bodyId, contacts, 8 );
	for ( int i = 0; i < contactCount; ++i )
	{
		const b2Manifold* manifold = &contacts[i].manifold;
		for ( int j = 0; j < manifold->pointCount; ++j )
		{
			if ( manifold->points[j].separation <= GHOST_AIR_GAP )
			{
				return false;
			}
		}
	}
	return true;
}

// Same rules as the single player update, measured at the player's support point (see GhostRun::players)
static void UpdatePlayer( GhostRun* run, int index )
{
	GhostPlayer* player = run->players + index;
	GhostMetrics* m = &player->metrics;
	if ( player->launched == false || m->finished )
	{
		return;
	}

	b2Transform xf = { ToVec2( b2Body_GetPosition( player->bodyId ) ), b2Body_GetRotation( player->bodyId ) };
	b2Vec2 v = b2Body_GetLinearVelocity( player->bodyId );
	PlayerFrame f = GetPlayerFrame( run, index, xf.p );

	// Gap and normal velocity of the lowest point of the shape
	b2Vec2 support = ShapeSupportPoint( player->shapeId, xf, b2Neg( f.normal ) );
	float h;
	if ( run->localFrame != NULL )
	{
		h = run->localFrame( run, support ).height;
		m->idealProgress += f.expected * GHOST_TIME_STEP;
	}
	else
	{
		h = b2Dot( b2Sub( support, run->origin ), f.normal );
		m->idealProgress = run->params.speed * GHOST_TIME_STEP * ( m->steps + 1 );
	}
	// Kick: both the center and the lowest point move away from the surface. The lowest point alone
	// rises when a face rocks across a polyline vertex, the center alone when a polygon tumbles.
	float vn = b2MinFloat( b2Dot( v, f.normal ),
						   b2Dot( b2Body_GetWorldPointVelocity( player->bodyId, ToPos( support ) ), f.normal ) );
	float vt = f.expected > 0.0f ? b2Dot( v, f.tangent ) * run->params.speed / f.expected : 0.0f;
	float s = f.progress;

	if ( run->params.mitigation == ghostMitigation_topological )
	{
		m->rejectedCount = player->topo->stats.culled + player->topo->stats.vetoed;
	}

	m->steps += 1;
	m->progress = s;

	if ( s >= run->measureStart )
	{
		m->maxHop = b2MaxFloat( m->maxHop, h );
		m->maxSink = b2MaxFloat( m->maxSink, -h );
		m->maxKick = b2MaxFloat( m->maxKick, vn );

		if ( run->params.speed > 0.0f )
		{
			m->minSpeedRatio = b2MinFloat( m->minSpeedRatio, vt / run->params.speed );
		}

		if ( player->lastTangentSpeed > 0.5f && player->lastTangentSpeed - vt > 0.1f * player->lastTangentSpeed )
		{
			m->snagCount += 1;
		}

		if ( -h > run->fallDepth )
		{
			m->fellThrough = true;
		}

		if ( g_ghostScenarios[run->scenarioIndex].measureAirborne )
		{
			bool airborne = PlayerAirborne( player->bodyId );
			if ( airborne )
			{
				m->airborneSteps += 1;
				if ( player->airborne == false )
				{
					m->liftOffCount += 1;
				}
			}
			player->airborne = airborne;
		}
	}

	if ( s >= run->pathLength )
	{
		m->finished = true;

		// Keep it from rolling back into the path of later players or disturbing dynamic surfaces
		b2Body_Disable( player->bodyId );
	}

	player->lastTangentSpeed = vt;
}

// Worst player, see GhostRun::metrics
static void AggregatePlayers( GhostRun* run )
{
	GhostMetrics a = { 0 };
	a.steps = run->stepCount;
	a.minSpeedRatio = FLT_MAX;
	a.finished = true;
	float worstTime = FLT_MAX;
	for ( int i = 0; i < run->playerCount; ++i )
	{
		const GhostPlayer* player = run->players + i;
		const GhostMetrics* m = &player->metrics;
		a.snagCount += m->snagCount;
		a.rejectedCount += m->rejectedCount;
		a.minSpeedRatio = b2MinFloat( a.minSpeedRatio, m->minSpeedRatio );
		a.maxHop = b2MaxFloat( a.maxHop, m->maxHop );
		a.maxSink = b2MaxFloat( a.maxSink, m->maxSink );
		a.maxKick = b2MaxFloat( a.maxKick, m->maxKick );
		a.finished = a.finished && m->finished;
		a.fellThrough = a.fellThrough || m->fellThrough;
		a.airborneSteps += m->airborneSteps;
		a.liftOffCount += m->liftOffCount;

		if ( player->launched && m->idealProgress > 0.0f && m->progress / m->idealProgress < worstTime )
		{
			worstTime = m->progress / m->idealProgress;
			a.progress = m->progress;
			a.idealProgress = m->idealProgress;
		}
	}
	run->metrics = a;
}

void GhostRun_Update( GhostRun* run )
{
	if ( run->playerCount > 0 )
	{
		run->stepCount += 1;
		for ( int i = 0; i < run->playerCount; ++i )
		{
			UpdatePlayer( run, i );
		}
		AggregatePlayers( run );
		return;
	}

	GhostMetrics* m = &run->metrics;
	b2Vec2 p = b2Sub( ToVec2( b2Body_GetPosition( run->playerId ) ), run->origin );
	b2Vec2 v = b2Body_GetLinearVelocity( run->playerId );

	float s, h, vt, vn;
	if ( run->localFrame != NULL )
	{
		// Curved frame: speeds are scaled to the launch speed by the local expected speed
		GhostLocalFrame frame = run->localFrame( run, ToVec2( b2Body_GetPosition( run->playerId ) ) );
		float expected = frame.expectedSpeed > 0.0f ? frame.expectedSpeed : run->params.speed;
		s = frame.progress;
		h = frame.height - run->restHeight;
		vt = expected > 0.0f ? b2Dot( v, frame.tangent ) * run->params.speed / expected : 0.0f;
		vn = b2Dot( v, frame.normal );
		m->idealProgress += expected * GHOST_TIME_STEP;
	}
	else
	{
		s = b2Dot( p, run->tangent );
		h = b2Dot( p, run->normal ) - run->restHeight;
		vt = b2Dot( v, run->tangent );
		vn = b2Dot( v, run->normal );
		m->idealProgress = run->params.speed * GHOST_TIME_STEP * ( m->steps + 1 );
	}

	if ( run->params.mitigation == ghostMitigation_topological )
	{
		m->rejectedCount = run->topo.stats.culled + run->topo.stats.vetoed;
	}

	m->steps += 1;
	m->progress = s;

	if ( s >= run->measureStart && m->finished == false )
	{
		m->maxHop = b2MaxFloat( m->maxHop, h );
		m->maxSink = b2MaxFloat( m->maxSink, -h );
		m->maxKick = b2MaxFloat( m->maxKick, vn );

		if ( run->params.speed > 0.0f )
		{
			m->minSpeedRatio = b2MinFloat( m->minSpeedRatio, vt / run->params.speed );
		}

		if ( run->lastTangentSpeed > 0.5f && run->lastTangentSpeed - vt > 0.1f * run->lastTangentSpeed )
		{
			m->snagCount += 1;
		}

		if ( -h > run->fallDepth )
		{
			m->fellThrough = true;
		}

		if ( g_ghostScenarios[run->scenarioIndex].measureAirborne )
		{
			bool airborne = PlayerAirborne( run->playerId );
			if ( airborne )
			{
				m->airborneSteps += 1;
				if ( run->airborne == false )
				{
					m->liftOffCount += 1;
				}
			}
			run->airborne = airborne;
		}
	}

	if ( s >= run->pathLength )
	{
		m->finished = true;
	}

	run->lastTangentSpeed = vt;
}
