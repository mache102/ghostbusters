// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

// Tests for the custom (ghostbusters) ghost vertex API.

#include "test_macros.h"

#include "box2d/box2d.h"

#include <stdint.h>

// Chain segments are one-sided: the solid side is to the right of point1 -> point2. A segment
// running right to left has its outside facing +y, so a ball coming up from below passes through
// when collision goes through the chain-segment path and is blocked by a plain two-sided segment.
static const b2Segment backFacingSegment = { { 2.0f, 0.0f }, { -2.0f, 0.0f } };
static const b2Vec2 ghostRight = { 4.0f, 0.0f };
static const b2Vec2 ghostLeft = { -4.0f, 0.0f };

static bool VecEqual( b2Vec2 a, b2Vec2 b )
{
	return a.x == b.x && a.y == b.y;
}

static b2WorldId CreateTestWorld( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2_zero;

	// Continuous treats only b2_chainSegmentShape as one-sided, keep it out of the way
	worldDef.enableContinuous = false;
	return b2CreateWorld( &worldDef );
}

// Launch a ball upward from below the segment and return its final height
static float LaunchBallFromBelow( b2WorldId worldId )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, -1.0f };
	bodyDef.linearVelocity = (b2Vec2){ 0.0f, 4.0f };
	b2BodyId ballId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Circle circle = { { 0.0f, 0.0f }, 0.25f };
	b2CreateCircleShape( ballId, &shapeDef, &circle );

	for ( int i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	return (float)b2Body_GetPosition( ballId ).y;
}

static int CreateChainSegmentTest( void )
{
	b2WorldId worldId = CreateTestWorld();
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	b2ShapeId shapeId = b2CustomCreateChainSegmentShape( groundId, &shapeDef, &backFacingSegment );
	ENSURE( b2Shape_IsValid( shapeId ) );
	ENSURE( b2Shape_GetType( shapeId ) == b2_chainSegmentShape );
	ENSURE( B2_IS_NULL( b2Shape_GetParentChain( shapeId ) ) );

	b2ChainSegment cs = b2Shape_GetChainSegment( shapeId );
	ENSURE( VecEqual( cs.ghost1, backFacingSegment.point1 ) );
	ENSURE( VecEqual( cs.ghost2, backFacingSegment.point2 ) );

	b2CustomChainSegment_SetGhostVertices( shapeId, ghostRight, ghostLeft );
	cs = b2Shape_GetChainSegment( shapeId );
	ENSURE( VecEqual( cs.ghost1, ghostRight ) );
	ENSURE( VecEqual( cs.ghost2, ghostLeft ) );

	b2Vec2 g1 = { 5.0f, 1.0f };
	b2Vec2 g2 = { -5.0f, 1.0f };
	b2CustomChainSegment_SetGhostVertex1( shapeId, g1 );
	cs = b2Shape_GetChainSegment( shapeId );
	ENSURE( VecEqual( cs.ghost1, g1 ) );
	ENSURE( VecEqual( cs.ghost2, ghostLeft ) );

	b2CustomChainSegment_SetGhostVertex2( shapeId, g2 );
	cs = b2Shape_GetChainSegment( shapeId );
	ENSURE( VecEqual( cs.ghost1, g1 ) );
	ENSURE( VecEqual( cs.ghost2, g2 ) );

	b2DestroyWorld( worldId );
	return 0;
}

static int ChainSegmentOneSidedTest( void )
{
	b2WorldId worldId = CreateTestWorld();
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	b2ShapeId shapeId = b2CustomCreateChainSegmentShape( groundId, &shapeDef, &backFacingSegment );
	b2CustomChainSegment_SetGhostVertices( shapeId, ghostRight, ghostLeft );

	float y = LaunchBallFromBelow( worldId );
	ENSURE( y > 1.0f );

	b2DestroyWorld( worldId );
	return 0;
}

static int SegmentGhostVerticesTest( void )
{
	// Plain segment: two-sided, the ball is blocked
	{
		b2WorldId worldId = CreateTestWorld();
		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreateSegmentShape( groundId, &shapeDef, &backFacingSegment );

		float y = LaunchBallFromBelow( worldId );
		ENSURE( y < 0.0f );
		b2DestroyWorld( worldId );
	}

	// Plain segment with custom ghost vertices: routed through the one-sided chain path
	{
		b2WorldId worldId = CreateTestWorld();
		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2ShapeId shapeId = b2CreateSegmentShape( groundId, &shapeDef, &backFacingSegment );
		b2CustomShape_SetGhostVertices( shapeId, ghostRight, ghostLeft );
		ENSURE( b2Shape_GetType( shapeId ) == b2_segmentShape );

		float y = LaunchBallFromBelow( worldId );
		ENSURE( y > 1.0f );
		b2DestroyWorld( worldId );
	}

	// Cleared ghost vertices: back to two-sided
	{
		b2WorldId worldId = CreateTestWorld();
		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2ShapeId shapeId = b2CreateSegmentShape( groundId, &shapeDef, &backFacingSegment );
		b2CustomShape_SetGhostVertices( shapeId, ghostRight, ghostLeft );
		b2CustomShape_ClearGhostVertices( shapeId );

		float y = LaunchBallFromBelow( worldId );
		ENSURE( y < 0.0f );
		b2DestroyWorld( worldId );
	}

	return 0;
}

// Shape slots are recycled without zeroing. A new segment in a recycled slot must not inherit
// custom ghost vertices from the destroyed shape.
static int RecycledSlotTest( void )
{
	b2WorldId worldId = CreateTestWorld();
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	b2ShapeId oldId = b2CreateSegmentShape( groundId, &shapeDef, &backFacingSegment );
	b2CustomShape_SetGhostVertices( oldId, ghostRight, ghostLeft );
	b2DestroyShape( oldId, false );

	b2ShapeId newId = b2CreateSegmentShape( groundId, &shapeDef, &backFacingSegment );
	ENSURE( newId.index1 == oldId.index1 );

	float y = LaunchBallFromBelow( worldId );
	ENSURE( y < 0.0f );

	b2DestroyWorld( worldId );
	return 0;
}

int GhostbustersTest( void )
{
	RUN_SUBTEST( CreateChainSegmentTest );
	RUN_SUBTEST( ChainSegmentOneSidedTest );
	RUN_SUBTEST( SegmentGhostVerticesTest );
	RUN_SUBTEST( RecycledSlotTest );

	return 0;
}
