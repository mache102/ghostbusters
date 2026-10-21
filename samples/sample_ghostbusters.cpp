// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

// Custom (ghostbusters) ghost collision samples. Ported from the pre-v3.2 fork benchmarks.
// These use the b2Custom* ghost vertex API declared in box2d.h.

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <imgui.h>

#include <float.h>
#include <math.h>

// Ball rolling across a row of unchained static boxes. This is the classic ghost collision setup:
// the ball catches the box corners at the junctions and gets launched upward.
class GhostGroundBoxes : public Sample
{
public:
	explicit GhostGroundBoxes( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.0f };
			m_context->camera.zoom = 25.0f;
		}

		m_context->enableSleep = false;
		m_tick = 0;
		m_liftoff_x_val = -HUGE_VALF;
		m_max_height_reached = BALL_START.y;

		// Create ground bodies
		for ( int i = 0; i < GROUND_N; i++ )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_staticBody;
			bodyDef.position = { i * GROUND_W, GROUND_HEIGHT };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeBox( GROUND_W / 2, GROUND_H / 2 );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		// Create ball
		b2BodyDef ballDef = b2DefaultBodyDef();
		ballDef.type = b2_dynamicBody;
		ballDef.position = { BALL_START.x, BALL_START.y };
		m_ballId = b2CreateBody( m_worldId, &ballDef );

		b2ShapeDef ballShapeDef = b2DefaultShapeDef();
		ballShapeDef.density = 1.0f;
		b2Circle circle = { { 0.0f, 0.0f }, BALL_RADIUS };
		b2CreateCircleShape( m_ballId, &ballShapeDef, &circle );

		// Apply initial force
		b2Body_ApplyForceToCenter( m_ballId, { FORCE * ( 0.5f / DT ), 0.0f }, true );
	}

	void Step() override
	{
		Sample::Step();
		m_tick++;

		b2Pos ballPos = b2Body_GetPosition( m_ballId );
		float x = (float)ballPos.x;
		float y = (float)ballPos.y;

		if ( y > m_max_height_reached )
		{
			m_max_height_reached = y;
		}
		if ( m_liftoff_x_val == -HUGE_VALF && y > LIFTOFF_THRESHOLD )
		{
			m_liftoff_x_val = x;
		}

		DrawScreenTextLine( "Tick: %d", m_tick );
		DrawScreenTextLine( "Current pos: (%.2f, %.2f)", x, y );
		DrawScreenTextLine( "Max Height Reached: %.2f", m_max_height_reached );

		if ( m_liftoff_x_val != -HUGE_VALF )
		{
			DrawScreenTextLine( "Liftoff detected at x = %.2f", m_liftoff_x_val );
		}
		else
		{
			DrawScreenTextLine( "Liftoff not yet detected" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostGroundBoxes( context );
	}

	static constexpr int GROUND_N = 200;
	static constexpr float GROUND_W = 1.0f;
	static constexpr float GROUND_H = 1.0f;
	static constexpr float GROUND_HEIGHT = -0.75f;
	static constexpr float BALL_RADIUS = 0.25f;
	static constexpr b2Vec2 BALL_START = { 0.0f, 0.0f };
	static constexpr float FORCE = 50.0f;
	static constexpr float DT = 1.0f / 60.0f;

	static constexpr float LIFTOFF_THRESHOLD = 0.1f; // height above ground to consider 'liftoff'

	b2BodyId m_ballId;
	int m_tick;
	float m_max_height_reached;
	float m_liftoff_x_val; // first x-value the ball took a ghost collision and 'flew' off the ground
};

static int sampleGhostGroundBoxes = RegisterSample( "Ghostbusters", "Ground Boxes", GhostGroundBoxes::Create );

// Ground Boxes, but with unchained plain segment tiles. Toggle custom ghost vertices on the
// segments (b2CustomShape_SetGhostVertices) to compare plain two-sided segment collision against
// the chain-segment collision path.
class GhostGroundSegments : public Sample
{
public:
	explicit GhostGroundSegments( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.0f };
			m_context->camera.zoom = 25.0f;
		}

		m_context->enableSleep = false;
		m_tick = 0;
		m_liftoff_x_val = -HUGE_VALF;
		m_max_height_reached = BALL_START.y;

		b2BodyDef groundDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &groundDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		// Segments run right to left so the chain-segment path treats +y as the solid's outside.
		for ( int i = 0; i < GROUND_N; ++i )
		{
			b2Segment segment = { { ( i + 1 ) * SEG_LEN, SURF_Y }, { i * SEG_LEN, SURF_Y } };
			m_segmentIds[i] = b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		ApplyGhostVertices();

		b2BodyDef ballDef = b2DefaultBodyDef();
		ballDef.type = b2_dynamicBody;
		ballDef.position = { BALL_START.x, BALL_START.y };
		m_ballId = b2CreateBody( m_worldId, &ballDef );

		b2ShapeDef ballShapeDef = b2DefaultShapeDef();
		ballShapeDef.density = 1.0f;
		b2Circle circle = { { 0.0f, 0.0f }, BALL_RADIUS };
		b2CreateCircleShape( m_ballId, &ballShapeDef, &circle );

		b2Body_ApplyForceToCenter( m_ballId, { FORCE * ( 0.5f / DT ), 0.0f }, true );
	}

	void ApplyGhostVertices()
	{
		for ( int i = 0; i < GROUND_N; ++i )
		{
			if ( s_useGhostVertices )
			{
				b2Vec2 ghost1 = { ( i + 2 ) * SEG_LEN, SURF_Y };
				b2Vec2 ghost2 = { ( i - 1 ) * SEG_LEN, SURF_Y };
				b2CustomShape_SetGhostVertices( m_segmentIds[i], ghost1, ghost2 );
			}
			else
			{
				b2CustomShape_ClearGhostVertices( m_segmentIds[i] );
			}
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Checkbox( "Custom Ghost Vertices", &s_useGhostVertices ) )
		{
			ApplyGhostVertices();
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();
		m_tick++;

		b2Pos ballPos = b2Body_GetPosition( m_ballId );
		float x = (float)ballPos.x;
		float y = (float)ballPos.y;

		if ( y > m_max_height_reached )
		{
			m_max_height_reached = y;
		}
		if ( m_liftoff_x_val == -HUGE_VALF && y > LIFTOFF_THRESHOLD )
		{
			m_liftoff_x_val = x;
		}

		DrawScreenTextLine( "Custom ghost vertices: %s", s_useGhostVertices ? "on" : "off" );
		DrawScreenTextLine( "Tick: %d", m_tick );
		DrawScreenTextLine( "Current pos: (%.2f, %.2f)", x, y );
		DrawScreenTextLine( "Max Height Reached: %.2f", m_max_height_reached );

		if ( m_liftoff_x_val != -HUGE_VALF )
		{
			DrawScreenTextLine( "Liftoff detected at x = %.2f", m_liftoff_x_val );
		}
		else
		{
			DrawScreenTextLine( "Liftoff not yet detected" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostGroundSegments( context );
	}

	static constexpr int GROUND_N = 200;
	static constexpr float SEG_LEN = 1.0f;
	static constexpr float SURF_Y = -0.25f;
	static constexpr float BALL_RADIUS = 0.25f;
	static constexpr b2Vec2 BALL_START = { 0.0f, 0.0f };
	static constexpr float FORCE = 50.0f;
	static constexpr float DT = 1.0f / 60.0f;
	static constexpr float LIFTOFF_THRESHOLD = 0.1f;

	static inline bool s_useGhostVertices = true;

	b2ShapeId m_segmentIds[GROUND_N];
	b2BodyId m_ballId;
	int m_tick;
	float m_max_height_reached;
	float m_liftoff_x_val;
};

static int sampleGhostGroundSegments =
	RegisterSample( "Ghostbusters", "Ground Segments", GhostGroundSegments::Create );

class GhostPathHorizontal : public Sample
{
public:
	explicit GhostPathHorizontal( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 95.0f, 6.0f };
			m_context->camera.zoom = 55.0f;
		}

		m_context->enableSleep = false;

		b2World_SetGravity( m_worldId, { 0.0f, -10.0f } );

		CreateHorizontalPath();
		CreateBalls();
	}

	void Step() override
	{
		Sample::Step();

		float farthestX = -FLT_MAX;

		for ( int i = 0; i < m_ballCount; ++i )
		{
			BallState& state = m_balls[i];
			b2Pos position = b2Body_GetPosition( state.bodyId );
			float x = (float)position.x;
			float y = (float)position.y;
			state.maxY = b2MaxFloat( state.maxY, y );
			float positiveDeviation = b2MaxFloat( y - state.startY, 0.0f );
			state.maxDeviation = b2MaxFloat( state.maxDeviation, positiveDeviation );
			// apply linear velocity always, kDownwardVelocity, if not in the air
			if ( positiveDeviation < 0.01f && y > -0.01f )
			{
				b2Body_SetLinearVelocity( state.bodyId, { state.speed, -kDownwardVelocity } );
			}

			farthestX = b2MaxFloat( farthestX, x );
		}

		DrawScreenTextLine( "Ghost Path (Horizontal)" );
		DrawScreenTextLine( "Path segments: %d (alternating restitution 0.0 / 1.0)", kPathSegmentCount );
		for ( int i = 0; i < m_ballCount; ++i )
		{
			DrawScreenTextLine( "speed=%6.2f max positive y dev=%.5f", m_balls[i].speed, m_balls[i].maxDeviation );
		}
		DrawScreenTextLine( "Farthest x: %.2f / %.2f", farthestX, kPathLength );
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostPathHorizontal( context );
	}

private:
	struct BallState
	{
		b2BodyId bodyId = b2_nullBodyId;
		float speed = 0.0f;
		float startY = 0.0f;
		float maxY = 0.0f;
		float maxDeviation = 0.0f;
	};

	void CreateHorizontalPath()
	{
		for ( int i = 0; i < kPathSegmentCount; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_staticBody;
			bodyDef.position = { 1.0f + 2.0f * i, -0.5f };

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.restitution = 0.0f; // ( i & 1 ) ? 1.0f : 0.0f;

			b2Polygon box = b2MakeBox( 1.0f, 0.5f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	void CreateBalls()
	{
		// static const float speeds[] = { 100.0f, 50.0f, 25.0f, 10.0f, 5.0f, 2.0f, 1.0f, 0.5f };

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.0f;

		b2Circle circle = { { 0.0f, 0.0f }, kBallRadius };

		m_ballCount = 8; // (int)( sizeof( speeds ) / sizeof( speeds[0] ) );

		for ( int i = 0; i < m_ballCount; ++i )
		{
			float startX = 41.0f + 4.0f * ( m_ballCount - 1 - i );
			float startY = kBallRadius;
			float speed = 100; // speeds[i];

			bodyDef.position = { startX, startY };
			bodyDef.linearVelocity = { speed, -kDownwardVelocity };
			bodyDef.enableSleep = false;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			m_balls[i].bodyId = bodyId;
			m_balls[i].speed = speed;
			m_balls[i].startY = startY;
			m_balls[i].maxY = startY;
			m_balls[i].maxDeviation = 0.0f;
		}
	}

	static constexpr int kPathSegmentCount = 100;
	static constexpr float kPathLength = 200.0f;
	static constexpr float kBallRadius = 0.25f;
	static constexpr int kMaxBalls = 8;
	static constexpr float kDownwardVelocity = 10.0f;

	BallState m_balls[kMaxBalls];
	int m_ballCount = 0;
};

static int sampleGhostPathHorizontal =
	RegisterSample( "Ghostbusters", "Ghost Path (Horizontal)", GhostPathHorizontal::Create );

class GhostPathDiagonal : public Sample
{
public:
	explicit GhostPathDiagonal( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 70.0f, 70.0f };
			m_context->camera.zoom = 60.0f;
		}

		m_context->enableSleep = false;

		b2Vec2 gravity = { kNormalForce * kInvSqrt2, -kNormalForce * kInvSqrt2 };
		b2World_SetGravity( m_worldId, gravity );

		CreateDiagonalPath();
		CreateBalls();
	}

	void Step() override
	{
		Sample::Step();

		float farthestPathDistance = -FLT_MAX;

		for ( int i = 0; i < m_ballCount; ++i )
		{
			BallState& state = m_balls[i];
			b2Pos position = b2Body_GetPosition( state.bodyId );
			float x = (float)position.x;
			float y = (float)position.y;
			float diagonal = y - x;
			float pathDistance = ( x + y ) * kInvSqrt2;

			float positiveDeviation = b2MaxFloat( diagonal - state.startDiagonal, 0.0f );
			state.maxDeviation = b2MaxFloat( state.maxDeviation, positiveDeviation );
			farthestPathDistance = b2MaxFloat( farthestPathDistance, pathDistance );
		}

		DrawScreenTextLine( "Ghost Path (Diagonal)" );
		DrawScreenTextLine( "Path segments: %d (alternating restitution 0.0 / 1.0)", kPathSegmentCount );
		for ( int i = 0; i < m_ballCount; ++i )
		{
			DrawScreenTextLine( "speed=%6.2f max positive diag dev=%.5f", m_balls[i].speed, m_balls[i].maxDeviation );
		}
		DrawScreenTextLine( "Farthest path distance: %.2f / %.2f", farthestPathDistance, kPathLength );
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostPathDiagonal( context );
	}

private:
	struct BallState
	{
		b2BodyId bodyId = b2_nullBodyId;
		float speed = 0.0f;
		float startDiagonal = 0.0f;
		float maxDeviation = 0.0f;
	};

	void CreateDiagonalPath()
	{
		const float angle = 0.25f * B2_PI;
		const b2Vec2 tangent = { kInvSqrt2, kInvSqrt2 };
		const b2Vec2 normal = { -kInvSqrt2, kInvSqrt2 };

		for ( int i = 0; i < kPathSegmentCount; ++i )
		{
			float pathDistance = 1.0f + 2.0f * i;
			b2Vec2 pathPoint = { pathDistance * tangent.x, pathDistance * tangent.y };
			b2Vec2 center = { pathPoint.x - ( kBallRadius + 0.5f ) * normal.x, pathPoint.y - ( kBallRadius + 0.5f ) * normal.y };

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_staticBody;
			bodyDef.position = { center.x, center.y };
			bodyDef.rotation = b2MakeRot( angle );

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.restitution = ( i & 1 ) ? 1.0f : 0.0f;

			b2Polygon box = b2MakeBox( 1.0f, 0.5f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	void CreateBalls()
	{
		static const float speeds[] = { 100.0f, 50.0f, 25.0f, 10.0f, 5.0f, 2.0f, 1.0f, 0.5f };
		const b2Vec2 tangent = { kInvSqrt2, kInvSqrt2 };

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.enableSleep = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.0f;

		b2Circle circle = { { 0.0f, 0.0f }, kBallRadius };

		m_ballCount = (int)( sizeof( speeds ) / sizeof( speeds[0] ) );

		for ( int i = 0; i < m_ballCount; ++i )
		{
			float startPathDistance = 41.0f + 4.0f * ( m_ballCount - 1 - i );
			float speed = speeds[i];

			b2Vec2 startPosition = { startPathDistance * tangent.x, startPathDistance * tangent.y };
			b2Vec2 linearVelocity = { speed * tangent.x, speed * tangent.y - kDownwardVelocity };

			bodyDef.position = { startPosition.x, startPosition.y };
			bodyDef.linearVelocity = linearVelocity;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			m_balls[i].bodyId = bodyId;
			m_balls[i].speed = speed;
			m_balls[i].startDiagonal = startPosition.y - startPosition.x;
			m_balls[i].maxDeviation = 0.0f;
		}
	}

	static constexpr int kPathSegmentCount = 100;
	static constexpr float kPathLength = 200.0f;
	static constexpr float kBallRadius = 0.25f;
	static constexpr int kMaxBalls = 8;
	static constexpr float kInvSqrt2 = 0.70710678118f;
	static constexpr float kNormalForce = 10.0f;
	static constexpr float kDownwardVelocity = 0.2f;

	BallState m_balls[kMaxBalls];
	int m_ballCount = 0;
};

static int sampleGhostPathDiagonal =
	RegisterSample( "Ghostbusters", "Ghost Path (Diagonal)", GhostPathDiagonal::Create );

class GhostChainSegmentRoll : public Sample
{
public:
	static constexpr int N_MAIN = 80;
	static constexpr float START_SPEED = 1000.0f;
	static constexpr float SEG_LEN = 2.0f;
	static constexpr float DROP = 8.0f;
	static constexpr float BALL_RADIUS = SEG_LEN * 0.15f;

	explicit GhostChainSegmentRoll( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { N_MAIN * SEG_LEN * 0.5f, 2.0f };
			m_context->camera.zoom = 25.0f;
		}

		m_context->enableSleep = false;

		// Total points in the closed loop: N_MAIN+1 horizontal + 1 bottom = N_MAIN+2
		// Segment i: pts[i] -> pts[(i+1) % M]
		const int M = N_MAIN + 2;
		b2Vec2 pts[N_MAIN + 2];

		for ( int i = 0; i <= N_MAIN; ++i )
		{
			pts[i] = { i * SEG_LEN, 0.0f };
		}
		// One vertex below the rightmost point to close the loop
		pts[N_MAIN + 1] = { (float)N_MAIN * SEG_LEN, -DROP };

		b2BodyDef groundDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &groundDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 0.0f;

		// Create chain segment shapes and remember their ids so we can set ghost vertices
		// BTW, the segments need to be created in CCW order to be outwards facing
		b2ShapeId segIds[N_MAIN + 2];
		for ( int i = 0; i < M; ++i )
		{
			b2Segment seg = { pts[( i + 1 ) % M], pts[i] };
			segIds[i] = b2CustomCreateChainSegmentShape( groundId, &shapeDef, &seg );
		}

		// Set ghost vertices: ghost1 = point before segment start, ghost2 = point after segment end
		for ( int i = 0; i < M; ++i )
		{
			b2Vec2 g2 = pts[( i - 1 + M ) % M];
			b2Vec2 g1 = pts[( i + 2 ) % M];
			b2CustomChainSegment_SetGhostVertices( segIds[i], g1, g2 );
		}

		// Dynamic ball starting just above the first segment
		b2BodyDef ballDef = b2DefaultBodyDef();
		ballDef.type = b2_dynamicBody;
		ballDef.position = { BALL_RADIUS, BALL_RADIUS + 0.05f };
		m_ballId = b2CreateBody( m_worldId, &ballDef );

		b2ShapeDef ballShapeDef = b2DefaultShapeDef();
		ballShapeDef.density = 1.0f;
		b2Circle circle = { { 0.0f, 0.0f }, BALL_RADIUS };
		b2CreateCircleShape( m_ballId, &ballShapeDef, &circle );

		// Push the ball to the right
		b2Body_ApplyForceToCenter( m_ballId, { START_SPEED, 0.0f }, true );
	}

	void Step() override
	{
		Sample::Step();

		b2Pos pos = b2Body_GetPosition( m_ballId );
		DrawScreenTextLine( "Ball pos: (%.2f, %.2f)", (float)pos.x, (float)pos.y );
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostChainSegmentRoll( context );
	}

	b2BodyId m_ballId;
};

static int sampleGhostChainSegmentRoll =
	RegisterSample( "Ghostbusters", "Chain Segment Roll", GhostChainSegmentRoll::Create );

// Chain segment version of Ground Boxes: identical ball/force parameters, using
// b2CustomCreateChainSegmentShape and b2CustomChainSegment_SetGhostVertices instead of ground boxes.
// Segments are CCW-wound (top surface goes right to left).
class GhostChainSegmentGround : public Sample
{
public:
	static constexpr int N_MAIN = 200;
	static constexpr float SEG_LEN = 1.0f; // matches GROUND_W in GhostGroundBoxes
	static constexpr float DROP = 8.0f;
	// Surface y matches Ground Boxes top face (GROUND_HEIGHT + GROUND_H/2 = -0.75 + 0.5)
	static constexpr float SURF_Y = -0.25f;
	static constexpr float BALL_RADIUS = 0.25f;
	static constexpr b2Vec2 BALL_START = { 0.0f, 0.0f };
	static constexpr float FORCE = 50.0f;
	static constexpr float DT = 1.0f / 60.0f;
	static constexpr float LIFTOFF_THRESHOLD = 0.1f;

	explicit GhostChainSegmentGround( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.0f };
			m_context->camera.zoom = 25.0f;
		}

		m_context->enableSleep = false;
		m_tick = 0;
		m_liftoff_x_val = -HUGE_VALF;
		m_max_height_reached = BALL_START.y;

		// M total vertices define M segments in the closed CCW loop:
		//   pts[0..N_MAIN] : top surface going WEST (right to left)
		//                     pts[0] = top-right, pts[N_MAIN] = top-left (near ball start)
		//   pts[N_MAIN+1]  : drop vertex below top-left corner
		//   loop closes back to pts[0] via diagonal return segment
		const int M = N_MAIN + 2;
		b2Vec2 pts[N_MAIN + 2];

		for ( int i = 0; i <= N_MAIN; ++i )
		{
			pts[i] = { ( N_MAIN - i ) * SEG_LEN, SURF_Y };
		}
		pts[N_MAIN + 1] = { 0.0f, SURF_Y - DROP };

		b2BodyDef groundDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &groundDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 0.0f;

		b2ShapeId segIds[N_MAIN + 2];
		for ( int i = 0; i < M; ++i )
		{
			b2Segment seg = { pts[i], pts[( i + 1 ) % M] };
			segIds[i] = b2CustomCreateChainSegmentShape( groundId, &shapeDef, &seg );
		}

		for ( int i = 0; i < M; ++i )
		{
			b2Vec2 g1 = pts[( i - 1 + M ) % M];
			b2Vec2 g2 = pts[( i + 2 ) % M];
			b2CustomChainSegment_SetGhostVertices( segIds[i], g1, g2 );
		}

		b2BodyDef ballDef = b2DefaultBodyDef();
		ballDef.type = b2_dynamicBody;
		ballDef.position = { BALL_START.x, BALL_START.y };
		m_ballId = b2CreateBody( m_worldId, &ballDef );

		b2ShapeDef ballShapeDef = b2DefaultShapeDef();
		ballShapeDef.density = 1.0f;
		b2Circle circle = { { 0.0f, 0.0f }, BALL_RADIUS };
		b2CreateCircleShape( m_ballId, &ballShapeDef, &circle );

		b2Body_ApplyForceToCenter( m_ballId, { FORCE * ( 0.5f / DT ), 0.0f }, true );
	}

	void Step() override
	{
		Sample::Step();
		m_tick++;

		b2Pos ballPos = b2Body_GetPosition( m_ballId );
		float x = (float)ballPos.x;
		float y = (float)ballPos.y;

		if ( y > m_max_height_reached )
		{
			m_max_height_reached = y;
		}
		if ( m_liftoff_x_val == -HUGE_VALF && y > LIFTOFF_THRESHOLD )
		{
			m_liftoff_x_val = x;
		}

		DrawScreenTextLine( "Tick: %d", m_tick );
		DrawScreenTextLine( "Current pos: (%.2f, %.2f)", x, y );
		DrawScreenTextLine( "Max Height Reached: %.2f", m_max_height_reached );

		if ( m_liftoff_x_val != -HUGE_VALF )
		{
			DrawScreenTextLine( "Liftoff detected at x = %.2f", m_liftoff_x_val );
		}
		else
		{
			DrawScreenTextLine( "Liftoff not yet detected" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostChainSegmentGround( context );
	}

	b2BodyId m_ballId;
	int m_tick;
	float m_max_height_reached;
	float m_liftoff_x_val;
};

static int sampleGhostChainSegmentGround =
	RegisterSample( "Ghostbusters", "Chain Segment Ground", GhostChainSegmentGround::Create );

// Kinematic variant of Chain Segment Ground: ground is a kinematic body that moves upward at
// constant velocity. Reduced FORCE (5.0) to make ball behavior easier to observe.
class GhostKinematicChainSegment : public Sample
{
public:
	static constexpr int N_MAIN = 200;
	static constexpr float SEG_LEN = 1.0f;
	static constexpr float DROP = 8.0f;
	static constexpr float SURF_Y = -0.25f;
	static constexpr float BALL_RADIUS = 0.25f;
	static constexpr b2Vec2 BALL_START = { 2.0f, 0.0f };
	static constexpr float FORCE = 5.0f;
	static constexpr float DT = 1.0f / 60.0f;
	static constexpr float LIFTOFF_THRESHOLD = 0.1f;
	static constexpr float KINEMATIC_SPEED = 2.0f;

	explicit GhostKinematicChainSegment( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.0f };
			m_context->camera.zoom = 25.0f;
		}

		m_context->enableSleep = false;
		m_tick = 0;
		m_liftoff_x_val = -HUGE_VALF;
		m_max_height_reached = BALL_START.y;

		const int M = N_MAIN + 2;
		b2Vec2 pts[N_MAIN + 2];

		for ( int i = 0; i <= N_MAIN; ++i )
		{
			pts[i] = { ( N_MAIN - i ) * SEG_LEN, SURF_Y };
		}
		pts[N_MAIN + 1] = { 0.0f, SURF_Y - DROP };

		b2BodyDef groundDef = b2DefaultBodyDef();
		groundDef.type = b2_kinematicBody;
		m_groundId = b2CreateBody( m_worldId, &groundDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 0.0f;

		b2ShapeId segIds[N_MAIN + 2];
		for ( int i = 0; i < M; ++i )
		{
			b2Segment seg = { pts[i], pts[( i + 1 ) % M] };
			segIds[i] = b2CustomCreateChainSegmentShape( m_groundId, &shapeDef, &seg );
		}

		for ( int i = 0; i < M; ++i )
		{
			b2Vec2 g1 = pts[( i - 1 + M ) % M];
			b2Vec2 g2 = pts[( i + 2 ) % M];
			b2CustomChainSegment_SetGhostVertices( segIds[i], g1, g2 );
		}

		b2BodyDef ballDef = b2DefaultBodyDef();
		ballDef.type = b2_dynamicBody;
		ballDef.position = { BALL_START.x, BALL_START.y };
		ballDef.isBullet = true; // prevent tunneling at high speed
		m_ballId = b2CreateBody( m_worldId, &ballDef );

		b2ShapeDef ballShapeDef = b2DefaultShapeDef();
		ballShapeDef.density = 1.0f;
		ballShapeDef.material.restitution = 0.0f;
		b2Circle circle = { { 0.0f, 0.0f }, BALL_RADIUS };
		b2CreateCircleShape( m_ballId, &ballShapeDef, &circle );

		b2Body_ApplyForceToCenter( m_ballId, { FORCE * ( 0.5f / DT ), 0.0f }, true );
	}

	void Step() override
	{
		Sample::Step();
		m_tick++;

		// Kinematic ground moves upward at fixed velocity (no acceleration)
		b2Body_SetLinearVelocity( m_groundId, { 0.0f, KINEMATIC_SPEED } );

		b2Pos ballPos = b2Body_GetPosition( m_ballId );
		float x = (float)ballPos.x;
		float y = (float)ballPos.y;

		if ( y > m_max_height_reached )
		{
			m_max_height_reached = y;
		}
		if ( m_liftoff_x_val == -HUGE_VALF && y > LIFTOFF_THRESHOLD )
		{
			m_liftoff_x_val = x;
		}

		DrawScreenTextLine( "Tick: %d", m_tick );
		DrawScreenTextLine( "Current pos: (%.2f, %.2f)", x, y );
		DrawScreenTextLine( "Max Height Reached: %.2f", m_max_height_reached );

		if ( m_liftoff_x_val != -HUGE_VALF )
		{
			DrawScreenTextLine( "Liftoff detected at x = %.2f", m_liftoff_x_val );
		}
		else
		{
			DrawScreenTextLine( "Liftoff not yet detected" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostKinematicChainSegment( context );
	}

	b2BodyId m_ballId;
	b2BodyId m_groundId;
	int m_tick;
	float m_max_height_reached;
	float m_liftoff_x_val;
};

static int sampleGhostKinematicChainSegment =
	RegisterSample( "Ghostbusters", "Kinematic Chain Segment", GhostKinematicChainSegment::Create );
