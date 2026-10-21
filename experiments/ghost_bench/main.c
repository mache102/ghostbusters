// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

// Reproducible cost benchmarks for topological ghosts (shared/topo_ghosts.h).
//
// Each configuration is run with the mitigation off (stock Box2D, no pre-solve), with
// topological ghosts, and where it applies with a chain-shape floor and no mitigation (Box2D's
// own ghost vertices, the ideal reference). The integration is the one a game would write: one
// TopoGhostContext per protected body on the heap, TopoGhost_PreStep before every b2World_Step and
// a dispatch pre-solve callback routing each contact to the context of its protected body.
//
// See README.md for the methodology. Usage:
//   ghost_bench [--repeats R] [--warmup W] [--steps M] [--bench 1,2,...] [--csv file] [--quick]

#if !defined( _WIN32 )
#define _POSIX_C_SOURCE 199309L
#endif

#include "topo_ghosts.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#ifndef GHOST_BENCH_BUILD_TYPE
#define GHOST_BENCH_BUILD_TYPE "unknown"
#endif
#ifndef GHOST_BENCH_C_FLAGS
#define GHOST_BENCH_C_FLAGS "unknown"
#endif

#define TIME_STEP ( 1.0f / 60.0f )
#define SUB_STEPS 4
#define MAX_REPEATS 64

// ---------------------------------------------------------------------------------------------
// Clock

static double NowMicroseconds( void )
{
#if defined( _WIN32 )
	static LARGE_INTEGER frequency;
	if ( frequency.QuadPart == 0 )
	{
		QueryPerformanceFrequency( &frequency );
	}
	LARGE_INTEGER counter;
	QueryPerformanceCounter( &counter );
	return 1e6 * (double)counter.QuadPart / (double)frequency.QuadPart;
#elif defined( CLOCK_MONOTONIC )
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return 1e6 * (double)ts.tv_sec + 1e-3 * (double)ts.tv_nsec;
#else
	// Portable C11 fallback (wall clock)
	struct timespec ts;
	timespec_get( &ts, TIME_UTC );
	return 1e6 * (double)ts.tv_sec + 1e-3 * (double)ts.tv_nsec;
#endif
}

static const char* ClockName( void )
{
#if defined( _WIN32 )
	return "QueryPerformanceCounter";
#elif defined( CLOCK_MONOTONIC )
	return "clock_gettime(CLOCK_MONOTONIC)";
#else
	return "timespec_get(TIME_UTC)";
#endif
}

// ---------------------------------------------------------------------------------------------
// Configurations

typedef enum ProtShape
{
	prot_circle,
	prot_capsule,
	prot_box,
	prot_hexagon,
	prot_alternate, // circle, box, circle, ...
	prot_circleAndBox,
} ProtShape;

typedef enum Mode
{
	mode_off,
	mode_topo,
	mode_chain,
	mode_count
} Mode;

static const char* g_modeNames[mode_count] = { "off", "topo", "chain" };

typedef struct Config
{
	char label[64];
	float tileWidth; // static box tiles of this width, top at y = 0
	float floorStart, floorEnd; // explicit floor extent, automatic when equal
	int bodyCount;
	ProtShape shape;
	float speed;   // driven horizontal speed of the protected bodies
	float spacing; // between protected bodies along x
	int debrisCount;
	bool debrisDynamic;
	float fieldLength;
	int pyramidBase;
	bool enableSleep;
	bool chainReference;
} Config;

static Config DefaultConfig( void )
{
	Config c = { 0 };
	c.tileWidth = 0.5f;
	c.bodyCount = 1;
	c.shape = prot_box;
	c.speed = 5.0f;
	c.spacing = 3.0f;
	c.enableSleep = true;
	c.chainReference = true;
	return c;
}

typedef struct Options
{
	int repeats, warmup, steps;
	unsigned benchMask;
	FILE* csv;
} Options;

// ---------------------------------------------------------------------------------------------
// Scene

typedef struct Scene
{
	b2WorldId worldId;
	b2BodyId* protectedIds;
	TopoGhostContext** contexts;
	int protectedCount;
	b2BodyId* trackedIds; // dynamic bodies, for the checksum
	int trackedCount, trackedCapacity;
	float speed;
	float startX;
} Scene;

static uint32_t g_rng;

static float RandomFloat( float lo, float hi )
{
	// xorshift32, deterministic across platforms
	g_rng ^= g_rng << 13;
	g_rng ^= g_rng >> 17;
	g_rng ^= g_rng << 5;
	return lo + ( hi - lo ) * (float)( g_rng & 0xFFFFFF ) / (float)0xFFFFFF;
}

static void Track( Scene* s, b2BodyId id )
{
	if ( s->trackedCount == s->trackedCapacity )
	{
		s->trackedCapacity = s->trackedCapacity ? 2 * s->trackedCapacity : 256;
		s->trackedIds = realloc( s->trackedIds, (size_t)s->trackedCapacity * sizeof( b2BodyId ) );
		if ( s->trackedIds == NULL )
		{
			fprintf( stderr, "out of memory\n" );
			exit( 1 );
		}
	}
	s->trackedIds[s->trackedCount++] = id;
}

// Static box tiles, one shape each, top surface at y = 0
static void BuildTiles( b2WorldId worldId, float x0, float x1, float width )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	int count = (int)ceilf( ( x1 - x0 ) / width );
	for ( int i = 0; i < count; ++i )
	{
		b2Vec2 center = { x0 + ( i + 0.5f ) * width, -0.25f };
		b2Polygon box = b2MakeOffsetBox( 0.5f * width, 0.25f, center, b2Rot_identity );
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}
}

// The same surface as one chain with a vertex at every tile seam. Points run right to left so
// the one-sided segments face +y.
static void BuildChainFloor( b2WorldId worldId, float x0, float x1, float width )
{
	int count = (int)ceilf( ( x1 - x0 ) / width );
	b2Vec2* points = malloc( (size_t)( count + 1 ) * sizeof( b2Vec2 ) );
	for ( int i = 0; i <= count; ++i )
	{
		points[i] = (b2Vec2){ x0 + ( count - i ) * width, 0.0f };
	}

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.points = points;
	chainDef.pointCount = count + 1;
	chainDef.ghost1 = (b2Vec2){ x0 + ( count + 1 ) * width, 0.0f };
	chainDef.ghost2 = (b2Vec2){ x0 - width, 0.0f };
	b2CreateChain( groundId, &chainDef );
	free( points );
}

static b2Polygon RandomHull( float radius )
{
	for ( ;; )
	{
		b2Vec2 points[8];
		int n = 3 + (int)RandomFloat( 0.0f, 5.99f );
		for ( int i = 0; i < n; ++i )
		{
			float angle = 2.0f * B2_PI * ( i + RandomFloat( 0.0f, 0.8f ) ) / n;
			float r = radius * RandomFloat( 0.6f, 1.0f );
			points[i] = (b2Vec2){ r * cosf( angle ), r * sinf( angle ) };
		}
		b2Hull hull = b2ComputeHull( points, n );
		if ( hull.count >= 3 )
		{
			return b2MakePolygon( &hull, 0.0f );
		}
	}
}

// Small random convex polygons over [x0, x0 + length]: static ones embedded in the floor top
// (gravel, bumps under 0.1 m), dynamic ones stacked loosely on the floor (a debris pile)
static void BuildDebris( Scene* s, const Config* c, float x0 )
{
	g_rng = 0x9E3779B9u;
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	if ( c->debrisDynamic == false )
	{
		b2BodyId groundId = b2CreateBody( s->worldId, &bodyDef );
		for ( int i = 0; i < c->debrisCount; ++i )
		{
			float radius = RandomFloat( 0.04f, 0.12f );
			b2Polygon p = RandomHull( radius );
			b2Vec2 center = { x0 + RandomFloat( 0.0f, c->fieldLength ), RandomFloat( -0.15f, -0.02f ) };
			b2Hull hull = { 0 };
			hull.count = p.count;
			memcpy( hull.points, p.vertices, sizeof( b2Vec2 ) * (size_t)p.count );
			b2Polygon placed = b2MakeOffsetPolygon( &hull, center, b2MakeRot( RandomFloat( -B2_PI, B2_PI ) ) );
			b2CreatePolygonShape( groundId, &shapeDef, &placed );
		}
		return;
	}

	int rows = 4;
	int columns = ( c->debrisCount + rows - 1 ) / rows;
	float pitch = c->fieldLength / columns;
	bodyDef.type = b2_dynamicBody;
	for ( int i = 0; i < c->debrisCount; ++i )
	{
		int column = i / rows, row = i % rows;
		bodyDef.position = (b2Pos){ x0 + ( column + 0.5f ) * pitch, 0.13f + 0.26f * row };
		bodyDef.rotation = b2MakeRot( RandomFloat( -B2_PI, B2_PI ) );
		b2Polygon p = RandomHull( b2MinFloat( 0.11f, 0.45f * pitch ) );
		b2BodyId id = b2CreateBody( s->worldId, &bodyDef );
		b2CreatePolygonShape( id, &shapeDef, &p );
		Track( s, id );
	}
}

// A pyramid of 0.5 m boxes on its own ground, far from the protected bodies
static void BuildPyramid( Scene* s, int base )
{
	float x0 = -400.0f;
	BuildTiles( s->worldId, x0 - 0.5f * base - 10.0f, x0 + 0.5f * base + 10.0f, 1.0f * base + 20.0f );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.25f, 0.25f );
	for ( int row = 0; row < base; ++row )
	{
		int count = base - row;
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position = (b2Pos){ x0 + 0.5f * ( i - 0.5f * ( count - 1 ) ), 0.25f + 0.5f * row };
			b2BodyId id = b2CreateBody( s->worldId, &bodyDef );
			b2CreatePolygonShape( id, &shapeDef, &box );
			Track( s, id );
		}
	}
}

static ProtShape BodyShape( ProtShape shape, int index )
{
	if ( shape == prot_alternate || shape == prot_circleAndBox )
	{
		return ( index & 1 ) ? prot_box : prot_circle;
	}
	return shape;
}

static b2BodyId CreateProtected( Scene* s, ProtShape shape, float x, bool preSolve )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.linearVelocity = (b2Vec2){ s->speed, 0.0f };
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.enablePreSolveEvents = preSolve;

	switch ( shape )
	{
		case prot_circle:
		{
			bodyDef.position = (b2Pos){ x, 0.5f };
			b2BodyId id = b2CreateBody( s->worldId, &bodyDef );
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( id, &shapeDef, &circle );
			return id;
		}
		case prot_capsule:
		{
			bodyDef.position = (b2Pos){ x, 0.25f };
			b2BodyId id = b2CreateBody( s->worldId, &bodyDef );
			b2Capsule capsule = { { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.25f };
			b2CreateCapsuleShape( id, &shapeDef, &capsule );
			return id;
		}
		case prot_hexagon:
		{
			b2Vec2 points[6];
			for ( int i = 0; i < 6; ++i )
			{
				points[i] = (b2Vec2){ 0.5f * cosf( i * B2_PI / 3.0f ), 0.5f * sinf( i * B2_PI / 3.0f ) };
			}
			b2Hull hull = b2ComputeHull( points, 6 );
			b2Polygon hexagon = b2MakePolygon( &hull, 0.0f );
			bodyDef.position = (b2Pos){ x, 0.5f * sinf( B2_PI / 3.0f ) };
			b2BodyId id = b2CreateBody( s->worldId, &bodyDef );
			b2CreatePolygonShape( id, &shapeDef, &hexagon );
			return id;
		}
		default:
		{
			bodyDef.position = (b2Pos){ x, 0.5f };
			b2BodyId id = b2CreateBody( s->worldId, &bodyDef );
			b2Polygon box = b2MakeBox( 0.5f, 0.5f );
			b2CreatePolygonShape( id, &shapeDef, &box );
			return id;
		}
	}
}

// Route each contact to the context of its protected body. Protected bodies carry their context
// as user data, so the dispatch is O(1) whatever the number of protected bodies.
static TopoGhostContext* ContactContext( b2ShapeId shapeIdA, b2ShapeId shapeIdB )
{
	TopoGhostContext* context = b2Body_GetUserData( b2Shape_GetBody( shapeIdA ) );
	return context != NULL ? context : b2Body_GetUserData( b2Shape_GetBody( shapeIdB ) );
}

static void DispatchPreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* userContext )
{
	(void)userContext;
	TopoGhostContext* context = ContactContext( shapeIdA, shapeIdB );
	if ( context != NULL )
	{
		TopoGhostPreSolve( shapeIdA, shapeIdB, manifold, context );
	}
}

static bool DispatchPreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* userContext )
{
	(void)userContext;
	TopoGhostContext* context = ContactContext( shapeIdA, shapeIdB );
	return context != NULL ? TopoGhostPreContinuous( shapeIdA, shapeIdB, point, normal, context ) : true;
}

static void CreateScene( Scene* s, const Config* c, Mode mode, int totalSteps )
{
	memset( s, 0, sizeof( *s ) );
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	worldDef.enqueueTask = NULL;
	worldDef.finishTask = NULL;
	worldDef.enableSleep = c->enableSleep;
	s->worldId = b2CreateWorld( &worldDef );
	s->speed = c->speed;

	// Bodies start at x <= 0 and travel speed * duration
	float travel = c->speed * totalSteps * TIME_STEP;
	float x0 = -( c->bodyCount - 1 ) * c->spacing - 5.0f;
	float x1 = travel + 10.0f + ( c->debrisCount > 0 ? c->fieldLength : 0.0f );
	if ( c->floorEnd > c->floorStart )
	{
		x0 = c->floorStart;
		x1 = c->floorEnd;
	}

	if ( mode == mode_chain )
	{
		BuildChainFloor( s->worldId, x0, x1, c->tileWidth );
	}
	else
	{
		BuildTiles( s->worldId, x0, x1, c->tileWidth );
	}

	if ( c->debrisCount > 0 )
	{
		BuildDebris( s, c, 1.0f );
	}
	if ( c->pyramidBase > 0 )
	{
		BuildPyramid( s, c->pyramidBase );
	}

	s->protectedCount = c->bodyCount;
	s->protectedIds = calloc( (size_t)b2MaxInt( 1, c->bodyCount ), sizeof( b2BodyId ) );
	s->contexts = calloc( (size_t)b2MaxInt( 1, c->bodyCount ), sizeof( TopoGhostContext* ) );
	for ( int i = 0; i < c->bodyCount; ++i )
	{
		b2BodyId id = CreateProtected( s, BodyShape( c->shape, i ), -i * c->spacing, mode == mode_topo );
		s->protectedIds[i] = id;
		Track( s, id );
		if ( mode == mode_topo )
		{
			TopoGhostContext* context = malloc( sizeof( TopoGhostContext ) );
			if ( context == NULL )
			{
				fprintf( stderr, "out of memory\n" );
				exit( 1 );
			}
			TopoGhost_Init( context, s->worldId, id );
			context->config.timeStep = TIME_STEP;
			b2Body_SetUserData( id, context );
			s->contexts[i] = context;
		}
	}

	if ( mode == mode_topo )
	{
		b2World_SetPreSolveCallback( s->worldId, DispatchPreSolve, DispatchPreContinuous, NULL );
	}
}

static void DestroyScene( Scene* s )
{
	b2DestroyWorld( s->worldId );
	for ( int i = 0; i < s->protectedCount; ++i )
	{
		free( s->contexts[i] );
	}
	free( s->contexts );
	free( s->protectedIds );
	free( s->trackedIds );
}

// Character-controller style drive: hold the horizontal speed, keep the vertical velocity
static void Drive( Scene* s )
{
	for ( int i = 0; i < s->protectedCount; ++i )
	{
		b2Vec2 v = b2Body_GetLinearVelocity( s->protectedIds[i] );
		v.x = s->speed;
		b2Body_SetLinearVelocity( s->protectedIds[i], v );
	}
}

static uint64_t Checksum( const Scene* s )
{
	uint64_t hash = 1469598103934665603ull;
	for ( int i = 0; i < s->trackedCount; ++i )
	{
		b2WorldTransform xf = b2Body_GetTransform( s->trackedIds[i] );
		float values[4] = { (float)xf.p.x, (float)xf.p.y, xf.q.c, xf.q.s };
		const unsigned char* bytes = (const unsigned char*)values;
		for ( size_t k = 0; k < sizeof( values ); ++k )
		{
			hash = ( hash ^ bytes[k] ) * 1099511628211ull;
		}
	}
	return hash;
}

// ---------------------------------------------------------------------------------------------
// Measurement

typedef struct Samples
{
	double* pre;   // TopoGhost_PreStep for all protected bodies, per step
	double* step;  // b2World_Step, per step
	double* total; // pre + step
	int count;

	// Topology statistics over the measured steps, summed over protected bodies
	double neighbors, edges, pieces, calls;
	long long capSteps; // body-steps whose neighbor snapshot hit TOPO_MAX_NEIGHBORS
	long long overflow; // TopoGhostStats.overflow
	long long vetoed;
	double travel; // mean x distance of the protected bodies per run
	float minY, maxY; // protected body heights over the measured steps (sanity: resting, hopping, fallen)
	uint64_t checksums[MAX_REPEATS];
	int runs;
} Samples;

static void InitSamples( Samples* m, int capacity )
{
	memset( m, 0, sizeof( *m ) );
	m->minY = FLT_MAX;
	m->maxY = -FLT_MAX;
	m->pre = calloc( (size_t)capacity, sizeof( double ) );
	m->step = calloc( (size_t)capacity, sizeof( double ) );
	m->total = calloc( (size_t)capacity, sizeof( double ) );
}

static void FreeSamples( Samples* m )
{
	free( m->pre );
	free( m->step );
	free( m->total );
}

static void RunOnce( const Config* c, Mode mode, const Options* o, Samples* m )
{
	Scene s;
	CreateScene( &s, c, mode, o->warmup + o->steps );

	float startX[256] = { 0 };
	for ( int i = 0; i < s.protectedCount && i < 256; ++i )
	{
		startX[i] = (float)b2Body_GetPosition( s.protectedIds[i] ).x;
	}

	for ( int i = 0; i < o->warmup + o->steps; ++i )
	{
		bool measure = i >= o->warmup;
		if ( i == o->warmup && mode == mode_topo )
		{
			for ( int k = 0; k < s.protectedCount; ++k )
			{
				s.contexts[k]->stats = (TopoGhostStats){ 0 };
			}
		}

		Drive( &s );

		double t0 = NowMicroseconds();
		if ( mode == mode_topo )
		{
			for ( int k = 0; k < s.protectedCount; ++k )
			{
				TopoGhost_PreStep( s.contexts[k] );
			}
		}
		double t1 = NowMicroseconds();
		b2World_Step( s.worldId, TIME_STEP, SUB_STEPS );
		double t2 = NowMicroseconds();

		if ( measure == false )
		{
			continue;
		}

		for ( int k = 0; k < s.protectedCount; ++k )
		{
			float y = (float)b2Body_GetPosition( s.protectedIds[k] ).y;
			m->minY = b2MinFloat( m->minY, y );
			m->maxY = b2MaxFloat( m->maxY, y );
		}

		int n = m->count++;
		m->pre[n] = t1 - t0;
		m->step[n] = t2 - t1;
		m->total[n] = t2 - t0;

		for ( int k = 0; k < s.protectedCount && mode == mode_topo; ++k )
		{
			const TopoGhostContext* context = s.contexts[k];
			m->neighbors += context->neighborCount;
			m->capSteps += context->neighborCount == TOPO_MAX_NEIGHBORS;
			for ( int j = 0; j < context->stepSceneCount; ++j )
			{
				m->edges += context->stepScenes[j].scene.edgeCount;
				m->pieces += context->stepScenes[j].scene.pieceCount;
			}
		}
	}

	for ( int k = 0; k < s.protectedCount; ++k )
	{
		if ( mode == mode_topo )
		{
			TopoGhostStats st = s.contexts[k]->stats;
			m->calls += st.replaced + st.culled + st.kept + st.unchanged;
			m->overflow += st.overflow;
			m->vetoed += st.vetoed;
		}
		float x = (float)b2Body_GetPosition( s.protectedIds[k] ).x;
		m->travel += ( x - ( k < 256 ? startX[k] : 0.0f ) ) / s.protectedCount;
	}

	if ( m->runs < MAX_REPEATS )
	{
		m->checksums[m->runs] = Checksum( &s );
	}
	m->runs += 1;
	DestroyScene( &s );
}

static int CompareDouble( const void* a, const void* b )
{
	double x = *(const double*)a, y = *(const double*)b;
	return ( x > y ) - ( x < y );
}

typedef struct Summary
{
	double median, p10, p90, mean;
} Summary;

static Summary Summarize( const double* values, int count )
{
	Summary r = { 0 };
	if ( count == 0 )
	{
		return r;
	}
	double* sorted = malloc( (size_t)count * sizeof( double ) );
	memcpy( sorted, values, (size_t)count * sizeof( double ) );
	qsort( sorted, (size_t)count, sizeof( double ), CompareDouble );
	r.median = sorted[count / 2];
	r.p10 = sorted[(int)( 0.1 * ( count - 1 ) + 0.5 )];
	r.p90 = sorted[(int)( 0.9 * ( count - 1 ) + 0.5 )];
	for ( int i = 0; i < count; ++i )
	{
		r.mean += sorted[i];
	}
	r.mean /= count;
	free( sorted );
	return r;
}

static bool Deterministic( const Samples* m )
{
	int n = m->runs < MAX_REPEATS ? m->runs : MAX_REPEATS;
	for ( int i = 1; i < n; ++i )
	{
		if ( m->checksums[i] != m->checksums[0] )
		{
			return false;
		}
	}
	return true;
}

static const char* g_benchTitle;

static void PrintTableHeader( const char* title, const char* note )
{
	g_benchTitle = title;
	printf( "\n## %s\n\n", title );
	if ( note != NULL )
	{
		printf( "%s\n\n", note );
	}
	printf( "| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | "
			"overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |\n" );
	printf( "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n" );
}

static void RunConfig( const Config* c, const Options* o )
{
	Samples modes[mode_count];
	int capacity = o->repeats * o->steps;
	int modeCount = c->chainReference ? 3 : 2;
	for ( int k = 0; k < mode_count; ++k )
	{
		InitSamples( modes + k, capacity );
	}

	// Interleave the modes and rotate their order every repeat
	for ( int r = 0; r < o->repeats; ++r )
	{
		for ( int k = 0; k < modeCount; ++k )
		{
			Mode mode = (Mode)( ( r + k ) % modeCount );
			RunOnce( c, mode, o, modes + mode );
		}
	}

	const Samples* off = modes + mode_off;
	const Samples* topo = modes + mode_topo;
	const Samples* chain = modes + mode_chain;
	Summary offStep = Summarize( off->step, off->count );
	Summary topoPre = Summarize( topo->pre, topo->count );
	Summary topoStep = Summarize( topo->step, topo->count );
	Summary topoTotal = Summarize( topo->total, topo->count );
	Summary chainStep = Summarize( chain->step, chain->count );

	double bodySteps = (double)topo->count * b2MaxInt( 1, c->bodyCount );
	double K = topo->neighbors / bodySteps;
	double E = topo->edges / bodySteps;
	double P = topo->pieces / bodySteps;
	double calls = topo->calls / bodySteps;
	double overhead = topoTotal.median - offStep.median;
	double overheadPct = offStep.median > 0.0 ? 100.0 * overhead / offStep.median : 0.0;
	double meanOverhead = topoTotal.mean - offStep.mean;
	double memKB = c->bodyCount * (double)sizeof( TopoGhostContext ) / 1024.0;
	bool det = Deterministic( off ) && Deterministic( topo ) && ( modeCount < 3 || Deterministic( chain ) );

	char chainText[32] = "—";
	char chainOverhead[48] = "—";
	if ( modeCount == 3 )
	{
		double d = topoTotal.median - chainStep.median;
		snprintf( chainText, sizeof( chainText ), "%.1f", chainStep.median );
		snprintf( chainOverhead, sizeof( chainOverhead ), "%+.1f (%+.0f%%)", d,
				  chainStep.median > 0.0 ? 100.0 * d / chainStep.median : 0.0 );
	}

	if ( off->minY > off->maxY )
	{
		// No protected body
		for ( int k = 0; k < mode_count; ++k )
		{
			modes[k].minY = modes[k].maxY = 0.0f;
		}
	}

	printf( "| %s | %.1f | %.1f | %.1f | %.2f | %.1f | %.1f | %.1f | %s | %+.1f (%+.0f%%) | %s | %.0f/%.0f | %.0f/%.0f | "
			"%lld | %lld | %.0f | %.2f/%.2f | %s |\n",
			c->label, K, E, P, calls, topoPre.median, offStep.median, topoStep.median, chainText, overhead, overheadPct,
			chainOverhead, offStep.p10, offStep.p90, topoTotal.p10, topoTotal.p90, topo->capSteps, topo->overflow, memKB,
			off->maxY, topo->maxY, det ? "ok" : "MISMATCH" );
	fflush( stdout );

	if ( o->csv != NULL )
	{
		fprintf( o->csv,
				 "\"%s\",\"%s\",%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
				 "%.3f,%.3f,%.3f,%.3f,%lld,%lld,%lld,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%016llx,%016llx\n",
				 g_benchTitle, c->label, c->bodyCount, K, E, P, calls, topoPre.median, topoPre.p10, topoPre.p90, topoPre.mean,
				 offStep.median, offStep.p10, offStep.p90, offStep.mean, topoStep.median, topoStep.p10, topoStep.p90,
				 topoStep.mean, topoTotal.median, topoTotal.p10, topoTotal.p90, topoTotal.mean,
				 modeCount == 3 ? chainStep.median : -1.0, modeCount == 3 ? chainStep.mean : -1.0, topo->capSteps,
				 topo->overflow, topo->vetoed, memKB, off->travel / off->runs, topo->travel / topo->runs, off->minY, off->maxY,
				 topo->minY, topo->maxY, meanOverhead,
				 det ? 1 : 0, (unsigned long long)off->checksums[0], (unsigned long long)topo->checksums[0] );
		fflush( o->csv );
	}

	for ( int k = 0; k < mode_count; ++k )
	{
		FreeSamples( modes + k );
	}
}

// ---------------------------------------------------------------------------------------------
// Benchmarks

static void BenchSimple( const Options* o )
{
	PrintTableHeader( "1. Baseline simple case",
					  "One protected body driven at 1.5 m/s over 2 or 4 flush static boxes spanning 20 m." );
	const ProtShape shapes[2] = { prot_circle, prot_box };
	const char* names[2] = { "circle", "box" };
	const int tiles[2] = { 2, 4 };
	for ( int si = 0; si < 2; ++si )
	{
		for ( int ti = 0; ti < 2; ++ti )
		{
			Config c = DefaultConfig();
			snprintf( c.label, sizeof( c.label ), "%s, %d boxes", names[si], tiles[ti] );
			c.shape = shapes[si];
			c.speed = 1.5f;
			c.tileWidth = 20.0f / tiles[ti];
			c.floorStart = -2.0f;
			c.floorEnd = 18.0f;
			RunConfig( &c, o );
		}
	}
}

static void BenchDensity( const Options* o )
{
	PrintTableHeader( "2. Neighborhood density sweep",
					  "Protected 1 x 1 box driven at 10 m/s over static box tiles of width w. The two narrowest "
					  "widths go past the 128 neighbor cap on purpose." );
	const float widths[] = { 2.0f, 1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f, 0.015625f };
	for ( int i = 0; i < (int)( sizeof( widths ) / sizeof( widths[0] ) ); ++i )
	{
		Config c = DefaultConfig();
		snprintf( c.label, sizeof( c.label ), "w = %g m", widths[i] );
		c.tileWidth = widths[i];
		c.speed = 10.0f;
		RunConfig( &c, o );
	}
}

static void BenchSpeed( const Options* o )
{
	PrintTableHeader( "3. Speed sweep",
					  "Protected 1 x 1 box over 0.5 m tiles. The query reach is queryMargin + 2 |v| dt." );
	const float speeds[] = { 2.5f, 5.0f, 10.0f, 20.0f, 40.0f, 80.0f };
	for ( int i = 0; i < (int)( sizeof( speeds ) / sizeof( speeds[0] ) ); ++i )
	{
		Config c = DefaultConfig();
		snprintf( c.label, sizeof( c.label ), "v = %g m/s", speeds[i] );
		c.speed = speeds[i];
		RunConfig( &c, o );
	}
}

static void BenchCount( const Options* o )
{
	PrintTableHeader( "4. Protected-body count sweep",
					  "N protected bodies (alternating circle r = 0.5 and 1 x 1 box) 3 m apart, driven at 5 m/s over "
					  "0.5 m tiles. K, E, P and calls are per protected body; overhead is the total. N = 0 is the noise "
					  "floor (topo mode registers the callback but has no protected body)." );
	const int counts[] = { 0, 1, 2, 4, 8, 16, 32, 64 };
	for ( int i = 0; i < (int)( sizeof( counts ) / sizeof( counts[0] ) ); ++i )
	{
		Config c = DefaultConfig();
		snprintf( c.label, sizeof( c.label ), "N = %d", counts[i] );
		c.bodyCount = counts[i];
		c.shape = prot_alternate;
		RunConfig( &c, o );
	}
}

static void BenchDebris( const Options* o )
{
	PrintTableHeader( "5. Dense debris",
					  "A protected circle (r = 0.5) and a protected 1 x 1 box, 3 m apart, driven at 4 m/s across a 50 m "
					  "field of small random convex polygons (seeded). Static: gravel embedded in the floor top. "
					  "Dynamic: a loose 4-row pile they plough through. No chain reference (the surface is not a "
					  "polyline)." );
	const int counts[] = { 250, 1000, 2500, 5000 };
	for ( int i = 0; i < 4; ++i )
	{
		Config c = DefaultConfig();
		snprintf( c.label, sizeof( c.label ), "static, %d pieces", counts[i] );
		c.bodyCount = 2;
		c.shape = prot_circleAndBox;
		c.speed = 4.0f;
		c.tileWidth = 1.0f;
		c.debrisCount = counts[i];
		c.fieldLength = 50.0f;
		c.chainReference = false;
		RunConfig( &c, o );
	}
	const int dynamicCounts[] = { 200, 1000 };
	for ( int i = 0; i < 2; ++i )
	{
		Config c = DefaultConfig();
		snprintf( c.label, sizeof( c.label ), "dynamic, %d pieces", dynamicCounts[i] );
		c.bodyCount = 2;
		c.shape = prot_circleAndBox;
		c.speed = 4.0f;
		c.tileWidth = 1.0f;
		c.debrisCount = dynamicCounts[i];
		c.debrisDynamic = true;
		c.fieldLength = dynamicCounts[i] / 20.0f;
		c.chainReference = false;
		RunConfig( &c, o );
	}
}

static void BenchWorldSize( const Options* o )
{
	PrintTableHeader( "6. Unprotected-body scaling",
					  "One protected 1 x 1 box driven at 5 m/s on 0.5 m tiles, plus a pyramid of B(B+1)/2 dynamic "
					  "boxes far away. Sleep disabled so the pyramid stays awake." );
	const int bases[] = { 0, 31, 44, 63, 89 };
	for ( int i = 0; i < 5; ++i )
	{
		Config c = DefaultConfig();
		snprintf( c.label, sizeof( c.label ), "%d bodies", bases[i] * ( bases[i] + 1 ) / 2 );
		c.pyramidBase = bases[i];
		c.enableSleep = false;
		c.chainReference = false;
		RunConfig( &c, o );
	}
}

static void BenchShapes( const Options* o )
{
	PrintTableHeader( "7. Shape-type sweep", "One protected body driven at 5 m/s over 0.5 m tiles." );
	const ProtShape shapes[] = { prot_circle, prot_capsule, prot_box, prot_hexagon };
	const char* names[] = { "circle r 0.5", "capsule 0.5 + 2 x 0.25", "box 1 x 1", "hexagon r 0.5" };
	for ( int i = 0; i < 4; ++i )
	{
		Config c = DefaultConfig();
		snprintf( c.label, sizeof( c.label ), "%s", names[i] );
		c.shape = shapes[i];
		RunConfig( &c, o );
	}
}

// ---------------------------------------------------------------------------------------------
// Header

static void ReadLineFrom( const char* path, const char* key, char* out, size_t size )
{
	snprintf( out, size, "unknown" );
	FILE* f = fopen( path, "r" );
	if ( f == NULL )
	{
		return;
	}
	char line[512];
	while ( fgets( line, sizeof( line ), f ) )
	{
		if ( strncmp( line, key, strlen( key ) ) == 0 )
		{
			const char* colon = strchr( line, ':' );
			if ( colon != NULL )
			{
				colon += 1;
				while ( *colon == ' ' || *colon == '\t' )
				{
					colon += 1;
				}
				snprintf( out, size, "%s", colon );
				out[strcspn( out, "\n" )] = 0;
			}
			break;
		}
	}
	fclose( f );
}

static void PrintHeader( const Options* o )
{
	char cpu[256], allowed[256];
	ReadLineFrom( "/proc/cpuinfo", "model name", cpu, sizeof( cpu ) );
	ReadLineFrom( "/proc/self/status", "Cpus_allowed_list", allowed, sizeof( allowed ) );
#if defined( __clang__ )
	const char* compiler = "clang " __clang_version__;
#elif defined( __GNUC__ )
	const char* compiler = "gcc " __VERSION__;
#elif defined( _MSC_VER )
	const char* compiler = "msvc";
#else
	const char* compiler = "unknown";
#endif
	b2Version version = b2GetVersion();

	printf( "# Topological ghosts: cost benchmarks\n\n" );
	printf( "- Box2D %d.%d.%d, build type %s, flags `%s`%s\n", version.major, version.minor, version.revision,
			GHOST_BENCH_BUILD_TYPE, GHOST_BENCH_C_FLAGS,
#if defined( NDEBUG )
			""
#else
			" (ASSERTS ON: not a valid timing build)"
#endif
	);
	printf( "- compiler: %s\n", compiler );
	printf( "- CPU: %s, allowed CPUs: %s\n", cpu, allowed );
	printf( "- clock: %s\n", ClockName() );
	printf( "- world: workerCount 1, no task system, dt 1/60 s, %d substeps\n", SUB_STEPS );
	printf( "- per configuration: %d warm-up + %d measured steps, %d repeats, modes interleaved with rotating order\n",
			o->warmup, o->steps, o->repeats );
	printf( "- sizeof(TopoGhostContext) = %zu bytes (%.1f KB) per protected body\n\n", sizeof( TopoGhostContext ),
			sizeof( TopoGhostContext ) / 1024.0 );
	printf( "Columns: K, E, P = average neighbors snapshotted, step-scene edges and pieces per protected body "
			"and step; calls = pre-solve calls per protected body and step; PreStep = TopoGhost_PreStep for all "
			"protected bodies; Step = b2World_Step (in topo mode it includes the pre-solve callbacks). All times "
			"are µs per step, medians over the pooled measured steps of all repeats. overhead vs off = median(PreStep "
			"+ Step, topo) - median(Step, off): what turning the mitigation on costs a game. overhead vs chain = the "
			"same against the chain floor with no mitigation (Box2D's own ghost vertices): the cost against the "
			"ideal, ghost-free result. Without mitigation the protected body is often launched by ghost collisions "
			"and flies with no contacts, which makes the off step cheaper; max y off/topo shows it (resting height "
			"0.5 for the 1 x 1 box and circle). p10/p90 are the per-step spread. cap steps = body-steps whose "
			"snapshot hit TOPO_MAX_NEIGHBORS (%d); overflow = TopoGhostStats.overflow events; mem = N x "
			"sizeof(TopoGhostContext); det = final-state checksums identical across repeats in every mode.\n",
			TOPO_MAX_NEIGHBORS );
	fflush( stdout );
}

int main( int argc, char** argv )
{
	Options o = { 7, 60, 600, 0x7F, NULL };
	for ( int i = 1; i < argc; ++i )
	{
		if ( strcmp( argv[i], "--repeats" ) == 0 && i + 1 < argc )
		{
			o.repeats = atoi( argv[++i] );
		}
		else if ( strcmp( argv[i], "--warmup" ) == 0 && i + 1 < argc )
		{
			o.warmup = atoi( argv[++i] );
		}
		else if ( strcmp( argv[i], "--steps" ) == 0 && i + 1 < argc )
		{
			o.steps = atoi( argv[++i] );
		}
		else if ( strcmp( argv[i], "--bench" ) == 0 && i + 1 < argc )
		{
			o.benchMask = 0;
			for ( const char* p = argv[++i]; *p; ++p )
			{
				if ( *p >= '1' && *p <= '7' )
				{
					o.benchMask |= 1u << ( *p - '1' );
				}
			}
		}
		else if ( strcmp( argv[i], "--csv" ) == 0 && i + 1 < argc )
		{
			o.csv = fopen( argv[++i], "w" );
			if ( o.csv == NULL )
			{
				fprintf( stderr, "cannot open %s\n", argv[i] );
				return 1;
			}
		}
		else if ( strcmp( argv[i], "--quick" ) == 0 )
		{
			o.repeats = 2;
			o.warmup = 10;
			o.steps = 60;
		}
		else
		{
			fprintf( stderr,
					 "usage: ghost_bench [--repeats R] [--warmup W] [--steps M] [--bench 1234567] [--csv file] [--quick]\n" );
			return 1;
		}
	}

	if ( o.repeats < 1 || o.repeats > MAX_REPEATS || o.steps < 1 || o.warmup < 0 )
	{
		fprintf( stderr, "repeats must be in [1, %d], steps >= 1, warmup >= 0\n", MAX_REPEATS );
		return 1;
	}

	if ( o.csv != NULL )
	{
		fprintf( o.csv, "bench,config,bodies,K,E,P,calls,pre_med,pre_p10,pre_p90,pre_mean,off_med,off_p10,off_p90,"
						"off_mean,topo_step_med,topo_step_p10,topo_step_p90,topo_step_mean,topo_total_med,topo_total_p10,"
						"topo_total_p90,topo_total_mean,chain_med,chain_mean,cap_steps,overflow,vetoed,mem_kb,"
						"off_travel_m,topo_travel_m,off_min_y,off_max_y,topo_min_y,topo_max_y,mean_overhead_us,deterministic,off_checksum,topo_checksum\n" );
	}

	PrintHeader( &o );

	void ( *benches[7] )( const Options* ) = { BenchSimple, BenchDensity, BenchSpeed, BenchCount,
											   BenchDebris, BenchWorldSize, BenchShapes };
	for ( int i = 0; i < 7; ++i )
	{
		if ( o.benchMask & ( 1u << i ) )
		{
			benches[i]( &o );
		}
	}

	if ( o.csv != NULL )
	{
		fclose( o.csv );
	}
	return 0;
}
