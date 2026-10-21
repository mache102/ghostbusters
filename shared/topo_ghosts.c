// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

#include "topo_ghosts.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

// Everything below works in the frame of the touched shape's body ("frame T"), which is also the
// frame the chain segment colliders return their manifolds in.

#define TOPO_MAX_OWNERS 32
#define TOPO_MAX_EDGES 192
#define TOPO_MAX_INTERVALS 6
#define TOPO_MAX_SUPPORTS 16
#define TOPO_MAX_SHAPE_EDGES 8

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

typedef struct TopoOwner
{
	const TopoGhostShape* shape;
	b2Polygon polygon; // in frame T, valid when isSolid
	bool isSolid;
	bool centroidInside; // push out through the outer faces of this solid
	bool fullyInside;	 // embedded: the neighbor faces become walls against phasing
} TopoOwner;

// A surface line the player is resting on
typedef struct TopoSupport
{
	b2Vec2 point;
	b2Vec2 normal;
} TopoSupport;

typedef struct TopoScene
{
	TopoOwner owners[TOPO_MAX_OWNERS];
	int ownerCount;
	TopoEdge edges[TOPO_MAX_EDGES];
	int edgeCount;
	int touchedEdgeCount; // edges [0, touchedEdgeCount) belong to the touched shape (owner 0)
	b2Vec2 centroid;	  // player centroid in frame T

	// Player core points and radius in frame T
	b2Vec2 playerPoints[B2_MAX_POLYGON_VERTICES];
	int playerPointCount;
	float playerRadius;

	TopoSupport supports[TOPO_MAX_SUPPORTS];
	int supportCount;
	float penetrationTolerance;

	// Capacity limit hit while building
	bool overflow;
} TopoScene;

// A surviving piece of an edge, used for ghost candidates and collision
typedef struct TopoPiece
{
	b2Vec2 a, b;
	int edgeIndex;
	int slot; // interval index within the edge
} TopoPiece;

TopoGhostConfig TopoGhost_DefaultConfig( void )
{
	TopoGhostConfig config;
	config.linearTolerance = B2_LINEAR_SLOP;
	config.angularTolerance = 0.01f;
	config.queryMargin = 0.25f;
	config.penetrationTolerance = 4.0f * B2_LINEAR_SLOP;
	config.timeStep = 1.0f / 60.0f;
	return config;
}

void TopoGhost_Init( TopoGhostContext* context, b2WorldId worldId, b2BodyId playerId )
{
	*context = (TopoGhostContext){ 0 };
	context->worldId = worldId;
	context->playerId = playerId;
	context->config = TopoGhost_DefaultConfig();
}

// Fill a snapshot from the live shape. Returns false for shapes that cannot contribute edges.
static bool SnapshotShape( TopoGhostShape* snapshot, b2ShapeId shapeId )
{
	*snapshot = (TopoGhostShape){ 0 };
	snapshot->shapeId = shapeId;
	snapshot->type = b2Shape_GetType( shapeId );
	snapshot->filter = b2Shape_GetFilter( shapeId );
	snapshot->bounds = b2Shape_GetBounds( shapeId );
	snapshot->transform = b2Body_GetTransform( b2Shape_GetBody( shapeId ) );

	switch ( snapshot->type )
	{
		case b2_polygonShape:
			snapshot->geometry = b2Shape_GetPolygon( shapeId );

			// Rounded polygons need radius aware matching, not supported yet
			return snapshot->geometry.radius == 0.0f;

		case b2_segmentShape:
		{
			b2Segment segment = b2Shape_GetSegment( shapeId );
			snapshot->geometry.vertices[0] = segment.point1;
			snapshot->geometry.vertices[1] = segment.point2;
			snapshot->geometry.count = 2;
			return true;
		}

		case b2_chainSegmentShape:
		{
			b2ChainSegment cs = b2Shape_GetChainSegment( shapeId );
			snapshot->geometry.vertices[0] = cs.segment.point1;
			snapshot->geometry.vertices[1] = cs.segment.point2;
			snapshot->geometry.count = 2;
			return true;
		}

		default:
			return false;
	}
}

static bool GatherNeighbor( b2ShapeId shapeId, void* userContext )
{
	TopoGhostContext* context = userContext;
	if ( B2_ID_EQUALS( b2Shape_GetBody( shapeId ), context->playerId ) || b2Shape_IsSensor( shapeId ) )
	{
		return true;
	}

	if ( context->neighborCount == TOPO_MAX_NEIGHBORS )
	{
		context->stats.overflow += 1;
		return false;
	}

	if ( SnapshotShape( context->neighbors + context->neighborCount, shapeId ) )
	{
		context->neighborCount += 1;
	}

	return true;
}

void TopoGhost_PreStep( TopoGhostContext* context )
{
	context->neighborCount = 0;
	if ( b2Body_IsValid( context->playerId ) == false )
	{
		return;
	}

	// A recycled contact skips pre-solve, which would silently disable the mitigation
	if ( b2Body_IsContactRecyclingEnabled( context->playerId ) )
	{
		b2Body_EnableContactRecycling( context->playerId, false );
	}

	context->playerTransform = b2Body_GetTransform( context->playerId );

	// Cover where the player can reach during the step
	b2AABB box = b2Body_ComputeBounds( context->playerId );
	b2Vec2 v = b2Body_GetLinearVelocity( context->playerId );
	float reach = context->config.queryMargin + 2.0f * b2Length( v ) * context->config.timeStep;
	box.lowerBound = b2Sub( box.lowerBound, (b2Vec2){ reach, reach } );
	box.upperBound = b2Add( box.upperBound, (b2Vec2){ reach, reach } );

	b2World_OverlapAABB( context->worldId, b2Pos_zero, box, b2DefaultQueryFilter(), GatherNeighbor, context );
}

static const TopoGhostShape* FindSnapshot( const TopoGhostContext* context, b2ShapeId shapeId )
{
	for ( int i = 0; i < context->neighborCount; ++i )
	{
		if ( B2_ID_EQUALS( context->neighbors[i].shapeId, shapeId ) )
		{
			return context->neighbors + i;
		}
	}
	return NULL;
}

static bool ShouldCollide( b2Filter a, b2Filter b )
{
	if ( a.groupIndex == b.groupIndex && a.groupIndex != 0 )
	{
		return a.groupIndex > 0;
	}

	return ( a.maskBits & b.categoryBits ) != 0 && ( a.categoryBits & b.maskBits ) != 0;
}

static bool AABBsOverlap( b2AABB a, b2AABB b, float margin )
{
	return a.lowerBound.x - margin <= b.upperBound.x && b.lowerBound.x - margin <= a.upperBound.x &&
		   a.lowerBound.y - margin <= b.upperBound.y && b.lowerBound.y - margin <= a.upperBound.y;
}

static void AddEdge( TopoScene* scene, b2Vec2 p1, b2Vec2 p2, int owner )
{
	if ( b2DistanceSquared( p1, p2 ) < FLT_EPSILON )
	{
		return;
	}

	if ( scene->edgeCount == TOPO_MAX_EDGES )
	{
		scene->overflow = true;
		return;
	}

	TopoEdge* edge = scene->edges + scene->edgeCount;
	edge->p1 = p1;
	edge->p2 = p2;
	edge->owner = owner;
	edge->intervals[0] = (TopoInterval){ 0.0f, 1.0f };
	edge->intervalCount = 1;
	scene->edgeCount += 1;
}

// A snapshot shape's collidable edges in frame T, outward normal on the right. Fills polygon (in
// frame T) for solid polygons. Returns the edge count.
static int ShapeEdges( const TopoGhostShape* shape, b2WorldTransform frameT, b2Vec2* p1s, b2Vec2* p2s, b2Polygon* polygon )
{
	b2Transform xf = b2InvMulWorldTransforms( frameT, shape->transform );
	switch ( shape->type )
	{
		case b2_polygonShape:
		{
			*polygon = b2TransformPolygon( xf, &shape->geometry );
			for ( int i = 0; i < polygon->count; ++i )
			{
				int j = i + 1 < polygon->count ? i + 1 : 0;
				p1s[i] = polygon->vertices[i];
				p2s[i] = polygon->vertices[j];
			}
			return polygon->count;
		}

		case b2_segmentShape:
		{
			// Two-sided: an edge in each direction
			p1s[0] = b2TransformPoint( xf, shape->geometry.vertices[0] );
			p2s[0] = b2TransformPoint( xf, shape->geometry.vertices[1] );
			p1s[1] = p2s[0];
			p2s[1] = p1s[0];
			return 2;
		}

		case b2_chainSegmentShape:
		{
			// One-sided, collides on the right of point1 -> point2
			p1s[0] = b2TransformPoint( xf, shape->geometry.vertices[0] );
			p2s[0] = b2TransformPoint( xf, shape->geometry.vertices[1] );
			return 1;
		}

		default:
			return 0;
	}
}

// Every player point is behind every face of the polygon by at least the player radius
static bool PlayerFullyInside( const TopoScene* scene, const b2Polygon* polygon )
{
	for ( int i = 0; i < polygon->count; ++i )
	{
		for ( int j = 0; j < scene->playerPointCount; ++j )
		{
			float offset = b2Dot( polygon->normals[i], b2Sub( scene->playerPoints[j], polygon->vertices[i] ) );
			if ( offset > -scene->playerRadius )
			{
				return false;
			}
		}
	}
	return true;
}

// Add a shape's collidable edges in frame T
static bool AddOwner( TopoScene* scene, const TopoGhostShape* shape, b2WorldTransform frameT )
{
	if ( scene->ownerCount == TOPO_MAX_OWNERS )
	{
		scene->overflow = true;
		return false;
	}

	b2Vec2 p1s[TOPO_MAX_SHAPE_EDGES], p2s[TOPO_MAX_SHAPE_EDGES];
	b2Polygon polygon;
	int count = ShapeEdges( shape, frameT, p1s, p2s, &polygon );

	int owner = scene->ownerCount;
	TopoOwner* o = scene->owners + owner;
	*o = (TopoOwner){ 0 };
	o->shape = shape;
	if ( shape->type == b2_polygonShape )
	{
		o->polygon = polygon;
		o->isSolid = true;
		o->centroidInside = b2PointInPolygon( &o->polygon, scene->centroid );
		o->fullyInside = o->centroidInside && PlayerFullyInside( scene, &o->polygon );
	}

	for ( int i = 0; i < count; ++i )
	{
		AddEdge( scene, p1s[i], p2s[i], owner );
	}

	scene->ownerCount += 1;
	return true;
}

// How deep the player reaches behind a line (positive when penetrating)
static float PlayerDepth( const TopoScene* scene, b2Vec2 point, b2Vec2 normal )
{
	float depth = -FLT_MAX;
	for ( int i = 0; i < scene->playerPointCount; ++i )
	{
		depth = b2MaxFloat( depth, -b2Dot( normal, b2Sub( scene->playerPoints[i], point ) ) );
	}
	return depth + scene->playerRadius;
}

// Record the surfaces of a shape that the player rests on: centroid in front and over the span,
// touching (within the speculative distance) and sunk less than the penetration tolerance
static void AddSupports( TopoScene* scene, const TopoGhostShape* shape, b2WorldTransform frameT, const TopoGhostConfig* config )
{
	b2Vec2 p1s[TOPO_MAX_SHAPE_EDGES], p2s[TOPO_MAX_SHAPE_EDGES];
	b2Polygon polygon;
	int count = ShapeEdges( shape, frameT, p1s, p2s, &polygon );
	for ( int i = 0; i < count && scene->supportCount < TOPO_MAX_SUPPORTS; ++i )
	{
		b2Vec2 d = b2Sub( p2s[i], p1s[i] );
		float length = b2Length( d );
		if ( length < config->linearTolerance )
		{
			continue;
		}

		b2Vec2 u = b2MulSV( 1.0f / length, d );
		b2Vec2 n = b2RightPerp( u );
		b2Vec2 r = b2Sub( scene->centroid, p1s[i] );
		float along = b2Dot( u, r );
		if ( b2Dot( n, r ) <= config->linearTolerance || along < 0.0f || along > length )
		{
			continue;
		}

		float depth = PlayerDepth( scene, p1s[i], n );
		if ( depth < -B2_SPECULATIVE_DISTANCE || depth > config->penetrationTolerance )
		{
			continue;
		}

		scene->supports[scene->supportCount++] = (TopoSupport){ p1s[i], n };
	}
}

// Remove [t0, t1] from the surviving intervals of an edge. On overflow the edge is left as it
// was: a possible ghost bump is better than a hole in real ground.
static bool SubtractInterval( TopoEdge* edge, float t0, float t1 )
{
	TopoInterval result[TOPO_MAX_INTERVALS + 1];
	int count = 0;
	for ( int i = 0; i < edge->intervalCount; ++i )
	{
		TopoInterval in = edge->intervals[i];
		if ( count + 2 > TOPO_MAX_INTERVALS + 1 )
		{
			return false;
		}

		if ( t1 <= in.t0 || in.t1 <= t0 )
		{
			result[count++] = in;
			continue;
		}

		if ( in.t0 < t0 )
		{
			result[count++] = (TopoInterval){ in.t0, t0 };
		}

		if ( t1 < in.t1 )
		{
			result[count++] = (TopoInterval){ t1, in.t1 };
		}
	}

	if ( count > TOPO_MAX_INTERVALS )
	{
		return false;
	}

	for ( int i = 0; i < count; ++i )
	{
		edge->intervals[i] = result[i];
	}
	edge->intervalCount = count;
	return true;
}

// Phase 1: cancel portions of edges that coincide with an antiparallel edge of another owner.
static void CancelInteriorEdges( TopoScene* scene, const TopoGhostConfig* config )
{
	float linTol = config->linearTolerance;
	for ( int i = 0; i < scene->edgeCount; ++i )
	{
		TopoEdge* e = scene->edges + i;
		b2Vec2 d = b2Sub( e->p2, e->p1 );
		float length = b2Length( d );
		b2Vec2 u = b2MulSV( 1.0f / length, d );

		for ( int j = 0; j < scene->edgeCount; ++j )
		{
			const TopoEdge* f = scene->edges + j;
			if ( f->owner == e->owner )
			{
				continue;
			}

			// Embedded in the neighbor's solid: this face is the wall that keeps the player from
			// phasing on into this solid, keep it. Otherwise (on the surface, or only partly sunk)
			// the face is interior. Push-out uses only the surviving outer faces.
			const TopoOwner* oe = scene->owners + e->owner;
			const TopoOwner* of = scene->owners + f->owner;
			if ( of->fullyInside && oe->fullyInside == false )
			{
				continue;
			}

			b2Vec2 fd = b2Sub( f->p2, f->p1 );
			float fLength = b2Length( fd );
			b2Vec2 fu = b2MulSV( 1.0f / fLength, fd );

			// Antiparallel
			if ( b2Dot( u, fu ) > 0.0f || b2AbsFloat( b2Cross( u, fu ) ) > config->angularTolerance )
			{
				continue;
			}

			// Coincident lines
			if ( b2AbsFloat( b2Cross( u, b2Sub( f->p1, e->p1 ) ) ) > linTol ||
				 b2AbsFloat( b2Cross( u, b2Sub( f->p2, e->p1 ) ) ) > linTol )
			{
				continue;
			}

			float ta = b2Dot( u, b2Sub( f->p1, e->p1 ) ) / length;
			float tb = b2Dot( u, b2Sub( f->p2, e->p1 ) ) / length;
			float t0 = b2MinFloat( ta, tb );
			float t1 = b2MaxFloat( ta, tb );

			// Snap to the ends so slivers under the tolerance disappear
			float tTol = linTol / length;
			if ( t0 < tTol )
			{
				t0 = -1.0f;
			}
			if ( t1 > 1.0f - tTol )
			{
				t1 = 2.0f;
			}

			if ( t1 > t0 && SubtractInterval( e, t0, t1 ) == false )
			{
				scene->overflow = true;
			}
		}
	}
}

static int CollectPieces( const TopoScene* scene, TopoPiece* pieces, int capacity, float linTol )
{
	int count = 0;
	for ( int i = 0; i < scene->edgeCount; ++i )
	{
		const TopoEdge* e = scene->edges + i;
		b2Vec2 d = b2Sub( e->p2, e->p1 );
		float length = b2Length( d );
		for ( int k = 0; k < e->intervalCount && count < capacity; ++k )
		{
			TopoInterval in = e->intervals[k];
			float t0 = b2MaxFloat( in.t0, 0.0f );
			float t1 = b2MinFloat( in.t1, 1.0f );
			if ( ( t1 - t0 ) * length < linTol )
			{
				continue;
			}

			pieces[count].a = b2MulAdd( e->p1, t0, d );
			pieces[count].b = b2MulAdd( e->p1, t1, d );
			pieces[count].edgeIndex = i;
			pieces[count].slot = k;
			count += 1;
		}
	}
	return count;
}

// Angle in (0, 2pi] turning counter-clockwise from ref to v
static float CCWAngle( b2Vec2 ref, b2Vec2 v )
{
	float angle = atan2f( b2Cross( ref, v ), b2Dot( ref, v ) );
	if ( angle <= 0.0f )
	{
		angle += 2.0f * B2_PI;
	}
	return angle;
}

static bool InFront( b2Vec2 p1, b2Vec2 p2, b2Vec2 point, float tolerance )
{
	b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( p2, p1 ) ) );
	return b2Dot( n, b2Sub( point, p1 ) ) > tolerance;
}

static bool Behind( b2Vec2 p1, b2Vec2 p2, b2Vec2 point, float tolerance )
{
	b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( p2, p1 ) ) );
	return b2Dot( n, b2Sub( point, p1 ) ) < -tolerance;
}

// A piece of the touched shape is covered when it meets another surface that the player is
// standing on (centroid in front of it and over its span) and lies behind that surface. The player
// cannot reach a covered piece without passing through a solid, so it is cancelled for this
// player, like an interior face. Beside the surface (e.g. falling past a cliff) the piece is live.
static bool IsCovered( const TopoScene* scene, const TopoPiece* pieces, int pieceCount, int self, float linTol )
{
	const TopoPiece* s = pieces + self;
	float linTolSqr = linTol * linTol;

	// Behind a support surface (not part of it): reachable only by sinking into the support
	b2Vec2 ns = b2RightPerp( b2Normalize( b2Sub( s->b, s->a ) ) );
	for ( int i = 0; i < scene->supportCount; ++i )
	{
		const TopoSupport* support = scene->supports + i;
		if ( b2Dot( ns, support->normal ) > 0.99f )
		{
			continue;
		}

		if ( b2Dot( support->normal, b2Sub( s->a, support->point ) ) <= linTol &&
			 b2Dot( support->normal, b2Sub( s->b, support->point ) ) <= linTol )
		{
			return true;
		}
	}

	for ( int i = 0; i < pieceCount; ++i )
	{
		if ( i == self )
		{
			continue;
		}

		const TopoPiece* c = pieces + i;
		b2Vec2 far;
		if ( b2DistanceSquared( c->a, s->a ) < linTolSqr || b2DistanceSquared( c->b, s->a ) < linTolSqr )
		{
			far = s->b;
		}
		else if ( b2DistanceSquared( c->a, s->b ) < linTolSqr || b2DistanceSquared( c->b, s->b ) < linTolSqr )
		{
			far = s->a;
		}
		else
		{
			continue;
		}

		b2Vec2 d = b2Sub( c->b, c->a );
		float length = b2Length( d );
		float t = b2Dot( b2Sub( scene->centroid, c->a ), d ) / length;
		if ( t < 0.0f || t > length )
		{
			continue;
		}

		if ( InFront( c->a, c->b, scene->centroid, linTol ) == false || Behind( c->a, c->b, far, linTol ) == false )
		{
			continue;
		}

		// Only while the player actually rests on c (same depth band as the support rule). At a
		// convex crest the centroid passes over the downslope while the player still rests on the
		// flat top; cancelling the top there leaves nothing holding the crest.
		float depth = PlayerDepth( scene, c->a, b2RightPerp( b2MulSV( 1.0f / length, d ) ) );
		if ( -B2_SPECULATIVE_DISTANCE <= depth && depth <= scene->penetrationTolerance )
		{
			return true;
		}
	}

	return false;
}

// Phase 2: pick the ghost vertex at `vertex` for piece s. atStart selects ghost1 (vertex is s.a)
// or ghost2 (vertex is s.b).
static b2Vec2 FindGhost( const TopoScene* scene, const TopoPiece* pieces, const bool* covered, int pieceCount, int self, bool atStart,
						 float linTol )
{
	const TopoPiece* s = pieces + self;
	b2Vec2 vertex = atStart ? s->a : s->b;
	b2Vec2 far = atStart ? s->b : s->a;
	float linTolSqr = linTol * linTol;

	// Covering rule: a surface meeting this vertex that the player stands in front of, with this
	// piece behind it, owns the vertex region (marblebuilds one-way fix, made player relative).
	// When several surfaces cover the vertex, take the one that makes this corner most concave,
	// which hands the most of the vertex region to the covering surfaces.
	b2Vec2 edge1 = b2Normalize( b2Sub( s->b, s->a ) );
	float bestCross = FLT_MAX;
	b2Vec2 covering = vertex;
	for ( int i = 0; i < pieceCount; ++i )
	{
		// A covered piece cannot shield or continue anything for this player
		if ( i == self || ( covered != NULL && covered[i] ) )
		{
			continue;
		}

		const TopoPiece* c = pieces + i;
		b2Vec2 other;
		if ( b2DistanceSquared( c->a, vertex ) < linTolSqr )
		{
			other = c->b;
		}
		else if ( b2DistanceSquared( c->b, vertex ) < linTolSqr )
		{
			other = c->a;
		}
		else
		{
			continue;
		}

		if ( InFront( c->a, c->b, scene->centroid, linTol ) == false || Behind( c->a, c->b, far, linTol ) == false )
		{
			continue;
		}

		// Convexity of the corner this ghost would create (same test as the chain collider)
		float cross = atStart ? b2Cross( b2Normalize( b2Sub( vertex, other ) ), edge1 )
							  : b2Cross( edge1, b2Normalize( b2Sub( other, vertex ) ) );
		if ( cross < bestCross )
		{
			bestCross = cross;
			covering = other;
		}
	}

	if ( bestCross < FLT_MAX )
	{
		return covering;
	}

	// localSeg angle rule. ghost1 comes from incoming pieces (ending at the vertex), smallest
	// clockwise turn from this piece. ghost2 comes from outgoing pieces, smallest counter-clockwise
	// turn. Pieces passing through the vertex contribute their part on the matching side.
	b2Vec2 ref = b2Sub( far, vertex );
	float bestAngle = FLT_MAX;
	b2Vec2 best = vertex;
	bool found = false;

	for ( int i = 0; i < pieceCount; ++i )
	{
		// A covered piece cannot shield or continue anything for this player
		if ( i == self || ( covered != NULL && covered[i] ) )
		{
			continue;
		}

		const TopoPiece* c = pieces + i;
		b2Vec2 candidate;
		if ( atStart )
		{
			if ( b2DistanceSquared( c->b, vertex ) < linTolSqr )
			{
				candidate = c->a;
			}
			else
			{
				continue;
			}
		}
		else
		{
			if ( b2DistanceSquared( c->a, vertex ) < linTolSqr )
			{
				candidate = c->b;
			}
			else
			{
				continue;
			}
		}

		b2Vec2 dir = b2Sub( candidate, vertex );
		float angle = atStart ? CCWAngle( dir, ref ) : CCWAngle( ref, dir );
		if ( angle < 1e-4f || angle > 2.0f * B2_PI - 1e-4f )
		{
			// Same direction, an overlapping duplicate
			continue;
		}

		if ( angle < bestAngle )
		{
			bestAngle = angle;
			best = candidate;
			found = true;
		}
	}

	if ( found == false )
	{
		// Pieces passing through the vertex (T-junctions, partial overlaps)
		for ( int i = 0; i < pieceCount; ++i )
		{
			if ( i == self )
			{
				continue;
			}

			const TopoPiece* c = pieces + i;
			b2Vec2 d = b2Sub( c->b, c->a );
			float length = b2Length( d );
			b2Vec2 u = b2MulSV( 1.0f / length, d );
			b2Vec2 r = b2Sub( vertex, c->a );
			float t = b2Dot( u, r );
			if ( t <= linTol || t >= length - linTol || b2AbsFloat( b2Cross( u, r ) ) > linTol )
			{
				continue;
			}

			// ghost1 needs the part before the vertex, ghost2 the part after
			b2Vec2 candidate = atStart ? c->a : c->b;
			b2Vec2 dir = b2Sub( candidate, vertex );
			float angle = atStart ? CCWAngle( dir, ref ) : CCWAngle( ref, dir );
			if ( angle < bestAngle && angle > 1e-4f )
			{
				bestAngle = angle;
				best = candidate;
				found = true;
			}
		}
	}

	if ( found == false )
	{
		// Nothing continues the surface: a convex corner
		b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( s->b, s->a ) ) );
		best = b2Sub( vertex, n );
	}

	return best;
}

// The player shape's points, radius and centroid in frame T. The shape's own centroid (not the
// body's) is used so compound players work.
static void SetPlayerGeometry( TopoScene* scene, b2ShapeId playerShapeId, b2Transform xf )
{
	scene->centroid = xf.p;
	scene->playerPointCount = 0;
	scene->playerRadius = 0.0f;
	switch ( b2Shape_GetType( playerShapeId ) )
	{
		case b2_polygonShape:
		{
			b2Polygon polygon = b2Shape_GetPolygon( playerShapeId );
			for ( int i = 0; i < polygon.count; ++i )
			{
				scene->playerPoints[i] = b2TransformPoint( xf, polygon.vertices[i] );
			}
			scene->playerPointCount = polygon.count;
			scene->playerRadius = polygon.radius;
			scene->centroid = b2TransformPoint( xf, polygon.centroid );
		}
		break;

		case b2_circleShape:
		{
			b2Circle circle = b2Shape_GetCircle( playerShapeId );
			scene->playerPoints[0] = b2TransformPoint( xf, circle.center );
			scene->playerPointCount = 1;
			scene->playerRadius = circle.radius;
			scene->centroid = scene->playerPoints[0];
		}
		break;

		case b2_capsuleShape:
		{
			b2Capsule capsule = b2Shape_GetCapsule( playerShapeId );
			scene->playerPoints[0] = b2TransformPoint( xf, capsule.center1 );
			scene->playerPoints[1] = b2TransformPoint( xf, capsule.center2 );
			scene->playerPointCount = 2;
			scene->playerRadius = capsule.radius;
			scene->centroid = b2Lerp( scene->playerPoints[0], scene->playerPoints[1], 0.5f );
		}
		break;

		default:
			break;
	}
}

// Build the local topology around the touched shape from the pre-step snapshot
static void BuildScene( TopoScene* scene, const TopoGhostContext* context, const TopoGhostShape* touched, b2ShapeId playerShapeId )
{
	b2WorldTransform frameT = touched->transform;
	scene->ownerCount = 0;
	scene->edgeCount = 0;
	scene->supportCount = 0;
	scene->overflow = false;
	scene->penetrationTolerance = context->config.penetrationTolerance;

	SetPlayerGeometry( scene, playerShapeId, b2InvMulWorldTransforms( frameT, context->playerTransform ) );

	AddOwner( scene, touched, frameT );
	scene->touchedEdgeCount = scene->edgeCount;

	b2Filter playerFilter = b2Shape_GetFilter( playerShapeId );
	float margin = 2.0f * context->config.linearTolerance;
	AddSupports( scene, touched, frameT, &context->config );

	for ( int i = 0; i < context->neighborCount; ++i )
	{
		const TopoGhostShape* neighbor = context->neighbors + i;
		if ( B2_ID_EQUALS( neighbor->shapeId, touched->shapeId ) )
		{
			continue;
		}

		if ( ShouldCollide( playerFilter, neighbor->filter ) == false )
		{
			continue;
		}

		if ( context->filterFcn != NULL && context->filterFcn( playerShapeId, neighbor->shapeId, context->filterContext ) == false )
		{
			continue;
		}

		// Any nearby surface can support the player
		AddSupports( scene, neighbor, frameT, &context->config );

		// Only shapes touching the touched shape can change its edge topology
		if ( AABBsOverlap( neighbor->bounds, touched->bounds, margin ) )
		{
			AddOwner( scene, neighbor, frameT );
		}
	}
}

static b2LocalManifold CollidePiece( const b2ChainSegment* cs, b2ShapeId playerShapeId, b2Transform xf )
{
	b2SimplexCache cache = b2_emptySimplexCache;
	switch ( b2Shape_GetType( playerShapeId ) )
	{
		case b2_polygonShape:
		{
			b2Polygon polygon = b2Shape_GetPolygon( playerShapeId );
			return b2CollideChainSegmentAndPolygon( cs, &polygon, xf, &cache );
		}
		case b2_circleShape:
		{
			b2Circle circle = b2Shape_GetCircle( playerShapeId );
			return b2CollideChainSegmentAndCircle( cs, &circle, xf );
		}
		case b2_capsuleShape:
		{
			b2Capsule capsule = b2Shape_GetCapsule( playerShapeId );
			return b2CollideChainSegmentAndCapsule( cs, &capsule, xf, &cache );
		}
		default:
			return (b2LocalManifold){ 0 };
	}
}

// Minimum translation out of the touched solid along one of its surviving (outer) faces.
// Returns the piece index, or -1 if there is none.
static int PushOut( const TopoScene* scene, const TopoPiece* pieces, int pieceCount, b2LocalManifold* manifold, float linTol )
{
	int bestPiece = -1;
	float bestDepth = FLT_MAX;
	for ( int i = 0; i < pieceCount; ++i )
	{
		if ( pieces[i].edgeIndex >= scene->touchedEdgeCount )
		{
			continue;
		}

		b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( pieces[i].b, pieces[i].a ) ) );
		float depth = PlayerDepth( scene, pieces[i].a, n );
		if ( depth < bestDepth )
		{
			bestDepth = depth;
			bestPiece = i;
		}
	}

	if ( bestPiece < 0 )
	{
		return -1;
	}

	// Up to two deepest player points along the chosen face
	b2Vec2 a = pieces[bestPiece].a;
	b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( pieces[bestPiece].b, a ) ) );
	*manifold = (b2LocalManifold){ 0 };
	manifold->normal = n;
	for ( int i = 0; i < scene->playerPointCount && manifold->pointCount < 2; ++i )
	{
		b2Vec2 v = scene->playerPoints[i];
		float depth = -b2Dot( n, b2Sub( v, a ) ) + scene->playerRadius;
		if ( depth < bestDepth - linTol )
		{
			continue;
		}

		b2LocalManifoldPoint* mp = manifold->points + manifold->pointCount;
		mp->point = b2MulSub( v, scene->playerRadius, n );
		mp->separation = -depth;
		mp->id = (uint16_t)( 0x4000 | i );
		manifold->pointCount += 1;
	}

	return bestPiece;
}

static float MinSeparation( const b2LocalManifold* m )
{
	float s = FLT_MAX;
	for ( int i = 0; i < m->pointCount; ++i )
	{
		s = b2MinFloat( s, m->points[i].separation );
	}
	return s;
}

void TopoGhostPreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* userContext )
{
	TopoGhostContext* context = userContext;

	bool playerIsA = B2_ID_EQUALS( b2Shape_GetBody( shapeIdA ), context->playerId );
	bool playerIsB = B2_ID_EQUALS( b2Shape_GetBody( shapeIdB ), context->playerId );
	if ( playerIsA == playerIsB )
	{
		return;
	}

	b2ShapeId playerShapeId = playerIsA ? shapeIdA : shapeIdB;
	b2ShapeId touchedId = playerIsA ? shapeIdB : shapeIdA;
	b2ShapeType playerType = b2Shape_GetType( playerShapeId );
	if ( playerType != b2_polygonShape && playerType != b2_circleShape && playerType != b2_capsuleShape )
	{
		context->stats.unchanged += 1;
		return;
	}

	// Pre-solve sees the pre-step poses, so the snapshot matches. A shape that entered range
	// without being snapshotted can be read live here (collide does not move bodies).
	TopoGhostShape live;
	const TopoGhostShape* touched = FindSnapshot( context, touchedId );
	if ( touched == NULL )
	{
		if ( SnapshotShape( &live, touchedId ) == false )
		{
			context->stats.unchanged += 1;
			return;
		}
		touched = &live;
	}

	b2WorldTransform frameT = touched->transform;
	b2WorldTransform framePlayer = context->playerTransform;

	TopoScene scene;
	BuildScene( &scene, context, touched, playerShapeId );
	if ( scene.overflow )
	{
		context->stats.overflow += 1;
	}

	const TopoGhostConfig* config = &context->config;
	CancelInteriorEdges( &scene, config );

	TopoPiece pieces[TOPO_MAX_EDGES * 2];
	int pieceCount = CollectPieces( &scene, pieces, TOPO_MAX_EDGES * 2, config->linearTolerance );

	b2Transform xf = b2InvMulWorldTransforms( frameT, framePlayer );
	b2LocalManifold best = { 0 };
	int bestPiece = -1;
	float bestSeparation = FLT_MAX;
	bool pushedOut = false;

	if ( scene.owners[0].centroidInside )
	{
		// Inside the touched solid: push out through its outer faces only
		bestPiece = PushOut( &scene, pieces, pieceCount, &best, config->linearTolerance );
		if ( bestPiece < 0 )
		{
			// Fully enclosed, keep Box2D's manifold
			context->stats.unchanged += 1;
			return;
		}

		context->stats.kept += 1;
		pushedOut = true;
	}

	bool covered[TOPO_MAX_EDGES * 2];
	for ( int i = 0; i < pieceCount; ++i )
	{
		covered[i] = IsCovered( &scene, pieces, pieceCount, i, config->linearTolerance );
	}

	for ( int i = 0; i < pieceCount && pushedOut == false; ++i )
	{
		if ( pieces[i].edgeIndex >= scene.touchedEdgeCount || covered[i] )
		{
			continue;
		}

		b2ChainSegment cs;
		cs.segment.point1 = pieces[i].a;
		cs.segment.point2 = pieces[i].b;
		cs.ghost1 = FindGhost( &scene, pieces, covered, pieceCount, i, true, config->linearTolerance );
		cs.ghost2 = FindGhost( &scene, pieces, covered, pieceCount, i, false, config->linearTolerance );
		cs.chainId = B2_NULL_INDEX;

		b2LocalManifold local = CollidePiece( &cs, playerShapeId, xf );
		if ( local.pointCount == 0 )
		{
			continue;
		}

		float separation = MinSeparation( &local );
		if ( separation < bestSeparation )
		{
			bestSeparation = separation;
			best = local;
			bestPiece = i;
		}
	}

	if ( bestPiece < 0 )
	{
		manifold->pointCount = 0;
		context->stats.culled += 1;
		return;
	}

	// Marshal frame T to the pre-solve convention: world normal from A to B, anchors relative to
	// each body origin (Box2D shifts them to the center of mass after pre-solve)
	b2Vec2 normalT = b2RotateVector( frameT.q, best.normal );
	b2Vec2 originDelta = b2SubPos( frameT.p, framePlayer.p );
	manifold->normal = playerIsA ? b2Neg( normalT ) : normalT;
	manifold->pointCount = best.pointCount;
	for ( int i = 0; i < best.pointCount; ++i )
	{
		b2ManifoldPoint* mp = manifold->points + i;
		b2Vec2 fromT = b2RotateVector( frameT.q, best.points[i].point );
		b2Vec2 fromPlayer = b2Add( fromT, originDelta );
		mp->anchorA = playerIsA ? fromPlayer : fromT;
		mp->anchorB = playerIsA ? fromT : fromPlayer;
		mp->separation = best.points[i].separation;

		// Separate id space per piece so warm starting does not match unrelated features
		// Keyed by the touched edge and its interval, which are stable step to step
		int edgeKey = ( ( pieces[bestPiece].edgeIndex & 0xf ) << 3 ) | ( pieces[bestPiece].slot & 0x7 );
		edgeKey |= pushedOut ? 0x40 : 0;
		mp->id = (uint16_t)( 0x8000 | ( edgeKey << 8 ) | ( best.points[i].id & 0xff ) );
		mp->normalImpulse = 0.0f;
		mp->tangentImpulse = 0.0f;
		mp->totalNormalImpulse = 0.0f;
		mp->normalVelocity = 0.0f;
		mp->persisted = false;
	}

	context->stats.replaced += 1;
}

bool TopoGhostPreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* userContext )
{
	TopoGhostContext* context = userContext;

	bool playerIsA = B2_ID_EQUALS( b2Shape_GetBody( shapeIdA ), context->playerId );
	bool playerIsB = B2_ID_EQUALS( b2Shape_GetBody( shapeIdB ), context->playerId );
	if ( playerIsA == playerIsB )
	{
		return true;
	}

	b2ShapeId playerShapeId = playerIsA ? shapeIdA : shapeIdB;
	b2ShapeId touchedId = playerIsA ? shapeIdB : shapeIdA;
	b2ShapeType playerType = b2Shape_GetType( playerShapeId );
	if ( playerType != b2_polygonShape && playerType != b2_circleShape && playerType != b2_capsuleShape )
	{
		return true;
	}

	// Continuous collision runs after the solve while other threads write body poses, so only the
	// pre-step snapshot is used, and the topology is judged at the pre-step pose (the pose the
	// player swept from), not at the unclamped end pose
	const TopoGhostShape* touched = FindSnapshot( context, touchedId );
	if ( touched == NULL )
	{
		return true;
	}

	b2WorldTransform frameT = touched->transform;
	TopoScene scene;
	BuildScene( &scene, context, touched, playerShapeId );
	if ( scene.overflow || scene.owners[0].centroidInside )
	{
		return true;
	}

	CancelInteriorEdges( &scene, &context->config );

	TopoPiece pieces[TOPO_MAX_EDGES * 2];
	float linTol = context->config.linearTolerance;
	int pieceCount = CollectPieces( &scene, pieces, TOPO_MAX_EDGES * 2, linTol );

	// Veto the time of impact only if it lands on the touched shape away from every surviving
	// piece, that is on cancelled (interior) portions only. Covering is a resting contact idea and
	// is not used here.
	b2Vec2 p = b2InvTransformPoint( (b2Transform){ b2Vec2_zero, frameT.q }, b2SubPos( point, frameT.p ) );
	float tolerance = 4.0f * linTol;
	bool onTouched = false;
	for ( int i = 0; i < scene.touchedEdgeCount; ++i )
	{
		const TopoEdge* e = scene.edges + i;
		if ( b2DistanceSquared( p, b2Lerp( e->p1, e->p2, b2ClampFloat( b2Dot( b2Sub( p, e->p1 ), b2Sub( e->p2, e->p1 ) ) /
																		b2LengthSquared( b2Sub( e->p2, e->p1 ) ),
																	0.0f, 1.0f ) ) ) < tolerance * tolerance )
		{
			onTouched = true;
			break;
		}
	}

	if ( onTouched == false )
	{
		return true;
	}

	// The hit is real if its normal matches a live face it lands on, or it lands on a live convex
	// corner (judged with the topological ghosts). Normal oriented from the touched shape to the player.
	b2Vec2 hitNormal = b2InvRotateVector( frameT.q, normal );
	if ( playerIsA )
	{
		hitNormal = b2Neg( hitNormal );
	}

	for ( int i = 0; i < pieceCount; ++i )
	{
		if ( pieces[i].edgeIndex >= scene.touchedEdgeCount )
		{
			continue;
		}

		b2Vec2 a = pieces[i].a;
		b2Vec2 b = pieces[i].b;
		b2Vec2 d = b2Sub( b, a );
		float t = b2ClampFloat( b2Dot( b2Sub( p, a ), d ) / b2LengthSquared( d ), 0.0f, 1.0f );
		if ( b2DistanceSquared( p, b2MulAdd( a, t, d ) ) > tolerance * tolerance )
		{
			continue;
		}

		b2Vec2 edge1 = b2Normalize( d );
		if ( b2Dot( b2RightPerp( edge1 ), hitNormal ) > 0.7f )
		{
			return true;
		}

		if ( b2DistanceSquared( p, a ) < tolerance * tolerance )
		{
			b2Vec2 ghost1 = FindGhost( &scene, pieces, NULL, pieceCount, i, true, linTol );
			if ( b2Cross( b2Normalize( b2Sub( a, ghost1 ) ), edge1 ) >= context->config.angularTolerance )
			{
				return true;
			}
		}

		if ( b2DistanceSquared( p, b ) < tolerance * tolerance )
		{
			b2Vec2 ghost2 = FindGhost( &scene, pieces, NULL, pieceCount, i, false, linTol );
			if ( b2Cross( edge1, b2Normalize( b2Sub( ghost2, b ) ) ) >= context->config.angularTolerance )
			{
				return true;
			}
		}
	}

	context->stats.vetoed += 1;
	return false;
}
