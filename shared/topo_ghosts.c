// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

#include "topo_ghosts.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

// The topology is built in a "scene frame". The step scene (built once per step in
// TopoGhost_PreStep) uses world axes with the origin at the player's pre-step position. The
// per-contact fallback scene uses the frame of the touched shape's body ("frame T"), which is also
// the frame the chain segment colliders return their manifolds in. Step scene pieces and ghosts are
// moved to frame T before collision.

// Capacity of the per-contact fallback scene (on the callback's stack)
#define TOPO_MAX_LOCAL_OWNERS 32
#define TOPO_MAX_LOCAL_EDGES 192
#define TOPO_MAX_LOCAL_PIECES ( 2 * TOPO_MAX_LOCAL_EDGES )

// Ghost vertices to compute in ComputeTopology
#define TOPO_GHOSTS_NONE ( -2 )
#define TOPO_GHOSTS_ALL ( -1 )

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
	// The context is large, avoid a temporary
	memset( context, 0, sizeof( *context ) );
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
		context->bypass = true;
		return false;
	}

	if ( SnapshotShape( context->neighbors + context->neighborCount, shapeId ) )
	{
		context->neighborCount += 1;
	}

	return true;
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

static void InitScene( TopoScene* scene, TopoOwner* owners, int ownerCapacity, TopoEdge* edges, int edgeCapacity,
					   TopoPiece* pieces, bool* covered, int pieceCapacity, float penetrationTolerance )
{
	B2_ASSERT( ownerCapacity <= TOPO_MAX_NEIGHBORS );
	scene->owners = owners;
	scene->edges = edges;
	scene->pieces = pieces;
	scene->covered = covered;
	scene->ownerCapacity = ownerCapacity;
	scene->edgeCapacity = edgeCapacity;
	scene->pieceCapacity = pieceCapacity;
	scene->ownerCount = 0;
	scene->edgeCount = 0;
	scene->pieceCount = 0;
	scene->supportCount = 0;
	scene->penetrationTolerance = penetrationTolerance;
	scene->overflow = false;
}

static void AddEdge( TopoScene* scene, b2Vec2 p1, b2Vec2 p2, int owner )
{
	if ( b2DistanceSquared( p1, p2 ) < FLT_EPSILON )
	{
		return;
	}

	if ( scene->edgeCount == scene->edgeCapacity )
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

// A snapshot shape's collidable edges in the scene frame, outward normal on the right. Fills
// polygon (in the scene frame) for solid polygons. Returns the edge count.
static int ShapeEdges( const TopoGhostShape* shape, b2WorldTransform frame, b2Vec2* p1s, b2Vec2* p2s, b2Polygon* polygon )
{
	b2Transform xf = b2InvMulWorldTransforms( frame, shape->transform );
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
static bool PlayerFullyInside( const TopoPlayer* player, const b2Polygon* polygon )
{
	for ( int i = 0; i < polygon->count; ++i )
	{
		for ( int j = 0; j < player->pointCount; ++j )
		{
			float offset = b2Dot( polygon->normals[i], b2Sub( player->points[j], polygon->vertices[i] ) );
			if ( offset > -player->radius )
			{
				return false;
			}
		}
	}
	return true;
}

// Add a shape's collidable edges in the scene frame
static bool AddOwner( TopoScene* scene, const TopoGhostShape* shape, b2WorldTransform frame )
{
	if ( scene->ownerCount == scene->ownerCapacity )
	{
		scene->overflow = true;
		return false;
	}

	b2Vec2 p1s[TOPO_MAX_SHAPE_EDGES], p2s[TOPO_MAX_SHAPE_EDGES];
	b2Polygon polygon;
	int count = ShapeEdges( shape, frame, p1s, p2s, &polygon );

	int owner = scene->ownerCount;
	TopoOwner* o = scene->owners + owner;
	*o = (TopoOwner){ 0 };
	o->shape = shape;
	if ( shape->type == b2_polygonShape )
	{
		o->polygon = polygon;
		o->isSolid = true;
		o->centroidInside = b2PointInPolygon( &o->polygon, scene->player.centroid );
		o->fullyInside = o->centroidInside && PlayerFullyInside( &scene->player, &o->polygon );
	}

	o->firstEdge = scene->edgeCount;
	for ( int i = 0; i < count; ++i )
	{
		AddEdge( scene, p1s[i], p2s[i], owner );
	}
	o->edgeCount = scene->edgeCount - o->firstEdge;

	scene->ownerCount += 1;
	return true;
}

// How deep the player reaches behind a line (positive when penetrating)
static float PlayerDepth( const TopoPlayer* player, b2Vec2 point, b2Vec2 normal )
{
	float depth = -FLT_MAX;
	for ( int i = 0; i < player->pointCount; ++i )
	{
		depth = b2MaxFloat( depth, -b2Dot( normal, b2Sub( player->points[i], point ) ) );
	}
	return depth + player->radius;
}

// Record the surfaces of a shape that the player rests on: centroid in front and over the span,
// touching (within the speculative distance) and sunk less than the penetration tolerance
static void AddSupports( TopoScene* scene, const TopoGhostShape* shape, b2WorldTransform frame, const TopoGhostConfig* config )
{
	b2Vec2 p1s[TOPO_MAX_SHAPE_EDGES], p2s[TOPO_MAX_SHAPE_EDGES];
	b2Polygon polygon;
	int count = ShapeEdges( shape, frame, p1s, p2s, &polygon );
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
		b2Vec2 r = b2Sub( scene->player.centroid, p1s[i] );
		float along = b2Dot( u, r );
		if ( b2Dot( n, r ) <= config->linearTolerance || along < 0.0f || along > length )
		{
			continue;
		}

		float depth = PlayerDepth( &scene->player, p1s[i], n );
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

static bool AreAdjacent( const TopoScene* scene, int a, int b )
{
	return ( ( scene->adjacency[a][b >> 5] >> ( b & 31 ) ) & 1u ) != 0;
}

// Owners can only share edges, vertices or crossings when their snapshot bounds overlap. Box2D
// computes the bounds from the same snapshot poses, so the margin only covers rounding.
static void ComputeAdjacency( TopoScene* scene, float margin )
{
	int n = scene->ownerCount;
	for ( int a = 0; a < n; ++a )
	{
		memset( scene->adjacency[a], 0, sizeof( scene->adjacency[a] ) );
	}

	for ( int a = 0; a < n; ++a )
	{
		b2AABB boundsA = scene->owners[a].shape->bounds;
		for ( int b = a + 1; b < n; ++b )
		{
			if ( AABBsOverlap( boundsA, scene->owners[b].shape->bounds, margin ) )
			{
				scene->adjacency[a][b >> 5] |= 1u << ( b & 31 );
				scene->adjacency[b][a >> 5] |= 1u << ( a & 31 );
			}
		}
	}
}

// The owner itself, then its adjacent owners in order. Pieces of other owners cannot meet the
// owner's pieces. Returns the count.
static int CandidateOwners( const TopoScene* scene, int owner, int* list )
{
	int count = 0;
	list[count++] = owner;
	for ( int b = 0; b < scene->ownerCount; ++b )
	{
		if ( AreAdjacent( scene, owner, b ) )
		{
			list[count++] = b;
		}
	}
	return count;
}

// Phase 1: cancel portions of edges that coincide with an antiparallel edge of another owner.
// Only edges of adjacent owners can coincide.
static void CancelInteriorEdges( TopoScene* scene, const TopoGhostConfig* config )
{
	float linTol = config->linearTolerance;
	for ( int i = 0; i < scene->edgeCount; ++i )
	{
		TopoEdge* e = scene->edges + i;
		b2Vec2 d = b2Sub( e->p2, e->p1 );
		float length = b2Length( d );
		b2Vec2 u = b2MulSV( 1.0f / length, d );
		const TopoOwner* oe = scene->owners + e->owner;

		for ( int b = 0; b < scene->ownerCount; ++b )
		{
			if ( AreAdjacent( scene, e->owner, b ) == false )
			{
				continue;
			}

			// Embedded in the neighbor's solid: this face is the wall that keeps the player from
			// phasing on into this solid, keep it. Otherwise (on the surface, or only partly sunk)
			// the face is interior. Push-out uses only the surviving outer faces.
			const TopoOwner* of = scene->owners + b;
			if ( of->fullyInside && oe->fullyInside == false )
			{
				continue;
			}

			for ( int j = of->firstEdge; j < of->firstEdge + of->edgeCount; ++j )
			{
				const TopoEdge* f = scene->edges + j;
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
}

// Surviving pieces, grouped by owner
static void CollectPieces( TopoScene* scene, float linTol )
{
	int count = 0;
	for ( int o = 0; o < scene->ownerCount; ++o )
	{
		TopoOwner* owner = scene->owners + o;
		owner->firstPiece = count;
		owner->truncated = false;
		for ( int i = owner->firstEdge; i < owner->firstEdge + owner->edgeCount; ++i )
		{
			const TopoEdge* e = scene->edges + i;
			b2Vec2 d = b2Sub( e->p2, e->p1 );
			float length = b2Length( d );
			for ( int k = 0; k < e->intervalCount; ++k )
			{
				TopoInterval in = e->intervals[k];
				float t0 = b2MaxFloat( in.t0, 0.0f );
				float t1 = b2MinFloat( in.t1, 1.0f );
				if ( ( t1 - t0 ) * length < linTol )
				{
					continue;
				}

				if ( count == scene->pieceCapacity )
				{
					owner->truncated = true;
					scene->overflow = true;
					continue;
				}

				TopoPiece* piece = scene->pieces + count;
				piece->a = b2MulAdd( e->p1, t0, d );
				piece->b = b2MulAdd( e->p1, t1, d );
				piece->ghost1 = piece->a;
				piece->ghost2 = piece->b;
				piece->edgeIndex = i;
				piece->slot = k;
				scene->covered[count] = false;
				count += 1;
			}
		}
		owner->pieceCount = count - owner->firstPiece;
	}
	scene->pieceCount = count;
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

// A piece is covered when it meets another surface that the player is standing on (centroid in
// front of it and over its span) and lies behind that surface. The player cannot reach a covered
// piece without passing through a solid, so it is cancelled for this player, like an interior
// face. Beside the surface (e.g. falling past a cliff) the piece is live. Candidates are the pieces
// of the listed owners.
static bool IsCovered( const TopoScene* scene, const int* ownerList, int ownerListCount, int self, float linTol )
{
	const TopoPiece* s = scene->pieces + self;
	b2Vec2 centroid = scene->player.centroid;
	float linTolSqr = linTol * linTol;

	// Behind a support surface (not part of it): reachable only by sinking into the support
	b2Vec2 ns = b2RightPerp( b2Normalize( b2Sub( s->b, s->a ) ) );
	for ( int i = 0; i < scene->supportCount; ++i )
	{
		const TopoSupport* support = scene->supports + i;
		// Only steep faces (tile sides, undersides) are hidden by a support. A face within 60
		// degrees of the support's orientation is a walkable continuation (a convex crest of the
		// same surface): the player reaches it by moving along, not by sinking.
		if ( b2Dot( ns, support->normal ) > 0.5f )
		{
			continue;
		}

		if ( b2Dot( support->normal, b2Sub( s->a, support->point ) ) <= linTol &&
			 b2Dot( support->normal, b2Sub( s->b, support->point ) ) <= linTol )
		{
			return true;
		}
	}

	for ( int k = 0; k < ownerListCount; ++k )
	{
		const TopoOwner* owner = scene->owners + ownerList[k];
		for ( int i = owner->firstPiece; i < owner->firstPiece + owner->pieceCount; ++i )
		{
			if ( i == self )
			{
				continue;
			}

			const TopoPiece* c = scene->pieces + i;
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
			float t = b2Dot( b2Sub( centroid, c->a ), d ) / length;
			if ( t < 0.0f || t > length )
			{
				continue;
			}

			if ( InFront( c->a, c->b, centroid, linTol ) == false || Behind( c->a, c->b, far, linTol ) == false )
			{
				continue;
			}

			// A gentle continuation of c (within 60 degrees, a convex crest of the same surface) is
			// reached by moving along, not hidden behind c
			b2Vec2 nc = b2RightPerp( b2MulSV( 1.0f / length, d ) );
			if ( b2Dot( ns, nc ) > 0.5f )
			{
				continue;
			}

			// Only while the player actually rests on c (same depth band as the support rule). At a
			// convex crest the centroid passes over the downslope while the player still rests on the
			// flat top; cancelling the top there leaves nothing holding the crest.
			float depth = PlayerDepth( &scene->player, c->a, b2RightPerp( b2MulSV( 1.0f / length, d ) ) );
			if ( -B2_SPECULATIVE_DISTANCE <= depth && depth <= scene->penetrationTolerance )
			{
				return true;
			}
		}
	}

	return false;
}

// Phase 2: pick the ghost vertex at `vertex` for piece s. atStart selects ghost1 (vertex is s.a)
// or ghost2 (vertex is s.b). Candidates are the pieces of the listed owners, in list order.
static b2Vec2 FindGhost( const TopoScene* scene, const bool* covered, const int* ownerList, int ownerListCount, int self,
						 bool atStart, float linTol )
{
	const TopoPiece* s = scene->pieces + self;
	b2Vec2 centroid = scene->player.centroid;
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
	for ( int k = 0; k < ownerListCount; ++k )
	{
		const TopoOwner* owner = scene->owners + ownerList[k];
		for ( int i = owner->firstPiece; i < owner->firstPiece + owner->pieceCount; ++i )
		{
			// A covered piece cannot shield or continue anything for this player
			if ( i == self || ( covered != NULL && covered[i] ) )
			{
				continue;
			}

			const TopoPiece* c = scene->pieces + i;
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

			if ( InFront( c->a, c->b, centroid, linTol ) == false || Behind( c->a, c->b, far, linTol ) == false )
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

	for ( int k = 0; k < ownerListCount; ++k )
	{
		const TopoOwner* owner = scene->owners + ownerList[k];
		for ( int i = owner->firstPiece; i < owner->firstPiece + owner->pieceCount; ++i )
		{
			// A covered piece cannot shield or continue anything for this player
			if ( i == self || ( covered != NULL && covered[i] ) )
			{
				continue;
			}

			const TopoPiece* c = scene->pieces + i;
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
	}

	if ( found == false )
	{
		// Pieces passing through the vertex (T-junctions, partial overlaps)
		for ( int k = 0; k < ownerListCount; ++k )
		{
			const TopoOwner* owner = scene->owners + ownerList[k];
			for ( int i = owner->firstPiece; i < owner->firstPiece + owner->pieceCount; ++i )
			{
				if ( i == self )
				{
					continue;
				}

				const TopoPiece* c = scene->pieces + i;
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
	}

	if ( found == false )
	{
		// Nothing continues the surface: a convex corner
		b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( s->b, s->a ) ) );
		best = b2Sub( vertex, n );
	}

	return best;
}

// Cancellation, pieces, and optionally covered flags and ghost vertices (with covered pieces
// excluded) for the pieces of one owner or all owners.
static void ComputeTopology( TopoScene* scene, const TopoGhostConfig* config, bool computeCovered, int ghostOwner )
{
	float linTol = config->linearTolerance;
	ComputeAdjacency( scene, 2.0f * linTol );
	CancelInteriorEdges( scene, config );
	CollectPieces( scene, linTol );

	int ownerList[TOPO_MAX_NEIGHBORS + 1];
	if ( computeCovered )
	{
		for ( int o = 0; o < scene->ownerCount; ++o )
		{
			const TopoOwner* owner = scene->owners + o;
			if ( owner->pieceCount == 0 )
			{
				continue;
			}

			int listCount = CandidateOwners( scene, o, ownerList );
			for ( int i = owner->firstPiece; i < owner->firstPiece + owner->pieceCount; ++i )
			{
				scene->covered[i] = IsCovered( scene, ownerList, listCount, i, linTol );
			}
		}
	}

	if ( ghostOwner == TOPO_GHOSTS_NONE )
	{
		return;
	}

	const bool* covered = computeCovered ? scene->covered : NULL;
	int first = ghostOwner == TOPO_GHOSTS_ALL ? 0 : ghostOwner;
	int last = ghostOwner == TOPO_GHOSTS_ALL ? scene->ownerCount : ghostOwner + 1;
	for ( int o = first; o < last; ++o )
	{
		const TopoOwner* owner = scene->owners + o;
		if ( owner->pieceCount == 0 )
		{
			continue;
		}

		int listCount = CandidateOwners( scene, o, ownerList );
		for ( int i = owner->firstPiece; i < owner->firstPiece + owner->pieceCount; ++i )
		{
			// Covered pieces are never collided
			if ( covered != NULL && covered[i] )
			{
				continue;
			}

			TopoPiece* piece = scene->pieces + i;
			piece->ghost1 = FindGhost( scene, covered, ownerList, listCount, i, true, linTol );
			piece->ghost2 = FindGhost( scene, covered, ownerList, listCount, i, false, linTol );
		}
	}
}

// The player shape's points, radius and centroid in a scene frame. The shape's own centroid (not
// the body's) is used so compound players work.
static void SetPlayerGeometry( TopoPlayer* player, b2ShapeId playerShapeId, b2Transform xf )
{
	player->centroid = xf.p;
	player->pointCount = 0;
	player->radius = 0.0f;
	switch ( b2Shape_GetType( playerShapeId ) )
	{
		case b2_polygonShape:
		{
			b2Polygon polygon = b2Shape_GetPolygon( playerShapeId );
			for ( int i = 0; i < polygon.count; ++i )
			{
				player->points[i] = b2TransformPoint( xf, polygon.vertices[i] );
			}
			player->pointCount = polygon.count;
			player->radius = polygon.radius;
			player->centroid = b2TransformPoint( xf, polygon.centroid );
		}
		break;

		case b2_circleShape:
		{
			b2Circle circle = b2Shape_GetCircle( playerShapeId );
			player->points[0] = b2TransformPoint( xf, circle.center );
			player->pointCount = 1;
			player->radius = circle.radius;
			player->centroid = player->points[0];
		}
		break;

		case b2_capsuleShape:
		{
			b2Capsule capsule = b2Shape_GetCapsule( playerShapeId );
			player->points[0] = b2TransformPoint( xf, capsule.center1 );
			player->points[1] = b2TransformPoint( xf, capsule.center2 );
			player->pointCount = 2;
			player->radius = capsule.radius;
			player->centroid = b2Lerp( player->points[0], player->points[1], 0.5f );
		}
		break;

		default:
			break;
	}
}

static bool IsSupportedPlayerType( b2ShapeType type )
{
	return type == b2_polygonShape || type == b2_circleShape || type == b2_capsuleShape;
}

static bool AcceptNeighbor( const TopoGhostContext* context, b2ShapeId playerShapeId, b2Filter playerFilter,
							const TopoGhostShape* neighbor )
{
	if ( ShouldCollide( playerFilter, neighbor->filter ) == false )
	{
		return false;
	}

	return context->filterFcn == NULL || context->filterFcn( playerShapeId, neighbor->shapeId, context->filterContext );
}

// The topology of the whole neighborhood for one player shape, built once per step
static void BuildStepScene( TopoGhostContext* context, TopoStepScene* step, b2ShapeId playerShapeId )
{
	TopoScene* scene = &step->scene;
	InitScene( scene, step->owners, TOPO_MAX_NEIGHBORS, step->edges, TOPO_MAX_STEP_EDGES, step->pieces, step->covered,
			   TOPO_MAX_STEP_PIECES, context->config.penetrationTolerance );

	// World axes, origin at the player: float precision stays local to the player
	step->playerShapeId = playerShapeId;
	step->frame = (b2WorldTransform){ context->playerTransform.p, b2Rot_identity };
	SetPlayerGeometry( &scene->player, playerShapeId, b2InvMulWorldTransforms( step->frame, context->playerTransform ) );

	b2Filter playerFilter = b2Shape_GetFilter( playerShapeId );
	for ( int i = 0; i < context->neighborCount; ++i )
	{
		const TopoGhostShape* neighbor = context->neighbors + i;
		if ( AcceptNeighbor( context, playerShapeId, playerFilter, neighbor ) == false )
		{
			continue;
		}

		// Any nearby surface can support the player
		AddSupports( scene, neighbor, step->frame, &context->config );
		AddOwner( scene, neighbor, step->frame );
	}

	ComputeTopology( scene, &context->config, true, TOPO_GHOSTS_ALL );
	if ( scene->overflow )
	{
		context->stats.overflow += 1;
	}
}

void TopoGhost_PreStep( TopoGhostContext* context )
{
	context->neighborCount = 0;
	context->stepSceneCount = 0;
	context->bypass = false;
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
	if ( context->bypass )
	{
		// Truncated neighborhoods drop arbitrary shapes (possibly the ground under the body), which is
		// worse than native contacts with ghost collisions
		return;
	}

	// All callbacks of the step see these poses, so the topology is built once here
	b2ShapeId shapeIds[4 * TOPO_MAX_PLAYER_SHAPES];
	int shapeCount = b2Body_GetShapes( context->playerId, shapeIds, 4 * TOPO_MAX_PLAYER_SHAPES );
	for ( int i = 0; i < shapeCount && context->stepSceneCount < TOPO_MAX_PLAYER_SHAPES; ++i )
	{
		if ( b2Shape_IsSensor( shapeIds[i] ) || IsSupportedPlayerType( b2Shape_GetType( shapeIds[i] ) ) == false )
		{
			continue;
		}

		BuildStepScene( context, context->stepScenes + context->stepSceneCount, shapeIds[i] );
		context->stepSceneCount += 1;
	}
}

static const TopoStepScene* FindStepScene( const TopoGhostContext* context, b2ShapeId playerShapeId )
{
	for ( int i = 0; i < context->stepSceneCount; ++i )
	{
		if ( B2_ID_EQUALS( context->stepScenes[i].playerShapeId, playerShapeId ) )
		{
			return context->stepScenes + i;
		}
	}
	return NULL;
}

static int FindOwner( const TopoScene* scene, b2ShapeId shapeId )
{
	for ( int i = 0; i < scene->ownerCount; ++i )
	{
		if ( B2_ID_EQUALS( scene->owners[i].shape->shapeId, shapeId ) )
		{
			return i;
		}
	}
	return -1;
}

// Per-contact fallback: the local topology around the touched shape, in frame T
static void BuildLocalScene( TopoScene* scene, const TopoGhostContext* context, const TopoGhostShape* touched,
							 b2ShapeId playerShapeId )
{
	b2WorldTransform frameT = touched->transform;
	SetPlayerGeometry( &scene->player, playerShapeId, b2InvMulWorldTransforms( frameT, context->playerTransform ) );

	AddOwner( scene, touched, frameT );

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

		if ( AcceptNeighbor( context, playerShapeId, playerFilter, neighbor ) == false )
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

// Minimum translation out of the touched solid along one of its surviving (outer) faces. Pieces
// are moved from the scene frame to frame T by xfTS, the player is in frame T.
// Returns the piece index, or -1 if there is none.
static int PushOut( const TopoPlayer* player, const TopoScene* scene, int owner, b2Transform xfTS, b2LocalManifold* manifold,
					float linTol )
{
	const TopoOwner* o = scene->owners + owner;
	int bestPiece = -1;
	float bestDepth = FLT_MAX;
	for ( int i = o->firstPiece; i < o->firstPiece + o->pieceCount; ++i )
	{
		b2Vec2 a = b2TransformPoint( xfTS, scene->pieces[i].a );
		b2Vec2 b = b2TransformPoint( xfTS, scene->pieces[i].b );
		b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( b, a ) ) );
		float depth = PlayerDepth( player, a, n );
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
	b2Vec2 a = b2TransformPoint( xfTS, scene->pieces[bestPiece].a );
	b2Vec2 n = b2RightPerp( b2Normalize( b2Sub( b2TransformPoint( xfTS, scene->pieces[bestPiece].b ), a ) ) );
	*manifold = (b2LocalManifold){ 0 };
	manifold->normal = n;
	for ( int i = 0; i < player->pointCount && manifold->pointCount < 2; ++i )
	{
		b2Vec2 v = player->points[i];
		float depth = -b2Dot( n, b2Sub( v, a ) ) + player->radius;
		if ( depth < bestDepth - linTol )
		{
			continue;
		}

		b2LocalManifoldPoint* mp = manifold->points + manifold->pointCount;
		mp->point = b2MulSub( v, player->radius, n );
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

// Rebuild the manifold of the player against a touched owner of a scene. frameT is the touched
// shape's (pre-step) body transform, xfTS maps the scene frame to frame T. Reads the scene only.
static void ReplaceManifold( TopoGhostContext* context, const TopoScene* scene, int owner, b2Transform xfTS,
							 b2WorldTransform frameT, b2ShapeId playerShapeId, bool playerIsA, b2Manifold* manifold )
{
	const TopoOwner* o = scene->owners + owner;
	if ( o->truncated )
	{
		// Some real faces of the touched shape are missing, keep Box2D's manifold
		context->stats.unchanged += 1;
		return;
	}

	float linTol = context->config.linearTolerance;
	b2WorldTransform framePlayer = context->playerTransform;
	b2Transform xf = b2InvMulWorldTransforms( frameT, framePlayer );
	b2LocalManifold best = { 0 };
	int bestPiece = -1;
	float bestSeparation = FLT_MAX;
	bool pushedOut = false;

	if ( o->centroidInside )
	{
		// Inside the touched solid: push out through its outer faces only
		TopoPlayer playerT;
		SetPlayerGeometry( &playerT, playerShapeId, xf );
		bestPiece = PushOut( &playerT, scene, owner, xfTS, &best, linTol );
		if ( bestPiece < 0 )
		{
			// Fully enclosed, keep Box2D's manifold
			context->stats.unchanged += 1;
			return;
		}

		context->stats.kept += 1;
		pushedOut = true;
	}

	for ( int i = o->firstPiece; i < o->firstPiece + o->pieceCount && pushedOut == false; ++i )
	{
		if ( scene->covered[i] )
		{
			continue;
		}

		const TopoPiece* piece = scene->pieces + i;
		b2ChainSegment cs;
		cs.segment.point1 = b2TransformPoint( xfTS, piece->a );
		cs.segment.point2 = b2TransformPoint( xfTS, piece->b );
		cs.ghost1 = b2TransformPoint( xfTS, piece->ghost1 );
		cs.ghost2 = b2TransformPoint( xfTS, piece->ghost2 );
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

	// Separate id space per piece so warm starting does not match unrelated features. Keyed by the
	// touched shape's edge and its interval, which are stable step to step.
	const TopoPiece* bp = scene->pieces + bestPiece;
	int edgeKey = ( ( ( bp->edgeIndex - o->firstEdge ) & 0xf ) << 3 ) | ( bp->slot & 0x7 );
	edgeKey |= pushedOut ? 0x40 : 0;

	for ( int i = 0; i < best.pointCount; ++i )
	{
		b2ManifoldPoint* mp = manifold->points + i;
		b2Vec2 fromT = b2RotateVector( frameT.q, best.points[i].point );
		b2Vec2 fromPlayer = b2Add( fromT, originDelta );
		mp->anchorA = playerIsA ? fromPlayer : fromT;
		mp->anchorB = playerIsA ? fromT : fromPlayer;
		mp->separation = best.points[i].separation;
		mp->id = (uint16_t)( 0x8000 | ( edgeKey << 8 ) | ( best.points[i].id & 0xff ) );
		mp->normalImpulse = 0.0f;
		mp->tangentImpulse = 0.0f;
		mp->totalNormalImpulse = 0.0f;
		mp->normalVelocity = 0.0f;
		mp->persisted = false;
	}

	context->stats.replaced += 1;
}

void TopoGhostPreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* userContext )
{
	TopoGhostContext* context = userContext;
	if ( context->bypass )
	{
		return;
	}

	bool playerIsA = B2_ID_EQUALS( b2Shape_GetBody( shapeIdA ), context->playerId );
	bool playerIsB = B2_ID_EQUALS( b2Shape_GetBody( shapeIdB ), context->playerId );
	if ( playerIsA == playerIsB )
	{
		return;
	}

	b2ShapeId playerShapeId = playerIsA ? shapeIdA : shapeIdB;
	b2ShapeId touchedId = playerIsA ? shapeIdB : shapeIdA;
	if ( IsSupportedPlayerType( b2Shape_GetType( playerShapeId ) ) == false )
	{
		context->stats.unchanged += 1;
		return;
	}

	// Pre-solve sees the pre-step poses, so the step topology applies as is
	const TopoStepScene* step = FindStepScene( context, playerShapeId );
	int owner = step != NULL ? FindOwner( &step->scene, touchedId ) : -1;
	if ( owner >= 0 )
	{
		b2WorldTransform frameT = step->scene.owners[owner].shape->transform;
		b2Transform xfTS = b2InvMulWorldTransforms( frameT, step->frame );
		ReplaceManifold( context, &step->scene, owner, xfTS, frameT, playerShapeId, playerIsA, manifold );
		return;
	}

	// Fallback: build the local topology for this contact. A shape that entered range without being
	// snapshotted can be read live here (collide does not move bodies).
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

	TopoOwner owners[TOPO_MAX_LOCAL_OWNERS];
	TopoEdge edges[TOPO_MAX_LOCAL_EDGES];
	TopoPiece pieces[TOPO_MAX_LOCAL_PIECES];
	bool covered[TOPO_MAX_LOCAL_PIECES];
	TopoScene scene;
	InitScene( &scene, owners, TOPO_MAX_LOCAL_OWNERS, edges, TOPO_MAX_LOCAL_EDGES, pieces, covered, TOPO_MAX_LOCAL_PIECES,
			   context->config.penetrationTolerance );
	BuildLocalScene( &scene, context, touched, playerShapeId );
	ComputeTopology( &scene, &context->config, true, 0 );
	if ( scene.overflow )
	{
		context->stats.overflow += 1;
	}

	// The scene is in frame T already
	ReplaceManifold( context, &scene, 0, b2Transform_identity, touched->transform, playerShapeId, playerIsA, manifold );
}

// Judge a time of impact on a touched owner of a scene, at the pre-step pose. Reads the scene only.
static bool JudgeContinuous( TopoGhostContext* context, const TopoScene* scene, int owner, b2WorldTransform frame, b2Pos point,
							 b2Vec2 normal, bool playerIsA )
{
	const TopoOwner* o = scene->owners + owner;
	float linTol = context->config.linearTolerance;

	// Veto the time of impact only if it lands on the touched shape away from every surviving
	// piece, that is on cancelled (interior) portions only. Covering is a resting contact idea and
	// is not used here.
	b2Vec2 p = b2InvTransformPoint( (b2Transform){ b2Vec2_zero, frame.q }, b2SubPos( point, frame.p ) );
	float tolerance = 4.0f * linTol;
	bool onTouched = false;
	for ( int i = o->firstEdge; i < o->firstEdge + o->edgeCount; ++i )
	{
		const TopoEdge* e = scene->edges + i;
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
	// corner (judged with the topological ghosts, covering ignored). Normal oriented from the
	// touched shape to the player.
	b2Vec2 hitNormal = b2InvRotateVector( frame.q, normal );
	if ( playerIsA )
	{
		hitNormal = b2Neg( hitNormal );
	}

	int ownerList[TOPO_MAX_NEIGHBORS + 1];
	int listCount = CandidateOwners( scene, owner, ownerList );

	for ( int i = o->firstPiece; i < o->firstPiece + o->pieceCount; ++i )
	{
		b2Vec2 a = scene->pieces[i].a;
		b2Vec2 b = scene->pieces[i].b;
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
			b2Vec2 ghost1 = FindGhost( scene, NULL, ownerList, listCount, i, true, linTol );
			if ( b2Cross( b2Normalize( b2Sub( a, ghost1 ) ), edge1 ) >= context->config.angularTolerance )
			{
				return true;
			}
		}

		if ( b2DistanceSquared( p, b ) < tolerance * tolerance )
		{
			b2Vec2 ghost2 = FindGhost( scene, NULL, ownerList, listCount, i, false, linTol );
			if ( b2Cross( edge1, b2Normalize( b2Sub( ghost2, b ) ) ) >= context->config.angularTolerance )
			{
				return true;
			}
		}
	}

	context->stats.vetoed += 1;
	return false;
}

bool TopoGhostPreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* userContext )
{
	TopoGhostContext* context = userContext;
	if ( context->bypass )
	{
		return true;
	}

	bool playerIsA = B2_ID_EQUALS( b2Shape_GetBody( shapeIdA ), context->playerId );
	bool playerIsB = B2_ID_EQUALS( b2Shape_GetBody( shapeIdB ), context->playerId );
	if ( playerIsA == playerIsB )
	{
		return true;
	}

	b2ShapeId playerShapeId = playerIsA ? shapeIdA : shapeIdB;
	b2ShapeId touchedId = playerIsA ? shapeIdB : shapeIdA;
	if ( IsSupportedPlayerType( b2Shape_GetType( playerShapeId ) ) == false )
	{
		return true;
	}

	// Continuous collision runs after the solve while other threads write body poses, so only the
	// pre-step snapshot is used, and the topology is judged at the pre-step pose (the pose the
	// player swept from), not at the unclamped end pose
	const TopoStepScene* step = FindStepScene( context, playerShapeId );
	int owner = step != NULL ? FindOwner( &step->scene, touchedId ) : -1;
	if ( owner >= 0 )
	{
		if ( step->scene.overflow || step->scene.owners[owner].centroidInside )
		{
			return true;
		}

		return JudgeContinuous( context, &step->scene, owner, step->frame, point, normal, playerIsA );
	}

	// Fallback: build the local topology for this event
	const TopoGhostShape* touched = FindSnapshot( context, touchedId );
	if ( touched == NULL )
	{
		return true;
	}

	TopoOwner owners[TOPO_MAX_LOCAL_OWNERS];
	TopoEdge edges[TOPO_MAX_LOCAL_EDGES];
	TopoPiece pieces[TOPO_MAX_LOCAL_PIECES];
	bool covered[TOPO_MAX_LOCAL_PIECES];
	TopoScene scene;
	InitScene( &scene, owners, TOPO_MAX_LOCAL_OWNERS, edges, TOPO_MAX_LOCAL_EDGES, pieces, covered, TOPO_MAX_LOCAL_PIECES,
			   context->config.penetrationTolerance );
	BuildLocalScene( &scene, context, touched, playerShapeId );
	if ( scene.overflow || scene.owners[0].centroidInside )
	{
		return true;
	}

	ComputeTopology( &scene, &context->config, false, TOPO_GHOSTS_NONE );
	return JudgeContinuous( context, &scene, 0, touched->transform, point, normal, playerIsA );
}
