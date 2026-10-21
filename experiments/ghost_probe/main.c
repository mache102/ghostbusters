// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

// Headless sweep over the ghost collision scenarios in shared/ghost_scenarios.c.
// Prints one markdown table per scenario.
//
// usage: ghost_probe [scenario name substring]
//        ghost_probe --defaults
//        ghost_probe --trace <scenario> <variant> <mitigation> <sweep value>
//        ghost_probe --players <scenario> [<variant> <mitigation> <sweep value>]
//        ghost_probe --bench <scenario> <variant> <repeats>

#include "ghost_scenarios.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void SetSweepValue( GhostParams* params, GhostSweepField field, float value )
{
	switch ( field )
	{
		case ghostSweep_speed:
			params->speed = value;
			break;
		case ghostSweep_gravityScale:
			params->gravityScale = value;
			break;
		case ghostSweep_knob:
			params->knob = value;
			break;
	}
}

static const char* SweepName( const GhostScenario* scenario )
{
	switch ( scenario->sweepField )
	{
		case ghostSweep_speed:
			return "speed";
		case ghostSweep_gravityScale:
			return "gravity scale";
		default:
			return scenario->knobName;
	}
}

// Static: GhostRun holds the topological ghost step scene (hundreds of KB), too big for the stack
static GhostRun s_run;

// Run one configuration to the end. The result (metrics, per player metrics) stays valid until the next call.
static const GhostRun* Simulate( int scenarioIndex, GhostParams params )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.enableSleep = false;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	GhostRun* run = &s_run;
	GhostRun_Create( run, worldId, scenarioIndex, params );

	int maxSteps = g_ghostScenarios[scenarioIndex].maxSteps;
	for ( int i = 0; i < maxSteps && run->metrics.finished == false; ++i )
	{
		GhostRun_PreStep( run );
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		GhostRun_Update( run );
	}

	GhostRun_Destroy( run );
	b2DestroyWorld( worldId );
	return run;
}

static GhostMetrics RunOnce( int scenarioIndex, GhostParams params )
{
	return Simulate( scenarioIndex, params )->metrics;
}

static int FindScenario( const char* name )
{
	for ( int i = 0; i < g_ghostScenarioCount; ++i )
	{
		if ( strstr( g_ghostScenarios[i].name, name ) != NULL )
		{
			return i;
		}
	}
	return -1;
}

static const char* Verdict( const GhostScenario* scenario, int variant, const GhostMetrics* m );

// Rest of a trace line: pose, velocity and touching manifolds of a body
static void PrintBodyState( b2BodyId bodyId )
{
	b2Pos p = b2Body_GetPosition( bodyId );
	b2Vec2 v = b2Body_GetLinearVelocity( bodyId );
	float angle = b2Rot_GetAngle( b2Body_GetRotation( bodyId ) );
	printf( " %.4f %.4f %.3f %.3f %.3f %d", (float)p.x, (float)p.y, v.x, v.y, angle, b2Body_GetContactCapacity( bodyId ) );

	// Manifold normals (pointing away from the body) and point separations
	b2ContactData contacts[8];
	int contactCount = b2Body_GetContactData( bodyId, contacts, 8 );
	for ( int j = 0; j < contactCount; ++j )
	{
		const b2Manifold* m = &contacts[j].manifold;
		if ( m->pointCount == 0 )
		{
			continue;
		}

		b2Vec2 n = m->normal;
		if ( B2_ID_EQUALS( b2Shape_GetBody( contacts[j].shapeIdA ), bodyId ) == false )
		{
			n = b2Neg( n );
		}

		b2ShapeId other =
			B2_ID_EQUALS( b2Shape_GetBody( contacts[j].shapeIdA ), bodyId ) ? contacts[j].shapeIdB : contacts[j].shapeIdA;
		static const char* typeNames[] = { "circle", "capsule", "segment", "polygon", "chainseg" };
		static const char* bodyNames[] = { "static", "kinematic", "dynamic" };
		b2AABB bounds = b2Shape_GetBounds( other );
		printf( " | %s/%s x[%.2f,%.2f] y[%.2f,%.2f] n=(%.2f,%.2f)", typeNames[b2Shape_GetType( other )],
				bodyNames[b2Body_GetType( b2Shape_GetBody( other ) )], bounds.lowerBound.x, bounds.upperBound.x,
				bounds.lowerBound.y, bounds.upperBound.y, n.x, n.y );
		for ( int k = 0; k < m->pointCount; ++k )
		{
			printf( " s=%.4f", m->points[k].separation );
		}
	}
	printf( "\n" );
}

// Verdict of a player against the same player on the smooth reference surface: symptoms beyond
// what the reference shows, with the absolute thresholds of Verdict as margins
static const char* RelativeVerdict( const GhostMetrics* m, const GhostMetrics* ref )
{
	if ( m->fellThrough && ref->fellThrough == false )
	{
		return "FELL";
	}
	// Lift-off: any, when the reference stays grounded. Bodies that leave the ground on the reference
	// too (polygons tumbling over polyline corners) only when clearly longer in the air.
	if ( ref->liftOffCount == 0 ? m->liftOffCount > 0 : m->airborneSteps > ref->airborneSteps + ref->airborneSteps / 2 )
	{
		return "LIFTS OFF";
	}
	if ( m->finished == false && ref->finished )
	{
		return "STUCK";
	}
	if ( m->maxKick > ref->maxKick + 0.5f || m->maxHop > ref->maxHop + 0.1f )
	{
		return "bumpy";
	}
	if ( m->minSpeedRatio < ref->minSpeedRatio - 0.1f )
	{
		return "slowed";
	}
	return "ok";
}

// Per player metrics of the reference variant run without mitigation, when the scenario has one.
// Returns false otherwise.
static bool ReferencePlayers( int si, GhostParams params, GhostMetrics* out )
{
	const GhostScenario* scenario = g_ghostScenarios + si;
	int reference = scenario->referenceVariant != NULL ? scenario->referenceVariant( params.variant ) : -1;
	if ( reference < 0 )
	{
		return false;
	}

	params.variant = reference;
	params.mitigation = ghostMitigation_none;
	const GhostRun* run = Simulate( si, params );
	for ( int i = 0; i < run->playerCount; ++i )
	{
		out[i] = run->players[i].metrics;
	}
	return true;
}

// One line per player of a multi-body run. reference: per player metrics on the smooth reference
// surface, may be NULL.
static void PrintPlayers( const GhostScenario* scenario, int variant, const GhostRun* run, const GhostMetrics* reference )
{
	// Airborne columns only where the scenario measures them
	bool air = scenario->measureAirborne;
	printf( "| player | verdict | vs reference | done | time | minV | snags | hop | sink | kick | cull | fell |%s\n",
			air ? " air | lifts | ref lifts |" : "" );
	printf( "|---|---|---|---|---|---|---|---|---|---|---|---|%s\n", air ? "---|---|---|" : "" );
	for ( int i = 0; i < run->playerCount; ++i )
	{
		const GhostPlayer* player = run->players + i;
		const GhostMetrics* m = &player->metrics;
		const char* relative = reference != NULL ? RelativeVerdict( m, reference + i ) : "-";
		char done[32];
		if ( m->finished )
		{
			snprintf( done, sizeof( done ), "yes (%d)", m->steps );
		}
		else
		{
			snprintf( done, sizeof( done ), "no" );
		}

		printf( "| %s | %s | %s | %s | %.3f | %.3f | %d | %.4f | %.4f | %.3f | %d | %s |", player->label,
				Verdict( scenario, variant, m ), relative, done, m->idealProgress > 0.0f ? m->progress / m->idealProgress : 0.0f,
				m->minSpeedRatio == FLT_MAX ? 0.0f : m->minSpeedRatio, m->snagCount, m->maxHop, m->maxSink, m->maxKick,
				m->rejectedCount, m->fellThrough ? "**yes**" : "no" );
		if ( air && reference != NULL )
		{
			printf( " %d | %d | %d |", m->airborneSteps, m->liftOffCount, reference[i].liftOffCount );
		}
		else if ( air )
		{
			printf( " %d | %d | - |", m->airborneSteps, m->liftOffCount );
		}
		printf( "\n" );
	}
}

static void RunPlayers( int si, int variant, int mitigation, float sweepValue )
{
	const GhostScenario* scenario = g_ghostScenarios + si;
	GhostParams params = GhostDefaultParams( si );
	params.variant = variant;
	params.mitigation = (GhostMitigation)mitigation;
	SetSweepValue( &params, scenario->sweepField, sweepValue );

	// The reference run reuses the run storage, so it goes first
	GhostMetrics reference[GHOST_MAX_PLAYERS];
	bool hasReference = ReferencePlayers( si, params, reference );

	const GhostRun* run = Simulate( si, params );
	printf( "#### %s / %s / %s / %s %g\n\n", scenario->name, scenario->variantNames[variant],
			GhostMitigationName( (GhostMitigation)mitigation ), SweepName( scenario ), sweepValue );
	if ( run->playerCount == 0 )
	{
		printf( "single player scenario, see the main table\n\n" );
		return;
	}

	PrintPlayers( scenario, variant, run, hasReference ? reference : NULL );
	printf( "\n" );
}

// Per player results: one configuration, or every variant x mitigation x sweep value when variant < 0
static int Players( const char* name, int variant, int mitigation, float sweepValue )
{
	int si = FindScenario( name );
	if ( si < 0 )
	{
		printf( "no scenario matching '%s'\n", name );
		return 1;
	}

	if ( variant >= 0 )
	{
		RunPlayers( si, variant, mitigation, sweepValue );
		return 0;
	}

	const GhostScenario* scenario = g_ghostScenarios + si;
	for ( int vi = 0; vi < scenario->variantCount; ++vi )
	{
		for ( int mi = 0; mi < ghostMitigation_count; ++mi )
		{
			if ( scenario->mitigationMask & GHOST_MITIGATE( mi ) )
			{
				for ( int wi = 0; wi < scenario->sweepCount; ++wi )
				{
					RunPlayers( si, vi, mi, scenario->sweep[wi] );
				}
			}
		}
	}

	return 0;
}

// Print the player state every step for one configuration
static int Trace( const char* name, int variant, int mitigation, float sweepValue )
{
	int si = FindScenario( name );
	if ( si < 0 )
	{
		printf( "no scenario matching '%s'\n", name );
		return 1;
	}

	GhostParams params = GhostDefaultParams( si );
	params.variant = variant;
	params.mitigation = (GhostMitigation)mitigation;
	SetSweepValue( &params, g_ghostScenarios[si].sweepField, sweepValue );

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.enableSleep = false;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Static: GhostRun holds the topological ghost step scene (hundreds of KB), too big for the stack
	static GhostRun run;
	GhostRun_Create( &run, worldId, si, params );

	if ( run.playerCount > 0 )
	{
		printf( "step player kick hop x y vx vy angle contacts\n" );
	}
	else
	{
		printf( "step x y vx vy angle contacts\n" );
	}
	for ( int i = 0; i < g_ghostScenarios[si].maxSteps && run.metrics.finished == false; ++i )
	{
		GhostRun_PreStep( &run );
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		GhostRun_Update( &run );

		if ( run.playerCount == 0 )
		{
			printf( "%d", i + 1 );
			PrintBodyState( run.playerId );
			continue;
		}

		// Multi-body: one line per moving player, with its running max kick and hop
		for ( int j = 0; j < run.playerCount; ++j )
		{
			const GhostPlayer* player = run.players + j;
			if ( player->launched == false || b2Body_IsEnabled( player->bodyId ) == false )
			{
				continue;
			}

			printf( "%d %s kick=%.3f hop=%.4f", i + 1, player->label, player->metrics.maxKick, player->metrics.maxHop );
			PrintBodyState( player->bodyId );
		}
	}

	if ( run.playerCount > 0 )
	{
		GhostMetrics reference[GHOST_MAX_PLAYERS];
		bool hasReference = ReferencePlayers( si, params, reference );
		printf( "\n" );
		PrintPlayers( g_ghostScenarios + si, variant, &run, hasReference ? reference : NULL );
	}

	GhostRun_Destroy( &run );
	b2DestroyWorld( worldId );
	return 0;
}

// Time world stepping for one scenario variant with and without topological ghosts
static int Bench( const char* name, int variant, int repeats )
{
	int si = FindScenario( name );
	if ( si < 0 )
	{
		printf( "no scenario matching '%s'\n", name );
		return 1;
	}

	GhostMitigation mitigations[2] = { ghostMitigation_none, ghostMitigation_topological };
	for ( int mi = 0; mi < 2; ++mi )
	{
		float milliseconds = 0.0f;
		int steps = 0;
		for ( int r = 0; r < repeats; ++r )
		{
			GhostParams params = GhostDefaultParams( si );
			params.variant = variant;
			params.mitigation = mitigations[mi];

			b2WorldDef worldDef = b2DefaultWorldDef();
			worldDef.enableSleep = false;
			b2WorldId worldId = b2CreateWorld( &worldDef );

			// Static: too big for the stack, see RunOnce
			static GhostRun run;
			GhostRun_Create( &run, worldId, si, params );

			uint64_t ticks = b2GetTicks();
			for ( int i = 0; i < g_ghostScenarios[si].maxSteps; ++i )
			{
				GhostRun_PreStep( &run );
				b2World_Step( worldId, 1.0f / 60.0f, 4 );
			}
			milliseconds += b2GetMilliseconds( ticks );
			steps += g_ghostScenarios[si].maxSteps;

			GhostRun_Destroy( &run );
			b2DestroyWorld( worldId );
		}

		printf( "%s / %s / %s: %.4f ms per step (%d steps)\n", g_ghostScenarios[si].name,
				g_ghostScenarios[si].variantNames[variant], GhostMitigationName( mitigations[mi] ), milliseconds / steps, steps );
	}

	return 0;
}

// One word verdict for a run: what a viewer of the sample would notice
static const char* Verdict( const GhostScenario* scenario, int variant, const GhostMetrics* m )
{
	if ( scenario->judgeManually )
	{
		return "see trace";
	}

	if ( scenario->blockedVariants & ( 1u << variant ) )
	{
		if ( m->fellThrough )
		{
			return "FELL";
		}
		return m->finished ? "PASSED THROUGH" : "ok (blocked)";
	}

	if ( m->fellThrough )
	{
		return "FELL";
	}
	if ( scenario->measureAirborne && m->liftOffCount > 0 )
	{
		return "LIFTS OFF";
	}
	if ( m->finished == false )
	{
		return "STUCK";
	}
	if ( m->maxKick > 0.5f || m->maxHop > 0.1f )
	{
		return "bumpy";
	}
	if ( m->minSpeedRatio < 0.9f )
	{
		return "slowed";
	}
	return "ok";
}

// Verdict of a measureAirborne scenario run judged against the reference variant without mitigation
// at the same parameters (its surface polyline has real corners that cost speed): lift-offs the
// reference does not show first, then the other symptoms. Multi-body runs: the most severe verdict
// of any player against the same player on the reference. The absolute verdict without a
// reference. Overwrites s_run.
static const char* AirborneVerdict( int si, GhostParams params, const GhostMetrics* m )
{
	const GhostScenario* scenario = g_ghostScenarios + si;
	int reference = scenario->referenceVariant != NULL ? scenario->referenceVariant( params.variant ) : -1;
	if ( m->fellThrough || reference < 0 )
	{
		return Verdict( scenario, params.variant, m );
	}

	// Multi-body: the most severe per player verdict (each player against itself on the reference)
	GhostMetrics players[GHOST_MAX_PLAYERS];
	if ( ReferencePlayers( si, params, players ) )
	{
		const GhostRun* run = Simulate( si, params );
		static const char* severity[] = { "FELL", "LIFTS OFF", "STUCK", "bumpy", "slowed" };
		const char* worst = "ok";
		int worstRank = 5;
		for ( int i = 0; i < run->playerCount; ++i )
		{
			const char* verdict = RelativeVerdict( &run->players[i].metrics, players + i );
			for ( int r = 0; r < worstRank; ++r )
			{
				if ( strcmp( verdict, severity[r] ) == 0 )
				{
					worst = verdict;
					worstRank = r;
					break;
				}
			}
		}
		if ( run->playerCount > 0 )
		{
			return worst;
		}
	}

	params.variant = reference;
	params.mitigation = ghostMitigation_none;
	GhostMetrics ref = RunOnce( si, params );
	return RelativeVerdict( m, &ref );
}

// Every scenario variant at its default parameters, none vs semrau vs topological
static int Defaults( void )
{
	GhostMitigation mitigations[3] = { ghostMitigation_none, ghostMitigation_semrau, ghostMitigation_topological };
	printf( "| scenario | variant | none | semrau | topological |\n|---|---|---|---|---|\n" );
	for ( int si = 0; si < g_ghostScenarioCount; ++si )
	{
		const GhostScenario* scenario = g_ghostScenarios + si;
		for ( int vi = 0; vi < scenario->variantCount; ++vi )
		{
			// The star marks the variant the sample opens with
			printf( "| %s | %s%s |", scenario->name, scenario->variantNames[vi], vi == scenario->defaultVariant ? " *" : "" );
			for ( int mi = 0; mi < 3; ++mi )
			{
				if ( ( scenario->mitigationMask & GHOST_MITIGATE( mitigations[mi] ) ) == 0 )
				{
					printf( " - |" );
					continue;
				}

				GhostParams params = GhostDefaultParams( si );
				params.variant = vi;
				params.mitigation = mitigations[mi];
				GhostMetrics m = RunOnce( si, params );
				if ( scenario->measureAirborne )
				{
					printf( " %s (kick %.2f, minV %.2f, lifts %d, air %d) |", AirborneVerdict( si, params, &m ), m.maxKick,
							m.minSpeedRatio > 1e30f ? 0.0f : m.minSpeedRatio, m.liftOffCount, m.airborneSteps );
					continue;
				}
				printf( " %s (kick %.2f, minV %.2f) |", Verdict( scenario, vi, &m ), m.maxKick, m.minSpeedRatio > 1e30f ? 0.0f : m.minSpeedRatio );
			}
			printf( "\n" );
		}
	}
	return 0;
}

int main( int argc, char** argv )
{
	if ( argc == 2 && strcmp( argv[1], "--defaults" ) == 0 )
	{
		return Defaults();
	}

	// ghost_probe --bench <scenario> <variant> <repeats>
	if ( argc == 5 && strcmp( argv[1], "--bench" ) == 0 )
	{
		return Bench( argv[2], atoi( argv[3] ), atoi( argv[4] ) );
	}

	// ghost_probe --trace <scenario> <variant> <mitigation> <sweep value>
	if ( argc == 6 && strcmp( argv[1], "--trace" ) == 0 )
	{
		return Trace( argv[2], atoi( argv[3] ), atoi( argv[4] ), (float)atof( argv[5] ) );
	}

	// Multi-body scenarios, one line per player:
	// ghost_probe --players <scenario> [<variant> <mitigation> <sweep value>]
	if ( argc == 6 && strcmp( argv[1], "--players" ) == 0 )
	{
		return Players( argv[2], atoi( argv[3] ), atoi( argv[4] ), (float)atof( argv[5] ) );
	}
	if ( argc == 3 && strcmp( argv[1], "--players" ) == 0 )
	{
		return Players( argv[2], -1, 0, 0.0f );
	}

	const char* filter = argc > 1 ? argv[1] : NULL;

	printf( "Ghost collision probe. 60 Hz, 4 substeps, gravity -10 * gravity scale.\n\n" );
	printf( "Columns: done = reached end of path (steps), time = progress / (launch speed * time), minV = min tangential "
			"speed / launch speed, snags = steps losing >10%% tangential speed, hop/sink = max height above/below rest (m), "
			"kick = max velocity along the surface normal (m/s), cull = contacts culled by the mitigation.\n\n" );

	for ( int si = 0; si < g_ghostScenarioCount; ++si )
	{
		const GhostScenario* scenario = g_ghostScenarios + si;
		if ( filter != NULL && strstr( scenario->name, filter ) == NULL )
		{
			continue;
		}

		printf( "### %s\n\n%s\n\n", scenario->name, scenario->summary );
		// Airborne columns (verdict, airborne steps, lift-offs) only where the scenario measures them
		bool air = scenario->measureAirborne;
		printf( "| variant | mitigation | %s | done | time | minV | snags | hop | sink | kick | cull | fell |%s\n",
				SweepName( scenario ), air ? " verdict | air | lifts |" : "" );
		printf( "|---|---|---|---|---|---|---|---|---|---|---|---|%s\n", air ? "---|---|---|" : "" );

		for ( int vi = 0; vi < scenario->variantCount; ++vi )
		{
			for ( int mi = 0; mi < ghostMitigation_count; ++mi )
			{
				if ( ( scenario->mitigationMask & GHOST_MITIGATE( mi ) ) == 0 )
				{
					continue;
				}

				for ( int wi = 0; wi < scenario->sweepCount; ++wi )
				{
					GhostParams params = GhostDefaultParams( si );
					params.variant = vi;
					params.mitigation = (GhostMitigation)mi;
					SetSweepValue( &params, scenario->sweepField, scenario->sweep[wi] );

					GhostMetrics m = RunOnce( si, params );
					float timeRatio = m.idealProgress > 0.0f ? m.progress / m.idealProgress : 0.0f;

					char done[32];
					if ( m.finished )
					{
						snprintf( done, sizeof( done ), "yes (%d)", m.steps );
					}
					else
					{
						snprintf( done, sizeof( done ), "no" );
					}

					char minV[32];
					if ( m.minSpeedRatio == FLT_MAX )
					{
						snprintf( minV, sizeof( minV ), "-" );
					}
					else
					{
						snprintf( minV, sizeof( minV ), "%.3f", m.minSpeedRatio );
					}

					printf( "| %s | %s | %g | %s | %.3f | %s | %d | %.4f | %.4f | %.3f | %d | %s |", scenario->variantNames[vi],
							GhostMitigationName( (GhostMitigation)mi ), scenario->sweep[wi], done, timeRatio, minV,
							m.snagCount, m.maxHop, m.maxSink, m.maxKick, m.rejectedCount, m.fellThrough ? "**yes**" : "no" );
					if ( air )
					{
						printf( " %s | %d | %d |", AirborneVerdict( si, params, &m ), m.airborneSteps, m.liftOffCount );
					}
					printf( "\n" );
				}
			}
		}

		printf( "\n" );
	}

	return 0;
}
