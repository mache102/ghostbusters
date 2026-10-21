// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

// Headless sweep over the ghost collision scenarios in shared/ghost_scenarios.c.
// Prints one markdown table per scenario.
//
// usage: ghost_probe [scenario name substring]

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

static GhostMetrics RunOnce( int scenarioIndex, GhostParams params )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.enableSleep = false;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Static: GhostRun holds the topological ghost step scene (hundreds of KB), too big for the stack
	static GhostRun run;
	GhostRun_Create( &run, worldId, scenarioIndex, params );

	int maxSteps = g_ghostScenarios[scenarioIndex].maxSteps;
	for ( int i = 0; i < maxSteps && run.metrics.finished == false; ++i )
	{
		GhostRun_PreStep( &run );
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		GhostRun_Update( &run );
	}

	b2DestroyWorld( worldId );
	return run.metrics;
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

	printf( "step x y vx vy angle contacts\n" );
	for ( int i = 0; i < g_ghostScenarios[si].maxSteps && run.metrics.finished == false; ++i )
	{
		GhostRun_PreStep( &run );
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		GhostRun_Update( &run );

		b2Pos p = b2Body_GetPosition( run.playerId );
		b2Vec2 v = b2Body_GetLinearVelocity( run.playerId );
		float angle = b2Rot_GetAngle( b2Body_GetRotation( run.playerId ) );
		printf( "%d %.4f %.4f %.3f %.3f %.3f %d", i + 1, (float)p.x, (float)p.y, v.x, v.y, angle,
				b2Body_GetContactCapacity( run.playerId ) );

		// Manifold normals (pointing away from the player) and point separations
		b2ContactData contacts[8];
		int contactCount = b2Body_GetContactData( run.playerId, contacts, 8 );
		for ( int j = 0; j < contactCount; ++j )
		{
			const b2Manifold* m = &contacts[j].manifold;
			if ( m->pointCount == 0 )
			{
				continue;
			}

			b2Vec2 n = m->normal;
			if ( B2_ID_EQUALS( b2Shape_GetBody( contacts[j].shapeIdA ), run.playerId ) == false )
			{
				n = b2Neg( n );
			}

			b2ShapeId other = B2_ID_EQUALS( b2Shape_GetBody( contacts[j].shapeIdA ), run.playerId ) ? contacts[j].shapeIdB
																								  : contacts[j].shapeIdA;
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
		printf( "| variant | mitigation | %s | done | time | minV | snags | hop | sink | kick | cull | fell |\n",
				SweepName( scenario ) );
		printf( "|---|---|---|---|---|---|---|---|---|---|---|---|\n" );

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

					printf( "| %s | %s | %g | %s | %.3f | %s | %d | %.4f | %.4f | %.3f | %d | %s |\n", scenario->variantNames[vi],
							GhostMitigationName( (GhostMitigation)mi ), scenario->sweep[wi], done, timeRatio, minV,
							m.snagCount, m.maxHop, m.maxSink, m.maxKick, m.rejectedCount, m.fellThrough ? "**yes**" : "no" );
				}
			}
		}

		printf( "\n" );
	}

	return 0;
}
