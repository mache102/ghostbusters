// SPDX-FileCopyrightText: 2026 mache102
// SPDX-License-Identifier: MIT

// Ghost collision challenge scenarios from shared/ghost_scenarios.c. The same scenarios are swept
// headlessly by experiments/ghost_probe. See notes/ghost_collisions.md.

#include "ghost_scenarios.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <imgui.h>

#include <stdio.h>

template <int ScenarioIndex>
class GhostCollisionSample : public Sample
{
public:
	explicit GhostCollisionSample( SampleContext* context )
		: Sample( context )
	{
		if ( s_initialized == false )
		{
			s_params = GhostDefaultParams( ScenarioIndex );
			s_initialized = true;
		}

		if ( m_context->restart == false )
		{
			m_context->camera.zoom = 8.0f;
		}

		m_relaunch = false;
		Launch();
	}

	~GhostCollisionSample() override
	{
		GhostRun_Destroy( &m_run );
	}

	void Launch()
	{
		GhostRun_Create( &m_run, m_worldId, ScenarioIndex, s_params );
		if ( s_follow )
		{
			b2Pos p = b2Body_GetPosition( FollowedBody() );
			m_context->camera.center = { p.x, p.y + 1.0f };
		}
	}

	// Multi-body scenarios: the first player still moving, else the last one
	b2BodyId FollowedBody() const
	{
		for ( int i = 0; i < m_run.playerCount; ++i )
		{
			const GhostPlayer& player = m_run.players[i];
			if ( player.launched && player.metrics.finished == false )
			{
				return player.bodyId;
			}
		}
		return m_run.playerCount > 0 ? m_run.players[m_run.playerCount - 1].bodyId : m_run.playerId;
	}

	bool DrawControls() override
	{
		const GhostScenario& scenario = g_ghostScenarios[ScenarioIndex];
		bool changed = false;

		ImGui::PushItemWidth( 12.0f * ImGui::GetFontSize() );
		changed |= ImGui::Combo( "Variant", &s_params.variant, scenario.variantNames, scenario.variantCount );

		// Before / after: switch between no mitigation, Semrau and topological ghosts
		const char* mitigations[ghostMitigation_count];
		int mitigationValues[ghostMitigation_count];
		int mitigationCount = 0;
		int current = 0;
		for ( int i = 0; i < ghostMitigation_count; ++i )
		{
			if ( scenario.mitigationMask & GHOST_MITIGATE( i ) )
			{
				if ( i == (int)s_params.mitigation )
				{
					current = mitigationCount;
				}
				mitigations[mitigationCount] = GhostMitigationName( (GhostMitigation)i );
				mitigationValues[mitigationCount] = i;
				mitigationCount += 1;
			}
		}

		if ( ImGui::Combo( "Mitigation", &current, mitigations, mitigationCount ) )
		{
			s_params.mitigation = (GhostMitigation)mitigationValues[current];
			changed = true;
		}

		ImGui::PopItemWidth();
		ImGui::PushItemWidth( 8.0f * ImGui::GetFontSize() );
		changed |= ImGui::SliderFloat( "Speed", &s_params.speed, 0.0f, 100.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Gravity scale", &s_params.gravityScale, 0.0f, 5000.0f, "%.1f",
									   ImGuiSliderFlags_Logarithmic );
		if ( scenario.knobName[0] != 'u' )
		{
			changed |= ImGui::InputFloat( scenario.knobName, &s_params.knob, 0.0f, 0.0f, "%.4f" );
		}
		ImGui::PopItemWidth();

		ImGui::Checkbox( "Follow", &s_follow );

		if ( ImGui::Button( "Relaunch" ) || changed )
		{
			m_relaunch = true;
		}

		if ( ImGui::Button( "Defaults" ) )
		{
			s_params = GhostDefaultParams( ScenarioIndex );
			m_relaunch = true;
		}

		return true;
	}

	void Step() override
	{
		if ( m_relaunch )
		{
			m_relaunch = false;
			GhostRun_Destroy( &m_run );
			CreateWorld();
			m_mouseJointId = b2_nullJointId;
			Launch();
		}

		GhostRun_PreStep( &m_run );
		Sample::Step();

		if ( m_didStep )
		{
			GhostRun_Update( &m_run );
		}

		if ( s_follow )
		{
			b2Pos p = b2Body_GetPosition( FollowedBody() );
			m_context->camera.center = { p.x, p.y + 1.0f };
		}

		const GhostScenario& scenario = g_ghostScenarios[ScenarioIndex];
		const GhostMetrics& m = m_run.metrics;
		DrawScreenTextLine( "%s", scenario.summary );
		DrawScreenTextLine( "variant: %s, mitigation: %s", scenario.variantNames[s_params.variant],
							GhostMitigationName( s_params.mitigation ) );
		DrawScreenTextLine( "progress %.2f / %.2f (time ratio %.3f)%s", m.progress, m_run.pathLength,
							m.idealProgress > 0.0f ? m.progress / m.idealProgress : 0.0f, m.finished ? " finished" : "" );
		DrawScreenTextLine( "min speed ratio %.3f, snags %d", m.minSpeedRatio > 1e30f ? 1.0f : m.minSpeedRatio, m.snagCount );
		DrawScreenTextLine( "max hop %.4f, max sink %.4f, max kick %.3f", m.maxHop, m.maxSink, m.maxKick );
		DrawScreenTextLine( "culled contacts %d%s", m.rejectedCount, m.fellThrough ? ", FELL THROUGH" : "" );
		if ( scenario.measureAirborne )
		{
			DrawScreenTextLine( "airborne steps %d, lift-offs %d", m.airborneSteps, m.liftOffCount );
		}

		// Multi-body scenarios: the lines above show the worst player, then one line per player
		for ( int i = 0; i < m_run.playerCount; ++i )
		{
			const GhostPlayer& player = m_run.players[i];
			const GhostMetrics& pm = player.metrics;
			const char* state = player.launched == false ? "waiting" : ( pm.finished ? "finished" : "moving" );
			char air[32] = "";
			if ( scenario.measureAirborne )
			{
				snprintf( air, sizeof( air ), " lift-offs %d", pm.liftOffCount );
			}
			DrawScreenTextLine( "%-8s %-8s kick %.2f hop %.3f minV %.2f snags %d progress %.1f%s%s", player.label, state,
								pm.maxKick, pm.maxHop, pm.minSpeedRatio > 1e30f ? 1.0f : pm.minSpeedRatio, pm.snagCount,
								pm.progress, air, pm.fellThrough ? " FELL" : "" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostCollisionSample( context );
	}

	// The run is referenced by the pre-solve callbacks, so it lives in the sample
	GhostRun m_run;
	bool m_relaunch;

	static inline GhostParams s_params;
	static inline bool s_initialized = false;
	static inline bool s_follow = true;
};

template <int ScenarioIndex>
static int RegisterGhostCollision()
{
	return RegisterSample( "Ghost Collisions", g_ghostScenarios[ScenarioIndex].name,
						   GhostCollisionSample<ScenarioIndex>::Create );
}

// Keep in sync with g_ghostScenarios
static int sampleGhost0 = RegisterGhostCollision<0>();
static int sampleGhost1 = RegisterGhostCollision<1>();
static int sampleGhost2 = RegisterGhostCollision<2>();
static int sampleGhost3 = RegisterGhostCollision<3>();
static int sampleGhost4 = RegisterGhostCollision<4>();
static int sampleGhost5 = RegisterGhostCollision<5>();
static int sampleGhost6 = RegisterGhostCollision<6>();
static int sampleGhost7 = RegisterGhostCollision<7>();
static int sampleGhost8 = RegisterGhostCollision<8>();
static int sampleGhost9 = RegisterGhostCollision<9>();
static int sampleGhost10 = RegisterGhostCollision<10>();
static int sampleGhost11 = RegisterGhostCollision<11>();
static int sampleGhost12 = RegisterGhostCollision<12>();
static int sampleGhost13 = RegisterGhostCollision<13>();
static int sampleGhost14 = RegisterGhostCollision<14>();
static int sampleGhost15 = RegisterGhostCollision<15>();
static int sampleGhost16 = RegisterGhostCollision<16>();
static int sampleGhost17 = RegisterGhostCollision<17>();
static int sampleGhost18 = RegisterGhostCollision<18>();
static int sampleGhost19 = RegisterGhostCollision<19>();
static int sampleGhost20 = RegisterGhostCollision<20>();
static int sampleGhost21 = RegisterGhostCollision<21>();
