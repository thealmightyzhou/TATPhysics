#pragma once

#include "../TATCommon/TATransformUtil.h"

class TATSolverBody
{
public:
	TATVector3		m_LinVel;
	TATVector3		m_AngVel;

	TAT_REGISTER_ATTRIBUTE(TATransform, WorldTransform);
	TAT_REGISTER_ATTRIBUTE(TATVector3, InvMass);
	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, DeltaAngVel);
	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, DeltaLinVel);
	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, PushVel);
	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, TurnVel);
	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, LinFactor);
	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, AngFactor);

	union 
	{
		void* m_OriginalBody;
		int m_OriginalBodyIndex;
	};

	inline TATVector3 GetVelocityInLocalPt(const TATVector3& relPos) const
	{
		return m_LinVel + m_DeltaLinVel + (m_AngVel + m_DeltaAngVel).Cross(relPos);
	}

	inline TATVector3 GetAngularVelocity() const
	{
		return m_AngVel + m_DeltaAngVel;
	}

	inline void ApplyImpulse(const TATVector3& linearComponent, const TATVector3& angularComponent, const float& impulseMagnitude)
	{
		m_DeltaLinVel += linearComponent * impulseMagnitude * m_LinFactor;
		m_DeltaAngVel += angularComponent * (impulseMagnitude * m_AngFactor);
	}

	inline void ApplyPushImpulse(const TATVector3& linearComponent, const TATVector3& angularComponent, const float& impulseMagnitude)
	{
		m_PushVel += linearComponent * impulseMagnitude * m_LinFactor;
		m_TurnVel += angularComponent * (impulseMagnitude * m_AngFactor);
	}

	void WriteBackVelocity()
	{
		m_LinVel += m_DeltaLinVel;
		m_AngVel += m_DeltaAngVel;
	}

	void WriteBackVelocityAndTransform(float timeStep, float splitImpulseTurnErp)
	{
		m_LinVel += m_DeltaLinVel;
		m_AngVel += m_DeltaAngVel;

		TATransform newTr;
		if (!m_PushVel.IsZero() || !m_PushVel.IsZero())
		{
			TATransformUtil::IntegrateTransform(m_WorldTransform, m_PushVel, m_TurnVel * splitImpulseTurnErp, timeStep, newTr);
			m_WorldTransform = newTr;
		}
	}
};