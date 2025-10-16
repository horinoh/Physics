#include "RigidBody.h"
#include "Shape.h"

Physics::RigidBody::RigidBody(const Physics::Shape* Sh, const float InvMass)
	: Shape(Sh), InvMass(InvMass)
{
	if (0.0f != InvMass) {
		InertiaTensor = Shape->GetInertiaTensor() / InvMass;
	}
	//!< InvMass ‚ð‰Á–¡‚µ‚½‚à‚Ì (‰Á–¡‚µ‚È‚¢‚à‚Ì‚ÍƒVƒFƒCƒv‚©‚çŽæ“¾‚·‚é‚±‚Æ)
	InvInertiaTensor = Shape->GetInvInertiaTensor() * InvMass;
}

LinAlg::Vec3 Physics::RigidBody::GetCenterOfMass() const 
{
	return Shape->GetCenterOfMass(); 
};

void Physics::RigidBody::ApplyImpulse(const Collision::Contact& Ct)
{
	const auto& WPointA = Ct.WPointA;
	const auto& WPointB = Ct.WPointB;

	const auto TotalInvMass = Ct.RigidBodyA->InvMass + Ct.RigidBodyB->InvMass;
	{
		//!< ”¼Œa (dS -> Õ“Ë“_)
		const auto RA = WPointA - Ct.RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WPointB - Ct.RigidBodyB->GetWorldCenterOfMass();
		{
			//!< ‹tŠµ«ƒeƒ“ƒ\ƒ‹ (ƒ[ƒ‹ƒhƒXƒy[ƒX)
			const auto InvWITA = Ct.RigidBodyA->GetWorldInverseInertiaTensor();
			const auto InvWITB = Ct.RigidBodyB->GetWorldInverseInertiaTensor();

			//!< (A Ž‹“_‚Ì) ‘Š‘Î‘¬“x
			const auto VelA = Ct.RigidBodyA->LinearVelocity + Ct.RigidBodyA->AngularVelocity.Cross(RA);
			const auto VelB = Ct.RigidBodyB->LinearVelocity + Ct.RigidBodyB->AngularVelocity.Cross(RB);
			const auto RelVelA = VelA - VelB;

			//!< –@üAÚü•ûŒü‚Ì—ÍÏ J ‚ð“K—p‚·‚é‚Ì‹¤’Êˆ—
			/*
			* ‰^“®—Ê•Û‘¶‘¥‚æ‚è
			* v_a = v_a0 + J / m
			*
			* Šp‰^“®—Ê•Û‘¶‘¥‚æ‚è
			* w_a = w_a0 + I^-1 (r_a \cross n) J
			*
			* v_total = v_a + r_a \cross w_a
			*
			* J = \frac{Coef (v_b - v_a)}{(m_a^-1 + m_b^-1) + ((I_a^-1 r_a \cross n) \cross r_a + (I_b^-1 r_b \cross n) \cross r_b) n}
			* ‚±‚±‚Å
			*	’e«ŒW”‚Ìê‡ Coef = (1 + e)
			*	–€ŽCŒW”‚Ìê‡ Coef = \mu
			*
			* ‚±‚±‚ÅˆÈ‰º‚Ì‚æ‚¤‚É’u‚­‚Æ
			*	V = v_b - v_a
			*	M = (m_a^-1 + m_b^-1)
			*	J_a = (I_a^-1 r_a \cross n) \cross r_a
			*	J_b = (I_b^-1 r_b \cross n) \cross r_b
			* &= \frac{Coef V}{M + (J_a + J_b) n}
			*/
			auto Apply = [&](const auto& Axis, const auto& Vel, const float Coef) {
				const auto AngJA = (InvWITA * RA.Cross(Axis)).Cross(RA);
				const auto AngJB = (InvWITB * RB.Cross(Axis)).Cross(RB);
				const auto AngFactor = (AngJA + AngJB).Dot(Axis);
				const auto J = Vel * Coef / (TotalInvMass + AngFactor);
				Ct.RigidBodyA->ApplyImpulse(WPointA, -J);
				Ct.RigidBodyB->ApplyImpulse(WPointB, J);
				};

			//!< –@ü•ûŒü —ÍÏJ (‰^“®—Ê•Ï‰»)
			const auto& Nrm = Ct.WNormal;
			const auto VelN = Nrm * RelVelA.Dot(Nrm);
			//!< ‚±‚±‚Å‚Í—¼ŽÒ‚Ì’e«ŒW”‚ðŠ|‚¯‚½‚¾‚¯‚ÌŠÈˆÕ‚ÈŽÀ‘•‚Æ‚·‚é
			const auto TotalElas = 1.0f + Ct.RigidBodyA->Elasticity * Ct.RigidBodyB->Elasticity;
			Apply(Nrm, VelN, TotalElas);

			//!< Úü•ûŒü —ÍÏJ (–€ŽC—Í)
			const auto VelT = RelVelA - VelN;
			const auto Tan = VelT.Normalize();
			//!< ‚±‚±‚Å‚Í—¼ŽÒ‚Ì–€ŽCŒW”‚ðŠ|‚¯‚½‚¾‚¯‚ÌŠÈˆÕ‚ÈŽÀ‘•‚Æ‚·‚é
			const auto TotalFric = Ct.RigidBodyA->Friction * Ct.RigidBodyB->Friction;
			Apply(Tan, VelT, TotalFric);
		}
	}
}

void Physics::RigidBody::Update(const float DeltaSec)
{
	//!< Ž¿—Ê–³ŒÀˆµ‚¢‚¾‚ª“®‚­‚à‚Ì‚ª‚ ‚é‚Ì‚Å‘ŠúI—¹‚Å‚«‚È‚¢A“®‚©‚È‚¢‚±‚Æ‚ª•ª‚©‚Á‚Ä‚¢‚é‚Ì‚Å‚ ‚ê‚Î‰Â”\
	//if (0.0f == InvMass) {
	//	return;
	//}

	//!< (‘¬“x‚É‚æ‚é) ˆÊ’u‚ÌXV
	{
		Position += LinearVelocity * DeltaSec;
	}

	//!< (Šp‘¬“x‚É‚æ‚é) ˆÊ’uA‰ñ“]‚ÌXV
	{
		//!< ƒ[ƒ‹ƒh‹óŠÔ‚Ì (‹t) Šµ«ƒeƒ“ƒ\ƒ‹
		const auto InvWIT = GetWorldInverseInertiaTensor();
		const auto WIT = GetWorldInertiaTensor();

		/*
		* ƒgƒ‹ƒN \tau = I \alpha = r \cross F
		* Šp‰^“®—Ê L = I \omega = r \cross P = r \cross (I \omega)
		* 
		* \tau = I \alpha
		* \tau = \omega \cross (I \omega) 
		* ‚æ‚è
		* I \alpha = \omega \cross (I \omega)
		* \alpha = I^-1 (\omega \cross (I \omega))
		*/
		const auto AngAccel = InvWIT * (AngularVelocity.Cross(WIT * AngularVelocity));
		
		//!< Šp‘¬“x‚ÌXV
		AngularVelocity += AngAccel * DeltaSec;

		//!< ƒfƒ‹ƒ^Šp‘¬“x‚ÌŽlŒ³”•\Œ»
		const auto DeltaAng = AngularVelocity * DeltaSec;
		const auto DeltaQuat = LinAlg::Quat(DeltaAng, DeltaAng.Length());
		
		//!< ‰ñ“]‚ÌXV (‰ÁŽZ‚Å‚Í‚È‚­ŽlŒ³”‚ÌæŽZ)
		Rotation = (DeltaQuat * Rotation).Normalize();

		//!< (‰ñ“]‚É‚æ‚é) ˆÊ’u‚ÌXV
		const auto WorldCenter = GetWorldCenterOfMass();
		Position = WorldCenter + DeltaQuat.Rotate(Position - WorldCenter);
	}
}