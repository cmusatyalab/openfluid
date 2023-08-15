

class Speaker : public Scene
{
public:

	Speaker(const char* name, float radius) : Scene(name), mRadius(radius) {}

	virtual void Initialize()
	{
		// const float mRadius = 0.1f;
		const float radius = mRadius;
		const float restDistance = radius*0.5f;

		int dx = int(ceilf(2.0f / (radius*0.5f)));
		int dy = int(ceilf(3.0f / (radius*0.5f)));
		int dz = int(ceilf(1.0f / (radius*0.5f)));

		CreateParticleGrid(Vec3(0.0f, (radius*0.5f), 0.0f), dx, dy, dz, restDistance, Vec3(0.0f), 0.1f, false, 0.0f, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid), restDistance*0.01f);

		g_solverDesc.featureMode = eNvFlexFeatureModeSimpleFluids;
		
		g_sceneLower = Vec3(0.0f, 0.0f, -0.75f);
		g_sceneUpper = Vec3(3.0f, 3.0f, -0.75f);

		g_numSubsteps = 2;

		g_params.radius = radius;
		g_params.dynamicFriction = 0.001f;
		g_params.restitution = 0.001f;
		g_params.viscosity = 0.01f;
		g_params.numIterations = 2;
		g_params.vorticityConfinement = 75.0f;
		g_params.fluidRestDistance = restDistance;
		g_params.relaxationFactor = 1.0f;
		g_params.smoothing = 0.5f;
		g_params.diffuseThreshold *= 0.25f;
		g_params.cohesion = 0.05f;

		g_maxDiffuseParticles = 0;		
		g_diffuseScale = 0.5f;
		g_params.diffuseBallistic = (int)(16 / (1.0f / radius));
		g_params.diffuseBuoyancy = 1.0f;
		g_params.diffuseDrag = 1.0f;

		g_emitters[0].mEnabled = false;

		g_params.numPlanes = 6;

		// limit velocity to CFL condition
		g_params.maxSpeed = 0.5f*radius*g_numSubsteps / g_dt;

		// g_maxDiffuseParticles = 0;
		g_maxDiffuseParticles = 64*1024;

		g_numExtraParticles = 80*1024;

		// g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.0f);

		g_waveFrequency = 1.0f;
		g_waveAmplitude = 2.0f;
		g_waveFloorTilt = 0.0f;

		// draw options		
		// g_drawPoints = true;
		// g_drawEllipsoids = false;
		// g_drawDiffuse = true;
		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawDiffuse = true;
	}

	float mRadius;
};




// class DamBreak : public Scene
// {
// public:

// 	DamBreak(const char* name, float radius) : Scene(name), mRadius(radius) {}

// 	virtual void Initialize()
// 	{
// 		const float radius = mRadius;
// 		const float restDistance = mRadius*0.65f;

// 		int dx = int(ceilf(1.0f / restDistance));
// 		int dy = int(ceilf(2.0f / restDistance));
// 		int dz = int(ceilf(1.0f / restDistance));

// 		CreateParticleGrid(Vec3(0.0f, restDistance, 0.0f), dx, dy, dz, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid), restDistance*0.01f);

// 		g_sceneLower = Vec3(0.0f, 0.0f, -0.5f);
// 		g_sceneUpper = Vec3(3.0f, 0.0f, -0.5f);

// 		g_numSubsteps = 2;

// 		g_params.radius = radius;
// 		g_params.fluidRestDistance = restDistance;
// 		g_params.dynamicFriction = 0.f;
// 		g_params.restitution = 0.001f;

// 		g_params.numIterations = 3;
// 		g_params.relaxationFactor = 1.0f;

// 		g_params.smoothing = 0.4f;
		
// 		g_params.viscosity = 0.001f;
// 		g_params.cohesion = 0.1f;
// 		g_params.vorticityConfinement = 80.0f;
// 		g_params.surfaceTension = 0.0f;

// 		g_params.numPlanes = 5;

// 		// limit velocity to CFL condition
// 		g_params.maxSpeed = 0.5f*radius*g_numSubsteps / g_dt;

// 		g_maxDiffuseParticles = 0;

// 		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.0f);

// 		g_waveFrequency = 1.0f;
// 		g_waveAmplitude = 2.0f;
// 		g_waveFloorTilt = 0.0f;

// 		// draw options		
// 		g_drawPoints = true;
// 		g_drawEllipsoids = false;
// 		g_drawDiffuse = true;
// 	}

// 	float mRadius;
// };
