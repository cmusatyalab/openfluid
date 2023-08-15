
class DamBreak : public Scene
{
public:

	DamBreak(const char* name, float radius) : Scene(name), mRadius(radius) {}

	virtual void Initialize()
	{
		const float radius = mRadius;
		const float restDistance = mRadius*0.65f;

		int dx = int(ceilf(1.75f / restDistance));
		int dy = int(ceilf(2.0f / restDistance));
		int dz = int(ceilf(1.25f / restDistance));

		CreateParticleGrid(Vec3(0.0f, restDistance, 0.0f), dx, dy, dz, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid), restDistance*0.01f);

		g_sceneLower = Vec3(0.0f, 0.0f, -1.25f);
		g_sceneUpper = Vec3(2.5f, 2.5f, 1.25f);

		g_numSubsteps = 2;

		g_params.radius = radius;
		g_params.fluidRestDistance = restDistance;
		g_params.dynamicFriction = 0.f;
		g_params.restitution = 0.001f;

		g_params.numIterations = 3;
		g_params.relaxationFactor = 1.0f;

		g_params.smoothing = 0.5f;
		
		g_params.viscosity = 0.001f;
		g_params.cohesion = 0.1f;
		g_params.vorticityConfinement = 80.0f;
		g_params.surfaceTension = 0.0f;

		g_params.numPlanes = 6;

		// limit velocity to CFL condition
		g_params.maxSpeed = 0.5f*radius*g_numSubsteps / g_dt;

		g_maxDiffuseParticles = 64*1024;
		g_diffuseScale = 0.5f;
		g_params.diffuseBallistic = (int)(16);
		g_params.diffuseBuoyancy = 1.0f;
		g_params.diffuseDrag = 1.0f;

		g_numExtraParticles = 80*1024;

		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.0f);

		g_waveFrequency = 1.0f;
		g_waveAmplitude = 2.0f;
		g_waveFloorTilt = 0.0f;

		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawDiffuse = true;
	}

	float mRadius;

	void CenterCamera() {
		float scene_w = (g_sceneUpper.x - g_sceneLower.x);
		float cam_z = g_sceneUpper.z + (scene_w / g_relativeW) * 1630.0f;
		float cam_y = (scene_w / g_relativeW) * 1630.0f * 0.2;

		g_camPos = Vec3((g_sceneLower.x + g_sceneUpper.x)*0.5f, cam_y + (g_sceneUpper.y / 2), cam_z);
		g_camAngle = Vec3(0.0f, -ATan2(cam_y , cam_z), 0.0f);

	}
};