
// class Viscosity : public Scene
// {
// public:

// 	Viscosity(const char* name, float viscosity = 1.0f, float dissipation = 0.0f) : Scene(name), viscosity(viscosity), dissipation(dissipation) {}

// 	virtual void Initialize()
// 	{
// 		float radius = 0.1f;
// 		float restDistance = radius*0.5f;

// 		g_solverDesc.featureMode = eNvFlexFeatureModeSimpleFluids;

// 		g_params.radius = radius;

// 		g_params.numIterations = 3;
// 		g_params.vorticityConfinement = 0.0f;
// 		g_params.fluidRestDistance = restDistance;
// 		g_params.smoothing = 0.35f;
// 		g_params.relaxationFactor = 1.f;
// 		g_params.restitution = 0.0f;
// 		g_params.collisionDistance = 0.00125f;
// 		g_params.shapeCollisionMargin = g_params.collisionDistance*0.25f;
// 		g_params.dissipation = dissipation;

// 		g_params.gravity[1] *= 2.0f;

// 		g_fluidColor = Vec4(1.0f, 1.0f, 1.0f, 0.0f);
// 		g_meshColor = Vec3(0.7f, 0.8f, 0.9f)*0.7f;

// 		g_params.dynamicFriction = 1.0f;
// 		g_params.staticFriction = 0.0f;
// 		g_params.viscosity = 20.0f + 20.0f*viscosity;
// 		g_params.adhesion = 0.1f*viscosity;
// 		g_params.cohesion = 0.05f*viscosity;
// 		g_params.surfaceTension = 0.0f;

// 		const float shapeSize = 2.0f;
// 		const Vec3 shapeLower = Vec3(-shapeSize*0.5f, 0.0f, -shapeSize*0.5f);
// 		const Vec3 shapeUpper = shapeLower + Vec3(shapeSize);
// 		const Vec3 shapeCenter = (shapeLower + shapeUpper)*0.5f;

// 		NvFlexDistanceFieldId sdf = CreateSDF(GetFilePathByPlatform("Flex/data/bunny.ply").c_str(), 128);
// 		AddSDF(sdf, shapeLower, Quat(), shapeSize);

// 		float emitterSize = 1.f;

// 		Emitter e;
// 		e.mEnabled = true;
// 		e.mWidth = int(emitterSize / restDistance);
// 		e.mPos = Vec3(shapeCenter.x - 0.2f, shapeUpper.y + 0.75f, shapeCenter.z);
// 		e.mDir = Vec3(0.0f, -1.0f, 0.0f);
// 		e.mRight = Vec3(1.0f, 0.0f, 0.0f);
// 		e.mSpeed = (restDistance*2.f / g_dt);

// 		g_sceneUpper.z = 5.0f;

// 		g_emitters.push_back(e);

// 		g_numExtraParticles = 64 * 1024;

// 		g_lightDistance *= 2.5f;

// 		// draw options		
// 		g_drawEllipsoids = true;

// 		g_emit = true;
// 		g_pause = false;
// 	}

// 	virtual void DoGui()
// 	{
// 		imguiSlider("Emitter Pos", &g_emitters.back().mPos.x, -1.0f, 1.0f, 0.001f);
// 	}

// 	float viscosity;
// 	float dissipation;
// };



class Viscosity : public Scene
{
public:

	Viscosity(const char* name, float viscosity = 1.0f, float dissipation = 0.0f) : Scene(name), viscosity(viscosity), dissipation(dissipation){}

	void Initialize()
	{
		float radius = 0.1f; // 10cm
		float restDistance = radius*0.5f;

		// convex rocks
		float minSize = 0.1f;
		float maxSize = 0.5f;

		for (int i=0; i < 3; i++)
			for (int j=0; j < 2; j++)
			AddRandomConvex(10, Vec3(20*radius*0.5f + i*maxSize*2.0f, 0.0f, j*maxSize*2.0f), minSize, maxSize, Vec3(0.0f, 1.0f, 0.0f), Randf(0.0f, k2Pi));

		CreateParticleGrid(Vec3(0.0f, radius*0.5f, -1.0f), 32, 32, 32, radius*0.55f, Vec3(0.0f), 2.0f, false, 0.0f, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid), 0.005f);


		g_sceneUpper = Vec3(-FLT_MAX, 2.0f, -FLT_MAX);
		g_solverDesc.featureMode = eNvFlexFeatureModeSimpleFluids;

		g_numSubsteps = 2;

		g_params.radius = radius;
		// g_params.dynamicFriction = 0.01f;
		g_params.dynamicFriction = 1.0f;
		g_params.staticFriction = 0.1f;
		// g_params.viscosity = 0.01f;
		g_params.viscosity = 40.0f + 20.0f*viscosity;
		g_params.numIterations = 3;
		g_params.adhesion = 0.018f*viscosity;
		
		// g_params.vorticityConfinement = 75.0f;
		g_params.vorticityConfinement = 0.0f;
		g_params.fluidRestDistance = restDistance;
		g_params.relaxationFactor = 1.0f;
		// g_params.smoothing = 0.5f;
		g_params.smoothing = 0.35f;
		// g_params.diffuseThreshold *= 0.25f;
		g_params.restitution = 0.0f;
		// g_params.cohesion = 0.05f;
		g_params.cohesion = 0.06f*viscosity;

		g_params.shapeCollisionMargin = g_params.collisionDistance*0.25f;

		g_maxDiffuseParticles = 0;		

		g_emitters[0].mEnabled = false;

		g_params.numPlanes = 6;

		g_waveFloorTilt = 0.0f;
		g_waveFrequency = 1.5f;
		g_waveAmplitude = 2.0f;
		
		// draw options		
		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawDiffuse = true;
		g_lightDistance = 1.8f;

		g_numExtraParticles = 80*1024;

		// g_fluidColor = Vec4(1.0f, 0.75f, 0.0f, 0.9f);
		g_fluidColor = Vec4(1.0f, 1.0f, 1.0f, 1.0f);
		// g_fluidColor = Vec4(255.0f/255.0f, 0.0f/255.0f, 127.0/255.0f, 1.0f);
		g_fluidColor = Vec4(227.0f/255.0f, 11.0f/255.0f, 93.0/255.0f, 0.9f);
		g_fluidColor = Vec4(243.0f/255.0f, 227.0f/255.0f, 207.0/255.0f, 0.0f);
		// g_fluidColor = Vec4(247.0f/255.0f, 145.0f/255.0f, 1.0/255.0f, 0.9f);
	}

	void CenterCamera() {
		// g_camPos = Vec3((g_sceneLower.x + g_sceneUpper.x)*0.5f, min(g_sceneUpper.y*1.25f, 6.0f), g_sceneUpper.z + min(g_sceneUpper.y, 6.0f)*4.5f);
		
		float scene_w = (g_sceneUpper.x - g_sceneLower.x);
		float cam_z = g_sceneUpper.z + (scene_w / g_relativeW) * 1630.0f;
		float cam_y = g_sceneUpper.y;
		// g_camPos = Vec3((g_sceneLower.x + g_sceneUpper.x)*0.5f, (g_sceneLower.y + g_sceneUpper.y)*0.5f, cam_z);
		// g_camPos = Vec3((g_sceneLower.x + g_sceneUpper.x)*0.5f, g_sceneUpper.y + cam_z * 0.1, cam_z);
		// g_camAngle = Vec3(0.0f, -DegToRad(5.0f), 0.0f);
		g_camPos = Vec3((g_sceneLower.x + g_sceneUpper.x)*0.5f, g_sceneUpper.y * 1.2, cam_z);
		g_camAngle = Vec3(0.0f, -ATan2(cam_y/2 * 1.2, cam_z), 0.0f);
	}
	float viscosity;
	float dissipation;
};
