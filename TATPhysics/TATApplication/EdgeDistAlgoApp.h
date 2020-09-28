#pragma once
#pragma once
#include "TATApplication.h"
#include "../TATGeometry/TATGeometryComputer.h"
#include "TAThread.h"
#include "../TATStage/TATActor.h"
#include "../TATNarrowPhase/TATSAT.h"

class EdgeDistAlgoApp :public TATApplication
{
public:
	EdgeDistAlgoApp(const TString& name) :TATApplication(name)
	{}

	virtual void Initialize()
	{
		__super::Initialize();

		TATGLEntry::Instance()->Initialize(1080, 720);

		m_MainCamera->SetPosition(TATVector3(0, 0, -20));
		m_MainCamera->SetDirection(TATVector3(0, 0, 1));
		m_MainCamera->SetSpeed(100);
	}

	virtual void CreateScene() 
	{
		TATMesh* cube = TATResourceManager::Instance()->LoadMesh("cube.obj");
		TATMesh* hs = TATResourceManager::Instance()->LoadMesh("halfSphere.obj");
		TATMesh* py = TATResourceManager::Instance()->LoadMesh("pyramid.obj");
		cube->Initialize();
		hs->Initialize();
		py->Initialize();

		TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("LightTemplate.tmaterial");
		mat->Initialize();
	
		TATActor* cubeActor = new TATActor(cube);
		cubeActor->Initialize();
		cubeActor->SetMaterial(mat);
		cubeActor->SetUseTransform(true);
		TATStageNode* node3 = m_RootNode->CreateChild("cube");
		node3->MountActor(cubeActor);
		TATRigidBody* rb = TATDynamicWorld::Instance()->CreateConvex(cube, 0.1f);
		rb->Initialize();
		rb->m_ControllByUser = true;
		TATDynamicWorld::Instance()->InitRigidBody(rb, TATransform::GetIdentity(), 0.1f, 0.8, 0.2, TATVector3(0, 0, 0));
		cubeActor->AttachTickable(rb);
		
		TATActor* cubeActor1 = new TATActor(cube);
		cubeActor1->Initialize();
		cubeActor1->SetMaterial(mat);
		cubeActor1->SetUseTransform(true);
		cubeActor1->m_WorldTransform = TATransform(TATQuaternion::GetIdentity(), TATVector3(50, 0, 0));
		TATStageNode* node4 = m_RootNode->CreateChild("py");
		node4->SetPosition(TATVector3(40, 0, 0));
		node4->MountActor(cubeActor1);
		TATRigidBody* rb1 = TATDynamicWorld::Instance()->CreateConvex(cube, 0.1f);
		rb1->Initialize();
		rb1->m_ControllByUser = true;
		TATDynamicWorld::Instance()->InitRigidBody(rb1, cubeActor1->m_WorldTransform, 0.1f, 0.8, 0.2, TATVector3(0, 0, 0));
		cubeActor1->AttachTickable(rb1);

		m_Rigid0 = rb;
		m_Rigid1 = rb1;
	}

	virtual void Run()
	{
		Initialize();

		CreateScene();

		m_PhysicThread->Run();

		m_RenderThread->Run();
	}

	virtual void BeginRenderOneFrame(float dt)
	{
		__super::BeginRenderOneFrame(dt);
	}

	virtual void RenderOneFrameEnd(float dt)
	{
		__super::RenderOneFrameEnd(dt);
	}

	virtual void SimulationStart(float dt)
	{
		__super::SimulationStart(dt);
	}

	virtual void SimulationEnd(float dt)
	{
		__super::SimulationEnd(dt);
	}

	virtual void Update(float dt)
	{
		__super::Update(dt);

		TATransform tr;
		TATQuaternion ori;
		static float x = 0.01f;
		x += dt;
		//x = 2.38;
		ori.FromAngleAxis(TATVector3(0.5, 1, 2), x);
		tr.SetRotation(ori);
		tr.SetOrigin(TATVector3(50, 50, 10));
		m_Rigid0->SetWorldTransform(tr);

		ori.FromAngleAxis(TATVector3(-1, 5, 3), 2 * x);
		tr.SetOrigin(TATVector3(0, 0, 0));
		tr.SetRotation(ori);
		m_Rigid1->SetWorldTransform(tr);


		TATSATDistPack cd;
		float dist = TATSATDistSolver::SolveConvexDistance(m_Rigid0->m_CollideShape->Cast<TATCollideShapeConvex>(),
			m_Rigid1->m_CollideShape->Cast<TATCollideShapeConvex>(), m_Rigid0->m_WorldTransform, m_Rigid1->m_WorldTransform, cd);

		TAT_RENDER_THREAD->m_LinePainter->Clear();
		TAT_RENDER_THREAD->m_LinePainter->PaintLine(cd.m_ClostPtA, cd.m_ClostPtB, TATVector3(1, 1, 1));
	}

	TATRigidBody* m_Rigid0;
	TATRigidBody* m_Rigid1;
};