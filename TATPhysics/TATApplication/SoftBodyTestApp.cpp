#include "SoftBodyTestApp.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATGLEntry.h"
#include "../TATResources/TATResourceManager.h"
#include "../TATStage/TATStageNode.h"
#include "TAThread.h"
#include "../TATStage/TATActor.h"
#include "../TATResources/TATMaterial.h"
#include "../TATResources/TATexture.h"
#include "../TATStage/TATLinePainter.h"
#include "../TATPositionBasedDynamics/TATPBDBody.h"

void SoftBodyTestApp::Initialize()
{
	__super::Initialize();

	TATGLEntry::Instance()->Initialize(1080, 720);
}

void SoftBodyTestApp::CreateScene()
{
	__super::CreateScene();

	m_MainCamera->SetPosition(TATVector3(20, 20, -300));
	m_MainCamera->SetWindowSize(TATGLEntry::Instance()->m_WindowWidth, TATGLEntry::Instance()->m_WindowHeight);
	m_MainCamera->SetDirection((TATVector3(0, 50, 10) - m_MainCamera->GetPosition()).Normalized());
	m_MainCamera->SetPlane(0.01, 100000);
	m_MainCamera->SetSpeed(100);

	TATMesh* plane = TATResourceManager::Instance()->LoadMesh("plane.obj");
	TATMesh* tet_halfSphere = TATResourceManager::Instance()->LoadMesh("halfSphere.tmodel");

	plane->Initialize();
	tet_halfSphere->Initialize();

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");
	mat->Initialize();

	TATActor* planeActor = new TATActor(plane);
	planeActor->Initialize();
	planeActor->SetMaterial(mat);
	TATStageNode* planeNode = m_RootNode->CreateChild("plane");
	planeNode->MountActor(planeActor);
	TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, 0), TATVector3(0, 1, 0));

	TATActor* softActor0 = new TATActor(tet_halfSphere);
	softActor0->Initialize();
	softActor0->SetMaterial(mat);
	TATStageNode* softNode0 = m_RootNode->CreateChild("softHalfSphere");
	softNode0->MountActor(softActor0);
	TATPBDBody* softBody0 = new TATPBDBody("softHalfSphere", tet_halfSphere->m_Loader->m_Buffer, 1.0f);
	softBody0->Initialize();
	softBody0->AddPositionConstraint(0.95, 0.95);
	softBody0->AddVolumeConstraint(0.95, 0.95);
	softBody0->SetDamping(0.8, 0.8);
	softActor0->AttachTickable(softBody0);

	softBody0->m_Particles[0].AddConstantForce(TATVector3(0, -10000, 0));
}

void SoftBodyTestApp::Run()
{
	Initialize();

	CreateScene();

	m_PhysicThread->Run();

	m_RenderThread->Run();
}

void SoftBodyTestApp::BeginRenderOneFrame(float dt)
{

}