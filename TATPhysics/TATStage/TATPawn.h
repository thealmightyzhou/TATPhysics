#pragma once
#include "TATActor.h"
#include "../TATBasis/TATickable.h"
#include "../TATGLRender/TATInputListener.h"
#include "../TATResources/TATMesh.h"

class TATPawn :public TATActor, public TATInputListener
{
public:
	TATPawn() {}

	TATPawn(TATMesh* mesh) :TATActor(mesh)
	{

	}

	virtual void OnKeyPressed(int key) {}
	virtual void OnKeyReleased(int key) {}
	virtual void OnKeyHold(int key) {}
	virtual void OnCursorMove(float dx, float dy) {}
	virtual void OnCursorPressed(int key) {}
	virtual void OnCursorReleased(int key) {}
	virtual void OnCursorHold(int key) {}
	virtual void OnCursorScroll(float offset) {}
};