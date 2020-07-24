#pragma once

#include <map>

class TATInputListener
{
public:
	TATInputListener();
	virtual void OnKeyPressed(int key) {}
	virtual void OnKeyReleased(int key) {}
	virtual void OnKeyHold(int key) {}
	virtual void OnCursorMove(float dx, float dy) {}
	virtual void OnCursorPressed(int key) {}
	virtual void OnCursorReleased(int key) {}
	virtual void OnCursorHold(int key) {}
	virtual void OnCursorScroll(float offset) {}

	int m_TaskIndex; //
	bool m_CoverAfter; //cover all which taskindex bigger than this

	static std::map<int, bool> m_KeyState;
};