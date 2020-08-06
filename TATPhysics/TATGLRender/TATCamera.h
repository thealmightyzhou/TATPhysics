#ifndef THEALMIGHTY_CAMERA_H
#define THEALMIGHTY_CAMERA_H

#include "TATGLHeader.h"
#include "TATInputListener.h"
#include <vector>
#include "../TATBasis/TATObject.h"
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATQuaternion.h"
#include "../TATApplication/TATWorldListener.h"

class TATCamera :public TATInputListener,public TATObject,public TATRenderListener
{
public:
	enum CameraMoveState
	{
		MOVE_NONE,
		MOVE_FORWARD,
		MOVE_BACKWARD,
		MOVE_LEFT,
		MOVE_RIGHT,
		MOVE_UP,
		MOVE_DOWN,
	};

	TATCamera(const TString& name):TATObject("camera_" + name)
	{
		Initialize();
		Update();
	}

	virtual ~TATCamera() {}

	void Initialize()
	{
		m_Freeze = false;
		m_Yaw = 0.0f;
		m_Pitch = 0.0f;
		m_Roll = 0.0f;
		m_MoveSpeed = 10.0f;
		m_MouseSensitivity = 0.1f;
		m_Zoom = 45.0f;
		m_NearPlane = 0.1f;
		m_FarPlane = 1000.0f;
		m_WindowWidth = 1280.0f;
		m_WindowHeight = 800.0f;
		m_WorldUp = TATVector3(0.0f, 1.0f, 0.0f);
		m_Position = TATVector3(0.0f, 0.0f, 0.0f);
		m_Front = TATVector3(0.0f, 0.0f, -1.0f);
		m_MoveState = MOVE_NONE;
		m_Orientation = TATQuaternion::GetIdentity();
		m_FirstCursor = true;
	}

	void SetWindowSize(int w, int h)
	{
		m_WindowWidth = w;
		m_WindowHeight = h;
	}

	void SetPlane(float nearPlane, float farPlane)
	{
		m_NearPlane = nearPlane;
		m_FarPlane = farPlane;
	}

	void SetEuler(float yaw, float pitch, float roll)
	{
		m_Yaw = yaw;
		m_Pitch = pitch;
		m_Roll = roll;
		m_Orientation.FromEuler(yaw, pitch, roll);
	}

	void SetOrientation(const TATQuaternion& q)
	{
		m_Orientation = q;
		m_Orientation.GetEulerZYX(m_Yaw, m_Pitch, m_Roll);
		m_Yaw = TAT_DEGREE(m_Yaw);
		m_Pitch = TAT_DEGREE(m_Pitch);
		m_Roll = TAT_DEGREE(m_Roll);
	}

	//call in render loop
	void Move(float deltaTime)
	{
		float velocity = m_MoveSpeed * deltaTime;

		if (TATInputListener::m_KeyState[GLFW_KEY_W])
			m_Position += m_Front * velocity;
		if (TATInputListener::m_KeyState[GLFW_KEY_S])
			m_Position -= m_Front * velocity;
		if (TATInputListener::m_KeyState[GLFW_KEY_A])
			m_Position -= m_Right * velocity;
		if (TATInputListener::m_KeyState[GLFW_KEY_D])
			m_Position += m_Right * velocity;
		if (TATInputListener::m_KeyState[GLFW_KEY_LEFT_SHIFT])
			m_Position += m_Up * velocity;
		if (TATInputListener::m_KeyState[GLFW_KEY_LEFT_CONTROL])
			m_Position -= m_Up * velocity;

		Update();
	}

	void Rotate(float xoffset, float yoffset, GLboolean constrainRoll = true)
	{
		if (m_Freeze)
			return;

		xoffset *= m_MouseSensitivity;
		yoffset *= m_MouseSensitivity;

		//m_Yaw -= xoffset;
		//m_Pitch -= yoffset;

		//if (constrainPitch)
		//{
		//	if (m_Pitch > 89.0f)
		//		m_Pitch = 89.0f;
		//	if (m_Pitch < -89.0f)
		//		m_Pitch = -89.0f;
		//}

		m_Pitch -= xoffset;
		m_Roll -= yoffset;

		if (constrainRoll)
		{
			if (m_Roll > 89.0f)
				m_Roll = 89.0f;
			if (m_Roll < -89.0f)
				m_Roll = -89.0f;
		}

		m_Orientation.FromEuler(TAT_RADIAN(m_Yaw), TAT_RADIAN(m_Pitch), TAT_RADIAN(m_Roll));

		Update();
	}

	void SetSpeed(float speed)
	{
		m_MoveSpeed = speed;
	}
	void SetPosition(const TATVector3& pos)
	{
		m_Position = pos;
	}
	void SetFreezeCamera(bool f)
	{
		m_Freeze = f;
		Update();
	}

	void SetDirection(const TATVector3& dir)
	{
		TATVector3 up = dir.Cross(TATVector3::UnitX()).Normalized();
		TATVector3 right = up.Cross(dir);
		right.Y = 0;
		right.SafeNormalize();
		TATQuaternion q;
		q.FromCoordinateSys(right, up, dir);
		SetOrientation(q);
	}

	void GetViewMatrix(glm::mat4& mat)
	{
		mat = glm::lookAt(m_Position.ToGLM(), (m_Position + m_Front).ToGLM(), m_Up.ToGLM());
	}

	void GetViewMatrix(float* mat)
	{
		glm::mat4 view;
		GetViewMatrix(view);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				mat[i * 4 + j] = view[i][j];
			}
		}

		int stop = 1;
	}

	void GetProjectionMatrix(glm::mat4& proj)
	{
		proj = m_ProjectionMat;
	}

	void GetProjectionMatrix(float* mat)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				mat[i * 4 + j] = m_ProjectionMat[i][j];
			}
		}
	}

	TATVector3 GetForward()
	{
		return m_Front;
	}
	TATVector3 GetPosition()
	{
		return m_Position;
	}
	TATVector3 GetRight()
	{
		return m_Right;
	}
	TATVector3 GetUp()
	{
		return m_Up;
	}

	//interface Implement
	virtual void OnCursorScroll(float offset) override
	{
		if (m_Zoom >= 1.0f && m_Zoom <= 45.0f)
			m_Zoom -= offset;
		if (m_Zoom <= 1.0f)
			m_Zoom = 1.0f;
		if (m_Zoom >= 45.0f)
			m_Zoom = 45.0f;
	}

	virtual void OnCursorMove(float dx, float dy) override
	{
		if (m_FirstCursor)
		{
			m_FirstCursor = false;
			return;
		}
	
		Rotate(dx, dy, true);
	}

	virtual void OnKeyPressed(int key) override
	{}

	virtual void OnKeyHold(int key) override
	{}

	virtual void BeginRenderOneFrame(float dt) override
	{
		Update();
		Move(dt);
	}

	virtual void RenderOneFrameEnd(float dt) override
	{}
	
private:
	void Update()
	{
		if (m_Freeze)
			return;

		TATVector3 front;
		//front.X = cos(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));
		//front.Y = sin(glm::radians(m_Pitch));
		//front.Z = sin(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));

		front = m_Orientation * TATVector3(0, 0, 1);

		m_Front = front.Normalized();
		m_Right = (m_Front.Cross(m_WorldUp)).Normalized();
		m_Up = (m_Right.Cross(m_Front)).Normalized();
		m_ViewMat = glm::lookAt(m_Position.ToGLM(), (m_Position + m_Front).ToGLM(), m_Up.ToGLM());
		m_ProjectionMat = glm::perspective(glm::radians(m_Zoom), m_WindowWidth / m_WindowHeight, m_NearPlane, m_FarPlane);
	}

	// Camera Attributes
	TATVector3 m_Position;
	TATVector3 m_Front;
	TATVector3 m_Up;
	TATVector3 m_Right;
	TATVector3 m_WorldUp;
	TATQuaternion m_Orientation;
	glm::mat4 m_ProjectionMat;
	glm::mat4 m_ViewMat;

	// Camera Options
	float m_MoveSpeed;
	float m_MouseSensitivity;
	float m_Zoom;
	float m_Yaw;
	float m_Pitch;
	float m_Roll;
	float m_NearPlane;
	float m_FarPlane;
	float m_WindowHeight;
	float m_WindowWidth;
	bool m_Freeze;
	bool m_FirstCursor;

	UINT m_MoveState;
};
#endif