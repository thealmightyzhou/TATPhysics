#ifndef THEALMIGHTY_CAMERA_H
#define THEALMIGHTY_CAMERA_H

#include "TATGLHeader.h"
#include "TATInputListener.h"
#include <vector>
#include "../TATBasis/TATObject.h"

class TATCamera :public TATInputListener,public TATObject
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
		m_Yaw = -90.0f;
		m_Pitch = 0.0f;
		m_MoveSpeed = 50.0f;
		m_MouseSensitivity = 0.1f;
		m_Zoom = 45.0f;
		m_NearPlane = 0.1f;
		m_FarPlane = 1000.0f;
		m_WindowWidth = 1280.0f;
		m_WindowHeight = 800.0f;
		m_WorldUp = glm::vec3(0.0f, 1.0f, 0.0f);
		m_Position = glm::vec3(0.0f, 0.0f, 0.0f);
		m_Front = glm::vec3(0.0f, 0.0f, -1.0f);
	}

	//call in render loop
	void Move(float deltaTime)
	{
		float velocity = m_MoveSpeed * deltaTime;
		if (m_MoveState & CameraMoveState::MOVE_FORWARD)
			m_Position += m_Front * velocity;
		if (m_MoveState & CameraMoveState::MOVE_BACKWARD)
			m_Position -= m_Front * velocity;
		if (m_MoveState & CameraMoveState::MOVE_LEFT)
			m_Position -= m_Right * velocity;
		if (m_MoveState & CameraMoveState::MOVE_RIGHT)
			m_Position += m_Right * velocity;
		if (m_MoveState & CameraMoveState::MOVE_UP)
			m_Position += m_Up * velocity;
		if (m_MoveState & CameraMoveState::MOVE_DOWN)
			m_Position -= m_Up * velocity;
	}

	void Rotate(float xoffset, float yoffset, GLboolean constrainPitch = true)
	{
		if (m_Freeze)
			return;

		xoffset *= m_MouseSensitivity;
		yoffset *= m_MouseSensitivity;

		m_Yaw += xoffset;
		m_Pitch += yoffset;

		// Make sure that when pitch is out of bounds, screen doesn't get flipped
		if (constrainPitch)
		{
			if (m_Pitch > 89.0f)
				m_Pitch = 89.0f;
			if (m_Pitch < -89.0f)
				m_Pitch = -89.0f;
		}

		Update();
	}

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
		Rotate(dx, dy, true);
	}

	virtual void OnKeyPressed(int key) override
	{
		//TODO use commander
		switch (key)
		{
		case GLFW_KEY_W:
			m_MoveState |= CameraMoveState::MOVE_FORWARD;
			break;
		case GLFW_KEY_S:
			m_MoveState |= CameraMoveState::MOVE_BACKWARD;
			break;
		case GLFW_KEY_A:
			m_MoveState |= CameraMoveState::MOVE_LEFT;
			break;
		case GLFW_KEY_D:
			m_MoveState |= CameraMoveState::MOVE_RIGHT;
			break;
		case GLFW_KEY_LEFT_SHIFT:
			m_MoveState |= CameraMoveState::MOVE_UP;
			break;
		case GLFW_KEY_LEFT_CONTROL:
			m_MoveState |= CameraMoveState::MOVE_DOWN;
			break;
		default:
			break;
		}
	}

	void SetSpeed(float speed)
	{
		m_MoveSpeed = speed;
	}
	void SetPosition(glm::vec3 pos)
	{
		m_Position = pos;
	}
	void SetFreezeCamera(bool f)
	{
		m_Freeze = f;
		Update();
	}

	void GetViewMatrix(glm::mat4& mat)
	{
		mat = glm::lookAt(m_Position, m_Position + m_Front, m_Up);
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

	glm::vec3 GetForward()
	{
		return m_Front;
	}
	glm::vec3 GetPosition()
	{
		return m_Position;
	}
	glm::vec3 GetRight()
	{
		return m_Right;
	}
	glm::vec3 GetUp()
	{
		return m_Up;
	}

private:
	void Update()
	{
		if (m_Freeze)
			return;

		glm::vec3 front;
		front.x = cos(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));
		front.y = sin(glm::radians(m_Pitch));
		front.z = sin(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));
		m_Front = glm::normalize(front);
		m_Right = glm::normalize(glm::cross(m_Front, m_WorldUp)); 
		m_Up = glm::normalize(glm::cross(m_Right, m_Front));
		m_ViewMat = glm::lookAt(m_Position, m_Position + m_Front, m_Up);
		m_ProjectionMat = glm::perspective(glm::radians(m_Zoom), m_WindowWidth / m_WindowHeight, m_NearPlane, m_FarPlane);
	}

	// Camera Attributes
	glm::vec3 m_Position;
	glm::vec3 m_Front;
	glm::vec3 m_Up;
	glm::vec3 m_Right;
	glm::vec3 m_WorldUp;
	glm::mat4 m_ProjectionMat;
	glm::mat4 m_ViewMat;

	// Camera Options
	float m_MoveSpeed;
	float m_MouseSensitivity;
	float m_Zoom;
	float m_Yaw;
	float m_Pitch;
	float m_NearPlane;
	float m_FarPlane;
	float m_WindowHeight;
	float m_WindowWidth;
	bool m_Freeze;

	UINT m_MoveState;
};
#endif