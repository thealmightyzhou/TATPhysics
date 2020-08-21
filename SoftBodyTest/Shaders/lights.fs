#version 330 core
out vec4 frag_color;

struct BaseLight
{
	vec3 m_Color;
	float m_Ambient;
	float m_Diffuse;
}

struct DirectionalLight
{
	BaseLight m_Base;
	vec3 m_Direction;
}

struct PointLight
{
	BaseLight m_Base;
	vec3 m_Position;
	float m_Constant;
	float m_Linear;
	float m_Exp;
}

uniform int _pl_num;
uniform PointLight _point_light;
uniform DirectionalLight _dir_light;

uniform int Ulight_type;
uniform vec3 Uview_pos;
uniform vec3 Ulight_color;
uniform float Uambient;
uniform float Udiffuse;
uniform float Uspecular_intensity;
uniform float Uspecular_power;
uniform vec3 Ulight_direction;
uniform vec3 Ulight_position;
uniform float Upl_constant;
uniform float Upl_linear;
uniform float Upl_exp;
uniform sampler2D Utex0;

in vec3 onormal;
in vec3 ofragPos;
in vec2 otexcoord;



vec4 CalcLightInternal(BaseLight light,vec3 dir,vec3 normal)
{
	vec4 ambientCol = vec4(light.m_Color * light.m_Ambient,1.0f);
	vec4 diffuseCol = vec4(0,0,0,0);
	vec4 specCol = vec4(0,0,0,0);
	
	float diffuseFactor = dot(normal,-dir);

	if(diffuseFactor > 0)
	{
		diffuseCol = vec4(light.m_Color * light.m_Diffuse * diffuseFactor,1.0f);
		vec3 vertex2eye = normalize(eye_pos - TransData.fragPos);
		vec3 light_reflect = normalize(reflect(dir,normal));
		float specFactor = dot(vertex2eye,light_reflect);
		if(specFactor > 0)
		{
			specFactor = pow(specFactor,Uspecular_power);
			specCol = vec4(light.m_Color * Uspecular_intensity * specFactor, 1.0f);
		}
	}
	
	return ambientCol + diffuseCol + specCol;
}

vec4 CalcDirectionalLight(BaseLight light,vec3 dir,vec3 normal)
{
	return CalcLightInternal(light,dir,normal);
}

vec4 CalcPointLight(BaseLight base,vec3 position,vec3 normal)
{
	vec3 lightDir =  position - Ulight_position;
	float dist = length(lightDir);
	vec4 color = CalcLightInternal(base,lightDir,normal);
	float attenuation = Upl_constant + Upl_linear * dist + Upl_exp * dist * dist;
	return color/attenuation;
}

void main()
{
	BaseLight base_light;
	base_light.m_Ambient = Uambient;
	base_light.m_Diffuse = Udiffuse;
	base_light.m_Color = Ulight_color;
	
	vec4 color(1,1,1,1);
 	if(Ulight_type == 0)
		color = CalcDirectionalLight(base_light,Ulight_direction,TransData.normal);
	else if(Ulight_type == 1)
		color = CalcPointLight(base_light,Ulight_position,TransData.normal);
		
	frag_color = vec4(color, 1.0f) * texture(tex0,TransData.texcoord);
}