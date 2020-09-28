#version 330 core
out vec4 frag_color;

struct DirectionalLight
{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	int specular_power;
	vec3 direction;
};

struct PointLight
{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	int specular_power;
	vec3 position;
	float constant;
	float linear;
	float exp;
};

in vec3 onormal;
in vec3 ofragPos;
in vec2 otexcoord;

uniform PointLight _point_light0;
uniform PointLight _point_light1;
uniform DirectionalLight _direct_light0;

uniform vec3 _view_pos;
uniform sampler2D _tex0;

vec4 CalcLightInternal(vec3 ambient,vec3 diffuse,vec3 specular,int specular_power,vec3 dir,vec3 normal)
{
	vec4 ambientCol = vec4(ambient,1.0f);
	vec4 diffuseCol = vec4(0,0,0,0);
	vec4 specCol = vec4(0,0,0,0);
	
	float diffuseFactor = dot(normal,-dir);

	if(diffuseFactor > 0)
	{
		diffuseCol = vec4(diffuse * diffuseFactor,1.0f);
		vec3 vertex2eye = normalize(_view_pos - ofragPos);
		vec3 light_reflect = normalize(reflect(dir,normal));
		float specFactor = dot(vertex2eye,light_reflect);
		if(specFactor > 0)
		{
			specFactor = pow(specFactor,specular_power);
			specCol = vec4(specular * specFactor, 1.0f);
		}
	}
	
	return ambientCol + diffuseCol + specCol;
}

vec4 CalcDirectionalLight(DirectionalLight dl,vec3 normal)
{
	return CalcLightInternal(dl.ambient,dl.diffuse,dl.specular,dl.specular_power,dl.direction,normal);
}

vec4 CalcPointLight(PointLight pl,vec3 fragPos,vec3 normal)
{
	vec3 lightDir =  fragPos - pl.position;
	float dist = length(lightDir);
	vec4 color = CalcLightInternal(pl.ambient,pl.diffuse,pl.specular,pl.specular_power,lightDir,normal);
	float attenuation = pl.constant + pl.linear * dist + pl.exp * dist * dist;
	return color/attenuation;
}

void main()
{

	vec4 color0 = CalcDirectionalLight(_direct_light0,onormal);

	vec4 color1 = CalcPointLight(_point_light0,ofragPos,onormal);
	
	vec4 color2 = CalcPointLight(_point_light1,ofragPos,onormal);
		
	frag_color = (color0 + color1 + color2) * texture(_tex0,otexcoord);
}