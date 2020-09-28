#version 330 core
out vec4 color;

in vec3 onormal;  
in vec3 ofragPos;  
in vec2 otexcoord;
  
uniform vec3 Ulight_position; 
uniform vec3 Uview_pos;
uniform vec3 Ulight_color;
uniform sampler2D tex0;

void main()
{
    // Ambient
    float ambientStrength = 0.1f;
    vec3 ambient = ambientStrength * Ulight_color;
  	
    // Diffuse 
    vec3 norm = normalize(onormal);
    vec3 lightDir = normalize(Ulight_position - ofragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * Ulight_color;
    
    // Specular
    float specularStrength = 0.5f;
    vec3 viewDir = normalize(Uview_pos - ofragPos);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * Ulight_color;  
        
    vec3 result = (ambient + diffuse + specular);
    color = vec4(result, 1.0f) * texture(tex0,otexcoord);
} 
