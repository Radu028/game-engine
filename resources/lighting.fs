#version 330

in vec3 fragPosition;
in vec3 fragNormal;
in vec2 fragTexCoord;
in vec4 fragColor;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform vec3 sunDirection;
uniform vec3 sunColor;
uniform float ambientStrength;
uniform vec3 viewPos;

const float specularStrength = 0.3;
const float shininess = 96.0;

void main()
{
    vec4 sampled = texture(texture0, fragTexCoord);
    vec3 albedo = sampled.rgb * colDiffuse.rgb * fragColor.rgb;
    float alpha = sampled.a * colDiffuse.a * fragColor.a;

    vec3 normal = normalize(fragNormal);
    vec3 lightDir = normalize(-sunDirection);
    vec3 viewDir = normalize(viewPos - fragPosition);

    float NdotL = max(dot(normal, lightDir), 0.0);

    float wrap = 0.35;
    float wrappedLambert = max((dot(normal, lightDir) + wrap) / (1.0 + wrap), 0.0);
    vec3 diffuse = wrappedLambert * sunColor;

    vec3 ambientSky = vec3(0.18, 0.32, 0.65);
    vec3 ambient = ambientStrength * mix(ambientSky, sunColor, 0.35);

    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), shininess);
    vec3 specular = specularStrength * spec * mix(sunColor * 1.1, vec3(1.0, 0.97, 0.9), 0.5);

    float rim = pow(clamp(1.0 - dot(normal, viewDir), 0.0, 1.0), 4.0);
    vec3 rimColor = rim * sunColor * 0.25;

    float shadowFactor = mix(0.55, 1.0, NdotL);
    vec3 lighting = ambient + diffuse * shadowFactor + specular + rimColor;

    vec3 color = albedo * lighting;

    float luma = dot(color, vec3(0.299, 0.587, 0.114));
    vec3 base = vec3(luma);
    float saturationBoost = 0.45;
    color = base + (color - base) * (1.0 + saturationBoost);

    color = pow(color, vec3(0.94));
    color = clamp(color, 0.0, 1.0);

    finalColor = vec4(color, alpha);
}

