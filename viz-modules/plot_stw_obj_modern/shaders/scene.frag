#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec4 FragPosLightSpace;

out vec4 FragColor;

uniform sampler2D shadowMap;
uniform vec3 lightPos;
uniform vec3 fillLightPos;   // Fill light from opposite side
uniform float fillLightStrength;
uniform vec3 viewPos;
uniform vec3 objectColor;
uniform float ambientStrength;
uniform float shadowSoftness;
uniform float flipNormals;  // 1.0 or -1.0
uniform float unlit;        // 1.0 = no lighting, just flat color
uniform float checkerboard; // 1.0 = use checkerboard pattern
uniform float checkerSize;  // Size of checker squares

float ShadowCalculation(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
    // Perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;

    // Check if outside shadow map
    if (projCoords.z > 1.0 || projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float currentDepth = projCoords.z;

    // Bias to prevent shadow acne (increased for large geometry)
    float bias = max(0.15 * (1.0 - dot(normal, lightDir)), 0.03);

    // PCF (Percentage Closer Filtering) for soft shadows
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    int samples = 2;

    for (int x = -samples; x <= samples; ++x) {
        for (int y = -samples; y <= samples; ++y) {
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize * shadowSoftness).r;
            shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;
        }
    }
    shadow /= float((2 * samples + 1) * (2 * samples + 1));

    return shadow;
}

void main()
{
    vec3 color = objectColor;

    // Checkerboard pattern based on world position
    if (checkerboard > 0.5) {
        float cx = floor(FragPos.x / checkerSize);
        float cz = floor(FragPos.z / checkerSize);
        float checker = mod(cx + cz, 2.0);
        // Dark gray and medium gray - more contrast
        color = mix(vec3(0.15, 0.15, 0.18), vec3(0.4, 0.4, 0.45), checker);
    }

    vec3 normal = normalize(Normal) * flipNormals;
    vec3 lightColor = vec3(1.0);

    // Ambient
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular (Blinn-Phong)
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    vec3 specular = 0.3 * spec * lightColor;

    // Shadow
    float shadow = ShadowCalculation(FragPosLightSpace, normal, lightDir);

    // Fill light (no shadows, softer)
    vec3 fillLightDir = normalize(fillLightPos - FragPos);
    float fillDiff = max(dot(normal, fillLightDir), 0.0);
    vec3 fillDiffuse = fillDiff * lightColor * fillLightStrength;

    // Final color
    vec3 lighting = ambient + (1.0 - shadow) * (diffuse + specular) + fillDiffuse;

    // Unlit mode for debugging
    if (unlit > 0.5) {
        FragColor = vec4(color, 1.0);
    } else {
        FragColor = vec4(lighting * color, 1.0);
    }
}
