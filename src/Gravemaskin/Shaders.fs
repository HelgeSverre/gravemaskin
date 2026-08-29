namespace Gravemaskin.Shell

/// GLSL 410 core — the macOS ceiling. All shaders are inline strings, no
/// asset files (house style).
[<RequireQualifiedAccess>]
module Shaders =

    let terrainVertex =
        """#version 410 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
layout(location = 3) in float materialLayer;

uniform mat4 viewProjection;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;
out float vLayer;

void main()
{
    vNormal = normal;
    vColor = color;
    vWorld = position;
    vLayer = materialLayer;
    gl_Position = viewProjection * vec4(position, 1.0);
}
"""

    /// Shared PS3-era shading core: gamma-correct (albedo linearized, lit in
    /// linear, gamma-encoded out), 3×3 PCF sun shadow from the depth map,
    /// hemispheric ambient, optional paint specular, distance fog.
    let private shadingCommon =
        """
uniform vec3 sunDirection;
uniform vec3 cameraPosition;
uniform sampler2DShadow shadowMap;
uniform mat4 lightViewProjection;

float shadowFactor(vec3 world, vec3 n)
{
    // Normal-offset + depth bias against acne; outside the map = lit.
    vec4 p = lightViewProjection * vec4(world + n * 0.05, 1.0);
    vec3 q = p.xyz / p.w * 0.5 + 0.5;
    if (q.x < 0.002 || q.x > 0.998 || q.y < 0.002 || q.y > 0.998 || q.z > 0.999)
        return 1.0;
    float sum = 0.0;
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
            sum += texture(shadowMap, vec3(q.xy + vec2(float(dx), float(dy)) / 2048.0, q.z - 0.0016));
    return sum / 9.0;
}

vec3 shade(vec3 albedoGamma, vec3 rawNormal, vec3 world, float specStrength)
{
    vec3 albedo = pow(max(albedoGamma, vec3(0.0)), vec3(2.2));
    vec3 n = normalize(rawNormal);
    float sun = max(dot(n, sunDirection), 0.0);
    float shadow = shadowFactor(world, n);
    // Hemispheric ambient (linear): sky from above, ground bounce below.
    vec3 ambient = mix(vec3(0.050, 0.042, 0.036), vec3(0.115, 0.125, 0.150), n.y * 0.5 + 0.5);
    vec3 lit = albedo * (ambient + vec3(1.15, 1.06, 0.90) * sun * shadow);
    if (specStrength > 0.0)
    {
        vec3 viewDirection = normalize(cameraPosition - world);
        float spec = pow(max(dot(reflect(-sunDirection, n), viewDirection), 0.0), 28.0);
        lit += vec3(spec) * specStrength * shadow;
    }
    float dist = length(world - cameraPosition);
    float fog = 1.0 - exp(-dist * 0.007);
    lit = mix(lit, vec3(0.34, 0.37, 0.43), fog);
    return pow(lit, vec3(1.0 / 2.2));
}
"""

    let terrainFragment =
        """#version 410 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;
in float vLayer;

uniform sampler2DArray detailTextures;

out vec4 fragColor;
"""
        + shadingCommon
        + """
void main()
{
    // Two scales of the per-material detail texture (tileable, generated at
    // startup): close detail + a broader octave to break the repeat.
    vec3 detailNear = texture(detailTextures, vec3(vWorld.xz * 0.9, vLayer)).rgb;
    vec3 detailFar = texture(detailTextures, vec3(vWorld.xz * 0.13, vLayer)).rgb;
    vec3 detail = detailNear * detailFar * 4.0;
    fragColor = vec4(shade(vColor * detail, vNormal, vWorld, 0.0), 1.0);
}
"""

    let solidVertex =
        """#version 410 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

uniform mat4 viewProjection;
uniform mat4 model;
uniform vec3 solidColor;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;

void main()
{
    vec4 world = model * vec4(position, 1.0);
    vNormal = mat3(model) * normal;
    vColor = solidColor;
    vWorld = world.xyz;
    gl_Position = viewProjection * world;
}
"""

    /// Grains: heavily hash-deformed, randomly flattened, and STRETCHED
    /// along their velocity — fast grains draw as streaks, so a pour reads
    /// as a ribbon of sand instead of a scatter of beads.
    let grainVertex =
        """#version 410 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec4 instance;      // xyz = world pos, w = size
layout(location = 2) in vec3 instanceColor;
layout(location = 3) in vec3 instanceVelocity;

uniform mat4 viewProjection;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;

float hash(vec2 p)
{
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

void main()
{
    float wobble = hash(position.xy * 7.31 + instance.xz);
    float squashSeed = hash(instance.xz * 3.7);
    vec3 squash = vec3(1.0, mix(0.35, 1.0, squashSeed), 1.0);
    vec3 local = position * squash * instance.w * mix(0.45, 1.55, wobble);

    float speed = length(instanceVelocity);
    if (speed > 0.5)
    {
        vec3 direction = instanceVelocity / speed;
        float stretch = min(1.0 + speed * 0.35, 3.5) - 1.0;
        local += direction * dot(local, direction) * stretch;
    }

    vec3 world = instance.xyz + local;
    vNormal = normalize(position);
    vColor = instanceColor;
    vWorld = world;
    gl_Position = viewProjection * vec4(world, 1.0);
}
"""

    /// Machine surfaces: matte base + a paint-like specular hint.
    let solidFragment =
        """#version 410 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;

out vec4 fragColor;
"""
        + shadingCommon
        + """
void main()
{
    fragColor = vec4(shade(vColor, vNormal, vWorld, 0.5), 1.0);
}
"""

    /// Depth-only pass for the sun shadow map.
    let depthVertex =
        """#version 410 core
layout(location = 0) in vec3 position;

uniform mat4 mvp;

void main()
{
    gl_Position = mvp * vec4(position, 1.0);
}
"""

    let depthFragment =
        """#version 410 core
void main() {}
"""

    /// Water: a translucent animated sheet with fresnel toward the horizon
    /// and a sun glint.
    let waterVertex =
        """#version 410 core
layout(location = 0) in vec3 position;

uniform mat4 viewProjection;
uniform float time;
uniform float waterLevel;

out vec3 vWorld;

void main()
{
    vec3 p = position;
    // Two crossing ripple trains, small enough to keep shorelines honest.
    p.y = waterLevel + sin(p.x * 1.7 + time * 1.3) * 0.02 + sin(p.z * 2.3 - time * 0.9) * 0.015;
    vWorld = p;
    gl_Position = viewProjection * vec4(p, 1.0);
}
"""

    let waterFragment =
        """#version 410 core
in vec3 vWorld;

uniform vec3 sunDirection;
uniform vec3 cameraPosition;
uniform float time;

out vec4 fragColor;

void main()
{
    // Ripple normal from the same waves the vertex shader displaces with.
    float dx = cos(vWorld.x * 1.7 + time * 1.3) * 0.034;
    float dz = cos(vWorld.z * 2.3 - time * 0.9) * 0.034;
    vec3 n = normalize(vec3(-dx, 1.0, -dz));
    vec3 viewDirection = normalize(cameraPosition - vWorld);
    float fresnel = pow(1.0 - max(dot(n, viewDirection), 0.0), 3.0);
    vec3 deep = vec3(0.05, 0.10, 0.12);
    vec3 sky = vec3(0.45, 0.52, 0.60);
    float glint = pow(max(dot(reflect(-sunDirection, n), viewDirection), 0.0), 90.0);
    vec3 color = mix(deep, sky, fresnel * 0.8) + vec3(glint) * 0.7;
    float dist = length(vWorld - cameraPosition);
    float fog = 1.0 - exp(-dist * 0.007);
    color = mix(color, vec3(0.63, 0.66, 0.70), fog);
    fragColor = vec4(color, 0.62 + fresnel * 0.3);
}
"""

    let clodVertex =
        """#version 410 core
layout(location = 0) in vec3 position;      // unit icosahedron
layout(location = 1) in vec4 instance;      // xyz = world pos, w = radius
layout(location = 2) in vec3 instanceColor;

uniform mat4 viewProjection;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;

void main()
{
    // Hash-deform per vertex per instance: clods, not marbles.
    float wobble = fract(sin(dot(position.xy + instance.xz, vec2(12.9898, 78.233))) * 43758.5453);
    vec3 local = position * instance.w * mix(0.8, 1.15, wobble);
    vec3 world = instance.xyz + local;
    vNormal = normalize(position);
    vColor = instanceColor;
    vWorld = world;
    gl_Position = viewProjection * vec4(world, 1.0);
}
"""

    let clodFragment =
        """#version 410 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;

out vec4 fragColor;
"""
        + shadingCommon
        + """
void main()
{
    fragColor = vec4(shade(vColor, vNormal, vWorld, 0.0), 1.0);
}
"""
