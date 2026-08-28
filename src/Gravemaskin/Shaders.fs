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

uniform mat4 viewProjection;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;

void main()
{
    vNormal = normal;
    vColor = color;
    vWorld = position;
    gl_Position = viewProjection * vec4(position, 1.0);
}
"""

    let terrainFragment =
        """#version 410 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;

uniform vec3 sunDirection;
uniform vec3 cameraPosition;

out vec4 fragColor;

// Cheap value noise so bare dirt doesn't read as a flat poster.
float hash(vec2 p)
{
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

void main()
{
    vec3 n = normalize(vNormal);
    float sun = max(dot(n, sunDirection), 0.0);
    // Hemispheric ambient: sky-blue-grey from above, ground bounce below.
    vec3 ambient = mix(vec3(0.18, 0.16, 0.14), vec3(0.35, 0.37, 0.40), n.y * 0.5 + 0.5);
    float grain = mix(0.92, 1.08, hash(floor(vWorld.xz * 8.0)));
    vec3 lit = vColor * grain * (ambient + vec3(1.0, 0.96, 0.88) * sun * 0.9);

    // Distance fog toward an overcast horizon.
    float dist = length(vWorld - cameraPosition);
    float fog = 1.0 - exp(-dist * 0.012);
    vec3 sky = vec3(0.63, 0.66, 0.70);
    fragColor = vec4(mix(lit, sky, fog), 1.0);
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

uniform vec3 sunDirection;
uniform vec3 cameraPosition;

out vec4 fragColor;

void main()
{
    vec3 n = normalize(vNormal);
    float sun = max(dot(n, sunDirection), 0.0);
    vec3 ambient = mix(vec3(0.16, 0.14, 0.12), vec3(0.33, 0.35, 0.38), n.y * 0.5 + 0.5);
    vec3 lit = vColor * (ambient + vec3(1.0, 0.96, 0.88) * sun * 0.9);
    float dist = length(vWorld - cameraPosition);
    float fog = 1.0 - exp(-dist * 0.012);
    fragColor = vec4(mix(lit, vec3(0.63, 0.66, 0.70), fog), 1.0);
}
"""
