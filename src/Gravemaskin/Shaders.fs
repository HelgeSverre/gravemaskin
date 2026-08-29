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

    let terrainFragment =
        """#version 410 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;
in float vLayer;

uniform vec3 sunDirection;
uniform vec3 cameraPosition;
uniform sampler2DArray detailTextures;

out vec4 fragColor;

void main()
{
    vec3 n = normalize(vNormal);
    float sun = max(dot(n, sunDirection), 0.0);
    // Hemispheric ambient: sky-blue-grey from above, ground bounce below.
    vec3 ambient = mix(vec3(0.18, 0.16, 0.14), vec3(0.35, 0.37, 0.40), n.y * 0.5 + 0.5);

    // Two scales of the per-material detail texture (tileable, generated at
    // startup): close detail + a broader octave to break the repeat.
    vec3 detailNear = texture(detailTextures, vec3(vWorld.xz * 0.9, vLayer)).rgb;
    vec3 detailFar = texture(detailTextures, vec3(vWorld.xz * 0.13, vLayer)).rgb;
    vec3 detail = detailNear * detailFar * 4.0;

    vec3 lit = vColor * detail * (ambient + vec3(1.0, 0.96, 0.88) * sun * 0.9);

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

uniform vec3 sunDirection;
uniform vec3 cameraPosition;

out vec4 fragColor;

void main()
{
    vec3 n = normalize(vNormal);
    float sun = max(dot(n, sunDirection), 0.0);
    vec3 ambient = mix(vec3(0.16, 0.14, 0.12), vec3(0.33, 0.35, 0.38), n.y * 0.5 + 0.5);
    vec3 viewDirection = normalize(cameraPosition - vWorld);
    float spec = pow(max(dot(reflect(-sunDirection, n), viewDirection), 0.0), 28.0) * 0.4;
    vec3 lit = vColor * (ambient + vec3(1.0, 0.96, 0.88) * sun * 0.9) + vec3(spec);
    float dist = length(vWorld - cameraPosition);
    float fog = 1.0 - exp(-dist * 0.012);
    fragColor = vec4(mix(lit, vec3(0.63, 0.66, 0.70), fog), 1.0);
}
"""

    /// Ground-hugging blob shadow: a disc that fades radially.
    let blobVertex =
        """#version 410 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

uniform mat4 viewProjection;
uniform mat4 model;

out vec2 vDisc;

void main()
{
    vDisc = position.xz * 2.0;
    gl_Position = viewProjection * (model * vec4(position, 1.0));
}
"""

    let blobFragment =
        """#version 410 core
in vec2 vDisc;

out vec4 fragColor;

void main()
{
    float alpha = (1.0 - smoothstep(0.35, 1.0, length(vDisc))) * 0.42;
    fragColor = vec4(0.02, 0.02, 0.03, alpha);
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
