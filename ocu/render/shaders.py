"""GLSL for the scene renderer.

One lighting model throughout, so a rock beside a tree beside the ground reads
as lit by one sun: a directional sun with a shadow map, a hemisphere of sky
light with the ground bouncing warm light back up, aerial perspective that
bleeds distance into the horizon colour, and a filmic tone curve on the way
out. The ground and the rocks are textured in the shader from noise anchored to
the world, so the texture neither swims as the camera moves nor repeats.
"""

COMMON = """
#version 330
uniform vec3 u_sun;          // unit vector toward the sun
uniform vec3 u_cam;
uniform float u_time;
uniform float u_fog;         // extinction per metre
uniform float u_exposure;
uniform float u_grain;       // sensor noise, as a fraction of full scale

float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}
float hash13(vec3 p3) {
    p3 = fract(p3 * 0.1031);
    p3 += dot(p3, p3.zyx + 31.32);
    return fract((p3.x + p3.y) * p3.z);
}
float vnoise2(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash12(i), hash12(i + vec2(1, 0)), u.x),
               mix(hash12(i + vec2(0, 1)), hash12(i + vec2(1, 1)), u.x), u.y);
}
float vnoise3(vec3 p) {
    vec3 i = floor(p), f = fract(p);
    vec3 u = f * f * (3.0 - 2.0 * f);
    float a = mix(mix(hash13(i), hash13(i + vec3(1, 0, 0)), u.x),
                  mix(hash13(i + vec3(0, 1, 0)), hash13(i + vec3(1, 1, 0)), u.x), u.y);
    float b = mix(mix(hash13(i + vec3(0, 0, 1)), hash13(i + vec3(1, 0, 1)), u.x),
                  mix(hash13(i + vec3(0, 1, 1)), hash13(i + vec3(1, 1, 1)), u.x), u.y);
    return mix(a, b, u.z);
}
float fbm2(vec2 p) {
    float t = 0.0, a = 0.5;
    for (int i = 0; i < 4; i++) { t += a * vnoise2(p); p = p * 2.03 + 7.1; a *= 0.5; }
    return t / 0.9375;
}
float fbm3(vec3 p) {
    float t = 0.0, a = 0.5;
    for (int i = 0; i < 3; i++) { t += a * vnoise3(p); p = p * 2.07 + 3.3; a *= 0.5; }
    return t / 0.875;
}

vec3 skyColor(vec3 d) {
    float h = clamp(d.z, 0.0, 1.0);
@SKY@
    float s = max(dot(d, u_sun), 0.0);
    col += vec3(1.0, 0.78, 0.52) * (pow(s, 6.0) * 0.20 + pow(s, 64.0) * 0.25);
    col += vec3(1.0, 0.92, 0.80) * pow(s, 900.0) * 14.0;
    col += vec3(0.32, 0.20, 0.09) * pow(s, 2.0) * (1.0 - h) * 0.35;
    return col;
}

vec3 fogColor(vec3 view) {
    // Haze takes the colour of the sky it sits in front of, warmer toward the sun.
    float s = max(dot(normalize(vec3(view.xy, 0.0)), normalize(vec3(u_sun.xy, 0.0001))), 0.0);
    vec3 base = @HAZE@;
    return base + vec3(0.28, 0.18, 0.08) * pow(s, 3.0) * 0.30;
}

vec3 applyFog(vec3 col, float dist, vec3 view) {
    float f = 1.0 - exp(-pow(dist * u_fog, 1.4));
    // Height-dependent: the haze lies in the low ground.
    return mix(col, fogColor(view), clamp(f, 0.0, 0.94));
}

vec3 aces(vec3 x) {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}
vec3 finish(vec3 linear) {
    vec3 c = aces(linear * u_exposure);
    // A touch of saturation and contrast, as a camera's pipeline adds.
    float l = dot(c, vec3(0.299, 0.587, 0.114));
    c = mix(vec3(l), c, 1.10);
    c = pow(c, vec3(1.0 / 2.2));
    // Sensor noise: a camera is never clean, and the grain moves every frame.
    float g = hash12(gl_FragCoord.xy + vec2(fract(u_time * 7.3) * 311.0, fract(u_time * 3.1) * 177.0)) - 0.5;
    float g2 = hash12(gl_FragCoord.yx * 1.7 + vec2(fract(u_time * 5.7) * 97.0, 13.0)) - 0.5;
    return clamp(c + vec3(g, (g + g2) * 0.5, g2) * u_grain * 2.0, 0.0, 1.0);
}
"""

SHADOWED = """
uniform mat4 u_shadow_vp;
uniform sampler2DShadow u_shadow;
float shadowAt(vec3 world, vec3 n) {
    vec4 sp = u_shadow_vp * vec4(world + n * 0.06, 1.0);
    vec3 c = sp.xyz / sp.w * 0.5 + 0.5;
    if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;
    float texel = 1.0 / 4096.0;
    float lit = 0.0;
    for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
            lit += texture(u_shadow, vec3(c.xy + vec2(x, y) * texel * 1.2, c.z - 0.0008));
    return lit / 9.0;
}
vec3 sunLight() { return vec3(1.0, 0.92, 0.78) * 3.2; }
vec3 skyLight(vec3 n) {
    float up = n.z * 0.5 + 0.5;
    vec3 sky = @AMBIENT@;
    vec3 bounce = vec3(0.26, 0.21, 0.14);
    return mix(bounce, sky, up) * 0.66;
}
"""

def _fill(text: str) -> str:
    from ocu.render import palette

    return (text.replace("@SKY@", palette.sky_glsl())
            .replace("@HAZE@", "vec3({:.5f}, {:.5f}, {:.5f})".format(*palette.haze_colour()))
            .replace("@AMBIENT@", "vec3({:.5f}, {:.5f}, {:.5f})".format(*palette.ambient_sky())))


COMMON = _fill(COMMON)
SHADOWED = _fill(SHADOWED)

TERRAIN_VERT = """
uniform mat4 u_vp;
in vec3 in_pos;
in vec3 in_normal;
in vec4 in_mat;
in vec4 in_extra;
out vec3 v_pos;
out vec3 v_normal;
out vec4 v_mat;
out vec4 v_extra;
void main() {
    v_pos = in_pos; v_normal = in_normal; v_mat = in_mat; v_extra = in_extra;
    gl_Position = u_vp * vec4(in_pos, 1.0);
}
"""

TERRAIN_FRAG = COMMON + SHADOWED + """
in vec3 v_pos;
in vec3 v_normal;
in vec4 v_mat;
in vec4 v_extra;
out vec4 f_color;

uniform int u_has_tex;
uniform sampler2DArray u_g_albedo;   // ground, scrub, track, dark, slope
uniform sampler2DArray u_g_normal;
uniform float u_footage;             // 1 when the ground photographs are the reference footage's, not Poly Haven's

// A tiling photo texture sampled twice at unrelated scales and rotations, so the
// repeat does not show.
vec3 layer_grade(int layer) {
    // The footage's tiles are already this scene's ground, at the colour it was measured to be.
    if (u_footage > 0.5) return vec3(1.0);
    // The photographs are graded toward this scene's ground: warm, dry, a little
    // darker than a studio capture, olive where there is vegetation.
    if (layer == 0) return vec3(0.74, 0.60, 0.46);
    if (layer == 1) return vec3(0.62, 0.70, 0.40);
    if (layer == 2) return vec3(0.86, 0.76, 0.62);
    if (layer == 3) return vec3(0.62, 0.50, 0.40);
    return vec3(0.82, 0.78, 0.72);
}
// Metres one tile covers. The footage's tiles are patches of ground about a metre across.
float tile_a() { return u_footage > 0.5 ? 1.35 : 2.6; }
float tile_b() { return u_footage > 0.5 ? 3.3 : 6.1; }
vec3 tex_albedo(int layer, vec2 xy) {
    vec2 a = xy / tile_a();
    vec2 b = mat2(0.80, -0.60, 0.60, 0.80) * xy / tile_b() + vec2(17.0, 5.0);
    vec3 c = pow(0.5 * (texture(u_g_albedo, vec3(a, layer)).rgb + texture(u_g_albedo, vec3(b, layer)).rgb), vec3(2.2));
    return c * layer_grade(layer);
}
vec2 tex_normal(int layer, vec2 xy) {
    vec2 a = xy / tile_a();
    vec2 b = mat2(0.80, -0.60, 0.60, 0.80) * xy / tile_b() + vec2(17.0, 5.0);
    vec2 na = texture(u_g_normal, vec3(a, layer)).xy * 2.0 - 1.0;
    vec2 nb = mat2(0.80, 0.60, -0.60, 0.80) * (texture(u_g_normal, vec3(b, layer)).xy * 2.0 - 1.0);
    return 0.5 * (na + nb);
}

void main() {
    vec3 p = v_pos;
    float dist = length(p - u_cam);
    vec3 view = normalize(p - u_cam);
    float near = clamp(1.0 - dist / 38.0, 0.0, 1.0);   // fine detail fades before it can alias
    float mid = clamp(1.0 - dist / 90.0, 0.0, 1.0);

    float n1 = fbm2(p.xy * 0.22);
    float n2 = vnoise2(p.xy * 1.6);
    float n3 = vnoise2(p.xy * 7.0);
    float n4 = near > 0.0 ? vnoise2(p.xy * 31.0) : 0.5;
    float n5 = near > 0.0 ? vnoise2(p.xy * 90.0) : 0.5;

    vec3 soil  = mix(vec3(0.34, 0.25, 0.16), vec3(0.46, 0.36, 0.23), n1);
    vec3 straw = mix(vec3(0.50, 0.43, 0.22), vec3(0.40, 0.37, 0.17), n1);
    vec3 olive = mix(vec3(0.18, 0.24, 0.09), vec3(0.28, 0.31, 0.12), n1);
    vec3 track = vec3(0.52, 0.45, 0.34) * (0.92 + 0.16 * n2);
    vec3 dark  = mix(vec3(0.20, 0.15, 0.10), vec3(0.28, 0.22, 0.15), n2);

    vec3 scrub = mix(straw, olive, smoothstep(0.25, 0.60, n1 + 0.3 * (n2 - 0.5)));
    vec3 albedo = mix(soil, scrub, 0.35 + 0.35 * smoothstep(0.35, 0.65, n1)) * v_mat.x + scrub * v_mat.y + track * v_mat.z + dark * v_mat.w;

    // Grass: blades are short strokes, so a fine anisotropic grain on vegetated ground.
    float veg = clamp(v_mat.y * 1.2 + v_mat.x * 0.35, 0.0, 1.0);
    float blades = near > 0.0 ? vnoise2(vec2(p.x * 38.0 + n3 * 3.0, p.y * 9.0)) * 0.6 + vnoise2(vec2(p.x * 9.0, p.y * 38.0 + n3 * 3.0)) * 0.4 : 0.5;
    albedo *= 1.0 + veg * near * 0.55 * (blades - 0.5);
    albedo *= 0.86 + 0.20 * n2 + 0.20 * (n3 - 0.5) * clamp(1.0 - dist / 48.0, 0.0, 1.0) + 0.14 * (n4 - 0.5) * near + 0.08 * (n5 - 0.5) * near;
    // Green flush where the ground is wet (low in the field's moisture).
    albedo = mix(albedo, albedo * vec3(0.78, 1.0, 0.70), smoothstep(0.55, 0.9, n1) * veg * 0.6);

    // Surface normal with a bump from the same noise, so slopes catch the light
    // unevenly. Only worth its cost where it can be seen.
    vec3 n = v_normal;
    if (near > 0.0) {
        float e = 0.12;
        float h0 = vnoise2(p.xy * 5.0) * 0.5 + vnoise2(p.xy * 19.0) * 0.25;
        float hx = vnoise2((p.xy + vec2(e, 0)) * 5.0) * 0.5 + vnoise2((p.xy + vec2(e, 0)) * 19.0) * 0.25;
        float hy = vnoise2((p.xy + vec2(0, e)) * 5.0) * 0.5 + vnoise2((p.xy + vec2(0, e)) * 19.0) * 0.25;
        n = normalize(v_normal + vec3(-(hx - h0), -(hy - h0), 0.0) * (0.9 * near + 0.25));
    }

    // Steep ground sheds soil and shows stone.
    float slope = 1.0 - v_normal.z;
    float rockiness = smoothstep(0.20, 0.42, slope);
    vec3 stone = vec3(0.40, 0.37, 0.33) * (0.8 + 0.4 * n2);
    albedo = mix(albedo, stone, rockiness);

    if (u_has_tex != 0) {
        // Photographic ground. Only the layers that matter here are read.
        float wg = v_mat.x, ws = v_mat.y, wt = v_mat.z, wd = v_mat.w;
        if (u_footage > 0.5) {
            // Open ground in the footage is not bare sand: it is sand under dry grass, leaves and stalks. Most of it takes the straw.
            ws += wg * 0.62;
            wg *= 0.38;
        }
        vec3 tex = vec3(0.0);
        vec2 nrm = vec2(0.0);
        float total = 0.0;
        if (wg > 0.02) { tex += wg * tex_albedo(0, p.xy); nrm += wg * tex_normal(0, p.xy); total += wg; }
        if (ws > 0.02) { tex += ws * tex_albedo(1, p.xy); nrm += ws * tex_normal(1, p.xy); total += ws; }
        if (wt > 0.02) { tex += wt * tex_albedo(2, p.xy); nrm += wt * tex_normal(2, p.xy); total += wt; }
        if (wd > 0.02) { tex += wd * tex_albedo(3, p.xy); nrm += wd * tex_normal(3, p.xy); total += wd; }
        tex /= max(total, 0.001); nrm /= max(total, 0.001);
        if (rockiness > 0.02) {
            tex = mix(tex, tex_albedo(4, p.xy), rockiness);
            nrm = mix(nrm, tex_normal(4, p.xy), rockiness);
        }
        // The photographs carry the detail; the procedural colour carries the
        // large-scale variation (wet and dry patches) a repeating texture cannot.
        float large = 0.72 + 0.56 * n1;
        vec3 photo = tex * large * 1.25;
        albedo = mix(albedo, photo, 0.88);
        n = normalize(n + vec3(nrm * 0.75 * (0.35 + 0.65 * near), 0.0));
    }
    // Country past the edge of the simulated ground: forest canopy, lumpy and dark in the folds, dry and pale in patches
    // on the open slopes, as in the footage's hilltop clip. It is only ever seen from far off, through the haze.
    float forest = v_extra.y;
    if (forest > 0.01) {
        float c1 = fbm2(p.xy * 0.040);
        float c2 = vnoise2(p.xy * 0.33);
        float c3 = vnoise2(p.xy * 1.9);
        vec3 canopy = mix(vec3(0.030, 0.046, 0.020), vec3(0.092, 0.112, 0.042), smoothstep(0.25, 0.75, 0.6 * c1 + 0.5 * c2));
        canopy = mix(canopy, vec3(0.17, 0.13, 0.060), smoothstep(0.60, 0.86, c1 + 0.25 * (c3 - 0.5)) * 0.8);
        canopy *= 0.72 + 0.56 * c3;
        albedo = mix(albedo, canopy, forest);
    }
    // A hole is bare earth, darker, with the rim scuffed.
    albedo = mix(albedo, albedo * 0.45 + vec3(0.05, 0.035, 0.02), clamp(v_extra.x * 3.0, 0.0, 0.9));

    float sh = shadowAt(p, v_normal);
    float diff = max(dot(n, u_sun), 0.0) * sh;
    vec3 lit = albedo * (sunLight() * diff + skyLight(n) * (0.55 + 0.45 * sh));
    // Contact darkening in hollows.
    lit *= 1.0 - clamp(v_extra.x * 1.6, 0.0, 0.5);

    lit = applyFog(lit, dist, view);
    f_color = vec4(finish(lit), 1.0);
}
"""

INSTANCE_VERT = """
uniform mat4 u_vp;
uniform float u_time;
uniform int u_kind;
in vec3 in_pos;
in vec3 in_normal;
in vec3 in_rgb;
in float in_ao;
in vec4 i_pos_yaw;
in vec4 i_scale;
in vec4 i_tint;
out vec3 v_world;
out vec3 v_normal;
out vec3 v_local;
out vec3 v_rgb;
out float v_ao;
out vec4 v_tint;
out float v_seed;
out float v_wood;
void main() {
    v_wood = step(1.5, length(in_normal));   // wood is marked by normals of length two (render/meshes.py)
    vec3 p = in_pos * i_scale.xyz;
    if (u_kind == 1 && i_tint.a > 0.0 && length(in_normal) < 1.5) {   // wood (normals of length 2) does not wobble
        // Leafy crowns have a broken outline: push the surface in and out along
        // its normal with noise, so a smooth lobe reads as a mass of leaves.
        vec3 q = (in_pos + vec3(i_scale.w)) * 9.0;
        float bump = fract(sin(dot(floor(q * 1.7), vec3(12.9898, 78.233, 37.719))) * 43758.5453);
        float smooth_bump = sin(q.x * 2.3 + q.y * 1.9) * sin(q.y * 2.7 - q.z * 2.1) * sin(q.z * 3.1 + q.x * 1.3);
        p += normalize(in_normal) * (0.35 * smooth_bump + 0.15 * (bump - 0.5)) * 0.06 * max(i_scale.x, i_scale.z) * step(0.45, in_pos.z);
    }
    float sway = i_tint.a;
    if (sway > 0.0) {
        float hf = clamp(in_pos.z, 0.0, 1.0);
        float ph = i_pos_yaw.x * 0.71 + i_pos_yaw.y * 0.43;
        float gust = sin(u_time * 0.6 + ph * 0.3) * 0.5 + 0.5;
        p.xy += vec2(sin(u_time * 1.15 + ph), cos(u_time * 0.93 + ph * 1.3))
                * sway * hf * hf * i_scale.z * (0.010 + 0.014 * gust);
    }
    float c = cos(i_pos_yaw.w), s = sin(i_pos_yaw.w);
    vec3 r = vec3(p.x * c - p.y * s, p.x * s + p.y * c, p.z);
    v_world = r + i_pos_yaw.xyz;
    vec3 n = in_normal / max(i_scale.xyz, vec3(0.001));
    n = normalize(n);
    v_normal = vec3(n.x * c - n.y * s, n.x * s + n.y * c, n.z);
    v_local = in_pos;
    v_rgb = in_rgb;
    v_ao = in_ao;
    v_tint = i_tint;
    v_seed = i_scale.w;
    gl_Position = u_vp * vec4(v_world, 1.0);
}
"""

INSTANCE_FRAG = COMMON + SHADOWED + """
uniform int u_kind;        // 0 rock, 1 foliage, 2 machine, 3 grass, 4 figure
uniform float u_grade;     // how far a figure is matched to a photograph beside it: 0 none, 1 fully
in vec3 v_world;
in vec3 v_normal;
in vec3 v_local;
in vec3 v_rgb;
in float v_ao;
in vec4 v_tint;
in float v_seed;
in float v_wood;
out vec4 f_color;

void main() {
    vec3 view = normalize(v_world - u_cam);
    float dist = length(v_world - u_cam);
    vec3 n = normalize(v_normal);
    if (!gl_FrontFacing) n = -n;
    vec3 albedo = v_rgb * v_tint.rgb;
    float ao = v_ao;
    float leaf = 0.0;
    // Figures (kind 4): the vertex says what it is made of, packed with its occlusion.
    int mat = 0;
    float soft = 0.0, specAmt = 0.0, specPow = 40.0, reflAmt = 0.0, fresAmt = 0.0, emit = 0.0, sss = 0.0;
    if (u_kind == 4) {
        mat = int(floor(v_ao * 0.5 + 0.001));
        ao = v_ao - 2.0 * float(mat);
    }

    if (u_kind == 0) {
        // Weathered stone: banding, lichen on the sunward tops, cracks, a rough grain.
        vec3 q = v_world * 2.3 + v_seed;
        float grain = fbm3(q * 2.5);
        float fine = vnoise3(v_world * 38.0 + v_seed);
        float band = sin(v_local.z * 9.0 + grain * 5.0) * 0.5 + 0.5;
        // Warm grey to brown, differently for each rock, with streaks down the faces.
        vec3 stone = mix(vec3(0.30, 0.28, 0.25), vec3(0.42, 0.36, 0.29), fbm3(q * 0.7 + 3.0));
        float streak = fbm3(vec3(v_world.xy * 5.0, v_world.z * 1.2) + v_seed);
        albedo = stone * (0.55 + 0.85 * grain) * (0.88 + 0.22 * band) * (0.85 + 0.3 * fine);
        albedo *= 0.75 + 0.5 * streak;
        float crack = smoothstep(0.46, 0.5, abs(vnoise3(q * 3.3) - 0.5));
        albedo *= mix(0.45, 1.0, crack);
        float lichen = smoothstep(0.55, 0.9, n.z) * smoothstep(0.5, 0.72, fbm3(q * 1.7 + 11.0));
        albedo = mix(albedo, vec3(0.22, 0.28, 0.11) * (0.7 + 0.5 * grain), lichen * 0.7);
        float dust = clamp(1.0 - v_local.z * 1.4, 0.0, 1.0);
        albedo = mix(albedo, vec3(0.34, 0.26, 0.17), dust * 0.35);
        n = normalize(n + (vec3(vnoise3(q * 5.0), vnoise3(q * 5.0 + 4.0), vnoise3(q * 5.0 + 9.0)) - 0.5) * 0.9
                        + (vec3(vnoise3(v_world * 31.0), vnoise3(v_world * 31.0 + 3.0), vnoise3(v_world * 31.0 + 7.0)) - 0.5) * 0.35);
    } else if (u_kind == 1 && v_wood > 0.5) {
        // Bark: dark, furrowed along the limb, paler where the furrows catch light, lichen-grey in patches. Lit as a
        // solid, not wrapped round like a leaf.
        float furrow = vnoise3(vec3(v_world.xy * 34.0, v_world.z * 2.2) + v_seed) * 0.65 + vnoise3(vec3(v_world.xy * 11.0, v_world.z * 1.2) + v_seed) * 0.35;
        float fine = vnoise3(vec3(v_world.xy * 95.0, v_world.z * 9.0) + v_seed);
        albedo *= 0.38 + 1.25 * furrow;
        albedo *= 0.80 + 0.4 * fine;
        albedo = mix(albedo, vec3(0.20, 0.19, 0.17), smoothstep(0.62, 0.85, fbm3(v_world * 3.0 + v_seed)) * 0.45);
        n = normalize(n + (vec3(furrow, vnoise3(v_world * 30.0 + 7.0), 0.0) - vec3(0.5, 0.5, 0.0)) * 0.7);
    } else if (u_kind == 1) {
        leaf = 1.0;
        vec3 q = v_world * 3.4 + v_seed;
        float clump = fbm3(q);
        float speck = vnoise3(v_world * 22.0 + v_seed);
        albedo *= 0.70 + 0.62 * clump;
        albedo *= 0.86 + 0.28 * speck;
        // Sun-bleached tips toward the top of the crown.
        albedo = mix(albedo, albedo * vec3(1.10, 1.05, 0.85), smoothstep(0.55, 1.0, v_local.z) * 0.25);
        // Leaf facets: the crown's surface is thousands of small planes, not one
        // smooth skin, and the light breaks up over them.
        vec3 facet = vec3(vnoise3(v_world * 9.0 + v_seed), vnoise3(v_world * 9.0 + v_seed + 5.0), vnoise3(v_world * 9.0 + v_seed + 11.0)) - 0.5;
        n = normalize(n + facet * 1.1);
    } else if (u_kind == 3) {
        leaf = 1.0;
        albedo *= 0.85 + 0.3 * vnoise3(v_world * 3.0);
    } else if (u_kind == 4) {
        // Dust kicked up to the height this figure stands in it, thicker in the hollows of a noisy field.
        float field_n = fbm3(v_world * 2.6);
        float dust = clamp(1.0 - v_local.z / max(v_seed, 0.05), 0.0, 1.0) * (0.45 + 0.55 * field_n);
        dust += smoothstep(0.75, 1.0, n.z) * 0.14 * field_n;
        dust = clamp(dust, 0.0, 0.85);
        vec3 dirt = vec3(0.46, 0.37, 0.27) * (0.8 + 0.4 * field_n);
        if (mat == 0) {            // paint under a clear coat, and dust
            float scratch = smoothstep(0.62, 0.7, vnoise3(v_world * 26.0 + vec3(0.0, 0.0, 3.0)));
            albedo *= 1.0 - 0.10 * scratch;
            albedo = mix(albedo, dirt, clamp(dust * 0.75, 0.0, 0.85));
            specAmt = 0.9 * (1.0 - dust); specPow = 110.0; reflAmt = 0.05 * (1.0 - dust); fresAmt = 0.35 * (1.0 - dust);
        } else if (mat == 1) {     // glass: dark, and it shows the sky
            albedo = vec3(0.012, 0.017, 0.022) + dirt * 0.05 * dust;
            specAmt = 1.3; specPow = 200.0; reflAmt = 0.16; fresAmt = 0.85;
        } else if (mat == 2) {     // rubber
            albedo *= 0.72 + 0.56 * vnoise3(v_world * 55.0);
            albedo = mix(albedo, dirt, dust * 0.55);
            specAmt = 0.04;
        } else if (mat == 3) {     // dark plastic and metal
            albedo = mix(albedo, dirt, dust * 0.4);
            specAmt = 0.3; specPow = 28.0; fresAmt = 0.1;
        } else if (mat == 4) {     // cloth: a weave, folds, dust at the hem
            albedo *= 0.86 + 0.28 * vnoise3(v_world * 120.0);
            n = normalize(n + (vec3(vnoise3(v_world * 13.0), vnoise3(v_world * 13.0 + 4.0), vnoise3(v_world * 13.0 + 9.0)) - 0.5) * 0.55);
            albedo = mix(albedo, dirt, dust * 0.5);
            soft = 0.35; specAmt = 0.02;
        } else if (mat == 5) {     // skin
            albedo *= 0.93 + 0.14 * vnoise3(v_world * 34.0);
            soft = 0.55; specAmt = 0.14; specPow = 18.0; sss = 0.35;
        } else if (mat == 6) {     // hide
            albedo *= 0.76 + 0.48 * vnoise3(v_world * vec3(70.0, 70.0, 26.0));
            albedo = mix(albedo, dirt, dust * 0.4);
            n = normalize(n + (vec3(vnoise3(v_world * 50.0), vnoise3(v_world * 50.0 + 3.0), vnoise3(v_world * 50.0 + 8.0)) - 0.5) * 0.35);
            soft = 0.3; specAmt = 0.04; fresAmt = 0.12; sss = 0.12;
        } else if (mat == 7) {     // bright metal
            albedo = mix(albedo, dirt, dust * 0.45);
            specAmt = 1.0; specPow = 120.0; reflAmt = 0.42 * (1.0 - dust); fresAmt = 0.4;
        } else if (mat == 8) {     // a lamp lens
            specAmt = 1.2; specPow = 150.0; reflAmt = 0.25; fresAmt = 0.5; emit = 0.12;
        } else if (mat == 10) {    // antler: ridged bone, polished at the tips
            albedo *= 0.8 + 0.4 * vnoise3(v_world * vec3(80.0, 80.0, 24.0));
            albedo = mix(albedo, dirt, dust * 0.2);
            specAmt = 0.22; specPow = 24.0; soft = 0.2;
        } else {                   // hair
            soft = 0.4; specAmt = 0.25; specPow = 14.0;
        }
    } else {
        // machine: small specular and no dirt
    }

    float sh = shadowAt(v_world, n);
    float wrap = leaf > 0.5 ? clamp((dot(n, u_sun) + 0.45) / 1.45, 0.0, 1.0)
               : (soft > 0.0 ? clamp((dot(n, u_sun) + soft) / (1.0 + soft), 0.0, 1.0) : max(dot(n, u_sun), 0.0));
    vec3 lit = albedo * (sunLight() * wrap * sh + skyLight(n) * (0.55 + 0.45 * sh)) * (0.30 + 0.80 * ao);
    if (leaf > 0.5) {
        // Light through leaves when the sun is behind them.
        float through = pow(max(dot(-view, u_sun), 0.0), 3.0) * (1.0 - clamp(dot(n, u_sun), 0.0, 1.0));
        lit += albedo * vec3(0.55, 0.65, 0.20) * through * 1.6 * sh * ao;
    }
    if (u_kind == 2) {
        vec3 h = normalize(u_sun - view);
        lit += vec3(1.0, 0.95, 0.85) * pow(max(dot(n, h), 0.0), 40.0) * 0.5 * sh;
    }
    if (u_kind == 4) {
        vec3 h = normalize(u_sun - view);
        lit += vec3(1.0, 0.95, 0.85) * pow(max(dot(n, h), 0.0), specPow) * specAmt * sh;
        // The sky in a glossy surface, more of it at a grazing angle, and the ground's bounce below.
        vec3 refl = reflect(view, n);
        vec3 env = refl.z > 0.0 ? skyColor(refl) : vec3(0.30, 0.24, 0.17) * 0.8;
        float fres = pow(1.0 - clamp(dot(n, -view), 0.0, 1.0), 4.0);
        lit = mix(lit, env * (0.35 + 0.65 * sh), clamp(reflAmt + fres * fresAmt, 0.0, 0.92));
        // Light through thin flesh when the sun is behind it.
        lit += albedo * vec3(0.70, 0.18, 0.06) * pow(max(dot(-view, u_sun), 0.0), 2.0) * sss * sh;
        lit += albedo * emit;
        if (u_grade > 0.0) {
            // Clean flat colour is what marks a figure out among photographs: take some of the saturation off, warm it to the light the
            // photographs were taken in, and lift the blacks and flatten the contrast a little, as haze and a lens do.
            float lum = dot(lit, vec3(0.299, 0.587, 0.114));
            lit = mix(lit, vec3(lum), 0.32 * u_grade);
            lit *= mix(vec3(1.0), vec3(1.0, 0.95, 0.88), u_grade);
            lit = mix(lit, lit * 0.82 + vec3(0.035, 0.032, 0.026), u_grade);
        }
    }
    lit = applyFog(lit, dist, view);
    f_color = vec4(finish(lit), 1.0);
}
"""

SHADOW_TERRAIN_VERT = """
#version 330
uniform mat4 u_vp;
in vec3 in_pos;
void main() { gl_Position = u_vp * vec4(in_pos, 1.0); }
"""

SHADOW_INSTANCE_VERT = """
#version 330
uniform mat4 u_vp;
in vec3 in_pos;
in vec4 i_pos_yaw;
in vec4 i_scale;
void main() {
    vec3 p = in_pos * i_scale.xyz;
    float c = cos(i_pos_yaw.w), s = sin(i_pos_yaw.w);
    vec3 r = vec3(p.x * c - p.y * s, p.x * s + p.y * c, p.z) + i_pos_yaw.xyz;
    gl_Position = u_vp * vec4(r, 1.0);
}
"""

SHADOW_FRAG = """
#version 330
void main() {}
"""

SKY_VERT = """
#version 330
out vec2 v_ndc;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    v_ndc = p * 2.0 - 1.0;
    gl_Position = vec4(v_ndc, 0.9999, 1.0);
}
"""

SKY_FRAG = COMMON + """
uniform mat4 u_inv_vp;
in vec2 v_ndc;
out vec4 f_color;
void main() {
    vec4 far = u_inv_vp * vec4(v_ndc, 1.0, 1.0);
    vec3 d = normalize(far.xyz / far.w - u_cam);
    vec3 col = skyColor(d);
    // Clouds: a thin layer of broken cumulus, lit from the sun side.
    if (d.z > 0.01) {
        float t = 900.0 / d.z;
        vec2 uv = (u_cam.xy + d.xy * t) * 0.0011 + vec2(u_time * 0.0008, 0.0);
        float c = fbm2(uv * 3.0) * 0.75 + fbm2(uv * 9.0) * 0.25;
        float cover = smoothstep(0.66, 0.86, c) * 0.55;   // the footage's sky is nearly clear: a few thin wisps
        float edge = smoothstep(0.50, 0.62, c) - smoothstep(0.66, 0.80, c);
        vec3 cloud = mix(vec3(0.80, 0.84, 0.90), vec3(1.0, 0.98, 0.94), edge + 0.3 * fbm2(uv * 21.0));
        cloud *= 0.82 + 0.25 * max(dot(vec3(d.xy, 0.0), vec3(u_sun.xy, 0.0)), 0.0);
        col = mix(col, cloud, cover * smoothstep(0.01, 0.25, d.z) * 0.85);
    }
    f_color = vec4(finish(col), 1.0);
}
"""


PBR_VERT = """
uniform mat4 u_vp;
uniform float u_time;
in vec3 in_pos;
in vec3 in_normal;
in vec2 in_uv;
in vec4 i_pos_yaw;
in vec4 i_scale;
in vec4 i_tint;
out vec3 v_world;
out vec3 v_normal;
out vec2 v_uv;
out vec3 v_local;
out vec4 v_tint;
out float v_seed;
void main() {
    vec3 p = in_pos * i_scale.xyz;
    float sway = i_tint.a;
    if (sway > 0.0) {
        float hf = clamp(in_pos.z / 0.18, 0.0, 1.0);
        float ph = i_pos_yaw.x * 0.71 + i_pos_yaw.y * 0.43;
        p.xy += vec2(sin(u_time * 1.7 + ph), cos(u_time * 1.3 + ph * 1.3)) * sway * hf * hf * i_scale.z * 0.16;
    }
    float c = cos(i_pos_yaw.w), s = sin(i_pos_yaw.w);
    vec3 r = vec3(p.x * c - p.y * s, p.x * s + p.y * c, p.z);
    v_world = r + i_pos_yaw.xyz;
    vec3 n = normalize(in_normal / max(i_scale.xyz, vec3(0.001)));
    v_normal = vec3(n.x * c - n.y * s, n.x * s + n.y * c, n.z);
    v_uv = in_uv;
    v_local = in_pos;
    v_tint = i_tint;
    v_seed = i_scale.w;
    gl_Position = u_vp * vec4(v_world, 1.0);
}
"""

PBR_FRAG = COMMON + SHADOWED + """
uniform sampler2D u_albedo;
uniform sampler2D u_normal;
uniform sampler2D u_arm;
uniform int u_mode;        // 0 rock, 1 stone, 2 grass, 3 shrub
in vec3 v_world;
in vec3 v_normal;
in vec2 v_uv;
in vec3 v_local;
in vec4 v_tint;
in float v_seed;
out vec4 f_color;

void main() {
    vec3 view = normalize(v_world - u_cam);
    float dist = length(v_world - u_cam);
    vec3 geo = normalize(v_normal);
    if (!gl_FrontFacing) geo = -geo;

    vec3 raw = texture(u_albedo, v_uv).rgb;
    // The plant textures are painted on a black ground rather than carrying an
    // alpha channel; what is black is not leaf.
    if (u_mode >= 2 && max(raw.r, max(raw.g, raw.b)) < 0.06) discard;
    vec3 albedo = pow(raw, vec3(2.2)) * v_tint.rgb;
    vec3 arm = texture(u_arm, v_uv).rgb;
    float ao = arm.r;
    float rough = arm.g;

    // Normal map, with the tangent frame taken from the screen-space derivatives
    // of the position and the texture coordinate.
    vec3 dp1 = dFdx(v_world), dp2 = dFdy(v_world);
    vec2 du1 = dFdx(v_uv), du2 = dFdy(v_uv);
    vec3 perp2 = cross(dp2, geo), perp1 = cross(geo, dp1);
    vec3 T = perp2 * du1.x + perp1 * du2.x;
    vec3 B = perp2 * du1.y + perp1 * du2.y;
    float inv = inversesqrt(max(max(dot(T, T), dot(B, B)), 1e-12));
    vec3 nm = texture(u_normal, v_uv).xyz * 2.0 - 1.0;
    nm.xy *= 1.0;
    vec3 n = normalize(mat3(T * inv, -B * inv, geo) * nm);
    if (!(dot(n, n) > 0.5)) n = geo;

    if (u_mode == 0) {
        // Dust settled round the foot of a rock, and a little where the sun has bleached it.
        float foot = clamp(1.0 - v_local.z * 2.2, 0.0, 1.0);
        albedo = mix(albedo, vec3(0.22, 0.16, 0.10), foot * 0.35);
    }

    float sh = shadowAt(v_world, geo);
    float wrap = u_mode >= 2 ? clamp((dot(n, u_sun) + 0.4) / 1.4, 0.0, 1.0) : max(dot(n, u_sun), 0.0);
    vec3 lit = albedo * (sunLight() * wrap * sh + skyLight(n) * (0.5 + 0.5 * sh)) * (0.35 + 0.65 * ao);
    if (u_mode >= 2) {
        float through = pow(max(dot(-view, u_sun), 0.0), 3.0) * (1.0 - clamp(dot(geo, u_sun), 0.0, 1.0));
        lit += albedo * vec3(0.55, 0.62, 0.22) * through * 1.4 * sh;
    }
    vec3 h = normalize(u_sun - view);
    float shininess = mix(6.0, 70.0, 1.0 - rough);
    lit += vec3(1.0, 0.95, 0.85) * pow(max(dot(n, h), 0.0), shininess) * (1.0 - rough) * 0.22 * sh;

    lit = applyFog(lit, dist, view);
    f_color = vec4(finish(lit), 1.0);
}
"""


#: A soft dark patch on the ground, drawn after the photographs, so that a figure is standing in the picture and not pasted on it.
BLOB_VERT = """
uniform mat4 u_vp;
uniform vec3 u_centre;
uniform vec2 u_radius;     // along the shadow, across it
uniform vec2 u_dir;        // unit, the way the shadow falls
in vec2 in_pos;
out vec2 v_p;
void main() {
    v_p = in_pos;
    vec2 across = vec2(-u_dir.y, u_dir.x);
    vec2 w = u_centre.xy + u_dir * in_pos.x * u_radius.x + across * in_pos.y * u_radius.y;
    gl_Position = u_vp * vec4(w, u_centre.z + 0.15, 1.0);  // a little above the simulated ground that is in the depth buffer, which the figures stand on only to within its mesh
}
"""

BLOB_FRAG = """
#version 330
uniform float u_strength;
in vec2 v_p;
out vec4 f_color;
void main() {
    float r = length(v_p);
    float a = u_strength * (1.0 - smoothstep(0.1, 1.0, r));
    f_color = vec4(0.0, 0.0, 0.0, a);
}
"""
