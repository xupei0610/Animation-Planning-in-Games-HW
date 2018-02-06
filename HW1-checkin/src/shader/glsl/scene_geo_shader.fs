"#version 330 core\n"
""
"struct SpotLight"
"{"
"   vec3 pos;"
"   vec3 dir;"
"   float cutoff_outer;"
"   float cutoff_diff;"
"   "
"   vec3 ambient;"
"   vec3 diffuse;"
"   vec3 specular;"
""
"   float coef_a0;"
"   float coef_a1;"
"   float coef_a2;"
"};"
""
"struct Material"
"{"
"   vec3 ambient;"
"   sampler2D diffuse;"
"   sampler2D normal;"
"   sampler2D specular;"
"   sampler2D displace;"
"   float shininess;"
"   float displace_amp;"
"   float displace_mid;"
"   float parallel_height;"
"};"
""
"layout (location = 0) out vec3 gPosition;"
"layout (location = 1) out vec3 gColor;"
"layout (location = 2) out vec3 gView;"
"layout (location = 3) out vec3 gNormal;"
"layout (location = 4) out vec3 gDiffuse;"
"layout (location = 5) out vec4 gSpecular;"
""
"in vec2 tex_coords;"
"in vec3 norm;"
"in vec3 world_pos;"
"in mat3 TBN;"
""
"uniform SpotLight headlight;"
"uniform Material material;"
"uniform int use_tangent;"
""
"layout (std140) uniform GlobalAttributes"
"{"
"   mat4 view;"
"   mat4 proj;"
"   vec3 cam_pos;"
"};"
""
"uniform vec3 global_ambient;"
""
"void main() {"
""
"   vec3 V = normalize(cam_pos - world_pos);"
""
"   vec2 t_tex_coords = tex_coords;"
"   if (material.parallel_height != 0.f)"
"   {"
"       vec3 dv = texture(material.displace, tex_coords).xyz;"
"       float df = 0.30*dv.x + 0.59*dv.y + 0.11*dv.z - material.displace_mid;"
"       t_tex_coords += V.xy/V.z * (df * material.parallel_height);"
"       if (t_tex_coords.x < 0.f || t_tex_coords.x > 1.f || t_tex_coords.y < 0.f || t_tex_coords.y > 1.f)"
"        discard;"
"   }"
""
"   vec3 N;"
"   if (use_tangent == 1)"
"   {   "
"       N = texture(material.normal, t_tex_coords).rgb;"
"       N = normalize(N * 2.f - 1.f);"
"       N = normalize(TBN * N);"
"   }"
"   else"
"   {"
"       N = norm;"
"   }"
""
"   vec3 diffuse = texture(material.diffuse, t_tex_coords).rgb * material.ambient;"
"   vec3 specular = texture(material.specular, t_tex_coords).rgb;"
""
""  // headlight
"   vec3 L = normalize(headlight.pos - world_pos);"
//        "   vec3 R = reflect(-L, N);"
"   vec3 H = normalize(L + V);" // blinn phong
"   float spec = pow(abs(dot(N, H)), material.shininess);"
""
"   float cosine = -dot(L, normalize(headlight.dir));"
"   float dist = length(headlight.pos - world_pos);"
"   float atten = max(min((cosine - headlight.cutoff_outer)/headlight.cutoff_diff, 1), 0);"
"   atten *= 1.f / (headlight.coef_a0 + headlight.coef_a1 * dist + headlight.coef_a2*dist*dist);"
""
"   vec3 h_ambient = headlight.ambient * diffuse;"
"   vec3 h_diffuse = abs(dot(L, N)) * headlight.diffuse * diffuse;"
"   vec3 h_specular = specular * headlight.specular * spec;"
""
"   gColor = global_ambient * diffuse + (h_ambient + h_diffuse + h_specular) * atten;"
"   gPosition = world_pos;"
"   gView = V;"
"   gNormal = N;"
"   gDiffuse = diffuse;"
"   gSpecular = vec4(specular, material.shininess);"
"}"
""