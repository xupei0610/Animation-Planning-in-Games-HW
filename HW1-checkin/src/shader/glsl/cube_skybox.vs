"#version 330 core\n"
"layout (location = 0) in vec3 v;"
""
"out vec3 tex_coords;"
""
"layout (std140) uniform GlobalAttributes"
"{"
"   mat4 view;"
"   mat4 proj;"
"   vec3 cam_pos;"
"};"
""
"void main()"
"{"
"    tex_coords = v;"
""
"    vec4 pos = proj * mat4(mat3(view)) * vec4(v, 1.f);"
"    gl_Position = pos.xyww;"
"}"
