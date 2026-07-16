"%VULKAN_SDK%\Bin\glslc.exe" static_mesh_shader.vert -o static_mesh_shader_vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" static_mesh_shader.frag -o static_mesh_shader_frag.spv
"%VULKAN_SDK%\Bin\glslc.exe" skeletal_mesh_shader.vert -o skeletal_mesh_shader_vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" skeletal_mesh_shader.frag -o skeletal_mesh_shader_frag.spv
pause
