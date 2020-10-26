# TATPhysics
Thealmighty's physic engine

visual studio options:
===========================================================================
C/C++:Preprocessor Definitinos:
_X86_
---------------------------------------------------------------------------
Linker:Additional Library Directiories:
TATPhysics\ThirdParty\glew-2.1.0\lib\Release\Win32
TATPhysics\ThirdParty\glfw-3.3.bin.WIN32\lib-vc2019 (or your vs version)
---------------------------------------------------------------------------
Linker:Additional Dependencies:
opengl32.lib
glfw3.lib
glew32s.lib
===========================================================================
CUDA:
project->Build dependencies->Build Customization->choose CUDA 
cudatest.cu->General->Item Type->CUDA C/C++

Include Directories:$(CUDA_PATH)\include
Libarary Directories:$(CUDA_PATH)\lib\Win32
Linker:Additional Dependencies:
cuda.lib
cudadevrt.lib
cudart.lib
cudart_static.lib
nvcuvid.lib
OpenCL.lib