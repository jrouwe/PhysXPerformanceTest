# SnippetPerformanceTest

A code snippet to be included into the PhysX repository to compare performance between Jolt and PhysX

To run:

- Clone the PhysX 4.1 repository at https://github.com/NVIDIAGameWorks/PhysX.git
- Add this repository as a submodule in physx\snippets\snippetperformancetest
- Edit physx\snippets\compiler\cmake\CMakeLists.txt and add PerformanceTest to the variable SNIPPETS_LIST.
- Edit physx\snippets\snippetrender\SnippetRender.cpp and change MAX_NUM_MESH_VEC3S to 200000
- Follow the build instructions of the PhysX library to compile the library and all snippets