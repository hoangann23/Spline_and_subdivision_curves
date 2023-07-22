# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("CSE167_FA22_HW4")

# Add sources
file(
	GLOB SOURCES_CURVES
	src/*.cpp
)

# Add executables
add_executable(Curves ${SOURCES_CURVES})

# Include headers
set(
	INCLUDE_DIRECTORIES
	${PROJECT_SOURCE_DIR}/include
)
target_include_directories(Curves PRIVATE ${INCLUDE_DIRECTORIES})

# Link libraries
set(
	LINK_DIRECTORIES
	${PROJECT_SOURCE_DIR}/lib
)
target_link_directories(Curves PRIVATE ${LINK_DIRECTORIES})
target_link_libraries(Curves glew32 freeglut FreeImage)

# Ensure .dll is with .exe
file(COPY "${LINK_DIRECTORIES}/glew32.dll" DESTINATION "${CMAKE_BINARY_DIR}")
file(COPY "${LINK_DIRECTORIES}/freeglut.dll" DESTINATION "${CMAKE_BINARY_DIR}")
file(COPY "${LINK_DIRECTORIES}/FreeImage.dll" DESTINATION "${CMAKE_BINARY_DIR}")

# Move assets to .exe
add_custom_target(CopyShaders ALL
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	"${PROJECT_SOURCE_DIR}/shaders"
	"${CMAKE_BINARY_DIR}/shaders"
)
add_dependencies(Curves CopyShaders)