cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)
# set(CMKAE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp")
set ( PRJ_LIBRARIES )

find_package ( OpenMP REQUIRED )

list ( APPEND PRJ_LIBRARIES OpenMP::OpenMP_CXX )


add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

target_link_libraries( ${PROJECT_NAME} 
        PRIVATE 
                ${PRJ_LIBRARIES}
    )
