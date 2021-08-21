
add_library(model_lib STATIC
quad_2d/quad_2d.cpp
)

target_include_directories(model_lib PUBLIC
quad_2d
)