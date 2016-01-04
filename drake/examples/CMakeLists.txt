
add_subdirectory(Atlas)
add_subdirectory(Valkyrie)
add_subdirectory(Acrobot/@AcrobotPlantCpp)
add_subdirectory(@VanDerPolCpp)

find_package(Boost)
if(Boost_FOUND) 
  include_directories(Boost_INCLUDE_DIRS)
  add_subdirectory(Pendulum)
  add_subdirectory(Quadrotor)
endif()
