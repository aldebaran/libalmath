find_package(qimodule)

qi_create_module(geometry_module
  SRC
  "src/registration.cpp"
  DEPENDS
  qigeometry almath
)


qi_create_gtest(test_geometry_module
  test/test_geometry_module.cpp
  DEPENDS qi qigeometry)

if (test_geometry_module)
  add_dependency(test_geometry_module geometry_module)
endif()
