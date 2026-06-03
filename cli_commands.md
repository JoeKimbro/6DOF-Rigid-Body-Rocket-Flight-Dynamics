  g++ -std=c++17 main.cpp 1DOF/1DOF.cpp 1DOF/Integrator1DOF.cpp -I. -o sim
  g++ -std=c++17 main.cpp 1DOF/1DOF.cpp 1DOF/Integrator1DOF.cpp 2DOF/2DOF.cpp 2DOF/Integrator2DOF.cpp -I. -o sim
  COMPILE WITH C++17 (CHECKS FOR ERRORS BEFORE RUN)

  ./sim
  TO RUN THE MAIN