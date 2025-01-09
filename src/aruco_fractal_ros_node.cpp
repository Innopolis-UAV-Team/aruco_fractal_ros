//
// Created by op on 11.10.2019.
//

#include "aruco_fractal.h"



int main(int argc, char** argv)
{
    ArucoFractalRos arucoFractal(argc, argv);
    arucoFractal.rosRun();
    return 0;
}