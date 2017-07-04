#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cmath>

#include <tools.h>

/**
* Helper function.
* @param func  pointer to the test function
* @param name  test name
* return true if the test passed, false otherwise
*/
bool Execute(bool (*func)(), std::string name) {
    std::cout << std::setw(20) << std::left << name << " ";
    const bool result = func();
    std::cout << ((result) ? "SUCCESS" : "FAILURE") << std::endl;
    return result;
}


bool TestComputeNorm() {
  
    {
        // Allocate the input and output arrays. Fill the x, y and z with 0.
        const int size = 1000;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, 0.);
        std::vector<float> norm(size);
        imua::ComputeNorm(&x[0], &y[0], &z[0], size, &norm[0]);

        // Verify that the output array only has 0.
        for (int i=0; i<size; ++i)
          if (norm[i]!=0.)
            return false;
    }
  
    {
        // Allocate the input and output arrays. Fill the x, y and z with 10.
        const int size = 1000;
        std::vector<float> x(size, 10.);
        std::vector<float> y(size, 10.);
        std::vector<float> z(size, 10.);
        std::vector<float> norm(size);
        imua::ComputeNorm(&x[0], &y[0], &z[0], size, &norm[0]);

        // Verify that the output array only has 0.
        for (int i=0; i<size; ++i)
          if (norm[i]<std::sqrt(300.)-0.00001f || norm[i]>std::sqrt(300.)+0.00001f) {
            printf("%f\n", norm[i]);
            return false;
        }
    }

    return true;
}


bool TestSmoothArrayBox() {

    {
        // Allocate the input and output arrays. Fill the input one with 0.
        const int size = 1000;
        std::vector<float> input(size, 0.);
        std::vector<float> output(size);
        imua::SmoothArrayBoxFilter(&input[0], size, &output[0]);

        // Verify that the output array only has 0.
        for (int i=0; i<size; ++i)
          if (output[i]!=0.)
            return false;
    }

    {
        // Allocate the input and output array. Fill the input one with 0.
        const int size = 1000;
        std::vector<float> input(size, 10.);
        std::vector<float> output(size);
        imua::SmoothArrayBoxFilter(&input[0], size, &output[0]);

        // Verify that the output array only has 0.
        for (int i=0; i<size; ++i)
          if (output[i]<9.99999f || output[i]>10.00001f)
            return false;
    }

    return true;
}


int main(int argc, char *argv[]) {

    bool result = true;

    result &= Execute(TestComputeNorm, "ComputeNorm");
    result &= Execute(TestSmoothArrayBox, "SmoothArrayBox");

    return (result) ? EXIT_SUCCESS : EXIT_FAILURE;
}