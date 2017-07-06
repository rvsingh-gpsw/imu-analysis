#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cmath>

#include <tools.h>
#include <types.h>
#include <generic.h>

#pragma mark Tools

/**
 * Helper function.
 * @param func  pointer to the test function
 * @param name  test name
 * return true if the test passed, false otherwise
 */
bool Execute(bool (*func)(), std::string name) {
    std::cout << std::setw(30) << std::setfill('.') << std::left << name;
    const bool result = func();
    std::cout << ((result) ? "SUCCESS" : "FAILURE") << std::endl;
    return result;
}


/**
 * Check if 2 values are approcimately equal
 * @param a        first value
 * @param b        second value
 * @param epsilon  precision
 * return true if a and b a approximately equal, false otherwise
 */
template<typename T>
bool Approx(const T a, const T b, const T epsilon) {
    return b>=a-epsilon && b<=a+epsilon;
}


#pragma mark Tests


bool TestComputeNorm() {
  
    // Check that we have expected values with an array of 0
    {
        const int size = 1000;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, 0.);
        std::vector<float> norm(size);

        if (!imua::ComputeNorm(&x[0], &y[0], &z[0], size, &norm[0]))
            return false;

        for (int i=0; i<size; ++i)
            if (norm[i]!=0.)
                return false;
    }
  
    // Check that we have expected values with an array of 10
    {
        const int size = 1000;
        std::vector<float> x(size, 10.);
        std::vector<float> y(size, 10.);
        std::vector<float> z(size, 10.);
        std::vector<float> norm(size);

        if (!imua::ComputeNorm(&x[0], &y[0], &z[0], size, &norm[0]))
            return false;

        for (int i=0; i<size; ++i)
            if (!Approx(norm[i], std::sqrt(300.f), 0.00001f))
                return false;
    }

    return true;
}


bool TestSmoothArrayStream() {


    // Check that nothing happens if we smooth an empty array
    if (imua::SmoothArrayStream(NULL, 0, NULL))
        return false;

    // Check that nothing happens if one of the pointers is NULL
    {
        const int size = 1000;
        std::vector<float> array(size);
        if (imua::SmoothArrayStream(&array[0], size, NULL))
            return false;
        if (imua::SmoothArrayStream(NULL, size, &array[0]))
            return false;
        if (imua::SmoothArrayStream(NULL, size, NULL))
            return false;
    }

    // Check that the function behaves well with different weight values
    {
        const int size = 1000;
        std::vector<float> input(size, 0.);
        std::vector<float> output(size);

        if (!imua::SmoothArrayStream(&input[0], size, &output[0]))
            return false;
        if (imua::SmoothArrayStream(&input[0], size, &output[0], -10.f))
            return false;
        if (!imua::SmoothArrayStream(&input[0], size, &output[0], 0.f))
            return false;
        if (!imua::SmoothArrayStream(&input[0], size, &output[0], 0.5f))
            return false;
        if (!imua::SmoothArrayStream(&input[0], size, &output[0], 1.f))
            return false;
        if (imua::SmoothArrayStream(&input[0], size, &output[0], 2.f))
            return false;
    }

    // Check that we have expected values with an array of 0
    {
        const int size = 1000;
        std::vector<float> input(size, 0.);
        std::vector<float> output(size);

        if (!imua::SmoothArrayStream(&input[0], size, &output[0]))
            return false;

        for (int i=0; i<size; ++i)
          if (output[i]!=0.)
            return false;
    }

    // Check that we have expected values with an array of 10
    {
        const int size = 1000;
        std::vector<float> input(size, 10.);
        std::vector<float> output(size);

        if (!imua::SmoothArrayStream(&input[0], size, &output[0]))
            return false;

        for (int i=0; i<size; ++i)
          if (!Approx(output[i], 10.f, 0.00001f))
            return false;
    }

    return true;
}


bool TestSmoothArrayBoxFilter() {

    // Check that nothing happens if we smooth an empty array
    if (imua::SmoothArrayBoxFilter(NULL, 0, NULL))
        return false;

    // Check that nothing happens if one of the pointers is NULL
    {
        const int size = 1000;
        std::vector<float> array(size);
        if (imua::SmoothArrayBoxFilter(&array[0], size, NULL))
            return false;
        if (imua::SmoothArrayBoxFilter(NULL, size, &array[0]))
            return false;
        if (imua::SmoothArrayBoxFilter(NULL, size, NULL))
            return false;
    }

    // Check that the function behaves well with different weight values
    {
        const int size = 1000;
        std::vector<float> input(size, 0.);
        std::vector<float> output(size);

        if (!imua::SmoothArrayBoxFilter(&input[0], size, &output[0]))
            return false;
        if (imua::SmoothArrayBoxFilter(&input[0], size, &output[0], -10.f))
            return false;
        if (imua::SmoothArrayBoxFilter(&input[0], size, &output[0], 0.f))
            return false;
        if (!imua::SmoothArrayBoxFilter(&input[0], size, &output[0], 0.5f))
            return false;
        if (!imua::SmoothArrayBoxFilter(&input[0], size, &output[0], 1.f))
            return false;
        if (!imua::SmoothArrayBoxFilter(&input[0], size, &output[0], 2.f))
            return false;
    }

    // Check that we have expected values with an array of 0
    {
        const int size = 1000;
        std::vector<float> input(size, 0.);
        std::vector<float> output(size);

        if (!imua::SmoothArrayBoxFilter(&input[0], size, &output[0]))
            return false;

        for (int i=0; i<size; ++i)
          if (output[i]!=0.)
            return false;
    }

    // Check that we have expected values with an array of 10
    {
        const int size = 1000;
        std::vector<float> input(size, 10.);
        std::vector<float> output(size);

        if (!imua::SmoothArrayBoxFilter(&input[0], size, &output[0]))
            return false;

        for (int i=0; i<size; ++i)
          if (!Approx(output[i], 10.f, 0.00001f))
            return false;
    }

    return true;
}


bool TestDetectJumps() {

    // Check that no jump is detected with an emptu structure
    {
        imua::IMU imu;
        std::vector<imua::Detection> detections;
        imua::generic::detectJumps(imu, detections);
        if (detections.size()!=0)
            return false;
    }

    // Check that we have 1 detection with a gyro of (x,y,z) values = 0
    {
        // Create accelerometer data
        const int   size          = 1000;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, 0.);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create IMU structure
        imua::IMU imu;
        imu.accl.x            = &x[0];
        imu.accl.y            = &y[0];
        imu.accl.z            = &z[0];
        imu.accl.t            = &t[0];
        imu.accl.size         = size;
        imu.accl.samplingRate = sampling_rate;

        // Detect jumps
        std::vector<imua::Detection> detections;
        imua::generic::detectJumps(imu, detections);
        if (detections.size()!=1)
            return false;
        if (!Approx(detections[0].start, 0.f, 0.00001f))
            return false;
        if (!Approx(detections[0].end, (size-1)/sampling_rate, 0.00001f))
            return false;
    }

    // Check that we have 2 detections with sythetic data
    {
        // Create accelerometer data
        const int   size          = 1000;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.f);
        std::vector<float> y(size, 0.f);
        std::vector<float> z(size, 9.81f);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create the first detection
        for (int i=200; i<400; ++i)
            z[i] = 0.f;

        // Create the second detection
        for (int i=600; i<800; ++i)
            z[i] = 0.f;

        // Create IMU structure
        imua::IMU imu;
        imu.accl.x            = &x[0];
        imu.accl.y            = &y[0];
        imu.accl.z            = &z[0];
        imu.accl.t            = &t[0];
        imu.accl.size         = size;
        imu.accl.samplingRate = sampling_rate;

        // Detect jumps
        std::vector<imua::Detection> detections;
        imua::generic::detectJumps(imu, detections);
        if (detections.size()!=2)
            return false;
    }

    // Check that we have no detections with sythetic data (too short jumps)
    {
        // Create accelerometer data
        const int   size          = 1000;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.f);
        std::vector<float> y(size, 0.f);
        std::vector<float> z(size, 9.81f);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create the first detection
        for (int i=100; i<110; ++i)
            z[i] = 0.f;

        // Create the second detection
        for (int i=200; i<220; ++i)
            z[i] = 0.f;

        // Create IMU structure
        imua::IMU imu;
        imu.accl.x            = &x[0];
        imu.accl.y            = &y[0];
        imu.accl.z            = &z[0];
        imu.accl.t            = &t[0];
        imu.accl.size         = size;
        imu.accl.samplingRate = sampling_rate;

        // Detect jumps
        std::vector<imua::Detection> detections;
        imua::generic::detectJumps(imu, detections);
        if (detections.size()!=0)
            return false;
    }

    return true;
}



int main(int argc, char *argv[]) {

    bool result = true;

    result &= Execute(TestComputeNorm, "ComputeNorm");
    result &= Execute(TestSmoothArrayStream, "SmoothArrayStream");
    result &= Execute(TestSmoothArrayBoxFilter, "SmoothArrayBoxFilter");
    result &= Execute(TestDetectJumps, "DetectJumps");


    return (result) ? EXIT_SUCCESS : EXIT_FAILURE;
}