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


bool TestDetectShakiness() {

    // Check that we have 0 detection with an empty structure
    {
        imua::IMU imu;
        std::vector<imua::Detection> detections;
        imua::generic::detectShakiness(imu, detections);
        if (detections.size()!=0)
            return false;
    }

    // Check that we have 0 detection with a gyro of (x,y,z) values = 0
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
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // Detection
        std::vector<imua::Detection> detections;
        imua::generic::detectShakiness(imu, detections);
        if (!detections.empty())
            return false;
    }

    // Check that we have 0 detection with a gyro constant
    {
        // Create accelerometer data
        const int   size          = 1000;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 10.);
        std::vector<float> z(size, 20.);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create IMU structure
        imua::IMU imu;
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // Detection
        std::vector<imua::Detection> detections;
        imua::generic::detectShakiness(imu, detections);
        if (!detections.empty())
            return false;
    }

    // Check that we have 3 detections with a gyro of (x,y,z) values = 0 except for 1 pic
    {
        // Create accelerometer data
        const int   size          = 1800;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, 0.);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create the first detection
        for (int i=890; i<910; ++i) {
            z[i] = 100.f;
        }

        // Create IMU structure
        imua::IMU imu;
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // Detection
        std::vector<imua::Detection> detections;
        imua::generic::detectShakiness(imu, detections);
        if (detections.size()!=3)
            return false;
        if (detections[0].value!=1.f)
            return false;
        if (detections[1].value!=2.f)
            return false;
        if (detections[2].value!=1.f)
            return false;
    }

    return true;
}


bool TestDetectPans() {

    // Check that we have no detection with an empty structure
    {
        imua::IMU imu;
        std::vector<imua::Detection> detections;
        imua::generic::detectPans(imu, detections);
        if (detections.size()!=0)
            return false;
    }

    // Check that we have 0 detection with a gyro of (x,y,z) values = 0
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
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // Detection
        std::vector<imua::Detection> detections;
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;
    }

    // Check left pan
    {
        // Create accelerometer data for a left pan
        const int   size          = 1000;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, 0.2);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create IMU structure
        imua::IMU imu;
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // We should have no pan, it's too slow
        std::vector<imua::Detection> detections;
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;

        // We should have one pan that lasts all the video
        std::fill(z.begin(), z.end(), 0.8f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (detections.size()!=1)
            return false;
        if (detections[0].value!=1.)
            return false;
        if (detections[0].start!=0.)
            return false;
        if (detections[0].end!=(size-1)/sampling_rate)
            return false;

        // Reset and check that we have a right pan
        std::fill(z.begin(), z.end(), 2.f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;
    }

    // Check right pan
    {
        // Create accelerometer data for a right pan
        const int   size          = 1000;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, -0.2);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create IMU structure
        imua::IMU imu;
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // We should have no pans, it's too slow
        std::vector<imua::Detection> detections;
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;

        // We should have one pan that lasts all the video
        std::fill(z.begin(), z.end(), -0.8f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (detections.size()!=1)
            return false;
        if (detections[0].value!=2.)
            return false;
        if (detections[0].start!=0.)
            return false;
        if (detections[0].end!=(size-1)/sampling_rate)
            return false;

        // Reset and check that we have a right pan
        std::fill(z.begin(), z.end(), -2.f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;
    }

    // No pan detected if x and / or y rotations are detected
    {
        // Create accelerometer data for a left pan
        const int   size          = 1000;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, 0.8);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Create IMU structure
        imua::IMU imu;
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // We should have one pan
        std::vector<imua::Detection> detections;
        imua::generic::detectPans(imu, detections);
        if (detections.empty())
            return false;

        // We should have one pan with small x rotation
        std::fill(x.begin(), x.end(), 0.1f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (detections.empty())
            return false;

        // We should have no pan with bigger x rotation
        std::fill(x.begin(), x.end(), 0.3f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;

        // We should have one pan with small y rotation
        std::fill(x.begin(), x.end(), 0.0f);
        std::fill(y.begin(), y.end(), 0.1f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (detections.empty())
            return false;

        // We should have no pan with bigger y rotation
        std::fill(y.begin(), y.end(), 0.3f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;

        // We should have one pan with small x and y rotations
        std::fill(x.begin(), x.end(), 0.1f);
        std::fill(y.begin(), y.end(), 0.1f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (detections.empty())
            return false;

        // We should have one pan with small x and y rotations
        std::fill(x.begin(), x.end(), 0.3f);
        std::fill(y.begin(), y.end(), 0.3f);
        detections.clear();
        imua::generic::detectPans(imu, detections);
        if (!detections.empty())
            return false;
    }

    // Check with multiple pans
    {
        // Create accelerometer data for a left pan
        const int   size          = 4200;
        const float sampling_rate = 200.f;
        std::vector<float> x(size, 0.);
        std::vector<float> y(size, 0.);
        std::vector<float> z(size, 0.);
        std::vector<float> t(size, 0.);
        for (int i=0; i<size; ++i)
            t[i] = i / sampling_rate;

        // Add 5 pans of 3 seconds
        for (int i=0; i<600; ++i) {
            z[i +  200] = 0.8;  //  1s ->  4s
            z[i + 1000] = -0.8; //  5s ->  8s
            z[i + 1800] = 0.8;  //  9s -> 12s
            z[i + 2600] = -0.8; // 13s -> 16s
            z[i + 3400] = 0.8;  // 17s -> 20s
        }

        // Create IMU structure
        imua::IMU imu;
        imu.gyro.x            = &x[0];
        imu.gyro.y            = &y[0];
        imu.gyro.z            = &z[0];
        imu.gyro.t            = &t[0];
        imu.gyro.size         = size;
        imu.gyro.samplingRate = sampling_rate;

        // We should have 5 pans
        std::vector<imua::Detection> detections;
        imua::generic::detectPans(imu, detections);
        if (detections.size()!=5)
            return false;

        // Check detections 0
        if (detections[0].value!=1.)
            return false;
        if (detections[0].start!=200/sampling_rate)
            return false;
        if (detections[0].end!=799/sampling_rate)
            return false;

        // Check detections 1
        if (detections[1].value!=2.)
            return false;
        if (detections[1].start!=1000/sampling_rate)
            return false;
        if (detections[1].end!=1599/sampling_rate)
            return false;

        // Check detections 2
        if (detections[2].value!=1.)
            return false;
        if (detections[2].start!=1800/sampling_rate)
            return false;
        if (detections[2].end!=2399/sampling_rate)
            return false;

        // Check detections 3
        if (detections[3].value!=2.)
            return false;
        if (detections[3].start!=2600/sampling_rate)
            return false;
        if (detections[3].end!=3199/sampling_rate)
            return false;

        // Check detections 4
        if (detections[4].value!=1.)
            return false;
        if (detections[4].start!=3400/sampling_rate)
            return false;
        if (detections[4].end!=3999/sampling_rate)
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
    result &= Execute(TestDetectShakiness, "DetectShakiness");
    result &= Execute(TestDetectPans, "DetectPans");


    return (result) ? EXIT_SUCCESS : EXIT_FAILURE;
}