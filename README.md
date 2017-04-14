# Description

This library provide detectors for fine grained action.


# Compile the code

## Copmpile the library

```
> mkdir build
> cd build
> cmake ..
> make
```

This will generate the libIMUAnalysis.a file.

## Compile the library and the executable

```
> mkdir build
> cd build
> cmake -DCOMPILE_EXECUTABLE=ON ..
> make
```

This will generate the detector executable.
To use the executable, use the command line

```
./detector generic path_to_video_file.mp4
```


# GPSCM commands

Checkout the code into the folder 'foo'

```
> mkdir foo
> cd foo
> gpscm init git@github.com:gopro/gopro-lib-imu-analysis.git@master
> gpscm sync
```


# List of action detected

## Generic

- Jumps
- Flips
- Spins
- Corners
- Shaky
- Pans

## Snowboarding

- Jumps

## Surfing

- Surfing

## Snowboarding

- Jumps

## Mountain-biking

- Jumps

