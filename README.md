# DECAR Mocap Tools #
[TOC]
## Setup 
### Option 1 - Simple Cloning
Simply clone this repository and initialize the submodules, which for now is only `decar_utils`:

    git submodule init
    git submodule update

These commands should prompt you for your Bitbucket username and password. 

### Option 2 - Adding as a submodule to your project repo
Instead of cloning, you may add this repo as its own submodule inside of your own project repository. To do this,

    git submodule add https://bitbucket.org/decargroup/decar_mocap_tools.git

and you will also be prompted for your Bitbucket username and password. Then, make sure to initialize the submodules using the `--recursive` option so that the submodules inside `decar_mocap_tools` also get initialized.

    git submodule update --init --recursive

## How this repository is organized
### Folder structure
At the highest-level folder, being the `decar_mocap_tools` folder, all the "user-facing" functions appear. The following subfolders also appear:

- `decar_animate/`
    - This is a git submodule. Contains the necessary dependencies for the `mocapPlay()` function.
- `decar_utils/`
    - This is a git submodule. Contains many helper functions used by various functions in this repo.
- `src/`
    - Core "back-end" functions that are called by the user-facing functions.
- `tests/`
    - Various tests and corresponding datasets for those tests.
- `utils/`
    - Auxiliary functions usually used by the user-facing functions. These are typically smaller and more general than the functions found in `src/`
### Function name nomenclature
The functions are typically prefixed with the physical device that it is relevant to. For example the functions

    mocapCsvToStruct()
    mocapFitSpline()

mainly concern motion capture system (mocap) data. Similarly,

    imuCsvToStruct()
    imuCalibrate()

mainly concern IMU data. 
## TODO

1. ~~add bias to LS~~
2. ~~add time difference to the LS~~
3. find a way to deal with the bugs with finding the header range
4. ~~ignore points where no Mocap data was collected when doing LS~~
5. IMU/pivot point offset
6. Move alignFrames to inside imuCalibrate
7. script to extract sensor characteristics (biases, covariances, etc)
8. ~~Add a static detector to the mocap data~~
9. Add a static detector to the IMU data
10. add comments and documentation to all functions
11. Calibrate full IMU pose in imuCalibrate 
12. Split up imuCalibrate into accelCalibrate and gyroCalibrate, and then have imuCalibrate call them with magCalibrate
13. Automate the initial guess of magCalibrate
14. Create "examples" folder showing minimal working example of imu, gyro, mag calibration
15. Allow imuMocapSync(filename1, filename2)
