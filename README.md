# DECAR Mocap Tools #

## Setup - Simple Cloning
Simply clone this repository and initialize the submodules, which for now is only `decar_utils`:

    git submodule init
    git submodule update

These commands should prompt you for your Bitbucket username and password. 

## Setup - Adding as a submodule to your project repo
Instead of cloning, you may add this repo as its own submodule inside of your own project repository. To do this,

    git submodule add https://bitbucket.org/decargroup/decar_mocap_tools.git

and you will also be prompted for your Bitbucket username and password. Then, make sure to initialize the submodules using the `--recursive` option so that the submodules inside `decar_mocap_tool` also get initialized.

    git submodule update --init --recursive

## TODO

1. ~~add bias to LS~~
2. add time difference to the LS
3. find a way to deal with the bugs with finding the header range
4. ~~ignore points where no Mocap data was collected when doing LS~~
5. IMU/pivot point offset
6. script to find m_a
7. script to extract sensor characteristics (biases, covariances, etc)
8. clean up/comment/standardize code