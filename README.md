# Python based Robot Interactive Development Environment (PyRIDE) for Pepper Robot

## Introduction
This repository contains PyRIDE for Aldebaran/Softbank Pepper robots. For more detailed introduction on PyRIDE, see [README](https://github.com/uts-magic-lab/pyride_pr2/blob/master/README.md) of PyRIDE for ROS/PR2. A small demonstration of PyRIDE on Pepper is shown below in which a Pepper retrieves twitter messages and announce them by a touch to the head.

[![First demo video for PyRIDE on Pepper robot](http://img.youtube.com/vi/FP9fUvsooLs/0.jpg)](http://www.youtube.com/watch?v=FP9fUvsooLs)

**WARNING:** This version of PyRIDE is compatible with NaoQi SDK version 2.4.2 and below. It may require code updates in order to be compatible with later version of NaoQi SDK.

## Compile source code
### Prerequisites
You need NaoQi SDK 2.4.2 with the corresponding cross compiler toolchain installed on your system. You will also need qibuild 3.x build system installed.

### Source code structure
PyRIDE for Pepper is dependent on several open source third-party libraries. Since there is no prebuild library binaries, this repository contains modified source code of these libraries so that you can build and install them manually on your Pepper robots. In addition, PyRIDE on Pepper has been partitioned into two components: PyRideCore and PyPepperServer. PyRideCore contains core functionalities that are available to all supported robot platforms. PyPepperServer contains code that is specific to the Pepper robot platform. Third-party libraries and PyRideCore are under ```libsrc``` directory. PyPepperServer is located under ```PyPepperServer``` directory. ```scripts``` contains default example Python scripts that run on PyRIDE for Pepper.

### Compile procedures
Assume you have placed PyRIDE for Pepper source code under a working qibuild worktree and you have a working cross compiler toolchain ```cross-atom```, use the following command to build libraries using qibuild build system.

```
qibuild package -c cross-atom --release
```

To install the compiled library package, e.g. commoncpp2, on local machine:

```
qitoolchain add-package -c cross-atom commoncpp2 ../../../package/commoncpp2-cross-atom.zip
```

You will need to compile code in the following sequence:
* crypto
* commoncpp2-1.8.1
* celt-0.11.1
* ccrtp-1.7.2
* PyRideCore
* PyPepperServer

### Manual binary installation
Under a Pepper terminal (using ssh), copy the following shared library objects extracted from the packages (or copy directly from the ```build-cross-atom-release/sdk/lib``` subdirectory under the source directories) built in the previous section to ```/home/nao/naoqi/lib``` directory on the robot.

* libcrypto.so
* libccext2.so
* libccrtp.so
* libccgnu2.so
* libcelt.so
* libpyridecore.so
* libpypepperserver.so

Similarly, copy the entire ```scripts``` under the repository to ```/home/nao/naoqi/lib``` and rename the directory to ```python```.

Create an ```autoload.ini``` file under ```/home/nao/naoqi/preference``` with the following content:

```
# autoload.ini
#
# Use this file to list the cross-compiled modules that you wish to load.
# You must specify the full path to the module, python module or program.

[user]
#the/full/path/to/your/liblibraryname.so  # load liblibraryname.so
/home/nao/naoqi/lib/libpypepperserver.so

[python]
#the/full/path/to/your/python_module.py   # load python_module.py

[program]
#the/full/path/to/your/program            # load program
```

```autoload.ini``` will automatically load PyPepperServer when NaoQi starts.

**NOTE:** PyRIDE configuration file ```pyrideconfig.xml``` will be automatically generated under ```/home/nao/naoqi/preference``` when PyRIDE for Pepper is successfully run and *properly* shutdown. As older NaoQi systems do not properly shutdown PyRIDE module when shutting down the robot by pressing the centre button, you may have to call ```PyPepper.saveConfiguration``` command to save important configuration, e.g. remote user access account, periodically.

### Using PyRIDE for Pepper
Check PyRIDE for ROS/PR2 [README](https://github.com/uts-magic-lab/pyride_pr2/blob/master/README.md) for the details on how to access embedded Python engine, remote client access. Check [PyRIDE API documentation for Pepper](http://uts-magic-lab.github.io/pyride_nao) for the details on the available Python methods for Pepper.
