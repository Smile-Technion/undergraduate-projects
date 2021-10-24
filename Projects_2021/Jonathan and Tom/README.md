# 034381 Research Project in Mechanical Eng.

Implementation and examination of the Impedance control for 6R robot arms.

## Description

In this project, we explored the implementation of the Impedance control for various environment. Our main work is in
`myROBOSUITE`
folder in whch we created a custom environment to put objects, run simulation and episodes. It can be serve as a benchmark for other testing. In
`ImpedanceControlMatlab`
 folder, we implemented both
 `position-based-impedance-control`
 and
 `dynamics-based-impedance-control`
 on a 2R robot arm.  Finally, all the reference paper and the spec of the robot is in
 `References`
 folder. Other unmentioned directories are trivial.
## Getting Started

### Dependencies

* For running python codes, you can download all the packages in the `requirements.txt`. It's recommended to intall them in a separate virtual environment. All the python code runs in linux system. You might encounter some problems if you have a WINDOWS computer.
* For running MATLAB code, you need a MATLAB version of 2018 or higher.

### Installing

* Download mujoco. Go to http://www.mujoco.org/ and download `MuJoCo 2.0`. You can get a free version for a month or ask professor for the activation key.
* Clone `mujoco-py` and `ARISE-Initiative/robosuite` to another folder. (Set up their PATH to your code editor. VScode is the best by test.)

### Executing program

* To run a single simulation, run the files below.
```
main_osc.py
```

* To add custom object,
    * update my_object.py
    * update `assets` directory
  
* To run a series of test with different parameters, a basic function is written in `testing_module.py`. Check out `test_kp_kd` or `test_perception_error.py` for example.

## Authors

[Jonathan Oh](https://github.com/garlicbutter)

[Tom Yu]()


## License

This project is licensed under the [MIT License] - see the LICENSE.md file for details.