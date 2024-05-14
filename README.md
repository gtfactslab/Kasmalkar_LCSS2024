# Feedback Linearization of an Underacuated Miniature Blimp with Zero Dynamics Mitigation using High Order Control Barrier Functions

This code accompanies our submission to RA-L.

# Prerequisites
To replicate the results of our paper requires the [GT-MAB hardware](https://github.com/thedancomplex/open-blimp/) along with an appropriate motion capture system.

This project is a ros2 package. Note that the top-level folder of this repository must be renamed to `blimp_mpc_ros` when installed.

# To install
Clone this repository into the `src` directory of your ROS workspace.

Dependencies to install:
* Gurobi optimizer (gurobipy)
* Numpy and matplotlib
* Python Control Systems Library
* OptiTrack motion capture ROS packages

# To run
`cd` into the `blimp_mpc_ros` directory (where all of the Python files are), i.e. you should be in `~/ros2_ws/src/blimp_mpc_ros/blimp_mpc_ros` if your ros2 workspace is `/ros2_ws`.

Make sure to run these commands in this directory; specifically, the `logs` folder should be in the same directory as where you run this command from. Otherwise the script will complain that it cannot find a `logs` directory to write the logfile into.

## To run code that commands the blimp
```
ros2 run blimp_mpc_ros <command> <logfile>
```
where <command> has the following format, according to which algorithm and trajectory you want to run:
```run_<algo>_<traj>```

`<algo>` can be `cbf`, `fbl`, or `lqr`
* `cbf` is used to run the feedback linearization controller with CBFs enabled
* `fbl` is used to run the feedback linearization controller without CBFs enabled
* `lqr` is used to run LQR

`<traj>` can be `line`, `helix`, or `triangle` depending on which trajectory is desired. Note that the trajectory referred to as "sawtooth" in the paper is called "triangle" in the codebase.

`logfile` should be a csv file (include the `.csv` extension when you run the command).

## To run a simulation
```
ros2 run blimp_mpc_ros run_blimp_sim <sim> <logfile>
```
where `<sim>` has the form
```<algo>_<traj>```
where `<algo>` and `<traj>` are specified as above.

## To view logged data in a plot
```
ros2 run blimp_mpc_ros run_blimp_data <logfile>
```
*Note:* The `Error` columns of the data (`x_error`, `y_error`, etc.) are incorrect due to a bug in the code. However, the error can always be computed by taking the difference between the state data, which is accurately recorded, and the reference trajectory (`x_ref`, `y_ref`, etc.).

## To generate the plots from the paper
Run `data_analysis.m` in MATLAB.

## To generate the animations of the blimp trajectories in the YouTube video
Run `video_gen.m` in MATLAB.

## Example
If I wanted to run the blimp under the feedback linearization + CBFs control strategy in the helix trajectory and log to `test.csv`, I would run
```
ros2 run blimp_mpc_ros run_cbf_helix test.csv
```

If I wanted to simulate this trajectory, I would run
```
ros2 run blimp_mpc_ros run_blimp_sim cbf_helix test.csv
```

If I wanted to view the logged data from `test.csv`, I would run
```
ros2 run blimp_mpc_ros run_blimp_data test.csv
```

# To modify
Each controller/trajectory combination has a dedicated class. For instance, `CtrlCBFHelix` is the controller that drives the blimp using the CBF-enabled feedback linearization tracking controller along the helical trajectory.

The class structure is as follows:
* `BlimpController`: The superclass of all of the other controller objects
* `CtrlCBF`: The superclass of all of the CBF-enabled feedback linearization controller objects
* `CtrlFBL`: The superclass of all of the CBF-disabled feedback linearization controller objects
* `CtrlLQR`: The superclass of all of the LQR controller objects

In the subclasses of each of these classes (`CtrlCBFHelix`, `CtrlLQRLine`, etc.), the desired trajectory is retrieved from the `Trajectories` object, which provides static methods -- `get_helix`, `get_line`, and `get_triangle` -- for generating trajectories.

To modify the parameters of a controller (such as feedback gains, CBF limits, etc.), visit the superclass for the controller you want (e.g. `CtrlCBF` for the CBF-enabled controllers) and make the changes there. To modify a trajectory, visit the `Trajectories` class to make the modifications.

The parameters of the blimp (mass, inertia, damping, etc.) can be found in `utilities.py`, along with a number of other utility functions.

`BlimpMPCNode` is the ROS 2 node that receives mocap data and publishes blimp commands.
`BlimpLogger` is responsible for logging data to logfiles after the run is complete.
`BlimpPlotter` is used to plot data from logfiles.
`BlimpSim` is used to run a simulation of the blimp dynamics.

## Notes
The following folders from the `blimp_mpc_ros/logs` directory contain the relevant test data. `run_blimp_data` may be used as shown above to visualize the data from these tests.
* `final_testing_4_25` contains the trajectory data from the feedback linearization controller with CBFs enabled
* `final_testing_4_22` contains the trajectory data from the LQR controllers
* `fbl_testing_5_2` contains the trajectory data from the feedback linearization controller without CBFs enabled
* `cbf_vs_fbl_data_4_27` contains the trajectory data from the aggressive trajectories, for which the feedback linearization controller was used, both with CBFs enabled and with CBFs disabled
* `videos_5_1` contains the trajectory data from the YouTube videos

The derivation of the math in the paper was performed using the MATLAB Symbolic Toolbox in the following files:
* `blimp_dynamics_organized.mlx` contains closed-form expressions for the full nonlinear dynamics of the blimp
* `cbf_fbl_paper_math.mlx` contains the step-by-step math from the paper, including the derivation of feedback linearization, zero dynamics, and the CBFs. Note that the matrix `G` was named `L` in the paper.
