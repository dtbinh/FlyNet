This simulink model contains a position controller, attitude controller, motor mixer, and motor/quad dynamics.

Compatibility:
- Windows 7
- Matlab R2015b

Contents:
- RunFirst.m: Run this script before running the model. It loads the initial conditions, quad parameters, and trajectory.
- MainModel.slx: Simulink model of the quadcopter. Run simulation after the RunFirst.m scripts has been run.
- plotpath.m: Simple script to plot target trajectory (blue) and simulated path (red). Run this script after the MainModel.slx simulation has been run.
- quadcopterDynamicSFunction.m: Contains the dynamics functions for the quadcopter based on motor RPM.
- SupportData: Collection of .mat files loaded by the RunFirst.m script. 'IC' contains the quadcopter initial conditions. 'Path_Triangle' contains sample waypoints for the quadcopter to follow (stored as timeseries object). 'quadModel_X' is the default quadcopter physical configuration where X and + represent the desired body axis orientation on the quad (still needs Aliencopter Bee parameters).

Known issues:

(11 Nov 2015)
- Update physical parameters for Aliencopter Bee.
- Tune gains once parameters are updated.
- Currently does not work with Linux compilers.
- Need to compare to VICON data to improve fidelity.
- Add drag moment, body gyro effects, ground effects, sensor dynamics.