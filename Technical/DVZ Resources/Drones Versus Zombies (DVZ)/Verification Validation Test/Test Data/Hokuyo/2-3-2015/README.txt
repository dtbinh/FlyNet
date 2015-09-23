README

Hokuyo Testing Febuary 3, 2015.

PROCESS:
	Hokuyo was placed on cart and aimed at walls inside vicon space (vicon off).
	Ran hokuyo_data.py (DVZ>Hardware Element > NTC_WIP)
	Process Data with matlab -> Hokuyo_PostProcess.m

Test1: 
	Hokuyo placed close to wall (the wall of the building)
Test2:
	Hokuyo moved farther from building wall so that the min angle of incidence was increased.
	
ISSUES:
	Hokuyo only scans over 180 degrees.
	Hokuyo returns Inf or NaN for range when scan ray has low angle of incidence with wall.


EDIT (4/21/15)
	Hokuyo can scan over any angular range up to 240 degrees. This is a settable parameter.
