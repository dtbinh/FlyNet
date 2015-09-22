Note on hokuyo setup:

Testing on 2-3-15 used hokuyo with scan angle set to 180 degree sweep instead of full 240. 

See this website for how to change this parameter and other hokuyo parameters:
http://wiki.ros.org/hokuyo_node/Tutorials/UsingDynparamToChangeHokuyoLaserParameters

Most successful method was to set params at the beginning of the python script. Use the following steps.
tabbed items should go into python script.

	From link:
		Using dynamic_reconfigure from Python code
		You can change hokuyo_node parameters from Python scripts by importing:
			import dynamic_reconfigure.client

		creating a node:
			rospy.init_node('myconfig_py', anonymous=True)

		creating a client instance (node_to_reconfigure is the name of the node to reconfigure, for example 'hokuyo_node')
			client = dynamic_reconfigure.client.Client('my_node')

		and calling update_configuration with a dictionary of changes to make:
			params = { 'my_string_parameter' : 'value', 'my_int_parameter' : 5 }
			config = client.update_configuration(params)

		config now contains the full configuration of the node after the parameter update.

For our purposes, this code worked to set the min/max angle to -/+ 120:
	import dynamic_reconfigure.client
	client = dynamic_reconfigure.client.Client('/hokuyo')
	params = { 'min_ang' : -2.1 , 'max_ang' : 2.1 }
	config = client.update_configuration(params)

IMPORTANT NOTE!!!: 
Must have dynamic reconfigure available for use first!
$ rosdep install dynamic_reconfigure 
$ rosmake dynamic_reconfigure 


