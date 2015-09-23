if current_time - start_time > 15 and current_time - start_time < 60:
		target_x = -1.5
		target_y = -1.5

		write_file = 1
		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)
		#velocity_controller(master,quad,target_velx,target_vely,x,y,inertial_yaw)

		if print_controllers == 0:
			print "Controllers running (-1.5,-1.5)..."
			print_controllers = 1

	elif current_time - start_time > 60 and current_time - start_time < 105:
		target_x = -1.5
		target_y = 1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 1:
			print "Controllers running (-1.5,1.5)..."
			print_controllers = 0

	elif current_time - start_time > 105 and current_time - start_time < 150:
		target_x = 1.5
		target_y = 1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 0:
			print "Controllers running (1.5,1.5)..."
			# Clear position controllers integral term
			quad.I_error_posx = 0
			quad.I_error_posy = 0
			print_controllers = 1

	elif current_time - start_time > 150 and current_time - start_time < 195:
		target_x = 1.5
		target_y = -1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 1:
			print "Controllers running (1.5,-1.5)..."
			# Clear position controllers integral term
			quad.I_error_posx = 0
			quad.I_error_posy = 0
			print_controllers = 0

	elif current_time - start_time > 195:
		target_x = -1.5
		target_y = -1.5

		position_controller(master,quad,target_x,target_y,x,y)
		velocity_controller(master,quad,quad.velx_inertial_send,quad.vely_inertial_send,x,y,inertial_yaw)

		if print_controllers == 0:
			print "Controllers running (-1.5,-1.5)..."
			# Clear position controllers integral term
			quad.I_error_posx = 0
			quad.I_error_posy = 0
			print_controllers = 1
