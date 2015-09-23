def vel_controller(ref_velx,ref_vely,ref_velz,ref_yaw,velx,vely,velz,yaw):
# Calculate error in velocities/yaw pose
    err_x = ref_velx - velx
    err_y = ref_vely - vely
    err_z = ref_velz - velz
    err_yaw = ref_yaw - yaw

    delta_t = time.time() - quad.previous_time()

    
