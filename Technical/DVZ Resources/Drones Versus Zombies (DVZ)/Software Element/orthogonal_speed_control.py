
#Unit Vector from quad to desired point
V1_mag = sqrt((X_des - x)**2 + (Y_des - y)**2)
V1 = ((X_des - X) / V1_mag, (Y_des - Y) / V1_mag)

#Projection of quad's velocity vector onto V1
V2 = (V1[0]*(x_dot*V1[0] + y_dot*V1[1]), V1[1]*(x_dot*V1[0]+y_dot*V1[1]))

#Project of quad's velocity vector onto vector orthogonal to V1
V3 = (x_dot - V2[0], y_dot - V2[1])

#Transformation of V3 into the quad's body from
V4 = (V3*cos(yaw), V3*sin(yaw))

#Commands to be added to the other controller
pitch_out += kv*V4[0]
roll_out += -kv*V4[1]