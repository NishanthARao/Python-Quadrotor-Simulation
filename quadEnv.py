#============================================================================================================================================
#Documentation
#--------------
#
#1.1) Set USE_PWM = 1 if you are giving inputs to the step() function as PWM pulses for each BLDC motors (1000us - 2000us)
#1.2) Set USE_PID = 1 if you are using any of the PID controllers for benchmarking purposes. If you want to switch over to any other algorithm in midst of the program execution from PID controller, set USE_PID = 0. If, again you want to use PID controller after your algorithm in midst of the program execution, make sure to call the rstEnv() function before you set the USE_PID flag to 1.
#
#2) The des_xyz(x_des, y_des, z_des) function is used to provide the desired positions, to which the drone must hover to and stabilize. You must call PID_position() function if you are using this.
# 
#3) The step(current_state, input) will calculate the next state of the system, based on the current_state and the input provided, for one time step.
#4) PID_position() calculates the desired roll, pitch and thrust values that are to be followed if the drone has to hover at the desired position.
#5) PID_attitude() calculates the desired rates that the drone must follow, in order to tilt to the commanded attitude
#6) The PID_rate() calculates the desired torques along the x, y, z direction that must be applied by the BLDC motors
#7) The quad_motor_speed() calculates the desired motor speeds based on the thrust, and torque values.
#8) The rstEnv() resets the simulation
#9) The pauseEnv() pauses the simulation
#10) The unpauseEnv() unpauses the simulation
#11) The time_elapsed() displays the simulation time

#============================================================================================================================================
import time
import numpy as np
from math import *

class quadrotor:
    def __init__(self, Ts = 1.0/50.0, USE_PWM=0, USE_PID=0):
        self.Ts = Ts
        self.g = 9.81
        self.m = 1.4
        self.l = 0.56
        self.kd = 0.0000013858
        self.kdx = 0.16481
        self.kdy = 0.31892
        self.kdz = 0.0000011
        self.jx = 0.05
        self.jy = 0.05
        self.jz = 0.24
        self.kt = 0.000013328
        self.jp = 0.044
        self.max_motor_speed_2 = 855625.0
        self.min_motor_speed_2 = 0.0
        self.u1_max = 43.5
        self.u1_min = 0.0
        self.u2_max = 6.25
        self.u2_min = -6.25
        self.u3_max = 6.25
        self.u3_min = -6.25
        self.u4_max = 2.25
        self.u4_min = -2.25
        
        self.x_kp = 0.35
        self.x_ki = 0.25
        self.x_kd = -0.35
        self.x_ki_lim = 0.25
        
        self.z_kp = 5.88
        self.z_ki = 0.0
        self.z_kd = -5.05
        
        self.phi_kp = 4.5
        self.phi_ki = 0.0
        self.phi_kd = 0.0
        self.phi_max = 0.7853
        self.phi_ki_lim = 0.0349
        
        self.psi_kp = 4.5
        self.psi_ki = 0.0
        self.psi_kd = 0.0
        self.psi_max = 10.0
        self.psi_ki_lim = 0.1396
        
        self.p_kp = 2.7
        self.p_ki = 1.0
        self.p_kd = -0.01
        self.p_max = 0.87266
        self.p_ki_lim = 0.174532
        
        self.R2D = 57.295779513
        self.D2R = 0.017453293
        
        self.x_des = 1.0
        self.y_des = 1.0
        self.z_des = 1.0
        
        self.phi_des = 0.0
        self.theta_des = 0.0
        self.psi_des = 0.0
        
        self.p_des = 0.0
        self.q_des = 0.0
        self.r_des = 0.0
        
        self.x_error_sum = 0.0
        self.y_error_sum = 0.0
        self.z_error_sum = 0.0
        
        self.phi_error_sum = 0.0
        self.theta_error_sum = 0.0
        self.psi_error_sum = 0.0
        
        self.p_error_sum = 0.0
        self.q_error_sum = 0.0
        self.r_error_sum = 0.0
        
        self.p_dot = 0.0
        self.q_dot = 0.0
        self.r_dot = 0.0
        
        self.state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.o = 0.0
        
        self.rstFlag = 0
        self.pauseFlag = 0
        
        self.time_elapse = 0.0
        
        self.USE_PID = USE_PID
        self.USE_PWM = USE_PWM
        if(self.USE_PWM == 1 and self.USE_PID == 0):
            self.input_vector = [1000, 1000, 1000, 1000]
            print(self.input_vector)
        else:
            self.input_vector = [0, 0, 0, 0]
    
    def des_xyz(self, x_des=1.0, y_des=1.0, z_des=1.0):
        self.x_des = x_des
        self.y_des = y_des
        self.z_des = z_des
        
    def step(self ,state, input_vector):
        #self.state = [phi, theta, psi, p, q, r, x_dot, y_dot, z_dot, x, y, z]
        if(self.USE_PWM == 1 and self.USE_PID == 0):
            if(input_vector[0] < 1000.0):
                input_vector[0] = 1000.0
                print("Warning!! PWM value[0] less than 1000.0")
            if(input_vector[1] < 1000.0):
                input_vector[1] = 1000.0
                print("Warning!! PWM value[1] less than 1000.0")
            if(input_vector[2] < 1000.0):
                input_vector[2] = 1000.0
                print("Warning!! PWM value[2] less than 1000.0")
            if(input_vector[3] < 1000.0):
                input_vector[3] = 1000.0
                print("Warning!! PWM value[3] less than 1000.0")
            
            w1 = (855625.0/1000.0)*input_vector[0] - 855625.0
            w2 = (855625.0/1000.0)*input_vector[1] - 855625.0
            w3 = (855625.0/1000.0)*input_vector[2] - 855625.0
            w4 = (855625.0/1000.0)*input_vector[3] - 855625.0
            
            self.input_vector[0] = self.kt * (w1 + w2 + w3 + w4)
            self.input_vector[1] = self.kt * self.l * (w4 - w2)
            self.input_vector[2] = self.kt * self.l * (w1 - w3)
            self.input_vector[3] = self.kd * (w1 + w3 - w2 - w4)
            
            self.o = sqrt(w1) + sqrt(w3) - sqrt(w2) - sqrt(w4)
        else:
            self.input_vector = input_vector
        
        if(self.USE_PID == 0 and self.USE_PWM == 0):
            self.quad_motor_speed()
        
        self.state = state
            
        if self.pauseFlag == 0:    
            x_ddot = (-(cos(self.state[0])*sin(self.state[1])*cos(self.state[2]) + sin(self.state[2])*sin(self.state[0]))*self.input_vector[0] - self.kdx*self.state[6]) / self.m
            y_ddot = (-(cos(self.state[0])*sin(self.state[2])*sin(self.state[1]) - cos(self.state[2])*sin(self.state[0]))*self.input_vector[0] - self.kdy*self.state[7]) / self.m
            z_ddot = ((-(cos(self.state[0])*cos(self.state[1]))*self.input_vector[0] - self.kdz*self.state[8]) / self.m) + self.g
            
            self.p_dot = (self.state[4]*self.state[5]*(self.jy - self.jz) - self.jp*self.state[3]*self.o + self.l*self.input_vector[1]) / self.jx
            self.q_dot = (self.state[3]*self.state[5]*(self.jz - self.jx) + self.jp*self.state[4]*self.o + self.l*self.input_vector[2]) / self.jy
            self.r_dot = (self.state[3]*self.state[4]*(self.jx - self.jy) + self.input_vector[3]) / self.jz
            
            phi_dot = self.state[3] + sin(self.state[0])*tan(self.state[1])*self.state[4] + cos(self.state[0])*tan(self.state[1])*self.state[5]
            theta_dot = cos(self.state[0])*self.state[4] - sin(self.state[0])*self.state[5]
            psi_dot = (sin(self.state[0])/cos(self.state[1]))*self.state[4] + (cos(self.state[0])/cos(self.state[1]))*self.state[5]
            
            self.state[6] = x_ddot*self.Ts + self.state[6]
            self.state[7] = y_ddot*self.Ts + self.state[7]
            self.state[8] = z_ddot*self.Ts + self.state[8]
            
            self.state[9] = self.state[6]*self.Ts + self.state[9]
            self.state[10] = self.state[7]*self.Ts + self.state[10]
            self.state[11] = self.state[8]*self.Ts + self.state[11]
            if(self.state[11] < 0.0): self.state[11] = 0.0
            
            self.state[3] = self.p_dot*self.Ts + self.state[3]
            if(self.state[3] > self.p_max): self.state[3] = self.p_max
            if(self.state[3] < -self.p_max): self.state[3] = -self.p_max
            self.state[4] = self.q_dot*self.Ts + self.state[4]
            if(self.state[4] > self.p_max): self.state[4] = self.p_max
            if(self.state[4] < -self.p_max): self.state[4] = -self.p_max        
            self.state[5] = self.r_dot*self.Ts + self.state[5]
            if(self.state[5] > self.p_max): self.state[5] = self.p_max
            if(self.state[5] < -self.p_max): self.state[5] = -self.p_max
            
            self.state[0] = phi_dot*self.Ts + self.state[0]
            if(self.state[0] > self.phi_max): self.state[0] = self.phi_max
            if(self.state[0] < -self.phi_max): self.state[0] = -self.phi_max
            self.state[1] = theta_dot*self.Ts + self.state[1]
            if(self.state[1] > self.phi_max): self.state[1] = self.phi_max
            if(self.state[1] < -self.phi_max): self.state[1] = -self.phi_max
            self.state[2] = psi_dot*self.Ts + self.state[2]
            if(self.state[2] > self.psi_max): self.state[2] = self.psi_max
            if(self.state[2] < -self.psi_max): self.state[2] = -self.psi_max
           
            self.time_elapse += self.Ts
            
            return self.state
        else:
            return self.state
        
    def rotateGFtoBF(self, X, Y, Z, PHI, THETA, PSI):
        X_ = cos(PSI)*cos(THETA)*X + sin(PSI)*cos(THETA)*Y - sin(THETA)*Z
        Y_ = (cos(PSI)*sin(PHI)*sin(THETA) - cos(PHI)*sin(PSI))*X + (sin(PHI)*sin(PSI)*sin(THETA)+cos(PHI)*cos(PSI))*Y + (cos(THETA)*sin(PHI))*Z
        Z_ = (cos(PHI)*cos(PSI)*sin(THETA) + sin(PHI)*sin(PSI))*X + (cos(PHI)*sin(PSI)*sin(THETA)-cos(PSI)*sin(PHI))*Y + (cos(PHI)*cos(THETA))*Z
        return (X_, Y_, Z_)
        
    def PID_position(self):
        if(self.rstFlag == 0):
            self.x_error_sum = 0.0
            self.y_error_sum = 0.0
            self.z_error_sum = 0.0
        
        if self.pauseFlag == 0:   
            (self.x_des, self.y_des, self.z_des) = self.rotateGFtoBF(self.x_des, self.y_des, self.z_des, 0.0, 0.0, self.psi_des)
            (x_bf, y_bf, z_bf) = self.rotateGFtoBF(self.state[9], self.state[10], self.state[11], self.state[0], self.state[1], self.state[2])
            (x_bf_dot, y_bf_dot, z_bf_dot) = self.rotateGFtoBF(self.state[6], self.state[7], self.state[8], self.state[0], self.state[1], self.state[2])
            
            x_error = self.x_des - x_bf
            if(abs(x_error) < self.x_ki_lim): self.x_error_sum += x_error
            cp = self.x_kp * x_error
            ci = self.x_ki * self.Ts * self.x_error_sum
            if(ci > self.phi_max): ci = self.phi_max
            if(ci < -self.phi_max): ci = -self.phi_max
            cd = self.x_kd * x_bf_dot
            self.theta_des = -(cp + ci + cd)
            if(self.theta_des > self.phi_max): self.theta_des = self.phi_max
            if(self.theta_des < -self.phi_max): self.theta_des = -self.phi_max
            
            y_error = self.y_des - y_bf
            #print y_error
            if(abs(y_error) < self.x_ki_lim): self.y_error_sum += y_error
            cp = self.x_kp * y_error
            ci = self.x_ki * self.Ts * self.y_error_sum
            if(ci > self.phi_max): ci = self.phi_max
            if(ci < -self.phi_max): ci = -self.phi_max
            cd = self.x_kd * y_bf_dot
            self.phi_des = cp + ci + cd
            if(self.phi_des > self.phi_max): self.phi_des = self.phi_max
            if(self.phi_des < -self.phi_max): self.phi_des = -self.phi_max
            
            z_error = self.z_des - z_bf
            if(abs(z_error) < self.x_ki_lim): self.z_error_sum += z_error
            cp = self.z_kp * z_error
            ci = self.z_ki * self.Ts * self.z_error_sum
            if(ci > self.u1_max): ci = self.u1_max
            if(ci < self.u1_min): ci = self.u1_min
            cd = self.z_kd * self.state[8]
            self.input_vector[0] = -(cp + ci + cd)/(cos(self.state[1])*cos(self.state[0])) + (self.m*self.g)/(cos(self.state[1])*cos(self.state[0]))
            if(self.input_vector[0] > self.u1_max): self.input_vector[0] = self.u1_max
            if(self.input_vector[0] < self.u1_min): self.input_vector[0] = self.u1_min
        
    def PID_attitude(self):
        if(self.rstFlag == 0):
            self.phi_error_sum = 0.0
            self.theta_error_sum = 0.0
            self.psi_error_sum = 0.0
        if self.pauseFlag == 0:   
            phi_error = self.phi_des - self.state[0]
            if(abs(phi_error) < self.phi_ki_lim): self.phi_error_sum += phi_error
            cp = self.phi_kp * phi_error
            ci = self.phi_ki * self.Ts * self.phi_error_sum
            if(ci > self.p_max): ci = self.p_max
            if(ci < -self.p_max): ci = -self.p_max
            cd = self.phi_kd * self.state[3]
            self.p_des = cp + ci + cd
            if(self.p_des > self.p_max): self.p_des = self.p_max
            if(self.p_des < -self.p_max): self.p_des = -self.p_max
            
            theta_error = self.theta_des - self.state[1]
            if(abs(theta_error) < self.phi_ki_lim): self.theta_error_sum += theta_error
            cp = self.phi_kp * theta_error
            ci = self.phi_ki * self.Ts * self.theta_error_sum
            if(ci > self.p_max): ci = self.p_max
            if(ci < -self.p_max): ci = -self.p_max
            cd = self.phi_kd * self.state[4]
            self.q_des = cp + ci + cd
            if(self.q_des > self.p_max): self.q_des = self.p_max
            if(self.q_des < -self.p_max): self.q_des = -self.p_max
            
            psi_error = self.psi_des - self.state[2]
            if(abs(psi_error) < self.psi_ki_lim): self.psi_error_sum += psi_error
            cp = self.psi_kp * psi_error
            ci = self.psi_ki * self.Ts * self.psi_error_sum
            if(ci > self.p_max): ci = self.p_max
            if(ci < -self.p_max): ci = -self.p_max
            cd = self.psi_kd * self.state[5]
            self.r_des = cp + ci + cd
            if(self.r_des > self.p_max): self.r_des = self.p_max
            if(self.r_des < -self.p_max): self.r_des = -self.p_max
        
    def PID_rate(self):
        if(self.rstFlag == 0):
            self.p_error_sum = 0.0
            self.q_error_sum = 0.0
            self.r_error_sum = 0.0
            self.rstFlag = 1
        
        if self.pauseFlag == 0:
            p_error = self.p_des - self.state[3]
            if(abs(p_error) < self.p_ki_lim): self.p_error_sum += p_error
            cp = self.p_kp * p_error
            ci = self.p_ki * self.Ts * self.p_error_sum
            if(ci > self.u2_max): ci = self.u2_max
            if(ci < self.u2_min): ci = self.u2_min
            cd = self.p_kd * self.p_dot 
            self.input_vector[1] = cp + ci + cd
            if(self.input_vector[1] > self.u2_max): self.input_vector[1] = self.u2_max
            if(self.input_vector[1] < self.u2_min): self.input_vector[1] = self.u2_min
            
            q_error = self.q_des - self.state[4]
            if(abs(q_error) < self.p_ki_lim): self.q_error_sum += q_error
            cp = self.p_kp * q_error
            ci = self.p_ki * self.Ts * self.q_error_sum
            if(ci > self.u3_max): ci = self.u3_max
            if(ci < self.u3_min): ci = self.u3_min
            cd = self.p_kd * self.q_dot
            self.input_vector[2] = cp + ci + cd
            if(self.input_vector[2] > self.u3_max): self.input_vector[2] = self.u3_max
            if(self.input_vector[2] < self.u3_min): self.input_vector[2] = self.u3_min
            
            r_error = self.r_des - self.state[5]
            if(abs(r_error) < self.p_ki_lim): self.r_error_sum += r_error
            cp = self.p_kp * r_error
            ci = self.p_ki * self.Ts * self.r_error_sum
            if(ci > self.u4_max): ci = self.u4_max
            if(ci < self.u4_min): ci = self.u4_min 
            cd = self.p_kd * self.r_dot
            self.input_vector[3] = cp + ci + cd
            if(self.input_vector[3] > self.u4_max): self.input_vector[3] = self.u4_max
            if(self.input_vector[3] < self.u4_min): self.input_vector[3] = self.u4_min
        
    def quad_motor_speed(self):
        w1 = self.input_vector[0]/(4.0*self.kt) + self.input_vector[2]/(2.0*self.kt*self.l) + self.input_vector[3]/(4.0*self.kd)
        w2 = self.input_vector[0]/(4.0*self.kt) - self.input_vector[1]/(2.0*self.kt*self.l) - self.input_vector[3]/(4.0*self.kd)
        w3 = self.input_vector[0]/(4.0*self.kt) - self.input_vector[2]/(2.0*self.kt*self.l) + self.input_vector[3]/(4.0*self.kd)
        w4 = self.input_vector[0]/(4.0*self.kt) + self.input_vector[1]/(2.0*self.kt*self.l) - self.input_vector[3]/(4.0*self.kd)
        
        if(w1 > self.max_motor_speed_2): w1 = self.max_motor_speed_2
        if(w1 < self.min_motor_speed_2): w1 = self.min_motor_speed_2
        
        if(w2 > self.max_motor_speed_2): w2 = self.max_motor_speed_2
        if(w2 < self.min_motor_speed_2): w2 = self.min_motor_speed_2
        
        if(w3 > self.max_motor_speed_2): w3 = self.max_motor_speed_2
        if(w3 < self.min_motor_speed_2): w3 = self.min_motor_speed_2
        
        if(w4 > self.max_motor_speed_2): w4 = self.max_motor_speed_2
        if(w4 < self.min_motor_speed_2): w4 = self.min_motor_speed_2
        
        o1 = sqrt(w1)
        o2 = sqrt(w2)
        o3 = sqrt(w3)
        o4 = sqrt(w4)
        
        self.input_vector[0] = self.kt * (w1 + w2 + w3 + w4)
        self.input_vector[1] = self.kt * self.l * (w4 - w2)
        self.input_vector[2] = self.kt * self.l * (w1 - w3)
        self.input_vector[3] = self.kd * (w1 + w3 - w2 - w4)
        
        self.o = o1 - o2 + o3 - o4
        
    def rstEnv(self):
        self.state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.rstFlag = 0
        self.time_elapse = 0.0
        if(self.USE_PWM == 1):
            self.input_vector[0] = 1000.0;
            self.input_vector[1] = 1000.0;
            self.input_vector[2] = 1000.0;
            self.input_vector[3] = 1000.0;
        else:
            self.input_vector[0] = 0.0;
            self.input_vector[1] = 0.0;
            self.input_vector[2] = 0.0;
            self.input_vector[3] = 0.0;
    
    def pauseEnv(self):
        self.pauseFlag = 1
        
    def unpauseEnv(self):
        self.pauseFlag = 0
    
    def time_elapsed(self):
        return self.time_elapse
        
        
        
        
