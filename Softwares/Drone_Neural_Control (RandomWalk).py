import math
import random
import time
import numpy as np
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class DroneNeuralControl:
    def __init__(self):
        self.sim = RemoteAPIClient().getObject('sim')

        # Constants
        self.INPUT_MAX = 0.5
        self.INPUT_OFFSET = 0.25
        self.OUTPUT_YAW_MAX = 0.5
        self.OUTPUT_PITCH_MAX = 0.005
        self.OUTPUT_PITCH_BACK_MAX = 0.0002
        self.OUTPUT_ROLL_MAX = 0.005
        self.NEURAL_CONTROL_ACTIVATION_THRESHOLD = 0.0001

        # Initial ANEC parameters
        self.loop_count = 0
        self.loop = 0
        self.Yaw_command = 0.0
        self.count = 0
        self.TAU = 0.01

        a = np.random.uniform(-1.0, 1.0, 3)
        print("Random Initial Conditions:", a)
        self.N1, self.N2, self.Sa_t1 = a

        self.rho = 10.0
        self.gamma = 28.0
        self.beta = (8.0 / 3.0)
        self.BIAS = 0.01

        self.sigma = 9.1671
        self.x_bar = 0.085
        self.Adaptive_bias = 1.0

        self.N3 = self.N4 = self.N5 = self.N6 = 0.0

        # ANOAC parameters
        self.INPUT_Max = 0.5
        self.N_OA1 = self.N_OA2 = 0.0
        self.N_OA1_out = self.N_OA2_out = 0.0
        self.N_OA1_out_t = self.N_OA2_out_t = 0.0
        self.N_OA1_out_t1 = self.N_OA2_out_t1 = 0.0
        self.w_A11 = self.w_A22 = 0.5
        self.w_A12 = self.w_A21 = self.w_A12_temp = self.w_A21_temp = -0.5
        self.emu = 0.01
        self.emu_f = 0.0003
        self.imu = 0.015
        self.imu_f = 0.0003
        self.offset = -0.01
        self.Left_ref = 0.0
        self.Right_ref = 0.0
        
        # Roll neuron and output activations
        self.N_OA5 = self.N_OA6 = 0.0
        self.N_OA5_out = self.N_OA6_out = 0.0
        self.N_OA5_out_t = self.N_OA6_out_t = 0.0
        self.N_OA5_out_t1 = self.N_OA6_out_t1 = 0.0
        self.w_A55 = self.w_A66 = 0.5
        self.w_A56 = self.w_A65 = self.w_A56_temp = self.w_A65_temp = -0.5
        

        # Temporal neuron states
        self.vt_Roll5_t = 0.0
        self.vt_Roll6_t = 0.0
        self.vt_Roll5 = 0.0
        self.vt_Roll6 = 0.0

        # Control output
        self.neuralYawOutput = 0.0
        self.neuralPitchOutput = 0.0
        self.neuralRollOutput = 0.0
        
        self.sampling_distance = 50.0
        self.TurningStep = 0.0
        self.TurningCount = 0.0
        self.C_NeuronOutput = 0.0
        self.D_value = 0.0
        self.Step_value = 0.0
        self.TurningComplete = 0

    def get_input(self):
        sensor_L = self.sim.getObject('/Proximity_sensor_left')
        sensor_R = self.sim.getObject('/Proximity_sensor_right')
        
        Ldet, Ldist, *_ = self.sim.readProximitySensor(sensor_L)
        Rdet, Rdist, *_ = self.sim.readProximitySensor(sensor_R)
        #If distance is equal to 0, set reflex and input to 0.
        if Ldist == 0:
            self.Left_ref = 0
            Ldistance = 0
        #Otherwise subtract input offset and set the correct input
        else :
            Ldist -= self.INPUT_OFFSET
            Ldistance = self.INPUT_MAX - Ldist
        #If distance is equal to 0, set reflex and input to 0.
        if Rdist == 0:
            self.Right_ref = 0
            Rdistance = 0
        #Otherwise subtract input offset and set the correct input
        else :
            Rdist -= self.INPUT_OFFSET
            Rdistance = self.INPUT_MAX - Rdist     
        
        Left_input = (Ldistance) * (2.0 / (1.0*self.INPUT_Max)) - 1
        Right_input = (Rdistance) * (2.0 / (1.0*self.INPUT_Max)) - 1

        safe_L = self.sim.getObject('/Right_safety_sensor')
        safe_R = self.sim.getObject('/Left_safety_sensor')
        SLdetect, SLdist, *_ = self.sim.readProximitySensor(safe_L)
        SRdetect, SRdist, *_ = self.sim.readProximitySensor(safe_R) 
        safeLeft = (SLdist - 0.22)*100
        safeRight = (SRdist - 0.22)*100
        if safeLeft < 0:
            safeLeft = 15.0
        if safeRight < 0:
            safeRight = 15.0
        self.LRollreflex = 1 if safeLeft > 0.5 else 0
        self.RRollreflex = 1 if safeRight > 0.5 else 0
            
        safeLeft_input = (-2.0/15.0)*safeLeft + 1.0
        safeRight_input = (-2.0/15.0)*safeRight + 1.0
    
        return Left_input, Right_input, safeLeft_input, safeRight_input

    def synaptic_plasticity(self, Left_input, Right_input):
        self.Left_ref = 1 if Left_input > -0.5 else 0
        self.Right_ref = 1 if Right_input > -0.5 else 0
        vt_L = 0.5 * self.N_OA1_out + 0.5
        vt_R = 0.5 * self.N_OA2_out + 0.5
        vt_L_t1 = 0.5 * self.N_OA1_out_t + 0.5
        vt_R_t1 = 0.5 * self.N_OA2_out_t + 0.5

        self.w_A11 += self.emu * vt_L_t1 * vt_L * self.Left_ref + self.emu_f * (self.offset - vt_L) * self.w_A11 * self.w_A11
        self.w_A22 += self.emu * vt_R_t1 * vt_R * self.Right_ref + self.emu_f * (self.offset - vt_R) * self.w_A22 * self.w_A22
        self.w_A12_temp -= self.imu * vt_L_t1 * vt_L * self.Left_ref + self.imu_f * (self.offset - vt_L) * self.w_A12_temp * self.w_A12_temp
        self.w_A21_temp -= self.imu * vt_R_t1 * vt_R * self.Right_ref + self.imu_f * (self.offset - vt_R) * self.w_A21_temp * self.w_A21_temp
        self.w_A12 = 0.5 * (self.w_A12_temp + self.w_A21_temp)
        self.w_A21 = self.w_A12
        
        # Change Roll outputs from range [-1, 1] to [0, 1] as described in formulas
        self.vt_Roll5_t = self.vt_Roll5
        self.vt_Roll6_t = self.vt_Roll6  # Note: This line likely intended to use vt_Roll_Neuron4
        self.vt_Roll5 = (self.N_OA5_out + 1) / 2
        self.vt_Roll6 = (self.N_OA6_out + 1) / 2

        # Calculate Roll synaptic weights
        self.w_A55 += (self.emu *self.vt_Roll5_t *self.vt_Roll5 *self.LRollreflex) + (self.emu_f *(self.offset - self.vt_Roll5) * self.w_A55 *self.w_A55)
        self.w_A66 += (self.emu *self.vt_Roll6_t *self.vt_Roll6 *self.RRollreflex) + (self.emu_f *(self.offset - self.vt_Roll6) *self.w_A66 *self.w_A66)
        self.w_A56_temp -= (self.imu *self.vt_Roll5_t *self.vt_Roll5 *self.LRollreflex) + (self.imu_f *(self.offset - self.vt_Roll5) * self.w_A56_temp *self.w_A56_temp)
        self.w_A65_temp -= (self.imu *self.vt_Roll6_t *self.vt_Roll6 *self.RRollreflex) + (self.imu_f *(self.offset - self.vt_Roll6) *self.w_A65_temp *self.w_A65_temp)
        self.w_A56 = 0.5 * (self.w_A56_temp + self.w_A65_temp)
        self.w_A65 = self.w_A65
                
    def map_output(self):
        self.neuralPitchOutput *= -1
        if self.neuralPitchOutput > 0:
            self.neuralPitchOutput *= self.OUTPUT_PITCH_MAX
        else:
            self.neuralPitchOutput *= self.OUTPUT_PITCH_BACK_MAX
            
        self.neuralYawOutput = (self.neuralYawOutput / 2) * self.OUTPUT_YAW_MAX
        # If the output for yaw is above the threshold, use the neural output.
        if abs(self.neuralYawOutput) < self.NEURAL_CONTROL_ACTIVATION_THRESHOLD:
            # Generate random noise in the range [-0.1, 0.1]
             self.neuralYawOutput = random.uniform(-0.1, 0.1)
                 
        self.neuralRollOutput *= self.OUTPUT_ROLL_MAX

    def send_output_command(self):
        self.sim.setFloatSignal('yaw_command', self.neuralYawOutput)
        self.sim.setFloatSignal('pitch_command', self.neuralPitchOutput)
        self.sim.setFloatSignal('roll_command', self.neuralRollOutput)
        self.sim.setFloatSignal('yaw_Iterations', 20)
        self.sim.setFloatSignal('pitch_Iterations', 20)
        self.sim.setFloatSignal('roll_Iterations', 20)

    def NeuralControl(self):
        Left_input, Right_input, Roll_Left_input, Roll_Right_input  = self.get_input()

        self.N_OA1 = 7.0 * Left_input + self.w_A11 * self.N_OA1_out + self.w_A21 * self.N_OA2_out
        self.N_OA2 = 7.0 * Right_input + self.w_A22 * self.N_OA2_out + self.w_A12 * self.N_OA1_out
        self.N_OA1_out = math.tanh(self.N_OA1)
        self.N_OA2_out = math.tanh(self.N_OA2)
        ANOAC_out = self.N_OA1_out - self.N_OA2_out
        
        self.N_OA5 = (Roll_Left_input * 7.0 + self.N_OA5_out * self.w_A55 + self.N_OA6_out * self.w_A65)
        self.N_OA6 = (Roll_Right_input * 7.0 + self.N_OA6_out * self.w_A66 + self.N_OA5_out * self.w_A56)
        #self.N_OA5 = (Roll_Left_input * 1.5)
        #self.N_OA6 = (Roll_Right_input * 1.5)
        self.N_OA5_out = math.tanh(self.N_OA5)
        self.N_OA6_out = math.tanh(self.N_OA6)
        self.Roll_Output = math.tanh(3.0*self.N_OA5_out - 3.0*self.N_OA6_out)
        
        print("Left_input: %f" %(Left_input))
        print("Right_input: %f" %(Right_input))
        print("self.N_OA1_out: %f" %(self.N_OA1_out))
        print("self.N_OA2_out %f" %(self.N_OA2_out))
        print("ANOAC_out: %f" %(ANOAC_out))
        """
        print("Roll_Left_input: %f" %(Roll_Left_input))
        print("Roll_Right_input: %f" %(Roll_Right_input))
        print("self.N_OA5_out: %f" %(self.N_OA5_out))
        print("self.N_OA6_out %f" %(self.N_OA6_out))
        print("self.Roll_Output: %f" %(self.Roll_Output))
        """
        # Compute pitchRight and pitchLeft using tanh activation
        pitch_left = math.tanh((self.N_OA2_out * 3.0) + (self.N_OA5_out * 1.0))
        pitch_right = math.tanh((self.N_OA1_out * 3.0) + (self.N_OA6_out * 1.0))
        # Compute the final pitch output
        pitch_output = math.tanh(pitch_right + pitch_left)

        self.N_OA1_out_t1 = self.N_OA1_out_t
        self.N_OA1_out_t = self.N_OA1_out
        self.N_OA2_out_t1 = self.N_OA2_out_t
        self.N_OA2_out_t = self.N_OA2_out
        self.N_OA5_out_t = self.N_OA5_out
        self.N_OA6_out_t = self.N_OA6_out

        #self.synaptic_plasticity(Left_input, Right_input)   
        
        #mapping haed direction
        #mapped_heading = 180.0*(heading_angle/phi) + 180.0 ;

        #Define heading angle generation  *********************************************************
        self.D_value = np.random.randn()

        #set input to the chaotic neuron (C1, C2)
        #if (count > 50) {
        if (self.count > self.sampling_distance) :
            self.C_NeuronOutput = 0.0
            self.count = 0
            self.TurningCount = 0
            self.TurningComplete = 0
        elif (self.count == 1) :
            #// Define moving step length
            self.Step_value = abs(np.random.randn())  # Single float, absolute value
            self.sampling_distance = 50 * self.Step_value
            if (self.sampling_distance < 50.0):
                self.sampling_distance = 50.0

            #// Define chaotic value for turning angle
            self.TurningStep = self.D_value*90.0
            #RandomAngle = D_value*90.0
            #ResultAngle = mapped_heading + RandomAngle;

            self.count += 1
            self.TurningCount = 0
            self.TurningComplete = 0

        else :
            if (self.TurningStep > 0.0) :
                temp = 1   # turn right flag
            elif (self.TurningStep < 0.0):
                temp = -1  #// turn left flag
            else :
                temp = 0
                
            #// Define turning direction
            if((self.TurningCount < (self.TurningStep)) and  (temp == 1) and self.TurningComplete == 0) : 
                if(self.TurningCount > (self.TurningStep-2.0)):
                    self.TurningComplete = 1
                
                self.C_NeuronOutput  = -1.0  #// turn left
                #chaotic_n = chaotic_n + 1
                
                self.TurningCount += 1

            elif ((self.TurningCount < ((-1)*self.TurningStep)) and  (temp == -1) and self.TurningComplete == 0):
                if (self.TurningCount > ((-1)*self.TurningStep - 2.0)):
                    self.TurningComplete = 1
                
                self.C_NeuronOutput  = 1.0 #// turn right
                #chaotic_p = chaotic_p + 1
                
                self.TurningCount += 1
            
            else :
                self.C_NeuronOutput  = 0.0
                #// TurningCount = 0;

            #// Filter the chaotic output.
            #// G_expOutput = G_Output + G_expOutput_t*0.25 ;
            #// C_expOutput = C_NeuronOutput ;
            #// G_expOutput_t = G_expOutput;
            self.count += 1
            #//TurningCount++;
        
        RW_out = self.C_NeuronOutput
        
        
        # Control output
        self.neuralYawOutput = RW_out + 0.5*ANOAC_out
        self.neuralPitchOutput = pitch_output
        self.neuralRollOutput = self.Roll_Output
        
        print("self.w_A11: %f" %(self.w_A11))
        print("self.w_A22: %f" %(self.w_A22))
        print("self.w_A12,21: %f" %(self.w_A12))
        print("D_value: %f" %(self.D_value))
        print("self.TurningStep: %f" %(self.TurningStep))
        print("Step_value: %f" %(self.Step_value))
        print("sampling_distance: %f" %(self.sampling_distance))
        print("RW_out: %f" %(RW_out))
        print("self.neuralYawOutput: %f" %(self.neuralYawOutput))
        print("self.neuralPitchOutput: %f" %(self.neuralPitchOutput))
        print("self.neuralRollOutput: %f" %(self.neuralRollOutput))
        print("-------------------------------------------------------")
        
        self.map_output()
        self.send_output_command()
        #time.sleep(0.1)

    def run(self):
        self.sim.setStepping(True)
        self.sim.startSimulation()
        time.sleep(0.1)
        try:
            while self.sim.getSimulationState() != self.sim.simulation_stopped:
                self.sim.step()
                self.NeuralControl()
        except Exception as e:
            print(f"[ERROR] {e}")
        finally:
            self.sim.stopSimulation()
            print("Simulation stopped.")

if __name__ == '__main__':
    drone = DroneNeuralControl()
    drone.run()
