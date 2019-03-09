#!/usr/bin/python2

import serial
import time
import Tkinter as tk
from PIL import ImageTk, Image

# bias = {5,-9,-7, -7,-8,+9, -9,0,3, 0,-7,-9}

class testGUI:
	def __init__(self, master):
		self.master = master
		self.servo_names = ['s0', 's1', 's2', 's3', 's4', 's5', 's6', 's7', 's8', 's9', 's10', 's11', 's12', 's13', 's14', 's15']
		self.servo_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

		#self.bias_list = [5, +2, -7, -7, -8, 9, -9, 0, 3, 0, -7, -9]
		#self.dir_list = [-1,1,-1, -1,1,1, -1,-1,1, -1,1,-1]

		self.bias_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		self.dir_list =  [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]		

		self.servo_scales = []
		for r, servo_name in enumerate(self.servo_names):
			tk.Label(text = servo_name, width = 10).grid(row = r%8, column = 0 if r<8 else 2)
			servo_scale = tk.Scale(self.servo_pos[r], orient = tk.HORIZONTAL, width = 10, to = 180, length=300)
			servo_scale.set(90)
			servo_scale.grid(row = r%8, column = 1 if r<8 else 3)
			self.servo_scales.append(servo_scale)

		r = r%8 + 1
		self.go_button = tk.Button(master, text = "GO", command = self.go_callback)
		self.go_button.grid(row = r, column = 0)
		
		self.q_button = tk.Button(master, text = "Q", command = self.q_callback)
		self.q_button.grid(row = r, column = 1)
		
		self.qp_button = tk.Button(master, text = "QP", command = self.qp_callback)
		self.qp_button.grid(row = r, column = 2)
		
		self.walk_button = tk.Button(master, text = "Walk", command = self.walk_callback)
		self.walk_button.grid(row = r, column = 3)

		self.walk2_button = tk.Button(master, text = "Walk2", command = self.walk2_callback)
		self.walk2_button.grid(row = r, column = 5)
		
		self.home_button = tk.Button(master, text = "Home", command = self.home_callback)
		self.home_button.grid(row = r, column = 4)
		
		# only for S5, pin 7
		self.scale = tk.Scale(self.servo_pos[5], orient = tk.HORIZONTAL, width = 10, to = 180, length=300)
		self.scale.set(90)
		self.scale.bind("<ButtonRelease-1>", self.scale_callback)
		self.scale.grid(row = r+1, column = 1)

		#self.img = ImageTk.PhotoImage(Image.open("joints_config.png"))
		#self.imglabel = tk.Label(self.master, image=self.img).grid(row = 0, column = 4, rowspan = 20, columnspan = 20, sticky = tk.W+tk.E+tk.N+tk.S, padx = 5, pady = 5);

		#self.client = actionlib.SimpleActionClient('hd_servo_sequence_action_server', hd_sequenceAction)
		#self.client.wait_for_server()

		try:
			#self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1)
			self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout = 1)
			results = self.ser.readline()
			print(results)
		except:
			print "Cannot connect to the robot"		

	def go_callback(self):
		for i, servo_name in enumerate(self.servo_names):
			self.servo_pos[i] = (self.servo_scales[i].get()-90.0)*self.dir_list[i] + self.bias_list[i]
			self.servo_pos[i] = self.servo_pos[i] * 3.14 / 180.0
		print self.servo_pos
		
		self.SendCommand()

	def scale_callback(self, event):
		self.servo_pos[5] = (self.scale.get()-90.0)*self.dir_list[5] + self.bias_list[5]
		self.servo_pos[5] = self.servo_pos[5] * 3.14 / 180.0
		
		self.SendCommand()

	def q_callback(self):
		self.ser.write("Q\r")
		result = self.ser.readline()
		print(result)
		
	def qp_callback(self):
		self.ser.write("QP\r")
		result = self.ser.readline()
		
		if(len(result) == 19):
			degs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			for i in range(18):
				pulse = ord(result[i]) * 10
				degs[i] = (pulse - 448) / 11.8 - 82
				#motor_state = MotorState()
				#motor_state.joint_id = i
				#motor_state.pulse = ord(results[i]) * 10
				#motor_state.degrees = (motor_state.pulse - 448) / 11.8 - 82
				#motor_state.radians = motor_state.degrees / 180.0 * 3.14
				#states_msg.motor_states.append(motor_state)
			print(degs)
				
	def SendCommand(self):
		commands = ''
		for i in range(16):
			pos = (self.servo_pos[i] / 3.14 * 180.0 + 82) * 11.8 + 448 #radians to pulse
			#if command.invert:
			#    m_pos = 1415 - (m_pos - 1415)
			commands += '#' + str(i) + ' P' + str(int(pos)) + ' T' + str(int(200)) + ' '
		commands += '\r'
		print commands
		self.ser.write(commands)				
	
	def walk_callback(self):
		upper_first_step = 140
		lower_first_step = 130
		shoulder_first_step = 130
		upper_second_step = 110
		lower_second_step = 150
		shoulder_second_step = 100
		upper_home = 55
		lower_home = 85
		shoulder_home = 100
		self.servo_pos[2] = (shoulder_first_step - 90.0)*self.dir_list[2] + self.bias_list[0]
		self.servo_pos[2] = self.servo_pos[2] * 3.14 / 180.0
		self.servo_scales[2].set(shoulder_first_step)
		self.SendCommand()
		time.sleep(.3)
		self.servo_pos[0] = (upper_first_step - 90.0)*self.dir_list[0] + self.bias_list[0]
		self.servo_pos[0] = self.servo_pos[0] * 3.14 / 180.0
		self.servo_scales[0].set(upper_first_step)
		self.SendCommand()
		time.sleep(.3)
		self.servo_pos[1] = (lower_first_step - 90.0)*self.dir_list[1] + self.bias_list[0]
		self.servo_pos[1] = self.servo_pos[1] * 3.14 / 180.0
		self.servo_scales[1].set(lower_first_step)
		self.SendCommand()
		time.sleep(.3)
		self.servo_pos[0] = (upper_second_step - 90.0)*self.dir_list[0] + self.bias_list[0]
		self.servo_pos[0] = self.servo_pos[0] * 3.14 / 180.0
		self.servo_scales[0].set(upper_second_step)
		self.servo_pos[1] = (lower_second_step - 90.0)*self.dir_list[1] + self.bias_list[0]
		self.servo_pos[1] = self.servo_pos[1] * 3.14 / 180.0
		self.servo_scales[1].set(lower_second_step)
		self.servo_pos[2] = (shoulder_second_step - 90.0)*self.dir_list[2] + self.bias_list[0]
		self.servo_pos[2] = self.servo_pos[2] * 3.14 / 180.0
		self.servo_scales[2].set(shoulder_second_step)
		self.SendCommand()
		time.sleep(.3)
		self.servo_pos[0] = (upper_home - 90.0)*self.dir_list[0] + self.bias_list[0]
		self.servo_pos[0] = self.servo_pos[0] * 3.14 / 180.0
		self.servo_scales[0].set(upper_home)
		self.servo_pos[1] = (lower_home - 90.0)*self.dir_list[1] + self.bias_list[0]
		self.servo_pos[1] = self.servo_pos[1] * 3.14 / 180.0
		self.servo_scales[1].set(lower_home)
		self.servo_pos[2] = (shoulder_home - 90.0)*self.dir_list[2] + self.bias_list[0]
		self.servo_pos[2] = self.servo_pos[2] * 3.14 / 180.0
		self.servo_scales[2].set(shoulder_home)
		self.SendCommand()

	def walk2_callback(self):
		upper_second_step = 130
		lower_second_step = 150
		shoulder_second_step = 100
		self.servo_pos[0] = (upper_second_step - 90.0)*self.dir_list[0] + self.bias_list[0]
		self.servo_pos[0] = self.servo_pos[0] * 3.14 / 180.0
		self.servo_scales[0].set(upper_second_step)
		self.servo_pos[1] = (lower_second_step - 90.0)*self.dir_list[1] + self.bias_list[0]
		self.servo_pos[1] = self.servo_pos[1] * 3.14 / 180.0
		self.servo_scales[1].set(lower_second_step)
		self.servo_pos[2] = (shoulder_second_step - 90.0)*self.dir_list[2] + self.bias_list[0]
		self.servo_pos[2] = self.servo_pos[2] * 3.14 / 180.0
		self.servo_scales[2].set(shoulder_second_step)
		self.SendCommand()
		
	def home_callback(self):
		upper_home = 55
		lower_home = 85
		shoulder_home = 100
		self.servo_pos[0] = (upper_home - 90.0)*self.dir_list[0] + self.bias_list[0]
		self.servo_pos[0] = self.servo_pos[0] * 3.14 / 180.0
		self.servo_scales[0].set(upper_home)
		self.servo_pos[1] = (lower_home - 90.0)*self.dir_list[1] + self.bias_list[0]
		self.servo_pos[1] = self.servo_pos[1] * 3.14 / 180.0
		self.servo_scales[1].set(lower_home)
		self.servo_pos[2] = (shoulder_home - 90.0)*self.dir_list[2] + self.bias_list[0]
		self.servo_pos[2] = self.servo_pos[2] * 3.14 / 180.0
		self.servo_scales[2].set(shoulder_home)
		self.SendCommand()
	
	
def main():
	root = tk.Tk()
	#rospy.init_node('hd_servo_gui')
	app = testGUI(root)
	root.mainloop()

if __name__ == "__main__":
    main()
