#Lattuada Valentina
#Mazzani Stefano
#Merlo Francesco
#Meroni Francesco

############################################################################

from ikpy.chain import Chain
import time
import tkinter as tk
from tkinter import messagebox
import numpy as np
from Vosk_Visual_Simulation import VoskNode
from Google_Visual_Simulation import GoogleNode
import matplotlib.pyplot as plt

'''
position :    XYZ cartesian position of the eef
orientation : RPY Euler orientation of the eef
pose :        position and orientation of the eef
q :           joint configuration in [rad]
'''

##########################################################
# CLASS BRIDGE ###########################################

class Bridge:
    def __init__(self, urdf):
        # Create the `Chain` object from the URDF file
        self.chain = Chain.from_urdf_file(urdf,
                                          active_links_mask=[False,  # base link joint
                                                             True, True, True, True,
                                                             False  # 'wrist' fixed joint
                                                             ]
                                          )
        # Initialize the joint pose of the robot
        self.q = [0, 0, 0, 0, 0, 0]
        # NOTE. `ikpy` appends one base link (`q[0]`) and considers the
        #       fixed joint between forearm and hand (`q[5]`) though the
        #       value is always 0
        # Initialize the current and target eef pose
        self.update_eef_pose_current()
        self.eef_pose_target = self.eef_pose_current

        # Defining the position of target
        self.target_x = [0.6, 0.6, 0.6, 0.8]
        self.target_y = [-0.3, -0.40, 0.2, 0.0]
        self.target_z = [0.1, 0.1, 0.1, 0.1]
        self.n = 0
        self.time_start = -1
        self.time_finish = -1
        self.last_target_reached=0
        self.cont = 0

    def update_eef_pose_target(self, dx=0.0, dy=0.0, dz=0.0):
        self.eef_pose_target[0, 3] = self.eef_pose_current[0, 3] + dx
        self.eef_pose_target[1, 3] = self.eef_pose_current[1, 3] + dy
        self.eef_pose_target[2, 3] = self.eef_pose_current[2, 3] + dz

    def update_eef_pose_current(self):
        self.eef_pose_current = self.chain.forward_kinematics(self.q,
                                                              full_kinematics=False
                                                              )
    def get_target(self):

        # Saving in 'hand' the coordinates of the end effector
        hand = self.eef_pose_current[:3, -1]
        # setting the tolerance (max distance to get close to a target)
        toll =0.05

        # Calculating the distances in x, y, z between hand and target
        deltax = hand[0]-self.target_x[self.n]
        deltay = hand[1]-self.target_y[self.n]
        deltaz = hand[2]-self.target_z[self.n]

        # Calculating the distance between hand and target
        distance = np.sqrt(deltax*deltax+deltay*deltay+deltaz*deltaz)

        # If distance < toll, and it is not the last target ==> select the next target
        if distance < toll and self.n<len(self.target_x):
            self.n +=1

        # If current target is last target ...
        if self.n==len(self.target_x):

            self.last_target_reached = 1                    # Break the cycle
            manager.destroy()                               # Destroy the figure
            self.time_finish = time.perf_counter()          # Set finish time stamp
            task_time = self.time_finish - self.time_start  # Calculating task time
            task_time = round(task_time, 1)
            print("*************************************")

            # print the message to user
            print("TIME: ", task_time)
            print("NUMBER OF ITERATIONS: ", self.cont)
            root = tk.Tk()
            root.withdraw()
            messagebox.showinfo("END OF THE TEST", "TIME :" + " " + str(task_time) + " s")
            messagebox.showinfo("END OF THE TEST", "NUMBER OF ITERATIONS: " + " " + str(self.cont) )
            root.quit()
            root.destroy()






if __name__ == "__main__":


    # 0. CREATING THE OBJECT

    bridge = Bridge("./bridge.urdf")

    # 1. SHOWING A MENU WITH THE CHOICES

    def get_selection():
        selected_interface= selection.get()
        if selected_interface:
            messagebox.showinfo("Selection", f"You selected: {selected_interface}")
            root.quit()
            root.destroy()

        else:
            messagebox.showwarning("Attention", "Please select an interface")


    # Creating selection window
    root = tk.Tk()
    root.title("Choose Interface")
    root.attributes('-fullscreen', True)

    # Defining the variable in which the system saves the selection
    selection = tk.StringVar(value="")

    # Creating widget
    label = tk.Label(root, text="Which interface do you want to use?", font=("Arial", 30))
    label.pack(pady=100)
    google_button = tk.Radiobutton(root, text="GOOGLE", variable=selection, value="GOOGLE", font=("Arial", 50),
                                   indicatoron=False, padx=20, pady=40, selectcolor= 'orange' )
    google_button.pack(pady=30)
    vosk_button = tk.Radiobutton(root, text="VOSK", variable=selection, value="VOSK", font=("Arial", 50),
                                   indicatoron=False, padx=20, pady=40, selectcolor = 'lightgreen')
    vosk_button.pack(pady=100)
    submit_button = tk.Button(root, text="Confirm", command=get_selection, font=("Arial", 30) )
    submit_button.pack(pady=200)

    root.mainloop()
    selected_interface = selection.get()
    print(f"Selected Interface: {selected_interface}")

    # 2. CREATE SIMULATION ENVIRONMENT

    # Setting parameters
    elev = 20
    azim = 195

    # Saving in x_h, y_h, z_h the coordinates of the hand
    [x_h, y_h, z_h] = bridge.eef_pose_current[:3, -1]

    # Saving in x_t, y_t, z_t the coordinates of the target

    x_t = bridge.target_x[bridge.n]
    y_t = bridge.target_y[bridge.n]
    z_t = bridge.target_z[bridge.n]

    # Generating figure

    ax = plt.figure().add_subplot(111, projection='3d')
    ax.scatter(x_t, y_t, z_t, c ='red', s=100)
    ax.scatter(x_h, y_h, -0.5, c='blue', s= 70)
    ax.scatter(x_t, y_t, -0.5, c='black', s=70)
    ax.plot([x_t, x_t], [y_t, y_t], [-0.5, 1], color='red', linestyle='--')
    ax.plot([x_t, x_t], [1, -1], [z_t, z_t], color='red', linestyle='--')
    ax.plot([0, 1.2], [y_t, y_t], [z_t, z_t], color='red', linestyle='--')

    ax.set_title('BRIDGE simulation')
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlim3d([-0., 1.0])
    ax.set_ylim3d([-1.0, 1.0])
    ax.set_zlim3d([-0.5, 0.5])
    ax.set_autoscale_on(False)
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    # 3. CREATE VOSK_NODE OR GOOGLE_NODE BASING ON SELECTION

    if selected_interface=='GOOGLE':
        node = GoogleNode()

    if selected_interface=='VOSK':
        node = VoskNode()

    # 4. RUN THE SIMULATION CYCLE

    # node.stop == 0: recognizer still on, node.stop == 1: recognizer off
    # last_target_reached = 0: targets remaining, last_target_reached = 1: all target completed

    while(node.stop==0 and bridge.last_target_reached==0):

        # 0. PLOTTING CURRENT STATE OF BRIDGE WITH "SPEAK TITLE"

        plt.cla()
        x_t = bridge.target_x[bridge.n]
        y_t = bridge.target_y[bridge.n]
        z_t = bridge.target_z[bridge.n]
        bridge.chain.plot(bridge.q, ax)
        ax.scatter(bridge.target_x[bridge.n], bridge.target_y[bridge.n], bridge.target_z[bridge.n], c ='red', s=100)

        # showing lines
        ax.plot([x_t, x_t], [y_t, y_t], [-0.5,1], color='red', linestyle='--')
        ax.plot([x_t, x_t], [1, -1], [z_t, z_t], color='red', linestyle='--')
        ax.plot([0, 1.2], [y_t, y_t], [z_t, z_t], color='red', linestyle='--')
        ax.set_title('SPEAK NOW...')
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlim3d([-0., 1.0])
        ax.set_ylim3d([-1.0, 1.0])
        ax.set_zlim3d([-0.5, 0.5])
        ax.set_autoscale_on(False)
        [x_h, y_h, z_h] = bridge.eef_pose_current[:3, -1]
        # showing shadows of hand and target
        ax.scatter(x_h, y_h, -0.5, c='blue', s=50)
        ax.scatter(x_t, y_t, -0.5, c='black', s=50)
        plt.show(block=False)

        plt.pause(0.002)          # WARNING: Set this parameter to get "SPEAK NOW" title and system synchronised


        # 1. CALLING NODE OF SPEECH RECOGNITION

        node.loop()
        bridge.cont += 1

        # 2. GET THE START TIME STAMP, ONLY AT THE FIRST ITERATION

        if bridge.time_start==-1:
            bridge.time_start = time.perf_counter()

        # 3. SAVING COORDINATES VARIATIONS

        [dx, dy, dz] = node.coord_variation

        # 4. UPDATE ROBOT STATE

        bridge.update_eef_pose_current()
        bridge.update_eef_pose_target(dx, dy, dz)
        # Compute the IK
        q_i = bridge.q  # later used to plot
        bridge.q = bridge.chain.inverse_kinematics_frame(bridge.eef_pose_target,
                                                         bridge.q,
                                                         optimizer="scalar"
                                                         )
        q_f = bridge.q  # re-assigned for the sake of clarity


        # 5. DISPLAY THE MOVEMENTS

        steps = 30

        for i in range(steps + 1):  # +1 needed as range(val) goes from 0 to val-1

            q = (q_f - q_i) / (steps) * i + q_i
            plt.cla()  # clear the current axes
            bridge.chain.plot(q, ax)
            # showing entity only if it is not the current command
            if node.current_command==node.current_entity:
                ax.set_title(node.current_command.upper())
            else:
                ax.set_title(node.current_command.upper()+' '+node.current_entity.upper())
            ax.scatter(bridge.target_x[bridge.n], bridge.target_y[bridge.n], bridge.target_z[bridge.n], c ='red', s=100)


            ax.plot([x_t, x_t], [y_t, y_t], [-0.5, 1], color='red', linestyle='--')
            ax.plot([x_t, x_t], [1, -1], [z_t, z_t], color='red', linestyle='--')
            ax.plot([0, 1.2], [y_t, y_t], [z_t, z_t], color='red', linestyle='--')
            ax.scatter(x_t, y_t, -0.5, c='black', s=70)
            ax.view_init(elev=elev, azim=azim)
            ax.set_xlim3d([-0., 1.0])
            ax.set_ylim3d([-1.0, 1.0])
            ax.set_zlim3d([-0.5, 0.5])
            ax.set_autoscale_on(False)
            # NOTE. These settings has to be re-assigned at every iteration as they are cleared by plt.cla()
            plt.show(block=False)
            plt.pause(0.001)

        bridge.update_eef_pose_current()
        [x_h, y_h, z_h] = bridge.eef_pose_current[:3, -1]
        ax.scatter(x_h, y_h, -0.5, c='blue', s=70)

        # 6. CALLING FUNCTION GET_TARGET TO ANALYSE RELATIVE POSITION BETWEEN HAND AND TARGET

        bridge.get_target()









