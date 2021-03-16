import math

# import imageio
import scipy.io
import numpy as np
import time
# import matplotlib
# import matplotlib.pyplot as plt


class Simulator:
    maintain_pose = False
    initial_state = [np.pi/2, -np.pi/2, np.pi/2, 0, 0, 0] # [1.86977729,1.24937109,2.00859607,0.90643269, 1.54136571, 0.29779118]  #[np.pi/2, -np.pi/2, np.pi/2, 0, 0, 0]
    state1 = [ 1.72891478, -1.36432606,  2.03544137,  0.99842782,  1.55507632,  0.15733697]
    initial_state = state1
    #[1.86977729 - 1.24937109  2.00859607  0.90643269  1.54136571  0.29779118]
    def __init__(self, default_xml="UR5gripper_with_slot.xml"):
        model = load_model_from_path(default_xml)
        self.sim = MjSim(model)
        self.data = self.sim.data
        self.sim.reset()
        self.sim.forward()
        self.model = model
        self.set_initial_state()
        self.graphics = True
        self.gripper_force = np.array([])

    #Set robot joints to its initial state
    def set_initial_state(self):
        state = self.sim.get_state()
        state.qpos[0:6] = self.initial_state
        self.sim.set_state(state)

    # Renderer Initialization
    def render(self):
        self.viewer = MjViewer(self.sim)
        self.viewer.render()
        self.viewer.cam.lookat[0] = -0.05  # x,y,z offset from the object (works if trackbodyid=-1)
        self.viewer.cam.lookat[1] = 0.260
        self.viewer.cam.lookat[2] = 0.87
        self.viewer.cam.elevation = -15  # camera rotation around the axis in the plane going through the frame origin (if 0 you just see a line)
        self.viewer.cam.distance = self.model.stat.extent * 0.08  # how much you "zoom in", model.stat.extent is the max limits of the arena
        self.viewer.cam.azimuth = 195
        self.viewer._hide_overlay = True
        vopt = self.viewer.vopt
        vopt.flags[10] = vopt.flags[11] = not vopt.flags[10] # Set contacts display to on

    def record_video(self): #Not Implemented
        self.viewer._record_video = True



    # def step(self, maintain_pos):
    #     if maintain_pos:
    #         self.sim.data.ctrl[:6] = np.dot(1 / 101, self.sim.data.qfrc_bias[0:6])
    #     self.sim.step()
    #     self.sim.forward()
    #     if self.graphics:
    #         self.viewer.render()

    def save_video(self, filename, fps): #Not Working Currently
        queue = self.viewer._video_queue
        writer = imageio.get_writer(filename, fps=fps)
        while True:
            frame = queue.get()
            if frame is None:
                break
            writer.append_data(frame)
        writer.close()
