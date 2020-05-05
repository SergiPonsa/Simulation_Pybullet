# ------------------------------------------------------------------------------------------------------

class RobotDataBase():
    __doc__ = """
    This class has the following functions:
    save_time()
        |- It save the time in the list to be able to have a record that when it happened
    """
    def __init__(self,name = "Database",time_step = (1.0/240.0)):
        self.name = name
        self.joints_angles_rad = []
        self.joint_angles_vel_rad = []
        self.joint_torques = []
        self.tcp_position = []
        self.tcp_orientation_q = []
        self.tcp_orientation_e = []
        self.time = []

        self.time_step = time_step
        self.counter = 0

    def save_time (self):
        self.counter += 1
        self.time.append(self.counter * self.time_step)
