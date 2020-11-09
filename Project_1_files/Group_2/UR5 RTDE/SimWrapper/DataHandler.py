import numpy as np

class DataObject:

    def __init__(self,size = 3):
        self.size = size
        self.x = np.asarray([])
        self.y = np.asarray([])
        self.z = np.asarray([])


    def log_data(self, data_vector):
        self.x = np.append(self.x,data_vector[0])
        self.y = np.append(self.y, data_vector[1])
        self.z = np.append(self.z, data_vector[2])
