import numpy as np
import scipy as sp
import scipy.interpolate
import pandas as pd
import time
from Experiments_pybullet import SubstractExcel_2_excel

if __name__ == '__main__':

    path_Excel_Mujoco = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Original_Mujoco_Training1_averageconverted.xlsx"
    #path_Excel_Pybullet = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Train_parameters_result.xlsx"
    path_Excel_Pybullet ="/home/sergi/GitHub/Simulation_Pybullet/Experiments_Training1/Original_average.xlsx"
    SubstractExcel_2_excel(path_Excel_Pybullet,path_Excel_Mujoco,title = 'diff_from_mujoco_NoTrained',folder = "Paper_sub")

    print ("Done")
