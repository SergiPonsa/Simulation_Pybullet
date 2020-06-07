import numpy as np
import scipy as sp
import scipy.interpolate
import pandas as pd
import time
from Experiments_pybullet import SubstractExcel_2_excel

if __name__ == '__main__':

    #path_Excel_Mujoco = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Original_Mujoco_Training1_averageconverted.xlsx"
    #path_Excel_Mujoco = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/positions_from_joints_Mujoco.xlsx"
    path_Excel_Mujoco = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/positions_from_joints_19.xlsx"
    #path_Excel_Pybullet = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Train_parameters_result.xlsx"
    #path_Excel_Pybullet ="/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Test_enverionment.xlsx"
    #path_Excel_Pybullet = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Train_parameters_result_tcp_2.xlsx"
    #path_Excel_Pybullet = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Train_parameters_result_tcp_euc.xlsx"
    path_Excel_Pybullet = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Train_parameters_result_tcp_euc_rishabh.xlsx"
    #path_Excel_Pybullet = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/Test_enverionment_tcp.xlsx"

    SubstractExcel_2_excel(path_Excel_Pybullet,path_Excel_Mujoco,title = 'diff_from_mujoco_Trained_tcp_euc_rishabh',folder = "Paper_sub")

    print ("Done")
