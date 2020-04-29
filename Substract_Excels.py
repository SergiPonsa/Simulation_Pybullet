import numpy as np
import scipy as sp
import scipy.interpolate
import pandas as pd
import time
from Experiments_pybullet import SubstractExcel_2_excel

if __name__ == '__main__':

    path_Excel_Mujoco = "/home/sergi/GitHub/Mujoco_TFM/Paper_resampled/Single_6_averageconverted.xlsx"
    path_Excel_Pybullet = "/home/sergi/GitHub/Simulation_Pybullet/Paper_Experiments/original_Single_6_average.xlsx"

    SubstractExcel_2_excel(path_Excel_Pybullet,path_Excel_Mujoco,title = 'diff_from_mujoco_6',folder = "Paper_sub")

    print ("Done")
