import numpy as np
import scipy as sp
import scipy.interpolate
import pandas as pd
import time

def ConvertToSameSamples_excel2d(path_Excel_1,path_Excel_2,title = ''):
    DF_Excel1 = pd.read_excel(path_Excel_1)
    DF_Excel2 = pd.read_excel(path_Excel_2)

    np_Excel1 = DF_Excel1.to_numpy()
    np_Excel2 = DF_Excel2.to_numpy()

    if(np_Excel1.shape[0]>np_Excel2.shape[0]):
        np_Excel_to_reshape = np_Excel1
        np_Excel_Ok = np_Excel2
        path_to_save = path_Excel_1.strip('.xlsx')+title+'.xlsx'

    if(np_Excel1.shape[0]<np_Excel2.shape[0]):
        np_Excel_to_reshape = np_Excel2
        np_Excel_Ok = np_Excel1
        path_to_save = path_Excel_2.strip('.xlsx')+title+'.xlsx'

    xmin = np_Excel_Ok[0,0]
    xmax = np_Excel_Ok[-1,0]
    number_samples = np_Excel_Ok.shape[0]
    print("Numer of samples")
    print(number_samples)

    xnew = np.linspace(xmin, xmax, number_samples)
    xold = np.squeeze(np_Excel_to_reshape[:,:1])
    yold = np_Excel_to_reshape[:,1:]
    print(xold.shape)
    print(yold.shape)
    print(yold.shape[1])
    print(yold[:,1].shape)

    ynew = np.zeros(np_Excel_Ok.shape)
    for i in range(yold.shape[1]):
        function_fx = sp.interpolate.interp1d(xold,yold[:,i],kind = 'nearest')
        yaux = function_fx(xnew)
        print(yaux)
        time.sleep(10)
        ynew[:,i] = np.transpose(yaux).copy()

    #convert to DF
    columns = list(DF_Excel1.columns)
    column = columns.pop(0)
    DF_Result = pd.DataFrame({})

    for j in range(len(columns)):
        column = columns.pop(0)
        DF_Result[column] = ynew[:,j]

    DF_Result.index = xnew
    DF_Result.to_excel(path_to_save, sheet_name='Sheet1')

if __name__ == '__main__':
    path_Excel_Mujoco = "/home/sergi/GitHub/Mujoco_TFM/Original_Mujoco_Training1_average.xlsx"
    path_Excel_Pybullet = "/home/sergi/GitHub/Simulation_Pybullet/Experiments_Training1/Original_average.xlsx"

    ConvertToSameSamples_excel2d(path_Excel_Mujoco,path_Excel_Pybullet,title = 'converted')

    print ("Done")
