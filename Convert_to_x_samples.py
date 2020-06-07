import numpy as np
import scipy as sp
import scipy.interpolate
import pandas as pd
import time

def ConvertToXSamples_excel2d(path_Excel_1,title = '',number_of_samples = 40):
    DF_Excel1 = pd.read_excel(path_Excel_1)


    np_Excel1 = DF_Excel1.to_numpy()
    path_to_save = path_Excel_1.strip('.xlsx')+title+'.xlsx'


    xmin = np_Excel1[0,0]
    xmax = np_Excel1[-1,0]
    number_samples = number_of_samples
    print("Numer of samples")
    print(number_samples)

    xnew = np.linspace(xmin, xmax, number_samples)
    xold = np.squeeze(np_Excel1[:,:1])
    yold = np_Excel1[:,1:]
    print(xold.shape)
    print(yold.shape)
    print(yold.shape[1])
    print(yold[:,1].shape)

    ynew = np.zeros([number_samples,np_Excel1.shape[1]])
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
    path_Excel_Mujoco = "/home/sergi/GitHubRishabh/gen3-mujoco_mod/Joint_Trajectori_19.xlsx"

    ConvertToXSamples_excel2d(path_Excel_Mujoco,title = 'converted',number_of_samples = 40)

    print ("Done")
