import numpy as np
import scipy.io
from scipy.signal import butter, lfilter, freqz ,filtfilt
#import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)
 

def steeringVector(xPos,yPos,zPos,f,c,thetaScanAngles,phiScanAngles):
    if len(xPos)==0:
        #raise ValueError
        assert False, '錯,請檢察input data'
    if len(yPos)==0:
        #raise ValueError
        assert False, '錯,請檢察input data'

    thetaScanAngles = thetaScanAngles*np.pi/180
    phiScanAngles = phiScanAngles*np.pi/180
    k = 2*np.pi*f/c
    P = len(xPos)
    if len(thetaScanAngles) > 1:
        N = len(phiScanAngles)

        u = np.sin(thetaScanAngles).reshape(np.sin(thetaScanAngles).shape[0],1) * np.cos(phiScanAngles)

        v = np.sin(thetaScanAngles).reshape(np.sin(thetaScanAngles).shape[0],1) * np.sin(phiScanAngles)

        w = np.tile(np.cos(thetaScanAngles),(181,1)).T
    else:
        u = np.sin(thetaScanAngles).T @ np.cos(phiScanAngles)
        v = np.sin(thetaScanAngles).T @ np.sin(phiScanAngles)
        w = np.cos(thetaScanAngles)
    def bsxfun(pos,vec):
        res = np.empty([np.shape(vec)[0],np.shape(vec)[1],len(pos)])
        j = 0
        for i in pos:
            res[:,:,j] = vec * i
            j += 1 
        return res

    uu = bsxfun(xPos,u)
    vv = bsxfun(yPos,v)
    ww = bsxfun(zPos,w)    
    #uu = np.reshape(xPos,(1,1,P))*u
    #vv = np.reshape(yPos,(1,1,P))*v
    #ww = np.reshape(zPos,(1,1,P))*w
    
    e = np.exp(1j*k*(uu+vv+ww))
    #print(uu[0,0,1])
    #print(vv[:,:,1])
    #print((uu+vv+ww)[0,0,1])


    return e,u,v,w


def crossSpectralMatrix(inputsignal):
    c = inputsignal @ inputsignal.conjugate().T
    R = c / np.shape(inputsignal)[1]
    return R 


def steeredResponseMUSIC(R,e,w,N,M):
    nPointsY = np.shape(e)[0]
    nPointsX = np.shape(e)[1]
    nMics = np.shape(e)[2]
    D = np.linalg.eig(R)[0]
    #print(D)
    Ev = np.linalg.eig(R)[1]
    #print(Ev)
    EVA = np.sort(D)
    I = np.argsort(D)
    EV = np.flip(Ev[:,I],1)
    En = EV[:,M:]
    SMUSIC = np.zeros([nPointsY,nPointsX])
    for i in range(nPointsY):
        for j in range(nPointsX):

            ee = np.reshape(e[i,j,:],(nMics,1))
            SMUSIC[i,j] = (1 / (ee.T.conjugate() @ En @ En.T.conjugate() @ ee))[0][0]
    
   

    return SMUSIC

def xcorr(a,b):
    norm_a = np.linalg.norm(a)
    a = a / norm_a
    norm_b = np.linalg.norm(b)
    b = b / norm_b
    c = np.correlate(a, b, mode = 'full')
    return c

def audio_filt(y):
   #以右邊為第一 逆時鐘增加 
    y_A_1 = y.T[:,1] #調整麥克風位置
    y_A_2 = y.T[:,3]
    y_B_1 = y.T[:,0]
    y_B_2 = y.T[:,2]

    c_A12 = xcorr(y_A_1, y_A_2)
    c_B12 = xcorr(y_B_1, y_B_2)
    c_total = np.vstack((np.abs(c_A12) , np.abs(c_B12)))
    c_total = c_total.T
    c_local = np.zeros([1,2])

    #print(np.shape(c_total))
    for i in range(2):
        c_local[0,i] = np.argmax(c_total[:,i]) + 1
    
    mid = (np.shape(c_total)[0] - 1) / 2
    corr_diff = mid - c_local
    

    return corr_diff

def responsePeak(spec , u , v , corr_diff):
    m = np.max(spec)
    x = np.where(spec == m)
    #print(len(x[0]))
    #print(corr_diff)
    if len(x[0])>1:
        p1 = [u[x[0][0],x[1][0]],v[x[0][0],x[1][0]]]
        p2 = [u[x[0][1],x[1][1]],v[x[0][1],x[1][1]]]
        if corr_diff[0] > 0:
            if p1[1] < p2[1] :
                ruy = p2[0]
                rvy = p2[1]
            else:
                ruy = p1[0]
                rvy = p1[1]

        else:
            if p1[1] < p2[1] :
                ruy = p1[0]
                rvy = p1[1]
            else:
                ruy = p2[0]
                rvy = p2[1]

        if corr_diff[1] > 0:
            if p1[0] < p2[0]:
                rux =  p2[0]
                rvx =  p2[1]
            else:
                rux =  p1[0]
                rvx =  p1[1]
        else:
            if p1[0] < p2[0]:
                rux =  p1[0]
                rvx =  p1[1]
            else:
                rux =  p2[0]
                rvx =  p2[1]
        if rux != ruy and rvx != rvy:
            ru = 0
            rv = 0
        else:
            ru = ruy
            rv = rvy

    elif len(x[0]) == 1:
        xresult = x[0][0]
        yresult = x[1][0]
        ru = u[xresult,yresult]
        rv = v[xresult,yresult]
    
    else:
        ru = 0
        rv = 0
    
    return ru,rv

def angcal(ru,rv):
    if ru != 0 and rv != 0:
        if ru >= 0 and rv >= 0:
            ami = np.arctan(ru/rv)/np.pi*180
            ele = np.arccos((ru**2+rv**2)**(0.5))/np.pi*180

        elif ru >= 0 and rv < 0 :
            ami = np.arctan(np.abs(rv)/np.abs(ru))/np.pi*180+90
            ele = np.arccos((ru**2+rv**2)**(0.5))/np.pi*180
        elif ru < 0 and rv < 0:
            ami = np.arctan(np.abs(ru)/np.abs(rv))/np.pi*180+180
            ele = np.arccos((ru**2+rv**2)**(0.5))/np.pi*180
        else:
            ami = np.arctan(np.abs(rv)/np.abs(ru))/np.pi*180+270
            ele = np.arccos((ru**2+rv**2)**(0.5))/np.pi*180
    else:
        ami = np.nan
        ele = np.nan

    return ami,ele

def butter_bandpass(lowcut, highcut, fs, order=5):
    b, a = butter(order, [lowcut, highcut], fs=fs, btype='band')
    return b, a

def butter_bandpass_filter(data, lowcut, highcut, fs, order=6):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y