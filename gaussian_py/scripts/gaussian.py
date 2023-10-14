#!/usr/bin/env python3
import rospy
from itertools import product
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from gaussian_py.msg import kl_suzu
from gaussian_py.msg import kl_suzuki
from statistics import stdev
import time
import math
from codecs import utf_16_be_encode
from sklearn.feature_selection import mutual_info_classif

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import csv
import pprint
import collections
import itertools
import random
import ast
import pandas as pd

# ガウスカーネル(RBFカーネル)を関数化
def kernel(x: np.array, x_prime: np.array, p, q, r):
    if np.array_equal(x, x_prime) == True:
        delta = 1
    else:
        delta = 0
    return p*np.exp(-1 * np.linalg.norm(x - x_prime)**2 / q) + (r * delta)

# 評価関数の作成
def CompositionEvaluationFunction(predict_mean: np.array, predict_variance: np.array, p, q):  
    u = predict_mean / predict_variance
    U1 = 1 / (1 + np.exp( - p * (u - q)))
    # 構図評価関数の最大値を取得
    mu_max_iti = np.unravel_index(U1.argmax(), U1.shape)
    mu_max = np.amax(U1)
    mu_max_iti_3D = mu_max_iti[0]
    len_mu_3D = len(str(mu_max_iti_3D))
    len_mu_3D_2 = str(mu_max_iti_3D)
    int_mu = int(mu_max_iti_3D)
    if len_mu_3D == 1:
        mu_max_iti_x = 0
        mu_max_iti_y = 0
        mu_max_iti_z = len_mu_3D_2
    if len_mu_3D == 2:
        mu_max_iti_x = 0
        mu_max_iti_y = len_mu_3D_2[-2]
        mu_max_iti_z = len_mu_3D_2[-1]
    if len_mu_3D == 3:
        mu_max_iti_x = len_mu_3D_2[-3]
        mu_max_iti_y = len_mu_3D_2[-2]
        mu_max_iti_z = len_mu_3D_2[-1]
    return U1, mu_max_iti_x, mu_max_iti_y, mu_max_iti_z, int_mu

class RecursiveGaussianProcess():
    def __init__(self, all_data):
        self.arg1 = 10           # 探索範囲の分割数（各軸あたり）
        self.arg2 = 27           # データセットの個数
        self.arg3 = 10*10*10
        # 探索範囲の設定
        self.xrange_min = - 0.75
        self.xrange_max = 0.75
        self.yrange_min = - 0.75
        self.yrange_max = 0.75
        self.zrange_min = 1.00
        self.zrange_max = 1.50
        # # 観測データの取得
        self.All_Data = all_data
        # 固定するデータセット位置の設定
        ###################################################################################################################################
        # 3次元配列で表現したいけど、計算がわけわからなくなるから一旦保留    
        # Data_set = np.array([[[0, 0, 0], [5, 0, 0], [9, 0, 0], [0, 0, 5], [5, 0, 5], [9, 0, 5], [0, 0, 9], [5, 0, 9], [9, 0, 9]], 
        #                 [[0, 5, 0], [5, 5, 0], [9, 5, 0], [0, 5, 5], [5, 5, 5], [9, 5, 5], [0, 5, 9], [5, 5, 9], [9, 5, 9]],
        #                 [[0, 9, 0], [5, 9, 0], [9, 9, 0], [0, 9, 5], [5, 9, 5], [9, 9, 5], [0, 9, 9], [5, 9, 9], [9, 9, 9]]])
        ###################################################################################################################################
        # 2次元配列で表現していく（座標など注意）
        self.Data_set = np.array([[0, 0, 0], [5, 0, 0], [9, 0, 0], [0, 0, 5], [5, 0, 5], [9, 0, 5], [0, 0, 9], [5, 0, 9], [9, 0, 9], 
                        [0, 5, 0], [5, 5, 0], [9, 5, 0], [0, 5, 5], [5, 5, 5], [9, 5, 5], [0, 5, 9], [5, 5, 9], [9, 5, 9],
                        [0, 9, 0], [5, 9, 0], [9, 9, 0], [0, 9, 5], [5, 9, 5], [9, 9, 5], [0, 9, 9], [5, 9, 9], [9, 9, 9]])
        # カーネル関数のハイパーパラメータ
        self.Theta_1 = 1
        self.Theta_2 = 1
        self.Theta_3 = 0.1

        # 探索範囲の設定
        ####################################################################
        # 探索範囲をarg1 = 10 等分に分割
        self.rangex = np.linspace(self.xrange_min, self.xrange_max, self.arg1)
        self.rangey = np.linspace(self.yrange_min, self.yrange_max, self.arg1)
        self.rangez = np.linspace(self.zrange_min, self.zrange_max, self.arg1)
        # メッシュ構造化
        self.Range_x, self.Range_y, self.Range_z = np.meshgrid(self.rangex, self.rangey, self.rangez)
        self.Range = np.stack([self.Range_x, self.Range_y, self.Range_z], 3)
        # 簡略化のために2次元配列化
        self.Range_xyz = self.Range.reshape([self.arg3, 3]) 

        # データセットの値
        self.Training_x = self.rangex[self.Data_set[:, 0]]
        self.Training_y = self.rangey[self.Data_set[:, 1]]
        self.Training_z = self.rangez[self.Data_set[:, 2]]
        self.Training_Data = np.c_[self.Training_x, self.Training_y, self.Training_z]
        
        self.sd = 0
        self.muob = 0 

        ############################################################################################
        # データ数
        self.data_size = len(all_data.KL)
        # print('self.data_size ', self.data_size)
        rospy.loginfo('data_size %d', self.data_size)
        # 更新回数
        self.Update_Step = self.data_size
        # 不変なカーネル行列の計算
        if self.Update_Step == 1 or self.Update_Step == 0:
            # print('Yessss!')
            # データセットのカーネル行列 K 
            self.K = np.zeros((self.arg2, self.arg2)) # np.zeros((2,4)):2x4の2次元配列を生成
            for x in range(self.arg2):  #range(stop): 指定した開始数から終了数までの連続した数値を要素として持つrenge型のオブジェクトを生成する．
                for x_prime in range(self.arg2):
                    self.K[x, x_prime] = kernel(self.Training_Data[x], self.Training_Data[x_prime], self.Theta_1, self.Theta_2, self.Theta_3)
            # Kの逆行列Kinv
            self.Kinv = np.linalg.inv(self.K)
            np.savetxt('/home/dars/bebop_ws/src/drone_data/K.csv', self.K, fmt='%f')
            np.savetxt('/home/dars/bebop_ws/src/drone_data/Kinv.csv', self.Kinv, fmt='%f')
            # データセットと未観測領域のカーネル行列 k
            self.k = np.zeros((self.arg2, self.arg3))
            for x_test1 in range(self.arg2):
                for x in range(self.arg3):
                    self.k[x_test1, x] = kernel(self.Training_Data[x_test1], self.Range_xyz[x], self.Theta_1, self.Theta_2, self.Theta_3)
            np.savetxt('/home/dars/bebop_ws/src/drone_data/k_kernel.csv', self.k, fmt='%f')
            # 未観測領域のカーネル行列 s
            self.s = np.zeros((self.arg3, 1))
            for x_test1 in range(self.arg3):
                self.s[x_test1] = kernel(self.Range_xyz[x_test1], self.Range_xyz[x_test1], self.Theta_1, self.Theta_2, self.Theta_3)
            np.savetxt('/home/dars/bebop_ws/src/drone_data/s.csv', self.s, fmt='%f')

            # 事前分布 mu_f, C_f
            self.mu_f = np.zeros((27, 1))
            self.C_f = self.K
        else:
            # print('Noooo!')
            self.K = np.loadtxt('/home/dars/bebop_ws/src/drone_data/K.csv')
            self.Kinv = np.loadtxt('/home/dars/bebop_ws/src/drone_data/Kinv.csv')
            self.k = np.loadtxt('/home/dars/bebop_ws/src/drone_data/k_kernel.csv')
            self.s = np.loadtxt('/home/dars/bebop_ws/src/drone_data/s.csv')
            self.mu_f = np.loadtxt('/home/dars/bebop_ws/src/drone_data/mu_f_after.csv')
            self.mu_f = np.array([self.mu_f]).T # 配列の設定的に必要
            self.C_f = np.loadtxt('/home/dars/bebop_ws/src/drone_data/C_f_after.csv')
            
    #########################################################################
    # 予測分布の計算 mu_p, C_p
    def PriorDistribution(self):
        J = self.k.T @ self.Kinv
        B = np.zeros((self.arg3, 1))
        for t in range(self.arg3):
            B[t] = self.s[t] - J[t] @ self.k[:, t]
        mu_p = J @ self.mu_f
        C_p = np.zeros((self.arg3, 1))
        for t in range(self.arg3):
            C_p[t] = B[t] + J[t] @ self.C_f @ J.T[:, t]
        return mu_p, C_p, J
    ##########################################################
    # 事後分布を計算（次の更新の事前分布）mu_f_after, C_f_after
    def PosteriorDistribution(self, mup, Cp, j):
        # 新しい観測地点の情報を読み取る
        New_Data_x = self.All_Data.KL[self.Update_Step - 1].kl_x
        New_Data_y = self.All_Data.KL[self.Update_Step - 1].kl_y
        New_Data_z = self.All_Data.KL[self.Update_Step - 1].kl_z
        New_Data_value = self.All_Data.KL[self.Update_Step - 1].kl_score
        # print('New_Data_x', New_Data_x)
        # print('New_Data_y', New_Data_y)
        # print('New_Data_z', New_Data_z)
        # print('New_Data_value', New_Data_value)
        minx = []
        miny = []
        minz = []
        for t in range(10):
            d1 = abs(New_Data_x - self.rangex[t])
            minx.append(d1)
            d2 = abs(New_Data_y - self.rangey[t])
            miny.append(d2)
            d3 = abs(New_Data_z - self.rangez[t])
            minz.append(d3)
        min_x = minx.index(min(minx))
        min_y = miny.index(min(miny))
        min_z = minz.index(min(minz))
        place = 100 * min_x + 10 * min_y + 1 * min_z
        ###############################################
        # 得た観測結果から事後分布を求める
        mu_f_before = self.mu_f
        C_f_before = self.C_f
        JJ = np.array([j[place]])
        G = C_f_before @ JJ.T / Cp[place]
        mu_f_after = mu_f_before + G * (New_Data_value - mup[place])
        C_f_after = C_f_before - G @ JJ @ C_f_before
        np.savetxt('/home/dars/bebop_ws/src/drone_data/mu_f_after.csv', mu_f_after, fmt='%f')
        np.savetxt('/home/dars/bebop_ws/src/drone_data/C_f_after.csv', C_f_after, fmt='%f')
        sd = Cp[place][0]
        muob = mup[place][0]
        return mu_f_after, C_f_after, sd, muob # これが次の更新の事前分布になる


def klCallback1(msg):
    # print('msg', msg)
    RGP = RecursiveGaussianProcess(msg)
    PRIORDISTRIBUTION = RGP.PriorDistribution()
    POSTERIORDISTRIBUTION = RGP.PosteriorDistribution(PRIORDISTRIBUTION[0], PRIORDISTRIBUTION[1], PRIORDISTRIBUTION[2])
    # 構図評価関数の作成
    ###########################################
    # rospy.loginfo('U_max')
    CEF = CompositionEvaluationFunction(PRIORDISTRIBUTION[0], PRIORDISTRIBUTION[1], p = 0.1, q = 10)
    # 構図評価値の最小値となる位置をsemiに格納する．
    # print('CEF[1]', CEF[1])
    # X = RGP.rangex[int(CEF[1])]
    # Y = RGP.rangey[int(CEF[2])]
    # Z = RGP.rangez[int(CEF[3])]
    # KL = PRIORDISTRIBUTION[0]
    # rospy.loginfo('X: %f', X)
    # KL_good = float(KL[CEF[4]])
    # semi.kl_score = KL_good
    # semi.kl_x = X
    # semi.kl_y = Y
    # semi.kl_z = Z
    # print('semi.kl_score', semi.kl_score)
    # print('semi.kl_x', semi.kl_x)
    # print('semi.kl_y', semi.kl_y)
    # print('semi.kl_z', semi.kl_z)
    # CC = PRIORDISTRIBUTION[1]
    # pri_range = len(PRIORDISTRIBUTION[1])
    #####################################################################
    #　構図評価関数を用いないで最小となる位置
    mu_min_iti = np.unravel_index(PRIORDISTRIBUTION[0].argmax(), PRIORDISTRIBUTION[0].shape)
    mu_min = np.amax(PRIORDISTRIBUTION[0])
    mu_min_iti_3D = mu_min_iti[0]
    len_mu_3D_min = len(str(mu_min_iti_3D))
    len_mu_3D_2_min = str(mu_min_iti_3D)
    int_mu = int(mu_min_iti_3D)
    if len_mu_3D_min == 1:
        mu_min_iti_x = 0
        mu_min_iti_y = 0
        mu_min_iti_z = len_mu_3D_2_min
    if len_mu_3D_min == 2:
        mu_min_iti_x = 0
        mu_min_iti_y = len_mu_3D_2_min[-2]
        mu_min_iti_z = len_mu_3D_2_min[-1]
    if len_mu_3D_min == 3:
        mu_min_iti_x = len_mu_3D_2_min[-3]
        mu_min_iti_y = len_mu_3D_2_min[-2]
        mu_min_iti_z = len_mu_3D_2_min[-1]
    semi.kl_score = mu_min
    semi.kl_x = RGP.rangex[int(mu_min_iti_x)]
    semi.kl_y = RGP.rangey[int(mu_min_iti_y)]
    semi.kl_z = RGP.rangez[int(mu_min_iti_z)]
    # print('semi.kl_score(no CEF)', semi.kl_score)
    # print('semi.kl_x(no CEF)', semi.kl_x)
    # print('semi.kl_y(no CEF)', semi.kl_y)
    # print('semi.kl_z(no CEF)', semi.kl_z)
    


def gauss_generate():
    kl_py_pub = rospy.Publisher('semi_opt', kl_suzu, queue_size = 10)
    rospy.init_node('gauss_generate')
    kl_sub1 = rospy.Subscriber("kl_suzuki", kl_suzuki, klCallback1, queue_size=1)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        kl_py_pub.publish(semi)
         #print(semi.kl_score)
        r.sleep()
        
    # rospy.spin()
if __name__ == '__main__':
    try:
        semi = kl_suzu()
        gauss_generate()
    except rospy.ROSInterruptException: pass


