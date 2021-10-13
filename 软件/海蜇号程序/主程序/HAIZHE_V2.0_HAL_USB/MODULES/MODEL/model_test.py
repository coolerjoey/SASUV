#coding:utf-8
import numpy as np
import matplotlib.pyplot as plt
import pylab

def print_model_info():
	x=0
	y=1
	second=8
	length=second*100

	'''
	0-time
	1-ax
	2-ay
	3-az
	4-vx
	5-vy
	6-vz
	7-px
	8-py
	9-pz
	'''

	data = np.loadtxt('model_test_data.txt')
	x_ref=data[0:length,x]
	y_ref=data[0:length,y]
	x_dr=data[length+1:length*2,x]
	y_dr=data[length+1:length*2:,y]
	x_kf=data[length*2+1:,x]
	y_kf=data[length*2+1:,y]
	plt.plot(x_ref,y_ref,'r',x_dr,y_dr,'b',x_kf,y_kf,'g')
	#plt.plot(x_ref,y_ref,'r')
	plt.show()
print_model_info()



