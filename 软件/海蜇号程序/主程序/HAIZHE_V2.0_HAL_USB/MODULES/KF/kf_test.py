#coding:utf-8
import numpy as np
import matplotlib.pyplot as plt
import pylab

def print_kf_info():
	x=3
	y=7
	second=16
	length=second*100

	'''
	0-time
	1-ax
	2-vx
	3-dx
	4-px
	5-ay
	6-vy
	7-dy
	8-py
	9-dvx
	10-dvy
	11-dpx
	12-dpy
	'''

	data = np.loadtxt('data.txt')
	x_ref=data[0:length,x]
	y_ref=data[0:length,y]
	x_dr=data[length+1:length*2,x]
	y_dr=data[length+1:length*2:,y]
	x_kf=data[length*2+1:,x]
	y_kf=data[length*2+1:,y]
	plt.plot(x_ref,y_ref,'r',x_dr,y_dr,'b',x_kf,y_kf,'g')
	#plt.plot(x_ref,y_ref,'r')
	plt.show()
def print_kf_predict():
	x=0
	y=5
	second=16

	'''
	0-time
	1-vx
	2-vy
	3-vz
	4-px
	5-py
	6-pz
	'''

	data = np.loadtxt('data.txt')
	ref1=data[0:second+1,x]
	ref2=data[0:second+1,y]
	plt.plot(ref1,ref2,'r')
	plt.show()

print_kf_info()



