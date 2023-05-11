#! /usr/bin/env python2

# ---------  Utility function of all the publisher scripts ------------
# Author:			Foad Hajiaghajani, Yuyang Bai
# Email:			foadhaji[at]buffalo.edu
# Institution:			University at Buffalo SUNY
# Last Modified:		Mar 2022
# ---------------------------------------------------------------------
import rospy
import cv2
import numpy as np
import io
from PIL import Image as Im
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vaisala_msgs.msg import UB_weather_info
from vaisala_msgs.msg import Full_weather_info
import requests as req
import xml.etree.ElementTree as xml
import pytz
from datetime import datetime

# ------------- generating url based on Ros Launch configuration -------------
# output:	The target url based on launch file configuration
# ----------------------------------------------------------------------------
def get_url():
	url = "https://exportdb.vaisala.io/export?"
	username,password,station,region,cam,earliesttime,latesttime,_,_ = parse_launch_file()

	#check the availablity of the configuration
	if username is None:
		raise Exception("username required")
	if password is None:
		raise Exception("password required")
	if station is None and region is None:
		raise Exception("station number or region number required")

	#adding configuration to url
	url = url + "username=" + username
	url = url + "&password=" + password
	url = url + "&station=" + station
	if region is not None:
		url = url + "&region=" + region
	if cam is not None:
		url = url + "&cam=" + cam
	if earliesttime is not None:
		url = url + "&earliesttime=" + earliesttime
	if latesttime is not None:
		url = url + "&latesttime=" + latesttime
	return url

# ------------- generating image url based on Ros Launch configuration -------------
# output:	The target url based on launch file configuration
# ----------------------------------------------------------------------------------
def get_image_url():
	url = "https://exportdb.vaisala.io/export/image.jpg?"
	username,password,station,region,cam,earliesttime,latesttime,_,_ = parse_launch_file()

	#check the availablity of the configuration
	if username is None:
		raise Exception("username required")
	if password is None:
		raise Exception("password required")
	if station is None and region is None:
		raise Exception("station number or region number required")

	#adding configuration to url
	url = url + "username=" + username
	url = url + "&password=" + password
	url = url + "&station=" + station
	if region is not None:
		url = url + "&region=" + region
	if cam is not None:
		url = url + "&cam=" + cam
	if earliesttime is not None:
		url = url + "&earliesttime=" + earliesttime
	if latesttime is not None:
		url = url + "&latesttime=" + latesttime
	return url

# -------------------- parse the ROS Launch file variable ----------------------
# output:		All the possible variables in the launch file
# explanation:	The no value optional variables in launch file will set as None
# ------------------------------------------------------------------------------
def parse_launch_file():
	username = rospy.get_param("/username")
	password = rospy.get_param('/password')
	station = rospy.get_param('/station')
	try:
		region = rospy.get_param('/region')
	except KeyError:
		region = None
	try:
		cam = rospy.get_param('/cam')
	except KeyError:
		cam = None
	try:
		earliesttime = rospy.get_param('/earliesttime')
	except KeyError:
		earliesttime = None
	try:
		latesttime = rospy.get_param('/latesttime')
	except KeyError:
		latesttime = None
	try:
		request_time = rospy.get_param('/request_time')
	except KeyError:
		request_time = None
	try:
		publish_time = rospy.get_param('/publish_time')
	except KeyError:
		publish_time = None
	return username,password,station,region,cam,earliesttime,latesttime,request_time,publish_time

# ---------- generate full_weather_info.msg file based on the request url ----------
# input:	The request url
# output:	The full_weather_info.msg contained with the information from url
# ----------------------------------------------------------------------------------
def generate_full_weather_msg(url):
	info = Full_weather_info()
	response = req.get(url)
	tree = xml.fromstring(response.content)
	result = tree[0][1]
	timestamp = result.attrib.get("timestamp")
	info.timestamp = UTC_to_EST(timestamp)
	iterator = {}
	for child in result:
		name = child.attrib.get("code")
		value = child.text
		iterator[name] = value
	key = []
	for k, v in iterator.items():
		key.append(k)
	if 'AL' in key:
		info.AL = int(float(iterator['AL']))
	if 'BI' in key:
		info.BI = float(iterator['BI'])
	if 'BT' in key:
		info.BT = float(iterator['BT'])
	if 'CF' in key:
		info.CF = float(iterator['CF'])
	if 'CL' in key:
		info.CL = int(float(iterator['CL']))
	if 'CLCHn' in key:
		info.CLCHn = float(iterator['CLCHn'])
	if 'CLSCn' in key:
		info.CLSCn = float(iterator['CLSCn'])
	if 'CN' in key:
		info.CN = float(iterator['CN'])
	if 'CS' in key:
		info.CS = float(iterator['CS'])
	if 'DA' in key:
		info.DA = float(iterator['DA'])
	if 'FDS' in key:
		info.FDS = int(float(iterator['FDS']))
	if 'FR' in key:
		info.FR = float(iterator['FR'])
	if 'GE' in key:
		info.GE = float(iterator['GE'])
	if 'HCS' in key:
		info.HCS = int(float(iterator['HCS']))
	if 'IL' in key:
		info.IL = float(iterator['IL'])
	if 'MST' in key:
		info.MST = int(float(iterator['MST']))
	if 'P' in key:
		info.P = float(iterator['P'])
	if 'PR' in key:
		info.PR = float(iterator['PR'])
	if 'PRnH' in key:
		info.PRnH = float(iterator['PRnH'])
	if 'PW' in key:
		info.PW = int(float(iterator['PW']))
	if 'RA' in key:
		info.RA = float(iterator['RA'])
	if 'RD' in key:
		info.RD = int(float(iterator['RD']))
	if 'RH' in key:
		info.RH = float(iterator['RH'])
	if 'RI' in key:
		info.RI = float(iterator['RI'])
	if 'RS' in key:
		info.RS = int(float(iterator['RS']))
	if 'SH' in key:
		info.SH = float(iterator['SH'])
	if 'SL' in key:
		info.SL = float(iterator['SL'])
	if 'SM' in key:
		info.SM = float(iterator['SM'])
	if 'SS' in key:
		info.SS = float(iterator['SS'])
	if 'ST' in key:
		info.ST = int(float(iterator['ST']))
	if ' T' in key:
		info.T = float(iterator['T'])
	if 'TB' in key:
		info.TB = float(iterator['TB'])
	if 'TD' in key:
		info.TD = float(iterator['TD'])
	if 'TF' in key:
		info.TF = float(iterator['TF'])
	if 'TG' in key:
		info.TG = float(iterator['TG'])
	if 'TR' in key:
		info.TR = float(iterator['TR'])
	if 'TS' in key:
		info.TS = float(iterator['TS'])
	if 'VI' in key:
		info.VI = float(iterator['VI'])
	if 'WAC' in key:
		info.WAC = float(iterator['WAC'])
	if 'WD' in key:
		info.WD = float(iterator['WD'])
	if 'WDM' in key:
		info.WDM = float(iterator['WDM'])
	if 'WL' in key:
		info.WL = float(iterator['WL'])
	if 'WS' in key:
		info.WS = float(iterator['WS'])
	if 'WSM' in key:
		info.WSM = float(iterator['WSM'])
	if 'WT' in key:
		info.WT = float(iterator['WT'])
	return info

# ----------- generate ub_weather_info.msg file based on the request url -----------
# input:	The request url
# output:	The ub_weather_info.msg contained with the information from url
# ----------------------------------------------------------------------------------
def generate_UB_weather_msg(url):
	info = UB_weather_info()
	response = req.get(url)
	tree = xml.fromstring(response.content)
	result = tree[0][1]
	timestamp = result.attrib.get("timestamp")
	info.timestamp = UTC_to_EST(timestamp)
	iterator = {}
	for child in result:
		name = child.attrib.get("code")
		value = child.text
		iterator[name] = value
	info.AL = int(float(iterator['AL']))
	info.FR = float(iterator['FR'])
	info.IL = float(iterator['IL'])
	info.PR1H = float(iterator['PRA1H'])
	info.PR3H = float(iterator['PRA3H'])
	info.PR6H = float(iterator['PRA6H'])
	info.PR12H = float(iterator['PRA12H'])
	info.PR24H = float(iterator['PRA24H'])
	info.RD = int(float(iterator['RD']))
	info.RH = float(iterator['RH'])
	info.RI = float(iterator['RI'])
	info.RS = int(float(iterator['RS']))
	info.SL = float(iterator['SL'])
	info.ST = int(float(iterator['ST']))
	info.T = float(iterator['T'])
	info.TS = float(iterator['TS'])
	info.VI = float(iterator['VI'])
	info.WD = float(iterator['WD'])
	info.WDM = float(iterator['WDM'])
	info.WL = float(iterator['WL'])
	info.WS = float(iterator['WS'])
	info.WSM = float(iterator['WSM'])
	return info

# ---------------- get image from the url and store into Image.msg ----------------
# input:	The request url
# output:	The Image.msg contained with the image from url
# ---------------------------------------------------------------------------------
def get_image(url):
	br = CvBridge()
	response = req.get(url)
	bytes_image = io.BytesIO(response.content)
	image = np.array(Im.open(bytes_image))
	return br.cv2_to_imgmsg(image)

# --------------------- Convert UTC timestamp to EST timestamp ---------------------
# input:	Timestamp in format of "YY-mm-dd HH:MM:SS"
# output:	Corresponding EST timestamp
# ----------------------------------------------------------------------------------
def UTC_to_EST(timestamp):
	Date, Time = timestamp.split()
	Y, m, d = Date.split("-")
	H, M, S = Time.split(":")
	UTC = pytz.utc
	EST = pytz.timezone("US/Eastern")
	fmt = "%Y-%m-%d %H:%M:%S"
	UTC_time = datetime(int(Y), int(m), int(d), int(H), int(M), int(S), tzinfo=UTC)
	EST_time = UTC_time.astimezone(EST).strftime(fmt)
	return EST_time
