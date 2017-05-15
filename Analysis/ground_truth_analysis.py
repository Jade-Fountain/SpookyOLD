import json
import numpy as np

def identifyMarkers(p):
	dp = []
	for i in range(len(p)):
		dp = dp + [np.linalg.norm(p[(i+1) % len(p)] - p[i])]
	upper = 0
	upper_i = 0
	lower = 100000
	lower_i = 0
	for i in range(len(dp)):
		if(dp[i] < lower):
			lower = dp[i]
			lower_i = i
		if(dp[i] > upper):
			upper = dp[i]
			upper_i = i

	middle_i = 0
	for i in range(len(dp)):
		if(i != upper_i or i != lower_i):
			middle_i = i
	return [upper_i, middle_i, lower_i]


def getFusionKitData(file_name):
	file = open(file_name)

	json_string = file.read()
	file.close()

	json_data = json.loads(json_string)

	data = np.zeros([len(json_data), 9])

	for i in range(len(json_data)):
		frame = json_data[i]
		if(len(frame["Markers"]) == 3):
			p = [np.zeros(3),np.zeros(3),np.zeros(3)]
			p[0] = np.array((frame["Markers"][0]["Position"]["X"], frame["Markers"][0]["Position"]["Y"], frame["Markers"][0]["Position"]["Z"]))
			p[1] = np.array((frame["Markers"][1]["Position"]["X"], frame["Markers"][1]["Position"]["Y"], frame["Markers"][1]["Position"]["Z"]))
			p[2] = np.array((frame["Markers"][2]["Position"]["X"], frame["Markers"][2]["Position"]["Y"], frame["Markers"][2]["Position"]["Z"]))

			#todo: add identification
			identity = identifyMarkers(p)

			for j in range(3):
				data[i,3*j:3*j+3] = p[identity[j]]

		# print i,data[i]
	return data

def getOptitrackData(file_name):
	optitrack_data = np.genfromtxt(file_name, delimiter=',')
	return optitrack_data

# print json_data

fkdata = getFusionKitData('calib_attempt1.fkmcp')
opdata = getOptitrackData("optitrack_data.csv")

