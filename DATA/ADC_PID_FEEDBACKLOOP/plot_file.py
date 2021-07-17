#!/usr/bin/python3
import matplotlib.pyplot as plt
import glob
import os

globalFileData = {}

def load_file_data(fileName,outputDictionary):
	# Dictionary with conent comprised of headers
	fileData = {}

	with open(fileName) as f:
		for line in f.readlines():
			if("ADCMAX" in line):
				fileData["ADCMAX"] = float(line.strip().split(": ")[1])
				continue
			
			if("kP" in line):
				fileData["kP"] = float(line.strip().split(": ")[1])
				continue
			
			if("kI" in line):
				fileData["kI"] = float(line.strip().split(": ")[1])
				continue
			
			if("kD" in line):
				fileData["kD"] = float(line.strip().split(": ")[1])
				continue
			
			if("MILLIS" in line):
				fileData["HEADERS"] = line.strip().split(",")
				# Setup dataEntries dictionary
				for h in fileData["HEADERS"]:
					fileData[h] = []
				continue

			if(fileData["HEADERS"] != None):
				# This line should be data
				dataPoints = line.strip().split(",")
				for d,h in zip(dataPoints,fileData["HEADERS"]):
					fileData[h].append(float(d))
	
	### Add to the outputDictionary
	outputDictionary[fileName] = fileData

### Load files in the current directory
logFiles = glob.glob("./18*.log")

### Load data
for foundFile in logFiles:
	print("Loading {}".format(foundFile))
	load_file_data(foundFile,globalFileData)

### Plot all files
for file in logFiles:
	data = globalFileData[file]
	plt.plot(data["MILLIS"],data["TARGET"])
	plt.plot(data["MILLIS"],data["RAW"],label="{}".format(file + " RAW"))
	# plt.plot(data["MILLIS"],data["AVERAGE"],label="{}".format(file + " AVG"))


plt.legend()
plt.show()
		
		
