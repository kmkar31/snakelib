import glob
import os
import hebi
import numpy as np
import matplotlib.pyplot as plt


directory = os.path.expanduser('~') + '/Biorobotics/snakelib_v2/src/logs/SnakeLogs'
# Get the latest file in the directory
print(directory)
listOfFiles = glob.glob(directory+'/*')
print(listOfFiles)
file = max(listOfFiles, key=os.path.getctime)
feedback = hebi.GroupFeedback(7)

# Parse the HEBI Log file
logFile = hebi.util.load_log(file)
max_efforts = []

# Get the component-wise max each feedback entry compared to the running max efforts
for x in logFile.feedback_iterate:
    max_efforts.append(x.effort)

    
print(np.shape(max_efforts))
plt.plot(max_efforts)
plt.show()
