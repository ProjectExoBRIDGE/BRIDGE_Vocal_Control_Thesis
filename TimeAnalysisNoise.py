#BOX PLOT OF WAITING TIMES

#Lattuada Valentina
#Mazzani Stefano
#Merlo Francesco
#Meroni Francesco



import speech_recognition as sr
from difflib import get_close_matches
import time
import pyaudio
import wave
import pickle
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import scipy
from scipy.stats import friedmanchisquare
from scipy.stats import wilcoxon
import statsmodels.api as sm
import pylab as py
from statistics import median
from scipy.stats import norm
from scipy.stats import normaltest
from scipy.stats import shapiro
import sys
from statsmodels.sandbox.stats.multicomp import multipletests



#IMPORT THE FILES WITH THE TIMES ARRAY

# IMPLEMENTING FUNCTION 'OPEN FILE'
# Input: file_name
# Output: returns the content of a file, this content can be stored in a variable

def openFile(file_name):                        # receiving the file name from which to extract the content

    with open(file_name, 'rb') as file:         # Deserialize and retrieve the variable from the file
        return pickle.load(file)                # return a variable with the content of file loaded

# Calling function

timesNotAdjusted = openFile('VoskTimes_noise0_#session3_adjusted0')
timesAdjusted= openFile('VoskTimes_noise0_#session3_adjusted1')

timesNotAdjusted = np.round(timesNotAdjusted, 3)
timesAdjusted = np.round(timesAdjusted, 3)

#####################################################################################################
# CHECKING IF THE ARRAYS HAVE THE SAME LENGTH
# This check is absolutely necessary in order to have valid statistic tests.

if( len(timesAdjusted)==len(timesNotAdjusted)):                   # if they all have the same length

    print("All arrays with the same length", len(timesAdjusted))

else:                   # if not...

    print("ARRAYS WITH DIFFERENT LENGTH")
    print("Length of timesAdjusted", len(timesAdjusted))
    print("Length of timesNotAdjusted", len(timesNotAdjusted))
    sys.exit()         # exit the program



#####################################################################################################
# CREATING THREE ARRAYS WITH ALL THE VALID TIMES, PAYING ATTENTION TO RESPECT THE
# ORDER AND POSITIONAL CORRESPONDENCE BETWEEN THE FILES

dim = len(timesAdjusted)

adjusted = []
notAdjusted = []


for i in range(dim):                # Setting all the times in position-i

    # CHECKING IF THE TIMES OF THE FILE-i IS VALID FOR ALL THE ARRAYS ( != - 100)

    if(timesAdjusted[i]!=-100 and timesNotAdjusted[i]!=-100 ): # if it is...

        adjusted.append(timesAdjusted[i])               # adding timeVosk-i in vosk array
        notAdjusted.append(timesNotAdjusted[i])         # adding timeGoogle-i in google array


    else:       # Otherwise, if at least one of the times is not valid (-100), don't add anyone of them...

        print("The file in position", i, "has been discarded for time analysis")


# The previous instruction could have built lists: we now convert them into arrays
adjusted = np.array(adjusted)
notAdjusted = np.array(notAdjusted)


#####################################################################################################
#GENERATING BOXPLOT

plt.figure()
plt.title('Boxplot times')
plt.grid()

labels= ['adjusted', 'notAdjusted']

plt.boxplot([adjusted,notAdjusted], labels=labels)
plt.ylim([0, 1])

plt.ylabel('Times [s]')
plt.show()






