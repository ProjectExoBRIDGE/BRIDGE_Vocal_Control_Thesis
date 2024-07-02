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


###################################################################################################
#DESCRIPTION OF THE CODE

#The codes inport the files with the process times of the libraries:
#1. Whisper
#2. Google
#3. Vosk

#And plot a boxplot with the processing times


####################################################################################################
#IMPORT THE FILES WITH THE TIMES ARRAY

# IMPLEMENTING FUNCTION 'OPEN FILE'
# Input: file_name
# Output: returns the content of a file, this content can be stored in a variable

def openFile(file_name):                        # receiving the file name from which to extract the content

    with open(file_name, 'rb') as file:         # Deserialize and retrieve the variable from the file
        return pickle.load(file)                # return a variable with the content of file loaded

# Calling function

timesWhisper = openFile('timesWhisper')
timesVosk = openFile('timesVosk')
timesGoogle = openFile('timesGoogle')

timesWhisper = np.round(timesWhisper, 3)
timesVosk = np.round(timesVosk, 3)
timesGoogle = np.round(timesGoogle, 3)


#####################################################################################################
# CHECKING IF THE ARRAYS HAVE THE SAME LENGTH
# This check is absolutely necessary in order to have valid statistic tests.

if( len(timesGoogle)==len(timesWhisper) and len(timesGoogle)==len(timesVosk)
    and len(timesVosk) == len(timesWhisper)):                   # if they all have the same length

    print("All arrays with the same length", len(timesVosk))

else:                   # if not...

    print("ARRAYS WITH DIFFERENT LENGTH")
    print("Length of timesGoogle", len(timesGoogle))
    print("Length of timesVosk", len(timesVosk))
    print("Length of timesWhisper", len(timesWhisper))
    sys.exit()         # exit the program



#####################################################################################################
# CREATING THREE ARRAYS WITH ALL THE VALID TIMES, PAYING ATTENTION TO RESPECT THE
# ORDER AND POSITIONAL CORRESPONDENCE BETWEEN THE FILES

dim = len(timesVosk)

google = []
whisper = []
vosk = []


for i in range(dim):                # Setting all the times in position-i

    # CHECKING IF THE TIMES OF THE FILE-i IS VALID FOR ALL THE ARRAYS ( != - 100)

    if(timesVosk[i]!=-100 and timesGoogle[i]!=-100 and timesWhisper[i]!=-100 ): # if it is...

        vosk.append(timesVosk[i])               # adding timeVosk-i in vosk array
        google.append(timesGoogle[i])           # adding timeGoogle-i in google array
        whisper.append(timesWhisper[i])         # adding timeWhisper-i in whisper array

    else:       # Otherwise, if at least one of the times is not valid (-100), don't add anyone of them...

        print("The file in position", i, "has been discarded for time analysis")


# The previous instruction could have built lists: we now convert them into arrays
google = np.array(google)
whisper = np.array(whisper)
vosk = np.array(vosk)



#####################################################################################################
# GENERATING HISTOGRAM WITH TIMES

plt.figure()

plt.hist(whisper,bins=200,density ='True')
plt.grid()
plt.xlim([0,2])

plt.title('Whisper Time')

plt.xlabel('Processing times [s]')
plt.ylabel('Density')

plt.show()


plt.figure()

plt.hist(vosk,bins=200,density ='True')
plt.grid()
plt.xlim([0,2])

plt.title('Vosk Time')

plt.xlabel('Processing times [s]')
plt.ylabel('Density')

plt.show()




plt.figure()

plt.hist(google,bins=200,density ='True')
plt.grid()
plt.xlim([0,2])

plt.title('Google Time')

plt.xlabel('Processing times [s]')
plt.ylabel('Density')

plt.show()



#####################################################################################################
# NORMAL DISTRIBUTION TEST

print("SHAPIRO NORMAL TEST ON TIMES:")
print("Normal test on timesWhisper", shapiro(whisper) )
print("Normal test on timesGoogle", shapiro(google) )
print("Normal test on timesVosk", shapiro(vosk) )

#####################################################################################################
# IMPLEMENTING THE QQ PLOT


sm.qqplot(whisper, line='45')
plt.title('QQ plot TimeWhisper')
py.show()

sm.qqplot(vosk, line='45' )
plt.title('QQ plot TimeVosk')
py.show()

sm.qqplot(google, line='45')
plt.title('QQ plot TimeGoogle')
py.show()


#####################################################################################################
#GENERATING BOXPLOT

plt.figure()
plt.title('Boxplot times')
plt.grid()

labels= ['Vosk', 'Whisper', 'Google']

plt.boxplot([vosk,whisper,google], labels=labels)
plt.ylim([0, 2])

plt.ylabel('Times [s]')
plt.show()


#####################################################################################################
#CALCULATING MEDIAN AND IQR


median_google = median(google)
median_vosk = median(vosk)
median_whisper = median(whisper)


q3_google, q1_google = np.percentile(google, [75 ,25])
q3_vosk, q1_vosk = np.percentile(vosk, [75 ,25])
q3_whisper, q1_whisper = np.percentile(whisper, [75 ,25])


print("-----------------------------------------------------------------------")

print("MEDIAN AND IQR")
print("median_google = ", median_google, "q1 = ", q1_google, "q3 = ", q3_google )
print("median_vosk = ", median_vosk, "q1 = ", q1_vosk, "q3 = ", q3_vosk)
print("median_whisper = ", median_whisper, "q1 = ", q1_whisper, "q3 = ", q3_whisper)



#####################################################################################################
# IMPLEMENTING THE FRIEDMAN TEST


print("-----------------------------------------------------------------------")

print("FRIEDMAN TEST ON TIMES: PAIRED SAMPLES, NOT NORMAL DISTRIBUTION")
print("H0: the column effects are all equal\n"
      "H1: that they are not all the same.")


print(friedmanchisquare(vosk,whisper,google))


#####################################################################################################
# IMPLEMENTING THE WILCOXON TEST


pval = []

print("-----------------------------------------------------------------------")
print("WILCOXON TEST ON TIMES: PAIRED SAMPLES, NOT NORMAL DISTRIBUTION")

print("H0: the two related paired samples come from the same distribution.\n"
      "In particular, it tests whether the distribution of the differences x - y is symmetric about zero. \n"
      "It is a non-parametric version of the paired T-test.")

print("°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°")
print("Google VS Vosk ")
result = wilcoxon(google,vosk)
print("Statistic = ", (result.statistic))
print("pvalue = ", (result.pvalue))
pval.append(result.pvalue)


print("°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°")
print("Whisper VS Vosk ")
result = wilcoxon(whisper,vosk)
print("Statistic = ", (result.statistic))
print("pvalue = ", (result.pvalue))
pval.append(result.pvalue)

print("°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°")
print("Whisper VS Google ")
result = wilcoxon(whisper,google)
print("Statistic = ", (result.statistic))
print("pvalue = ", (result.pvalue))
pval.append(result.pvalue)


#####################################################################################################
# BONFERRONI CORRECTION

print("-----------------------------------------------------------------------")
print("BONFERRONI CORRECTION ON WILCOXON P VALUES")

print("pvalues = ", pval)
p_adjusted = multipletests(pval, alpha=0.05, method='bonferroni')

print(p_adjusted)


#####################################################################################################
# IMPLEMENTING THE BLAND-ALTMAN

#create Bland-Altman plot: Whisper VS Vosk

f, ax = plt.subplots(3,figsize=(10,10))

sm.graphics.mean_diff_plot( whisper, vosk, ax = ax[0])
ax[0].set_xlim([0, 7])
ax[0].set_title("Bland-Altman: Whisper VS Vosk")



#create Bland-Altman plot: Whisper VS Google

sm.graphics.mean_diff_plot( whisper , google, ax = ax[1])
ax[1].set_xlim([0, 7])
ax[1].set_title("Bland-Altman: Whisper VS Google")


#create Bland-Altman plot: Google VS Vosk


sm.graphics.mean_diff_plot( google , vosk, ax = ax[2])
ax[2].set_xlim([0, 7])
ax[2].set_title("Bland-Altman: Google VS Vosk")
plt.show()


