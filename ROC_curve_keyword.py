# AUTOMATIC AUDIO ACQUISITION

# Lattuada Valentina
# Mazzani Stefano
# Merlo Francesco
# Meroni Francesco

from difflib import get_close_matches
import numpy as np
import matplotlib.pyplot as plt


##########################################################################################################
# DESCRIPTION OF THE CODE

# The code receives in input:
# 1. An array with all the words analysed
# 2. An array with the gold indicator that indicates if the word is keyword (1) or not (0)
# Then it calculates the sensibility and the specificity of the recognition of the keyword and plot
# the ROC curve

###########################################################################################################
# GENERATING THE ARRAY WITH THE WORDS

# words that are accepted
words_positves = ['comando','commando','comanda','comand','comandato','gommando','comandi','omand','domando',
                  'cotanto','covando','accomando']

# words that are not accepted
words_negatives = ['telecomando','radiocomando','comodo','contento','scusando','passando','colorando','calmando',
                   'arrabiando','chiamando','quando','raccomando']


# concatenate all the words
words = np.concatenate((words_positves, words_negatives))
# generating gold vector: 1 = positive , 0 = negative
gold = np.concatenate((np.ones(len(words_positves)),np.zeros(len(words_negatives))))

#CHECKING IF THE NUMBER OF POSITIVES AND NEGATIVES ARE THE SAME

if(len(words_positves)!=len(words_negatives)):
    print("ERROR: NUMBER OF POSITIVES AND NEGATIVES MUST BE THE SAME!")

positive_gold = sum(gold)                   # number of positives by the gold: gold = 1
negative_gold = len(gold) - positive_gold   # number of negatives by the gold: gold = 0

recognize = np.empty(shape = len(words))        # initializing the recognize array
pace = 0.05                                     # pace of the cutoff

sensitivity = np.empty(shape = int(1/pace))     #initializing sensitivity array
specificity = np.empty(shape = int(1/pace))     #initializing specificity array

cutoff = np.arange(0.0, 1.0, pace)

for k in range(len(cutoff)):                 #setting the cutoff

    for i in range(len(words)):             # setting the word-i

        try:
            get_close_matches(words[i], ['comando'], n=1, cutoff=cutoff[k])[0]
            recognize[i] = 1                # if recognizes the word: recognize[i] = 1
        except:
            recognize[i]= 0                # if not: recognize[i] = 0

    # Counting True Positives (TP) and True Negatives (TN)

    TP = 0
    TN = 0

    for i in range(len(words)):

        if (recognize[i] == 1 and gold[i] == 1):      # if the word is recognized as positive, and it really is...
            TP = TP + 1                               # Add 1 to TP

        if (recognize[i] == 0 and gold[i] == 0):      # if the word is recognized as negative, and it really is...
            TN = TN + 1                               # Add 1 to TN

    sensitivity[k] = TP / positive_gold
    specificity[k]= TN / negative_gold



###########################################################################################################
#PLOTTING THE ROC CURVE


plt.figure(1)
plt.plot(1-specificity, sensitivity)
plt.grid()
plt.xlabel('1-specificity')
plt.ylabel('sensitivity')
plt.title('ROC curve for keyword recognition')

for i in range(len(sensitivity)):
    plt.plot(1-specificity[i] , sensitivity[i], 'ro')           # Plotting the cutoff values
    plt.annotate(round(cutoff[i],2), (1-specificity[i], sensitivity[i]))

plt.show()

















