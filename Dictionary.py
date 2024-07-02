#DICTIONARY AND KEYWORDS, SAVED AS FILES

import pickle
import pandas as pd


#Lattuada Valentina
#Mazzani Stefano
#Merlo Francesco
#Meroni Francesco

#DESCRIPTION OF THE CODE
#The script contains:
# 1.The dictionaries
# 2.The codes
# 3.The keywords
# 4.Then when the user runs it, the program saves all the dictionaries, the codes and the keywords in a file with
#a name specified by user.


##########################################################################################################
#1. WRITING THE DICTIONARIES, KEYWORDS AND CODES AS ARRAYS


adjustableCommandsITA = ['avanti', 'indietro', 'sinistra', 'destra','alza','abbassa']

not_adjustableCommandsITA = ['ancora','meno','emergenza','esci','none']

entitiesITA = ['poco','medio','tanto','none']


commandsCodes = ['0','1','2','3','4','5','6','7','8','9','10']
entitiesCodes = ['A','B','C','D']

keyword_ITA = 'comando'



##########################################################################################################
#2. CREATING A FUNCTION THAT RECEIVES A VARIABLE (OR ARRAY) AND SAVES IT AS A FILE IN THE MAIN DIRECTORY

#The function receives as input:
#1. The variabile (or the array)
#2. The name of the file the user wants to save

#And it saves the variable in the directory as a file with the name specified by user


def saveFile(data, file_name):                    # receiving the variable (data) and the file_name

    with open(file_name, 'wb') as file:           # Open the file in binary mode

        pickle.dump(data, file)                   # Serialize and write the variable to the file

    print("The data", file_name, " has been saved successfully.")




##########################################################################################################
# 3. CALLING THE FUNCTION FOR THE DICTIONARIES, CODES AND KEYWORDS, TO SAVE THEM AS FILES

saveFile(adjustableCommandsITA, 'adjustableCommandsITA')
saveFile(not_adjustableCommandsITA, 'not_adjustableCommandsITA')
saveFile(entitiesITA, 'entitiesITA')
saveFile(commandsCodes, 'commandsCodes')
saveFile(entitiesCodes, 'entitiesCodes')
saveFile(keyword_ITA, 'keyword_ITA')







