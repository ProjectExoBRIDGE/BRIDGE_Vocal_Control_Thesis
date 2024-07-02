#AUTOMATIC AUDIO TRANSCRIPTION WITH VOSK

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
import xlsxwriter
import sys

r = sr.Recognizer()


########################################################################################################
# DESCRIPTION OF THE CODE

# The code recognizes the audio files in the 'acquisition' folder, and builds the two confusion matrices:
# 1. One for the commands
# 2. One for the entities
# Row of the matrix: the word analysed
# Column of the matrix: the word of the dictionary being recognized
# The words are recognized with Vosk
# The matrices are built with different values of cutoff in difflib function



########################################################################################################
# file path to the acquisition folder:

file_path = "C:/Users/Utente/PycharmProjects/ProvaSpeechToText1/Acquisitions/"

# IMPORT THE NAMES_ARRAY FILE

with open('names_array', 'rb') as file:
    # Deserialize and retrieve the variable from the file
    names_array = pickle.load(file)         # save the file imported in the variable names_array


########################################################################################################
# IMPORT THE DICTIONARIES AND KEYWORDS

# IMPLEMENTING FUNCTION 'OPEN FILE'
# Input: file_name
# Output: returns the content of a file, this content can be stored in a variable

def openFile(file_name):                        # receiving the file name from which to extract the content

    with open(file_name, 'rb') as file:         # Deserialize and retrieve the variable from the file
        return pickle.load(file)                # return a variable with the content of file loaded


# Calling function, to extract the content from the dictionaries, codes and keywords, and save them in a variable

adjustableCommands = openFile('adjustableCommandsITA')
not_adjustableCommands = openFile('not_adjustableCommandsITA')
commands = adjustableCommands + not_adjustableCommands
entities = openFile('entitiesITA')
commandsCodes = openFile('commandsCodes')
entitiesCodes = openFile('entitiesCodes')



########################################################################################################
# GENERATING THE ARRAY WITH CUTOFF

pace = 0.05
cutoff = np.arange(0.5, 1.0, pace)

synonims = ['basso','bassa']

########################################################################################################
#BUILDING CONFUSION MATRIX


# 1. Create a file

file_excel = xlsxwriter.Workbook('Matrix_Vosk_CUTOFFS_CORRECTION.xlsx')


for k in range(len(cutoff)):                # setting the cutoff - k

    print("--------------------------------------------------------------------------------")
    print("Analysis with cutoff: ", cutoff[k])

    # Initializing the confusion matrix of commands and entities

    commands_matrix = np.zeros(( len(commands), len(commands) ))
    entities_matrix = np.zeros(( len(entities), len(entities) ))

    # Initializing counters

    cont_file_discarded = 0                             # cont the number of times in which hotword is NOT recognized
    cont_no_command = 0                                 # cont the number of times in which command is NOT recognized
    n_entities = 0                                      # cont the number of files in which entity is specified
    cont_file_no_entities =0                            # cont the number of files in which no entity should be recognized


    for i in range(len(names_array)):                    # setting the file - i

        file = sr.AudioFile(file_path + names_array[i])  # loading file - i

        # Splitting up all the features of the name of the file
        # For instance: 'comando_avanti poco_sbj1_session1.wav': ['comando','avanti poco','sbj1','session1.wav']
        features = names_array[i].split('_')

        print('---------------------------------------------')
        print(names_array[i])

        # 0. GETTING THE AUDIO FROM THE FILE - i

        try:        # Try to listen to the file - i

            with file as source:
                audio = r.record(source)  # listening audio of file-i

            text = r.recognize_vosk(audio, language='it')  # saving the transcription in 'text'
            text = text[14: len(text) - 3]  # ATTENTION: this line is to return just the phrase in the string
            text = text.lower()           # all in lower case

            text_words = text.split()     # separating the words of the text recognized
            print(text_words)             # printing separated words

        except:
            print("NO AUDIO RECOGNIZABLE IN THIS FILE, FILE DISCARDED")
            cont_file_discarded += 1      # increase counter of no hotword not recognized

            continue                      # jump to the next file - i


        # 1. CHECKING IF IT RECOGNIZES HOTWORD

        hotword = features[0]           # saving the hotword specified in file name

        try:                            # Try to recognize the hotword in the file

            get_close_matches(text_words[0], [hotword], n=1, cutoff = 0.8)[0]
            # It tries to take the first element [0] in outcome by get_close_matches

        except:                             # if it cannot (no audio in the file, or no hotword recognized)...
            cont_file_discarded += 1        # increase counter of no hotword not recognized
            print("NO HOTWORD RECOGNIZED, FILE DISCARDED")
            continue                        # jump to the next file - i


        # 2. CHECKING IF IT RECOGNIZES A COMMAND (NOT NECESSARILY THE RIGHT ONE)

        composed_command = features[1]                # saving the composed command: for istance: 'avanti poco'

        # Searching for a possible command recognized by the system:

        found = 0  # found = 0: no command found, found = 1: command found
        j = 1      # index of the array of text_words (start by one, because the first word is supposed to be hotword)
        recognized_command = 'none'                  # initialising the recognized_command as 'none'

        while (found == 0 and j < len(text_words)):  # unless it finds the command or exceeds the text_words array

            try:                                     # try to find the command in the word -j
                recognized_command = get_close_matches(text_words[j], commands+synonims, n=1, cutoff=cutoff[k])[0]

                if(recognized_command in synonims):     # if the recognized command is a synonim of 'abbassa'
                    recognized_command = commands[5]    # set recognized command to 'abbassa'

                found = 1                            # if it can recognize a command... found = 1

            except:            # if cannot...
                j += 1         # just increment the index (to try next words) and repeat the cycle



        if (found == 0):        # checking at the end of the cycle if command was recognized, if not...
            cont_no_command +=1 # adding one to no_command cont files
            print("NO COMMAND RECOGNIZED, SAVED AS 'none' ")

        # Out of the cycle, recognized_command will be 'none' (if no command recognized) or one of the commands
        # of the commands array (not necessarily the right one!)


        # 3. CHECKING IF THE RECOGNIZED COMMAND IS THE RIGHT ACTUAL COMMAND

        actual_command = composed_command.split()[0]  # for instance: 'avanti poco' : 'avanti'

        row = commands.index(actual_command)          # this will be the row of the matrix
        col = commands.index(recognized_command)      # this will be the column of the matrix

        commands_matrix[row][col] += 1                # adding one to the row: actual_command, col: recognized_command


        # 4. FOR ALL VALID COMMANDS, CHECKING IF IT RECOGNIZES AN ENTITY (NOT NECESSARILY THE RIGHT ONE)

        if(found==0):        # if no command recognized or no entity available in the file
            continue         # don't prosecute with entity analysis

        found = 0            # found = 0: no entity found, found = 1: entity found
        j = 1                # index of the array of text_words (start by one, because first word is supposed to be hotword)
        recognized_entity = 'none'
        n_entities += 1      # incrementing the counter of the entity analysed (it's used then for the accuracy)

        while (found == 0 and j < len(text_words)):  # unless it finds the entity or exceeds the text_words array

            try:                                     # try to find the entity in the word -j
                recognized_entity = get_close_matches(text_words[j], entities, n=1, cutoff=cutoff[k])[0]
                found = 1                            # if it can... found = 1

            except:            # if cannot...
                j += 1         # just increment the index (to try next words) and repeat the cycle


        # 5. CHECKING IF THE RECOGNIZED ENTITY IS THE RIGHT ACTUAL ENTITY

        if(len(composed_command.split())<2):        # if no entity is supposed to exist in the file...
            cont_file_no_entities +=1               # increase counter of file with no entity available in the name
            actual_entity = 'none'                  # set the actual entity to 'none'

        else:                                       # else... if there should be an entity in the file...
                                                    # save it in actual entity...
            actual_entity = composed_command.split()[1]      #for istance: 'avanti poco': 'poco'

        row = entities.index(actual_entity)              # this will be the row of the matrix
        col = entities.index(recognized_entity)          # this will be the column of the matrix

        entities_matrix[row][col] += 1                   # adding one to the row: actual_entity, col: recognized_entity


    ########################################################################################################
    # PRINT CONFUSION MATRICES

    print("COMMAND MATRIX")
    print(commands_matrix)

    print("ENTITIES MATRIX")
    print(entities_matrix)


    # PRINT COUNTERS

    print("The number of files analysed is: ", len(names_array))
    print("Number of rejected file due to missing hotword: ", cont_file_discarded)
    print("Number of file with missing command: ", cont_no_command)
    print("Number of file in which no entity should be recognized: ", cont_file_no_entities)

    ########################################################################################################
    #CALCULATE THE GLOBAL ACCURACY

    commands_accuracy = sum(np.diag(commands_matrix))/len(names_array)
    entities_accuracy = sum(np.diag(entities_matrix))/n_entities


    print("The command accuracy is: ", commands_accuracy)
    print("The entity accuracy is: ", entities_accuracy)


    ########################################################################################################
    # SAVING ALL OF THE COMMANDS IN AN EXCEL FILE

    # 1. Create a new sheet

    commands_excel = file_excel.add_worksheet()

    # 2. Filling cells vith values

    for row in range(len(commands_matrix)):              # setting row

        for col in range(len(commands_matrix)):          # setting col

            cell = chr(col+65+1)+str(row+2)              # fill cell [row][col]
            commands_excel.write(cell, commands_matrix[row][col])

    # 3. Add the names of the commands and accuracy

    for i in range(len(commands_matrix)):

        cell1 = 'A'+str(i+2)
        cell2 = chr(65+1+i)+str(1)

        commands_excel.write(cell1, commands[i])
        commands_excel.write(cell2, commands[i])

    commands_excel.write('F15', 'Accuracy is:')
    commands_excel.write('G15', commands_accuracy)

    commands_excel.write('F16', 'File discarded:')
    commands_excel.write('G16', cont_file_discarded)

    commands_excel.write('F17', 'File with no command:')
    commands_excel.write('G17', cont_no_command)

    commands_excel.write('F18', 'Cutoff:')
    commands_excel.write('G18', cutoff[k])



    # SAVING ALL OF THE ENTITY IN AN EXCEL FILE

    # 1. Create a sheet for entities matrix

    entities_excel = file_excel.add_worksheet()

    # 2. Filling cells vith values

    for row in range(len(entities_matrix)):

        for col in range(len(entities_matrix)):

            cell = chr(col+65+1)+str(row+2)

            # Calling writing function

            entities_excel.write(cell, entities_matrix[row][col])

    # 3. Add the names of the commands and accuracy

    for i in range(len(entities_matrix)):

        cell1 = 'A'+str(i+2)
        cell2 = chr(65+1+i)+str(1)

        entities_excel.write(cell1, entities[i])
        entities_excel.write(cell2, entities[i])

    entities_excel.write('F15', 'Accuracy is:')
    entities_excel.write('G15', entities_accuracy)
    entities_excel.write('F16', 'Cutoff:')
    entities_excel.write('G16', cutoff[k])


file_excel.close()  # close the file, out of the for cycle







