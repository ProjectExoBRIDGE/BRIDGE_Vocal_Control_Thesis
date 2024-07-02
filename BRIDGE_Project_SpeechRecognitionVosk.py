# VOCAL INTERFACE BRIDGE PROJECT
# SPEECH RECOGNITION

# Lattuada Valentina
# Mazzani Stefano
# Merlo Francesco
# Meroni Francesco


#Import useful libraries



import speech_recognition as sr             # Speech_Recognition library
from difflib import get_close_matches       # Difflib library
import time
import pickle
import numpy as np

######################################################################################################
# Import of ITA Dictionary
# These arrays contain the dictionary of all the available commands (in Italian and English),
# and the codes associated to each word of them.

# CREATING A FUNCTION THAT RECEIVES THE NAME OF THE FILE AND IMPORT IT IN A VARIABLE
# Input: file_name
# Output: returns the file that can be saved in a variable

def openFile(file_name):

    with open(file_name, 'rb') as file:
        # Deserialize and retrieve the variable from the file
        return pickle.load(file)


#Calling the function

adjustableCommands = openFile('adjustableCommandsITA')
not_adjustableCommands = openFile('not_adjustableCommandsITA')
commands = adjustableCommands + not_adjustableCommands
entities = openFile('entitiesITA')
commandsCodes = openFile('commandsCodes')
entitiesCodes = openFile('entitiesCodes')
hotword = openFile('keyword_ITA')



###################################################################################################
#IMPLEMENTATION OF USEFUL FUNCTIONS THAT WILL BE USED IN THE CODE



# 1.IMPLEMENTING THE FUNCTION FIND_WORD_CODE

# Input: commands dictionary, entities dictionary, codes lists, current_command and current_entity
# Output: it returns the specific code of the command

def findWordCode(commands, entities, commandsCodes, entitiesCodes, current_command, current_entity):

    # finding indexes:

    command_index = commands.index(current_command)                 # find index of command
    entity_index = entities.index(current_entity)                   # find index of entity

    code = commandsCodes[command_index] + entitiesCodes[entity_index]

    return  code    # return the whole code


######################################################################################################
# IMPLEMENTATION OF THE MAIN CODE

language = 'it'

# Creating the istances

r = sr.Recognizer()        # r is an instance that belongs to class Recognizer
mic = sr.Microphone()      # mic is an instance that belongs to class Microphone


print("Seleziona tra le seguenti opzioni:")
print("COMANDO + AVANTI/INDIETRO, SINISTRA/DESTRA, ALZA/ABBASSA + (POCO,MEDIO,TANTO)")
print("Se si vuole eseguire lo stesso comando: COMANDO + ANCORA")
print("Se si vuole eseguire lo stesso comando ma ridotto di una entità: COMANDO + MENO")
print("Se si vuole eseguire uscire dal programma: COMANDO + ESCI")
print("In caso di pericolo, dire COMANDO + EMERGENZA")


# GETTING USER SPEECH

cont=0         # initialing counter: counts all the commands
stop =0        # this is a boolean variable: stop = 0: Continues asking for words,
                                           # stop = 1: stop asking for words
previous_command ='none'   # this variable saves the last command pronounced by the user, that can be reused
                         # if the user expresses the command "again or bit more".
previous_entity = 'none'


while(stop==0):                     # Unless the user expresses their willing to stop the listening program...
    
    cont = cont+1           #Increase counter

    with mic as source:
        print("--------------------------------------------------------------")
        print("Say something to BRIDGE, attempt number", cont, " :")
        r.adjust_for_ambient_noise(source)
        audio = r.listen(source, phrase_time_limit = 3)    # getting speech with a specific maximum time listening
    print("BRIDGE is processing...")
    start_processing = time.perf_counter()         # getting the istant of time in which BRIDGE starts processing the word
    text = r.recognize_vosk(audio, language=language)       # saving in text what he recognized after processing
    text = text.lower()                                     # all in lower case
    text = text[14: len(text)-3]                                     # this is to return just the string
    finish_processing = time.perf_counter()                          # getting the instant of time in which the process ends
    print("I got these words from your speech: ", text)              # showing all the words pronounced
    print("Process time: ", finish_processing - start_processing)


    # Separating the words in the text

    text_words = text.split()

    # Calling get_Close_matches function, that returns the hotword just if the user has pronounced it...

    # 1. CHECKING FOR HOTWORD

    try:
        get_close_matches(text_words[0], [hotword], n=1, cutoff = 0.8)[0]
        # It tries to take the first element [0] in outcome by get_close_matches
    except:                                                  # if it cannot...
        print("NO KEYWORD PRONOUNCED")                       # show an error
        continue

    # 2. CHECKING FOR COMMAND

    found=0                                     # found = 0: no command found, found = 1: command found
    i = 1                                       # index of the array of text_words
    current_command = 'none'                    # initialising the current_command as 'none'

    while (found == 0 and i < len(text_words)): # unless it finds the command or exceeds the text_words array

        try:                                    # try to find the command in the word -i
            current_command = get_close_matches(text_words[i], commands, n=1, cutoff=0.8)[0]
            found = 1                           # if it can... found = 1

        except:                                 # if cannot...
            i = i + 1                           # just increment the index (to try next words) and repeat the cycle

    if(found==0):                               # checking at the end of the cycle if command was recognized
        print("NO COMMAND PRONOUNCED")          # if not... print an error
        continue                                # and jump to the next listening session

    #3. CHECKING FOR ENTITIES

    found = 0                                   # found = 0: no command found, found = 1: command found
    i = 1                                       # index of the array of text_words
    current_entity = entities[3]                # initialising the current_command as 'none'

    while (found == 0 and i < len(text_words)): # unless it finds the entity or exceeds the text_words array

        try:                                     # try to find the entity in the word -i
            current_entity = get_close_matches(text_words[i], entities, n=1, cutoff=0.8)[0]
            found = 1                            # if it can... found = 1

        except:                                  # if cannot...
            i = i + 1                            # just increment the index (to try next words) and repeat the cycle

    # checking at the end of the cycle if entity was recognized but the command was not an EXIT command...
    if (found == 0 and current_command!=commands[8] and current_command!=commands[9]):
        current_entity = entities[0]                               # setting the smallest entity
        print("NO ENTITY PRONOUNCED \n DEFAULT: ", current_entity) # print a message



    #CHECKING IF THE USER EXPRESSES THE INTENTION TO REPEAT A COMMAND

    if(current_command == commands[6]):             # if the user says 'again'
        current_command = previous_command             # re-use the previous valid command
        current_entity = previous_entity               # re-use the previous entity

    elif(current_command == commands[7]):           # if the user says 'less'
        current_command = previous_command          # the command remains the same
        previous_entity_index = entities.index(previous_entity)      # finding the position of the previous entity
        current_entity = entities[ max(0, previous_entity_index-1)]  # return the reduced entity ...
        previous_entity = current_entity                             # if it is not the smallest one already

    elif(current_command != commands[10]):          # if the command is not a 'repeat' or 'modulate' action...
        previous_command = current_command          # and the word is not 'none' (the command is valid)
        previous_entity = current_entity            # saves the current command as the new previous command


    # RETURN THE CODE

    code = findWordCode(commands, entities, commandsCodes, entitiesCodes, current_command, current_entity)

    # SHOWING RESULTS

    print("Current command: ", current_command)
    print("Current entity: ", current_entity)
    print("Code: ", code)

    #CHECKING IF THE PROGRAM NEEDS TO STOP

    if(current_command==commands[8]):               # checking if it's emergency command
        print("EMERGENCY CALLING...")
        stop = 1  # stop = 1: break the while cycle

    if(current_command==commands[9]):
        print("BRIDGE IS ARRESTING...")
        stop = 1  # stop = 1: break the while cycle




