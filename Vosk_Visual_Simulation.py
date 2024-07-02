#!/usr/bin/env python

# import rospy
import speech_recognition as sr  # Speech_Recognition library
from difflib import get_close_matches  # Difflib library
import time
import pickle
import numpy as np
from mpl_toolkits import mplot3d
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import pyautogui


class VoskNode():
    def __init__(self):
        # Import of ITA Dictionary
        # These arrays contain the dictionary of all the available commands (in Italian and English),
        # and the codes associated to each word of them.
        self.adjustableCommands = self.openFile('adjustableCommandsITA')
        self.not_adjustableCommands = self.openFile('not_adjustableCommandsITA')
        self.commands = self.adjustableCommands + self.not_adjustableCommands
        self.entities = self.openFile('entitiesITA')
        self.commandsCodes = self.openFile('commandsCodes')
        self.entitiesCodes = self.openFile('entitiesCodes')
        self.hotword = self.openFile('keyword_ITA')
        self.commands_synonims = ['_abbassa','basso', 'bassa','scendi',
                         '_alza','sopra','eleva','solleva',
                         '_ancora','ripeti','ulteriormente',
                         '_esci','fermati',
                         '_meno','ridotto']

        self.entities_synonims = ['_poco','pochino','leggermente','lievemente',
                                  '_medio','mediamente',
                                  '_tanto','molto']

        self.distances = [0.05,0.10,0.20,0.0]
        self.current_command = 'none'
        self.current_entity = 'none'


        self.language = 'it'


        self.phrase_time_limit = 3
        # Creating the istances

        self.r = sr.Recognizer()  # r is an instance that belongs to class Recognizer
        self.mic = sr.Microphone()  # mic is an instance that belongs to class Microphone

        print("Seleziona tra le seguenti opzioni:")
        print("COMANDO + AVANTI/INDIETRO, SINISTRA/DESTRA, ALZA/ABBASSA + (POCO,MEDIO,TANTO)")
        print("Se si vuole eseguire lo stesso comando: COMANDO + ANCORA")
        print("Se si vuole eseguire lo stesso comando ma ridotto di una entità: COMANDO + MENO")
        print("Se si vuole eseguire uscire dal programma: COMANDO + ESCI")
        print("In caso di pericolo, dire COMANDO + EMERGENZA")


        self.cont = 0  # initialing counter: counts all the commands
        self.stop = 0  # this is a boolean variable: stop = 0: Continues asking for words,
                       # stop = 1: stop asking for words
        self.previous_command = 'none'  # this variable saves the last command pronounced by the user, that can be reused
        # if the user expresses the command "again or bit more".
        self.previous_entity = 'none'
        self.coord_variation = [0,0,0]
        self.num = 0


    def findWordCode(self, commands, entities, commandsCodes, entitiesCodes, current_command, current_entity):

        # finding indexes:
        command_index = commands.index(current_command)  # find index of command
        entity_index = entities.index(current_entity)    # find index of entity
        code = commandsCodes[command_index] + entitiesCodes[entity_index]

        return code  # return the whole code


    def visualSimulation(self, code):

        self.coord_variation = [0,0,0]
        # Calculating the abs value of the variation, specified by the last letter of code
        # A = little, B = middle, C = high
        variation = self.distances[self.entitiesCodes.index(code[-1])]
        sign =  [1,-1, 1,-1,1,-1]           # 1 = positive, -1 = negative
        c_ind = [ 0,0 ,1,1,2, 2]            # 0 = x, 1 = y, 2 = z
        command_code = int(code[0])         # casting command code as integer

        if command_code < len(self.adjustableCommands): # only if the command is an adjustable one...
            # Assign variation
            self.coord_variation[c_ind[command_code]] = sign[command_code]*variation
            # for instance: coord_variation[2] = +-1*variation ==> dz = +- variation




    def loop(self):
        self.current_command = 'none'
        self.current_entity = 'none'
        self.coord_variation=[0.0,0.0,0.0]
        self.cont += 1  # Increase counter

        with self.mic as source:
            print("--------------------------------------------------------------")
            print("Say something to BRIDGE, attempt number", self.cont, " :")
            self.r.adjust_for_ambient_noise(source, duration=0.5)
            audio = self.r.listen(source, phrase_time_limit=self.phrase_time_limit)  # getting speech with a specific maximum time listening
        print("BRIDGE is processing...")
        start_processing = time.perf_counter()  # getting the istant of time in which BRIDGE starts processing the word
        text = self.r.recognize_vosk(audio, language=self.language)  # saving in text what he recognized after processing
        text = text.lower()             # all in lower case
        text = text[14: len(text) - 3]  # this is to return just the string
        finish_processing = time.perf_counter()              # getting the instant of time in which the process ends
        print("I got these words from your speech: ", text)  # showing all the words pronounced
        print("Process time: ", finish_processing - start_processing)


        text_words = text.split()               # Separating the words in the text
        print(text_words)                       # show words pronounced

        # 2. CHECKING FOR COMMAND

        found = 0                 # found = 0: no command found, found = 1: command found
        i = 0                     # index of the array of text_words
        current_command = 'none'  # initialising the current_command as 'none'

        while (found == 0 and i < len(text_words)):  # unless it finds the command or exceeds the text_words array

            try:  # try to find the command in the word -i, between commands + synonims

                current_command = get_close_matches(text_words[i], self.commands+self.commands_synonims, n=1, cutoff=0.7)[0]

                if (current_command in self.commands_synonims):      # if the recognized command is a synonim

                        # Searching for the main command associated to that synonim

                        syn_position = self.commands_synonims.index(current_command)         # return the position of the synonim

                        got = 0         # got the main command with '_', for instance: '_abbassa'

                        while(got==0):
                            synonim = self.commands_synonims[syn_position]

                            if (synonim[0]=='_'):       # if the first letter is '_', that means it's the main command

                                got = 1                 # got = 1: found the main command
                                current_command = self.commands_synonims[syn_position]       # the main command is the word without '_'
                                current_command = current_command[1:]

                            syn_position = syn_position - 1

                found = 1                           # if it can recognize a command... found = 1

            except:        # if cannot...
                i = i + 1  # just increment the index (to try next words) and repeat the cycle

        if (found == 0):  # checking at the end of the cycle if command was recognized
            print("NO COMMAND PRONOUNCED")  # if not... print an error
            return                          # and jump to the next listening session

        # 3. CHECKING FOR ENTITIES

        found = 0  # found = 0: no command found, found = 1: command found
        i = 0      # index of the array of text_words
        current_entity = self.entities[3]  # initialising the current_command as 'none'
        command_index = self.commands.index(current_command)        # saving the index of the command


        # 3.1.CHECKING IF THE COMMAND IS AN ADJUSTABLE COMMAND, IF IT IS, SET ENTITY

        if ( command_index < len(self.adjustableCommands)):    # if it is adjustable...

            # Look for entity ...

            while (found == 0 and i < len(text_words)):  # unless it finds the entity or exceeds the text_words array

                try:  # try to find the entity in the word -i, between commands + synonims

                    current_entity = get_close_matches(text_words[i], self.entities+self.entities_synonims, n=1, cutoff=0.7)[0]

                    if (current_entity in self.entities_synonims):  # if the recognized entity is a synonim

                        # Searching for the main command associated to that synonim

                        syn_position = self.entities_synonims.index(current_entity)  # return the position of the synonim

                        got = 0  # got the main command with '_', for instance: '_poco'

                        while (got == 0):
                            synonim = self.entities_synonims[syn_position]

                            if (synonim[0] == '_'):  # if the first letter is '_', that means it's the main entity

                                got = 1  # got = 1: found the main command
                                current_entity = self.entities_synonims[syn_position]  # the main command is the word without '_'
                                current_entity = current_entity[1:]

                            syn_position = syn_position - 1

                    found = 1  # if it can recognize an entity... found = 1

                except:  # if cannot...
                    i = i + 1  # just increment the index (to try next words) and repeat the cycle

            # Checking at the end of the cycle if entity was not recognized ( we already know
            # that it is NOT an exit command)


            if (found == 0):                       # no entity found at the end of the cycle
                current_entity = self.entities[0]  # setting the smallest entity
                print("NO ENTITY PRONOUNCED \n DEFAULT: ", current_entity)  # print a message



        # 4.CHECKING IF THE USER EXPRESSES THE INTENTION TO REPEAT A COMMAND

        if (current_command == self.commands[6]):    # if the user says 'again'
            current_command = self.previous_command  # re-use the previous valid command
            current_entity = self.previous_entity    # re-use the previous entity

        elif (current_command == self.commands[7]):  # if the user says 'less'
            current_command = self.previous_command  # the command remains the same
            previous_entity_index = self.entities.index(self.previous_entity)  # finding the position of the previous entity
            current_entity = self.entities[max(0, previous_entity_index - 1)]  # return the reduced entity ...
            self.previous_entity = current_entity    # if it is not the smallest one already

        elif (current_command != self.commands[10]):  # if the command is not a 'repeat' or 'modulate' action...
            self.previous_command = current_command   # and the word is not 'none' (the command is valid)
            self.previous_entity = current_entity     # saves the current command as the new previous command

        self.current_entity = current_entity

        # 5. RETURN THE CODE

        code = self.findWordCode(self.commands, self.entities, self.commandsCodes, self.entitiesCodes, current_command, current_entity)
        self.current_command = current_command

        # 6. SHOWING RESULTS

        print("Current command: ", current_command)
        print("Current entity: ", current_entity)
        print("Code: ", code)


        # 7. CHECKING IF THE PROGRAM NEEDS TO STOP

        if (current_command == self.commands[8]):  # checking if it's emergency command
            print("EMERGENCY CALLING...")
            self.stop = 1  # stop = 1: break the while cycle

        if (current_command == self.commands[9]):
            print("BRIDGE IS ARRESTING...")
            self.stop = 1  # stop = 1: break the while cycle

        # 8. CREATING THE VISUAL SIMULATION

        self.visualSimulation(code)




    def openFile(self, file_name):
        # CREATING A FUNCTION THAT RECEIVES THE NAME OF THE FILE AND IMPORT IT IN A VARIABLE
        # Input: file_name
        # Output: returns the file that can be saved in a variable
        with open(file_name, 'rb') as file:
            # Deserialize and retrieve the variable from the file
            return pickle.load(file)


if __name__ == "__main__":
    # rospy.init_node('vosk_stt')
    vosk_node = VoskNode()
    # while not rospy.is_shutdown():

    arrest = 0                  # arrest = 0: continue the while, arrest = 1: stop the cycle of the entire listening

    while(arrest ==0):

        vosk_node.stop = 0

        with vosk_node.mic as source:
            print("--------------------------------------------------------------")
            print("Say something to BRIDGE, attempt number", vosk_node.cont, " :")
            vosk_node.r.adjust_for_ambient_noise(source, duration=0.5)
            audio = vosk_node.r.listen(source, phrase_time_limit=vosk_node.phrase_time_limit)  # getting speech with a specific maximum time listening
        print("BRIDGE is processing...")
        start_processing = time.perf_counter()  # getting the istant of time in which BRIDGE starts processing the word
        text = vosk_node.r.recognize_vosk(audio, language=vosk_node.language)  # saving in text what he recognized after processing
        text = text.lower()             # all in lower case
        text = text[14: len(text) - 3]  # this is to return just the string
        finish_processing = time.perf_counter()              # getting the instant of time in which the process ends
        print("I got these words from your speech: ", text)  # showing all the words pronounced
        print("Process time: ", finish_processing - start_processing)

        # Separating the words in the text

        text_words = text.split()

        # 1. CHECKING FOR HOTWORD OR STOP PROGRAM

        listen = 0  # listen = 0: no keyword found, Listen = 1: keyword found
        i = 0       # index of the array of text_words

        while (arrest ==0 and listen == 0 and i < len(text_words)):  # unless it finds the keyword (listen = 1)
                                                                     # or exceeds the text_words array
                                                                     # or the user wants to arrest the program (arrest=1)

            try:
                text = get_close_matches(text_words[i], [vosk_node.hotword, 'stop'], n=1, cutoff=0.8)[0]
                # It tries to take the first element [0] in outcome by get_close_matches

                if text==vosk_node.hotword:
                    listen = 1  # if it finds the keyword: found = 1
                    print("HERE I AM")
                if text == 'stop': # if the user wants to stop the program: arrest = 1
                    print("I AM STOPPING")
                    arrest = 1 # arrest = 1: stopping program


            except:  # if it cannot recognize neither the kewyord or the arrest command

                i += 1  #next iteration

        if (listen == 0):  # if no keyword found at the end of the cycle
            print("NO KEYWORD RECOGNIZED")  # show an error


        while(listen==1 and vosk_node.stop == 0):   # while keyword is still active and user doesn't want to exit...

            vosk_node.loop()


