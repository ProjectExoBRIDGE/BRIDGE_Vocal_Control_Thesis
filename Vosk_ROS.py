#!/usr/bin/env python

# import rospy
import speech_recognition as sr  # Speech_Recognition library
from difflib import get_close_matches  # Difflib library
import time
import pickle
import numpy as np


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
        self.synonims = ['_abbassa','basso', 'bassa','scendi',
                         '_alza','sopra','eleva','solleva',
                         '_ancora','ripeti',
                         '_esci','fermati']

        self.stop = 0

        self.language = 'it'

        times = []
        print("Hey, welcome to Bridge!")
        print("You will be asked to read some sentences, and press enter key when you have finished")
        print("Press any key to start")
        input()
        print("-------------------------------------------------------")
        start_processing = time.perf_counter()
        print("Please read this sentence:")
        print(self.hotword + " " + self.commands[0] + " " + self.entities[0])
        input()
        end_processing = time.perf_counter()
        times.append(end_processing - start_processing)

        print("-------------------------------------------------------")
        start_processing = time.perf_counter()
        print("Please read this sentence:")
        print(self.hotword + " " + self.commands[2] + " " + self.entities[0])
        input()
        end_processing = time.perf_counter()
        times.append(end_processing - start_processing)

        print("-------------------------------------------------------")
        start_processing = time.perf_counter()
        print("Please read this sentence:")
        print(self.hotword + " " + self.commands[8])
        input()
        end_processing = time.perf_counter()
        times.append(end_processing - start_processing)

        print("PHRASE TIME LIMIT SET TO ", max(times))

        self.phrase_time_limit = max(times)

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


    def findWordCode(self, commands, entities, commandsCodes, entitiesCodes, current_command, current_entity):

        # finding indexes:

        command_index = commands.index(current_command)  # find index of command
        entity_index = entities.index(current_entity)    # find index of entity

        code = commandsCodes[command_index] + entitiesCodes[entity_index]

        return code  # return the whole code




    def loop(self):
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

        # Separating the words in the text

        text_words = text.split()

        # Calling get_close_matches function, that returns the hotword just if the user has pronounced it...

        # 1. CHECKING FOR HOTWORD

        found = 0   # found = 0: no keyword found, found = 1: keyword found
        i = 0       # index of the array of text_words

        while (found == 0 and i < len(text_words)):     # unless it finds the keyword or exceeds the text_words array


            try:
                get_close_matches(text_words[i], [self.hotword], n=1, cutoff=0.8)[0]
                # It tries to take the first element [0] in outcome by get_close_matches
                found = 1           # if it finds the keyword: found = 1
            except:  # if it cannot...

                i += 1

        if(found == 0):                     # if no keyword found at the end of the cycle
            print("NO KEYWORD RECOGNIZED")  # show an error
            return                          # next audio


        # Now, at the end of the cycle, if found = 1, the system considers all the words after the keyword

        text_words = text_words[i:]
        print(text_words)

        # 2. CHECKING FOR COMMAND

        found = 0                 # found = 0: no command found, found = 1: command found
        i = 1                     # index of the array of text_words
        current_command = 'none'  # initialising the current_command as 'none'

        while (found == 0 and i < len(text_words)):  # unless it finds the command or exceeds the text_words array

            try:  # try to find the command in the word -i, between commands + synonims

                current_command = get_close_matches(text_words[i], self.commands+self.synonims, n=1, cutoff=0.7)[0]

                if (current_command in self.synonims):      # if the recognized command is a synonim

                        # Searching for the main command associated to that synonim

                        syn_position = self.synonims.index(current_command)         # return the position of the synonim

                        got = 0         # got the main command with '_', for instance: '_abbassa'

                        while(got==0):
                            synonim = self.synonims[syn_position]

                            if (synonim[0]=='_'):       # if the first letter is '_', that means it's the main command

                                got = 1                 # got = 1: found the main command
                                current_command = self.synonims[syn_position]       # the main command is the word without '_'
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
        i = 1      # index of the array of text_words
        current_entity = self.entities[3]  # initialising the current_command as 'none'


        command_index = self.commands.index(current_command)        # saving the index of the command


        # 3.1.CHECKING IF THE COMMAND IS AN ADJUSTABLE COMMAND, IF IT IS, SET ENTITY

        if ( command_index < len(self.adjustableCommands)):    # if it is adjustable...

            # Look for entity ...
            while (found == 0 and i < len(text_words)):  # unless it finds the entity or exceeds the text_words array

                try:  # try to find the entity in the word -i
                    current_entity = get_close_matches(text_words[i], self.entities, n=1, cutoff=0.8)[0]
                    found = 1  # if it can... found = 1

                except:        # if cannot...
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

        # 5. RETURN THE CODE

        code = self.findWordCode(self.commands, self.entities, self.commandsCodes, self.entitiesCodes, current_command, current_entity)

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

    while (vosk_node.stop == 0):
        vosk_node.loop()

