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



class GoogleNode():

    def __init__(self):
        # Import of ITA Dictionary
        # These arrays contain the dictionary of all the available commands (in Italian and English),
        # and the codes associated to each word of them.

        self.commands = ['avanti','indietro','sinistra','destra','salire','scendere']
        self.entities = ['movimento piccolo','movimento medio','movimento grande']
        self.dictionary = self.commands+self.entities+['none']

        self.distances = [0.05,0.10,0.20,0.0]
        self.current_entity = 'movimento medio'
        self.current_command = 'none'

        self.stop = 0
        self.language = 'it'
        self.phrase_time_limit = 5

        # Creating the istances

        self.r = sr.Recognizer()  # r is an instance that belongs to class Recognizer
        self.mic = sr.Microphone()  # mic is an instance that belongs to class Microphone

        print("Seleziona tra le seguenti opzioni:")
        print("AVANTI, INDIETRO, SINISTRA, DESTRA, SALIRE, SCENDERE")
        print("MOVIMENTO PICCOLO , MOVIMENTO MEDIO, MOVIMENTO GRANDE")


        self.cont = 0  # initialing counter: counts all the commands
        self.coord_variation = [0,0,0]



    def visualSimulation(self, command_index):

        self.coord_variation = [0.0,0.0,0.0]

        # Checking if the command is a changing entity command...

        if self.dictionary[command_index]=='movimento piccolo' or \
           self.dictionary[command_index]=='movimento medio'   or \
           self.dictionary[command_index]== 'movimento grande' :

           self.current_entity = self.dictionary[command_index]  # setting current_entity
           return

        # Checking if the command

        sign = [1, -1, 1, -1, 1, -1]  # 1 = positive, -1 = negative
        c_ind = [0, 0, 1, 1, 2, 2]    # 0 = x, 1 = y, 2 = z

        # if the command is a movement one...

        if command_index < len(self.commands):
            variation = self.distances[ self.entities.index(self.current_entity) ]
            self.coord_variation[ c_ind[command_index] ] = sign[command_index]*variation
            # for instance: coord_variation[2] = +-1*variation ==> dz = +- variation



    def loop(self):
        self.current_command = 'none'

        self.coord_variation = [0.0, 0.0, 0.0]
        self.cont += 1  # Increase counter

        with self.mic as source:
            print("--------------------------------------------------------------")
            print("Say something to BRIDGE, attempt number", self.cont, " :")
            self.r.adjust_for_ambient_noise(source, duration=0.5)
            audio = self.r.listen(source, phrase_time_limit=self.phrase_time_limit)  # getting speech with a specific maximum time listening
        print("BRIDGE is processing...")
        start_processing = time.perf_counter()  # getting the istant of time in which BRIDGE starts processing the word
        try:
            text = self.r.recognize_google(audio, language=self.language)  # saving in text what he recognized after processing
            text = text.lower()             # all in lower case
        except:
            text = 'none'
        finish_processing = time.perf_counter()              # getting the instant of time in which the process ends
        print("I got these words from your speech: ", text)  # showing all the words pronounced
        print("Process time: ", finish_processing - start_processing)

        # 0. CHECKING IF THE COMMAND IS STOP

        words = text.split()

        try:
            words.index('esci')
            self.stop=1
            return
        except:
            self.stop = 0

        # 1. RECOGNIZING COMMAND

        try:
            # Try to find the position of the command pronounced
            command_index = self.dictionary.index(text)
            self.current_command=self.dictionary[command_index]

        except:
            # if it cannot... assign position to -1
            command_index = len(self.dictionary)-1

        # 2. CALLING VISUAL SIMULATION

        self.visualSimulation(command_index)







