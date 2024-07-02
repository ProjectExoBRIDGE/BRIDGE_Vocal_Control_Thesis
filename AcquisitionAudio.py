# AUTOMATIC AUDIO ACQUISITION

# Lattuada Valentina
# Mazzani Stefano
# Merlo Francesco
# Meroni Francesco

import pyaudio
import wave
import pickle

#DESCRIPTION OF THE CODE

# 1. The code acquires authomatically the audiofiles
# 2. It saves them in the directory, in a specific folder called ACQUISITIONS
# 3. If it doesn't already exist, it produces an array with all the acquisition NAMES, and saves it in a file
# 4. If the array of the file names already exists, then:
# - it checks if the file X is already saves, (if it is, it just leaves it there...)
# - if the file name X is not already existing, it adds the new file name

# The purpose of the array called 'names_array' is to contain all the names of the files with the
# acquisitions.
# This array will be saved as a file, so that other scripts and functions can import this files and
# have accesses to the acquisitions by calling their names.


###############################################################################################
# IMPORT THE DICTIONARIES AND KEYWORDS FUNCTION


# IMPLEMENTING FUNCTION 'OPEN FILE'
# Input: file_name
# Output: returns the content of a file, this content can be stored in a variable

def openFile(file_name):                        #receiving the file name from which to extract the content

    with open(file_name, 'rb') as file:         #Deserialize and retrieve the variable from the file
        return pickle.load(file)                #return a variable with the content of file loaded

# Calling functions, to extract the content from the dictionaries, codes and keywords, and save them in a file

adjustableCommands = openFile('adjustableCommandsITA')
not_adjustableCommands = openFile('not_adjustableCommandsITA')
commands = adjustableCommands + not_adjustableCommands
entities = openFile('entitiesITA')
commandsCodes = openFile('commandsCodes')
entitiesCodes = openFile('entitiesCodes')
hotword = openFile('keyword_ITA')



############################################################################################

# IMPLEMENTATION OF THE AUTOMATIC AUDIO ACQUISITION FUNCTION

# This function receives as input:
# 1.The name of the dictionary word recorded by the subject
# 2.A number that identifies the subject
# 3.A number 0: no noise, 1: with noise

# And it saves an audio file named as 'word_subject.wav', and add the name of the file in the names_array
# If the array of the file names already exists, then:
# - it checks if the file X is already saves, (if it is, it just leaves it there...)
# - if the file name X is not already existing, it adds the new file name
# If the array with the name doesn't exist in the directory, then it creates a new one


def audioAcquisition(dictionary_word, subject_ID, session, noise):  #receiving the phrase, the subID, and noise

    try:                                                #try to import the names_array file
        with open('names_array', 'rb') as file:         #if it can...
            names_array = pickle.load(file)             #uses this array
    except:                                             #if it cannot, because it doesn't exist...
        names_array = []                                #generate a new names_array


    # Setting acquisition constants
    FORMAT = pyaudio.paInt16    # Format of audio samples (16-bit signed integers)
    CHANNELS = 1                # Number of audio channels (1 for mono, 2 for stereo)
    RATE = 40000                # Sample rate (samples per second)
    CHUNK = 1024                # Number of frames per buffer
    RECORD_SECONDS = 3.0        # Duration of recording in seconds

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    # Getting audio

    print("-----------------------------------------------------------------------")
    print("Please, read these words: ", hotword, dictionary_word)
    frames = []             # initialising the empty frame array
    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("Thank you.")
    stream.stop_stream()
    stream.close()

    # Composing the file name, as following:

    FILENAME = hotword+'_'+dictionary_word+'_sbj'+str(subject_ID)+'_session'+str(session)

    if noise==1:                                # if there is noise: noise=1
        FILENAME = FILENAME+'_noise.wav'        # add '_noise.wav'
    else:                                       # if there isn't noise: noise=0
        FILENAME = FILENAME+'.wav'              # just add '.wav'

    # Saving the file in a specific file path

    # file path:
    file_path = "C:/Users/Utente/PycharmProjects/ProvaSpeechToText1/Acquisitions/"+FILENAME

    wf = wave.open(file_path, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    print("Recording saved as", FILENAME)
    p.terminate()

    # Adding the filename to names_array

    # Checking if the file_name is already in the array

    for i in range(len(names_array)):                       # checking the single element-i of the names_array

        if names_array[i] == FILENAME:                      # it finds the file_name in the array
            print("File already existing, but overwrited")  # just print this message
            return                                          # and stop function

    # If the function is already running (no file already existing)

    print("Adding file...")
    names_array.append(FILENAME)                            # add the name to the names_array

    # At the end, saving the new names_array as a file 'names_array'

    with open('names_array', 'wb') as file:
        # Serialize and write the variable to the file
        pickle.dump(names_array, file)


#################################################################################################
# IMPLEMENTATION OF THE MAIN CODE


# Asking the user these two information:
# 1.What is the ID of the subject (an integer number)
# 2.What is the number of the recording session
# 3.If there's noise 0:no noise, 1: with noise

# Then calling the function for each word

subject_ID = int(input("What is your subject ID?\n"))
session = int(input("What is the number of the recording session?\n"))
noise = int(input("Is there noise 0: no noise, 1: noisy audio\n"))

for i in range(len(adjustableCommands)):            # setting the adjustable command - i

    audioAcquisition(adjustableCommands[i], subject_ID, session, noise)    # saving just the command - i (with no entity)

    for j in range(len(entities)-1):                 # setting the entity - j

        dictionary_word = adjustableCommands[i]+' '+entities[j]   # composing the dictionary word: command + entity
        audioAcquisition(dictionary_word, subject_ID,session, noise)        # calling function


for i in range(len(not_adjustableCommands)):                      # recording the not adjustable commands (no entity)

    audioAcquisition(not_adjustableCommands[i], subject_ID, session, noise) # saving the not adjustable commands








