import serial
from time import sleep
import FreeSimpleGUI as sg  
import tkinter as tk
import os

def hide_cursor():
    os.system("unclutter -idle 0")

def openSerialConnection():
    ser = serial.Serial ("/dev/ttyUSB0", 921600, timeout=0.1, write_timeout=(1))    #Open port with baud rate
    return ser

def echoSerialData(ser):
    received_data = ser.read()              #read serial port
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    print(received_data)                   #print received data

def readSerialData(ser):
    received_data = ser.read()              #read serial port
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    return received_data

def readSerialDataByLine(ser):
    received_data = ser.readline()              #read serial port
    return received_data

def writeSerialData(ser, data):
    encodedData = data.encode()
    ser.write(encodedData)                #transmit data serially  


root = tk.Tk()                               #get screen dimensions
width = root.winfo_screenwidth()
height = root.winfo_screenheight()         
root.withdraw()


testControlFrame =      [
                         [sg.Button("Start Test", expand_x=True, expand_y=True)],
                         [sg.Button("Close", expand_x=True, expand_y=True)]     
                        ]

testInstructionsFrame = [
                         [sg.Multiline(key="instructionsTextbox",write_only=True, reroute_cprint=True, expand_y=True, expand_x=True)]
                        ]

testResultsFrame =      [
                         [sg.Multiline(key="resultsTextbox", write_only=True, reroute_cprint=True, expand_y=True, expand_x=True)]
                        ]

printerTestLayout  =    [
                         [sg.Frame("Test Controls", testControlFrame, background_color='grey', size=((width / 3) - 10, height - 15)), 
                          sg.Frame("Test Instructions", testInstructionsFrame, background_color='grey', size=((width / 3) - 10, (height) - 15)), 
                          sg.Frame('Test Results', testResultsFrame, background_color='grey', size=((width / 3) - 10, (height) - 15))] 
                        ]

window = sg.Window('Title', printerTestLayout, no_titlebar=True, finalize=True)

window.maximize()

try:
    serialConnection = openSerialConnection()
except:
    print("Failed to open serial connection, insert the USB to serial cable into the USB port of this Raspberry Pi and try to execute this script again")
    sg.popup('Failed to open serial connection, insert the USB to serial cable into the USB port of this Raspberry Pi and try to execute this script again', no_titlebar=True)
    exit()

testCount = 1

while True:
    #echoSerialData(serialConnection)
    sdEncoded = readSerialDataByLine(serialConnection)
    
    try:
        sdDecoded = sdEncoded.decode("utf-8")                                       #decode
    except:
        sg.cprint("Could not decode", key="resultsTextbox")                         #could not decode
        sg.cprint(sdEncoded, key="resultsTextbox")
        sdDecoded = ""
        
    if sdDecoded != "":                                                             #if a line has been read
        if sdDecoded == "ALL TESTS PASSED\n":                                       #all tests passed     
            sg.cprint(sdDecoded, key="resultsTextbox", colors="green on white")
            
        #elif sdDecoded == "   \n":                                                 
        #    sg.cprint("", key="resultsTextbox")

        elif sdDecoded == "PRINTHEAD TYPE TEST PASS\n":                             #printhead type test PASS                     
            sg.cprint(sdDecoded, key="resultsTextbox", colors="green on white")

        elif sdDecoded == "PRINTHEAD TYPE TEST FAIL\n":                             #printhead type test FAIL                      
            sg.cprint(sdDecoded, key="resultsTextbox", colors="red on white")

        elif sdDecoded == "PRINTHEAD TEMPERATURE TEST PASS\n":                      #printhead temp test PASS                     
            sg.cprint(sdDecoded, key="resultsTextbox", colors="green on white")

        elif sdDecoded == "PRINTHEAD TEMPERATURE TEST FAIL\n":                      #printhead temp test FAIL                      
            sg.cprint(sdDecoded, key="resultsTextbox", colors="red on white")

        elif sdDecoded[0:10] == "TUTestVal -":                                      #getTakeUpTorque()          
            sg.cprint(sdDecoded, key="resultsTextbox")

        elif sdDecoded[0:10] == "SHOOTTestVa":                                      #getShootAverage()         
            sg.cprint(sdDecoded, key="resultsTextbox")

        elif sdDecoded[0:10] == "LOWSTOCKTes":                                      #getLowStockSensor()       
            sg.cprint(sdDecoded, key="resultsTextbox")

        elif sdDecoded[0:14] == "PRINTHEADTemp":                                    #getPrintheadTemperatureInCelsius()     
            sg.cprint(sdDecoded, key="resultsTextbox")

        elif sdDecoded[0:14] == "PRINTHEADType":                                    #getPrintHeadType()     8 == 80mm
            sg.cprint(sdDecoded, key="resultsTextbox")

        elif sdDecoded == "STUCK TU HUB TEST - TEST PASS\n":                        #stuck hub PASS                      
            sg.cprint(sdDecoded, key="resultsTextbox", colors="green on white")

        elif sdDecoded == "STUCK TU HUB TEST - TEST FAIL\n":                        #stuck hub FAIL                      
            sg.cprint(sdDecoded, key="resultsTextbox", colors="red on white")

        elif sdDecoded == "!!!\n":                                                  #new test started, clear screen'
            for i in range(100):
                sg.cprint("", key="resultsTextbox")
        
        elif sdDecoded == "\n":                                                     #new line -- why did i put this here?
            sg.cprint("", key="resultsTextbox")
        
        else:
            sg.cprint(sdDecoded, key="instructionsTextbox")
   
    event, values = window.read(timeout=(10))
    if event == sg.WIN_CLOSED or event == "Exit":
        break

    if event == "Close":
        window.Close()
    elif event == "Start Test":
        writeSerialData(serialConnection, " \r")
    


        


    

    

    

