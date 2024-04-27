import serial
import time  
import os
import sys
import struct
on="1"
def fileread(line):
    
    # Step 2: Save the file locally in Colab
    downloads_path = r'C:\Users\amrit\Downloads'
    file1_path = os.path.join(downloads_path, '_init_roboarm.txt')
    with open(file1_path, 'w') as file:
        file.write(line)

    downloads_path = r'C:\Users\amrit\Downloads'
    file_path = os.path.join(downloads_path, 'roboarm.txt')
    while not os.path.exists(file_path):
        time.sleep(3)
    #print("Found roboarm.txt")
    # Read and print the content of the file
    with open(file_path, 'r') as file:
        content = [i for i in file.read().split(",")]      
        #print(content)
    return content

try:
    # Open the serial port
    ser = serial.Serial('COM6', 115200)  
    # Wait for a moment to allow the serial connection to initialize
    time.sleep(2)
    
    while True:
        if ser.is_open:
            # Read bytes from the serial port
            ser.flush()
            data = ser.readline()
            
            
            try:
                # Attempt to decode the bytes as UTF-8
                line = data.decode('utf-8').strip()
                print("Received:", line)  
                if line=="Robo arm ready!":
                    ser.write(fileread(line)[0].encode())
                    print("Data Sent")
                    
                elif line=="2":
                    ser.write(fileread(line)[1].encode())
                elif line=="-0.5528708847336269":
                    ser.write(fileread(line)[2].encode())
                elif line=="0.1485990549217123":
                    ser.write(fileread(line)[3].encode())                    
                elif line=="Task Ended :(":
                    print("Received 'Task Ended :('. Ending the program.")
                    sys.exit()
                    
            except UnicodeDecodeError:
                # If decoding fails, print the raw bytes
                print("Received (raw):", data)
            
            # Write '1' back to the serial port
            
        else:
            print("Serial port is not open.")
            break  # Exit the loop if the serial port is closed
        
except serial.SerialException as e:
    print("Serial Error:", e)
except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    if ser.is_open:
        ser.close()  # Close the serial port if it was opened

