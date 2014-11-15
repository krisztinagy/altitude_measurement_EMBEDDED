#import GPIO for LED control
import RPi.GPIO as GPIO

#usb specific imports
import usb.core
import usb.util
import usb.legacy
import usb.control

#serial import
import serial

#i2c import
import smbus

#BMP barometric sensor import
import Adafruit_BMP.BMP085 as BMP085

#other imports
import os
import time
import sched
import datetime
import threading



#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#XXXXXXXXXXXXXXXXXXXXXXXXXX   VARIABLE DEFINES    XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#setting up GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)
#the pin is active-low, so for turning the LED off, the pin needs to be set to high
GPIO.output(16, GPIO.HIGH)

#defining a scheduler for the timing of the GPS and Altitude data read and save
s1 = sched.scheduler(time.time, time.sleep)

#lists (empty) to store devices
sensor = []
mass = []
hid = []
hub = []
wifi = []

global_flag_bit = 0

#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#XXXXXXXXXXXXXXXXXXXXXXXXXX   FUNCTION DEFINES    XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#defining type functions
#device typer is defined either in bDeviceClass or in bInterfaceClass
def is_mass_storage(dev):
    if dev.bDeviceClass == 0x08: #0x08 stands for mass storage type
        return True
    for cfg in dev:
        if usb.util.find_descriptor(cfg, bInterfaceClass = 0x08) is not None:
           return True
        
def is_wifi_device(dev):
    if dev.bDeviceClass == 0x02: #0x02 stands for (wifi) adapter devices
        return True
    for cfg in dev:
        if usb.util.find_descriptor(cfg, bInterfaceClass = 0x02) is not None:
           return True
        
def is_human_interface_device(dev):
    if dev.bDeviceClass == 0x03: #0x03 stands for HID type
        return True
    for cfg in dev:
        if usb.util.find_descriptor(cfg, bInterfaceClass = 0x03) is not None:
            return True

def is_hub(dev):
    if dev.bDeviceClass == 0x09: #0x09 stands for hub type
        return True
    for cfg in dev:
        if usb.util.find_descriptor(cfg, bInterfaceClass = 0x09) is not None:
            return True
        
#defining the GPS data saver function
#function is called with endpoint and the target file
def GPS_data_save(endpoint, file, sc):

    # 512-2=510 useful bytes/packet, 38400 byte/s => t=510/38400= 0.01328125s
    #reenter function in selected time period
    sc.enter(0.01328125, 1, GPS_data_save, (endpoint, file, sc,))

    GPIO.output(16, GPIO.LOW)
    
    #accessing global variable
    global converter

    #taking over control from kernel
    try:
        converter.detach_kernel_driver(0)
    except:
        True

    #reading packets from endpoint    
    data = converter.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, 10000)
    
    #for the very first message clearing everything before the first $ symbol(ASCII = 36), to make the NMEA message start with message type
    #pop every character that is not "$"

    global global_flag_bit
    if(global_flag_bit == 0):
        if(data.count(36) > 0):
            index = data.index(36)
            while(index > 2):
                data.pop(0)
                index -= 1
            global_flag_bit = 1

    #for every further message the whole message is printed in file, exept for the two preamble  chars at the beginning
            
    ##  writing every char(except the first two) into file
    ##  first two characters are preamble, not payload characters
    j = 0
    while (j < len(data)):
        if (j > 1):
            file.write(chr(data[j]))
        j += 1

        
    #At the site, there is no possibility of attaching a screen to the Raspberry Pi, so some other
    #feedback must be used concerning the work of the device.
    #The flashing of the ACT LED shows, that the program is running without error. The ligh is on for 1s.
    #If this LED is turned off, then the program has some kind of an error.
    #Detaching the power and the communication interfaces usually solves the program.
    #Waiting some time for the GPS receiver to setup may also help.
    #There might be some problem with the USB serial converter, since reattaching it usually solves the problem.

    #The GPIO16 pin is low-active.
##    GPIO.output(16, GPIO.LOW)
##    time.sleep(0.3)
##    GPIO.output(16, GPIO.HIGH)
    GPIO.output(16, GPIO.HIGH)

#setting OS system time
def GPS_get_time_date(endpoint):

    #wait for OS to set up
    time.sleep(0.5)

    #accessing global variable
    global converter

    #taking over control from kernel
    try:
        converter.detach_kernel_driver(0)
    except:
        True

    #reading two packets from endpoint to determine GPS time(sometimes one packet is not enough,
    #since only certain messages contain the desired information
    data1 = converter.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, 10000)
    data1.pop(0)#deleting the first two dummy bytes
    data1.pop(0)
    data2 = converter.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, 10000)
    data2.pop(0)#deleting the first two dummy bytes
    data2.pop(0)

    data = data1 + data2 #concatenating the two data arrays

    #for debug purposes
    print (len(data))
##    k = 0
##    while (k < len(data)):
##        print(data[k])
##        k += 1

    #looking for $GPRMC messages
    #searching for first "R" character, and deleting everyting before it
    while(data.count(82) == 0):
        print("'R' char not in list, reading stream again")
        data3 = converter.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, 10000)
        data3.pop(0)
        data3.pop(0)
        data = data+ data3
        
    if(data.count(82)>0):
        R_index=data.index(82)
        while(R_index > 0):
            data.pop(0)
            R_index -= 1
            
        while(data.count(44) == 0):
            print("',' char not in list, reading stream again")
            data3 = converter.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, 10000)
            data3.pop(0)
            data3.pop(0)
            data = data+ data3
            
        if(data.count(44)>0):
            #deleting everything until the first "," character
            comma_index = data.index(44) + 1
            while(comma_index > 0):
                data.pop(0)
                comma_index -= 1
            #reading out time from message, and deleting the already saved characters
            time_now = []
            i = 0
            while(i<9):
                time_now.append(data[0])
                data.pop(0)
                i += 1
            #deleting everything before the date
            i = 0
            while(i < 8):
                comma_index = data.index(44) + 1
                while(comma_index > 0):
                    data.pop(0)
                    comma_index -= 1
                i += 1
            #reading out date from RMC message
            date_now = []
            i = 0
            while(i<6):
                date_now.append(data[0])
                data.pop(0)
                i += 1


    #making the expected string format from the date and time array  
    string_time = ""
    string_date = ""

    string_time = chr(time_now[0])+chr(time_now[1])+chr(58)+chr(time_now[2])+chr(time_now[3])+chr(58)+chr(time_now[4])+chr(time_now[5])
    print(string_time)

    string_date = "20"+chr(date_now[4])+chr(date_now[5])+chr(date_now[2])+chr(date_now[3])+chr(date_now[0])+chr(date_now[1])
    print(string_date)

    #giving the expected string format to the OS to set system time
    os.system('date +%Y%m%d -s \"' +  string_date + '\"')
    os.system('date +%T -s \"' +  string_time + '\"')

#accessing gyroscope for Euler angles
def ALT_data_save(endpoint_in, endpoint_out, file, sc):

    #accessing global variable
    global converter

    #function calling itself in every 0.2 s
    sc.enter(0.2, 1, ALT_data_save, (endpoint_in, endpoint_out, file, sc,))

    #taking control over from kernel
    try:
        converter.detach_kernel_driver(2)
    except:
        True

    #polling Euler angles (0x80, 0x0c) message
    command = [0x75, 0x65, 0x0c, 0x07, 0x07, 0x01, 0x00, 0x01, 0x0c, 0x00, 0x00, 0x02, 0xfc]
    converter.write( endpoint_out.bEndpointAddress, command )

    data = []
    #reading the polled data
    data = converter.read( endpoint_in.bEndpointAddress, endpoint_in.wMaxPacketSize, 100)
    data = data[0:32]

    #pasting a time stamp before every polled data line
    file.write(str(datetime.datetime.now()))
    file.write(": ")

    #saving the polled data into the file
    j = 0
    while (j < len(data)):
        if (j > 1):
            file.write(hex(data[j]))
        j += 1
        
    #print newline between data lines
    file.write("\n")   
    
def RAD_data_save(endpoint_in, file, sc):
    
    #function calling itself in every 0.1 s
    sc.enter(0.1, 1, RAD_data_save, (endpoint_in, file, sc,))

    #accessing global variable
    global converter

    #taking control over from kernel
    try:
        converter.detach_kernel_driver(1)
    except:
        True


    #reading data stream
    data = converter.read( endpoint_in.bEndpointAddress, endpoint_in.wMaxPacketSize, 1000)

    cross_index=data.index(35)
    while(cross_index > 0):
        data.pop(0)
        cross_index -= 1

    #pasting a time stamp before every polled data line
    file.write(str(datetime.datetime.now()))
    file.write(": ")
    file.write("\n")

    #saving the polled data into the file
    j = 0
    while (j < len(data)):
        file.write(chr(data[j]))
        j += 1


    #print newline between data lines
    file.write("\n")

def BAR_data_save(file, sc):

    #function calling itself in every 0.1 s
    sc.enter(0.1, 1, BAR_data_save, (file, sc,))

    #defining variables for measured data    
    temp = bar_sensor.read_temperature()
    pressure = bar_sensor.read_pressure()
    altitude = bar_sensor.read_altitude()
    sealevel_pressure = bar_sensor.read_sealevel_pressure()

    #saving out data into file
    file.write(str(datetime.datetime.now()))
    file.write(": ")
    file.write("\n")
    file.write(str(temp))
    file.write("\n")
    file.write(str(pressure))
    file.write("\n")
    file.write(str(altitude))
    file.write("\n")
    file.write(str(sealevel_pressure))
    file.write("\n")


    
    
#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#XXXXXXXXXXXXXXXXXXXXXXXXXX         MAIN          XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#opening ports
try:
    port_gps = serial.Serial("/dev/ttyUSB0", 38400)
    port_alt = serial.Serial('/dev/ttyUSB1', 115200)
    port_rad = serial.Serial('/dev/ttyUSB2', 115200)
except:
    True

#selecting all devices connected with USB
devices = usb.core.find(find_all = True)

#sorting usb devices into categories (lists)
#anything not matching the mass storage, HID, hub categories is supposed to be a sensor
for dev in devices:
    if is_mass_storage(dev):
        mass.append(dev)
    elif is_human_interface_device(dev):
        hid.append(dev)
    elif is_hub(dev):
        hub.append(dev)
    else:
        sensor.append(dev)

#manually deleting the obsolete device(something on the Pi Board)
for i in sensor:
    if i.idVendor == 0x0424:
        if i.idProduct == 0xec00:
            sensor.remove(i)
            print "torolve"

converter = sensor[0]
    
converter.reset()


#printing the number of members of each group
print "mass storage:", len(mass)
print "hub:", len(hub)
print "hid:", len(hid)
print "sensor:", len(sensor)


#printing the configuration of the converter to which the sensors are attached
print "converter active", converter.get_active_configuration()


#XXXXXXXXXXXXXXXXXXXXXXXXXX    ACCESSING GPS MODULE     XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#taking control over from kernel
try:
    converter.detach_kernel_driver(0)
except:
    True
    
#gaining control over the device    
cfg = converter.get_active_configuration()
endpoint_GPS = cfg[(0,0)][0]
    
#setting system time
GPS_get_time_date(endpoint_GPS)

#making and opening of the data file
os.chdir("/var/www")
time_stamp = str(datetime.datetime.now())
time_stamp =time_stamp[0:19]
time_stamp = time_stamp.replace('-', '').replace(' ', '_').replace(':', '')
file_name_GPS = 'GPS_' + time_stamp + '.txt'
data_GPS = open(file_name_GPS, "a+")

#XXXXXXXXXXXXXXXXXXXXXXXX    ACCESSING ALTITUDE MODULE  (GYRO)   XXXXXXXXXXXXXXXXXXXXXXXXXXXX

#taking control over from kernel
try:
    converter.detach_kernel_driver(2)
except:
    True
    
#gaining control over the device    
endpoint_ALT_IN = cfg[(2,0)][0]
endpoint_ALT_OUT = cfg[(2,0)][1]

#state control message
command = [0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xe0, 0xc6]
converter.write( endpoint_ALT_OUT.bEndpointAddress, command )
data = converter.read( endpoint_ALT_IN.bEndpointAddress, endpoint_ALT_IN.wMaxPacketSize, 10000)

command = [0x75, 0x65, 0x0c, 0x05, 0x05, 0x11, 0x01, 0x01, 0x00, 0x03, 0x19]
converter.write( endpoint_ALT_OUT.bEndpointAddress, command )

#making and opening of the data file
file_name_ALT = "GYR_"  + time_stamp + ".txt"
data_ALT = open(file_name_ALT, "a+")

#XXXXXXXXXXXXXXXXXXXXXXXX    ACCESSING RADIO ALTITUDE MODULE     XXXXXXXXXXXXXXXXXXXXXXXXXXXX

#gaining control over the device    
endpoint_RAD_IN = cfg[(1,0)][0]

#making and opening of the data file
file_name_RAD = "ALT_" + time_stamp + ".txt"
data_RAD = open(file_name_RAD, "a+")

#XXXXXXXXXXXXXXXXXXXXX    ACCESSING BAROMETRIC ALTITUDE MODULE     XXXXXXXXXXXXXXXXXXXXXXXXXX

#instantiating a new barometric sensor device
bar_sensor = BMP085.BMP085()

#making and opening of the data file
file_name_BAR = "BAR_" + time_stamp + ".txt"
data_BAR = open(file_name_BAR, "a+")

#XXXXXXXXXXXXXXXXXXXXXXXXXXXXX    SCHEDULING SAVE FUNCTIONS    XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#configuration of timing of data_save() function /defined at "FUNCTION DEFINES"/

s1.enter(3.1, 1, ALT_data_save, (endpoint_ALT_IN, endpoint_ALT_OUT, data_ALT, s1,))
s1.enter(3.01328125, 1, GPS_data_save, (endpoint_GPS, data_GPS, s1,))
s1.enter(3.15, 1, RAD_data_save, (endpoint_RAD_IN, data_RAD, s1,))
s1.enter(3.13, 1, BAR_data_save, (data_BAR, s1,))

s1.run()


#closing used files
data_GPS.close()
data_ALT.close()
data_RAD.close()
data_BAR.close()

#giving back control to kernel (not successful yet)
usb.util.release_interface(sensor[0], 0)
print "released"
try:
    converter.attach_kernel_driver(0)
except:
    True

sb.util.release_interface(sensor[0], 1)
print "released"
try:
    converter.attach_kernel_driver(1)
except:
    True

