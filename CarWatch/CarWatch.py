#!/usr/bin/env python3

import serial
import serial.tools.list_ports;

import RPi.GPIO as GPIO
import os, time

from decimal import Decimal
from datetime import datetime
from random import randrange

from simple_salesforce import Salesforce

import sqlite3
import _thread
import threading

import obd

import sys #for color output

#sqllite db
#CREATE TABLE car_status(carnumber text, statusdate integer, longitude Decimal(9,6) , latitude Decimal(9,6) , sent boolean, speed decimal(9,2), elevation decimal(9,2));
#CREATE TABLE car (carnumber text);


# definition de constantes pour changer l'affichage couleur
C_DB = "\x1b[0;32m"
C_GPS = "\x1b[0;33m"
C_OBD= "\x1b[0;36m"
C_SF = "\x1b[0;35m"
C_GPS_READ = "\x1b[0;93m"
C_ERROR= "\x1b[0;31m"
# definition de constantes pour remettre la couleur a normal
ENDC = "\x1b[0;0m"


# constantes pour afficher le debug pour chaque thread
FULL_DEBUG_GPS = False
SMALL_DEBUG_GPS = False
DEBUG_GPS = True

DEBUG_OBD = True

MODE_SHARED_DB_CONNECTION = True
MODE_STORE_LAST_POSITION = False

# constantes pour indiquer à SF quelle voiture est suivie
# CAR_NUMBER = "AZ 4242" #"AZ 2424"


# ***************************************************************************    
# ***************************************************************************    
# la lecture du port GPS
# ***************************************************************************    
# ***************************************************************************      


def watch_gps( gpsPort):
    # les variables dans lesquelles la thread gps stoque ce qu'elle a trouvé
    global lastGpsPositionMeasured;
    global lastGpsLongitude;
    global lastGpsLattitude;
    global lastGpsSpeed
    global lastGpsElevation;
    global lastGpsMeasureTime;
    global lastGpsTime

    # ?
    GPIO.setmode(GPIO.BOARD)

    # Enable Serial Communication
    port = serial.Serial(gpsPort, 115200);
    port.timeout=1;

    print ("GPS : Starting Up Serial Monitor")

    # on vide toutes les entrees et sorties du ports USB pour la carte GPS
    port.flushInput()
    port.flushOutput()

    # la sequence d initialisation
    
    port.write('AT\r\n'.encode('ascii'))
    rcv = port.read(100)
    print(C_GPS_READ+"GPS : ",rcv, ENDC)
    time.sleep(.1)

    port.write('AT+CGNSPWR=1\r\n'.encode('ascii'))     # to power the GPS
    rcv = port.read(100)
    print(C_GPS_READ+"GPS : ",rcv, ENDC)
    time.sleep(.1)

    port.write('AT+CGNSIPR=115200\r\n'.encode('ascii')) # Set the baud rate of GPS
    rcv = port.read(100)
    print(C_GPS_READ+"GPS : ",rcv, ENDC)
    time.sleep(.1)

    port.write('AT+CGNSTST=1\r\n'.encode('ascii'))    # Send data received to UART
    rcv = port.read(100)
    print(C_GPS_READ+"GPS : ",rcv, ENDC)
    time.sleep(.1)

    port.write('AT+CGNSINF\r\n'.encode('ascii'))       # Print the GPS information
    rcv = port.read(200)
    print(C_GPS_READ+"GPS : ",rcv, ENDC)
    time.sleep(.1)
    
    ck=1
    while ck==1:
        # lecture de la prochaine ligne depuis la carete GPS
        line = port.read_until("\r\n".encode('ascii'));
        lineStr = line.decode('ascii');

        # en mode debug complet, on affiche chaque ligne renvoyée par la carte 
        if FULL_DEBUG_GPS == True :
            if (len(lineStr)>2):
                lineStr2 = lineStr[0: len(lineStr)-2];        
                print(C_GPS_READ+"GPS :",lineStr2, ENDC);
        
        # si la ligne est une ligne de position GPS (commencent par $GNRMC
        if (lineStr.startswith("$GNRMC")) :
            # on extrait la ligne sans les deux characteres de fin de ligne
            lineStr = lineStr[0: len(lineStr)-2];
            
            # en mode debug GPS seulement, on affiche ces lignes
            if SMALL_DEBUG_GPS == True :
                print(C_GPS_READ+"GPS :",lineStr, ENDC);
                
            # ion decoupe la ligne selon les morceaux séparés par des virgules
            items = lineStr.split(",");
            
            # on recupere les moreceaux de la ligne qui correspondent au champs utiles
            timeStr = items[1]; # l'heure connue par le GPS
            lat = items[3];     # la lattitude
            lon = items[5];     # la longitude
            speed = items[7];   # la vitesse
            elevation=items[8]  # l'altitude
            dateStr = items[9]; # la date connue par le GPS
            
            # si les champs longitude et lattitude ne sont pas vide, alors on va memoriser les
            if ((len(lat)>0) and (len(lon)>0)) :
                # on decode la latitude comme dans le programme d'exemple
                s3=lat[2:len(lat)]
                s3=Decimal(s3)
                s3=s3/60
                s33=int(lat[0:2])
                s3 = s3 + s33
                s3 = int(s3*1000000)/1000000

                # on decode la longituide comme dans le programme d'exemple
                s5=lon[3:len(lon)]
                s5=Decimal(s5)
                s5=s5/60
                s55=int(lon[0:3])
                s5 = s5 + s55
                s5 = int(s5*1000000)/1000000

                # on crée une chaine datetime au format ISO avec les morceaux optebnus de la carte,
                # et on la fait analyser par le datetime python
                theDatetime = datetime.strptime(dateStr+'-'+timeStr+'000Z+0000',"%d%m%y-%H%M%S.%fZ%z");
                # on cree le timestamp (Unix) correspondant : le nombre de secondes depuis le 1/1/1970
                theDateTimeTimestamp = datetime.timestamp(theDatetime);
                
                # on decode la vitesse , et on la convertit de mph to kph
                theSpeed = float(speed)*1.609; 
                
                #on decode l'altitude
                theElevation = float(elevation);
                
                # en mode debug standard, on fait un joli affichage des infos GPS
                # la constante C_GPS permet d'afficher dans une couleur particulière pour le GPS (facile à lire)
                if DEBUG_GPS == True :
                   print(C_GPS+"--------------------------------------------", ENDC)
                   print(C_GPS+"GPS :  TIME  :", datetime.fromtimestamp(theDateTimeTimestamp), ENDC )
                   print(C_GPS+"GPS :  LAT.  :", s3 , ENDC)
                   print(C_GPS+"GPS :  LONG. :", s5 , ENDC)
                   print(C_GPS+"GPS :  ELEV  :", theElevation , ENDC)
                   print(C_GPS+"GPS :  SPEED :", theSpeed , ENDC)
                   print(C_GPS+"--------------------------------------------", ENDC)
                
                # on va placer les valeurs lues dans les variables globales
                # mais pour eviter qu'une autre thread les lise pendant qu'on les modifie on demande le verrou
                # et on le libère une fois les valeurs stoquées
                threadGpsLock.acquire()
                lastGpsPositionMeasured = True
                lastGpsMeasureTime = int(datetime.timestamp(datetime.now())); 
                lastGpsLongitude = s5;
                lastGpsLattitude = s3;
                lastGpsSpeed = theSpeed;
                lastGpsElevation = theElevation;
                lastGpsTime = int(theDateTimeTimestamp) 
                threadGpsLock.release()
                
# ***************************************************************************    
# ***************************************************************************    
# la lecture du port OBD
# ***************************************************************************    
# ***************************************************************************      
       
def watch_odb(obdPort, delay):
    # les variables dans lesquelles la thread gps stoque ce qu'elle qa trouvé
    global lastObdMeasured;
    global lastObdMeasureTime;
    global lastObdSpeed
    global lastObdFuelLevel
    global lastObdRpm
    
    # l'ouvertur de la connection obd sur le port
    connection = obd.OBD(obdPort)
    
    # tant que l'on n'est pas interrompu
    while True:
        try:
            # on lit les trois valeurs qui nous interesse (vitesse, les tours par minute, le niveau de carburant
            response_speed = connection.query(obd.commands.SPEED) 
            response_rpm = connection.query(obd.commands.RPM ) 
            response_fuellevel = connection.query(obd.commands.FUEL_LEVEL ) 
     
            # si on est en mode debug, on affiche le detail
            if DEBUG_OBD == True:
                print(C_OBD+ "--------------------------------------------", ENDC)
                print(C_OBD+ "ODB : SPEED :", response_speed.value, ENDC) 
                print(C_OBD+ "ODB : RPM   :", response_rpm.value, ENDC) 
                print(C_OBD+ "ODB : FUEL  :", response_fuellevel.value, ENDC) 
                print(C_OBD+ "--------------------------------------------", ENDC)

            # on essaye de decoder les trois reponses
            try :
                speedValue = response_speed.value.magnitude;
            except:
                speedValue = -1
                
            try :
                rpmValue = response_rpm.value.magnitude;
            except:
                rpmValue = -1
                
            try :
                fuelLevelValue = response_fuellevel.value.magnitude;
            except:
                fuelLevelValue = -1
            
            # on va placer les valeurs lues dans les variables globales
            # mais pour eviter qu'une autre thread les lise pendant qu'on les modifie on demande le verrou
            # et on le libère une fois les valeurs stoquées           
            threadObdLock.acquire()
            lastObdMeasured = True
            lastObdMeasureTime = int(datetime.timestamp(datetime.now())) 
            lastObdSpeed = speedValue;
            lastObdRpm = rpmValue;
            lastObdFuelLevel = fuelLevelValue;
            threadObdLock.release()
        except Exception as e:
            print(C_ERROR+"OBD : Error: unable to getOBD Data", ENDC)
            print(C_ERROR+str(e), ENDC)

        time.sleep(delay)
    
    
# ***************************************************************************    
# ***************************************************************************    
# le programme principal
# ***************************************************************************    
# ***************************************************************************    
    
    
# les ports usb serie utilisés par les deux cartes
gpsPortDevice = "";             
obdPortDevice = "";
     
print("******************");
print("SETUP");
print("******************");
for port in serial.tools.list_ports.comports() :
    print ("Device :", port.device) 
    print ("Name   :", port.name) 
    print ("Desc   :", port.description) 
    print ("")
#    if (port.description.startswith("OBDLink SX")) :
#        obdPortDevice = port.device
    if (port.description.startswith("CP2102 USB to UART Bridge Controller")) :
        gpsPortDevice = port.device
    if (port.description.startswith("OBDLink SX")) :
        obdPortDevice = port.device

threadGpsLock = threading.Lock()
threadObdLock = threading.Lock()
#threadDbLock = threading.Lock()


try:

    # comme on a trouve une carte GPS, on peut lancer le thread de lecture sur le port GPS
    if gpsPortDevice != "":
        _thread.start_new_thread( watch_gps, (gpsPortDevice,  ) )
        
    # watch odb every second
    if obdPortDevice != "":
        _thread.start_new_thread( watch_odb, (obdPortDevice, 1) )
        
    #store infos every 2 seconds = 10 is ok
  ##  _thread.start_new_thread( store_pos_in_db, ("Thread-DB", 2, ) )
    
    # send info every 5 minutes
  ##  _thread.start_new_thread( send_data_to_sf, ("Thread-SF", 300, True, ) )
    
except Exception as e:
   print(C_ERROR+ "Error: unable to start one of the thread", ENDC)
   print(C_ERROR+ str(e), ENDC)

# une boulce infinie tant que le programme n'est pas interrompu depuis l'exterieur
while 1:
   pass
