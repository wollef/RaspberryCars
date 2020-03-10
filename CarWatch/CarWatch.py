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
SMALL_DEBUG_GPS = True
DEBUG_GPS = True

DEBUG_OBD = True

MODE_SHARED_DB_CONNECTION = True
MODE_STORE_LAST_POSITION = False

# constantes pour indiquer le numero du capteur (si on a plusieurs Raspberry qui execute le programme :
# chacun devra avoir une valeur differente
# les données ne devront pas se melanger dans SF)
RASPBERRY_NUMBER = "RASP_1"

CAR_DB = "/home/pi/RaspberryCars/Database/raspberrycars.db"

# ***************************************************************************    
# ***************************************************************************    
# la lecture du port GPS
# ***************************************************************************    
# ***************************************************************************      


def watch_gps( gpsPort):
    # la demande d'utilisation des variables dans lesquelles la thread gps stoque ce qu'elle a trouvé
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
                # on indique qu'on a eu une nouvelle mesure reussie
                lastGpsPositionMeasured = True
                # on memorise l'heure
                lastGpsMeasureTime = int(datetime.timestamp(datetime.now()));
                # on memorise le resultta du GPS
                lastGpsLongitude = s5;
                lastGpsLattitude = s3;
                lastGpsSpeed = theSpeed;
                lastGpsElevation = theElevation;
                lastGpsTime = int(theDateTimeTimestamp)
                # et on  libère le verrou une fois les valeurs stoquées           
                threadGpsLock.release()
                
# ***************************************************************************    
# ***************************************************************************    
# la lecture du port OBD
# ***************************************************************************    
# ***************************************************************************      
       
def watch_odb(obdPort, delay):
    # la demande d'utilisation des variables dans lesquelles la thread gps stoque ce qu'elle qa trouvé
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
            threadObdLock.acquire()
            # on indique qu'on a eu une nouvelle mesure reussie
            lastObdMeasured = True
            # on memorise l'heure
            lastObdMeasureTime = int(datetime.timestamp(datetime.now()))
            # on memorise le resultta du OBD
            lastObdSpeed = speedValue;
            lastObdRpm = rpmValue;
            lastObdFuelLevel = fuelLevelValue;
            # et on  libère le verrou une fois les valeurs stoquées           
            threadObdLock.release()
        except Exception as e:
            print(C_ERROR+"OBD : Error: unable to getOBD Data", ENDC)
            print(C_ERROR+str(e), ENDC)

        time.sleep(delay)
    
    
# ***************************************************************************    
# ***************************************************************************    
# l ecriture dans la DB Locale
# ***************************************************************************    
# ***************************************************************************        
    
def store_pos_in_db(delay):

    # la declaration de partage des variables memoriséé par le thread GPS
    global lastGpsPositionMeasured;
    global lastGpsLongitude;
    global lastGpsLattitude;
    global lastGpsSpeed
    global lastGpsElevation;
    global lastGpsMeasureTime;
    global lastGpsTime;

    # la declaration de partage des variables memoriséé par le thread OBD
    global lastObdMeasured;
    global lastObdMeasureTime;
    global lastObdSpeed
    global lastObdFuelLevel
    global lastObdRpm


    # initialisation des variables GPS
    lastGpsPositionMeasured = False;
    lastGpsLongitude = 0;
    lastGpsLattitude =0;
    lastGpsSpeed =0;
    lastGpsElevation =0;
    lastGpsMeasureTime =0;
    lastGpsTime = 0
    
    # initialisation OBD
    lastObdMeasured = False;
    lastObdMeasureTime = 0;
    lastObdSpeed = 0;
    lastObdFuelLevel= 0;
    lastObdRpm= 0;

    # deux mode possibles :
    # - un ou on garde la connection DB active entre chaque attente
    # - un ou on la referme a chaque fois
    if MODE_SHARED_DB_CONNECTION == True:
        permanentDBConnectionB = sqlite3.connect(CAR_DB, isolation_level=None);
    
    # on boucle en permanence
    while True:
        # on essaye d'ecrie vers la DB
        print("DB: Try to store position");
        
        # on bloque le verrou sur les variables partagées
        threadGpsLock.acquire()
        threadObdLock.acquire()        

        # on memorise l'heure courante (que l'on va memoriser : la dernière mesure est faite depuis quelques secondes
        statustime=int(datetime.timestamp(datetime.now()))

        # un affichage pour le debug
        print (C_DB+ "--------------------------------------------", ENDC)
        print (C_DB+ "DB: db.time  :", datetime.utcfromtimestamp(statustime).strftime('%Y-%m-%d %H:%M:%S') , ENDC)
        print (C_DB+ "--------------------------------------------", ENDC)
        print (C_DB+ "DB: g.status :", lastGpsPositionMeasured, ENDC)
        print (C_DB+ "DB: g.time   :", datetime.utcfromtimestamp(lastGpsMeasureTime).strftime('%Y-%m-%d %H:%M:%S'), ENDC)
        print (C_DB+ "DB: g.lat.   :", lastGpsLattitude, ENDC)
        print (C_DB+ "DB: g.long   :", lastGpsLongitude, ENDC)
        print (C_DB+ "DB: g.speed  :", lastGpsSpeed, ENDC)
        print (C_DB+ "DB: g.elev   :", lastGpsElevation, ENDC)
        print (C_DB+ "DB: g.gtime  :", datetime.utcfromtimestamp(lastGpsTime).strftime('%Y-%m-%d %H:%M:%S'), ENDC)
        print (C_DB+ "--------------------------------------------", ENDC)
        print (C_DB+ "DB: o.status :", lastObdMeasured, ENDC)
        print (C_DB+ "DB: o.time   :", datetime.utcfromtimestamp(lastObdMeasureTime).strftime('%Y-%m-%d %H:%M:%S'), ENDC)
        print (C_DB+ "DB: o.speed  :", lastObdSpeed, ENDC)
        print (C_DB+ "DB: o.rpm    :", lastObdRpm, ENDC)
        print (C_DB+ "DB: o.fuel   :", lastObdFuelLevel, ENDC)
        print (C_DB+ "--------------------------------------------", ENDC)

        # si un des threads a une nouvelle donnée
        if lastGpsPositionMeasured == True or lastObdMeasured == True :
 
            # on bloque l'accès à la DB, pour que le threqd SF ne trqvqille pqs en meme temps
            threadDbLock.acquire()
    
            # si on est dans le mode DB en permanence active, on utilise la connection, sinon on l'ouvre
            if MODE_SHARED_DB_CONNECTION == True:
                conn = permanentDBConnectionB
            else :
                print(C_SF+"DB : Connect to DB", ENDC);
                conn = sqlite3.connect(CAR_DB, isolation_level=None);

            print("DB : Store position");
            # on cree le texte de la requete pour inserer dans la base
            sql = ''' INSERT INTO car_status(statusdate, gps_longitude , gps_latitude, sent, gps_speed, gps_elevation, obd_speed, obd_fuellevel, obd_rpm, gps_measure_time, obd_measure_time, gps_time)
                  VALUES(?,?,?,?,?,?,?,?,?,?,?,?) '''
         
            # on ouvre le curseur vers la DB, et on lui demande d'executer la requete avec ces paramètres
            cur = conn.cursor()
            cur.execute(sql, (statustime, lastGpsLongitude, lastGpsLattitude, 0, lastGpsSpeed, lastGpsElevation, lastObdSpeed,lastObdFuelLevel,lastObdRpm,lastGpsMeasureTime,lastObdMeasureTime,lastGpsTime))
            
            # si on est dans le mode où on ouvre et ferme la connection à chaqiue fois, on la referme
            if MODE_SHARED_DB_CONNECTION == False:
                conn.close()
                print("DB : Connection closed");
            # on libere le verrou d'accès à la DB (le thread SF peut maintenant lire
            threadDbLock.release()

        # on lib ere les verrous d'accès au variables pour que les thread OBD et GPS puissent y écrire
        threadGpsLock.release()
        threadObdLock.release()
        
        time.sleep(delay)

# ***************************************************************************    
# ***************************************************************************    
# le programme d'envoi à SF de ce qui est dans la base locale
# ***************************************************************************    
# ***************************************************************************        

def send_data_to_sf(delay):

    # if mode shared DB : create connection une fois
    if MODE_SHARED_DB_CONNECTION == True:
        permanentDBConnectionA = sqlite3.connect(CAR_DB, isolation_level=None);


    while True:
        # on demande l'acces exclusif à la DB
        threadDbLock.acquire()
        print(C_SF+"SF : Got DB lock", ENDC);

        try:
            # if mode shared DB : crée connection à chaque fois
            if MODE_SHARED_DB_CONNECTION == True:
                conn = permanentDBConnectionA
            else :
                print(C_SF+"SF : Connect to DB", ENDC);
                conn = sqlite3.connect(CAR_DB, isolation_level=None);

            print(C_SF+"SF : Look for new Status", ENDC);
            
            # on demande un curseur sur la DB
            cursor = conn.cursor()
            # on lit les lignes qu'on n'a pas encore envoyée (  where sent != 1)
            cursor.execute("SELECT rowid, statusdate, gps_longitude , gps_latitude, gps_speed, gps_elevation, obd_speed, obd_fuellevel, obd_rpm,  gps_measure_time, obd_measure_time, gps_time FROM car_status where sent != 1 order by statusdate asc")
            rows = cursor.fetchall()
            # on referme le curseru
            cursor.close()

            # on regarde combien de mesure on a eu
            count = 0;
            for row in rows:
                count = count + 1

            # si il y a des lignes, on va les mettre dans SF
            if count>0 :
                print(C_SF+"SF : Status to transfer to SF :" , count, ENDC);

                print(C_SF+'SF : Connect to SF', ENDC)

                #on se connecte à SF
                sf = Salesforce(
  
                );

                # on cree :
                # - un tableau new_positions avec toutes les positions au format SF
                # - un tableau rowsToMark avec les lignes à marquer comme lues
                print(C_SF+"SF : Prepare new position objects", ENDC);
                new_positions = []
                last_positions = []
                rowsToMark = []
                for row in rows:
                    # conversion de format pour les dates
                    datestring = datetime.utcfromtimestamp(row[1]).strftime('%Y-%m-%dT%H:%M:%SZ')
                    gmtime = datetime.utcfromtimestamp(row[9]).strftime('%Y-%m-%dT%H:%M:%SZ')
                    omtime = datetime.utcfromtimestamp(row[10]).strftime('%Y-%m-%dT%H:%M:%SZ')
                    gtime = datetime.utcfromtimestamp(row[11]).strftime('%Y-%m-%dT%H:%M:%SZ')
                    
                    new_positions.append({
                        'Tracker_ID__c': RASPBERRY_NUMBER,
                        'Requested_On__c': datestring,
                        'GPS_Longitude__c' : row[2],
                        'GPS_Latitude__c' : row[3],
                        'GPS_Speed__c' : row[4],
                        'GPS_Elevation__c' : row[5],                
                        'OBD_Speed__c' : row[6],                
                        'OBD_Fuel_Level__c' : row[7],                
                        'OBD_RPM__c' : row[8],
                        'GPS_Measure_Time__c' : gmtime,
                        'OBD_Measure_Time__c' : omtime,
                        'GPS_Time__c' : gtime,
                        
                    })
                        
                    rowsToMark.append((1,row[0]));
                
                # on demande à SF d'inserer toutes les nouvelles lignes
                print(C_SF+'SF : Send Big Object Positions', ENDC)
                print(C_SF+'SF : All Positions are : ',new_positions, ENDC)
                createdPositions1 = sf.bulk.Car_Monitoring_Data__b.insert(new_positions)
                
                # on dit à la DB que toutes ces lignes ont éte utilisées
                print(C_SF+'SF : Update database records', ENDC)
                print(C_SF+'SF : Records to update are : ',rowsToMark, ENDC)
                cursor = conn.cursor()
                cursor.executemany('UPDATE car_status SET sent=? WHERE rowid=?', (rowsToMark) )
                cursor.close()

                # on demande à la DB de valider la sauvearde des lignes
                print(C_SF+'SF : Commit changes in DB', ENDC)
                conn.commit

                print(C_SF+'SF : Transfer to SF done', ENDC)
            else :
                print(C_SF+'SF : Nothing to transfer to SF', ENDC)

            # close DB connection si one est en mode de connection non permanente
            if MODE_SHARED_DB_CONNECTION == False:
                conn.close()
                print("SF : DB Connection closed");

        except Exception as e:
            print(C_ERROR+"SF : Error: unable to send data to SF", ENDC)
            print(C_ERROR+str(e), ENDC)

        # on libere l'accès à la DB pour qu'elle puisse etre mise à jour par les autres taches
        threadDbLock.release()
        print(C_SF+"SF : DB lock released", ENDC);

        # on attend le delai demandé
        print(C_SF+'SF : Wait', ENDC)
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
threadDbLock = threading.Lock()


try:

    # comme on a trouve une carte GPS, on peut lancer le thread de lecture sur le port GPS
    if gpsPortDevice != "":
        _thread.start_new_thread( watch_gps, (gpsPortDevice,  ) )
        
    # watch odb every second
    if obdPortDevice != "":
        _thread.start_new_thread( watch_odb, (obdPortDevice, 1, ) )
        
    #store infos every 2 seconds = 10 is ok
    _thread.start_new_thread( store_pos_in_db, (2, ) )
    
    # send info every 5 minutes
    _thread.start_new_thread( send_data_to_sf, (300, ) )
    
except Exception as e:
   print(C_ERROR+ "Error: unable to start one of the thread", ENDC)
   print(C_ERROR+ str(e), ENDC)

# une boulce infinie tant que le programme n'est pas interrompu depuis l'exterieur
while 1:
   pass
