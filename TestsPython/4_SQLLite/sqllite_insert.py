import sqlite3

conn = sqlite3.connect('/home/pi/RaspberryCars/raspberrycars.db');

cur = conn.cursor();

print ("Contenu de la table avant l'insertion")
cur.execute("select * from car_status");
rows = cur.fetchall();
for row in rows :
    print (row)

cur.execute(
    "insert into car_status (gps_datetime, gps_longitude, gps_latitude, gps_speed) values ( ?, ?, ? , ? ) ",
    (4, 6.101010, 49.757575, 30)
)

conn.commit();

print ("Contenu de la table après l'insertion")
cur.execute("select * from car_status");
rows = cur.fetchall();
for row in rows :
    print (row);
    
conn.close();