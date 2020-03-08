import sqlite3

conn = sqlite3.connect('/home/pi/RaspberryCars/raspberrycars.db');

cur = conn.cursor();
cur.execute("select * from car_status");

rows = cur.fetchall();

for row in rows :
    print (row)