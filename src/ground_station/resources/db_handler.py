import sqlite3, os

PWD = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.join(PWD, 'gps_data.db')

conn = sqlite3.connect(DB_PATH)
c = conn.cursor()

# Create table
try:
    c.execute('''CREATE TABLE competition
                (pointid text, lat_dec real, lat_hem text,
                 long real, ))
