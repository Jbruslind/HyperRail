import sqlite3
from pathlib import Path, PosixPath
from communication.constants import DB_PATH

dir = PosixPath(DB_PATH)
db = dir/"development.sqlite3"

def dict_factory(cursor, row):
    d = {}
    for idx, col in enumerate(cursor.description):
        d[col[0]] = row[idx]
    return d

"""Uses row_factory, this makes elements of each row returned by a get query
referencable by column name in addition to index"""
class DatabaseReader:

    def __init__ (self):
        self.db_file = db

        self.conn = None
        try:
            self.conn = sqlite3.connect(self.db_file.expanduser())
            self.conn.row_factory = sqlite3.Row
        except Exception as e:
            print(e)

    def get_all_waypoints(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM waypoints")

        rows = cur.fetchall()

        for row in rows:
            print(row)
        
        cur.close()

    def get_waypoint_headings(self):
        cur = self.conn.cursor()
        cur.execute("PRAGMA table_info('waypoints')")
        rows = cur.fetchall()

        for row in rows:
            print(row[1])

        cur.close()

    def get_waypoints_for_program(self, prog):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM waypoints WHERE program_id = ?;", (prog,))

        rows = cur.fetchall()

        for row in rows:
            print(tuple(row))
        
        cur.close()
        return rows

    def get_all_programs(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM programs")
        rows = cur.fetchall()
        for row in rows:
            print(row)
        cur.close()
        return rows
    
    def get_program_headings(self):
        cur = self.conn.cursor()
        cur.execute("PRAGMA table_info('programs')")
        rows = cur.fetchall()

        for row in rows:
            print(row)

        cur.close() 


    def __del__(self):
        self.conn.close()

if __name__ == "__main__":
    db = DatabaseReader()
    # db.get_program_headings()
    # db.get_all_programs()
    db.get_waypoints_for_program(5)
    # db.get_waypoint_headings()
    # db.get_all_waypoints()
