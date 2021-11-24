import sqlite3
import time
from datetime import datetime
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

    def create_program_run(self, program_id):
        cur = self.conn.cursor()
        timestamp = datetime.fromtimestamp(time.time())
        cur.execute("INSERT into program_runs(program_id,started_at) VALUES(?,?)", (program_id, timestamp))

        return cur.lastrowid

    def get_all_program_runs(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM program_runs")

        rows = cur.fetchall()
        for row in rows:
            print(row)
        
        cur.close()

    def get_program_run_headings(self):
        cur = self.conn.cursor()
        cur.execute("PRAGMA table_info('program_runs')")
        rows = cur.fetchall()

        for row in rows:
            print(row[1])

        cur.close()


    def get_program_runs(self, program_id):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM program_runs WHERE program_id=?", (program_id,))

        rows = cur.fetchall()
        for row in rows:
            print(tuple(row))
        
        self.conn.commit()
        cur.close()

    def image_path(self, run_waypoint_id):
        cur = self.conn.cursor()
        cur.execute("SELECT uri FROM camera_images WHERE run_waypoint_id=?", (run_waypoint_id,))
        path = cur.fetchall()
        cur.close()
        return path
    
    def get_image_paths(self, program_run_id):
        cur = self.conn.cursor()
        cur.execute("SELECT id FROM run_waypoints WHERE program_run_id=?", (program_run_id,))
        rows = cur.fetchall()
        images = []
        for row in rows:
            images.append(row['id'])
        print(images)

        cur.execute(f"SELECT uri FROM camera_images WHERE run_waypoint_id IN ({','.join(['?']*len(images))})", images)
        run_paths = cur.fetchall()
        paths = []
        for p in run_paths:
            print(p['uri'])
            paths.append(p['uri'])
        
        cur.close()
        return paths

    # Make fake records for testing
    def create_test_images(self):
        cur = self.conn.cursor()
        cur.execute("DELETE from camera_images")
        id = [97, 98, 99, 100]
        path = ["4.png", "5.png", "6.png", "7.png", "8.png"]
        c = "camera_mock"
        it = "image_mock"
        for i, p in zip(id, path):
            t = datetime.fromtimestamp(time.time()) 
            cur.execute ("INSERT into camera_images(run_waypoint_id, camera_name, image_type, uri, created_at) VALUES(?,?,?,?,?)", (i, c, it, p, t,))
            self.conn.commit()
        cur.close
    

    def add_image(self, camera_image):
        cur = self.conn.cursor()
        t = datetime.fromtimestamp(time.time()) 
        cur.execute ("INSERT into camera_images(\
            run_waypoint_id,\
            camera_name,\
            image_type,\
            uri,\
            metadata,\
            created_at) VALUES(?,?,?,?,?,?)",
            (camera_image['run_waypoint_id'],
             camera_image['camera_name'],
             camera_image['image_type'],
             camera_image['uri'],
             camera_image['metadata'],
             t,))
        self.conn.commit()
        cur.close

    def get_images(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * from camera_images")
        images = cur.fetchall()
        print(images)
        for i in images:
            print(i['run_waypoint_id'])


    def __del__(self):
        self.conn.close()


if __name__ == "__main__":
    db = DatabaseReader()
    # db.get_program_headings()
    # db.get_all_programs()
    # db.get_waypoints_for_program(5)
    # db.get_waypoint_headings()
    # db.get_all_waypoints()
    # db.get_all_program_runs()
    # db.get_program_run_headings()
    # print(db.create_program_run(5))
    # db.get_program_runs(5)
    db.create_test_images()
    db.get_images()
    db.get_image_paths(10)
# requests.post()