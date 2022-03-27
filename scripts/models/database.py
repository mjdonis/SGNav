import rospy
import sqlite3
import datetime
import os
import math

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion

class Database:
    """Clase que maneja la base de datos (sqlite)"""
    def __init__(self):
        # crea archivo y conecta
        try:
            self.database_path = os.path.expanduser("~") + "/robot.db"
            rospy.loginfo("Creating database... (" + self.database_path + ")")
            conn = sqlite3.connect(self.database_path)
            rospy.loginfo("Database created")

            # crea tabla
            sql = """CREATE TABLE IF NOT EXISTS obstacle (
                        id integer PRIMARY KEY NOT NULL,
                        filename text NOT NULL,
                        timestamp text NOT NULL,
                        x integer NOT NULL,
                        y integer NOT NULL,
                        z integer NOT NULL,
                        angle integer NOT NULL
                    );"""

            try:
                conn.cursor().execute(sql)
                rospy.loginfo("Table created")

                conn.commit()
                conn.close()
            except sqlite3.Error as e:
                rospy.loginfo(e)

        except sqlite3.Error as e:
            rospy.loginfo(e)

    def save_image(self, current_pose, filename, timestamp):
        # crea conexion
        try:

            conn = sqlite3.connect(self.database_path)
            print("db created")

            rospy.loginfo("Getting pose data...")
            # tomar datos para db
            p = Point()
            p = current_pose.position
            x = p.x
            y = p.y
            z = p.z

            q = Quaternion()
            q = current_pose.orientation
            euler = euler_from_quaternion(q)
            angle = euler[2] / (math.pi * 180)

            sql = """INSERT INTO obstacle(filename, timestamp, x, y, z, angle)
                        VALUES(""" + str(filename) + "," + str(timestamp) + "," + str(x) + "," + str(y) + "," + str(
                z) + "," + str(angle) + """
                    );"""
            try:
                conn.cursor().execute(sql)
                conn.commit()

                rospy.loginfo("Saved to database")
            except sqlite3.Error as e:
                print(e)

            conn.close()

        except sqlite3.Error as e:
            print.info(e)