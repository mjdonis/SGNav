import rospy
import os
import datetime
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion

from .database import Database

class Camera:
    """Manejo de la camara (generico)"""
    def __init__(self):
        # crea subscriber al topic de la camara para tomar imagen
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, callback=self.get_image)
        rospy.loginfo("Subscribed to camera topic")

        # flag para saber cuando hay que sacar una foto (async)
        self.take_image = False
        # bridge para convertir tipo sensor_msgs.Image (topic) a matriz opencv
        self.bridge = CvBridge()

        rospy.loginfo("Creating images directory...")
        # crear dir donde guarda las img
        self.image_path = os.path.join(os.path.expanduser("~"), 'robot_images')

        # para guardar la pose actual
        self.current_pose = Pose()

        # conexion con la db para guardar las imagenes
        self.db = Database()

        # si ya existe, no lo crea de vuelta
        try:
            os.mkdir(self.image_path)
            rospy.loginfo("Directory created (" + self.image_path + ")")
        except FileExistsError:
            rospy.loginfo("Images directory already exists (" + self.image_path + ")")

    def set_current_pose(self, pose):
        """Guarda en current_pose la pose enviada como parametro"""
        self.current_pose = pose

    def get_image(self, message):
        """Funcion callback del subscriber, se llama cada vez que llega un mensaje al topic
           Si take_flag = True, se debe "sacar una foto", es decir guardar el siguiente mensaje en el topic
           devuelve una lista con el nombre del archivo y el timestamp para la db"""
        # si el flag es False, no hay que hacer nada
        # si es True, "saca la foto"
        if self.take_image:
            rospy.loginfo("Getting obstacle image...")

            # toma mensaje del topic (imagen de la camara)
            image = message.data
            # convierte el tipo de dato sensor_msgs.Image a uno que pueda manejar opencv
            image_cv = self.bridge.imgmsg_to_cv2(image)

            # guardar imagen como png con opencv y timestamp
            timestamp = datetime.datetime.now().strftime("%Y/%m/%d_%H-%m-%S")
            filename = self.image_path + "/image_" + timestamp + ".png"
            cv2.imwrite(filename, image_cv)
            rospy.loginfo("Saved image: " + filename)

            # guarda imagen y datos en la db
            self.db.save_image(self.current_pose, filename, timestamp)

            # vuelve a dejar el flag en False hasta que haya que guardar otra foto
            self.take_image = False