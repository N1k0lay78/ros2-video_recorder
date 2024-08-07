#!/usr/bin/python

import rclpy
import cv2
import getpass
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from datetime import datetime
from cv_bridge import CvBridge


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        
        self.declare_parameter('from_topic', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('fps', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('directory', rclpy.Parameter.Type.STRING) 

        self.VIDEO_TOPIC = self.get_parameter('from_topic').value
        if not self.VIDEO_TOPIC:
            self.get_logger().error(f"topic with video not specified")
            raise Exception(f"topic with video not specified")
        
        self.FPS = self.get_parameter('fps').value
        if self.FPS <= 0:
            self.get_logger().error(f"video stream speed (FPS) not specified")
            raise Exception(f"video stream speed (FPS) not specified")
        
        path = self.get_parameter('directory').value
        if not path:
            self.get_logger().error(f"directory not specified")
            raise Exception(f"directory not specified")
        # если указан не абсолютный путь, то задаём папку в директории пользователя
        elif not path.startswith("/home"):
            # с проверкой / в начале не абсолютного пути, если его там нет то добавляем
            path = f"/home/{getpass.getuser()}{'' if path.startswith('/') else '/'}{path}"
        # добавляем / в конец, если его там нет то добавляем
        path = f"{path}{'' if path.endswith('/') else '/'}"
        # создаём папку, если она не существует
        if not os.path.isdir(path):
            os.makedirs(path)
        self.PATH = path

        self.subscription = self.create_subscription(Image, self.VIDEO_TOPIC, self.image_callback, 1)
        self.video_saver = self.create_timer(1 / self.FPS, self.update)
        # начал записывать видео
        self.start_write = False
        # последнее полученное изображение
        self.frame = None

        self.get_logger().info(f"video recorder ready to write video in directory {self.PATH}")

    def create_video_stream(self, size):
        # создаём файл только после того, как узнали разрешение видео
        self.bridge = CvBridge()
        filename = f"{datetime.today().strftime('%Y-%m-%d %H:%M:%S')}.mp4"
        self.video_writer = \
            cv2.VideoWriter(
                filename = f"{self.PATH}{filename}",  # создаём название файла с теущим временем
                fourcc = cv2.VideoWriter_fourcc(*'mp4v'),
                fps = self.FPS,
                frameSize = size,
            )
        self.start_write = True
        self.get_logger().info(f"start record video at file {filename}")

    def image_callback(self, msg):
        if not self.start_write:
            # создаём файл в который записываем видео после того узнали разрешение видео (получили первый кадр)
            self.create_video_stream((msg.width, msg.height))
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.frame = cv_image

    def update(self):
        # сохраняем видео с определённой частотой кадров независимо от того пришёл новый кадр или нет
        # если не пришёл, то будет сохраняться предыдущий
        if self.frame is not None and self.start_write:
            self.video_writer.write(self.frame)

    def destroy_node(self):
        self.video_writer.release()
        super().destroy_node()

def main():
    rclpy.init()
    video_recorder = VideoRecorder()
    rclpy.spin(video_recorder)
    video_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()