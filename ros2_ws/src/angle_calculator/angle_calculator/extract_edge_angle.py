#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import tf_transformations

class LineAngleHeadingComparator(Node):
    def __init__(self):
        super().__init__('line_angle_heading_comparator')
        self.calculation_active = False
        self.ref_angle = None
        self.current_heading = None

        # 구독자 설정
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)
        self.create_subscription(Empty, 'end_flag', self.end_flag_callback, 10)
        
        # RealSense 파이프라인 초기화 (컬러 스트림)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        
        # 10Hz 타이머 설정 (0.1초 주기)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("라인 각도 및 heading 비교 노드 시작 - goal_pose 수신 대기 중")

    def extract_line_angle(self, image):
        """
        이미지에서 가장 긴 직선을 검출하고 해당 선의 각도를 계산합니다.
        검출된 선은 이미지에 그려서 반환합니다.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
        if lines is None:
            return None, image

        longest_line = None
        max_length = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = math.hypot(x2 - x1, y2 - y1)
            if length > max_length:
                max_length = length
                longest_line = (x1, y1, x2, y2)
        if longest_line is None:
            return None, image

        x1, y1, x2, y2 = longest_line
        angle = math.degrees(math.atan2((y2 - y1), (x2 - x1)))
        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return angle, image

    def goal_pose_callback(self, msg):
        self.get_logger().info("goal_pose 수신: 계산 시작")
        self.calculation_active = True

        # 기준 프레임 캡처 및 선 각도 추출
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().error("기준 프레임 캡처 실패")
            return
        color_image = np.asanyarray(color_frame.get_data())
        self.ref_angle, _ = self.extract_line_angle(color_image.copy())
        if self.ref_angle is None:
            self.get_logger().warn("기준 프레임에서 선을 검출하지 못했습니다.")
        else:
            self.get_logger().info("기준 선 각도: {:.2f}°".format(self.ref_angle))

    def amcl_pose_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]  # 라디안 단위
        self.current_heading = math.degrees(yaw)
        self.get_logger().info("현재 추정 heading (/amcl_pose): {:.2f}°".format(self.current_heading))

