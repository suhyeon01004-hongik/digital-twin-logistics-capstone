import cv2
import numpy as np
import re
import rclpy
from cv_bridge import CvBridge
from pyzbar import pyzbar
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner_node')

        self.declare_parameter('image_topic', '/box_detector/roi_image')
        self.declare_parameter('show_image', False)

        self.image_topic = self.get_parameter('image_topic').value
        self.show_image = self.get_parameter('show_image').value

        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.qr_pub = self.create_publisher(String, '/qr_code_data', 10)
        self.viz_pub = self.create_publisher(Image, '~/scanned_roi', 1)

        self.scanned_qrs = set()

        self.get_logger().info(f'QR 스캐너 노드 (ROI 한정 최적화 버전) 시작: {self.image_topic}')

    def image_callback(self, msg):
        try:
            # ROI 이미지 디코딩
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # --- 병합된 전처리 로직 ---
            # 원본 화면 전체가 아닌, 좁은 ROI 조각에만 적용되므로 연산량이 매우 적음
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Adaptive Thresholding (조명 편차 보정)
            processed = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2,
            )

            # --- QR 인식 ---
            # 이진화가 완료된 고품질 흑백 이미지를 pyzbar에 넘김
            decoded_objects = pyzbar.decode(processed)

            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')

                # 인식된 위치 사각형(Polygon)을 원본에 표시 (디버깅용)
                points = obj.polygon
                if len(points) == 4:
                    pts = np.array(points, dtype=np.intp)
                    pts = pts.reshape((-1, 1, 2))
                    cv2.polylines(cv_image, [pts], True, (255, 0, 0), 2)

                self.process_logistic_data(qr_data)

            # 시각화 토픽 발행
            if self.viz_pub.get_subscription_count() > 0 or self.show_image:
                viz_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                self.viz_pub.publish(viz_msg)

                if self.show_image:
                    try:
                        cv2.imshow('Detected QR Box', cv_image)
                        cv2.waitKey(1)
                    except Exception:
                        pass

        except Exception as e:
            self.get_logger().error(f'QR 영상 처리 중 오류 발생: {e}')

    def process_logistic_data(self, data):
        if data in self.scanned_qrs:
            return

        compact_match = re.fullmatch(r'([A-D])([1-4])-(\d{3})', data)
        if compact_match:
            region, size, number = compact_match.groups()
            log_msg = (
                f'\n--- [새로운 택배 QR 인식 (ROI 스캔 성공)] ---\n'
                f'1. QR 번호    : {number}\n'
                f'2. 배송 지역  : {region}\n'
                f'3. 택배 크기  : {size}호\n'
                f'4. 원본 코드  : {data}\n'
                f'----------------------------'
            )
            self.get_logger().info(log_msg)

            self.scanned_qrs.add(data)
            self.qr_pub.publish(String(data=data))
            return

        fields = data.split(',')

        if len(fields) == 4:
            sender, receiver, hubs, size = fields
            log_msg = (
                f'\n--- [새로운 택배 정보 인식 (ROI 스캔 성공)] ---\n'
                f'1. 보내는 주소: {sender}\n'
                f'2. 받는 주소  : {receiver}\n'
                f'3. 거친 구역들: {hubs}\n'
                f'4. 택배 크기  : {size}\n'
                f'----------------------------'
            )
            self.get_logger().info(log_msg)

            self.scanned_qrs.add(data)
            self.qr_pub.publish(String(data=data))
        else:
            self.get_logger().warn(f'인식된 데이터 형식이 올바르지 않습니다: {data}')
            self.scanned_qrs.add(data)
            self.qr_pub.publish(String(data=data))


def main(args=None):
    rclpy.init(args=args)
    node = QRScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
