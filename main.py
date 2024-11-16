import rospy
import cv2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class Kamera:
    def __init__(self):
        """Kamera ile nesne algılama."""
        self.bridge = CvBridge()
        self.model = YOLO("yolov8x.pt")  # YOLO modelini yükle
        self.target = False

        # Kameradan görüntü almak için abone ol
        rospy.Subscriber("/wide_stereo/right/image_raw", Image, self.image_callback)


    def image_callback(self, msg):
        """Kameradan gelen görüntüyü işler."""
        try:
            # ROS mesajını OpenCV formatına dönüştür
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLO ile nesne algılama
            results = self.model(frame)
            detections = results[0]

            # Algılanan nesneleri kontrol et
            self.target = False  # Her işlemde başlangıçta False olarak ayarla
            for box in detections.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box koordinatları
                conf = box.conf[0]  # Güven skoru
                cls = int(box.cls[0])  # Sınıf ID
                label = f"{self.model.names[cls]}"

                # Bounding box çiz
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if label == "car" or label == "cars":  # Hedef nesne algılandı mı?
                    self.target = True
                    break
            # cv2.imshow("PR2 Kamera Görüntüsü", frame)
            # cv2.waitKey(1)


        except Exception as e:
            rospy.logerr(f"Görüntü işlenirken hata oluştu: {e}")
        return self.target
    
class LidarHareket:
    def __init__(self, kamera):
        """Lidar verisiyle robot hareketini kontrol eden sınıf."""
        self.lidar_data = None
        self.pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
        self.kamera = kamera

        self.flag_kollar_indirildi = False

        # Robot hareketini kontrol etmek için Publisher
        self.pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
        
        # Kollar için Publisherlar
        self.pub_left_arm = rospy.Publisher('/l_arm_controller/command', JointTrajectory, queue_size=10)
        self.pub_right_arm = rospy.Publisher('/r_arm_controller/command', JointTrajectory, queue_size=10)

        # Lidar verisi için abone ol
        rospy.Subscriber('/base_scan', LaserScan, self.lidar_callback)

    def lidar_callback(self, msg):
        """Lidar verisini alır ve saklar."""
        self.lidar_data = msg.ranges

    def lidar_verisi(self):
        """Lidar'dan gelen veriyi okuyup mesafeleri hesaplar."""
        if not self.lidar_data:
            rospy.logwarn("Henüz lidar verisi alınmadı.")
            return None
        forward_distance = self.lidar_data[len(self.lidar_data) // 2]  # Ön mesafe
        return forward_distance

    def ileri_git(self, speed=3.0):
        """Robotun ileri hareketini başlatır."""
        twist = Twist()
        twist.linear.x = speed
        self.pub.publish(twist)

    def saga_git(self, duration=1.0, speed=3.0):
        """Robotun sağa hareketini sağlar."""
        twist = Twist()
        twist.angular.z = -speed
        self.pub.publish(twist)
        rospy.sleep(duration)
        twist.angular.z = 0.0  # Hareketi durdur
        self.pub.publish(twist)

    def sola_git(self, duration=1.0, speed=3.0):
        """Robotun sola hareketini sağlar."""
        twist = Twist()
        twist.angular.z = speed
        self.pub.publish(twist)
        rospy.sleep(duration)
        twist.angular.z = 0.0  # Hareketi durdur
        self.pub.publish(twist)

    def dur(self):
        """Robotun durmasını sağlar."""
        twist = Twist()
        self.pub.publish(twist)

    def kollar_asagi_yukari(self, asagi=True):
        """Robotun kollarını aşağı indirir veya yukarı kaldırır."""
        joint_positions = [0.0, 1.0, 0.0, 1.5, 0.0, 1.0, 0.0] if asagi else [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]

        # Sol kol hareketi
        left_trajectory = JointTrajectory()
        left_trajectory.joint_names = [
            'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
            'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'
        ]
        left_point = JointTrajectoryPoint()
        left_point.positions = joint_positions
        left_point.time_from_start = rospy.Duration(2)
        left_trajectory.points = [left_point]

        # Sağ kol hareketi
        right_trajectory = JointTrajectory()
        right_trajectory.joint_names = [
            'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
            'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'
        ]
        right_point = JointTrajectoryPoint()
        right_point.positions = joint_positions
        right_point.time_from_start = rospy.Duration(2)
        right_trajectory.points = [right_point]

        # Hareketi yayınla
        self.pub_left_arm.publish(left_trajectory)
        self.pub_right_arm.publish(right_trajectory)

        rospy.sleep(2)  # Hareketin bitmesini bekle

    def hareket_kontrol(self):
        """Robotun lidar ve kamera verilerine göre hareketini kontrol eder."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            forward_distance = self.lidar_verisi()
            if forward_distance is None:
                continue

            if self.kamera.target:  # Hedef algılandıysa
                rospy.loginfo("Hedef algılandı! Nesneye yaklaşılıyor.")
                if forward_distance < 0.35:
                    print(forward_distance)
                    self.dur()
                    self.kollar_asagi_yukari(asagi = False)
                elif forward_distance <= 1.0 and not self.flag_kollar_indirildi:
                    print(forward_distance)
                    self.kollar_asagi_yukari(asagi = True)
                    self.flag_kollar_indirildi = True  # Kolların indirildiğini işaretle
                    self.ileri_git(speed=5.0)
                elif forward_distance <= 1.0 and self.flag_kollar_indirildi:
                    print(forward_distance)
                    self.ileri_git(speed=5.0)
                elif forward_distance > 1.0:
                    print(forward_distance)
                    self.ileri_git(speed=5.0)  # Daha yavaş yaklaş
            else:
                if forward_distance < 1.0:  # Çok yakında bir engel varsa
                    rospy.loginfo("Engel algılandı! Sağa veya sola geçiliyor.")
                    self.dur()
                    rospy.sleep(1)
                    self.saga_git()  # Sağdan geçmeyi dene
                else:
                    rospy.loginfo("Güvenli mesafe, ileri gidiliyor.")
                    self.ileri_git(speed=10.0)

            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("lidar_hareket_kontrol")

        # Kamera ve Lidar sınıflarını başlat
        kamera = Kamera()
        lidar_hareket = LidarHareket(kamera)

        # Hareket kontrolünü başlat
        lidar_hareket.hareket_kontrol()

    except rospy.ROSInterruptException:
        pass