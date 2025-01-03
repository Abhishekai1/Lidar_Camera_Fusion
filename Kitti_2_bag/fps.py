import rospy
from sensor_msgs.msg import Image, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import time

class FusionFPSMonitor:
    def __init__(self, image_topic, pointcloud_topic):
        self.image_topic = image_topic
        self.pointcloud_topic = pointcloud_topic

        # Initialize timing variables
        self.start_time = time.time()
        self.frame_count = 0

        # Subscribe to image and point cloud topics
        image_sub = Subscriber(self.image_topic, Image)
        pointcloud_sub = Subscriber(self.pointcloud_topic, PointCloud2)

        # Synchronize topics with ApproximateTimeSynchronizer
        ats = ApproximateTimeSynchronizer([image_sub, pointcloud_sub], queue_size=10, slop=0.5)
        ats.registerCallback(self.callback)

        rospy.loginfo("Monitoring FPS for LiDAR-Camera fusion on topics:")
        rospy.loginfo("  Image Topic: {}".format(self.image_topic))
        rospy.loginfo("  PointCloud Topic: {}".format(self.pointcloud_topic))

    def callback(self, image_msg, point_cloud_msg):
        rospy.loginfo("Callback triggered. Processing synchronized data...")
        # Increment frame count for each synchronized message pair
        self.frame_count += 1

        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time

        if elapsed_time >= 1.0:  # Update FPS every second
            fps = self.frame_count / elapsed_time
            rospy.loginfo("Fusion FPS: {:.2f}".format(fps))

            # Reset timing variables
            self.start_time = time.time()
            self.frame_count = 0

def main():
    rospy.init_node("fusion_fps_monitor", anonymous=True)

    # Specify the topics for image and point cloud
    image_topic = "/pcOnImage_image"  # Replace with your image topic
    pointcloud_topic = "/points2"     # Replace with your point cloud topic

    # Initialize the FPS monitor
    FusionFPSMonitor(image_topic, pointcloud_topic)

    rospy.loginfo("Starting FPS Monitor for LiDAR-Camera Fusion...")
    rospy.spin()

if __name__ == "__main__":
    main()

