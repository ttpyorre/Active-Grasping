# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray
import numpy as np



class QualityMetrics(Node):
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('quality_metrics')

    self.surface_normals = self.create_subscription(
      PointXYZ,
      '/surface_normals',
      self.quality_listener,
      10)
   
    # We are subscribing to the joint states, aswell as the current pixel coordinates we see
  # Listeners:
  def joint_listener(self, normal_vector):
    """
    Callback function.
    """
    # Calculate dot product. We also need the 
    # cos(th) = a*b/(abs(a*b))
    
    
    

def main(args=None):
  # Initializing rclpy
  rclpy.init(args=args)

  # Creating the node
  quality_metrics = QualityMetrics()

  # Spin the node, callback function gets called
  rclpy.spin(quality_metrics)
  
  quality_metrics.destroy_node()

  # Let's shutdown the ROS client
  rclpy.shutdown()

