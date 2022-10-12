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
from std_msgs.msg import Float64MultiArray
import numpy as np

class QualityMetrics(Node):
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('quality_metrics')
    '''
    self.surface_normals = self.create_subscription(
      PointXYZ,
      '/surface_normals',
      self.quality_listener,
      10)
    self.surface_normals
    '''
  def quality_listener(self, normal_vector):
    """
    Callback function.
    """
    
    # Calculate dot product, and compare vecotrs.
    for i in normal_vector:
      for j in normal_vector:
        # We need to see the coordinates of the normal_vecotrs
        r = self.distance_between_vector(i, j)

        th1 = self.dot_product_angle(i, r)
        th2 = self.dot_product_angle(j, r)
        if(self.good_angle(th1) and self.good_angle(th2)):
          # We publish it to a file to plot :)
    
    # Let's publish it to a file
  
  #Getting the angle from the dot product
  def distance_between_vector(self, vector_a_coord, vector_b_coord):
    r = np.array([vector_a_coord(0) - vector_b_coord(0), vector_a_coord(1) - vector_b_coord(1), vector_a_coord(2) - vector_b_coord(2) ])
    return r


  def dot_product_angle(self, vector_a, vector2_b):
    # vector_a * vector_b
    ab = vector_a.x*vector_b.x + vector_a.y*vector_b.y + vector_a.z*vector_b.z
    
    # Now let's get the abs of both vectors.
    tot_a = sqrt(pow(vector_a.x, 2) + pow(vector_a.y, 2) + pow(vector_a.z, 2))
    tot_b = sqrt(pow(vector_b.x, 2) + pow(vector_b.y, 2) + pow(vector_b.z, 2))

    tot_ab = tot_a * tot_b
    
    # We calculate them together
    calc = ab / tot_ab

    theta = np.arccos(calc)

    return theta

  def good_angle(self, theta):

    if(-10 < theta < 10 or 170 < theta < 190):
      return True
    else:
      return False

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

