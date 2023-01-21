import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from std_msgs.msg import Int8MultiArray
import struct

n = joint1 = rospy.get_param("/map_size")


def mapcallback(data=None):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.FLOAT32, 1),
              ]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()

    x, y, z = np.meshgrid(np.linspace(0,n,n), np.linspace(0,n,n), np.linspace(0,n,n))

    pointsColor = np.zeros( (n**3, 1), \
        dtype={ 
            "names": ( "x", "y", "z", "rgba" ), 
            "formats": ( "f4", "f4", "f4", "u4" )} )

    pointsColor["x"] = x.reshape((-1,1)).astype(np.float32)
    pointsColor["y"] = y.reshape((-1,1)).astype(np.float32)
    pointsColor["z"] = z.reshape((-1,1)).astype(np.float32)

    # 0 - free node, 1 - start, 2 - goal, 3 - path, 4 - obstacle, 5 - frontier, 6 - explored
    map = np.array(data.data)
    r_list= np.array([255,0,255,255,0,120,255])
    g_list= np.array([255,255,0,255,0,120,165])
    b_list= np.array([255,0,0,0,0,120,0])
    r = np.take(r_list, map)
    g = np.take(g_list, map)
    b = np.take(b_list, map)

    C = np.zeros((n**3, 4), dtype=np.uint8) + 255
    C[:,2] = r.astype(np.uint8)
    C[:,1] = g.astype(np.uint8)
    C[:,0] = b.astype(np.uint8)
    
    C = C.view("uint32")
    pointsColor["rgba"] = C
  

    # for j in range(n*n*n):
    #     print(x[j])
  
    # h = np.array(x1.data)

    # h = np.reshape(h, (n, n, n))
    msg = PointCloud2()
    msg.height = 1
    msg.width = n**3
    msg.header = header
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step   = 16
    msg.row_step     = msg.point_step * n**3
    msg.is_dense     = False
    msg.data         = pointsColor.tostring()

   

    
    # points = np.array([x,y,z,r]).reshape(4,-1).T

    # pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('map_publisher', anonymous=True)
    rospy.Subscriber("planning_info", Int8MultiArray, mapcallback, queue_size=1000, buff_size=1000)
    
    pub = rospy.Publisher('map', PointCloud2, queue_size=100)
    rate = rospy.Rate(30)
    rospy.spin()

  