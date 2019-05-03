#! /usr/bin/env python
import rospy
import numpy

from sensor_msgs.msg import *
from math import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from numpy.ma.core import abs


class TorsoDetection:
    threshold = 0.07  # distance between two ranges
    threshold2 = 0.05  # two mini-clusters distance
    pi = 3.14159
    threshold_angle = 130.0
    
    maxLenght = 0.80
    minLenght = 0.25
    outOfRange = 20
    togetherPoint_thresold = 0.015
    curve_thr = 50.0
    
    threshold_diagonal = 0.05

    torsoPublish = std_msgs.msg.Float32()

    markerPublisher = visualization_msgs.msg.MarkerArray()    
                           
    def __init__(self):
        self.laserResoloution = 0.25

        rospy.Subscriber("/scan_ubg", LaserScan, self.up_laser_cb, None, 10)
        
        self.torsoPublisher = rospy.Publisher(
            "/TorsoDetectionRange", std_msgs.msg.Float32, queue_size=1
        )
        self.torsoPublisher2 = rospy.Publisher(
            "/TorsoDetectionPoint", std_msgs.msg.Float32, queue_size=1
        )
        self.markerPublisher = rospy.Publisher(
            "/TorsoMarker", visualization_msgs.msg.MarkerArray, queue_size=10
        )

        self.rate = rospy.Rate(50)  # 100hz
        
    def visualize(self, data):
        arr = MarkerArray()

        i = 0
        for points in data:    
            m1 = self.genMarker(points[0], points[1], True, i)
            m1.lifetime = rospy.Duration(1, 1)
            m2 = self.genMarker(points[2], points[3], False, i + 1)
            m2.lifetime = rospy.Duration(1, 1)
            arr.markers.append(m1)
            arr.markers.append(m2)            
            i += 2

        self.markerPublisher.publish(arr)

    def genMarker(self, x, y, is_start, id):
        m = visualization_msgs.msg.Marker()         
        # m.header.frame_id = '/ubg_laser'
        m.header.frame_id = '/velodyne'
        m.header.stamp = rospy.Time.now()
        m.ns = 'torso_points'
        m.action = visualization_msgs.msg.Marker.ADD
        m.pose.orientation.w = 1.0
        m.id = id
        m.type = visualization_msgs.msg.Marker.SPHERE                       
        
        if is_start:
            m.color.r = 1.0
        else:
            m.color.g = 1.0
        
        m.color.a = 1.0
        
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.2
        
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        
        return m

    def cluster(self, laser):
        print("Start ...", len(laser.ranges))
        clustersPoints = []
        clustersPoints.append([])
        appendFlag = False

        i = 0
        while i < len(laser.ranges) - 1:  # clustering
            rad = (self.laserResoloution * pi) / 180
            b = laser.ranges[i]  # * 100  # reform to cm
            c = laser.ranges[i + 1]
            distance = sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad)))

            if (distance < self.threshold) and (laser.ranges[i] < self.outOfRange) and \
                    (laser.ranges[i] != 0):
                clustersPoints[len(clustersPoints) - 1].append(i)
                appendFlag = True

            elif (distance > self.threshold) and appendFlag and \
                    (laser.ranges[i] < self.outOfRange) and (laser.ranges[i] != 0):
                clustersPoints.append([])
                appendFlag = False

            i += 1

        del clustersPoints[len(clustersPoints) - 1]  # yeduneye akhari xalie hamishe

        i = 0
        while i < len(clustersPoints) - 1:  # better clustering
            a = ((clustersPoints[i + 1][0]) - (clustersPoints[i][len(clustersPoints[i]) - 1]))
            b = laser.ranges[clustersPoints[i][len(clustersPoints[i]) - 1]]
            c = laser.ranges[clustersPoints[i + 1][0]]

            localDegreeOfCluster = a * self.laserResoloution
            rad2 = (localDegreeOfCluster * pi) / 180
            clusterDistance = sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad2)))

            if clusterDistance < self.threshold2:
                clustersPoints[i] = clustersPoints[i] + clustersPoints[i + 1]
                del clustersPoints[i + 1]
                i -= 1  # baraye inke 2bare haman index ra chek konad

            i += 1

        return clustersPoints

    def cluster_filtering(self, clustersPoints, laser):
        clustersDistance = []
        indexes = []
        markIndexes = []

        i = 0
        while i < len(clustersPoints):  # clusters Distance
            a = len(clustersPoints[i]) - 1
            b = laser.ranges[clustersPoints[i][len(clustersPoints[i]) - 1]]
            c = laser.ranges[clustersPoints[i][0]]
            localDegreeOfCluster = a * self.laserResoloution
            rad3 = (localDegreeOfCluster * pi) / 180
            clustersDistance.append(sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad3))))
            if (len(clustersPoints[i]) % 2) == 0:
                middle = (len(clustersPoints[i]) / 2) - 1  # b xatere inke indexha az 0 shuru mishe

            elif (len(clustersPoints[i]) % 2) == 1:
                middle = (len(clustersPoints[i]) / 2)  # this is ok

            indexes.append(
                clustersPoints[i][middle])  # index haye miani #pointhaye mianie cluster ha
            markIndexes.append(i)  # indexe clusterha
            i = i + 1

        # print(clustersDistance)
        # print(indexes)
        # print(markIndexes)
        i = 0
        goodClusters = []
        goodIndexes = []
        goodMarkers = []

        while i < len(clustersDistance):  # good distance choose
            if clustersDistance[i] > self.minLenght and clustersDistance[i] < self.maxLenght:
                goodClusters.append(clustersDistance[i])  # faselehaye xub dar in zakhire mishavand
                goodIndexes.append(indexes[i])  # index haye mianie xube laser.
                goodMarkers.append(markIndexes[i])  # indexe clusterhaye xub.

                j = 0
                length = int(len(goodMarkers) * 0.2)  # yany 20% az aan
                while j < length:  # removing hands clusters
                    # del clustersPoints[goodMarkers[i]][j]  # TODO
                    # del clustersPoints[goodMarkers[i]][len(goodMarkers) - j]  # TODO
                    j += 1

            i += 1

        return goodMarkers, goodIndexes

    def curve_detection(self, goodMarkers, laser, clustersPoints):
        i = 0
        dist1 = []
        dist2 = []
        dist3 = []
        angle = []

        while i < len(goodMarkers):  # curve detection
            gm = goodMarkers[i]
            a = laser.ranges[clustersPoints[gm][0]]
            z = laser.ranges[clustersPoints[gm][len(clustersPoints[gm]) - 1]]
            n1 = len(clustersPoints[gm]) - 1
            localDegreeOfCluster = n1 * self.laserResoloution
            rad6 = (localDegreeOfCluster * pi) / 180
            Distance1 = (sqrt((pow(a, 2) + pow(z, 2)) - (2 * a * z * cos(rad6))))
            dist3.append(Distance1)
            dist1.append([])
            dist2.append([])
            angle.append([])

            j = 0
            while j < len(clustersPoints[gm]) - 2:  # baraye inke avvali va akhari dar bala hesab shode and.
                n2 = ((clustersPoints[gm][j + 1]) - (clustersPoints[gm][0]))
                n3 = (clustersPoints[gm][len(clustersPoints[gm]) - 1]) - \
                     (clustersPoints[gm][j + 1])
                b = laser.ranges[clustersPoints[gm][j + 1]]
                localDegreeOfCluster2 = n2 * self.laserResoloution
                localDegreeOfCluster3 = n3 * self.laserResoloution
                rad7 = (localDegreeOfCluster2 * pi) / 180
                rad8 = (localDegreeOfCluster3 * pi) / 180
                Distance2 = sqrt((pow(a, 2) + pow(b, 2)) - (2 * a * b * cos(rad7)))
                Distance3 = sqrt((pow(z, 2) + pow(b, 2)) - (2 * z * b * cos(rad8)))
                dist1[i].append(Distance2)
                dist2[i].append(Distance3)
                angl = numpy.arccos((pow(dist1[i][j], 2) + pow(dist2[i][j], 2) - pow(dist3[i], 2))
                                    / (2 * dist1[i][j] * dist2[i][j]))
                angl *= 180 / pi  # radian to degree
                angle[i].append(angl)
                j += 1

            i += 1

        # print("angle", angle)
        # print("dist1", len(dist1))
        # print("dist2", len(dist2))
        # print("dist3", len(dist3))
        # print("goodcluster", goodMarkers)

        return angle

    def curve_filtering(self, angle, clustersPoints, goodMarkers):
        i = 0
        curv = []
        cmp2 = 0
        cmp3 = 1
        curv_indexes = []
        while i < len(angle):

            j = 0
            while j < len(angle[i]) - 1:
                cmp3 += 1

                if (abs(angle[i][j] - angle[i][j + 1]) < 25) and \
                        (abs(angle[i][j] - self.threshold_angle) < 25):
                    cmp2 += 1

                if ((len(angle) - cmp3) == 0) and \
                        (abs(angle[i][j + 1] - self.threshold_angle) < 25):  # baraye inke zaviye akhar ham barresi shavad
                    cmp2 += 1

                j += 1

            if (len(angle[i]) - cmp2) < ((self.curve_thr * len(angle[i])) / 100):  # yani dar suraT varede if mishavad k ekhtelafe cmp2 ba har goodcluster bayad kamtar az 20% an bashad, yany 80% sahih bashad
                curv.append(clustersPoints[goodMarkers[i]])
                curv_indexes.append(goodMarkers[i])

            i += 1

        print ("goodmarkersss", goodMarkers)
        print ("curve_indexes", curv_indexes)

        return curv_indexes

    def up_laser_cb(self, laser):
        self.laserResoloution = 360.0 / len(laser.ranges)
        clustersPoints = self.cluster(laser)
        good_markers, good_indices = self.cluster_filtering(clustersPoints, laser)
        angle = self.curve_detection(good_markers, laser, clustersPoints)
        curv_indexes = self.curve_filtering(angle, clustersPoints, good_markers)

        i2 = 0
        positions2 = []
        x12 = []
        y12 = []
        x22 = []
        y22 = []
        while i2 < len(curv_indexes):
            '''Create Markers'''
            positions2.append([])
            end = len(clustersPoints[curv_indexes[i2]]) - 1
            res = self.laserResoloution
            ii1 = clustersPoints[curv_indexes[i2]][0]
            ii9 = clustersPoints[curv_indexes[i2]][end]
            rad4 = ((ii1 * res * pi) / 180) - pi
            rad5 = ((ii9 * res * pi) / 180) - pi

            x12.append(laser.ranges[ii1] * cos(rad4))  # red
            y12.append(laser.ranges[ii1] * sin(rad4))

            x22.append(laser.ranges[ii9] * cos(rad5))  # green
            y22.append(laser.ranges[ii9] * sin(rad5))

            positions2[i2].append(x12[i2])
            positions2[i2].append(y12[i2])
            positions2[i2].append(x22[i2])
            positions2[i2].append(y22[i2])

            ''' for human detection '''
            mid2 = clustersPoints[curv_indexes[i2]][end / 2]  # torso Index Middle
            range2send2 = Float32(laser.ranges[mid2])
            point2send2 = Float32(mid2)
            self.torsoPublisher.publish(range2send2)
            self.torsoPublisher2.publish(point2send2)
            self.rate.sleep()
            i2 += 1

        if curv_indexes:
            '''Trigger Marker.'''
            self.visualize(positions2)


if __name__ == '__main__':
    rospy.init_node('agn_torso_detection', anonymous=True)
    TorsoDetection = TorsoDetection()
    rospy.spin() 
