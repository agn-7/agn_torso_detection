#! /usr/bin/env python
import rospy
import numpy

from sensor_msgs.msg import *
from math import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from numpy.ma.core import abs


class TorsoDetection:
    threshold = 0.05  # 2ta range
    threshold2 = 0.05  # fasele 2ta point
    pi = 3.14159
    threshold_angle = 130.0

    maxLenght = 0.80
    minLenght = 0.25
    farLenght = 0.30
    outOfRange = 3
    togetherMin = 0.16
    togetherMax = 35
    togetherPoint_thresold = 0.015

    threshold_diagonal = 0.05

    laserResoloution = 0.25

    torsoPublish = std_msgs.msg.Float32()

    markerPublisher = visualization_msgs.msg.MarkerArray()

    def __init__(self):

        rospy.Subscriber("/scan_ubg", LaserScan, self.upLaserCb, None,
                         10)  # laser e balaE madde nazar ast

        self.torsoPublisher = rospy.Publisher("/TorsoDetectionRange", std_msgs.msg.Float32,
                                              queue_size=1)
        self.torsoPublisher2 = rospy.Publisher("/TorsoDetectionPoint", std_msgs.msg.Float32,
                                               queue_size=1)
        self.markerPublisher = rospy.Publisher("/TorsoMarker", visualization_msgs.msg.MarkerArray,
                                               queue_size=10)

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
            i = i + 2
        self.markerPublisher.publish(arr)
        # self.gtorsoPublisher.publish(arr)

    def genMarker(self, x, y, is_start, id):
        m = visualization_msgs.msg.Marker()
        m.header.frame_id = '/laser'  # '/ubg_laser'
        m.header.stamp = rospy.Time.now()
        m.ns = 'torso_points'
        m.action = visualization_msgs.msg.Marker.ADD
        m.pose.orientation.w = 1.0
        m.id = id
        m.type = visualization_msgs.msg.Marker.SPHERE

        if (is_start == True):
            m.color.r = 1.0
        else:
            m.color.g = 1.0

        m.color.a = 1.0;

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 1  # 0

        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1

        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1

        return m

    def upLaserCb(self, laser):
        veto = True
        if self.torsoPublisher.get_num_connections() > 0 or veto:  # for scenario used
            print("Starttttttttttttttttttttt")
            i = 180  # qablan 360
            clustersCount = 0
            localFirstPoint = 120  # zavie 0 daraje #180 #360   # zavie 45 ta 135 robrooye robot beshe 280 theta0 = 45
            localEndPoint = 600  # zavie 180daraje #540 #719   #   "   "   "  "    "        "   beshe 800 theta1 = 1355
            theta0 = 45  # not used yet
            clustersPoints = []
            clustersPoints.append([])
            appendFlag = False
            badPointsIndex = []

            while i < localEndPoint:  # clustring
                rad = (self.laserResoloution * pi) / 180
                b = laser.ranges[i]  # * 100  # baraye tabDl b Cm
                c = laser.ranges[i + 1]
                Distance = sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad)))

                if (Distance < self.threshold) and (laser.ranges[i] < self.outOfRange) and (
                        laser.ranges[i] != 0):
                    clustersPoints[len(clustersPoints) - 1].append(i)
                    appendFlag = True

                elif (Distance > self.threshold) and (appendFlag == True) and (
                        laser.ranges[i] < self.outOfRange) and (laser.ranges[i] != 0):
                    clustersPoints.append([])
                    appendFlag = False

                i = i + 1

            del clustersPoints[len(
                clustersPoints) - 1]  # yeduneye akhari xalie hamishe
            # print(clustersPoints)

            i = 0

            while i < len(
                    clustersPoints) - 1:  # better clustring # -1 baraye inke y duneye akhari hamishe xalie
                a = ((clustersPoints[i + 1][0]) - (clustersPoints[i][len(clustersPoints[i]) - 1]))

                b = laser.ranges[clustersPoints[i][len(clustersPoints[i]) - 1]]
                c = laser.ranges[clustersPoints[i + 1][0]]

                localDegreeOfCluster = a * self.laserResoloution
                rad2 = (localDegreeOfCluster * pi) / 180

                clusterDistance = (sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad2))))

                if clusterDistance < self.threshold2:
                    # clustersPoints[i].append(clustersPoints[i+1])
                    clustersPoints[i] = clustersPoints[i] + clustersPoints[i + 1]
                    del clustersPoints[i + 1]
                    i = i - 1  # baraye inke 2bare haman index ra chek konad

                i = i + 1

            clustersDistance = []
            indexes = []
            markIndexes = []
            i = 0
            # clustersDistance.append([])
            while i < len(clustersPoints):  # clusters Distance
                # length = b^2 + c^2 - 2bc*cosDegree
                a = len(clustersPoints[i]) - 1
                b = laser.ranges[clustersPoints[i][len(clustersPoints[i]) - 1]]
                c = laser.ranges[clustersPoints[i][0]]
                localDegreeOfCluster = a * self.laserResoloution
                rad3 = (localDegreeOfCluster * pi) / 180
                clustersDistance.append(sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad3))))
                if (len(clustersPoints[i]) % 2) == 0:
                    middle = (len(
                        clustersPoints[i]) / 2) - 1  # b xatere inke indexha az 0 shuru mishe

                elif (len(clustersPoints[i]) % 2) == 1:
                    middle = (len(clustersPoints[i]) / 2)  # this is ok

                indexes.append(
                    clustersPoints[i][middle])  # index haye miani #pointhaye mianie cluster ha
                markIndexes.append(i)  # indexe clusterha
                i = i + 1

            ###print(clustersDistance)
            # print(indexes)
            ###print (markIndexes)
            i = 0;
            goodClusters = []
            goodIndexes = []
            goodMarkers = []
            length = 0

            while i < len(clustersDistance):  # good distance choose
                if clustersDistance[i] > self.minLenght and clustersDistance[i] < self.maxLenght:
                    goodClusters.append(
                        clustersDistance[i])  # faselehaye xub dar in zakhire mishavand
                    goodIndexes.append(indexes[i])  # index haye mianie xube laser.
                    goodMarkers.append(markIndexes[
                                           i])  # index haye xub dar clustersPoints.  ya indexe clusterhaye xub
                    j = 0
                    length = int(len(goodMarkers) * 0.2)  # yany 20% az aan
                    while j < length:  # baraye inke dastha hazf shavand
                        del clustersPoints[goodMarkers[i]][j]
                        del clustersPoints[goodMarkers[i]][len(goodMarkers) - j]
                        j += 1

                i = i + 1

            # print (goodClusters, "goodclusters")
            # print (goodIndexes, "goodindexesss")
            ###print (goodMarkers, "goodmarkersss")

            ##################tashkhise zaviyeye har cluster (curv detection)#########################
            i = 0
            dist1 = []
            dist2 = []
            dist3 = []
            angle = []

            while i < len(goodMarkers):  # curv detection
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
                j = 0  # jashh hamin jasss na bala pishe i :|

                while j < len(clustersPoints[
                                  gm]) - 2:  # baraye inke avvali va akhari dar bala hesab shode and.
                    n2 = ((clustersPoints[gm][j + 1]) - (clustersPoints[gm][0]))
                    n3 = ((clustersPoints[gm][len(clustersPoints[gm]) - 1]) - (
                    clustersPoints[gm][j + 1]))
                    b = laser.ranges[clustersPoints[gm][j + 1]]
                    localDegreeOfCluster2 = n2 * self.laserResoloution
                    localDegreeOfCluster3 = n3 * self.laserResoloution
                    rad7 = (localDegreeOfCluster2 * pi) / 180
                    rad8 = (localDegreeOfCluster3 * pi) / 180
                    Distance2 = (sqrt((pow(a, 2) + pow(b, 2)) - (2 * a * b * cos(rad7))))
                    Distance3 = (sqrt((pow(z, 2) + pow(b, 2)) - (2 * z * b * cos(rad8))))
                    dist1[i].append(Distance2)
                    dist2[i].append(Distance3)
                    angl = numpy.arccos(
                        (pow(dist1[i][j], 2) + pow(dist2[i][j], 2) - pow(dist3[i], 2)) / (
                                    2 * dist1[i][j] * dist2[i][j]))
                    angl = (angl * (180 / pi))  # baraye tabDle radian b daraje
                    angle[i].append(angl)
                    j = j + 1

                i = i + 1

            ###print("angle", angle)
            # print("dist1", len(dist1))
            # print("dist2", len(dist2))
            # print("dist3", len(dist3))
            ###print("goodcluster", goodMarkers)
            #######################################

            ##############filter kardane clusterha ba zavie monaseb##################
            i = 0
            curv = []
            cmp2 = 0
            cmp3 = 1
            curv_indexes = []
            while i < len(angle):
                j = 0

                while j < len(angle[i]) - 1:
                    cmp3 = cmp3 + 1

                    if (abs(angle[i][j] - angle[i][j + 1]) < 25) and (
                            abs(angle[i][j] - self.threshold_angle) < 25):
                        cmp2 = cmp2 + 1

                    if ((len(angle) - cmp3) == 0) and (abs(angle[i][
                                                               j + 1] - self.threshold_angle) < 25):  # baraye inke zaviye akhar ham barresi shavad
                        cmp2 = cmp2 + 1

                    j = j + 1

                if (len(angle[i]) - cmp2) < ((20.0 * len(angle[
                                                             i])) / 100):  # yani dar suraT varede if mishavad k ekhtelafe cmp2 ba har goodcluster bayad kamtar az 20% an bashad, yany 80% sahih bashad
                    curv.append(clustersPoints[goodMarkers[i]])
                    # curv.append([])   #niaz b in nist chon xode clustersPoints[goodMarkers[i]] b surate arayeE az arayehas.
                    curv_indexes.append(goodMarkers[i])

                i = i + 1

            print ("goodmarkersss", goodMarkers)
            ###print ("curv", curv)
            # print ("cmp2", cmp2)
            print ("curv_indexes", curv_indexes)
            ########################################

            ########################Pair Leg Detection#########wear, bayad moqayeseE shavad, yany hame ba hame, na motevali#####
            #         Legs = []
            #         Legs_Indexes = []
            #         i = 0
            #         k = -1
            #
            #         while i < len(curv) - 1:
            #             end1 = len(clustersPoints[curv_indexes[i]]) - 1
            #             end2 = len(clustersPoints[curv_indexes[i + 1]]) - 1
            #             a = ((clustersPoints[curv_indexes[i + 1]][0]) - (clustersPoints[curv_indexes[i]][end1]) )
            #             b = laser.ranges[clustersPoints[curv_indexes[i]][end1]]
            #             c = laser.ranges[clustersPoints[curv_indexes[i + 1]][0]]
            #             localDegreeOfCluster = a * self.laserResoloution
            #             rad8 = (localDegreeOfCluster * pi) / 180
            #             twoLegsDistance = sqrt( (pow(b, 2) + pow(c, 2) ) - (2 * b * c * cos(rad8) ) )
            #             d = len(clustersPoints[curv_indexes[i]])
            #             e = laser.ranges[clustersPoints[curv_indexes[i]][0]]
            #             f = len(clustersPoints[curv_indexes[i + 1]])
            #             g = laser.ranges[clustersPoints[curv_indexes[i + 1]][end2]]
            #             localDegreeOfCluster2 = d * self.laserResoloution
            #             localDegreeOfCluster3 = f * self.laserResoloution
            #             rad9 = (localDegreeOfCluster2 * pi) / 180
            #             rad10 = (localDegreeOfCluster3 * pi) / 180
            #             diagonal1 = sqrt(pow(e, 2) + pow(b, 2) - (2 * b * e * cos(rad9)))
            #             diagonal2 = sqrt(pow(g, 2) + pow(c, 2) - (2 * g * c * cos(rad10)))
            #
            #             if (twoLegsDistance < self.farLenght) and (abs(diagonal1 - diagonal2) < self.threshold_diagonal):
            #                 #print("im hereeee") #this is OK
            #                 k = k + 1
            #                 Legs.append(curv[i] + curv[i + 1])
            #                 Legs_Indexes.append([])
            #                 Legs_Indexes[k].append(curv_indexes[i])
            #                 Legs_Indexes[k].append(curv_indexes[i + 1])
            #                 i = i + 1  # baraye inke agar vared if shod yany inke 1 joft pa kamel shod va Dgar niaz b moqayese paye dovvome aan joft pa ba clustere Dgar nist.
            #
            #             i = i + 1
            #
            #         print("leg", Legs)
            #         print("leg_indexes", Legs_Indexes)
            #         #print("goods", goodMarkers)
            ##################################################################

            cmpIndexes = []
            i = 0

            while i < len(
                    goodMarkers):  # ijade 1 araye baraye pyda kardane un clustri k ruberuye robote.
                cmpIndexes.append(abs(360 - goodIndexes[i]))  # 360 shot roobe rooye robot
                i = i + 1

            # print (cmpIndexes)

            i = 0
            minIndex = 0
            localMin = 180  # bishtarin chizi k Mkan dare bashe = 360 - 180 = 180
            while i < len(cmpIndexes):  # Ntekhabe clustere ruberuye robot.
                if cmpIndexes[i] < localMin:
                    localMin = cmpIndexes[i]
                    minIndex = i

                i = i + 1

            bestIndexes = 0
            bestIndexes1 = 0
            if len(goodIndexes) > 0:
                bestIndex = goodIndexes[
                    minIndex]  # bhtarin shot. k shote mianie aan cluster ast. # be manie [j] ast
                bestIndex1 = goodMarkers[minIndex]  # bhtarin cluster. # be manie [i] ast
                ###print ('These are bests ===> ',bestIndex,bestIndex1)

            # print(clustersPoints)

            ###new markkkkkkkkkkkkkk###
            i2 = 0
            positions2 = []
            x12 = []
            y12 = []
            x22 = []
            y22 = []
            ###if len(goodIndexes) > 0:    #modified  #create marker
            while i2 < len(curv_indexes):
                positions2.append([])
                # x1.append([])
                # y1.append([])
                # x2.append([])
                # y2.append([])
                lif = clustersPoints[
                    curv_indexes[i2]]  # LIF means, is, Legs Indexes First
                end = len(clustersPoints[curv_indexes[i2]]) - 1
                t0 = theta0
                res = self.laserResoloution
                # i1 =  clustersPoints[i][0]
                # i9 =  clustersPoints[i][end]
                ii1 = clustersPoints[curv_indexes[i2]][0]
                # print (bestIndex1, ii1, ii9)
                ii9 = clustersPoints[curv_indexes[i2]][end]
                rad4 = ((ii1 * res * pi) / 180) - (
                            pi / 2)  # "pi/2" baraye taqir noqte 0 e mokhtasat mibashad
                rad5 = ((ii9 * res * pi) / 180) - (pi / 2)

                x12.append(laser.ranges[ii1] * cos(rad4))  # qermez
                y12.append(laser.ranges[ii1] * sin(rad4))  # + (laser.ranges[ii1] / 2)
                ###print ("zaviye ii1 R(ii1)", ii1 * res - 90, ii1, laser.ranges[ii1])

                x22.append(laser.ranges[ii9] * cos(rad5))  # sabz
                y22.append(laser.ranges[ii9] * sin(rad5))  # + (laser.ranges[ii9] / 4)
                ###print ("zaviye ii9 R(ii9)", ii9 * res - 90, ii9, laser.ranges[ii9])

                positions2[i2].append(x12[i2])
                positions2[i2].append(y12[i2])
                positions2[i2].append(x22[i2])
                positions2[i2].append(y22[i2])
                # self.visualize(positions) # for test is here
                #############4 human detection##############
                mid2 = clustersPoints[curv_indexes[i2]][end / 2]  # torso Index Middle
                range2send2 = Float32(laser.ranges[mid2])
                point2send2 = Float32(mid2)
                self.torsoPublisher.publish(range2send2)
                self.torsoPublisher2.publish(point2send2)
                self.rate.sleep()  # fkkonam sorat ro
                ##############################################

                i2 = i2 + 1

            ###print (positions, "<=============position")
            if (curv_indexes):
                self.visualize(positions2)

                ##end of new mark#####
            ####self.visualize.lifetime = rospy.Duration(10, 0)
            ###print(clustersPoints, "clusterrrrs")
            ###print(angle, "anglessss")
            ###print ("goodmarkersss", goodMarkers)
            ###print(dist1, "dist11111")
            ###print(dist2, "dist22222")
            ###print(dist3, "dist33333")


if __name__ == '__main__':
    rospy.init_node('new_torso_detection', anonymous=True)
    TorsoDetection = TorsoDetection()
    rospy.spin() 
