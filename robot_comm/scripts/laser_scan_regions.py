#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class LaserScanRegions(object):

    def __init__(self):
        # self._regions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 6 - Segments 60 degrees
        self._regions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 8 - Segments 45 degrees
        self._heading = 0
        self._init()
        self._laser_scan_sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)

    def _init(self):
        laser_scan_first_read = None
        while laser_scan_first_read is None:
            try:
                laser_scan_first_read = rospy.wait_for_message('/scan', LaserScan, 10)
            except rospy.ROSException as roserr:
                rospy.logerr(str(roserr))
        
        self._updateRegionsReads(laser_scan_first_read)

    def _updateRegionsReads(self, sensor_reads, range_max=10.0):

        self._regions[0] = min(min(sensor_reads.ranges[0:22] + sensor_reads.ranges[338:359:-1]), range_max)
        self._regions[1] = min(min(sensor_reads.ranges[23:67]), range_max)
        self._regions[2] = min(min(sensor_reads.ranges[68:112]), range_max)
        self._regions[3] = min(min(sensor_reads.ranges[113:157]), range_max)
        self._regions[4] = min(min(sensor_reads.ranges[158:202]), range_max)
        self._regions[5] = min(min(sensor_reads.ranges[203:247]), range_max)
        self._regions[6] = min(min(sensor_reads.ranges[248:292]), range_max)
        self._regions[7] = min(min(sensor_reads.ranges[293:337]), range_max)
        min_read = min(sensor_reads.ranges)

        print(min_read)
        # self._heading = sensor_reads.ranges.index(min_read)
        rospy.loginfo('H[{}]-> R[{}]: {}'.format(self._heading, self.getClosestReadIndex(), self._regions))

    def _scan_callback(self, scan_reads):
        self._updateRegionsReads(scan_reads)

    def getRegionsReads(self):
        return self._regions
            
    def getClosestReadIndex(self):
        min_read = min(self._regions)
        return self._regions.index(min_read)  

    def getHeading(self):
        return self._heading


def testLaserScanRegions():
    rospy.init_node('test_obstacle_detector')
    pub = rospy.Publisher('closest_region', String, queue_size=1)
    opearate_indx = False
    closest_region = ''
    od = LaserScanRegions()
    while not rospy.is_shutdown():
        if opearate_indx:
            closest_indx = od.getClosestReadIndex()
            closest_region = 'H[{}], Region_{}: {}'.format(od.getHeading(), closest_indx, od.getRegionsReads()[closest_indx])
        else:
            closest_region = str(od.getRegionsReads())
        pub.publish(closest_region)


if __name__ == '__main__':
    testLaserScanRegions()

