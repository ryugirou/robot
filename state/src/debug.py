#!/usr/bin/env python
# coding: UTF-8
from enum import IntEnum
import time
import rospy
#スーパークラス
class SuperClass(object): #objectクラスを継承するのを忘れない
	def __init__(self):
		self.__name ="Taro" #nameを初期化
		
#サブクラス
class SubClass(SuperClass): #SuperClassを継承する
	def __init__(self):
		super(SubClass, self).__init__() #super()を使ってスーパークラスの__init__()を呼び出す

class Trajectorys(IntEnum):
    TEST = 0
    TRSZ_TO_RZ = 1
    RZ_TO_TS1 = 2
    TS1_TO_RZ = 3
    RZ_TO_TS2 = 4
    TS2_TO_RZ = 5
    RZ_TO_TS3 = 6
    TS3_TO_RZ = 7
    RZ_TO_TS4 = 8
    TS4_TO_RZ = 9
    RZ_TO_TS5 = 10
    TS5_TO_RZ = 11

    RZ_TO_KZ = 12
    KZ_TO_RZ = 13

    TS1_TO_WP = 14
    TS2_TO_WP = 15

if __name__ == '__main__':
    rospy.init_node('debug', anonymous=True)
    start = time.time()
    t = 10
    time.sleep(1)
    time.sleep(2)
    elapsed_time = time.time() - start
    # rospy.loginfo("%s : %f sec",Trajectorys.TS5_TO_RZ,elapsed_time)
    rospy.loginfo(Trajectorys(0))