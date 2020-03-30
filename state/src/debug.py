#!/usr/bin/env python
# coding: UTF-8
from enum import IntEnum

class Trajectorys(IntEnum):
    TEST = 0

    #TR
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

    TS1_TO_KZ = 12
    TS2_TO_KZ = 13
    TS3_TO_KZ = 14
    TS4_TO_KZ = 15
    TS5_TO_KZ = 16

    KZ_TO_RZ = 17

    @staticmethod
    def goal():
      print(int(Trajectorys.RZ_TO_TS1))

Trajectorys.goal()
print(Trajectorys.RZ_TO_TS1)