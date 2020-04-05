#!/usr/bin/env python
# coding: UTF-8
from enum import IntEnum

class Trajectorys(IntEnum):
    TEST = 0

    #TR
    TRSZ_TO_RZ = 1
    RZ_TO_TS1 = 2
    TS1_TO_RZ2 = 3
    RZ_TO_TS2 = 4
    TS2_TO_RZ3 = 5
    RZ_TO_TS3 = 6
    TS3_TO_RZ3 = 7
    RZ_TO_TS4 = 8
    TS4_TO_RZ3 = 9
    RZ_TO_TS5 = 10
    TS5_TO_RZ = 11

    RZ_TO_KZ = 12

    TS1_TO_WP = 13
    TS2_TO_WP = 14
    TS3_TO_WP = 15
    TS4_TO_WP = 16
    TS5_TO_WP = 17
    
    WP_TO_KZ = 18

    KZ_TO_RZ = 19
    KZ_TO_RZ2 = 20
    KZ_TO_RZ3 = 21
    #PR
    # PRSZ_TO_PP1 = 12
    # PP1_TO_RZ = 13
    # RZ_TO_PP2 = 14
    # PP2_TO_RZ = 15
    # RZ_TO_PP3 = 16
    # PP3_TO_RZ = 17
    # RZ_TO_PP4 = 18
    # PP4_TO_RZ = 19
    # RZ_TO_PP5 = 20
    # PP5_TO_RZ = 21
    RZ1 = 22
    RZ2 = 23
    RZ3 = 24
    PRSZ = 25