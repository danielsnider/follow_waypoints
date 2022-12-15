#! /usr/bin/env python3

from follow_waypoints.save_waypoints import SaveWaypoint
import unittest


class CaseA(unittest.TestCase):

    def setUp(self):
        self.rc = SaveWaypoint()

    def runTest(self):
        
        resp = self.rc.angle()
        self.assertEquals(resp, )
        self.rc.shutdownhook()


class CaseB(unittest.TestCase):

    def setUp(self):
        self.rc = SaveWaypoint()

    def runTest(self):

        resp = self.rc.open()
        self.assertEquals(resp,)
        self.rc.shutdownhook()

class CaseC(unittest.TestCase):
     
    def setUp(self):
        self.rc = SaveWaypoint()

    def runTest(self):

        resp = self.rc.save()
        self.assertEquals(resp,)
        self.rc.shutdownhook()

class CaseD(unittest.TestCase):
     
    def setUp(self):
        self.rc = SaveWaypoint()

    def runTest(self):

        resp = self.rc.create()
        self.assertEquals(resp, )
        self.rc.shutdownhook()


class MyTestSuite(unittest.TestSuite):

    def __init__(self):
        super(MyTestSuite, self).__init__()
        self.addTest(CaseA())
        self.addTest(CaseB())
        self.addTest(CaseC())
        self.addTest(CaseD())