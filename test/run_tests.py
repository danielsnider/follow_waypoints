#! /usr/bin/env python3

import rosunit
import integration_test

# rosunit
rosunit.unitrun('follow_waypoints', 'integration_test',
                'integration_test.MyTestSuite')