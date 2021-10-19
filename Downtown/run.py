#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2021 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

"""
Tutorial for traffic light control via the TraCI interface.
This scenario models a pedestrian crossing which switches on demand.
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess


# the directory in which this script resides
THISDIR = os.path.dirname(__file__)


# we need to import python modules from the $SUMO_HOME/tools directory
# If the the environment variable SUMO_HOME is not set, try to locate the python
# modules relative to this script
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci  # noqa
from sumolib import checkBinary  # noqa
import randomTrips  # noqa


# this is the main entry point of this script
if __name__ == "__main__":
    net = 'unreal_layout_custom_widths_final_pedestrians.net.xml'

    randomTrips.main(randomTrips.get_options([
        '--net-file', net,
        '--output-trip-file', 'pedcrossing.trip.xml',
        '--route-file', 'pedcrossing.rou.xml',
        '--seed', '8',  # make runs reproducible
        '--prefix', 'car',
        '--additional-file', 'additional.xml',
        # prevent trips that start and end on the same edge
        '--min-distance', '100',
        '--max-distance', '800',
        '--trip-attributes', 'type=\"Mazada_3\"',
        '--edge-permission', 'passenger',
        '--end', '800',
        '--binomial', '4',
        '--period', '3']))

    # generate the pedestrians for this simulation
    randomTrips.main(randomTrips.get_options([
        '--net-file', net,
        '--output-trip-file', 'pedestrians.trip.xml',
        '--seed', '11',  # make runs reproducible
        '--pedestrians',
        '--prefix', 'ped',
        # prevent trips that start and end on the same edge
        '--min-distance', '1',
        '--trip-attributes', 'departPos="random" arrivalPos="random"',
        '--end', '500',
        '--binomial', '4',
        '--period', '3']))
