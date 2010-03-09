#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__author__ = 'Juan Gonzalez'
__copyright__ = 'Copyright (C) 2010 Juan Gonzalez (juan@iearobotics.com)'
__license__ = 'GPLv3 license'

#-- This is an example for testing the servocontroller located in the
#-- Modular Robot Open Rave plugin
#-- The two servos of the Minicube-I modular robot are both set to 45
#-- and -45 degrees.
#-- The Minicube-I modular robot is composed of two Y1 modules.


from openravepy import *
from numpy import *
import time
import sys

def run():

    #-- Read the name of the xml fiel passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'models/Minicube-I.env.xml'

    env = Environment()
    env.Load(file_env)
    env.SetViewer('qtcoin')

    with env:
        robot = env.GetRobots()[0]
        robot.SetController(env.CreateController('sinoscontroller'))
        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    #-- Set all amplitudes to 45
    A = [45 for j in robot.GetJoints()]
    robot.GetController().SendCommand('setamplitude '+' '.join(str(f) for f in A))

    #-- Set the initial phase to 0, 120, 240....
    phase0 = [i*120 for i in range(robot.GetDOF())]
    robot.GetController().SendCommand('setinitialphase '+' '.join(str(f) for f in phase0))

    #-- Set the period
    robot.GetController().SendCommand('setperiod 1.5');

    while True:
        time.sleep(1.0)


if __name__=='__main__':
    run()
