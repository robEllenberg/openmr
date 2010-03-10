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

    #-- Configure the camera view
    Tcamera = array (((-0.34606934, -0.51347446,  0.78522629,  0.49086213),
                      ( 0.93809277, -0.17619586,  0.29822361,  0.21498504),
                      (-0.01477666,  0.83982098,  0.54266238,  0.28141484)))
    env.SetCamera(Tcamera)

    with env:
        robot = env.GetRobots()[0]
        robot.SetController(env.CreateController('servocontroller'))
        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    starttime = time.time()
    while True:
        #-- Set the servos position to 45 degrees
        ref_pos = [45 for j in robot.GetJoints()]
        robot.GetController().SendCommand('setpos '+' '.join(str(f) for f in ref_pos))
        time.sleep(1.0)

	#-- Set the servos position to -45 degrees
	ref_pos = [-45 for j in robot.GetJoints()]
        robot.GetController().SendCommand('setpos '+' '.join(str(f) for f in ref_pos))
        time.sleep(1.0)

if __name__=='__main__':
    run()
