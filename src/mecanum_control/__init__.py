#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mecanum wheel control package for ROS.

This package provides a ROS interface for controlling a mecanum wheeled vehicle
through CAN communication, featuring smooth trajectory planning and various
control modes including forward, backward, lateral, and rotational movements.
"""

from .motor_controller import MotorController
from .mecanum_controller import MecanumController
from .python_can_controller import PythonCANController
from .trajectory_planner import TrajectoryPlanner

# 定义公共接口
__all__ = [
    'MotorController',
    'MecanumController',
    'PythonCANController',
    'TrajectoryPlanner'
]

# 版本信息
__version__ = '1.0.0'

# 作者信息
__author__ = 'Your Name'
__email__ = 'your.email@example.com'
__maintainer__ = 'Your Name'

# 包说明
__description__ = 'ROS package for controlling mecanum wheeled vehicles using CAN communication'