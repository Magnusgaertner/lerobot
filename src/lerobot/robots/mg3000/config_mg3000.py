#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field
from typing import Dict

@dataclass
class JointConfig:
    max_torque: int = 100
    p: int = 0
    i: int = 0
    d: int = 0
    max_velocity: int = 0
    max_acceleration: int = 0

from lerobot.cameras import CameraConfig
from ..config import RobotConfig

@RobotConfig.register_subclass("mg3000")
@dataclass
class MG3000Config(RobotConfig):
    port: str
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | Dict[str, float] | None = None
    cameras: Dict[str, CameraConfig] = field(default_factory=dict)
    use_degrees: bool = False
    joints: Dict[str, JointConfig] = field(default_factory=dict)
