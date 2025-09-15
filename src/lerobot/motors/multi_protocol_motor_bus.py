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

from typing import Dict, Any
from lerobot.motors.motors_bus import MotorsBus

class MultiProtocolMotorBus:
    @property
    def is_connected(self) -> bool:
        return all(bus.is_connected for bus in self.buses.values())
    """
    High-level wrapper for multiple MotorsBus instances, each handling a different protocol.
    Routes commands to the correct bus based on motor name.
    """
    def __init__(self, buses: Dict[str, MotorsBus], motor_to_bus: Dict[str, str]):
        """
        Args:
            buses: Dict mapping protocol name (e.g. 'p0', 'p1') to MotorsBus instance.
            motor_to_bus: Dict mapping motor name to protocol name.
        """
        self.buses = buses
        self.motor_to_bus = motor_to_bus

    def read(self, data_name: str, motor: str, **kwargs) -> Any:
        bus = self.buses[self.motor_to_bus[motor]]
        return bus.read(data_name, motor, **kwargs)

    def write(self, data_name: str, motor: str, value: Any, **kwargs) -> None:
        bus = self.buses[self.motor_to_bus[motor]]
        bus.write(data_name, motor, value, **kwargs)

    def connect(self):
        for bus in self.buses.values():
            bus.connect()

    def disconnect(self):
        for bus in self.buses.values():
            bus.disconnect()

    def sync_read(self, data_name: str, motors: list[str], **kwargs) -> Dict[str, Any]:
        result = {}
        for bus_name, bus in self.buses.items():
            bus_motors = [m for m in motors if self.motor_to_bus[m] == bus_name]
            if bus_motors:
                result.update(bus.sync_read(data_name, bus_motors, **kwargs))
        return result

    def sync_write(self, data_name: str, values: Dict[str, Any], **kwargs) -> None:
        for bus_name, bus in self.buses.items():
            bus_values = {m: v for m, v in values.items() if self.motor_to_bus[m] == bus_name}
            if bus_values:
                bus.sync_write(data_name, bus_values, **kwargs)
