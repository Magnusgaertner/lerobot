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
from contextlib import contextmanager, ExitStack


class MultiProtocolMotorBus:
    @property
    def motors(self) -> dict:
        all_motors = {}
        for bus in self.buses.values():
            all_motors.update(bus.motors)
        return all_motors
    
    @property
    def model_resolution_table(self) -> dict:
        resolution_table = {}
        for bus in self.buses.values():
            resolution_table.update(getattr(bus, "model_resolution_table", {}))
        return resolution_table
    """
    High-level wrapper for multiple MotorsBus instances, each handling a different protocol.
    Routes commands to the correct bus based on motor name.
    """

    def __len__(self):
        return sum(len(bus) for bus in self.buses.values())

    def __repr__(self):
        return f"MultiProtocolMotorBus(buses={list(self.buses.keys())}, motors={list(self.motor_to_bus.keys())})"

    @property
    def models(self) -> list[str]:
        models = []
        for bus in self.buses.values():
            models.extend(bus.models)
        return models

    @property
    def ids(self) -> list[int]:
        ids = []
        for bus in self.buses.values():
            ids.extend(bus.ids)
        return ids

    def _model_nb_to_model(self, motor_nb: int) -> str:
        for bus in self.buses.values():
            try:
                return bus._model_nb_to_model(motor_nb)
            except Exception:
                continue
        raise KeyError(f"Model number {motor_nb} not found in any bus.")

    def _id_to_model(self, motor_id: int) -> str:
        for bus in self.buses.values():
            if motor_id in bus.ids:
                return bus._id_to_model(motor_id)
        raise KeyError(f"Motor ID {motor_id} not found in any bus.")

    def _id_to_name(self, motor_id: int) -> str:
        for bus in self.buses.values():
            if motor_id in bus.ids:
                return bus._id_to_name(motor_id)
        raise KeyError(f"Motor ID {motor_id} not found in any bus.")

    def _get_motor_id(self, motor):
        bus = self.buses[self.motor_to_bus[motor]]
        return bus._get_motor_id(motor)

    def _get_motor_model(self, motor):
        bus = self.buses[self.motor_to_bus[motor]]
        return bus._get_motor_model(motor)

    def _get_motors_list(self, motors):
        if motors is None:
            all_motors = []
            for bus in self.buses.values():
                all_motors.extend(bus._get_motors_list(None))
            return all_motors
        elif isinstance(motors, str):
            return [motors]
        elif isinstance(motors, list):
            return motors
        else:
            raise TypeError("motors must be None, str, or list")

    def _get_ids_values_dict(self, values):
        # Forward to the correct bus based on motor name
        if isinstance(values, (int, float)):
            # Not enough info to route, return as is
            return values
        elif isinstance(values, dict):
            result = {}
            for bus_name, bus in self.buses.items():
                bus_values = {
                    m: v for m, v in values.items() if self.motor_to_bus[m] == bus_name
                }
                if bus_values:
                    result.update(bus._get_ids_values_dict(bus_values))
            return result
        else:
            raise TypeError("values must be int, float, or dict")

    def _validate_motors(self) -> None:
        for bus in self.buses.values():
            bus._validate_motors()

    def _is_comm_success(self, comm: int) -> bool:
        # Assume all buses use the same logic
        return list(self.buses.values())[0]._is_comm_success(comm)

    def _is_error(self, error: int) -> bool:
        return list(self.buses.values())[0]._is_error(error)

    def _assert_motors_exist(self) -> None:
        for bus in self.buses.values():
            bus._assert_motors_exist()

    def _assert_protocol_is_compatible(self, instruction_name: str) -> None:
        for bus in self.buses.values():
            bus._assert_protocol_is_compatible(instruction_name)

    def connect(self, handshake: bool = True) -> None:
        for bus in self.buses.values():
            bus.connect(handshake)

    def disconnect(self, disable_torque: bool = True) -> None:
        for bus in self.buses.values():
            bus.disconnect(disable_torque)

    @classmethod
    def scan_port(cls, port: str, *args, **kwargs) -> dict[int, list[int]]:
        # Not clear how to route, so call scan_port on all buses
        results = {}
        for bus in cls.buses.values():
            results.update(bus.scan_port(port, *args, **kwargs))
        return results

    def setup_motor(
        self,
        motor: str,
        initial_baudrate: int | None = None,
        initial_id: int | None = None,
    ) -> None:
        bus = self.buses[self.motor_to_bus[motor]]
        bus.setup_motor(motor, initial_baudrate, initial_id)

    def _find_single_motor(
        self, motor: str, initial_baudrate: int | None
    ) -> tuple[int, int]:
        bus = self.buses[self.motor_to_bus[motor]]
        return bus._find_single_motor(motor, initial_baudrate)

    def configure_motors(self) -> None:
        for bus in self.buses.values():
            bus.configure_motors()

    def disable_torque(self, motors=None, num_retry=0) -> None:
        # Route motors to correct bus
        if motors is None:
            for bus in self.buses.values():
                bus.disable_torque(None, num_retry)
        else:
            motors_list = self._get_motors_list(motors)
            for bus_name, bus in self.buses.items():
                bus_motors = [
                    m for m in motors_list if self.motor_to_bus[m] == bus_name
                ]
                if bus_motors:
                    bus.disable_torque(bus_motors, num_retry)

    def _disable_torque(self, motor, model, num_retry=0) -> None:
        # Find bus by motor name
        bus = self.buses[self.motor_to_bus[motor]]
        bus._disable_torque(motor, model, num_retry)

    def enable_torque(self, motors=None, num_retry=0) -> None:
        if motors is None:
            for bus in self.buses.values():
                bus.enable_torque(None, num_retry)
        else:
            motors_list = self._get_motors_list(motors)
            for bus_name, bus in self.buses.items():
                bus_motors = [
                    m for m in motors_list if self.motor_to_bus[m] == bus_name
                ]
                if bus_motors:
                    bus.enable_torque(bus_motors, num_retry)

    @contextmanager
    def torque_disabled(self, motors=None):
        motors_list = self._get_motors_list(motors)
        with ExitStack() as stack:
            for bus_name, bus in self.buses.items():
                bus_motors = [
                    m for m in motors_list if self.motor_to_bus[m] == bus_name
                ]
                if bus_motors:
                    stack.enter_context(bus.torque_disabled(bus_motors))
            yield

    def set_timeout(self, timeout_ms: int | None = None):
        for bus in self.buses.values():
            bus.set_timeout(timeout_ms)

    def get_baudrate(self) -> int:
        # Return baudrate of first bus (assume all same)
        return list(self.buses.values())[0].get_baudrate()

    def set_baudrate(self, baudrate: int) -> None:
        for bus in self.buses.values():
            bus.set_baudrate(baudrate)

    @property
    def is_calibrated(self) -> bool:
        return all(getattr(bus, "is_calibrated", False) for bus in self.buses.values())

    def read_calibration(self) -> dict[str, Any]:
        result = {}
        for bus in self.buses.values():
            result.update(bus.read_calibration())
        return result

    def write_calibration(
        self, calibration_dict: dict[str, Any], cache: bool = True
    ) -> None:
        for bus_name, bus in self.buses.items():
            bus_calib = {
                m: v
                for m, v in calibration_dict.items()
                if self.motor_to_bus[m] == bus_name
            }
            print(f"Bus {bus_name} calibration: {bus_calib}")
            if bus_calib:
                bus.write_calibration(bus_calib, cache)

    def reset_calibration(self, motors=None):
        motors_list = self._get_motors_list(motors)
        for bus_name, bus in self.buses.items():
            bus_motors = [m for m in motors_list if self.motor_to_bus[m] == bus_name]
            if bus_motors:
                bus.reset_calibration(bus_motors)

    def set_half_turn_homings(self, motors=None):
        motors_list = self._get_motors_list(motors)
        result = {}
        for bus_name, bus in self.buses.items():
            bus_motors = [m for m in motors_list if self.motor_to_bus[m] == bus_name]
            if bus_motors:
                result.update(bus.set_half_turn_homings(bus_motors))
        return result

    def _get_half_turn_homings(self, positions):
        # Route positions to correct bus
        result = {}
        for bus_name, bus in self.buses.items():
            bus_positions = {
                m: v for m, v in positions.items() if self.motor_to_bus[m] == bus_name
            }
            if bus_positions:
                result.update(bus._get_half_turn_homings(bus_positions))
        return result

    def record_ranges_of_motion(self, motors=None, display_values=True):
        motors_list = self._get_motors_list(motors)
        mins, maxes = {}, {}
        for bus_name, bus in self.buses.items():
            bus_motors = [m for m in motors_list if self.motor_to_bus[m] == bus_name]
            if bus_motors:
                bus_mins, bus_maxes = bus.record_ranges_of_motion(
                    bus_motors, display_values
                )
                mins.update(bus_mins)
                maxes.update(bus_maxes)
        return mins, maxes

    def _normalize(self, ids_values):
        # Route ids_values to correct bus
        result = {}
        for bus_name, bus in self.buses.items():
            bus_ids_values = {
                id_: v
                for id_, v in ids_values.items()
                if self.motor_to_bus[bus._id_to_name(id_)] == bus_name
            }
            if bus_ids_values:
                result.update(bus._normalize(bus_ids_values))
        return result

    def _unnormalize(self, ids_values):
        result = {}
        for bus_name, bus in self.buses.items():
            bus_ids_values = {
                id_: v
                for id_, v in ids_values.items()
                if self.motor_to_bus[bus._id_to_name(id_)] == bus_name
            }
            if bus_ids_values:
                result.update(bus._unnormalize(bus_ids_values))
        return result

    def _encode_sign(self, data_name, ids_values):
        result = {}
        for bus_name, bus in self.buses.items():
            bus_ids_values = {
                id_: v
                for id_, v in ids_values.items()
                if self.motor_to_bus[bus._id_to_name(id_)] == bus_name
            }
            if bus_ids_values:
                result.update(bus._encode_sign(data_name, bus_ids_values))
        return result

    def _decode_sign(self, data_name, ids_values):
        result = {}
        for bus_name, bus in self.buses.items():
            bus_ids_values = {
                id_: v
                for id_, v in ids_values.items()
                if self.motor_to_bus[bus._id_to_name(id_)] == bus_name
            }
            if bus_ids_values:
                result.update(bus._decode_sign(data_name, bus_ids_values))
        return result

    def _serialize_data(self, value, length):
        # Use first bus
        return list(self.buses.values())[0]._serialize_data(value, length)

    def _split_into_byte_chunks(self, value, length):
        return list(self.buses.values())[0]._split_into_byte_chunks(value, length)

    def ping(self, motor, num_retry=0, raise_on_error=False):
        bus = self.buses[self.motor_to_bus[motor]]
        return bus.ping(motor, num_retry, raise_on_error)

    def broadcast_ping(self, num_retry=0, raise_on_error=False):
        result = {}
        for bus in self.buses.values():
            ping_result = bus.broadcast_ping(num_retry, raise_on_error)
            if ping_result:
                result.update(ping_result)
        return result

    def read(self, data_name, motor, *, normalize=True, num_retry=0):
        bus = self.buses[self.motor_to_bus[motor]]
        return bus.read(data_name, motor, normalize=normalize, num_retry=num_retry)

    def _read(
        self, address, length, motor_id, *, num_retry=0, raise_on_error=True, err_msg=""
    ):
        for bus in self.buses.values():
            if motor_id in bus.ids:
                return bus._read(
                    address,
                    length,
                    motor_id,
                    num_retry=num_retry,
                    raise_on_error=raise_on_error,
                    err_msg=err_msg,
                )
        raise KeyError(f"Motor ID {motor_id} not found in any bus.")

    def write(self, data_name, motor, value, *, normalize=True, num_retry=0):
        bus = self.buses[self.motor_to_bus[motor]]
        bus.write(data_name, motor, value, normalize=normalize, num_retry=num_retry)

    def _write(
        self,
        addr,
        length,
        motor_id,
        value,
        *,
        num_retry=0,
        raise_on_error=True,
        err_msg="",
    ):
        for bus in self.buses.values():
            if motor_id in bus.ids:
                return bus._write(
                    addr,
                    length,
                    motor_id,
                    value,
                    num_retry=num_retry,
                    raise_on_error=raise_on_error,
                    err_msg=err_msg,
                )
        raise KeyError(f"Motor ID {motor_id} not found in any bus.")

    def sync_read(self, data_name, motors=None, *, normalize=True, num_retry=0):
        motors_list = self._get_motors_list(motors)
        result = {}
        for bus_name, bus in self.buses.items():
            bus_motors = [m for m in motors_list if self.motor_to_bus[m] == bus_name]
            if bus_motors:
                result.update(
                    bus.sync_read(
                        data_name, bus_motors, normalize=normalize, num_retry=num_retry
                    )
                )
        return result

    def _sync_read(
        self, addr, length, motor_ids, *, num_retry=0, raise_on_error=True, err_msg=""
    ):
        result = {}
        for bus in self.buses.values():
            bus_motor_ids = [id_ for id_ in motor_ids if id_ in bus.ids]
            if bus_motor_ids:
                bus_result, comm = bus._sync_read(
                    addr,
                    length,
                    bus_motor_ids,
                    num_retry=num_retry,
                    raise_on_error=raise_on_error,
                    err_msg=err_msg,
                )
                result.update(bus_result)
        return result, 0  # comm not aggregated

    def _setup_sync_reader(self, motor_ids, addr, length):
        for bus in self.buses.values():
            bus_motor_ids = [id_ for id_ in motor_ids if id_ in bus.ids]
            if bus_motor_ids:
                bus._setup_sync_reader(bus_motor_ids, addr, length)

    def sync_write(self, data_name, values, *, normalize=True, num_retry=0):
        for bus_name, bus in self.buses.items():
            bus_values = {
                m: v for m, v in values.items() if self.motor_to_bus[m] == bus_name
            }
            if bus_values:
                bus.sync_write(
                    data_name, bus_values, normalize=normalize, num_retry=num_retry
                )

    def _sync_write(
        self, addr, length, ids_values, num_retry=0, raise_on_error=True, err_msg=""
    ):
        for bus in self.buses.values():
            bus_ids_values = {id_: v for id_, v in ids_values.items() if id_ in bus.ids}
            if bus_ids_values:
                bus._sync_write(
                    addr,
                    length,
                    bus_ids_values,
                    num_retry=num_retry,
                    raise_on_error=raise_on_error,
                    err_msg=err_msg,
                )

    def _setup_sync_writer(self, ids_values, addr, length):
        for bus in self.buses.values():
            bus_ids_values = {id_: v for id_, v in ids_values.items() if id_ in bus.ids}
            if bus_ids_values:
                bus._setup_sync_writer(bus_ids_values, addr, length)

    @property
    def is_calibrated(self) -> bool:
        return all(getattr(bus, "is_calibrated", False) for bus in self.buses.values())

    @property
    def is_connected(self) -> bool:
        return all(bus.is_connected for bus in self.buses.values())

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
            bus_values = {
                m: v for m, v in values.items() if self.motor_to_bus[m] == bus_name
            }
            if bus_values:
                bus.sync_write(data_name, bus_values, **kwargs)
