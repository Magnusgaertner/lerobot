import scservo_sdk as scs
from scservo_sdk import SCS_SETEND

class SafePacketHandler:
    def __init__(self, protocol_version=0):
        self.protocol_version = protocol_version
        self._handler = scs.PacketHandler(protocol_version)

    def __getattr__(self, name):
        # Delegate all attribute access to the underlying PacketHandler
        SCS_SETEND(self.protocol_version)
        return getattr(self._handler, name)
