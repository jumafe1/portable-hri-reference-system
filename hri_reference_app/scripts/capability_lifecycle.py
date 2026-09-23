"""Reusable YASMIN states for the Capabilities2 acquisition lifecycle."""

from capabilities2_msgs.srv import EstablishBond, FreeCapability, UseCapability
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import ABORT, SUCCEED, TIMEOUT


SERVICE_WAIT_SECONDS = 1.0
SERVICE_RESPONSE_SECONDS = 10.0
SERVICE_RETRIES = 10


def service_options():
    """Return bounded waits shared by all Capabilities2 service states."""
    return {
        "wait_timeout": SERVICE_WAIT_SECONDS,
        "response_timeout": SERVICE_RESPONSE_SECONDS,
        "maximum_retry": SERVICE_RETRIES,
    }


class EstablishBondState(ServiceState):
    """Create a Capabilities2 bond and store it on the blackboard."""

    def __init__(self):
        super().__init__(
            EstablishBond,
            "/capabilities/establish_bond",
            lambda _: EstablishBond.Request(),
            response_handler=self.handle_response,
            **service_options(),
        )

    @staticmethod
    def handle_response(blackboard, response):
        if not response.bond_id:
            blackboard["message"] = "capabilities2_returned_empty_bond"
            return ABORT
        blackboard["bond_id"] = response.bond_id
        return SUCCEED


class UseCapabilityState(ServiceState):
    """Acquire the capability and provider named in the blackboard."""

    def __init__(self):
        super().__init__(
            UseCapability,
            "/capabilities/use_capability",
            self.create_request,
            **service_options(),
        )

    @staticmethod
    def create_request(blackboard):
        return UseCapability.Request(
            capability=blackboard["capability"],
            preferred_provider=blackboard["preferred_provider"],
            bond_id=blackboard["bond_id"],
        )


class FreeCapabilityState(ServiceState):
    """Release the acquired capability and record successful cleanup."""

    def __init__(self):
        super().__init__(
            FreeCapability,
            "/capabilities/free_capability",
            self.create_request,
            response_handler=self.handle_response,
            **service_options(),
        )

    @staticmethod
    def create_request(blackboard):
        return FreeCapability.Request(
            capability=blackboard["capability"],
            bond_id=blackboard["bond_id"],
        )

    @staticmethod
    def handle_response(blackboard, _response):
        blackboard["released"] = True
        return SUCCEED
