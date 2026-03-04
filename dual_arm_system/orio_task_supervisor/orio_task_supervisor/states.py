"""FSM state definitions for the task supervisor."""

from enum import Enum, auto


class SystemState(Enum):
    HOME                     = auto()
    WAIT_FOR_ITEM            = auto()
    ARM1_PICK                = auto()
    ARM1_PLACE_LABEL_STATION = auto()
    ARM2_PICK_LABEL          = auto()
    ARM2_PLACE_LABEL         = auto()
    ARM1_PICK_LABELLED_ITEM  = auto()
    ARM1_PLACE_DROP_ZONE     = auto()
    ERROR                    = auto()

    def __str__(self) -> str:
        return self.name


class ExecutionMode(Enum):
    """Discrete = one cycle then HOME; Continuous = loop indefinitely."""
    DISCRETE   = 'discrete'
    CONTINUOUS = 'continuous'
