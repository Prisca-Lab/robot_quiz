from enum import Enum, IntEnum

class BodyConditions(IntEnum):
    FRONT = 1
    SIDE = 2


class PersonalityConditions(IntEnum):
    ANTAGONIST = 1
    AGREEABLENESS = 2


class ExperimentConditions(Enum):
    """Define the experiment conditions
    Note: each key definition has to contain an integer identifier
    """
    # _1_FRONT_ANTAGONIST = BodyConditions.FRONT, PersonalityConditions.ANTAGONIST
    # _2_FRONT_AGREEABLENESS = BodyConditions.FRONT, PersonalityConditions.AGREEABLENESS
    _1_SIDE_ANTAGONIST = BodyConditions.SIDE, PersonalityConditions.ANTAGONIST
    _2_SIDE_AGREEABLENESS = BodyConditions.SIDE, PersonalityConditions.AGREEABLENESS

    def __init__(self, body, personality):
        self.body = body
        self.personality = personality

    def __str__(self):
        return f"{self.body.name} {self.personality.name}"

    @classmethod
    def value_of(cls, value):
        # search for the integer identifier in the enum keys
        for k, v in cls.__members__.items():
            if str(value) in k:
                return v
        else:
            raise ValueError(f"'{cls.__name__}' enum not found for '{value}'")