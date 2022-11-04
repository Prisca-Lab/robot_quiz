from enum import Enum, IntEnum

class FaceConditions(IntEnum):
    FRONT = 1
    SIDE = 2


class PersonalityConditions(IntEnum):
    ANTAGONIST = 1
    AGREEABLENESS = 2


class ExperimentConditions(Enum):
    """Define the experiment conditions
    Note: each key definition has to contain an integer identifier
    """
    _1_FRONT_ANTAGONIST = FaceConditions.FRONT, PersonalityConditions.ANTAGONIST
    _2_FRONT_AGREEABLENESS = FaceConditions.FRONT, PersonalityConditions.AGREEABLENESS
    _3_SIDE_ANTAGONIST = FaceConditions.SIDE, PersonalityConditions.ANTAGONIST
    _4_SIDE_AGREEABLENESS = FaceConditions.SIDE, PersonalityConditions.AGREEABLENESS

    def __init__(self, face, personality):
        self.face = face
        self.personality = personality

    def __str__(self):
        return f"{self.face.name} {self.personality.name}"

    @classmethod
    def value_of(cls, value):
        # search for the integer identifier in the enum keys
        for k, v in cls.__members__.items():
            if str(value) in k:
                return v
        else:
            raise ValueError(f"'{cls.__name__}' enum not found for '{value}'")