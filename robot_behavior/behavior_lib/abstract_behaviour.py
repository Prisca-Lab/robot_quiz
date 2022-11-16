from abc import ABC, abstractmethod
import logging 
log = logging.getLogger(__name__)
class BehaviourMode(ABC):
    def __init__(self, name):
        self.name = name
        self._data = None
        log.debug(f"Created mode {name}")

    @abstractmethod
    def execute(self, data):
       """Execute the behaviour"""

    @abstractmethod
    def stop(self):
        """Stop the behaviour"""

    @property
    def data(self):
        log.debug(f"Getting value for {self.name}")
        return self._data

    @data.setter
    def data(self, value):
        log.debug(f"Setting value: {value} for {self.name}")
        self._data = value
