# Python program showing
# abstract base class work

from abc import ABC, abstractmethod

class BehaviourMode(ABC):
    def __init__(self, name):
        self.name = name
        self._data = None
        print(f"Created mode {name}")

    @abstractmethod
    def execute(self, data):
       """Execute the behaviour"""

    @abstractmethod
    def stop(self):
        """Stop the behaviour"""

    @property
    def data(self):
        print("Getting value...")
        return self._data

    @data.setter
    def data(self, value):
        print("Setting value...")
        self._data = value
