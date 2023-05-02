from abc import ABC, abstractmethod


class DataSource(ABC):
    """Represents a Source of data, like a http-server etc."""

    # TODO: implement
    @abstractmethod
    def get_data():
        pass
