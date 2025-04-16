from enum import Enum, auto


class ZeroBasedEnum(Enum):
    """
    Generic enum class where values start at 0 instead of 1
    """
    def _generate_next_value_(name, start, count, last_values):
        """
        Generate the next value when not given.

        name: the name of the member
        start: the initial start value or None
        count: the number of existing members
        last_value: the last value assigned or None
        """
        return count
