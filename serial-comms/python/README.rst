--------------------------------
Python Library for Beluga Serial
--------------------------------

This is the Python Library for communicating with a Beluga node over serial. The library implements a producer-consumer
system where there are at least 3 processes running. The user-defined process for sending commands and consuming the
data from Beluga, a serial read process for reading data from Beluga and decoding the data, and a dispatching process
for processing the data and dispatching it to the proper locations.

There are two different ways the data can be retrieved: through the queues or through callback functions. However,
when registering a callback function, one must note that it will be called in a different process which may cause
issues.
