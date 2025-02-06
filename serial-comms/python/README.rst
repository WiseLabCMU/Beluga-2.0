--------------------------------
Python Library for Beluga Serial
--------------------------------

This is the Python Library for communicating with a Beluga node over serial. The library implements a producer-consumer
system where there are at least 3 processes running. The user-defined process for sending commands and consuming the
data from Beluga, a serial read process for reading data from Beluga and decoding the data, and a dispatching process
for processing the data and dispatching it to the proper locations.

Interactions
------------
There are two different ways the data can be retrieved: through the queues or through callback functions.

Message Queues
^^^^^^^^^^^^^^
BelugaSerial contains 3 message queues that can be read from: The neighbor updates queue, range updates queue, and the
range exchange queue. Each of these queues have a depth of 1 meaning they only contain the latest reported data. When
reading the data from the queue, the data is removed from the the queue, and the queue remains empty until the processing
task inserts new data into the queue.

Callback Functions
^^^^^^^^^^^^^^^^^^
BelugaSerial allows the use of callback functions as an alternative to queues. However, keep in mind that callback
functions should have limited execution time, as they are called directly from the processing task.

.. warning::
    BelugaSerial uses the multiprocessing library to reduce response and processing times. If the callback function
    relies on something tied to the parent process, using a callback is not recommended.

    For example, if you're using this in a ROS node that publishes ranges, and you pass a publisher method as a
    callback, nothing will be published. This is because the task tries to publish ranges in a separate process,
    and ROS expects all messages to be published in the same process that ROS and the publishers were initialized in.

Installing
----------

.. code-block:: bash

    pip install git+https://github.com/WiseLabCMU/Beluga-2.0.git#subdirectory=serial-comms/python

Beluga Serial Command Shell
---------------------------
The python package comes with a shell that can be ran within the environment. To invoke it, run the following command
in your Python environment

.. code-block:: bash

    beluga-miniterm
