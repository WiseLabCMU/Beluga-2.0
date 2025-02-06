-----------------------------
C++ Library for Beluga Serial
-----------------------------

This is the C++ library for communicating with a Beluga node over serial. The library implements 2 things:

1. A serial communication library designed to communicate with most serial devices on the Linux operating system.
2. The implementation of the communications with a Beluga node as described in the architecture diagram

The library implements a producer-consumer system where there are at least 3 threads running: The user-defined thread
for sending commands to Beluga and consuming the data from Beluga, a serial read thread for reading data from Beluga
and decoding the data, and a dispatching thread for processing the data from Beluga and dispatching the data to the
proper locations.

Installing/Using
-------------------------------------

The C++ library comes packaged with a CMake file meaning that it can easily be integrated into a project that uses
a CMake build system.

Downloading and using in cmake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The first option (and least convenient) is to download the beluga directory and drop it right into your project.
After that, add the following to your CMakeLists.txt file:

.. code-block:: cmake

    add_subdirectory(beluga-serial)
    target_link_libraries( executable-name PRIVATE beluga-serial )

Including in cmake project via FetchContent
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The second option (preferable one) is to include the package via FetchContent

.. code-block:: cmake

    include( FetchContent )
    FetchContent_Declare(
        beluga-serial
        GIT_REPOSITORY https://github.com/WiseLabCMU/Beluga-2.0.git
        GIT_TAG master
        SOURCE_SUBDIR serial-comms/cpp/beluga-serial
    )
    FetchContent_MakeAvailable(beluga-serial)

    target_link_libraries( target-name PRIVATE beluga-serial )
