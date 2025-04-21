==========
Beluga GUI
==========

Interface application for easily interacting with a Beluga Modem.

Overview
========
The GUI makes it easier to configure and get readings from a
Beluga Modem without having to learn the AT commands. The GUI
consists of 3 tabs: The terminal (does not work), configuration,
and ranges tabs. Additionally, the Beluga GUI also makes it
easier to collect data for evaluation purposes.

.. warning::

    This application is not built for production use. There
    are definitely a ton of bugs that I do not have the time
    nor the motivation to track down and fix.

Requirements
============

- Python3.9 or newer
- Qt5
- MacOS or Linux

.. note::

    Windows 10 or 11 is not supported. This is because the serial
    communication does not work in Windows for some reason.

Tabs
====
Terminal Tab
------------
This tab does not work. Don't use it.

Configuration Tab
-----------------
The configuration tab is where a Beluga modem can be connected to
and configured. Upon connection, the GUI will sync with the modem
to get all the current settings. Additionally, when UWB is active,
the GUI will not allow the user to configure settings that require
the UWB to be disabled. The modem can also be rebooted from the
configuration tab.

Ranges Tab
----------
The ranges tab is where the ranging data is displayed. There
are 4 ways available to view the data: table format, distance over time,
RSSI over time, and distance compared to RSSI. Ranging can also be
stopped and started from the ranging tab. This allows for evaluation
of the data without the live updates. Additionally, this tab can
collect data to assist in constructing a channel model.

Data Collection
===============
The ranges tab comes with a button to record data. If pressed,
a dialog will pop up requesting a file name (press the ... button to specify),
number of samples, and the timeout. If an existing file is selected,
to save the recorded data, then it will not overwrite it if the
data capture is canceled or times out. Data can be saved in 3
formats: as a .meas file, as a log file, or as a JSON file. The .meas
and the log files use the same format while JSON files are saved as JSON.
The saved files record the UWB settings, the number of trx failures at
each stage with respect to the initiator, and the specified number of samples
from ranging.

Installation
============
It is recommended that this application is installed in a Python virtual
environment.

Linux/MacOS
-----------

.. code-block:: bash

    python3 -m venv .venv
    source .venv/bin/activate
    pip install git+https://github.com/WiseLabCMU/Beluga-2.0.git#subdirectory=interface_app
    beluga_gui  # This should launch the application

Building a binary (Linux Only)
------------------------------
1. Clone the repo
2. Navigate to the interface_app directory
3. ``python3 -m venv .venv``
4. ``source .venv/bin/activate``
5. ``pip install .``
6. ``pyinstaller beluga_gui.spec``
