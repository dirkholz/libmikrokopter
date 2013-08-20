libmikrokopter
==============

libmikrokopter is a simple C++ library for communicating with MikroKopter_ micro aerial vehicles (quadcopters, hexacopters, and octocopters). It features: 

- serial communication, 
- the MikroKopter UART protocols for both NaviCtrl and FlightCtrl, and 
- a simple command-line tool for displaying received debug output and sending an external control signal. 

License
^^^^^^^

The library is provided under BSD license (see LICENSE.txt). For protocol details, please refer to `original license file`__ from MikroKopter_.

.. _MikroKopter: http://www.mikrokopter.de/en/home
.. _FCLICENSE: http://svn.mikrokopter.de/filedetails.php?repname=FlightCtrl&path=%2FLICENSE.TXT
__ FCLICENSE_



Get and build libmikrokopter
============================

source
^^^^^^

We use git for our source control. You can get the latest stable version of the library by doing the following::

   git clone git://github.com/dirkholz/libmikrokopter.git

dependencies
^^^^^^^^^^^^
libmikrokopter requires Boost for serial IO. On Ubuntu systems you can get the required libraries through apt::

   sudo apt-get install libboost-system-dev libboost-thread-dev


build and install
^^^^^^^^^^^^^^^^^
To build you should just follow a normal cmake recipe::
   
   cd libmikrokopter
   mkdir -p build
   cd build
   cmake ..
   make
   sudo make install

uninstall
^^^^^^^^^
To uninstall resetusb simply run::

   cd resetusb/build
   sudo make uninstall


Testing 
--------

In order to test the library and the connected MikroKopter simply run::

    test_kopter

The program outputs both the received debug values as well as a string description of the values (here a patched version of the firmware is used)::

    void mikrokopter::Kopter::printFlightControlDebugData(const mikrokopter::protocol::FlightControlDebugData&, const string (&)[32])
    Update Interval: 43ms.
    AngleNick        = -42
    AngleRoll        = -32
    AccNick          = -21
    AccRoll          = -44
    YawGyro          = -23
    Altitude [0.1m]  = 0
    AccZ             = 685
    Gas              = 412
    Compass Value    = -1
    Voltage [0.1V]   = 203
    Receiver Level   = 0
    Gyro Compass     = 0
    AdWertRoll       = 1022
    AdWertNick       = 1010
    AdWertGier       = 1039
    Motor 4          = 0
    Stick Nick       = 0
    Stick Roll       = 0
    Stick Gier       = 0
    Stick Gas        = 127
    Servo            = 535
    Hovergas         = 0
    Current [0.1A]   = 5
    Capacity [mAh]   = 317
    Height Setpoint  = 0
    External Control = 0
    26               = 0
    Compass Setpoint = -1
    I2C-Error        = 0
    BL Limit         = 250
    GPS_Nick         = 0
    GPS_Roll         = 0


