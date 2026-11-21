=========
SocketCAN
=========

SocketCAN Device Drivers
========================

- ``include/nuttx/net/netdev.h``. All structures and APIs
  needed to work with drivers are provided in this header file.
  The structure struct net_driver_s defines the interface and is
  passed to the network via ``netdev_register()``.

- ``include/nuttx/can.h``. CAN & CAN FD frame data structures and dlc to len size

  .. code-block:: c

     uint8_t can_bytes2dlc(uint8_t nbytes);
     uint8_t can_dlc2bytes(uint8_t dlc);

- ``int netdev_register(FAR struct net_driver_s *dev, enum net_lltype_e lltype)'``.
  Each driver registers itself by calling ``netdev_register()``.

- ``Include/nuttx/net/can.h``. contains lookup tables for CAN
  dlc to CAN FD len sizes named

- **Initialization sequence is as follows**.

  #. ``xxx_netinitialize(void)`` is called on startup of NuttX in this
     function you call your own init function to initialize your
     CAN driver
  #. In your own init function you create the net_driver_s
     structure set required init values and register the required
     callbacks for SocketCAN
  #. Then you ensure that the CAN interface is in down mode
     (usually done by calling the d_ifdown function)
  #. Register the net_driver_s using netdev_register

- **Receive sequence is as follows**.

  #. Device generates interrupt
  #. Process this interrupt in your interrupt handler
  #. When a new CAN frame has been received you process this frame
  #. When the CAN frame is a normal CAN frame you allocate the
     can_frame struct, when it's a CAN FD frame you allocate a
     canfd_frame struct (note you can of course preallocate and
     just use the pointer).
  #. Copy the frame from the driver to the struct you've
     allocated in the previous step.
  #. Point the net_driver_s d_buf pointer to the allocated can_frame
  #. Call the ``can_input(FAR struct net_driver_s *dev)``
     function ``include/nuttx/net/can.h``

- **Transmit sequence is as follows**.

  #. Socket layer executes d_txavail callback
  #. An example of the txavail function can be found in
     ``arch/arm/src/s32k1xx/s32k1xx_flexcan.c``
  #. An example of the txpoll function can be found in
     ``arch/arm/src/s32k1xx/s32k1xx_flexcan.c``
  #. In your ``transmit(struct driver_s *priv)`` function you
     check the length of ``net_driver_s.d_len`` whether it
     matches the size of a ``struct can_frame`` or
     ``struct canfd_frame`` then you cast the content of the
     ``net_driver_s.d_buf`` pointer to the correct CAN frame struct

SocketCAN protocol stack
========================

SocketCAN is a CAN protocol stack implementation based on the BSD socket API,
providing a more standardized and flexible CAN communication interface.
SocketCAN uses the network protocol stack framework, allowing applications
to use standard socket system calls (such as ``socket()``, ``bind()``,
``send()``, ``recv()``, etc.) for CAN communication.

Architecture
------------

The SocketCAN implementation follows the standard network layer hierarchy:

#. **Application Layer Interface**: Uses standard socket API
   (AF_CAN address family)
#. **Protocol Layer**: CAN protocol processing (located in ``net/can/``)
#. **Device Layer**: CAN network device drivers

File Locations
--------------

Files supporting SocketCAN can be found in the following locations:

-  **Protocol Implementation**: The SocketCAN protocol stack is located in
   the ``net/can/`` directory
-  **Header File**: ``include/nuttx/net/can.h``
-  **Main Modules**:

   - ``can.c`` - Protocol-independent core: connection management, input,
     callbacks and polling
   - ``can_raw.c`` - CAN_RAW protocol: socket interface, transmission (with
     and without buffering), reception, filters and socket options
   - ``can_sockif.c`` - Selection of the socket interface of a CAN protocol

Configuration Options
---------------------

To enable SocketCAN, configure the following options:

-  ``CONFIG_NET_CAN`` - Enable SocketCAN support
-  ``CONFIG_NET_CAN_NOTIFIER`` - Enable CAN notifier (optional)
-  ``CONFIG_NET_CAN_NBUFFERS`` - Number of CAN buffers
-  ``CONFIG_NET_RECV_BUFSIZE`` - Receive buffer size

Usage Notes
-----------

SocketCAN uses the standard socket programming model:

.. code-block:: c

   /* Create CAN socket */
   int sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);

   /* Bind to CAN interface */
   struct sockaddr_can addr;
   addr.can_family = AF_CAN;
   addr.can_ifindex = if_nametoindex("can0");
   bind(sock, (struct sockaddr *)&addr, sizeof(addr));

   /* Send CAN frame */
   struct can_frame frame;
   frame.can_id = 0x123;
   frame.can_dlc = 8;
   /* Fill data */
   send(sock, &frame, sizeof(frame), 0);

   /* Receive CAN frame */
   recv(sock, &frame, sizeof(frame), 0);

Local Loopback
--------------

As in Linux, ``CAN_RAW_LOOPBACK`` is enabled by default: a frame sent on a
CAN_RAW socket is also delivered to the other CAN_RAW sockets bound to the
same interface on the same node (subject to their filters).  The sending
socket receives its own frames only with ``CAN_RAW_RECV_OWN_MSGS``; those
copies are flagged with ``MSG_CONFIRM`` in ``msg_flags`` and can be used as a
transmit confirmation.  Disable the loopback with:

.. code-block:: c

   int off = 0;
   setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &off, sizeof(off));

If the network device is not a loopback device, the stack loops the frame
back itself after the driver accepted it.  Looped back frames are counted in
the received CAN statistics.

Socket Address
--------------

``getsockname()`` returns a ``struct sockaddr_can`` with ``can_family`` set
to ``AF_CAN`` and ``can_ifindex`` set to the index of the interface the
socket is bound to, or 0 if the socket is not bound to an interface.  The
address buffer must be at least ``sizeof(struct sockaddr_can)`` bytes,
otherwise ``EINVAL`` is returned.

Features
--------

-  **Standard Socket API**: Uses familiar socket programming interface
-  **Filtering Support**: Set CAN ID filters via socket options
-  **Non-blocking I/O**: Supports non-blocking mode and polling
-  **Multiple Connections**: Supports multiple sockets accessing the same CAN bus simultaneously
-  **Read Buffering**: Supports data read buffering to prevent data loss
-  **CAN FD Support**: Supports CAN FD frames if enabled in configuration
-  **Extensible**: Easy to extend for additional CAN protocols


ISO-TP
======

ISO-TP (``CAN_ISOTP``) is a socket protocol that provides message-oriented
communication over CAN. It is enabled with ``CONFIG_NET_CAN_ISOTP=y`` option.

By default a message may carry up to 4095 bytes, the maximum length the
classic 12-bit First Frame ``FF_DL`` field can encode. Selecting
``CONFIG_LIBC_CAN_ISOTP_LONG_PDU=y`` enables the 32-bit ``FF_DL`` escape
sequence (ISO 15765-2:2016), extending the maximum message length to
2^32-1 bytes at the cost of a larger First Frame header and wider internal
length fields.
Leave it disabled on classic-CAN-only configurations that never transfer long
messages so no extra code or memory is spent. A ``send`` of a message larger
than the configured limit fails with ``EMSGSIZE``.

A socket is created with ``AF_CAN`` / ``SOCK_DGRAM`` / ``CAN_ISOTP`` and
bound to a ``sockaddr_can`` specifying the interface and the TX/RX CAN IDs
via ``can_addr.tp.tx_id`` and ``can_addr.tp.rx_id``. Each ``send`` call
transmits one complete message; each ``recv`` call returns one complete
reassembled message.

Segmentation, flow control, and reassembly are handled transparently by the
kernel. TX waits up to 15000 ms for a Flow Control frame after a First Frame
(N_Bs); RX waits up to 1000 ms for Consecutive Frames after a First Frame
(N_Cr).

The sender honors every Flow Control the receiver may send. STmin values
0x00-0x7F are milliseconds and 0xF1-0xF9 are 100-900 us; a reserved value
counts as 127 ms. A WAIT status starts the N_Bs wait again. An overflow
status ends the transfer with ``EMSGSIZE``, and a reserved status or a Flow
Control shorter than 3 bytes with ``EBADMSG``; ``send`` returns the error
with ``CAN_ISOTP_WAIT_TX_DONE``. This receiver never sends WAIT, so
``wftmax`` in ``CAN_ISOTP_RECV_FC`` is accepted and not used.

``CAN_ISOTP_TX_PADDING`` pads every frame sent, flow control included, to 8
bytes with ``txpad_content``. CAN FD frames are always padded up to the next
valid CAN FD length, with ``txpad_content`` or, without
``CAN_ISOTP_TX_PADDING``, with 0xCC. On receive, ``CAN_ISOTP_CHK_PAD_LEN``
asks for frames padded to at least 8 bytes with ``CAN_ISOTP_RX_PADDING``,
or for frames no longer than needed without it, and
``CAN_ISOTP_CHK_PAD_DATA`` asks for padding bytes equal to
``rxpad_content``. A message that fails the check is dropped and ``recv``
returns ``EBADMSG``; a flow control that fails it ends the transfer with
``EBADMSG``.

``CAN_ISOTP_EXTEND_ADDR`` selects extended (or mixed) addressing: every frame
sent starts with ``ext_address``, and only frames that start with
``rx_ext_address`` (with ``CAN_ISOTP_RX_EXT_ADDR``) or else ``ext_address``
are received. The address byte leaves one byte less for data in each frame.

A ``CAN_ISOTP_LISTEN_MODE`` socket receives messages like any other but sends
no flow control, so it can follow a transfer between two other nodes.

With ``CAN_ISOTP_HALF_DUPLEX`` a socket drops every frame but flow control
while it sends a message, and flow control while it receives one.

``CAN_ISOTP_FORCE_TXSTMIN`` replaces the receiver's STmin with the time in ns
set by the ``CAN_ISOTP_TX_STMIN`` option. ``CAN_ISOTP_FORCE_RXSTMIN`` drops
consecutive frames that come sooner than the ``CAN_ISOTP_RX_STMIN`` option
after the last one taken. ``frame_txtime`` in ``CAN_ISOTP_OPTS`` is added
between consecutive frames. It starts at 0 rather than Linux's 50 us,
because frames go to the CAN controller's queue and the bus paces them; 0
keeps the current value and ``CAN_ISOTP_FRAME_TXTIME_ZERO`` sets 0.
