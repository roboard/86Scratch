__author__ = 'Copyright (c) 2018 Android Lin All rights reserved.'
"""

Copyright (c) 2018 Android Lin <acen@dmp.com.tw> All rights reserved.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU  General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
"""

import threading
import time
import struct
import ctypes


class PyMataUICommandHandler(threading.Thread):
    """
    This class handles all data interchanges with Firmata
    The receive loop runs in its own thread.

    Messages to be sent to Firmata are queued through a deque to allow for priority
    messages to take precedence. The deque is checked within the receive loop for any
    outgoing messages.

    There is no blocking in either communications direction.

    There is blocking when accessing the data tables through the _data_lock
    """
    
    # tell python helper s2a_fm status to 86Scratch UI
    _s2a_status = 0
    message_no = 0
    
    _data_status = [0, 0, 0, 0]
    _data_avg_sec = [0, 0, 0, 0]
    _local_ip = [0, 0, 0, 0]
    
    start_time = 0
    next_time = 3
    
    # the following defines are from Firmata.h

    KILL_PROCESS = 0x80  # kill python helper by UI
    REQUEST = 0xC0  # send the working status of python helper to UI
    DATA_STATUS = 0xA0 # send the status between 86Duino board and python helper
    ERROR_MESSAGE = 0xB0 # send error message to 86Duino
    LOCAL_IP = 0x90 # get local IP
    
    # This is a map that allows the look up of command handler methods using a command as the key.
    # This is populated in the run method after the python interpreter sees all of the command handler method
    # defines (python does not have forward referencing)

    # The "key" is the command, and the value contains is a list containing the  method name and the number of
    # parameter bytes that the method will require to process the message (in some cases the value is unused)
    command_dispatch = {}

    # this deque is used by the methods that assemble messages to be sent to Firmata. The deque is filled outside of
    # of the message processing loop and emptied within the loop.
    command_deque_2 = None

    # a lock to protect the data tables when they are being accessed
    data_lock = None

    # the stepper library version number.
    stepper_library_version = 0

    def __init__(self, pymata):
        """
        constructor for CommandHandler class
        @param pymata: A reference to the pymata instance.
        """

        # reference pointer to pymata
        self.pymata = pymata

        threading.Thread.__init__(self)
        self.daemon = True

        self.stop_event = threading.Event()
        
        self.start_time = time.time()

    def stop(self):
        self.stop_event.set()

    def is_stopped(self):
        return self.stop_event.is_set()

    def send_python_helper_status(self):
        command = [0xFE, 0xFE, self.REQUEST+1, self._s2a_status]
        self.send_command(command)
    
    def send_data_status(self):
        with self.pymata.data_lock:
            command = [0xFE, 0xFE, self.DATA_STATUS+1, ctypes.c_ushort(self._data_avg_sec[0]).value/256, ctypes.c_ushort(self._data_avg_sec[0]).value%256, ctypes.c_ushort(self._data_avg_sec[1]).value/256, ctypes.c_ushort(self._data_avg_sec[1]).value%256, ctypes.c_ushort(self._data_avg_sec[2]).value/256, ctypes.c_ushort(self._data_avg_sec[2]).value%256, ctypes.c_ushort(self._data_avg_sec[3]).value/256, ctypes.c_ushort(self._data_avg_sec[3]).value%256]
        self.send_command(command)
        
    def send_error_message(self):
        command = [0xFE, 0xFE, self.ERROR_MESSAGE, self.message_no]
        # print command
        self.send_command(command)
        
    def set_s2a_status(self, s2a_status):
        with self.pymata.data_lock:
            self._s2a_status = s2a_status
        
    def set_error_no(self, error_no):
        with self.pymata.data_lock:
            self.message_no = error_no
        
    def set_local_ip(self, ip_1, ip_2, ip_3, ip_4):
        with self.pymata.data_lock:
            self._local_ip[0] = ip_1
            self._local_ip[1] = ip_2
            self._local_ip[2] = ip_3
            self._local_ip[3] = ip_4
    
    def set_local_ip_str(self, str_ip):
        i = 0
        strlist = str_ip.split('.')
        for value in strlist:
            self._local_ip[i] = int(value)
            i = i + 1
        
    def get_error_no(self):
        return self.message_no
    
    def set_data_status(self, S2P, P2Duino, Duino2P, P2S):
        with self.pymata.data_lock:
            self._data_status[0] += S2P
            self._data_status[1] += P2Duino
            self._data_status[2] += Duino2P
            self._data_status[3] += P2S
        
    def data_status(self, data):
        self.send_data_status()
        
    def ui_request(self, data):
        self.send_python_helper_status()
    
    def ui_kill(self, data):
        if self._s2a_status > 5:
            self.pymata.close(True)
        else:
            self.pymata.close(False)
            
    def ui_local_ip(self, data):
        command = [0xFE, 0xFE, self.LOCAL_IP+1, self._local_ip[0], self._local_ip[1], self._local_ip[2], self._local_ip[3]]
        self.send_command(command)
        
    def send_command(self, command):
        """
        This method is used to transmit a non-sysex command.
        @param command: Command to send to firmata includes command + data formatted by caller
        @return : No return value.
        """
        send_message = ""
        for i in command:
            send_message += chr(i)

        self.pymata.ui.write(send_message)
        
    def send_command_bytes(self, command):
        send_message = ""
        for i in command:
            send_message += bytes(i)

        self.pymata.ui.write(send_message)

    def run(self):
        """
        This method starts the thread that continuously runs to receive and interpret
        messages coming from Firmata. This must be the last method in this file
        It also checks the deque for messages to be sent to Firmata.
        """
        # To add a command to the command dispatch table, append here.
        # Hash tabale : COMMAND / FUNCTION / Parameter numbars
        self.command_dispatch.update({self.KILL_PROCESS: [self.ui_kill, 2]})
        self.command_dispatch.update({self.REQUEST: [self.ui_request, 2]})
        self.command_dispatch.update({self.DATA_STATUS: [self.data_status, 2]})
        self.command_dispatch.update({self.LOCAL_IP: [self.ui_local_ip, 2]})
        
        now_clientStatus = self.pymata.ui.clientStatus()
        prev_clientStatus = now_clientStatus
        
        while not self.is_stopped():
            # if UI Client (86Scratch) disconnect from the Server, we will exit python helper.
            
            if self.pymata.ui.clientStatus() != now_clientStatus:
                prev_clientStatus = now_clientStatus
                now_clientStatus = self.pymata.ui.clientStatus()
                
            if prev_clientStatus == True and now_clientStatus == False:
                self.ui_kill(0)
                
            if (time.time() - self.start_time) > self.next_time:
                self._data_avg_sec[0] = self._data_status[0]
                self._data_avg_sec[1] = self._data_status[1]
                self._data_avg_sec[2] = self._data_status[2]
                self._data_avg_sec[3] = self._data_status[3]
                self.start_time = time.time()
                self._data_status[0] = 0
                self._data_status[1] = 0
                self._data_status[2] = 0
                self._data_status[3] = 0
                
            if len(self.pymata.command_deque_2):
                # get next byte from the deque and process it
                data = self.pymata.command_deque_2.popleft()

                # this list will be populated with the received data for the command
                command_data = []

                # process sysex commands
                if data == 0xFE:
                    while len(self.pymata.command_deque_2) == 0:
                        time.sleep(0.001)

                    data = self.pymata.command_deque_2.popleft()
                    if data == 0xFE:
                        # next char is the actual sysex command
                        # wait until we can get data from the deque
                        while len(self.pymata.command_deque_2) == 0:
                            time.sleep(0.001)

                        sysex_command = self.pymata.command_deque_2.popleft()
                        # retrieve the associated command_dispatch entry for this command
                        dispatch_entry = self.command_dispatch.get(sysex_command)

                        if type(dispatch_entry) == type(None):
                            continue
                        
                        # get a "pointer" to the method that will process this command
                        method = dispatch_entry[0]

                        # invoke the method to process the command
                        method(command_data)
                        
                        continue
            else:
                time.sleep(0.001)