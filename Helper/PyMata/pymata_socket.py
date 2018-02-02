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
import sys

import socket

# For 86Duino
class PyMataClientSocket(threading.Thread):

    # class variables
    # Connect the socket to the port where the server is listening
    server_address = None

    timeout = 1
    command_deque = None

    def __init__(self, server_address, command_deque, ui_server):
        """
        Constructor:
        @param command_deque: A reference to the deque shared with the _command_handler
        """
        # Create a TCP/IP socket
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.server_address = server_address
        self.command_deque = command_deque
        self.ui_server = ui_server
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def is_stopped(self):
         return self.stop_event.is_set()

    def open(self, verbose):
        if verbose:
            print("connect to Server: " + str(self.server_address))

        try:
            self.client.settimeout(2)
            self.client.connect(self.server_address)
            # self.client.setblocking(0)
            time.sleep(1)
            return self.client

        except Exception:
            # opened failed - will report back to caller
            raise

    def close(self):
        try:
            self.client.close()
        except OSError:
            pass

    def write(self, message):
        self.ui_server.set_data_status(0, len(message), 0, 0)
        
        try:
            if sys.version_info[0] < 3:
                self.client.settimeout(3)
                self.client.send(message)
                #print "send: "
                #for ch in data:
                    #print(hex(ord(ch)))
            else:
                self.client.send(str.encode(message))
        except Exception:
            print 'Disconnect to Server'
            raise
            
    # noinspection PyExceptClausesOrder
    def run(self):
        while not self.is_stopped():
            # we can get an OSError: [Errno9] Bad file descriptor when shutting down
            # just ignore it
            try:
                data = self.client.recv(256)
                if data:
                    #print "recv: "
                    for ch in data:
                        #print(hex(ord(ch)))
                        self.command_deque.append(ord(ch))
                    self.ui_server.set_data_status(0, 0, len(data), 0)
                else:
                    time.sleep(.1)
            except socket.error:
                time.sleep(0.001)
                pass
            except OSError:
                pass
            except IOError:
                self.stop()
        self.close()


serverSocket = None

# For UI
class PyMataServerSocket(threading.Thread):

    # class variables
    # Connect the socket to the port where the server is listening
    server_address = None

    timeout = 1
    command_deque = None
    clientConnect = False

    def __init__(self, server_address, command_deque):
        """
        Constructor:
        @param command_deque: A reference to the deque shared with the _command_handler
        """
        # Create a TCP/IP socket
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.server_address = server_address
        self.command_deque = command_deque
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def is_stopped(self):
         return self.stop_event.is_set()

    def open(self, verbose):
        if verbose:
            print("Create the UI Server: " + str(self.server_address))

        try:
            self.server.bind(self.server_address)
            self.server.listen(1)
            self.server.settimeout(10)
            # time.sleep(1)
            return self.server

        except Exception:
            # opened failed - will report back to caller
            raise

    def close(self):
        try:
            if serverSocket != None:
                serverSocket.close()
            self.server.close()
        except OSError:
            pass

    def write(self, message):
        global serverSocket
        
        if serverSocket != None:
            try:
                if sys.version_info[0] < 3: 
                    serverSocket.send(message)
                    #print "send: "
                    #for ch in data:
                        #print(hex(ord(ch)))
                else:
                    serverSocket.send(str.encode(message))
            except Exception:
                print 'Close UI Server'
                raise
    
    def clientStatus(self):
        return self.clientConnect
        
    # noinspection PyExceptClausesOrder
    def run(self):
    
        global serverSocket
        
        try:
            (csock, adr) = self.server.accept()
        except Exception:
            self.clientConnect = False
            raise
        
        # print "Client Info: ", csock, adr
        serverSocket = csock
        self.clientConnect = True
        start_time = time.time()
        
        while not self.is_stopped():
            try:
                data = csock.recv(256)
                if data:
                    #print "recv: "
                    start_time = time.time()
                    for ch in data:
                        # print(hex(ord(ch)))
                        self.command_deque.append(ord(ch))
                else:
                    time.sleep(.1)
            except socket.error:
                if (time.time() - start_time) > 5:
                    self.clientConnect = False
                time.sleep(0.001)
                pass
            except OSError:
                pass
            except IOError:
                self.stop()
        csock.close()
        self.close()
