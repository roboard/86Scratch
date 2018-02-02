#!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
Created on Wed Nov  25 13:17:15 2013

@author: Alan Yorinks
Copyright (c) 2013-14 Alan Yorinks All right reserved.
Modified by Android Lin <acen@dmp.com.tw> 2018 to support 86Duino boards.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

"""
import sys
from PyMata.pymata import PyMata
import scratch_http_server
from scratch_command_handlers import ScratchCommandHandlers
import time
import thread

#noinspection PyBroadException
def s2a_fm():

    """
    This is the "main" function of the program.
    It will instantiate PyMata for communication with an Arduino micro-controller
    and the command handlers class.
    It will the start the HTTP server to communicate with Scratch 2.0
    @return : This is the main loop and should never return
    """
    global s2a_fm_status
    
    # total number of pins on arduino board
    total_pins_discovered = 0
    # number of pins that are analog
    number_of_analog_pins_discovered = 0
    # Ethernet/WiFi autodetect
    autodetect = 0

    print 's2a_fm version 1.5   Copyright(C) 2013-14 Alan Yorinks    All Rights Reserved '

    # get the com_port from the command line or default if none given
    # if user specified the com port on the command line, use that when invoking PyMata,
    # else use '/dev/ttyACM0'
    
    projectName = 'Hello, 86Duino'
    com_port = '/dev/ttyACM0'
    ui_server_port = 8386
    
    if len(sys.argv) == 2:
        if str(sys.argv[1]) == 'autodetect':
            autodetect = 1
        else:
            com_port = str(sys.argv[1])
    
    elif len(sys.argv) == 3:
        if str(sys.argv[1]).find('com') == 0:
            com_port = str(sys.argv[1])
            ui_server_port = int(sys.argv[2])
        elif str(sys.argv[1]) == 'autodetect':
            autodetect = 1
            ui_server_port = int(sys.argv[2])
        else:
            host_ip = str(sys.argv[1])
            host_port = int(sys.argv[2])

    elif len(sys.argv) == 4:
        host_ip = str(sys.argv[1])
        host_port = int(sys.argv[2])
        ui_server_port = int(sys.argv[3])
        
    try:
        # instantiate PyMata
        if autodetect == 1:
            firmata = PyMata(host_port=2000, ui_server_port=ui_server_port, projectName=projectName, autodetect=autodetect)    
        elif str(sys.argv[1]).find('com') == 0:
            firmata = PyMata(port_id=com_port, ui_server_port=ui_server_port)  # pragma: no cover
        elif len(sys.argv) == 3 or len(sys.argv) == 4:
            firmata = PyMata(host_ip=host_ip, host_port=host_port, ui_server_port=ui_server_port)  # pragma: no cover
    except Exception:
        print 'Could not instantiate PyMata - is your Arduino plugged in?'
        return

    firmata.set_s2a_fm_status(5)
    
    # determine the total number of pins and the number of analog pins for the Arduino
    # get the arduino analog pin map
    # it will contain an entry for all the pins with non-analog set to firmata.IGNORE
    # firmata.analog_mapping_query()
    # time.sleep(.1)
    capability_map = firmata.get_analog_mapping_request_results()

    firmata.capability_query()
    print "Please wait for Total Arduino Pin Discovery to complete. This can take up to 30 additional seconds."

    # count the pins
    for pin in capability_map:
        total_pins_discovered += 1
        # non analog pins will be marked as IGNORE
        if pin != firmata.IGNORE:
            number_of_analog_pins_discovered += 1

    # instantiate the command handler
    scratch_command_handler = ScratchCommandHandlers(firmata, com_port, total_pins_discovered,
                                                     number_of_analog_pins_discovered)

    firmata.set_s2a_fm_status(6)
    
    # wait for a maximum of 30 seconds to retrieve the Arduino capability query
    start_time = time.time()

    pin_capability = firmata.get_capability_query_results()
    while not pin_capability:
        firmata.set_s2a_fm_status(7)
        if time.time() - start_time > 30:
            firmata.set_s2a_fm_status(99)
            firmata.set_error_no(4)
            print ''
            print "Could not determine pin capability - exiting."
            firmata.send_error_message()
            firmata.close(True)
            return
            # keep sending out a capability query until there is a response
        pin_capability = firmata.get_capability_query_results()
        time.sleep(.1)

    firmata.set_s2a_fm_status(8)
    
    # we've got the capability, now build a dictionary with pin as the key and a list of all the capabilities
    # for the pin as the key's value
    pin_list = []
    total_pins_discovered = 0
    for entry in pin_capability:
        # bump up pin counter each time IGNORE is found
        if entry == firmata.IGNORE:
            scratch_command_handler.pin_map[total_pins_discovered] = pin_list
            total_pins_discovered += 1
            pin_list = []
        else:
            pin_list.append(entry)

    print "Arduino Total Pin Discovery completed in %d seconds" % (int(time.time() - start_time))

    firmata.set_s2a_fm_status(9)
    
    thread.start_new_thread(firmata.send_command_check_active, (firmata,))
    
    try:
        # start the server passing it the handle to PyMata and the command handler.
        scratch_http_server.start_server(firmata, scratch_command_handler)
    except Exception:
        firmata.set_error_no(5)
        firmata.send_error_message()
        firmata.close(True)
        return

    except KeyboardInterrupt:
        # give control back to the shell that started us
        firmata.set_error_no(99)
        firmata.close(True)
        return

if __name__ == "__main__":
        s2a_fm()