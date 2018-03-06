# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 14:45:49 2013

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

import logging
from BaseHTTPServer import BaseHTTPRequestHandler
from BaseHTTPServer import HTTPServer
from string import split
import time
import thread

server = None
scratch_connected_time = 0
waiting_for_first_scratch_poll = True
MAX_TIMES = 0
PRE_TIMES = 0
NOW_TIMES = 0

class GetHandler(BaseHTTPRequestHandler):
    """
    This class contains the HTTP server that Scratch2 communicates with
    Scratch sends HTTP GET requests and this class processes the requests.

    HTTP GET requests are accepted and the appropriate command handler is
    called to process the command.
    """
    
    firmata = None
    
    # instance handle for the scratch command handler
    scratch_command_handler = None
    
    # this is a 'classmethod' because we need to set data before starting
    # the HTTP server.
    #noinspection PyMethodParameters
    @classmethod
    def set_items(self, firmata, scratch_command_handler):
        """
        This method stores the input parameters for later use.
        It is a class method, because these values need to established
        prior to instantiating the class
        """
        # instance variable for PyMata
        #noinspection PyAttributeOutsideInit
        self.firmata = firmata

        # instance variable for scratch command handler
        #noinspection PyAttributeOutsideInit
        self.command_handler = scratch_command_handler

    #noinspection PyPep8Naming
    def do_GET(self):
        """
        Scratch2 only sends HTTP GET commands. This method processes them.
        It differentiates between a "normal" command request and a request
        to send policy information to keep Flash happy on Scratch.
        (This may change when Scratch is converted to HTML 5
        """
        
        global scratch_connected_time
        global waiting_for_first_scratch_poll
        global PRE_TIMES
        global NOW_TIMES
        global MAX_TIMES
    
        # print self.path
        # print time.time()
        # skip over the / in the command
        cmd = self.path[1:]
        self.firmata.set_data_status(len(cmd), 0, 0, 0)
        scratch_connected_time = time.time()
        
        # create a list containing the command and all of its parameters
        cmd_list = split(cmd, '/')

        NOW_TIMES = int(round(time.time() * 1000))
        if NOW_TIMES - PRE_TIMES > MAX_TIMES:
            MAX_TIMES = NOW_TIMES - PRE_TIMES
            print MAX_TIMES
        PRE_TIMES = NOW_TIMES
        
        # get the command handler method for the command and call the handler
        # cmd_list[0] contains the command. look up the command method
        try:
            s = self.command_handler.do_command(cmd_list)
            if cmd_list[0] == 'poll':
                waiting_for_first_scratch_poll = False
        except SystemExit:
            return

        # if pin was not enabled for reporter block, a "NoneType" can be returned by the command_handler
        if (s is None) or (len(s) == 0):
            err_statement = ("do_GET: Do you have all active pins enabled? " + str(cmd_list))
            logging.info(err_statement)
            print err_statement
            return
        else:
            self.send_resp(s)

    # we can't use the standard send_response since we don't conform to its
    # standards, so we craft our own response handler here
    def send_resp(self, response):
        """
        This method sends Scratch an HTTP response to an HTTP GET command.
        """

        crlf = "\r\n"
        # http_response = str(response + crlf)
        http_response = "HTTP/1.1 200 OK" + crlf
        http_response += "Content-Type: text/html; charset=ISO-8859-1" + crlf
        http_response += "Content-Length" + str(len(response)) + crlf
        http_response += "Access-Control-Allow-Origin: *" + crlf
        http_response += crlf
        #add the response to the nonsense above
        if response != 'okay':
            http_response += str(response + crlf)
        # send it out the door to Scratch
        self.wfile.write(http_response)
        self.firmata.set_data_status(0, 0, 0, len(http_response))

def updateTime(firmata):
    
    global scratch_connected_time
    global waiting_for_first_scratch_poll
    
    unstable_linking = 0
    
    scratch_connected_starttime = time.time()
    
    while True:
        if waiting_for_first_scratch_poll == False:
            if scratch_connected_time != 0:
                scratch_connected_time = 0
                scratch_connected_starttime = time.time()
                firmata.set_s2a_fm_status(10)
                if unstable_linking == 1:
                    unstable_linking = 0
                    firmata.set_error_no(100)
                    firmata.send_error_message()
            else:
                if time.time() - scratch_connected_starttime > 1:
                    unstable_linking = 1
                    firmata.set_s2a_fm_status(9)
                    firmata.set_error_no(6)
                    firmata.send_error_message()
                    scratch_connected_starttime = time.time()
        else:
            if time.time() - scratch_connected_starttime > 25:
                unstable_linking = 1
                firmata.set_s2a_fm_status(9)
                firmata.set_error_no(6)
                firmata.send_error_message()
                time.sleep(1)
        time.sleep(0.001)

def close_server():
    
    global server
    
    if server != None:    
        def kill_me_please(s):
            s.shutdown()
        thread.start_new_thread(kill_me_please, (server,))
    return

def start_server(firmata, command_handler):
    """
       This function populates class variables with essential data and
       instantiates the HTTP Server
    """
    
    global server
    global PRE_TIMES
    global NOW_TIMES
    
    GetHandler.set_items(firmata, command_handler)
    try:
        server = HTTPServer(('localhost', 50209), GetHandler)
        print 'Starting HTTP Server!'
        print 'Use <Ctrl-C> to exit the extension\n'
        print 'Please start Scratch or Snap!'
        thread.start_new_thread(updateTime, (firmata,))
    except Exception:
        logging.debug('Exception in scratch_http_server.py: HTTP Socket may already be in use - restart Scratch')
        print 'HTTP Socket may already be in use - restart Scratch'
        raise
    
    PRE_TIMES = NOW_TIMES = int(round(time.time() * 1000))
    
    try:
        #start the server
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info('scratch_http_server.py: keyboard interrupt exception')
        print "Goodbye !"
        raise KeyboardInterrupt
    except Exception:
        logging.debug('scratch_http_server.py: Exception %s' % str(Exception))
        raise