# -*- coding: utf-8 -*-

"""
Created on Wed Nov  25 13:17:15 2013

@author: Alan Yorinks
Copyright (c) 2013-14 Alan Yorinks All right reserved.

@co-author: Sjoerd Dirk Meijer, fromScratchEd.nl (language support)

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
import datetime
import ConfigParser

class ScratchCommandHandlers:
    """
    This class processes any command received from Scratch 2.0

    If commands need to be added in the future, a command handler method is
    added to this file and the command_dict at the end of this file is
    updated to contain the method. Command names must be the same in the json .s2e Scratch
    descriptor file.
    """
    # get translation strings from xlate.cfg
    config = ConfigParser.ConfigParser()
    config.read('xlate.cfg')

    ln_languages = config.get('translation_lists', 'ln_languages').split(',')
    ln_ENABLE = config.get('translation_lists', 'ln_ENABLE').split(',')
    ln_DISABLE = config.get('translation_lists', 'ln_DISABLE').split(',')
    ln_INPUT = config.get('translation_lists', 'ln_INPUT').split(',')
    ln_OUTPUT = config.get('translation_lists', 'ln_OUTPUT').split(',')
    ln_PWM = config.get('translation_lists', 'ln_PWM').split(',')
    ln_SERVO = config.get('translation_lists', 'ln_SERVO').split(',')
    ln_TONE = config.get('translation_lists', 'ln_TONE').split(',')
    ln_SONAR = config.get('translation_lists', 'ln_SONAR').split(',')
    ln_OFF = config.get('translation_lists', 'ln_OFF').split(',')
    ln_ON = config.get('translation_lists', 'ln_ON').split(',')
    ln_DELAY = config.get('translation_lists', 'ln_DELAY').split(',')
    ln_VELOCITY = config.get('translation_lists', 'ln_VELOCITY').split(',')
    
    # pin counts for the board
    total_pins_discovered = 0
    number_of_analog_pins_discovered = 0

    # lists to keep track of which pins need to be included in the poll responses
    digital_poll_list = []
    analog_poll_list = []
    encoder_poll_list = []

    # detected pin capability map
    pin_map = {}

    # instance variable for PyMata
    firmata = None

    # debug state - 0 == off and 1 == on
    debug = 0
    
    imu_busy = 0
    imu_busy_ID = 0
    imu_active = 0
    
    one_servo_busy = 0
    one_servo_is_moving = 0
    one_servo_busy_ID = 0

    # base report string to be modified in response to a poll command
    # PIN and VALUE will be replaced with pin number and the current value for the pin
    digital_reporter_base = 'digital_read/PIN VALUE'
    analog_reporter_base = 'analog_read/PIN VALUE'
    encoder_reporter_base = 'read_encoder_count/MODULE VALUE'
    imu_response_base = 'read_imu/TYPE VALUE'
    imu_init_response_base = '_busy ID'
    one_servo_response_base = '_busy ID'

    # convenience definition for cr + lf
    end_of_line = "\r\n"

    # indices into the command list sent to each command method
    CMD_COMMAND = 0  # this is the actual command
    CMD_ENABLE_DISABLE = 1  # enable or disable pin
    CMD_PIN = 1  # pin number for all commands except the Enable/Disable
    CMD_PIN_ENABLE_DISABLE = 2
    CMD_DIGITAL_MODE = 3  # pin mode
    CMD_VALUE = 2  # value pin to be set to
    CMD_TONE_FREQ = 2  # frequency for tone command
    CMD_TONE_DURATION = 3  # tone duration
    CMD_SERVO_DEGREES = 2  # number of degrees for servo position
    CMD_DEBUG = 1  # debugger on or off

    # noinspection PyPep8Naming
    def check_CMD_ENABLE_DISABLE(self, command):
        if command in self.ln_ENABLE:
            return 'Enable'
        if command in self.ln_DISABLE:
            return 'Disable'

    # noinspection PyPep8Naming
    def check_CMD_DIGITAL_MODE(self, command):
        if command in self.ln_INPUT:
            return 'Input'
        if command in self.ln_OUTPUT:
            return 'Output'
        if command in self.ln_PWM:
            return 'PWM'
        if command in self.ln_SERVO:
            return 'Servo'
        if command in self.ln_TONE:
            return 'Tone'
        if command in self.ln_SONAR:
            return 'SONAR'
    
    def check_CMD_SERVO_MODE(self, command):
        if command in self.ln_DELAY:
            return 'Delay'
        if command in self.ln_VELOCITY:
            return 'Velocity'

    # noinspection PyPep8Naming
    def check_DEBUG(self, command):
        if command in self.ln_OFF:
            return 'Off'
        if command in self.ln_ON:
            return 'On'

    def __init__(self, firmata, com_port, total_pins_discovered, number_of_analog_pins_discovered):
        """
        The class constructor creates the pin lists for the pins that will report
        data back to Scratch as a result of a poll request.
        @param total_pins_discovered:
        @param number_of_analog_pins_discovered:
        """
        self.firmata = firmata  
        self.com_port = com_port
        self.total_pins_discovered = total_pins_discovered
        self.number_of_analog_pins_discovered = number_of_analog_pins_discovered
        self.first_poll_received = False
        self.debug = 0

        # Create a pin list for poll data based on the total number of pins( digital table)
        # and a pin list for the number of analog pins.
        # Pins will be marked using Firmata Pin Types
        for x in range(self.total_pins_discovered):
            self.digital_poll_list.append(self.firmata.IGNORE)

        for x in range(self.number_of_analog_pins_discovered):
            self.analog_poll_list.append(self.firmata.IGNORE)

        for x in range(4):
            self.encoder_poll_list.append(self.firmata.IGNORE)
    def do_command(self, command):
        """
        This method looks up the command that resides in element zero of the command list
        within the command dictionary and executes the method for the command.
        Each command returns string that will be eventually be sent to Scratch
        @param command: This is a list containing the Scratch command and all its parameters
        @return: String to be returned to Scratch via HTTP
        """
        method = self.command_dict.get(command[0])
        if command[0] != "poll":
            # turn on debug logging if requested
            if self.debug == 'On':
                debug_string = "DEBUG: "
                debug_string += str(datetime.datetime.now())
                debug_string += ": "
                for data in command:
                    debug_string += "".join(map(str, data))
                    debug_string += ' '
                logging.debug(debug_string)
                print debug_string
        #print command
        return method(self, command)

    #noinspection PyUnusedLocal
    def poll(self, command):
        # look for first poll and when received let the world know we are ready!
        """
        This method scans the data tables and assembles data for all reporter
        blocks and returns the data to the caller.
        @param command: This is a list containing the Scratch command and all its parameters It is unsused
        @return: 'okay'
        """
        import s2a_fm
        
        if not self.first_poll_received:
            logging.info('Scratch detected! Ready to rock and roll...')
            print 'Scratch detected! Ready to rock and roll...'
            self.first_poll_received = True
            self.firmata.set_s2a_fm_status(10)

        # assemble all output pin reports

        # first get the current digital and analog pin values from firmata
        digital_response_table = self.firmata.get_digital_response_table()
        analog_response_table = self.firmata.get_analog_response_table()
        encoder_response_table = self.firmata.get_encoder_response_table()
        imu_response_table = self.firmata.get_imu_response_table()
        imu_init_response_table = self.firmata.get_imu_init_response_table()
        one_servo_response_table = self.firmata.get_one_servo_response_table()
        
        # for each pin in the poll list that is set as an INPUT,
        # retrieve the pins value from the response table and build the response
        # string

        # digital first
        responses = ''
        for pin in range(self.total_pins_discovered):
            if self.digital_poll_list[pin] == self.firmata.INPUT:
                pin_number = str(pin)
                pin_entry = digital_response_table[pin]
                value = str(pin_entry[1])
                report_entry = self.digital_reporter_base
                report_entry = report_entry.replace("PIN", pin_number)
                report_entry = report_entry.replace("VALUE", value)
                responses += report_entry
                responses += self.end_of_line

        # now check for any analog reports to be added
        for pin in range(self.number_of_analog_pins_discovered):
            if self.analog_poll_list[pin] != self.firmata.IGNORE:
                pin_number = str(pin)
                pin_entry = analog_response_table[pin]
                value = str(pin_entry[1])
                report_entry = self.analog_reporter_base
                report_entry = report_entry.replace("PIN", pin_number)
                report_entry = report_entry.replace("VALUE", value)
                responses += report_entry
                responses += self.end_of_line
        
        for module in range(4):
            if self.encoder_poll_list[module] == self.firmata.ENCODER:
                module_number = str(module)
                value = str(encoder_response_table[module][1])
                report_entry = self.encoder_reporter_base
                report_entry = report_entry.replace("MODULE", module_number)
                report_entry = report_entry.replace("VALUE", value)
                responses += report_entry
                responses += self.end_of_line

        # for 86Duino Servo86
        if self.one_servo_is_moving == 1:
            if one_servo_response_table[0] != 99:
                id = str(self.one_servo_busy_ID)
                report_entry = self.one_servo_response_base
                report_entry = report_entry.replace("ID", id)
                responses += report_entry
                responses += self.end_of_line
            else:
                self.one_servo_busy = 0
                one_servo_response_table[0] = 0
                self.one_servo_is_moving = 0

        # for 86Duino FreeIMU1
        if self.imu_active == 1:
            type = 'pitch'
            val = str(imu_response_table[0])
            report_entry_0 = self.imu_response_base
            report_entry_0 = report_entry_0.replace("TYPE", type)
            report_entry_0 = report_entry_0.replace("VALUE", val)
            responses += report_entry_0
            responses += self.end_of_line
            
            type = 'roll'
            val = str(imu_response_table[1])
            report_entry_1 = self.imu_response_base
            report_entry_1 = report_entry_1.replace("TYPE", type)
            report_entry_1 = report_entry_1.replace("VALUE", val)
            responses += report_entry_1
            responses += self.end_of_line
            
        
        if self.imu_busy == 1:
            if imu_init_response_table[1] != 99:
                id = str(self.imu_busy_ID)
                report_entry = self.imu_init_response_base
                report_entry = report_entry.replace("ID", id)
                responses += report_entry
                responses += self.end_of_line
            else:
                self.imu_busy = 0
                imu_init_response_table[1] = 0
                self.imu_active = 1
        
        if responses == '':
            responses = 'okay'

        return responses

    #noinspection PyUnusedLocal
    def send_cross_domain_policy(self, command):
        """
        This method returns cross domain policy back to Scratch upon request.
        It keeps Flash happy.
        @param command: Command and all possible parameters in list form
        @return: policy string
        """
        policy = "<cross-domain-policy>\n"
        policy += "  <allow-access-from domain=\"*\" to-ports=\""
        policy += str(self.com_port)
        policy += "\"/>\n"
        policy += "</cross-domain-policy>\n\0"
        return policy

    #noinspection PyUnusedLocal
    def reset_arduino(self, command):
        """
        This method will send the reset command to the arduino and the poll tables
        @param command: Command and all possible parameters in list form
        @return: 'okay'
        """
        
        # reset the tables
        for x in range(self.total_pins_discovered):
            self.digital_poll_list[x] = self.firmata.IGNORE

        for x in range(self.number_of_analog_pins_discovered):
            self.analog_poll_list[x] = self.firmata.IGNORE
            
        for x in range(4):
            self.encoder_poll_list[x] = self.firmata.IGNORE
        
        self.firmata.reset()
        self.debug = 0
        self.imu_active = 0
        
        return 'okay'

    def digital_pin_mode(self, command):
        """
        This method will set the poll list table appropriately and
        send the arduino a set_pin  configuration message.
        @param command: Command and all possible parameters in list form
        @return: 'okay'
        """
        if not command[self.CMD_PIN_ENABLE_DISABLE].isdigit():
            logging.debug('digital_pin_mode: The pin number must be set to a numerical value')
            print 'digital_pin_mode: The pin number must be set to a numerical value'
            return 'okay'

        pin = int(command[self.CMD_PIN_ENABLE_DISABLE])

        # test for a valid pin number
        if pin >= self.total_pins_discovered:
            logging.debug('digital_pin_mode: pin %d exceeds number of pins on board' % pin)
            print 'digital_pin_mode: pin %d exceeds number of pins on board' % pin
            return 'okay'
        # ok pin is range, but make
        else:
            # now test for enable or disable

            if self.check_CMD_ENABLE_DISABLE(command[self.CMD_ENABLE_DISABLE]) == 'Enable':
                # choices will be input or some output mode
                if self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'Input':
                    if self.valid_digital_pin_mode_type(pin, self.firmata.INPUT):
                        # set the digital poll list for the pin
                        self.digital_poll_list[pin] = self.firmata.INPUT
                        # send the set request to the Arduino
                        self.firmata.set_pin_mode(pin, self.firmata.INPUT, self.firmata.DIGITAL)
                    else:
                        logging.debug('digital_pin_mode: Pin %d does not support INPUT mode' % pin)
                        print 'digital_pin_mode: Pin %d does not support INPUT mode ' % pin
                        return 'okay'
                elif self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'SONAR':
                        # any digital input pin can be used for SONAR
                        if self.valid_digital_pin_mode_type(pin, self.firmata.INPUT):
                            self.digital_poll_list[pin] = self.firmata.INPUT
                            self.firmata.sonar_config(pin, pin)
                        else:
                            logging.debug('digital_pin_mode: Pin %d does not support SONAR mode' % pin)
                            print 'digital_pin_mode: Pin %d does not support SONAR mode' % pin
                            return 'okay'
                else:
                    # an output mode, so just clear the poll bit
                    if self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'Output':
                        if self.valid_digital_pin_mode_type(pin, self.firmata.OUTPUT):
                            self.digital_poll_list[pin] = self.firmata.OUTPUT
                            self.firmata.set_pin_mode(pin, self.firmata.OUTPUT, self.firmata.DIGITAL)
                        else:
                            logging.debug('digital_pin_mode: Pin %d does not support OUTPUT mode' % pin)
                            print 'digital_pin_mode: Pin %d does not support OUTPUT mode' % pin
                            return 'okay'
                    elif self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'PWM':
                        if self.valid_digital_pin_mode_type(pin, self.firmata.PWM):
                            self.digital_poll_list[pin] = self.firmata.PWM
                            self.firmata.set_pin_mode(pin, self.firmata.PWM, self.firmata.DIGITAL)
                        else:
                            logging.debug('digital_pin_mode: Pin %d does not support PWM mode' % pin)
                            print 'digital_pin_mode: Pin %d does not support PWM mode' % pin
                            return 'okay'
                    elif self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'Tone':
                        # Tone can be on any pin so we look for OUTPUT
                        if self.valid_digital_pin_mode_type(pin, self.firmata.OUTPUT):
                            self.digital_poll_list[pin] = self.digital_poll_list[pin] = self.firmata.TONE_TONE
                            self.firmata.set_pin_mode(pin, self.firmata.OUTPUT, self.firmata.DIGITAL)
                        else:
                            logging.debug('digital_pin_mode: Pin %d does not support TONE mode' % pin)
                            print 'digital_pin_mode: Pin %d does not support TONE mode' % pin
                            return 'okay'
                    elif self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'Servo':
                        if self.valid_digital_pin_mode_type(pin, self.firmata.SERVO):
                            self.digital_poll_list[pin] = self.firmata.SERVO
                            self.firmata.servo_config(pin)
                        else:
                            logging.debug('digital_pin_mode: Pin %d does not support SERVO mode' % pin)
                            print 'digital_pin_mode: Pin %d does not support SERVO mode' % pin
                            return 'okay'
                    else:
                        logging.debug('digital_pin_mode: Unknown output mode %s' % command[self.CMD_DIGITAL_MODE])
                        print 'digital_pin_mode: Unknown output mode %s' % command[self.CMD_DIGITAL_MODE]
                        return 'okay'
            if self.check_CMD_ENABLE_DISABLE(command[self.CMD_ENABLE_DISABLE]) == 'Disable':
                # disable pin of any type by setting it to IGNORE in the table
                self.digital_poll_list[pin] = self.firmata.IGNORE
                # this only applies to Input pins. For all other pins we leave the poll list as is
                if self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'Input':
                    # send a disable reporting message
                    self.firmata.disable_digital_reporting(pin)
                if self.check_CMD_DIGITAL_MODE(command[self.CMD_DIGITAL_MODE]) == 'SONAR':
                    # send a disable reporting message
                    self.firmata.disable_digital_reporting(pin)
            # normal http return for commands
            return 'okay'

    def digital_pin_mode_ja(self, command_ja):
        """
        This method will call digital_pin_mode after reordering
        command arguments (from Japanese order to English order).
        @param command: Command and all possible parameters in list form
        @return: 'okay'
        """
        command = ['digital_pin_mode'] + [command_ja[i] for i in [3, 1, 2]]
        return self.digital_pin_mode(command)

    def valid_digital_pin_mode_type(self, pin, pin_mode):
        """
        This is a utility method to determine if the pin supports the pin mode
        @param pin: Pin number
        @param pin_mode: Pin Mode
        @return: True if the mode is supported or False if it not supported.
        """
        pin_modes = self.pin_map[pin]
        if pin_mode in pin_modes:
            return True
        else:
            return False

    def analog_pin_mode(self, command):
        """
        This method will set the poll list table appropriately and
        send the arduino the correct configuration message.
        @param command: Command and all possible parameters in list form
        @return: 'okay'
        """

        if not command[self.CMD_PIN_ENABLE_DISABLE].isdigit():
            logging.debug('analog_pin_mode: The pin number must be set to a numerical value')
            print 'analog_pin_mode: The pin number must be set to a numerical value'
            return 'okay'

        pin = int(command[self.CMD_PIN_ENABLE_DISABLE])
        # Normally analog pins act as inputs only, but the DUE allow analog ins
        # test for a valid pin number
        if pin >= self.number_of_analog_pins_discovered:
            print 'analog_pin_mode: pin %d exceeds number of analog pins on board' % pin
            logging.debug('analog_pin_mode: pin %d exceeds number of analog pins on board' % pin)
            return 'okay'
        else:
            # now test for enable or disable
            if self.check_CMD_ENABLE_DISABLE(command[self.CMD_ENABLE_DISABLE]) == 'Enable':
                # enable the analog pin
                self.analog_poll_list[pin] = self.firmata.INPUT
                self.firmata.set_pin_mode(pin, self.firmata.INPUT, self.firmata.ANALOG)
            else:
                # Set analog poll list entry for the pin to IGNORE.
                # Disable reporting
                self.analog_poll_list[pin] = self.firmata.IGNORE
                self.firmata.disable_analog_reporting(pin)  

        return 'okay'

    def analog_pin_mode_ja(self, command_ja):
        """
        This method will call analog_pin_mode after reordering
        command arguments (from Japanese order to English order).
        @param command: Command and all possible parameters in list form
        @return: 'okay'
        """
        command = ['analog_pin_mode'] + [command_ja[i] for i in [2, 1]]
        return self.analog_pin_mode(command)

    def digital_write(self, command):
        """
        This method outputs a 0 or a 1 to the designated digital pin that has been previously
        been configured as an output.

        If the pin is configured as an INPUT, writing a HIGH value with digitalWrite()
        will enable an internal 20K pullup resistor (see the tutorial on digital pins on arduino site).
        Writing LOW will disable the pullup. The pullup resistor is enough to light an LED dimly,
        so if LEDs appear to work, but very dimly, this is a likely cause.
        The remedy is to set the pin to an output.

        @param command: Command and all possible parameters in list form
        @return: okay
        """
        # test pin as a digital output pin in poll list table

        if not command[self.CMD_PIN].isdigit():
            logging.debug('digital_write: The pin number must be set to a numerical value')
            print 'digital_write: The pin number must be set to a numerical value'
            return 'okay'

        pin = int(command[self.CMD_PIN])

        if self.digital_poll_list[pin] == self.firmata.OUTPUT:
            self.firmata.digital_write(pin, int(command[self.CMD_VALUE]))
            return 'okay'
        # for pullup - see description above
        elif self.digital_poll_list[pin] == self.firmata.INPUT:
            self.firmata.digital_write(pin, int(command[self.CMD_VALUE]))
            return 'okay'
        else:
            print 'digital write: Pin %d must be enabled before writing to it.' % pin
            logging.debug('digital write: Pin %d must be enabled before writing to it.' % pin)
            return 'okay'

    def analog_write(self, command):
        """
        This method write the value (0-255) to the digital pin that has been
        previously been specified as a PWM pin. NOTE: Pin number is the digital
        pin number and not an analog pin number.
        @param command: Command and all possible parameters in list form
        @return: okay or _problem
        """
        if command[self.CMD_VALUE] == 'VAL':
            logging.debug('analog_write: The value field must be set to a numerical value')
            print 'analog_write: The value field must be set to a numerical value'
            return 'okay'

        if not command[self.CMD_PIN].isdigit():
            logging.debug('analog_write: The pin number must be set to a numerical value')
            print 'analog_write: The pin number must be set to a numerical value'
            return 'okay'

        pin = int(command[self.CMD_PIN])

        if self.digital_poll_list[pin] == self.firmata.PWM:
            # check to make sure that the value is in the range of 0-255
            if 0 <= int(command[self.CMD_VALUE]) <= 255:
                self.firmata.analog_write(pin, int(command[self.CMD_VALUE]))
                return 'okay'
            else:
                print 'analog_write data value %d is out of range. It should be between 0-255' % \
                      int(command[self.CMD_VALUE])
                logging.debug('analog_write data value %d is out of range. It should be between 0-255' %
                              int(command[self.CMD_VALUE]))
                return '_problem analog_write data value %d is out of range. It should be between 0-255' % \
                       int(command[self.CMD_VALUE])
        else:
            print'analog_write: Pin %d must be enabled before writing to it.' % pin
            logging.debug('analog_write: Pin %d must be enabled before writing to it.' % pin)
            return '_problem Pin must be enabled before writing to it.'

    def play_tone(self, command):
        # check to make sure pin was configured for tone
        """
        This method will play a tone for the specified pin in command
        @param command: Command and all possible parameters in list form
        @return: okay or _problem
        """

        if not command[self.CMD_PIN].isdigit():
            logging.debug('play_tome: The pin number must be set to a numerical value')
            print 'play_tone: The pin number must be set to a numerical value'
            return 'okay'

        pin = int(command[self.CMD_PIN])

        if self.digital_poll_list[pin] == self.firmata.TONE_TONE:
            #noinspection PyUnusedLocal
            value = command[1]
            self.firmata.play_tone(pin, self.firmata.TONE_TONE, int(command[self.CMD_TONE_FREQ]),
                                   int(command[self.CMD_TONE_DURATION]))
            return 'okay'
        else:
            print 'play_tone: Pin %d was not enabled as TONE.' % pin
            logging.debug('play_tone: Pin %d was not enabled as TONE.' % pin)
            return 'okay'

    def play_tone_educake(self, command):
        # check to make sure pin was configured for tone
        """
        This method will play a tone for the specified pin in command
        @param command: Command and all possible parameters in list form
        @return: okay or _problem
        """

        sound_level = command[1]
        freq = 0
        
        if sound_level == 'Do1':
            freq = 33
        elif sound_level == 'Re_b1':
            freq = 35
        elif sound_level == 'Re1':
            freq = 37
        elif sound_level == 'Mi_b1':
            freq = 39
        elif sound_level == 'Mi1':
            freq = 41
        elif sound_level == 'Fa1':
            freq = 44
        elif sound_level == 'So_b1':
            freq = 46
        elif sound_level == 'So1':
            freq = 49
        elif sound_level == 'La_b1':
            freq = 52
        elif sound_level == 'La1':
            freq = 55
        elif sound_level == 'Si_b1':
            freq = 58
        elif sound_level == 'Si1':
            freq = 62
        elif sound_level == 'Do2':
            freq = 65
        elif sound_level == 'Re_b2':
            freq = 69
        elif sound_level == 'Re2':
            freq = 73
        elif sound_level == 'Mi_b2':
            freq = 78
        elif sound_level == 'Mi2':
            freq = 82
        elif sound_level == 'Fa2':
            freq = 87
        elif sound_level == 'So_b2':
            freq = 92
        elif sound_level == 'So2':
            freq = 98
        elif sound_level == 'La_b2':
            freq = 104
        elif sound_level == 'La2':
            freq = 110
        elif sound_level == 'Si_b2':
            freq = 117
        elif sound_level == 'Si2':
            freq = 123
        elif sound_level == 'Do3':
            freq = 131
        elif sound_level == 'Re_b3':
            freq = 139
        elif sound_level == 'Re3':
            freq = 147
        elif sound_level == 'Mi_b3':
            freq = 156
        elif sound_level == 'Mi3':
            freq = 165
        elif sound_level == 'Fa3':
            freq = 175
        elif sound_level == 'So_b3':
            freq = 185
        elif sound_level == 'So3':
            freq = 196
        elif sound_level == 'La_b3':
            freq = 208
        elif sound_level == 'La3':
            freq = 220
        elif sound_level == 'Si_b3':
            freq = 233
        elif sound_level == 'Si3':
            freq = 247
        elif sound_level == 'Do4':
            freq = 262
        elif sound_level == 'Re_b4':
            freq = 277
        elif sound_level == 'Re4':
            freq = 294
        elif sound_level == 'Mi_b4':
            freq = 311
        elif sound_level == 'Mi4':
            freq = 330
        elif sound_level == 'Fa4':
            freq = 349
        elif sound_level == 'So_b4':
            freq = 370
        elif sound_level == 'So4':
            freq = 392
        elif sound_level == 'La_b4':
            freq = 415
        elif sound_level == 'La4':
            freq = 440
        elif sound_level == 'Si_b4':
            freq = 466
        elif sound_level == 'Si4':
            freq = 494
        elif sound_level == 'Do5':
            freq = 523
        elif sound_level == 'Re_b5':
            freq = 554
        elif sound_level == 'Re5':
            freq = 587
        elif sound_level == 'Mi_b5':
            freq = 622
        elif sound_level == 'Mi5':
            freq = 659
        elif sound_level == 'Fa5':
            freq = 698
        elif sound_level == 'So_b5':
            freq = 740
        elif sound_level == 'So5':
            freq = 784
        elif sound_level == 'La_b5':
            freq = 830
        elif sound_level == 'La5':
            freq = 880
        elif sound_level == 'Si_b5':
            freq = 932
        elif sound_level == 'Si5':
            freq = 988
        elif sound_level == 'Do6':
            freq = 1047
        elif sound_level == 'Re_b6':
            freq = 1109
        elif sound_level == 'Re6':
            freq = 1175
        elif sound_level == 'Mi_b6':
            freq = 1245
        elif sound_level == 'Mi6':
            freq = 1319
        elif sound_level == 'Fa6':
            freq = 1397
        elif sound_level == 'So_b6':
            freq = 1480
        elif sound_level == 'So6':
            freq = 1586
        elif sound_level == 'La_b6':
            freq = 1661
        elif sound_level == 'La6':
            freq = 1760
        elif sound_level == 'Si_b6':
            freq = 1865
        elif sound_level == 'Si6':
            freq = 1976
        elif sound_level == 'Do7':
            freq = 2093
        elif sound_level == 'Re_b7':
            freq = 2218
        elif sound_level == 'Re7':
            freq = 2349
        elif sound_level == 'Mi_b7':
            freq = 2489
        elif sound_level == 'Mi7':
            freq = 2637
        elif sound_level == 'Fa7':
            freq = 2794
        elif sound_level == 'So_b7':
            freq = 2960
        elif sound_level == 'So7':
            freq = 3136
        elif sound_level == 'La_b7':
            freq = 3322
        elif sound_level == 'La7':
            freq = 3520
        elif sound_level == 'Si_b7':
            freq = 3729
        elif sound_level == 'Si7':
            freq = 3951
        elif sound_level == 'Do8':
            freq = 4186
        elif sound_level == 'Re_b8':
            freq = 4435
        elif sound_level == 'Re8':
            freq = 4699
        elif sound_level == 'Mi_b8':
            freq = 4978
        elif sound_level == 'Mi8':
            freq = 5274
        elif sound_level == 'Fa8':
            freq = 5588
        elif sound_level == 'So_b8':
            freq = 5920
        elif sound_level == 'So8':
            freq = 6272
        elif sound_level == 'La_b8':
            freq = 6645
        elif sound_level == 'La8':
            freq = 7040
        elif sound_level == 'Si_b8':
            freq = 7459
        elif sound_level == 'Si8':
            freq = 7902
        elif sound_level == 'Do9':
            freq = 8372
        elif sound_level == 'Re_b9':
            freq = 8870
        elif sound_level == 'Re9':
            freq = 9397
        elif sound_level == 'Mi_b9':
            freq = 9956
        elif sound_level == 'Mi9':
            freq = 10548
        elif sound_level == 'Fa9':
            freq = 11175
        elif sound_level == 'So_b9':
            freq = 11840
        elif sound_level == 'So9':
            freq = 12544
        elif sound_level == 'La_b9':
            freq = 13290
        elif sound_level == 'La9':
            freq = 14080
        elif sound_level == 'Si_b9':
            freq = 14917
        elif sound_level == 'Si9':
            freq = 15804
        
        self.firmata.play_tone_educake(250, self.firmata.TONE_TONE, int(freq), int(command[2]))
        return 'okay'
    def tone_off(self, command):
       # check to make sure pin was configured for tone
        """
        This method will force tone to be off.
        @param command: Command and all possible parameters in list form
        @return: okay
        """

        if not command[self.CMD_PIN].isdigit():
            logging.debug('tone_off: The pin number must be set to a numerical value')
            print 'tone_off: The pin number must be set to a numerical value'
            return 'okay'

        pin = int(command[self.CMD_PIN])

        if self.digital_poll_list[pin] == self.firmata.TONE_TONE:
            #noinspection PyUnusedLocal
            value = command[1]
            self.firmata.play_tone(pin, self.firmata.TONE_NO_TONE, 0, 0)  
            return 'okay'
        else:
            print 'tone_off: Pin %d was not enabled as TONE.' % pin
            logging.debug('tone_off: Pin %d was not enabled as TONE.' % pin)
            return 'okay'

    def debug_control(self, command):
        """
        This method controls command block debug logging
        @param command: Either On or Off
        @return: okay
        """
        self.debug = self.check_DEBUG(command[self.CMD_DEBUG])
        return 'okay'

    def set_servo_position(self, command):
        # check to make sure pin was configured for servo
        """
        This method will command the servo position if the digital pin was
        previously configured for Servo operation.
        A maximum of 180 degrees is allowed
        @param command: Command and all possible parameters in list form
        @return: okay
        """

        if not command[self.CMD_PIN].isdigit():
            logging.debug('servo_position: The pin number must be set to a numerical value')
            print 'servo_position: The pin number must be set to a numerical value'
            return 'okay'

        pin = int(command[self.CMD_PIN])

        if self.digital_poll_list[pin] == self.firmata.SERVO:
            if 0 <= int(command[self.CMD_SERVO_DEGREES]) <= 180:
                self.firmata.analog_write(pin,  int(command[self.CMD_SERVO_DEGREES]))
                return 'okay'
            else:
                print "set_servo_position: Request of %d degrees. Servo range is 0 to 180 degrees" % int(command[1])
                # noinspection PyPep8
                logging.debug("set_servo_position: Request of %d degrees. Servo range is 0 to 180 degrees"
                        % int(command[1]))
                return 'okay'
        else:
            print 'set_servo_position: Pin %d was not enabled for SERVO operations.' % pin
            logging.debug('set_servo_position: Pin %d was not enabled for SERVO operations.' % pin)
            return '_problem Pin was not enabled for SERVO operations.'

    def digital_read(self, command):
        """
        This method retrieves digital input information for Snap!
        @param command: Command and all possible parameters in list form
        @return: Current value of digital input pin
        """
        digital_response_table = self.firmata.get_digital_response_table()
        if self.digital_poll_list[int(command[self.CMD_PIN])] == self.firmata.INPUT:
            pin_number = command[self.CMD_PIN]
            pin_entry = digital_response_table[int(pin_number)]
            value = str(pin_entry[1])
            report_entry = value
            report_entry += self.end_of_line
            return report_entry

    def analog_read(self, command):
        """
        This method retrieves analog input information for Snap!
        @param: command: Command and all possible parameters in list form
        @return: Current value of analog input pin
        """
        analog_response_table = self.firmata.get_analog_response_table()
        if self.analog_poll_list[int(command[self.CMD_PIN])] != self.firmata.IGNORE:
            pin_number = command[self.CMD_PIN]
            pin_entry = analog_response_table[int(pin_number)]
            value = str(pin_entry[1])
            report_entry = value
            report_entry += self.end_of_line
            return report_entry

    def set_encoder_mode(self, command):
        """
        This method is added for 86Duino Encoder library.
        """
        
        if self.check_CMD_ENABLE_DISABLE(command[self.CMD_ENABLE_DISABLE]) == 'Enable':
            
            module = int(command[2])
            
            if module == 0:
                self.digital_poll_list[42] = self.firmata.ENCODER
                self.digital_poll_list[43] = self.firmata.ENCODER
                self.digital_poll_list[44] = self.firmata.ENCODER
            elif module == 1:
                self.digital_poll_list[18] = self.firmata.ENCODER
                self.digital_poll_list[19] = self.firmata.ENCODER
                self.digital_poll_list[20] = self.firmata.ENCODER
            elif module == 2:
                self.digital_poll_list[33] = self.firmata.ENCODER
                self.digital_poll_list[34] = self.firmata.ENCODER
                self.digital_poll_list[35] = self.firmata.ENCODER
            elif module == 3:
                self.digital_poll_list[36] = self.firmata.ENCODER
                self.digital_poll_list[37] = self.firmata.ENCODER
                self.digital_poll_list[38] = self.firmata.ENCODER
                
            mode = command[3]
            encoder_mode = 0
            
            if mode == 'Pulse%2FDIR':
                encoder_mode = 0
            elif mode == 'CW%2FCCW':
                encoder_mode = 1
            elif mode == 'Pulse%20A%2FB':
                encoder_mode = 2
            
            self.encoder_poll_list[module] = self.firmata.ENCODER
            self.firmata.set_encoder_mode(1, module, encoder_mode)
        
        if self.check_CMD_ENABLE_DISABLE(command[self.CMD_ENABLE_DISABLE]) == 'Disable':
            module = int(command[2])
            
            if module == 0:
                self.digital_poll_list[42] = self.firmata.IGNORE
                self.digital_poll_list[43] = self.firmata.IGNORE
                self.digital_poll_list[44] = self.firmata.IGNORE
            elif module == 1:
                self.digital_poll_list[18] = self.firmata.IGNORE
                self.digital_poll_list[19] = self.firmata.IGNORE
                self.digital_poll_list[20] = self.firmata.IGNORE
            elif module == 2:
                self.digital_poll_list[33] = self.firmata.IGNORE
                self.digital_poll_list[34] = self.firmata.IGNORE
                self.digital_poll_list[35] = self.firmata.IGNORE
            elif module == 3:
                self.digital_poll_list[36] = self.firmata.IGNORE
                self.digital_poll_list[37] = self.firmata.IGNORE
                self.digital_poll_list[38] = self.firmata.IGNORE
            
            self.encoder_poll_list[module] = self.firmata.IGNORE
            self.firmata.set_encoder_mode(0, 0, 0)
        
        return 'okay'
        
    def move_one_servo_ex(self, command):
        """
        This method is added for 86Duino Servo library.
        """
        mode = 0
        id = int(command[1])
        pin = int(command[2])
        angle = int(command[3])
        if self.check_CMD_SERVO_MODE(command[4]) == 'Delay':
            mode = 0
        elif self.check_CMD_SERVO_MODE(command[4]) == 'Velocity':
            mode = 1
        msec_velocity = int(command[5])
        
        self.firmata.move_one_servo_ex(pin, angle, mode, msec_velocity)
        
        self.one_servo_is_moving = 1
        self.one_servo_busy_ID = id
        
        return 'okay'
        
    def config_servo_ex(self, command):
        """
        This method is added for 86Duino Servo library.
        """
        
        mode = 0
        pin = int(command[1])
        angle = int(command[2])
        if self.check_CMD_SERVO_MODE(command[3]) == 'Delay':
            mode = 0
        elif self.check_CMD_SERVO_MODE(command[3]) == 'Velocity':
            mode = 1
        msec_velocity = int(command[4])
        
        self.firmata.config_servo_ex(pin, angle, mode, msec_velocity)
        
        return 'okay'
        
    def move_servo_all(self, command):
        """
        This method is added dor 86Duino Servo library by Acen.
        """
        
        self.firmata.move_servo_all()
        
        return 'okay'
        
    def init_imu(self, command):
        """
        This method is added dor 86Duino FreeIMU1 library.
        """
        
        id = int(command[1])
        
        self.firmata.init_imu(id)
        
        self.imu_busy = 1
        self.imu_busy_ID = id
        
        return 'okay'
		
	
    def perform_forward(self, command):
        id = int(command[1])
        motion = 0
        times = int(command[2])
        self.firmata.perform_motion(id, motion, times)
        if id not in self.motion_busy_ID:
            self.motion_busy_ID.append(id)
        return 'okay'

    def perform_left(self, command):
        id = int(command[1])
        motion = 1
        times = int(command[2])
        self.firmata.perform_motion(id, motion, times)
        if id not in self.motion_busy_ID:
            self.motion_busy_ID.append(id)
        return 'okay'

    def perform_right(self, command):
        id = int(command[1])
        motion = 2
        times = int(command[2])
        self.firmata.perform_motion(id, motion, times)
        if id not in self.motion_busy_ID:
            self.motion_busy_ID.append(id)
        return 'okay'

    def perform_home(self, command):
        id = int(command[1])
        motion = 3
        times = 1
        self.firmata.perform_motion(id, motion, times)
        if id not in self.motion_busy_ID:
            self.motion_busy_ID.append(id)
        return 'okay'

    def perform_idle(self, command):
        id = int(command[1])
        motion = 4
        times = int(command[2])
        self.firmata.perform_motion(id, motion, times)
        if id not in self.motion_busy_ID:
            self.motion_busy_ID.append(id)
        return 'okay'

    def perform_hello(self, command):
        id = int(command[1])
        motion = 5
        times = int(command[2])
        self.firmata.perform_motion(id, motion, times)
        if id not in self.motion_busy_ID:
            self.motion_busy_ID.append(id)
        return 'okay'


	
    # This table must be at the bottom of the file because Python does not provide forward referencing for
    # the methods defined above.
    command_dict = {'crossdomain.xml': send_cross_domain_policy, 'reset_all': reset_arduino,
                    'digital_pin_mode': digital_pin_mode,  'analog_pin_mode': analog_pin_mode,
                    'digital_pin_mode_ja': digital_pin_mode_ja,
                    'analog_pin_mode_ja': analog_pin_mode_ja,
                    'digital_write': digital_write, 'analog_write': analog_write,
                    'play_tone': play_tone, 'play_tone_educake': play_tone_educake, 'tone_off': tone_off,
                    'set_servo_position': set_servo_position, 'poll': poll,
                    'debugger': debug_control, 'digital_read': digital_read, 'analog_read': analog_read,
                    'config_servo_ex': config_servo_ex, 'move_servo_all': move_servo_all,
                    'set_encoder_mode': set_encoder_mode, 'init_imu': init_imu, 'move_one_servo_ex': move_one_servo_ex,
                    'perform_forward': perform_forward,
                    'perform_left': perform_left,
                    'perform_right': perform_right,
                    'perform_home': perform_home,
                    'perform_idle': perform_idle,
                    'perform_hello': perform_hello
                    }
