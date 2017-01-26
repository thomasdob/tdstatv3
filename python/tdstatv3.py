#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import sys, platform
import time, datetime, timeit
import usb.core
import os.path
import collections
import numpy
import scipy.integrate

usb_vid = "0xa0a0"
usb_pid = "0x0002"
current_range_list = ["20 mA", u"200 µA", u"2 µA"]
shunt_calibration = [1.,1.,1.] # fine adjustment for shunt resistors, containing values of R1/10ohm, R2/1kohm, R3/100kohm
currentrange = 0
units_list = ["Potential (V)", "Current (mA)", "DAC Code"]
dev = None
current_offset = 0. # current offset in DAC counts
potential_offset = 0. # potential offset in DAC counts
potential = 0. # measured potential in V
current = 0. # measured current in mA
last_potential_values = collections.deque(maxlen=200)
last_current_values = collections.deque(maxlen=200)
raw_potential = 0 # measured potential in ADC counts
raw_current = 0 # measured current in ADC counts
last_raw_potential_values = collections.deque(maxlen=200)
last_raw_current_values = collections.deque(maxlen=200)
overcounter, undercounter = 0, 0 # for automatic current ranging
skipcounter = 0
time_of_last_adcread = 0.
adcread_interval = 0.09 # ADC sampling interval (in seconds)

if platform.system() != "Windows":
	# On Linux/OSX, use the Qt timer
	busyloop_interval = 0
	qt_timer_period = 1e3*adcread_interval # convert to ms
else:
	# On MS Windows, system timing is inaccurate, so use a busy loop instead
	busyloop_interval = adcread_interval
	qt_timer_period = 0

class AverageBuffer:
	def __init__(self, samples_to_average):
		self.samples_to_average = samples_to_average
		self.samples = []
		self.averagebuffer = []
	
	def add_sample(self, sample):
		self.samples.append(sample)
		if len(self.samples) >= self.samples_to_average:
			self.averagebuffer.append(sum(self.samples)/len(self.samples))
			self.samples = []
			
	def clear(self):
		self.samples = []
		self.averagebuffer = []

class States: # Simple state machine list
    NotConnected, Idle_Init, Idle, Measuring_Offset, Stationary_Graph, Measuring_CV, Measuring_CD, Measuring_Rate = range(8)
    
state = States.NotConnected

def current_to_string(currentrange, current_in_mA):
	abs_value = abs(current_in_mA)
	if currentrange == 0:
		if abs_value <= 9.9995:
			return u"%+6.3f mA"%current_in_mA
		else:
			return u"%+6.2f mA"%current_in_mA
	elif currentrange == 1:
		if abs_value < 9.9995e-2:
			return u"%+06.2f µA"%(current_in_mA*1e3)
		else:
			return u"%+6.1f µA"%(current_in_mA*1e3)
	elif currentrange == 2:
		return u"%+6.3f µA"%(current_in_mA*1e3)
		
def potential_to_string(potential_in_V):
	return u"%+6.3f V"%potential_in_V

def twocomplement_to_decimal(msb, middlebyte, lsb):
	ovh = (msb > 63) and (msb < 128) # check for overflow high (B22 set)
	ovl = (msb > 127) # check for overflow low (B21 set)
	combined_value = (msb%64)*2**16+middlebyte*2**8+lsb # get rid of overflow bits
	if not ovh and not ovl:
		if msb > 31: # B21 set -> negative number
			answer = combined_value - 2**22
		else:
			answer = combined_value
	else: # overflow
		if msb > 127: # B23 set -> negative number
			answer = combined_value - 2**22
		else:
			answer = combined_value
	return answer

def decimal_to_dac_string(value):
	value = int(round(value))
	if value > (2**19 - 1):
		value = 2**19 - 1
		#print "Requested DAC setting too high. It has been clipped to the maximum value."
	elif value < -2**19:
		value = -2**19
		#print "Requested DAC setting too low. It has been clipped to the minimum value."
	code = 2**19 + value
	byte1 = code / 2**12
	byte2 = (code % 2**12) / 2**4
	byte3 = (code - byte1*2**12 - byte2*2**4)*2**4
	return chr(byte1)+chr(byte2)+chr(byte3)
	
def dac_string_to_decimal(valuestr):
	code = 2**12*ord(valuestr[0])+2**4*ord(valuestr[1])+ord(valuestr[2])/2**4
	return code - 2**19
	
def cv_sweep(time_elapsed, ustart, ustop, ubound, lbound, scanrate, n):
	if scanrate < 0:
		try:
			return -cv_sweep(time_elapsed, -ustart, -ustop, -lbound, -ubound, -scanrate, n)
		except TypeError:
			return None
	srt_0 = ubound-ustart
	srt_1 = (ubound-lbound)*2.*n
	srt_2 = abs(ustop-ubound)
	srtime = scanrate*time_elapsed
	if srtime < srt_0:
		return ustart+srtime
	elif srtime < srt_0+srt_1:
		srtime = srtime - srt_0
		return lbound + abs((srtime)%(2*(ubound-lbound))-(ubound-lbound))
	elif srtime < srt_0+srt_1+srt_2:
		srtime = srtime - srt_0 - srt_1
		if ustop > ubound:
			return ubound + srtime
		else:
			return ubound - srtime
	else:
		return None
		
def charge_from_cv(time_arr, current_arr):
	zero_crossing_indices = []
	charge_arr = []
	running_index = 0
	while running_index < len(current_arr):
		counter = 0
		while running_index < len(current_arr) and current_arr[running_index] >= 0.:
			running_index += 1
			counter += 1
		if counter >= 10:
			zero_crossing_indices.append(running_index-counter)
		counter = 0
		while running_index < len(current_arr) and current_arr[running_index] <= 0.:
			running_index += 1
			counter += 1
		if counter >= 10:
			zero_crossing_indices.append(running_index-counter)
	for index in range(0,len(zero_crossing_indices)-1):
		zc_index1 = zero_crossing_indices[index]
		zc_index2 = zero_crossing_indices[index+1]
		charge_arr.append(numpy.trapz(current_arr[zc_index1:zc_index2],time_arr[zc_index1:zc_index2])*1000./3.6) # convert coulomb to uAh
	return charge_arr

def make_groupbox_indicator(title_name, default_text):
	myfont = QtGui.QFont()
	myfont.setPointSize(14)
	label = QtGui.QLabel(text=default_text, alignment=QtCore.Qt.AlignCenter)
	label.setFont(myfont)
	box = QtGui.QGroupBox(title=title_name, flat=False)
	format_box_for_display(box)
	layout = QtGui.QVBoxLayout()
	layout.addWidget(label, 0, alignment=QtCore.Qt.AlignCenter)
	layout.setSpacing(0)
	layout.setContentsMargins(30,3,30,0)
	box.setLayout(layout)
	return label, box

def add_my_tab(tab_frame, tab_name):
	widget = QtGui.QWidget()
	tab_frame.addTab(widget, tab_name)
	return widget

def format_box_for_display(box):
	color = box.palette().color(QtGui.QPalette.Background)
	rgb = color.red(), color.green(), color.blue()
	r, g, b = tuple(int(0.9*i) for i in rgb) # make background 10% darker to make the border color
	box.setStyleSheet("QGroupBox { border: 1px solid rgb(%d,%d,%d); border-radius: 4px; margin-top: 0.5em; font-weight: normal; color: gray;} QGroupBox::title {subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px;}"%(r,g,b))

def format_box_for_parameter(box):
	color = box.palette().color(QtGui.QPalette.Background)
	rgb = color.red(), color.green(), color.blue()
	r, g, b = tuple(int(0.7*i) for i in rgb) # make background 30% darker to make the border color
	box.setStyleSheet("QGroupBox { border: 1px solid rgb(%d,%d,%d); border-radius: 4px; margin-top: 0.5em; font-weight: bold} QGroupBox::title {subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px;}"%(r,g,b))

def make_label_entry(parent, labelname):
	hbox = QtGui.QHBoxLayout()
	label = QtGui.QLabel(text=labelname)
	entry = QtGui.QLineEdit()
	hbox.addWidget(label)
	hbox.addWidget(entry)
	parent.addLayout(hbox)
	return entry
	
def log_message(message):
	global statustext
	statustext.moveCursor(QtGui.QTextCursor.End)
	if str(statustext.toPlainText()) == "":
		statustext.insertPlainText(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ")+message)
	else:
		statustext.insertPlainText("\n"+datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ")+message)
	statustext.centerCursor()

def connect_usb():
	global dev, mainwidget, hardware_usb_vid, hardware_usb_pid, state
	if dev is not None:
		QtGui.QMessageBox.critical(mainwidget, "Already connected", "The USB device is already connected.")
		return
	usb_vid_string = str(hardware_usb_vid.text())
	usb_pid_string = str(hardware_usb_pid.text())
	dev = usb.core.find(idVendor=int(usb_vid_string, 0), idProduct=int(usb_pid_string, 0))
	if dev is None:
		QtGui.QMessageBox.critical(mainwidget, "USB Device Not Found", "No USB device was found with VID %s and PID %s. Verify the vendor/product ID and check the USB connection."%(usb_vid_string,usb_pid_string))
	else:
		log_message("USB Interface connected.")
		try:
			hardware_device_manufacturer.setText(dev.manufacturer)
			hardware_device_model.setText(dev.product)
			hardware_device_serial.setText(dev.serial_number)
			get_calibration()
			set_cell_status(False)
			set_control_mode(False)
			set_current_range()
			state = States.Idle_Init
		except ValueError:
			pass
			
def not_connected_errormessage():
	global mainwidget
	QtGui.QMessageBox.critical(mainwidget, "Not connected", "This command cannot be executed because the USB device is not connected. Press the \"Connect\" button and try again.")
	
def check_state(desired_states):
	global state, mainwidget
	if state not in desired_states:
		if state == 0:
			not_connected_errormessage()
		else:
			QtGui.QMessageBox.critical(mainwidget, "Error", "This command cannot be executed in the current state.")
		return False
	else:
		return True
			
def send_command(command_string, expected_response, log_msg=None):
	global dev, mainwidget
	if dev is not None:
		dev.write(0x01,command_string)
		response = dev.read(0x81,64).tostring()
		if response != "OK":
			QtGui.QMessageBox.critical(mainwidget, "Unexpected Response", "The command \"%s\" resulted in an unexpected response. The expected response was \"%s\"; the actual response was \"%s\""%(command_string,expected_response,response))
		else:
			if log_msg != None:
				log_message(log_msg)
		return True
	else:
		not_connected_errormessage()
		return False
		
def set_cell_status(true_for_cell_on):
	global cell_status_monitor
	if true_for_cell_on:
		if send_command("CELL ON", "OK"):
			cell_status_monitor.setText("CELL ON")
	else:
		if send_command("CELL OFF", "OK"):
			cell_status_monitor.setText("CELL OFF")

def set_control_mode(true_for_galvanostatic):
	if true_for_galvanostatic:
		if send_command("GALVANOSTATIC", "OK"):
			control_mode_monitor.setText("GALVANOSTATIC")
	else:
		if send_command("POTENTIOSTATIC", "OK"):
			control_mode_monitor.setText("POTENTIOSTATIC")

def set_current_range():
	global hardware_manual_control_range_dropdown, current_range_list, current_range_monitor, currentrange
	index = hardware_manual_control_range_dropdown.currentIndex()
	commandstring = ["RANGE 1","RANGE 2","RANGE 3"][index]
	if send_command(commandstring, "OK"):
		current_range_monitor.setText(current_range_list[index])
		currentrange = index
		
def auto_current_range():
	global current, hardware_manual_control_range_dropdown, currentrange, overcounter, undercounter, cv_range_checkboxes
	relativecurrent = abs(current/(20./100.**currentrange))
	if relativecurrent > 1.05 and currentrange != 0 and cv_range_checkboxes[currentrange-1].isChecked(): # switch to higher current range (if possible) after three detections
		overcounter += 1
	else:
		overcounter = 0
	if relativecurrent < 0.0095 and currentrange != 2 and cv_range_checkboxes[currentrange+1].isChecked(): # switch to lower current range (if possible) after three detections
		undercounter += 1
	else:
		undercounter = 0
	if overcounter > 3:
		currentrange -= 1
		hardware_manual_control_range_dropdown.setCurrentIndex(currentrange)
		set_current_range()
		overcounter = 0
		return 2 # skip next two measurements to suppress artifacts
	elif undercounter > 3:
		currentrange += 1
		hardware_manual_control_range_dropdown.setCurrentIndex(currentrange)
		set_current_range()
		undercounter = 0
		return 2 # skip next two measurements to suppress artifacts
	else:
		return 0
		
def current_range_from_current(current):
	current = abs(current)
	if current <= 0.002:
		return 2 # lowest current range
	elif current <= 0.2:
		return 1
	else:
		return 0 # highest current range

def get_next_enabled_current_range(desired_currentrange):
	global cv_range_checkboxes
	range_found = False
	found_currentrange = desired_currentrange
	for i in range(desired_currentrange,-1,-1): # look for an enabled current range, going up in current range
		if cv_range_checkboxes[i].isChecked():
			found_currentrange = i
			range_found = True
			break
	if not range_found:
		for i in range(desired_currentrange,3): # look for an enabled current range, going down in current range
			if cv_range_checkboxes[i].isChecked():
				found_currentrange = i
				break
	return found_currentrange
		
def set_offset():
	global potential_offset, current_offset
	send_command("OFFSETSAVE "+decimal_to_dac_string(potential_offset)+decimal_to_dac_string(current_offset), "OK", "Offset values saved to flash memory.")

def get_offset():
	global dev, hardware_calibration_potential_offset, hardware_calibration_current_offset, potential_offset, current_offset
	if dev is not None:
		dev.write(0x01,"OFFSETREAD")
		response = dev.read(0x81,64).tostring()
		if response != 6*chr(255):
			potential_offset = dac_string_to_decimal(response[0:3])
			current_offset = dac_string_to_decimal(response[3:6])	
			hardware_calibration_potential_offset.setText("%d"%potential_offset)
			hardware_calibration_current_offset.setText("%d"%current_offset)	
			log_message("Offset values read from flash memory.")
		else:
			log_message("No offset values were found in flash memory.")
	else:
		not_connected_errormessage()

def float_to_twobytes(value):
	value = int(round(value))
	if value > (2**15 - 1):
		value = 2**15 - 1
	elif value < -2**15:
		value = -2**15
	code = 2**15 + value
	byte1 = code / 2**8
	byte2 = code % 2**8
	return chr(byte1)+chr(byte2)
	
def twobytes_to_float(valuestr):
	code = 2**8*ord(valuestr[0])+ord(valuestr[1])
	return float(code - 2**15)

def set_shunt_calibration():
	global shunt_calibration
	send_command("SHUNTCALSAVE "+float_to_twobytes((shunt_calibration[0]-1.)*1e6)+float_to_twobytes((shunt_calibration[1]-1.)*1e6)+float_to_twobytes((shunt_calibration[2]-1.)*1e6), "OK", "Shunt calibration values saved to flash memory.")
	
def get_shunt_calibration():
	global dev, hardware_calibration_shuntvalues, shunt_calibration
	if dev is not None:
		dev.write(0x01,"SHUNTCALREAD")
		response = dev.read(0x81,64).tostring()
		if response != 6*chr(255):
			for i in range(0,3):
				shunt_calibration[i] = 1.+twobytes_to_float(response[2*i:2*i+2])/1e6
				hardware_calibration_shuntvalues[i].setText("%.4f"%shunt_calibration[i])
			log_message("Shunt calibration values read from flash memory.")
		else:
			log_message("No shunt calibration values were found in flash memory.")
	else:
		not_connected_errormessage()
		
def zero_offset():
	global last_raw_potential_values, last_raw_current_values, hardware_calibration_potential_offset, hardware_calibration_current_offset
	if not check_state([States.Idle]):
		return
	pot_offs = int(round(np.average(list(last_raw_potential_values))))
	cur_offs = int(round(np.average(list(last_raw_current_values))))
	hardware_calibration_potential_offset.setText("%d"%pot_offs)
	hardware_calibration_current_offset.setText("%d"%cur_offs)
	offset_changed_callback()

def offset_changed_callback():
	global potential_offset, current_offset, hardware_calibration_potential_offset, hardware_calibration_current_offset
	try:
		potential_offset = int(hardware_calibration_potential_offset.text())
		hardware_calibration_potential_offset.setStyleSheet("")
	except ValueError:
		hardware_calibration_potential_offset.setStyleSheet("QLineEdit { background: red; }")
	try:
		current_offset = int(hardware_calibration_current_offset.text())
		hardware_calibration_current_offset.setStyleSheet("")
	except ValueError:
		hardware_calibration_current_offset.setStyleSheet("QLineEdit { background: red; }")

def shunt_calibration_changed_callback():
	global hardware_calibration_shuntvalues, shunt_calibration
	for i in range(0,3):
		try:
			shunt_calibration[i] = float(hardware_calibration_shuntvalues[i].text())
			hardware_calibration_shuntvalues[i].setStyleSheet("")
		except ValueError:
			hardware_calibration_shuntvalues[i].setStyleSheet("QLineEdit { background: red; }")

def set_dac_cal():
	global hardware_calibration_dac_offset, hardware_calibration_dac_gain
	try:
		dac_offset = int(hardware_calibration_dac_offset.text())
		hardware_calibration_dac_offset.setStyleSheet("")
	except ValueError:
		hardware_calibration_dac_offset.setStyleSheet("QLineEdit { background: red; }")
		return
	try:
		dac_gain = int(hardware_calibration_dac_gain.text())
		hardware_calibration_dac_gain.setStyleSheet("")
	except ValueError:
		hardware_calibration_dac_gain.setStyleSheet("QLineEdit { background: red; }")
		return
	send_command("DACCALSET "+decimal_to_dac_string(dac_offset)+decimal_to_dac_string(dac_gain-2**19), "OK", "DAC calibration saved to flash memory.")

def get_dac_cal():
	global dev, hardware_calibration_dac_offset, hardware_calibration_dac_gain
	dev.write(0x01,"DACCALGET")
	response = dev.read(0x81,64).tostring()
	if response != 6*chr(255):
		dac_offset = dac_string_to_decimal(response[0:3])
		dac_gain = dac_string_to_decimal(response[3:6])+2**19
		hardware_calibration_dac_offset.setText("%d"%dac_offset)
		hardware_calibration_dac_gain.setText("%d"%dac_gain)
		log_message("DAC calibration read from flash memory.")
	else:
		log_message("No DAC calibration values were found in flash memory.")

def set_calibration():
	set_dac_cal()
	set_offset()
	set_shunt_calibration()
	return

def get_calibration():
	get_dac_cal()
	get_offset()
	get_shunt_calibration()
	return

def dac_calibrate():
	send_command("DACCAL", "OK", "DAC calibration performed.")
	get_dac_cal()

def set_output(value_units_index, value):
	global current_range_list, potential_offset, current_offset, hardware_manual_control_range_dropdown, currentrange
	if value_units_index == 0:
		send_command("DACSET "+decimal_to_dac_string(value/8.*2.**19+int(round(potential_offset/4.))), "OK")
	elif value_units_index == 1:
		send_command("DACSET "+decimal_to_dac_string(value/(25./(shunt_calibration[currentrange]*100.**currentrange))*2.**19+int(round(current_offset/4.))), "OK")
	elif value_units_index == 2:
		send_command("DACSET "+decimal_to_dac_string(value), "OK")

def set_output_from_gui():
	global mainwidget, hardware_manual_control_output_entry, hardware_manual_control_output_dropdown
	value_units_index = hardware_manual_control_output_dropdown.currentIndex()
	if value_units_index == 0: # Potential (V)
		try:
			value = float(hardware_manual_control_output_entry.text())		
		except ValueError:
			QtGui.QMessageBox.critical(mainwidget, "Not a number", "The value you have entered is not a floating-point number.")
			return
	elif value_units_index == 1: # Current (mA)
		try:
			value = float(hardware_manual_control_output_entry.text())
		except ValueError:
			QtGui.QMessageBox.critical(mainwidget, "Not a number", "The value you have entered is not a floating-point number.")
			return
	elif value_units_index == 2: # DAC Code
		try:
			value = int(hardware_manual_control_output_entry.text())
		except ValueError:
			QtGui.QMessageBox.critical(mainwidget, "Not a number", "The value you have entered is not an integer number.")
			return
	else:
		return
	set_output(value_units_index, value)

def wait_for_adcread():
	global app, time_of_last_adcread, busyloop_interval
	if busyloop_interval == 0:
		return
	else:
		time.sleep(busyloop_interval/2.) # sleep for some time to prevent wasting too many CPU cycles
		app.processEvents()
		while timeit.default_timer() < time_of_last_adcread + busyloop_interval:
			pass # busy loop (this is the only way to get accurate timing on MS Windows)

def read_potential_current():
	global dev, potential_offset, current_offset, currentrange, potential, current, raw_potential, raw_current, time_of_last_adcread
	wait_for_adcread()
	time_of_last_adcread = timeit.default_timer()
	dev.write(0x01,"ADCREAD")
	response = dev.read(0x81,64)
	if response.tostring() != "WAIT":
		raw_potential = twocomplement_to_decimal(response[0], response[1], response[2])
		raw_current = twocomplement_to_decimal(response[3], response[4], response[5])
		potential = (raw_potential-potential_offset)/2097152.*8.
		current = (raw_current-current_offset)/2097152.*25./(shunt_calibration[currentrange]*100.**currentrange)
		potential_monitor.setText(potential_to_string(potential))
		current_monitor.setText(current_to_string(currentrange, current))

def idle_init():
	global plot_frame, potential_plot_curve, current_plot_curve, legend, state
	plot_frame.clear()
	try:
		legend.scene().removeItem(legend)
	except AttributeError:
		pass
	except NameError:
		pass
	plot_frame.setLabel('bottom', 'Sample #', units="")
	plot_frame.setLabel('left', 'Value', units="")
	legend = plot_frame.addLegend()
	plot_frame.enableAutoRange()
	plot_frame.setXRange(0,200,update=True)
	potential_plot_curve = plot_frame.plot(pen='g', name='Potential (V)')
	current_plot_curve = plot_frame.plot(pen='r', name='Current (mA)')
	state = States.Idle

def update_live_graph():
	global potential, current, last_potential_values, last_current_values, potential_plot_curve, current_plot_curve, raw_potential, raw_current, last_raw_potential_values, last_raw_current_values
	last_potential_values.append(potential)
	last_current_values.append(current)
	last_raw_potential_values.append(raw_potential)
	last_raw_current_values.append(raw_current)
	xvalues = range(last_potential_values.maxlen-len(last_potential_values),last_potential_values.maxlen)
	potential_plot_curve.setData(xvalues, list(last_potential_values))
	current_plot_curve.setData(xvalues, list(last_current_values))

def cv_choose_file():
	global mainwidget, cv_file_entry
	filedialog = QtGui.QFileDialog()
	cv_filename = filedialog.getSaveFileName(mainwidget, "Choose where to save the CV measurement data", "", "ASCII data (*.txt)",options=QtGui.QFileDialog.DontConfirmOverwrite)
	cv_file_entry.setText(cv_filename)

def cv_getparams():
	global mainwidget, cv_lbound, cv_ubound, cv_startpot, cv_stoppot, cv_scanrate, cv_numcycles, cv_numsamples, cv_filename
	try:
		cv_lbound = float(cv_lbound_entry.text())
		cv_ubound = float(cv_ubound_entry.text())
		cv_startpot = float(cv_startpot_entry.text())
		cv_stoppot = float(cv_stoppot_entry.text())
		cv_scanrate = float(cv_scanrate_entry.text())/1e3 #convert to V/s
		cv_numcycles = int(cv_numcycles_entry.text())
		cv_numsamples = int(cv_numsamples_entry.text())
		cv_filename = str(cv_file_entry.text())
		return True
	except ValueError:
		QtGui.QMessageBox.critical(mainwidget, "Not a number", "One or more parameters could not be interpreted as a number.")
		return False

def cv_validate_parameters():
	global mainwidget, cv_lbound, cv_ubound, cv_startpot, cv_stoppot, cv_scanrate, cv_numcycles, cv_numsamples, cv_filename
	if cv_ubound < cv_lbound:
		QtGui.QMessageBox.critical(mainwidget, "CV error", "The upper bound cannot be lower than the lower bound.")
		return False
	if cv_scanrate == 0:
		QtGui.QMessageBox.critical(mainwidget, "CV error", "The scan rate cannot be zero.")
		return False
	if (cv_scanrate > 0) and (cv_ubound < cv_startpot):
		QtGui.QMessageBox.critical(mainwidget, "CV error", "For a positive scan rate, the start potential must be lower than the upper bound.")
		return False
	if (cv_scanrate < 0) and (cv_lbound > cv_startpot):
		QtGui.QMessageBox.critical(mainwidget, "CV error", "For a negative scan rate, the start potential must be higher than the lower bound.")
		return False
	if cv_numsamples < 1:
		QtGui.QMessageBox.critical(mainwidget, "CV error", "The number of samples to average must be at least 1.")
		return False
	else:
		return True

def validate_file(filename):
	global mainwidget
	if os.path.isfile(filename):
		if QtGui.QMessageBox.question(mainwidget, "File exists", "The specified output file already exists. Do you want to overwrite it?", QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.No) != QtGui.QMessageBox.Yes:
			return False
	try:
		tryfile = open(filename, 'w', 0)
		tryfile.close()
		return True
	except IOError:
		QtGui.QMessageBox.critical(mainwidget, "File error", "The specified output file path is not valid.")
		return False

def cv_scanrate_changed_callback():
	try:
		cv_scanrate = float(cv_scanrate_entry.text())
		numsamples = int(20./abs(cv_scanrate))+1
		cv_numsamples_entry.setText("%d"%numsamples)
	except:
		pass

def cv_start():
	global cv_filename, cv_startpot, cv_starttime, cv_numsamples, cv_time_data, cv_potential_data, cv_current_data, plot_frame, legend, cv_plot_curve, cv_outputfile, hardware_manual_control_range_dropdown, preview_cancel_button, state, skipcounter, current
	if check_state([States.Idle,States.Stationary_Graph]) and cv_getparams() and cv_validate_parameters() and validate_file(cv_filename):
		cv_outputfile = open(cv_filename, 'w', 0)
		cv_outputfile.write("Elapsed time(s)\tPotential(V)\tCurrent(A)\n")
		set_output(0, cv_startpot)
		set_control_mode(False) # potentiostatic control
		hardware_manual_control_range_dropdown.setCurrentIndex(0) # start at highest current range
		set_current_range()
		time.sleep(.1) # allow DAC some time to settle
		cv_time_data = AverageBuffer(cv_numsamples)
		cv_potential_data = AverageBuffer(cv_numsamples)
		cv_current_data = AverageBuffer(cv_numsamples)
		set_cell_status(True)
		time.sleep(.1) # allow feedback loop some time to settle
		read_potential_current()
		time.sleep(.1)
		read_potential_current()
		hardware_manual_control_range_dropdown.setCurrentIndex(get_next_enabled_current_range(current_range_from_current(current)))
		set_current_range()
		time.sleep(.1)
		read_potential_current()
		time.sleep(.1)
		read_potential_current()
		hardware_manual_control_range_dropdown.setCurrentIndex(get_next_enabled_current_range(current_range_from_current(current)))
		set_current_range()
		preview_cancel_button.hide()
		try:
			legend.scene().removeItem(legend)
		except AttributeError:
			pass
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.setLabel('bottom', 'Potential', units="V")
		plot_frame.setLabel('left', 'Current', units="A")
		cv_plot_curve = plot_frame.plot(pen='y')
		log_message("CV measurement started. Saving to: %s"%cv_filename)
		state = States.Measuring_CV
		skipcounter = 2 # skip first two data points to suppress artifacts
		cv_starttime = timeit.default_timer()

def cv_update():
	global cv_startpot, cv_stoppot, cv_ubound, cv_lbound, cv_scanrate, cv_numcycles, cv_outputfile, cv_starttime, cv_time_data, cv_potential_data, potential, cv_current_data, current, cv_plot_curve, preview_cancel_button, state, skipcounter
	elapsed_time = timeit.default_timer()-cv_starttime
	cv_output = cv_sweep(elapsed_time, cv_startpot, cv_stoppot, cv_ubound, cv_lbound, cv_scanrate, cv_numcycles)
	if cv_output == None: # this signifies the end of the CV scan
		state = States.Idle_Init
		set_cell_status(False)
		cv_outputfile.close()
		charge_arr = charge_from_cv(cv_time_data.averagebuffer, cv_current_data.averagebuffer)
		log_message("CV measurement finished. Calculated charges (in uAh): [" + ', '.join("%.2f"%value for value in charge_arr) + "]")
		state = States.Stationary_Graph
		preview_cancel_button.show()
		
	else:
		set_output(0, cv_output)
		read_potential_current()
		if skipcounter == 0 and state == States.Measuring_CV:
			cv_time_data.add_sample(elapsed_time)
			cv_potential_data.add_sample(potential)
			cv_current_data.add_sample(1e-3*current) #convert to A
			if len(cv_time_data.samples) == 0 and len(cv_time_data.averagebuffer) > 0: # a new average was just calculated
				cv_outputfile.write("%e\t%e\t%e\n"%(cv_time_data.averagebuffer[-1],cv_potential_data.averagebuffer[-1],cv_current_data.averagebuffer[-1])) # write it out
				cv_plot_curve.setData(cv_potential_data.averagebuffer,cv_current_data.averagebuffer) # update the graph
			skipcounter = auto_current_range()
		elif state == States.Measuring_CV:
			skipcounter -= 1

def cv_stop():
	global cv_outputfile, cv_time_data, cv_current_data, preview_cancel_button, state
	if check_state([States.Measuring_CV]):
		set_cell_status(False)
		cv_outputfile.close()
		charge_arr = charge_from_cv(cv_time_data.averagebuffer, cv_current_data.averagebuffer)
		log_message("CV measurement interrupted. Calculated charges (in uAh): [" + ', '.join("%.2f"%value for value in charge_arr) + "]")
		state = States.Stationary_Graph
		preview_cancel_button.show()

def cv_preview():
	global cv_startpot, cv_stoppot, cv_ubound, cv_lbound, cv_scanrate, cv_numcycles, plot_frame, legend, preview_cancel_button, state
	if check_state([States.Idle,States.Stationary_Graph]) and cv_getparams() and cv_validate_parameters():
		time_arr = []
		potential_arr = []
		timeval = 0.
		cv_val = cv_sweep(timeval, cv_startpot, cv_stoppot, cv_ubound, cv_lbound, cv_scanrate, cv_numcycles)
		while cv_val != None:
			time_arr.append(timeval)
			potential_arr.append(cv_val)
			timeval += abs((cv_ubound-cv_lbound)/100./cv_scanrate)
			cv_val = cv_sweep(timeval, cv_startpot, cv_stoppot, cv_ubound, cv_lbound, cv_scanrate, cv_numcycles)
		try:
			legend.scene().removeItem(legend)
		except AttributeError:
			pass
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.setLabel('bottom', 'Time', units='s')
		plot_frame.setLabel('left', 'Potential', units='V')
		plot_frame.plot(time_arr, potential_arr, pen='g')
		preview_cancel_button.show()
		state = States.Stationary_Graph
		
def cv_preview_cancel():
	global preview_cancel_button, state
	state = States.Idle_Init
	preview_cancel_button.hide()

def cv_get_ocp():
	global potential
	cv_startpot_entry.setText('%5.3f'%potential)

def cd_choose_file():
	global mainwidget, cd_file_entry
	filedialog = QtGui.QFileDialog()
	cd_filename = filedialog.getSaveFileName(mainwidget, "Choose where to save the charge/discharge measurement data", "", "ASCII data (*.txt)",options=QtGui.QFileDialog.DontConfirmOverwrite)
	cd_file_entry.setText(cd_filename)
	
def cd_getparams():
	global mainwidget, cd_lbound, cd_ubound, cd_chargecurrent, cd_dischargecurrent, cd_numcycles, cd_numsamples, cd_filename
	try:
		cd_lbound = float(cd_lbound_entry.text())
		cd_ubound = float(cd_ubound_entry.text())
		cd_chargecurrent = float(cd_chargecurrent_entry.text())/1e3 # convert to mA
		cd_dischargecurrent = float(cd_dischargecurrent_entry.text())/1e3 # convert to mA
		cd_numcycles = int(cd_numcycles_entry.text())
		cd_numsamples = int(cd_numsamples_entry.text())
		cd_filename = str(cd_file_entry.text())
		return True
	except ValueError:
		QtGui.QMessageBox.critical(mainwidget, "Not a number", "One or more parameters could not be interpreted as a number.")
		return False

def cd_validate_parameters():
	global mainwidget, cd_lbound, cd_ubound, cd_chargecurrent, cd_dischargecurrent, cd_numcycles, cd_numsamples, cv_filename
	if cd_ubound < cd_lbound:
		QtGui.QMessageBox.critical(mainwidget, "Charge/discharge error", "The upper bound cannot be lower than the lower bound.")
		return False
	if cd_chargecurrent == 0.:
		QtGui.QMessageBox.critical(mainwidget, "Charge/discharge error", "The charge current cannot be zero.")
		return False
	if cd_dischargecurrent == 0.:
		QtGui.QMessageBox.critical(mainwidget, "Charge/discharge error", "The discharge current cannot be zero.")
		return False
	if cd_chargecurrent*cd_dischargecurrent > 0:
		QtGui.QMessageBox.critical(mainwidget, "Charge/discharge error", "Charge and discharge current must have opposite sign.")
		return False
	if cd_numcycles <= 0:
		QtGui.QMessageBox.critical(mainwidget, "Charge/discharge error", "The number of half cycles must be positive and non-zero.")
		return False
	if cd_numsamples < 1:
		QtGui.QMessageBox.critical(mainwidget, "Charge/discharge error", "The number of samples to average must be at least 1.")
		return False
	else:
		return True

def cd_start():
	global cd_charges, cd_currentsetpoint, cd_filename, cd_chargecurrent, cd_starttime, cd_numsamples, cd_currentcycle, cd_time_data, cd_potential_data, cd_current_data, plot_frame, legend, cd_plot_curves, cd_outputfile_raw, cd_outputfile_capacities, hardware_manual_control_range_dropdown, preview_cancel_button, cd_current_cycle_entry, state
	if check_state([States.Idle,States.Stationary_Graph]) and cd_getparams() and cd_validate_parameters() and validate_file(cd_filename):
		cd_currentcycle = 1
		cd_charges = []
		cd_plot_curves = []
		cd_outputfile_raw = open(cd_filename, 'w', 0)
		cd_outputfile_raw.write("Elapsed time(s)\tPotential(V)\tCurrent(A)\n")
		base, extension = os.path.splitext(cd_filename)
		cd_outputfile_capacities = open(base+'_capacities'+extension, 'w', 1)
		cd_outputfile_capacities.write("Cycle number\tCharge capacity (Ah)\tDischarge capacity (Ah)\n")
		cd_currentsetpoint = cd_chargecurrent
		hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(cd_currentsetpoint))
		set_current_range()
		set_output(1, cd_currentsetpoint)
		set_control_mode(True)
		time.sleep(.2) # allow DAC some time to settle
		cd_starttime = timeit.default_timer()
		cd_time_data = AverageBuffer(cd_numsamples)
		cd_potential_data = AverageBuffer(cd_numsamples)
		cd_current_data = AverageBuffer(cd_numsamples)
		set_cell_status(True)
		preview_cancel_button.hide()
		try:
			legend.scene().removeItem(legend)
		except AttributeError:
			pass
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.setLabel('bottom', 'Inserted/extracted charge', units="Ah")
		plot_frame.setLabel('left', 'Potential', units="V")
		cd_plot_curves.append(plot_frame.plot(pen='y'))
		log_message("Charge/discharge measurement started. Saving to: %s"%cd_filename)
		cd_current_cycle_entry.setText("1")
		state = States.Measuring_CD
	
def cd_stop():
	global cd_charges, cd_outputfile_raw, cd_outputfile_capacities, preview_cancel_button, cd_current_cycle_entry, state
	if check_state([States.Measuring_CD]):
		state = States.Idle_Init
		set_cell_status(False)
		cd_outputfile_raw.close()
		cd_outputfile_capacities.close()
		log_message("Charge/discharge measurement interrupted. Calculated charges (in uAh): [" + ', '.join("%.2f"%(value*1e6) for value in cd_charges) + "]")
		cd_current_cycle_entry.setText("")
		state = States.Stationary_Graph
		preview_cancel_button.show()

def cd_update():
	global cd_currentsetpoint, cd_charges, cd_ubound, cd_lbound, cv_scanrate, cd_chargecurrent, cd_dischargecurrent, cd_numcycles, cd_currentcycle, cd_outputfile_raw, cd_outputfile_capacities, cd_starttime, cd_time_data, cd_potential_data, potential, cd_current_data, current, cd_plot_curves, preview_cancel_button, cd_current_cycle_entry, state
	elapsed_time = timeit.default_timer()-cd_starttime
	if cd_currentcycle > cd_numcycles: # end of charge/discharge measurements
		state = States.Idle_Init
		set_cell_status(False)
		cd_outputfile_raw.close()
		cd_outputfile_capacities.close()
		log_message("Charge/discharge measurement finished. Calculated charges (in uAh): [" + ', '.join("%.2f"%(value*1e6) for value in cd_charges) + "]")
		cd_current_cycle_entry.setText("")
		state = States.Stationary_Graph
		preview_cancel_button.show()
		
	else:
		read_potential_current()
		cd_time_data.add_sample(elapsed_time)
		cd_potential_data.add_sample(potential)
		cd_current_data.add_sample(1e-3*current) #convert to A
		if len(cd_time_data.samples) == 0 and len(cd_time_data.averagebuffer) > 0: # a new average was just calculated
			cd_outputfile_raw.write("%e\t%e\t%e\n"%(cd_time_data.averagebuffer[-1],cd_potential_data.averagebuffer[-1],cd_current_data.averagebuffer[-1])) # write it out
			charge = numpy.abs(scipy.integrate.cumtrapz(cd_current_data.averagebuffer,cd_time_data.averagebuffer,initial=0.)/3600.) # cumulative charge in Ah
			cd_plot_curves[cd_currentcycle-1].setData(charge,cd_potential_data.averagebuffer) # update the graph
		if (cd_currentsetpoint > 0 and potential > cd_ubound) or (cd_currentsetpoint < 0 and potential < cd_lbound):
			if cd_currentsetpoint == cd_chargecurrent:
				cd_currentsetpoint = cd_dischargecurrent
			else:
				cd_currentsetpoint = cd_chargecurrent
			hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(cd_currentsetpoint))
			set_current_range()
			set_output(1, cd_currentsetpoint)
			cd_plot_curves.append(plot_frame.plot(pen='y'))
			cd_charges.append(numpy.abs(numpy.trapz(cd_current_data.averagebuffer,cd_time_data.averagebuffer)/3600.)) # cumulative charge in Ah
			if cd_currentcycle % 2 == 0:
				cd_outputfile_capacities.write("%d\t%e\t%e\n"%(cd_currentcycle/2,cd_charges[cd_currentcycle-2],cd_charges[cd_currentcycle-1]))
			cd_time_data.clear()
			cd_potential_data.clear()
			cd_current_data.clear()
			cd_currentcycle += 1
			cd_current_cycle_entry.setText("%d"%cd_currentcycle)

def rate_choose_file():
	global mainwidget, rate_file_entry
	filedialog = QtGui.QFileDialog()
	rate_filename = filedialog.getSaveFileName(mainwidget, "Choose where to save the rate testing measurement data", "", "ASCII data (*.txt)",options=QtGui.QFileDialog.DontConfirmOverwrite)
	rate_file_entry.setText(rate_filename)
	
def rate_getparams():
	global mainwidget, rate_lbound, rate_ubound, rate_crates, rate_currents, rate_numcycles, rate_filename
	try:
		rate_lbound = float(rate_lbound_entry.text())
		rate_ubound = float(rate_ubound_entry.text())
		one_c_current = float(rate_capacity_entry.text())/1e3 # convert to mA
		rate_crates = [float(x) for x in rate_crates_entry.text().split(",")]
		rate_currents = [one_c_current*rc for rc in rate_crates]
		rate_numcycles = int(rate_numcycles_entry.text())
		cd_numsamples = int(cd_numsamples_entry.text())
		rate_filename = str(rate_file_entry.text())
		return True
	except ValueError:
		QtGui.QMessageBox.critical(mainwidget, "Not a number", "One or more parameters could not be interpreted as a number.")
		return False

def rate_validate_parameters():
	global mainwidget, rate_lbound, rate_ubound, rate_currents, rate_numcycles, rate_filename
	if rate_ubound < rate_lbound:
		QtGui.QMessageBox.critical(mainwidget, "Rate testing error", "The upper bound cannot be lower than the lower bound.")
		return False
	if 0. in rate_currents:
		QtGui.QMessageBox.critical(mainwidget, "Rate testing error", "The charge/discharge current cannot be zero.")
		return False
	if rate_numcycles <= 0:
		QtGui.QMessageBox.critical(mainwidget, "Charge/discharge error", "The number of half cycles must be positive and non-zero.")
		return False
	else:
		return True

def rate_start():
	global mainwidget, rate_current_crate_entry, state, rate_lbound, rate_ubound, rate_crates, rate_currents, rate_numcycles, rate_filename, rate_current_crate_index, rate_halfcycle_countdown, rate_chg_charges, rate_dis_charges, rate_outputfile_raw, rate_outputfile_capacities, rate_starttime, rate_time_data, rate_potential_data, rate_current_data, rate_plot_scatter_chg, rate_plot_scatter_dis, plot_frame, legend
	if check_state([States.Idle,States.Stationary_Graph]) and rate_getparams() and rate_validate_parameters() and validate_file(rate_filename):
		rate_current_crate_index = 0
		rate_halfcycle_countdown = 2*rate_numcycles
		rate_chg_charges = []
		rate_dis_charges = []
		rate_outputfile_raw = open(rate_filename, 'w', 0)
		rate_outputfile_raw.write("Elapsed time(s)\tPotential(V)\tCurrent(A)\n")
		base, extension = os.path.splitext(rate_filename)
		rate_outputfile_capacities = open(base+'_capacities'+extension, 'w', 0)
		rate_outputfile_capacities.write("C-rate\tCharge capacity (Ah)\tDischarge capacity (Ah)\n")
		rate_current = rate_currents[rate_current_crate_index] if rate_halfcycle_countdown%2 == 0 else -rate_currents[rate_current_crate_index]
		hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(rate_current))
		set_current_range()
		set_output(1, rate_current)
		set_control_mode(True)
		time.sleep(.2) # allow DAC some time to settle
		rate_starttime = timeit.default_timer()
		numsamples = max(1,int(36./rate_crates[rate_current_crate_index]))
		rate_time_data = AverageBuffer(numsamples)
		rate_potential_data = AverageBuffer(numsamples)
		rate_current_data = AverageBuffer(numsamples)
		set_cell_status(True)
		preview_cancel_button.hide()
		try:
			legend.scene().removeItem(legend)
		except AttributeError:
			pass
		plot_frame.clear()
		legend = plot_frame.addLegend()
		plot_frame.enableAutoRange()
		plot_frame.setLabel('bottom', 'C-rate')
		plot_frame.setLabel('left', 'Inserted/extracted charge', units="Ah")
		rate_plot_scatter_chg = plot_frame.plot(symbol='o', pen=None, symbolPen='r', symbolBrush='r', name='Charge')
		rate_plot_scatter_dis = plot_frame.plot(symbol='o', pen=None, symbolPen=(100,100,255), symbolBrush=(100,100,255), name='Discharge')
		log_message("Rate testing started. Saving to: %s"%rate_filename)
		rate_current_crate_entry.setText("1")
		state = States.Measuring_Rate
	
def rate_stop():
	global state
	if check_state([States.Measuring_Rate]):
		state = States.Idle_Init
		set_cell_status(False)
		rate_outputfile_raw.close()
		rate_outputfile_capacities.close()
		log_message("Rate testing finished.")
		rate_current_crate_entry.setText("")
		state = States.Stationary_Graph
		preview_cancel_button.show()

def rate_update():
	global mainwidget, rate_current_crate_entry, state, rate_lbound, rate_ubound, rate_crates, rate_currents, rate_numcycles, rate_filename, rate_current_crate_index, rate_halfcycle_countdown, rate_chg_charges, rate_dis_charges, rate_outputfile_raw, rate_outputfile_capacities, rate_starttime, rate_time_data, rate_potential_data, rate_current_data, rate_plot_scatter_chg, rate_chg_charges, rate_plot_scatter_dis, rate_dis_charges, current, potential
	elapsed_time = timeit.default_timer()-rate_starttime
	read_potential_current()
	rate_time_data.add_sample(elapsed_time)
	rate_potential_data.add_sample(potential)
	rate_current_data.add_sample(1e-3*current) #convert to A
	if len(rate_time_data.samples) == 0 and len(rate_time_data.averagebuffer) > 0: # a new average was just calculated
		rate_outputfile_raw.write("%e\t%e\t%e\n"%(rate_time_data.averagebuffer[-1],rate_potential_data.averagebuffer[-1],rate_current_data.averagebuffer[-1])) # write it out
	if (rate_halfcycle_countdown%2 == 0 and potential > rate_ubound) or (rate_halfcycle_countdown%2 != 0 and potential < rate_lbound): # end of half cycle
		rate_halfcycle_countdown -= 1
		if rate_halfcycle_countdown == 1: # calculate and plot charge capacity for current C-rate
			charge = numpy.abs(scipy.integrate.trapz(rate_current_data.averagebuffer,rate_time_data.averagebuffer)/3600.) # charge in Ah
			rate_chg_charges.append(charge)
			rate_plot_scatter_chg.setData(rate_crates[0:rate_current_crate_index+1], rate_chg_charges)
		elif rate_halfcycle_countdown == 0: # calculate and plot discharge capacity for current C-rate; go to next one
			charge = numpy.abs(scipy.integrate.trapz(rate_current_data.averagebuffer,rate_time_data.averagebuffer)/3600.) # charge in Ah
			rate_dis_charges.append(charge)
			rate_plot_scatter_dis.setData(rate_crates[0:rate_current_crate_index+1], rate_dis_charges)
			rate_outputfile_capacities.write("%e\t%e\t%e\n"%(rate_crates[rate_current_crate_index],rate_chg_charges[-1],rate_dis_charges[-1]))
			if rate_current_crate_index == len(rate_crates)-1: # last C-rate was measured
				state = States.Idle_Init
				set_cell_status(False)
				rate_outputfile_raw.close()
				rate_outputfile_capacities.close()
				log_message("Rate testing finished.")
				rate_current_crate_entry.setText("")
				state = States.Stationary_Graph
				preview_cancel_button.show()
				return
			else: # new C-rate
				rate_current_crate_index += 1
				rate_halfcycle_countdown = 2*rate_numcycles
				set_output(1, 0.) # set zero current while range switching
				hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(rate_currents[rate_current_crate_index]))
				set_current_range() # set new current range
				numsamples = max(1,int(36./rate_crates[rate_current_crate_index])) # set the appropriate amount of samples to average for the new C-rate
				rate_time_data.samples_to_average = numsamples
				rate_potential_data.samples_to_average = numsamples
				rate_current_data.samples_to_average = numsamples
				
		rate_current = rate_currents[rate_current_crate_index] if rate_halfcycle_countdown%2 == 0 else -rate_currents[rate_current_crate_index]
		set_output(1, rate_current)
		rate_time_data.clear()
		rate_potential_data.clear()
		rate_current_data.clear()
		rate_current_crate_entry.setText("%d"%rate_crates[rate_current_crate_index])

app = QtGui.QApplication([])
win = QtGui.QMainWindow()
win.setGeometry(300,300,1024,700)
win.setWindowTitle('USB potentiostat/galvanostat')
win.setWindowIcon(QtGui.QIcon('icon/icon.png'))

potential_monitor, potential_monitor_box = make_groupbox_indicator("Measured potential","+#.### V")
potential_monitor.setFont(QtGui.QFont("Courier", 32))
current_monitor, current_monitor_box = make_groupbox_indicator("Measured current","+#.### mA")
current_monitor.setFont(QtGui.QFont("Courier", 32))
potential_current_display_frame = QtGui.QHBoxLayout()
potential_current_display_frame.setSpacing(1)
potential_current_display_frame.setContentsMargins(0,0,0,0)
potential_current_display_frame.addWidget(potential_monitor_box)
potential_current_display_frame.addWidget(current_monitor_box)

mode_display_frame = QtGui.QHBoxLayout()
mode_display_frame.setSpacing(1)
mode_display_frame.setContentsMargins(0,0,0,5)
cell_status_monitor, cell_status_monitor_box = make_groupbox_indicator("Cell status","        ")
control_mode_monitor, control_mode_monitor_box = make_groupbox_indicator("Control mode","             ")
current_range_monitor, current_range_monitor_box = make_groupbox_indicator("Current range","     ")
mode_display_frame.addWidget(cell_status_monitor_box)
mode_display_frame.addWidget(control_mode_monitor_box)
mode_display_frame.addWidget(current_range_monitor_box)
plot_frame = pg.PlotWidget()

display_plot_frame = QtGui.QVBoxLayout()
display_plot_frame.setSpacing(0)
display_plot_frame.setContentsMargins(0,0,0,0)
display_plot_frame.addLayout(potential_current_display_frame)
display_plot_frame.addLayout(mode_display_frame)
display_plot_frame.addWidget(plot_frame)

preview_cancel_vlayout = QtGui.QVBoxLayout(plot_frame)
preview_cancel_hlayout = QtGui.QHBoxLayout()
preview_cancel_vlayout.setAlignment(QtCore.Qt.AlignTop)
preview_cancel_vlayout.addLayout(preview_cancel_hlayout)
preview_cancel_hlayout.setAlignment(QtCore.Qt.AlignRight)
preview_cancel_button = QtGui.QPushButton("Back to live graph")
preview_cancel_button.clicked.connect(cv_preview_cancel)
preview_cancel_hlayout.addWidget(preview_cancel_button)
preview_cancel_button.hide()

tab_frame = QtGui.QTabWidget()
tab_frame.setFixedWidth(305)

tab_names = ["Hardware","CV","Charge/disch.","Rate testing"]
tabs = [add_my_tab(tab_frame, tab_name) for tab_name in tab_names]

hardware_vbox = QtGui.QVBoxLayout()
hardware_vbox.setAlignment(QtCore.Qt.AlignTop)

hardware_usb_box = QtGui.QGroupBox(title="USB Interface", flat=False)
format_box_for_parameter(hardware_usb_box)
hardware_usb_box_layout = QtGui.QVBoxLayout()
hardware_usb_box.setLayout(hardware_usb_box_layout)
hardware_usb_vid = make_label_entry(hardware_usb_box_layout, "USB Vendor ID")
hardware_usb_vid.setText(usb_vid)
hardware_usb_pid = make_label_entry(hardware_usb_box_layout, "USB Product ID")
hardware_usb_pid.setText(usb_pid)
hardware_usb_connectButton = QtGui.QPushButton("Connect")
hardware_usb_connectButton.clicked.connect(connect_usb)
hardware_usb_box_layout.addWidget(hardware_usb_connectButton)
hardware_usb_box_layout.setSpacing(6)
hardware_usb_box_layout.setContentsMargins(3,10,3,3)
hardware_vbox.addWidget(hardware_usb_box)

hardware_device_info_box = QtGui.QGroupBox(title="Device Information", flat=False)
format_box_for_parameter(hardware_device_info_box)
hardware_device_info_box_layout = QtGui.QVBoxLayout()
hardware_device_info_box.setLayout(hardware_device_info_box_layout)
hardware_device_manufacturer = make_label_entry(hardware_device_info_box_layout, "Manufacturer")
hardware_device_manufacturer.setReadOnly(True)
hardware_device_model = make_label_entry(hardware_device_info_box_layout, "Model")
hardware_device_model.setReadOnly(True)
hardware_device_serial = make_label_entry(hardware_device_info_box_layout, "Serial #")
hardware_device_serial.setReadOnly(True)
hardware_device_info_box_layout.setSpacing(6)
hardware_device_info_box_layout.setContentsMargins(3,10,3,3)
hardware_vbox.addWidget(hardware_device_info_box)

hardware_calibration_box = QtGui.QGroupBox(title="Calibration", flat=False)
format_box_for_parameter(hardware_calibration_box)
hardware_calibration_box_layout = QtGui.QVBoxLayout()
hardware_calibration_box.setLayout(hardware_calibration_box_layout)
hardware_calibration_dac_hlayout = QtGui.QHBoxLayout()
hardware_calibration_box_layout.addLayout(hardware_calibration_dac_hlayout)
hardware_calibration_dac_vlayout = QtGui.QVBoxLayout()
hardware_calibration_dac_hlayout.addLayout(hardware_calibration_dac_vlayout)
hardware_calibration_dac_offset = make_label_entry(hardware_calibration_dac_vlayout, "DAC Offset")
hardware_calibration_dac_gain = make_label_entry(hardware_calibration_dac_vlayout, "DAC Gain")
hardware_calibration_dac_calibrate_button = QtGui.QPushButton("Auto\nCalibrate")
hardware_calibration_dac_calibrate_button.setMaximumHeight(100)
hardware_calibration_dac_calibrate_button.clicked.connect(dac_calibrate)
hardware_calibration_dac_hlayout.addWidget(hardware_calibration_dac_calibrate_button)

hardware_calibration_offset_hlayout = QtGui.QHBoxLayout()
hardware_calibration_box_layout.addLayout(hardware_calibration_offset_hlayout)
hardware_calibration_offset_vlayout = QtGui.QVBoxLayout()
hardware_calibration_offset_hlayout.addLayout(hardware_calibration_offset_vlayout)
hardware_calibration_potential_offset = make_label_entry(hardware_calibration_offset_vlayout, "Pot. Offset")
hardware_calibration_potential_offset.editingFinished.connect(offset_changed_callback)
hardware_calibration_potential_offset.setText("0")
hardware_calibration_current_offset = make_label_entry(hardware_calibration_offset_vlayout, "Curr. Offset")
hardware_calibration_current_offset.editingFinished.connect(offset_changed_callback)
hardware_calibration_current_offset.setText("0")
hardware_calibration_adc_measure_button = QtGui.QPushButton("Auto\nZero")
hardware_calibration_adc_measure_button.setMaximumHeight(100)
hardware_calibration_adc_measure_button.clicked.connect(zero_offset)
hardware_calibration_offset_hlayout.addWidget(hardware_calibration_adc_measure_button)

hardware_calibration_shunt_resistor_layout = QtGui.QHBoxLayout()
hardware_calibration_box_layout.addLayout(hardware_calibration_shunt_resistor_layout)
hardware_calibration_shuntvalues = [make_label_entry(hardware_calibration_shunt_resistor_layout, "R%d"%i) for i in range(1,4)]
for i in range(0,3):
	hardware_calibration_shuntvalues[i].editingFinished.connect(shunt_calibration_changed_callback)
	hardware_calibration_shuntvalues[i].setText("%.4f"%shunt_calibration[i])

hardware_calibration_button_layout = QtGui.QHBoxLayout()
hardware_calibration_get_button = QtGui.QPushButton("Load from device")
hardware_calibration_get_button.clicked.connect(get_calibration)
hardware_calibration_button_layout.addWidget(hardware_calibration_get_button)
hardware_calibration_set_button = QtGui.QPushButton("Save to device")
hardware_calibration_set_button.clicked.connect(set_calibration)
hardware_calibration_button_layout.addWidget(hardware_calibration_set_button)

hardware_calibration_box_layout.addLayout(hardware_calibration_button_layout)
hardware_calibration_box_layout.setSpacing(6)
hardware_calibration_box_layout.setContentsMargins(3,10,3,3)
hardware_vbox.addWidget(hardware_calibration_box)

hardware_manual_control_box = QtGui.QGroupBox(title="Manual Control", flat=False)
format_box_for_parameter(hardware_manual_control_box)
hardware_manual_control_box_layout = QtGui.QVBoxLayout()
hardware_manual_control_box.setLayout(hardware_manual_control_box_layout)

hardware_manual_control_cell_layout = QtGui.QHBoxLayout()
hardware_manual_control_cell_layout.addWidget(QtGui.QLabel("Cell connection"))
hardware_manual_control_cell_on_button = QtGui.QPushButton("On")
hardware_manual_control_cell_on_button.clicked.connect(lambda: set_cell_status(True))
hardware_manual_control_cell_layout.addWidget(hardware_manual_control_cell_on_button)
hardware_manual_control_cell_off_button = QtGui.QPushButton("Off")
hardware_manual_control_cell_off_button.clicked.connect(lambda: set_cell_status(False))
hardware_manual_control_cell_layout.addWidget(hardware_manual_control_cell_off_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_cell_layout)

hardware_manual_control_mode_layout = QtGui.QHBoxLayout()
hardware_manual_control_mode_layout.addWidget(QtGui.QLabel("Mode"))
hardware_manual_control_mode_potentiostat_button = QtGui.QPushButton("Potentiostatic")
hardware_manual_control_mode_potentiostat_button.clicked.connect(lambda: set_control_mode(False))
hardware_manual_control_mode_layout.addWidget(hardware_manual_control_mode_potentiostat_button)
hardware_manual_control_mode_galvanostat_button = QtGui.QPushButton("Galvanostatic")
hardware_manual_control_mode_galvanostat_button.clicked.connect(lambda: set_control_mode(True))
hardware_manual_control_mode_layout.addWidget(hardware_manual_control_mode_galvanostat_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_mode_layout)

hardware_manual_control_range_layout = QtGui.QHBoxLayout()
hardware_manual_control_range_layout.addWidget(QtGui.QLabel("Current range"))
hardware_manual_control_range_dropdown = QtGui.QComboBox()
hardware_manual_control_range_dropdown.addItems(current_range_list)
hardware_manual_control_range_layout.addWidget(hardware_manual_control_range_dropdown)
hardware_manual_control_range_set_button = QtGui.QPushButton("Set")
hardware_manual_control_range_set_button.clicked.connect(set_current_range)
hardware_manual_control_range_layout.addWidget(hardware_manual_control_range_set_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_range_layout)

hardware_manual_control_output_layout = QtGui.QHBoxLayout()
hardware_manual_control_output_dropdown = QtGui.QComboBox()
hardware_manual_control_output_dropdown.addItems(units_list)
hardware_manual_control_output_layout.addWidget(hardware_manual_control_output_dropdown)
hardware_manual_control_output_entry = QtGui.QLineEdit()
hardware_manual_control_output_entry.returnPressed.connect(set_output_from_gui)
hardware_manual_control_output_layout.addWidget(hardware_manual_control_output_entry)
hardware_manual_control_output_set_button = QtGui.QPushButton("Set")
hardware_manual_control_output_set_button.clicked.connect(set_output_from_gui)
hardware_manual_control_output_layout.addWidget(hardware_manual_control_output_set_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_output_layout)

hardware_manual_control_box_layout.setSpacing(6)
hardware_manual_control_box_layout.setContentsMargins(3,10,3,3)

hardware_vbox.addWidget(hardware_manual_control_box)
hardware_vbox.setSpacing(6)
hardware_vbox.setContentsMargins(3,3,3,3)

tabs[0].setLayout(hardware_vbox)

cv_vbox = QtGui.QVBoxLayout()
cv_vbox.setAlignment(QtCore.Qt.AlignTop)

cv_params_box = QtGui.QGroupBox(title="Cyclic voltammetry parameters", flat=False)
format_box_for_parameter(cv_params_box)
cv_params_layout = QtGui.QVBoxLayout()
cv_params_box.setLayout(cv_params_layout)
cv_lbound_entry = make_label_entry(cv_params_layout, "Lower bound (V)")
cv_ubound_entry = make_label_entry(cv_params_layout, "Upper bound (V)")

cv_hbox = QtGui.QHBoxLayout()
cv_label = QtGui.QLabel(text="Start potential (V)")
cv_startpot_entry = QtGui.QLineEdit()
cv_get_button = QtGui.QPushButton("OCP")
myfont = QtGui.QFont()
myfont.setPointSize(8)
cv_get_button.setFont(myfont)
cv_get_button.setFixedWidth(32)
cv_get_button.clicked.connect(cv_get_ocp)

cv_hbox.addWidget(cv_label)
cv_hbox.addWidget(cv_startpot_entry)
cv_hbox.addWidget(cv_get_button)
cv_params_layout.addLayout(cv_hbox)

cv_stoppot_entry = make_label_entry(cv_params_layout, "Stop potential (V)")
cv_scanrate_entry = make_label_entry(cv_params_layout, "Scan rate (mV/s)")
cv_scanrate_entry.editingFinished.connect(cv_scanrate_changed_callback)
cv_numcycles_entry = make_label_entry(cv_params_layout, "Number of cycles")
cv_numsamples_entry = make_label_entry(cv_params_layout, "Samples to average")
cv_numsamples_entry.setText("1")

cv_params_layout.setSpacing(6)
cv_params_layout.setContentsMargins(3,10,3,3)
cv_vbox.addWidget(cv_params_box)

cv_range_box = QtGui.QGroupBox(title="Autoranging", flat=False)
format_box_for_parameter(cv_range_box)
cv_range_layout = QtGui.QVBoxLayout()
cv_range_box.setLayout(cv_range_layout)
cv_range_checkboxes = []
for current in current_range_list:
	checkbox = QtGui.QCheckBox(current)
	cv_range_checkboxes.append(checkbox)
	cv_range_layout.addWidget(checkbox)
	checkbox.setChecked(True)

cv_range_layout.setSpacing(6)
cv_range_layout.setContentsMargins(3,10,3,3)
cv_vbox.addWidget(cv_range_box)

cv_file_box = QtGui.QGroupBox(title="Output data filename", flat=False)
format_box_for_parameter(cv_file_box)
cv_file_layout = QtGui.QVBoxLayout()
cv_file_box.setLayout(cv_file_layout)
cv_file_choose_layout = QtGui.QHBoxLayout()
cv_file_choose_button = QtGui.QPushButton("Choose")
cv_file_choose_button.clicked.connect(cv_choose_file)
cv_file_choose_layout.addWidget(cv_file_choose_button)
cv_file_entry = QtGui.QLineEdit()
cv_file_choose_layout.addWidget(cv_file_entry)
cv_file_layout.addLayout(cv_file_choose_layout)
cv_file_layout.setSpacing(6)
cv_file_layout.setContentsMargins(3,10,3,3)
cv_vbox.addWidget(cv_file_box)

cv_preview_button = QtGui.QPushButton("Preview sweep")
cv_preview_button.clicked.connect(cv_preview)
cv_vbox.addWidget(cv_preview_button)
cv_start_button = QtGui.QPushButton("Start cyclic voltammetry")
cv_start_button.clicked.connect(cv_start)
cv_vbox.addWidget(cv_start_button)
cv_stop_button = QtGui.QPushButton("Stop cyclic voltammetry")
cv_stop_button.clicked.connect(cv_stop)
cv_vbox.addWidget(cv_stop_button)

cv_vbox.setSpacing(6)
cv_vbox.setContentsMargins(3,3,3,3)

tabs[1].setLayout(cv_vbox)

################"

cd_vbox = QtGui.QVBoxLayout()
cd_vbox.setAlignment(QtCore.Qt.AlignTop)

cd_params_box = QtGui.QGroupBox(title="Charge/discharge parameters", flat=False)
format_box_for_parameter(cd_params_box)
cd_params_layout = QtGui.QVBoxLayout()
cd_params_box.setLayout(cd_params_layout)
cd_lbound_entry = make_label_entry(cd_params_layout, "Lower bound (V)")
cd_ubound_entry = make_label_entry(cd_params_layout, "Upper bound (V)")
cd_chargecurrent_entry = make_label_entry(cd_params_layout, u"Charge current (µA)")
cd_dischargecurrent_entry = make_label_entry(cd_params_layout, u"Discharge current (µA)")
cd_numcycles_entry = make_label_entry(cd_params_layout, "Number of half cycles")
cd_numsamples_entry = make_label_entry(cd_params_layout, "Samples to average")
cd_numsamples_entry.setText("1")

cd_params_layout.setSpacing(6)
cd_params_layout.setContentsMargins(3,10,3,3)
cd_vbox.addWidget(cd_params_box)

cd_file_box = QtGui.QGroupBox(title="Output data filename", flat=False)
format_box_for_parameter(cd_file_box)
cd_file_layout = QtGui.QVBoxLayout()
cd_file_box.setLayout(cd_file_layout)
cd_file_choose_layout = QtGui.QHBoxLayout()
cd_file_choose_button = QtGui.QPushButton("Choose")
cd_file_choose_button.clicked.connect(cd_choose_file)
cd_file_choose_layout.addWidget(cd_file_choose_button)
cd_file_entry = QtGui.QLineEdit()
cd_file_choose_layout.addWidget(cd_file_entry)
cd_file_layout.addLayout(cd_file_choose_layout)
cd_file_layout.setSpacing(6)
cd_file_layout.setContentsMargins(3,10,3,3)
cd_vbox.addWidget(cd_file_box)

cd_start_button = QtGui.QPushButton("Start charge/discharge")
cd_start_button.clicked.connect(cd_start)
cd_vbox.addWidget(cd_start_button)
cd_stop_button = QtGui.QPushButton("Stop charge/discharge")
cd_stop_button.clicked.connect(cd_stop)
cd_vbox.addWidget(cd_stop_button)

cd_vbox.setSpacing(6)
cd_vbox.setContentsMargins(3,3,3,3)

cd_info_box = QtGui.QGroupBox(title="Information", flat=False)
format_box_for_parameter(cd_info_box)
cd_info_layout = QtGui.QVBoxLayout()
cd_info_box.setLayout(cd_info_layout)
cd_current_cycle_entry = make_label_entry(cd_info_layout, "Current half cycle")
cd_current_cycle_entry.setReadOnly(True)

cd_info_layout.setSpacing(6)
cd_info_layout.setContentsMargins(3,10,3,3)
cd_vbox.addWidget(cd_info_box)

tabs[2].setLayout(cd_vbox)

#########################

rate_vbox = QtGui.QVBoxLayout()
rate_vbox.setAlignment(QtCore.Qt.AlignTop)

rate_params_box = QtGui.QGroupBox(title="Rate testing parameters", flat=False)
format_box_for_parameter(rate_params_box)
rate_params_layout = QtGui.QVBoxLayout()
rate_params_box.setLayout(rate_params_layout)
rate_lbound_entry = make_label_entry(rate_params_layout, "Lower bound (V)")
rate_ubound_entry = make_label_entry(rate_params_layout, "Upper bound (V)")
rate_capacity_entry = make_label_entry(rate_params_layout, u"C (µAh)")
rate_crates_entry = make_label_entry(rate_params_layout, u"C-rates")
rate_crates_entry.setText("1, 2, 5, 10, 20, 50, 100")
rate_numcycles_entry = make_label_entry(rate_params_layout, u"Cycles per C-rate")

rate_params_layout.setSpacing(6)
rate_params_layout.setContentsMargins(3,10,3,3)
rate_vbox.addWidget(rate_params_box)

rate_file_box = QtGui.QGroupBox(title="Output data filename", flat=False)
format_box_for_parameter(rate_file_box)
rate_file_layout = QtGui.QVBoxLayout()
rate_file_box.setLayout(rate_file_layout)
rate_file_choose_layout = QtGui.QHBoxLayout()
rate_file_choose_button = QtGui.QPushButton("Choose")
rate_file_choose_button.clicked.connect(rate_choose_file)
rate_file_choose_layout.addWidget(rate_file_choose_button)
rate_file_entry = QtGui.QLineEdit()
rate_file_choose_layout.addWidget(rate_file_entry)
rate_file_layout.addLayout(rate_file_choose_layout)

rate_file_layout.setSpacing(6)
rate_file_layout.setContentsMargins(3,10,3,3)
rate_vbox.addWidget(rate_file_box)

rate_start_button = QtGui.QPushButton("Start Rate Test")
rate_start_button.clicked.connect(rate_start)
rate_vbox.addWidget(rate_start_button)
rate_stop_button = QtGui.QPushButton("Stop Rate Test")
rate_stop_button.clicked.connect(rate_stop)
rate_vbox.addWidget(rate_stop_button)

rate_vbox.setSpacing(6)
rate_vbox.setContentsMargins(3,3,3,3)

rate_info_box = QtGui.QGroupBox(title="Information", flat=False)
format_box_for_parameter(rate_info_box)
rate_info_layout = QtGui.QVBoxLayout()
rate_info_box.setLayout(rate_info_layout)
rate_current_crate_entry = make_label_entry(rate_info_layout, "Current C-rate")
rate_current_crate_entry.setReadOnly(True)

rate_info_layout.setSpacing(6)
rate_info_layout.setContentsMargins(3,10,3,3)
rate_vbox.addWidget(rate_info_box)

tabs[3].setLayout(rate_vbox)

#########################

hbox = QtGui.QHBoxLayout()
hbox.addLayout(display_plot_frame)
hbox.addWidget(tab_frame)

vbox = QtGui.QVBoxLayout()
statustext = QtGui.QPlainTextEdit()
statustext.setFixedHeight(100)
vbox.addLayout(hbox)
vbox.addWidget(statustext)

mainwidget = QtGui.QWidget()
win.setCentralWidget(mainwidget)
vbox.setContentsMargins(0,0,0,0)
mainwidget.setLayout(vbox)

def periodic_update():
	global state
	if state == States.Idle_Init:
		idle_init()
	elif state == States.Idle:
		read_potential_current()
		update_live_graph()
	elif state == States.Measuring_CV:
		cv_update()
	elif state == States.Measuring_CD:
		cd_update()
	elif state == States.Measuring_Rate:
		rate_update()
	elif state == States.Stationary_Graph:
		read_potential_current()

timer = QtCore.QTimer()
timer.timeout.connect(periodic_update)
timer.start(qt_timer_period)

log_message("Program started. Press the \"Connect\" button in the hardware tab to connect to the USB interface.")

win.show()

sys.exit(app.exec_())
