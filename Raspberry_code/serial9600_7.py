import serial #pyserial 3.5
import serial.tools.list_ports
import time
import mysql.connector #mysql-connector-python 8.2.0
from datetime import datetime
import sys
import requests

#################################################
# Script to read serial data from Arduino Indoor
# Use arduino_serial_number to find correct USB port to read
# Insert data into local MYSQL Database
# Commented section: send data to server requesting php page
# Compatible with ws_slave_1.7 version
#################################################

time.sleep(1)
#connect to mariadb database
mydb = mysql.connector.connect(
  host="127.0.0.1",
  user="",
  password="",
  database=""
)
mycursor = mydb.cursor()

arduino_serial_number = '8593732313035181F030'
errorCount = 0
insertData = False
rowPyPrevious = ""

#select port
def find_arduino(serial_number):
    for pinfo in serial.tools.list_ports.comports():
        if pinfo.serial_number == serial_number:
            return pinfo.device
    raise IOError("Could not find an arduino - is it plugged in?")
port = find_arduino(serial_number=arduino_serial_number)
#begin serial
ser = serial.Serial(port, 9600, timeout=3)
ser.close()
ser.open()

## PREOCESSING
while True:
	radio_link = ser.readline().decode('utf-8',errors='ignore').rstrip()#cerco testa del treno dati
	print(radio_link)
	sys.stdout.flush()
	if radio_link == "%1": #inizio treno dati ws principale con %1
		line = ser.readline().decode().rstrip()
		single = line.split()
		single[0:2] = [' '.join(single[0:2])] #unisce con spazio in mezzo date e time e crea datetime
		
		#check se inserire i dati
		if single[0] != rowPyPrevious and single[0] != "1970-1-1 0:0:0":
			print(single)
			sys.stdout.flush() #for update journalmd
			rowPyPrevious = single[0]
			insertData = True

		#DATI DATABASE
		if insertData:
			insertData = False
			#nowdatetime = datetime.today().strftime('%Y-%m-%d %H:%M:%S')
			ws_datetime = single[0]
			tp = float(single[1])
			tpdht = float(single[2])
			ur = float(single[4])
			slpressure = float(single[7])
			wind = float(single[8])
			#DERIVATE
			#dewpoint - wetbulb
			if not ur or (not tp and not tpdht):
				dewpoint = None
				wetbulb = None
			elif not tp:
				dewpoint = pow(ur/100,0.125)*(112+(0.9*tpdht)) + (0.1*tpdht) - 112
				wetbulb = tpdht * (0.45 + 0.006 * ur * pow(slpressure/106000, 0.5))
			else:
				dewpoint = pow(ur/100,0.125)*(112+(0.9*tp)) + (0.1*tp) - 112
				wetbulb = tp * (0.45 + 0.006 * ur * pow(slpressure/106000, 0.5))
			#heatindex
			if not ur or not tp:
				heatindex = None
			elif tp>27 and ur>40:
				heatindexF = -42.379 + 2.04901523*(1.8*tp + 32) + 10.14333127*ur - 0.22475541*(1.8*tp + 32)*ur - 0.00683783*pow(1.8*tp + 32,2) - 0.05481717*pow(ur,2) + 0.00122874*pow(1.8*tp + 32,2)*ur + 0.00085282*(1.8*tp + 32)*pow(ur,2) - 0.00000199*pow(tp*1.8 + 32,2)*pow(ur,2)
				heatindex = heatindexF/1.8 -32/1.8
			else:
				heatindex = tp
			#windchill
			if not tp:
				windchill = None
			elif wind >= 1.3:
				windchill = (13.12 + 0.6215 * tp) - (11.37 * pow(wind*3.6, 0.16)) + (0.3965 * tp * pow(wind*3.6, 0.16))
			else:
				windchill = tp

			#import in meteo.dataday
			sql = "INSERT INTO dataday(date, tp, tpdht, tp2, ur, ur2, pressione, pressionelivellodelmare, wind, windAng, mmPioggia, rainrate, nData, tpmax, tpmin, urmax, urmin, windmax, rainratemax, wetbulb, dewpoint, heatindex, windchill) values (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
			val = single + [wetbulb, dewpoint, heatindex, windchill]
			try:
				mycursor.execute(sql, val)
				mydb.commit()
			except mysql.connector.errors.IntegrityError:
				print("error inserting dataday")

			#import in meteo.control
			line_control = ser.readline().decode().rstrip()
			single_control = line_control.split()
			
			sql2 = "INSERT INTO control(date, volt, bmp_status, sd_status, sd_tx_status, reset_tx, ack_tx) values(%s, %s, %s, %s, %s, %s, %s)"
			val2 = [ws_datetime] + single_control
			try:
				mycursor.execute(sql2, val2)
				mydb.commit()
			except mysql.connector.errors.IntegrityError:
				print("error inserting control")
			
			# Send to server
			'''url = "http://212.227.60.35/php-obtain/dataday.php?p="
			url = url + "date," + str(ws_datetime).replace(":", "%3A").replace(" ", "+")
			url = url + ",tp," + str(tp) + ",tpdht," + str(tpdht) + ",tp2," + str(tp2) + ",ur," + str(ur) + ",ur2," + str(ur2) + ",dewpoint," + str(dewpoint) + ",heatindex," + str(heatindex)
			url = url + ",pressione," + str(pressure) + ",pressionelivellodelmare," + str(slpressure) + ",wind," + str(wind) + ",windAng," + str(windang) + ",windchill," + str(windchill)
			url = url + ",mmPioggia," + str(mmpioggia) + ",rainrate," + str(rainrate) + ",wetbulb," + str(wetbulb) + ",nData," + str(ndata)
			print(url)
			r = requests.get(url)
			if r.status_code != 200:
				print("Received " + str(r.status_code) + " " + str(r.text))
				sys.stdout.flush()
			print(mycursor.rowcount, "data inserted.")'''

			# Send to server
			'''url2 = "http://212.227.60.35/php-obtain/control.php?p="
			url2 = url2 + "date," + str(ws_datetime).replace(":", "%3A").replace(" ", "+")
			url2 = url2 + ",bmp_status," + str(bmp180_status) + ",sd_status," + str(sd_status) + ",sd_tx_status," + str(sd_tx_status) + ",volt," + str(volt) + ",reset_tx," + str(reset_tx) + ",ack_tx," + str(ack_tx)
			print(url2)
			r2 = requests.get(url2)
			if r2.status_code != 200:
				print("Received " + str(r2.status_code) + " " + str(r2.text))
				sys.stdout.flush()
			print(mycursor.rowcount, "controls inserted.")'''
