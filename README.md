# Radio Weather Station - GSM Weather Station
<p>Weather stazione with outdoor sensors, radio connected with an indoor lcd and database.</p>
<li>Arduino mega2560 collects data from external sensors and save data in a SD card.</li>
<li>Data sent via radio with a LoRa module to an indoor Arduino.</li>
<li>Outdoor Arduino mega2560 shows data on 1.8" TFT color screen.</li>
<li>Raspeberry Pi collects data from indoor Arduino, it stores data in a mariaDB database and hosts the <a href="https://meteocremolino.ddns.net">station site</a>.</li>
<li>Now data is also sent online by Arduino with GSM Module.</li>

## Changelog
<p>Originally the station was made with only two arduino, the indoor one was a server for a html page (see Webbino code version v1.3). That's why two arduino are still present in the project, indoor arduino can be removed. The LoRa module LLCC68 replaces radio module NRF24L01 + PA + LNA, for better reliability</p>
<p>Data stored from the sensor is the average of 60 measures over one minute, sampling frequency of 1Hz. A basic filtering is performed, based on the range of the instruments' datasheets.</p>

### January 2025
  <p>
  The site is no longer hosted on Raspberry Pi, which continues to save data in the database. The data is sent to a server with a POST request to a PHP page. A SIM800L GSM module and a pay-as-you-go SIM card are used, given the low amount of MB required (about 1-2 MB per day). The Arduino code remains valid, removing the initSIM800L(), resetSIM800(), and serverRequest() functions.
</p>

## Weather features
<p>
  <strong>Sensors:</strong>
<li>Thermohygrometer Sensirion SHT35 (I2C)</li>
<li>Barometer Bosch BMPT180 (I2C, indoor)</li>
<li>Thermohygrometer DHT22 (digital) (optional)</li>
<li>Thermometer ds18b20 (digital) (optional)</li>
<li>Anemometer DeAgostini (reed contact)</li>
<li>Wind vane DeAgostini (photodiode,voltage output)</li>
<li>Rain gauge DeAgostini (reed contact)</li>
</p>

<p>
  <strong>Variables measured:</strong>
<li>Temperature</li>
<li>Humidity</li>
<li>Atmospheric pressure</li>
<li>Wind Speed</li>
<li>Wind Direction</li>
<li>Precipitation in mm/m^2</li>
<li>Rain rate</li>
<li>DewPoint, WetBulb, HeatIndex, WindChill</li>
</p>

## Components
<p>
  <strong>Outdoor station components:</strong>
<li>Arduino Mega2560</li>
<li>SD breakout board 3.3V</li>
<li>RTC DS3231 - commands sampling at 1Hz and attaches a timestamp to the data</li>
<li>Weather sensors listed above connected via cable to the Arduino</li>
<li>12V 12Ah lead-acid battery</li>
<li>Solar charge controller</li>
<li>Solar panel 50W</li>
<li>Waterproof case</li>
<li>12V DC -> 5V DC Step down - LM2596</li>
<li>I2C extender (optional)</li>
<li>MCP1700-3302E/TO-92 LDO to power LoRa module</li>
</p>

<p>
  <strong>Indoor station components:</strong>
<li>Arduino Mega2560</li>
<li>Screen 1.8" TFT</li>
<li>Barometer BMP180</li>
<li>LoRa receiver</li>
<li>USB cable</li>
<li>Raspberry Pi 3B</li>
<li>MCP1700-3302E/TO-92 LDO to power LoRa module</li>
<li>SIM800L - powered by external 5V power supply </li>
</p>
