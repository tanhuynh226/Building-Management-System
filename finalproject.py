import RPi.GPIO as GPIO
import time
import Freenove_DHT as DHT
import threading
import requests
from datetime import date
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

# Access humidity from CMIS
API_KEY = "REDACTED"
today = date.today()
date = today.strftime("%Y-%m-%d")
response = requests.get('http://et.water.ca.gov/api/data?appKey=' + API_KEY + '&targets=75&startDate=' + date + '&endDate=' + date + '&dataItems=hly-rel-hum')
response = response.json()
humidity = response['Data']['Providers'][0]['Records'][0]['HlyRelHum']['Value']
#humidity = 90 # I enable hardcoding of humidity whenever CMIS Web API is down

GPIO.setwarnings(False) # disable warnings

greenLEDPin = 29       # define led pins
redLEDPin = 37
blueLEDPin = 36
motionSensorPin = 11    # define motion sensor pin
DHTPin = 7              # define the pin of DHT11
decreaseTempBTN = 22    # define button pins
increaseTempBTN = 12
securityBTN = 32

# Global Variables
currTemp = []
weather_index = 0
userTemp = 72
doorWindowStatus = 1
doorWindowStatusChanged = 0
hvacStatus = 0
hvacStatusPrev = 0
hvacStatusChanged = 0
lightStatus = 0
start = 0
hvacOldState = 0
energy = 0
cost = 0


def setup():
    GPIO.setmode(GPIO.BOARD)        # use PHYSICAL GPIO Numbering
    GPIO.setup(greenLEDPin, GPIO.OUT)    # set green LED to OUTPUT mode
    GPIO.setup(redLEDPin, GPIO.OUT)    # set red LED to OUTPUT mode
    GPIO.setup(blueLEDPin, GPIO.OUT)    # set blue LED to OUTPUT mode
    GPIO.setup(motionSensorPin, GPIO.IN)  # set sensorPin to INPUT mode
    GPIO.setup(decreaseTempBTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # set user temperatures to INPUT mode
    GPIO.setup(increaseTempBTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(securityBTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)      # set door/window open/close button to INPUT mode
    

def loop():
    mcp.output(3,1)     # turn on LCD backlight
    lcd.begin(16,2)     # set number of LCD lines and columns
    
    ambient_light_thread = threading.Thread(target=ambient_light_control) # initialize threads
    room_temperature_thread = threading.Thread(target=room_temperature)
    lcd_display_thread = threading.Thread(target=lcd_display)
    
    ambient_light_thread.daemon = True     # enable daemon
    room_temperature_thread.daemon = True
    lcd_display_thread.daemon = True
        
    ambient_light_thread.start()      # start threading
    room_temperature_thread.start()      
    lcd_display_thread.start()
    
    while True:
        GPIO.add_event_detect(decreaseTempBTN, GPIO.FALLING, callback=user_temperature, bouncetime=1000) # bounce time set so that it doesn't accidentally detect the same input twice
        GPIO.add_event_detect(increaseTempBTN, GPIO.FALLING, callback=user_temperature, bouncetime=1000)
        GPIO.add_event_detect(securityBTN, GPIO.FALLING, callback=security, bouncetime=5000)
        time.sleep(1e6)
        
def ambient_light_control():
    global lightStatus
    while True:
        GPIO.output(greenLEDPin,GPIO.LOW)
        lightStatus = 0
        if GPIO.input(motionSensorPin) == GPIO.HIGH:
            begin = time.time()
            stop = 10 + begin
            while begin <= stop:
                GPIO.output(greenLEDPin,GPIO.HIGH) # turn on led
                lightStatus = 1
                time.sleep(1)
                if GPIO.input(motionSensorPin) == GPIO.HIGH: # extend time for another 10 seconds as long as there is motion
                    stop = stop + 10
                begin = time.time()
        time.sleep(1)

def room_temperature():
    global weather_index, userTemp, hvacStatus, hvacStatusPrev, hvacStatusChanged, hvacOldState, start
    while True:
        dht = DHT.DHT(DHTPin)   #create a DHT class object
        for i in range(0,15):            
            chk = dht.readDHT11()     #read DHT11 and get a return value. Then determine whether data read is normal according to the return value.
            if (chk is dht.DHTLIB_OK):      #read DHT11 and get a return value. Then determine whether data read is normal according to the return value.
                break
            time.sleep(0.1)
        currTemp.append(dht.temperature)
        if len(currTemp) >= 3: # Calculate weather index using the average temperature from the last three readings
            averageTemp = ((currTemp[len(currTemp)-1] + currTemp[len(currTemp)-2] + currTemp[len(currTemp)-3]) / 3) * (9/5) + 32 # Calculate average and convert to fahrenheit
            weather_index = round(averageTemp + 0.05 * float(humidity)) # Weather index equation
            #print("Weather index: " + str(weather_index))
        
        hvacStatusPrev = hvacStatus
        # Check whether the HVAC needs to be off or on AC/Heat mode
        if userTemp - 3 >= weather_index and doorWindowStatus == 1 and len(currTemp) >= 3:
            if hvacStatus != 2: # start timer if heater is just turning on
                start = time.time()
            GPIO.output(redLEDPin,GPIO.HIGH)
            GPIO.output(blueLEDPin,GPIO.LOW)
            hvacStatus = 2
            hvacOldState = hvacStatus
        elif userTemp + 3 <= weather_index and doorWindowStatus == 1 and len(currTemp) >= 3:
            if hvacStatus != 1: # start timer if AC is just turning on
                start = time.time()
            GPIO.output(blueLEDPin,GPIO.HIGH)
            GPIO.output(redLEDPin,GPIO.LOW)
            hvacStatus = 1
            hvacOldState = hvacStatus
        else:
            GPIO.output(redLEDPin,GPIO.LOW)
            GPIO.output(blueLEDPin,GPIO.LOW)
            hvacStatus = 0
        
        # Signal that HVAC status is different from before for LCD display to update
        if hvacStatusPrev != hvacStatus:
            hvacStatusChanged = 1
            
        time.sleep(1)
        
def lcd_display():
    global userTemp, currTemp, hvacStatus, hvacStatusPrev, hvacStatusChanged, lightStatus, doorWindowStatus, doorWindowStatusChanged, start
    while(True):         
        lcd.clear()         # clear LCD
        lcd.setCursor(0,0)  # set cursor position
        
        # Print new door/window status
        if doorWindowStatusChanged == 1:
            if doorWindowStatus == 0:
                lcd.message('DOOR/WINDOW OPEN\n')
                lcd.message('  HVAC HALTED')
                hvacStatusChanged = 1
            elif doorWindowStatus == 1:
                lcd.message('DOOR/WINDOW \nCLOSED')
            doorWindowStatusChanged = 0
            time.sleep(3) # Displays for 3 seconds as specified by assignment requirements
        # Print new HVAC status
        elif hvacStatusChanged == 1 and len(currTemp) >= 3:
            if hvacStatus == 2:
                lcd.message('   HVAC HEAT')
            elif hvacStatus == 1:
                lcd.message('   HVAC AC')
            elif hvacStatus == 0:
                lcd.message('   HVAC OFF')
            seconds = time.time() - start
            calculate_energy_costs(seconds)
            hvacStatusChanged = 0
            time.sleep(3) # Displays for 3 seconds as specified by assignment requirements
        else:
            # Print temperature and door status to LCD
            lcd.message(str(weather_index) + '/' + str(userTemp))
            if doorWindowStatus == 0:
                lcd.setCursor(10,0)
                lcd.message('D:OPEN\n')
            elif doorWindowStatus == 1:
                lcd.setCursor(10,0)
                lcd.message('D:SAFE\n')
            
            # Print HVAC status to LCd
            if hvacStatus == 0:
                lcd.message('H:OFF')
            elif hvacStatus == 1:
                lcd.message('H:COOL')
            elif hvacStatus == 2:
                lcd.message('H:HEAT')
                
            lcd.setCursor(11,1)
            # Print light status to LCD
            if lightStatus == 0:
                lcd.message('L:OFF')
            elif lightStatus == 1:
                lcd.message('L:ON')
            time.sleep(2) # Default display updating every 2 seconds
    
def calculate_energy_costs(seconds):
    global hvacOldState, hvacStatus, energy, cost
    hours = seconds / 3600 # convert seconds to hours
    if hvacStatus == 0: # can check for HVAC turning off since HVAC never goes straight from cool to heat or heat to cool
        if hvacOldState == 1: # cool
            energy += (18000 * hours) / 1000
            cost += ((18000 * hours) / 1000) * 0.5
        elif hvacOldState == 2: # heat
            energy += (36000 * hours) / 1000
            cost += ((36000 * hours) / 1000) * 0.5
        print("energy: " + "{:.2f}".format(energy) + "KWh , cost: $" + "{:.2f}".format(cost)) # round to 2 decimals when printing
    
       
def security(pin):
    global doorWindowStatus, hvacStatus, doorWindowStatusChanged
    if doorWindowStatus == 0: # Open window becomes closed
        doorWindowStatus = 1
        doorWindowStatusChanged = 1
    elif doorWindowStatus == 1: # Closed window becomes open
        doorWindowStatus = 0
        hvacStatus = 0
        doorWindowStatusChanged = 1
    time.sleep(3)
        
def user_temperature(pin):
    global userTemp
    if pin == decreaseTempBTN:
        userTemp -= 1
    elif pin == increaseTempBTN:
        userTemp += 1
        
    #print("Set temperature: " + str(userTemp))
    time.sleep(1)
    
PCF8574_address = 0x27  # I2C address of the PCF8574 chip.
PCF8574A_address = 0x3F  # I2C address of the PCF8574A chip.
# Create PCF8574 GPIO adapter.
try:
    mcp = PCF8574_GPIO(PCF8574_address)
except:
    try:
        mcp = PCF8574_GPIO(PCF8574A_address)
    except:
        print ('I2C Address Error !')
        exit(1)
# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)
        
def destroy():
    GPIO.cleanup()                     # Release GPIO resource

if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    setup()
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        destroy()


