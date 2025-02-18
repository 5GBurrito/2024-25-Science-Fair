#! /usr/bin/python

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
from access_control import AccessControl
import face_recognition
import imutils
import pickle
import time
from time import sleep
import cv2
import smbus2 as smbus
import RPi.GPIO as GPIO

# I2C LCD setup
LCD_I2C_ADDRESS = 0x27
LCD_WIDTH = 16
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_ENABLE = 0b00000100
LCD_BACKLIGHT = 0x08
LCD_CMD = 0
LCD_CHR = 1
bus = smbus.SMBus(1)

# Solenoid Pin (GPIO 37) Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.OUT)

# LCD Functions
def lcd_write_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(LCD_I2C_ADDRESS, bits_high)
    lcd_toggle_enable(bits_high)
    bus.write_byte(LCD_I2C_ADDRESS, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    sleep(0.0005)
    bus.write_byte(LCD_I2C_ADDRESS, (bits | LCD_ENABLE))
    sleep(0.0005)
    bus.write_byte(LCD_I2C_ADDRESS, (bits & ~LCD_ENABLE))
    sleep(0.0005)

def lcd_init():
    lcd_write_byte(0x33, LCD_CMD)
    lcd_write_byte(0x32, LCD_CMD)
    lcd_write_byte(0x06, LCD_CMD)
    lcd_write_byte(0x0C, LCD_CMD)
    lcd_write_byte(0x28, LCD_CMD)
    lcd_write_byte(0x01, LCD_CMD)
    sleep(0.0005)

def lcd_display_string(message, line):
    lcd_write_byte(line, LCD_CMD)
    for char in message.ljust(LCD_WIDTH, " "):
        lcd_write_byte(ord(char), LCD_CHR)

# Keypad configuration
keypad_layout = [
    ['1', '2', '3', 'A'],
    ['4', '5', '6', 'B'],
    ['7', '8', '9', 'C'],
    ['*', '0', '#', 'D']
]
row_pins = [36, 38, 40, 16]  # Row pins
col_pins = [18, 33, 35, 26]

# Initialize AccessControl
access_control = AccessControl(row_pins, col_pins, keypad_layout)

# Define correct credentials
correct_keypad_code = "2319"
correct_card_id = 989317600971  # Replace with your correct card ID

# Initialize 'currentname' to trigger only when a new person is identified.
currentname = "unknown"
encodingsP = "encodings.pickle"

# Load the known faces and embeddings along with OpenCV's Haar
# cascade for face detection
print("[INFO] loading encodings + face detector...")
data = pickle.loads(open(encodingsP, "rb").read())

# Initialize the video stream and allow the camera sensor to warm up
vs = VideoStream(src=0, framerate=10).start()
time.sleep(2.0)

# Start the FPS counter
fps = FPS().start()

# Initialize LCD
lcd_init()

# Display initial message to look into the camera
lcd_display_string("Please look into", LCD_LINE_1)
lcd_display_string("the camera.", LCD_LINE_2)

# Loop over frames from the video file stream
while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=500)
    boxes = face_recognition.face_locations(frame)
    encodings = face_recognition.face_encodings(frame, boxes)
    names = []

    for encoding in encodings:
        matches = face_recognition.compare_faces(data["encodings"], encoding)
        name = "Unknown"

        if True in matches:
            matchedIdxs = [i for (i, b) in enumerate(matches) if b]
            counts = {}

            for i in matchedIdxs:
                name = data["names"][i]
                counts[name] = counts.get(name, 0) + 1

            name = max(counts, key=counts.get)

            if currentname != name:
                currentname = name
                print(f"[DEBUG] Identified: {currentname}, please input code")
                lcd_display_string(f"Hello, {name}!", LCD_LINE_1)
                lcd_display_string("Input passcode", LCD_LINE_2)

                if access_control.check_keypad_code(correct_keypad_code):
                    print("[DEBUG] Correct keypad code entered!")
                    lcd_display_string("Correct code", LCD_LINE_1)
                    lcd_display_string("Scan your ID", LCD_LINE_2)

                    if access_control.check_rfid_card(correct_card_id):
                        print("[DEBUG] Correct ID card scanned!")
                        lcd_display_string("ID card correct!", LCD_LINE_1)
                        lcd_display_string("Unlocking door.", LCD_LINE_2)

                        GPIO.setmode(GPIO.BOARD)
                        GPIO.setup(37, GPIO.OUT)
                        # Unlock solenoid and show message for 5 seconds
                        GPIO.output(37, GPIO.HIGH)  # Unlock solenoid
                        sleep(5)
                        GPIO.output(37, GPIO.LOW)  # Lock solenoid

                        lcd_display_string("Unlock successful", LCD_LINE_1)
                        lcd_display_string("Access granted.", LCD_LINE_2)
                        sleep(5)  # Wait for a while before next interaction
                        lcd_display_string("Please look into", LCD_LINE_1)
                        lcd_display_string("the camera.", LCD_LINE_2)

                        name = "Larry"
                        currentname = "Detravius DeMarcus-DingleBob Jones III"
                    else:
                        print("[DEBUG] Incorrect ID card.")
                        lcd_display_string("Incorrect ID card", LCD_LINE_1)
                        lcd_display_string("Please try again", LCD_LINE_2)
                        name = "Larry"
                        currentname = "Detravius DeMarcus-DingleBob Jones III"

                else:
                    print("[DEBUG] Incorrect keypad code.")
                    lcd_display_string("Incorrect keypad", LCD_LINE_1)
                    lcd_display_string("Please try again", LCD_LINE_2)
                    name = "Larry"
                    currentname = "Detravius DeMarcus-DingleBob Jones III"

        names.append(name)

    for ((top, right, bottom, left), name) in zip(boxes, names):
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 225), 2)
        y = top - 15 if top - 15 > 15 else top + 15
        cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)

    cv2.imshow("Facial Recognition is Running", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

    fps.update()

fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

cv2.destroyAllWindows()
vs.stop()

# Cleanup GPIO
GPIO.cleanup()