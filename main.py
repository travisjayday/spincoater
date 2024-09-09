import sys
sys.path.append('.')
from st77xx2 import *
import gc

THROT = 15

# 6V @ 4A
throttle_to_rpm = {
    4: 1238, 
    5: 1670,
    6: 2014,
    7: 2388,
    8: 2622,
    9: 2938,
    10: 3238,
    11: 3431,
    12: 3678,
    13: 3845,
    14: 4058,
    15: 4150,
    16: 4387,
    17: 4541, 
    18: 4691, 
    19: 4825, 
    20: 4965, 
    21: 5121,
    22: 5233, 
    23: 5370, 
    24: 5465, 
    25: 5587, 
    26: 5677,
    27: 5784,
    28: 5888,
    29: 5973,
    30: 6088,
    31: 6163
    

    #100: 10000,
}

# Extract the sorted throttle values for interpolation
sorted_throttles = sorted(throttle_to_rpm.keys())
rpm_table = [0] * 101  

for i in range(101):
    for j in range(len(sorted_throttles) - 1):
        if sorted_throttles[j] <= i <= sorted_throttles[j + 1]:
            rpm1 = throttle_to_rpm[sorted_throttles[j]]
            rpm2 = throttle_to_rpm[sorted_throttles[j + 1]]
            throttle1 = sorted_throttles[j]
            throttle2 = sorted_throttles[j + 1]
            
            rpm_table[i] = int(rpm1 + (rpm2 - rpm1) * (i - throttle1) / (throttle2 - throttle1))
            break


ESCPIN = machine.Pin(22)
esc = machine.PWM(ESCPIN)
esc.freq(50)

# throttle = 0 to 100
def set_throttle(throttle):
    global current_rpm 

    throttle = max(0, min(throttle, 100))

    # Update ESC duty cycle based on the throttle (ESC duty cycle typically expects nanoseconds)
    # Let's assume duty_min corresponds to 0% throttle and duty_max corresponds to 100% throttle
    duty_min = 1000000  # Example minimum duty in nanoseconds (1ms)
    duty_max = 2000000  # Example maximum duty in nanoseconds (2ms)
    
    # Calculate the duty cycle based on throttle value
    duty_ns = duty_min + (throttle / 100) * (duty_max - duty_min)
    esc.duty_ns(int(duty_ns))

    # update RPM 
    throttle_int = int(round(throttle))
    if throttle_int < 0 or throttle_int > 100:
        return
    current_rpm = rpm_table[throttle_int]

print("Arming ESC...")
set_throttle(0)  # Start with throttle at 0%

spi=machine.SPI(
    1,
    baudrate=100_000_000,
    polarity=0,
    phase=0,
    sck=machine.Pin(10,machine.Pin.OUT),
    mosi=machine.Pin(11,machine.Pin.OUT),
    miso=None#machine.Pin(12,machine.Pin.IN)
)

import lvgl as lv
import lv_utils
lv.init()
print("initted lv")
event_loop = lv_utils.event_loop()
print("initted event loop")

lcd=St7789(rot=1,res=(240,240),spi=spi,cs=9,dc=8,bl=13,rst=12,rp2_dma=None,factor=8)
lcd.set_backlight(30)
#lcd.clear(0xF800) 
print("Display init done")

# Screen dimensions
screen_width = 240  # Assuming a 240x240 screen
screen_height = 240

min_rpm = 1300
setting_max_rpm = min_rpm
setting_ramp_t = 60
setting_dur_t = 60
current_rpm = 0
program_running = False

def interpolate_rpm_to_throttle(rpm, throttle_to_rpm):
    # Get a sorted list of throttle positions and RPM values
    throttle_positions = sorted(throttle_to_rpm.keys())
    rpm_values = [throttle_to_rpm[t] for t in throttle_positions]
    
    # If the RPM is below or above the known range, return the min or max throttle
    if rpm <= rpm_values[0]:
        return throttle_positions[0]
    if rpm >= rpm_values[-1]:
        return throttle_positions[-1]
    
    # Find the two RPM values between which the given RPM lies
    for i in range(1, len(rpm_values)):
        if rpm_values[i-1] <= rpm <= rpm_values[i]:
            # Perform linear interpolation
            throttle_lower = throttle_positions[i-1]
            throttle_upper = throttle_positions[i]
            rpm_lower = rpm_values[i-1]
            rpm_upper = rpm_values[i]
            
            # Calculate the interpolated throttle
            throttle = throttle_lower + (rpm - rpm_lower) * (throttle_upper - throttle_lower) / (rpm_upper - rpm_lower)
            return throttle

    return 0.0 # If something goes wrong

idle_update_t = time.time() 

def idle_task():
    global idle_update_t
    #if time.time() - idle_update_t >= 0.05:
    ts = time.ticks_ms()
    update_screens()
    idle_update_t = time.time()

    if program_running is False:
        print("User aborted program")
        set_throttle(0)
        return False
    return True

def run_program():
    global current_rpm
    print(f"Running program: Max RPM: {setting_max_rpm} | Ramp up time: {setting_ramp_t} | Duration: {setting_dur_t}")
    target_throttle = interpolate_rpm_to_throttle(setting_max_rpm, throttle_to_rpm)
    gc.collect()
    print("Target throttle is " + str(target_throttle))
    min_throttle = 4
    throttle_range = target_throttle-min_throttle  # from 0 to target_throttle
    time_per_step = 100 # assume a small time step for smooth control (in ms)
    throttle_steps = int((1000 * setting_ramp_t) / time_per_step)
    
    print(f"Ramping up Throttle in {throttle_steps} steps")

    start_time = time.ticks_ms()
    for step in range(throttle_steps + 1):
        current_throttle = (step / throttle_steps) * throttle_range + min_throttle
        set_throttle(current_throttle)

        next_step_time = start_time + (step + 1) * time_per_step 
        while time.ticks_ms() < next_step_time:
            if not idle_task(): return

    # Max throttle
    print("Throttle reached Max")
    set_throttle(target_throttle)
    current_rpm = setting_max_rpm
    end_duration_time = time.ticks_ms() + setting_dur_t * 1000
    while time.ticks_ms() < end_duration_time:
        if not idle_task(): return
    
    # Get the start time for ramp down
    print("Ramping down throttle")
    start_time = time.ticks_ms()
    for step in range(throttle_steps, -1, -1):
        current_throttle = (step / throttle_steps) * throttle_range
        set_throttle(current_throttle)

        next_step_time = start_time + (throttle_steps - step + 1) * time_per_step
        while time.ticks_ms() < next_step_time:
            if not idle_task(): return

    current_rpm = 0
    print("Done")

red_btn= lv.style_t()
red_btn.init()

# Set the main background color of the button (using a solid color)
red_btn.set_bg_color(lv.color_hex(0xFF5733))
red_btn.set_bg_opa(lv.OPA.COVER)  # Make sure the color fully covers the button


update_methods = {}

# Use a larger font for the labels
large_font = lv.font_montserrat_20  # You can choose any suitable font size

def create_ok_btn(scr): 
    btn = lv.button(scr)
    btn.set_size(40, 40)
    btn.align(lv.ALIGN.TOP_RIGHT, 0, 0)
    oklabel = lv.label(btn)
    oklabel.set_text("OK")
    oklabel.set_style_text_font(large_font, 0)  # Set the large font for the label
    oklabel.align(lv.ALIGN.CENTER, 0, 0)

def create_menu_scr():

    scr = lv.obj()

    # Button properties
    num_buttons = 4
    button_height = 50  # Height of each button
    button_width = 220  # Increase width to accommodate larger text
    y_offset = 25
    spacing = (screen_height - (num_buttons * button_height)) // (num_buttons + 1)


    # Create and position the buttons
    buttons = []
    labels = ["Set Max RPM", "Set Ramp Time", "Set Spin Duration", "Start"]

    for i in range(num_buttons):
        btn = lv.button(scr)
        btn.set_size(button_width, button_height)

        if labels[i] == 'Start': 
            btn.add_style(red_btn, 0)
        
        # Calculate the vertical position for this button
        y_position = spacing + i * (button_height + spacing)
        
        # Align the button to the right edge
        btn.align(lv.ALIGN.CENTER, 0, y_offset + y_position - screen_height // 2)
        
        # Add a label to the button
        label = lv.label(btn)
        label.set_text(labels[i])
        label.set_style_text_font(large_font, 0)  # Set the large font for the label
        label.align(lv.ALIGN.CENTER, 0, 0)
    
        buttons.append(btn)
    
    return scr

# Create the second screen
def create_gauge_screen(rmin, rmax, label_text, get_val, label_postfix=""):
    scr = lv.obj()

    # Create the first scale with a line needle
    scale_line = lv.scale(scr)
    scale_line.set_size(150, 150)
    scale_line.set_mode(lv.scale.MODE.ROUND_INNER)
    scale_line.set_style_bg_opa(lv.OPA.COVER, 0)
    #scale_line.set_style_bg_color(lv.palette_lighten(lv.PALETTE.RED, 5), 0)
    scale_line.set_style_radius(lv.RADIUS_CIRCLE, 0)
    scale_line.set_style_clip_corner(True, 0)
    scale_line.align(lv.ALIGN.CENTER, lv.pct(2), 0)

    scale_line.set_label_show(True)
    scale_line.set_total_tick_count(21)
    scale_line.set_major_tick_every(5)

    scale_line.set_style_length(3, lv.PART.ITEMS)
    scale_line.set_style_length(8, lv.PART.INDICATOR)
    scale_line.set_range(rmin, rmax)

    scale_line.set_angle_range(270)
    scale_line.set_rotation(135)

    # Create a simple line needle (as a placeholder)
    needle_line = lv.line(scale_line)
    needle_line.set_style_line_width(5, lv.PART.MAIN)
    needle_line.set_style_line_rounded(True, lv.PART.MAIN)

    # Add a label to display the numeric RPM value
    label = lv.label(scr)
    label.set_text(label_text + str(get_val()) + label_postfix)
    label.set_style_text_font(lv.font_montserrat_24, 0)  # Set the large font for the label
    label.align(lv.ALIGN.CENTER, 0, 80)  # Position it below the gauge

    create_ok_btn(scr)

    # Manually set the needle position to a value, for example, 20
    def update():
        lv.scale.set_line_needle_value(scale_line, needle_line, rmax, get_val())
        label.set_text(label_text + str(get_val()) + label_postfix)

    update_methods[id(scr)] = update

    return scr

scr_menu = create_menu_scr()
scr_set_rpm = create_gauge_screen(min_rpm, 6000, "Max RPM: ", lambda: setting_max_rpm)
scr_set_ramp = create_gauge_screen(60, 180, "Ramp Time: ", lambda: setting_ramp_t, label_postfix="s")
scr_set_dur = create_gauge_screen(10, 180, "Spin Duration: ", lambda: setting_dur_t, label_postfix="s")
scr_set_program = create_gauge_screen(0, 10000, "RPM: ", lambda: current_rpm)

lv.screen_load(scr_menu)

print("Done")

gpioJoyUp = machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP)
gpioJoyDown = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)
gpioJoyLeft = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)
gpioJoyRight = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
gpioButtonA = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)
gpioButtonB = machine.Pin(17, machine.Pin.IN, machine.Pin.PULL_UP)
gpioButtonX = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
gpioButtonY = machine.Pin(21, machine.Pin.IN, machine.Pin.PULL_UP)

joystate = {}
tmp = False 

def on_button_press(pin):
    global setting_max_rpm, setting_ramp_t, setting_dur_t, program_running
    global tmp
    active = lv.screen_active() 
    if pin == gpioJoyUp: 
        print("up")
        if pin.value(): # Rising means user let go
            joystate['up'] = (False, time.time())
        else:
            if active == scr_set_rpm:
                setting_max_rpm += 100
            elif active == scr_set_ramp:
                setting_ramp_t += 1
            elif active == scr_set_dur:
                setting_dur_t += 5
            joystate['up'] = (True, time.time())
    elif pin == gpioJoyDown: 
        print("down")
        if pin.value(): # Rising means user let go
            joystate['down'] = (False, time.time())
        else:
            if active == scr_set_rpm:
                setting_max_rpm -= 100
            elif active == scr_set_ramp:
                setting_ramp_t -= 1
            elif active == scr_set_dur:
                setting_dur_t -= 5
            joystate['down'] = (True, time.time())
    elif pin == gpioJoyLeft: 
        print("left")
    elif pin == gpioJoyRight: 
        print("right")
    elif pin == gpioButtonA: 
        print("Button A")
        if lv.screen_active() == scr_menu:
            lv.screen_load(scr_set_rpm)
        elif active == scr_set_program:
            program_running = False
        else:
            lv.screen_load(scr_menu)
    elif pin == gpioButtonB: 
        print("Button B")
        if lv.screen_active() == scr_menu:
            lv.screen_load(scr_set_ramp)
        elif active == scr_set_program:
            program_running = False
        else:
            lv.screen_load(scr_menu)
    elif pin == gpioButtonX: 
        print("Button X")
        if lv.screen_active() == scr_menu:
            lv.screen_load(scr_set_dur)
        elif active == scr_set_program:
            program_running = False
        else:
            lv.screen_load(scr_menu)
    elif pin == gpioButtonY: 
        print("Button Y")
        '''if not tmp:
            tmp = True
            t = 4
            print("Setting throttle to", t)
            set_throttle(THROT)
        else:
            tmp = False
            print("Stopping")
            set_throttle(0)

        return'''
        if not program_running: 
            print("Running")
            program_running = True
        else:
            print("Stopping")
            program_running = False


gpioButtonA.irq(trigger=machine.Pin.IRQ_FALLING, handler=on_button_press)
gpioButtonB.irq(trigger=machine.Pin.IRQ_FALLING, handler=on_button_press)
gpioButtonX.irq(trigger=machine.Pin.IRQ_FALLING, handler=on_button_press)
gpioButtonY.irq(trigger=machine.Pin.IRQ_FALLING, handler=on_button_press)
gpioJoyUp.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=on_button_press)
gpioJoyDown.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=on_button_press)
gpioJoyLeft.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=on_button_press)
gpioJoyRight.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=on_button_press)

def update_screens():
    active_scr = lv.screen_active()
    if id(active_scr) in update_methods.keys():
        update_methods[id(active_scr)]()

def handle_input():
    global setting_max_rpm, setting_ramp_t, setting_dur_t
    active_scr = lv.screen_active()
    t = time.time()
    if active_scr == scr_set_rpm or active_scr == scr_set_ramp or active_scr == scr_set_dur:
        if joystate['up'][0] and t - joystate['up'][1] > 0.8:
            if active_scr == scr_set_rpm:
                setting_max_rpm += 300 
            elif active_scr == scr_set_ramp:
                setting_ramp_t += 2
            elif active_scr == scr_set_dur:
                setting_dur_t += 10
        if joystate['down'][0] and t - joystate['down'][1] > 0.8:
            if active_scr == scr_set_rpm:
                setting_max_rpm -= 300 
            elif active_scr == scr_set_ramp:
                setting_ramp_t -= 2
            elif active_scr == scr_set_dur:
                setting_dur_t -= 10

scr = lv.obj()

joystate['up'] = (False, 0)
joystate['down'] = (False, 0)

i = 0
try: 
    while True:

        if program_running:
            lv.screen_load(scr_set_program)
            run_program()
            program_running = False
            lv.screen_load(scr_menu)

        time.sleep(0.1)
        handle_input()
        update_screens()
        i += 1
        if i % 50 == 0:
            gc.collect()
            free_memory = gc.mem_free()
            print("Free memory: " + str(free_memory))
finally:
    gc.collect()
    event_loop.deinit()
    lcd.off()
    lcd = None
    event_loop = None
    spi.deinit()
    esc.deinit()
    gc.collect()
    #lv.task_handler()
    #time.sleep(0.05)