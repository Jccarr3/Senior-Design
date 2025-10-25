
import sys


try:
    from zumo_2040_robot.extras.splash_loader import splash_loader
    splash_loader(
        default_program = None, # "my_program.py"
        splash_delay_s = 6, # delay while waiting for a button
        run_file_delay_ms = 700 # extra delay to show the action
        )

except Exception as e:
    from zumo_2040_robot.motors import Motors
    Motors()   # turn off Motors ASAP
    exc = e    # enable access to original exception in REPL
    from zumo_2040_robot.rgb_leds import RGBLEDs
    RGBLEDs()  # turn off RGB LEDs
    from zumo_2040_robot.buzzer import Buzzer
    buzzer = Buzzer()

    from zumo_2040_robot.display import Display
    Display.show_exception(e)
    buzzer.play("O2c4")
    raise

finally:
    from zumo_2040_robot.motors import Motors
    Motors()   # turn off Motors ASAP
    from zumo_2040_robot.buzzer import Buzzer
    Buzzer()   # turn off Buzzer
    from zumo_2040_robot.rgb_leds import RGBLEDs
    RGBLEDs()  # turn off RGB LEDs

    # don't leave extra classes lying around
    del Motors, Buzzer, RGBLEDs, splash_loader

    # make the REPL friendlier, if you enter it the right way
    from zumo_2040_robot import robot
    import sys
