# AXOBOTL Python Code
# Extreme Axolotls Robotics team 4028X for 2023-2024 VEX IQ Full Volume Challenge
#from multiprocessing import Value
from vex import *


# Use PID and other techniques to be an awesome drive controller
class AxolotlDriver:
    def __init__(self, lm, rm, inertial, wheelTravel:vexnumber=300, trackWidth:vexnumber=320, wheelBase:vexnumber=320,
                    units=DistanceUnits.MM, externalGearRatio=1.0):
        if (not (isinstance(lm, Motor) or isinstance(lm, MotorGroup)) or
            not (isinstance(rm, Motor) or isinstance(rm, MotorGroup))):
            raise TypeError('Must pass two motors or motor groups')
        self.lm = lm
        self.rm = rm
        self.inertial: Inertial = inertial
    
    # Mimic the functions of DriveTrain: drive straight
    def drive(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        pass

    def getMotorPosition(self):
        # This is based on the theory that the average of the two motor
        # positions is more accurate than any single motor position in case
        # of wheel slippage on one side or the other?
        return (self.lm.position() + self.rm.position()) / 2

    # This is based on the VEX IQ Blocks implementation published by Caution Tape
    # "How to make VEX IQ robot drive straight with VEXcode blocks"
    # https://www.cautiontape.ca/vex-iq-robot-drive-straight-with-vexcode-blocks/
    #
    # TODO: Calculate distance in other units, esp. MM and IN - do that using
    # knowledge of the wheel diameter. Or possibly even better, a one-time manual
    # calibration program that computes the bot's motor turns per inch and
    # prints that to screen so we can use that in this code.
    #
    # The bigger the kp value is, the more aggressively the bot will curve to
    # try to regain the correct heading. But making it too aggressive will cause
    # the bot to wiggle left and right too much. Find a nice value that gently
    # brings the bot back to the target heading when it drifts.
    def driveStraight(self, targetMotorDegrees, headingDegrees, velocity, kp):
        # This is not PID - it's just P: Proportional Control
        if kp <= 0.0:
            raise ValueError("Kp should be larger than 0.0")
        error = 0
        tweak = 0
        # Fine tune Kp based on robot design and speed
        self.lm.set_position(0, RotationUnits.DEG)
        self.rm.set_position(0, RotationUnits.DEG)
        self.lm.set_velocity(0)
        self.rm.set_velocity(0)
        self.lm.spin(DirectionType.FORWARD)
        self.rm.spin(DirectionType.FORWARD)

        # Uses motor position as a sort of odemetry - but it might not be super accurate
        # Test motor turns depending on whether going forward or reverse (negative velocity)
        while ((velocity > 0 and self.getMotorPosition() < targetMotorDegrees) or
               (velocity < 0 and self.getMotorPosition() > targetMotorDegrees)):
            # How far off from where we need to be?
            error = headingDegrees - self.inertial.heading()
            # How much should we tweak our velocity to rotate the bot towards the
            # correct heading?
            # For small errors, the tweak is very small. But for large errors,
            # the tweak is very large. Not lineraly
            tweak = error * kp
            # Bring the bot back on course by tweaking the velocity of the
            # two motors so the drivetrain can turn towards the target heading
            self.lm.set_velocity(velocity - tweak, VelocityUnits.PERCENT)
            self.rm.set_velocity(velocity + tweak, VelocityUnits.PERCENT)
            wait(20, TimeUnits.MSEC)

        # Done, mission accomplished?
        self.lm.stop()
        self.rm.stop()

class Bot:
    MODES = ["AUTO_RED", "GOAL_2", "GOAL_1", "GOAL_3", "CURVE"]
    MODE_COLORS = [Color.RED, Color.YELLOW_GREEN, Color.WHITE, Color.PURPLE, Color.YELLOW]
    MODE_PEN_COLORS = [Color.WHITE, Color.BLACK, Color.BLACK, Color.WHITE, Color.BLACK]

    def __init__(self):
        self.isAutoRunning = False
        self.modeNumber = 3
        self.isCalibrated = False
        self.cancelCalibration = False
        self.screenColor = Color.BLACK
        self.penColor = Color.WHITE

    def setup(self):
        self.brain = Brain()
        self.inertial = Inertial()
        self.setupPortMappings()
        self.setupDrive()
        self.setupAutoDriveTrain()

    def setupPortMappings(self):
        self.motorLeft = Motor(Ports.PORT1,1,True)
        self.motorRight = Motor(Ports.PORT6,1, False)
        self.driveTrain = None  # Default is MANUAL mode, no driveTrain

    def fillScreen(self, screenColor, penColor):
        self.screenColor = screenColor
        self.penColor = penColor
        self.brain.screen.clear_screen()
        self.brain.screen.set_fill_color(screenColor)
        self.brain.screen.set_pen_color(screenColor)
        self.brain.screen.draw_rectangle(0, 0, 170, 100, screenColor)
        self.brain.screen.set_pen_color(penColor)
        self.brain.screen.set_font(FontType.MONO20)
        self.brain.screen.set_cursor(1, 1)
        
    def print(self, message):
        penColor = Bot.MODE_PEN_COLORS[self.modeNumber]
        self.brain.screen.set_fill_color(self.screenColor)
        self.brain.screen.set_pen_color(self.penColor)
        self.brain.screen.print(message)
        self.brain.screen.new_line()

    def setupDrive(self):
        self.motorLeft.set_velocity(0, PERCENT)
        self.motorLeft.set_max_torque(100, PERCENT)
        self.motorLeft.spin(REVERSE)
        self.motorRight.set_velocity(0, PERCENT)
        self.motorRight.set_max_torque(100, PERCENT)
        self.motorRight.spin(REVERSE)

    def stopAll(self):
        if self.driveTrain:
            self.driveTrain.stop(BRAKE)

    def setupAutoDriveTrain(self, calibrate=True):
        # Use DriveTrain in autonomous. Easier to do turns.
        # Last updated on Nov 14, 2023:
        # Track width: 7-7/8 inches (7.875)
        # Wheel base : 6-1/2 inches (6.5)

        # NEW VALUES for the Test Bot (orange and pink) March 25, 2024:
        # Trackwidth = 18.5 cm
        # Wheelbase = 10-1/16 cm ()

        if not self.driveTrain:
            self.driveTrain = DriveTrain(self.motorLeft,
                                            self.motorRight,
                                            wheelTravel=200,
                                            trackWidth=185,    # Old: 200.025,
                                            wheelBase=100.625, # Old: 165.1,
                                            units=DistanceUnits.MM,
                                            externalGearRatio=2)  # TODO: Is this correct?
            if calibrate:
                return self.calibrate()
            return True

    def calibrate(self):
        self.print("Calibrating...")
        self.inertial.calibrate()
        countdown = 3000/50  
        while (self.inertial.is_calibrating()
                and countdown > 0
                and not self.cancelCalibration):
            wait(50, MSEC)
            countdown = countdown - 1
        if self.cancelCalibration:
            self.print("Cancelled Calibration!")
            return False
        elif countdown > 0 and not self.inertial.is_calibrating():
            self.print("Calibrated")
            self.brain.play_sound(SoundType.TADA)
            self.isCalibrated = True
            return True
        else:
            self.stopAll()
            self.print("FAILED Calibration")
            self.brain.play_sound(SoundType.POWER_DOWN)
            return False

    def runSmartDriveTest(self):
        pass

    def runPidDriveTest(self):
        driver = AxolotlDriver(self.motorLeft, self.motorRight, self.inertial)
        driver.driveStraight(3*360, 0, 50, 0.035)

    def run(self):
        self.setup()
        self.fillScreen(Color.BLUE_VIOLET, Color.WHITE)
        self.print("=====")
        self.print("4028X")
        self.print("Extreme")
        self.print("Axolotls!")
        self.print("=====")
        self.runPidDriveTest()

        
# Where it all begins!    
bot = Bot()
bot.run()