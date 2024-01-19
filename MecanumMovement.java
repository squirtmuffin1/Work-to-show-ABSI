package org.firstinspires.ftc.teamcode.generic;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@SuppressWarnings({"unused", "SpellCheckingInspection"})
public class MecanumMovement {

    public static DcMotor
            backLeft, backRight, frontLeft, frontRight;

    public static boolean reverseLeftDriveMotors = true;
    public int driveMotorStatus = 0;

    //imu turn correction method variables
    public static double turn;
    public static double turn1;
    public static double turn2;

    public double averageDriveDistance;
    public double averageStrafeDistance;
    public double calculatedXDistance;
    public double calculatedYDistance;
    public boolean combinedDriveDone = false;
    boolean poleCentered = false;
    public boolean usePoleHeading = false;
    public boolean poleTurnDone = false;
    public boolean useColorSensor = false;
    public boolean colorLineUpDone = false;
    public double colorRedError;

    Navigation nav;

    public void setNav(Navigation inNav) {
        nav = inNav;
    }
    public static Sensors colorSensorsExt;

    public void setColor(Sensors inSensor){colorSensorsExt = inSensor;}


    //--------- EXACT GEAR CONSTANTS ----------
    static double FAST_GEAR = 0.8;
    //-----------------------------------------


    //---------- TODO DEFINE PLEASE -----------
    boolean controlMode = false;
    //-----------------------------------------


    //-------- Tick Tracking Variables --------
    double currentTicks;
    double referenceTicks;
    //-----------------------------------------


    //------ Minimum robot driving speeds -----
    public static double MIN_STRAFE_SPEED = 0.3;
    public static double MIN_DRIVE_SPEED = 0.12;
    public static double MIN_TURN_SPEED = 0.18;
    //-----------------------------------------

    //-------Robot Ticks per inch stuff

    // TODO: find out how to put this stuff in the hardware class using methods
    public static final double TICKS_PER_INCH = 32.1665435703; // 5000 / 57.00
    public static final double STRAFE_TICKS_PER_INCH = 36.5714008858;
    public static final double TICKS_PER_DEGREE = 5.85572546559; // was 3 (3-21-22)  5.85572546559, then 3... then  3.75  TODO - This goes fartherfor short turns than long... Currently optimized for 30 degree turns
    public static final double CORRECTION_TICKS_PER_DEGREE = 3;

    //TODO find the difference for the NEW TICK stuff might be done using RUN_TO_POSITION stuff
    public static final double NEW_TICKS_PER_INCH = 32.1665435703;
    public static final double NEW_STRAFE_TICKS_PER_INCH = 36.5714008858;
    public static final double NEW_TICKS_PER_DEGREE = 5.85572546559;
    //-----------------------------------------------------------------

    //point to point nav method variables

    public int navStep = 0;
    // used in turnTo method
    public int turnTonavStep = 0;
    public boolean pointIsDone = false;
    public double angleToTurn;
    public double hdg1 = 0;
    public double hdg2 = 0;

    //imu turn correction method variables
    public int correctionStep = 0;
    public int correctionStep2 = 0;
    public int correctionStep3 = 0;
    public double correctionTurn = 0;
    public boolean correctionDone = false;
    public double imuHdg;
    public double correctedImuHdg;
    public double convertedImuHdg;
    public double expectedHdg = 0;
    public double initHeading = 0;
    public double initialFR = 0;
    public double initialBR = 0;
    public double initialFL = 0;
    public double initialBL = 0;
    int step = 0;
    public double distanceLeftToMoveY;
    public double distanceRightToMoveX;
    double xDistance;
    double yDistance;
    double heading;
    public double turnErrorDirection;
    public double fRPosition;
    public double fLPosition;
    public double bLPosition;
    public double bRPosition;
    double prevFR;
    double prevFL;
    double prevBL;
    double prevBR;
    double distFR;
    double distFL;
    double distBR;
    double distBL;
    double legs;
    double deltaYNorm;
    double deltaXNorm;
    public double twistSpeed;
    public double modifiedSpeed;
    public double xModifiedSpeed;
    public double yModifiedSpeed;

    public double powerInsightFrontRight=0;
    public double captureTheXError = 100;
    public double captureTheYError = 100;
    public double captureThePoleError = 100;

    double xDirection = 1;
    double yDirection = 1;


    //turnDistance variables
    int goStep = 0;
    boolean goDone = false;

    //----robotAcceleration from previous hardware class
    //the power the motors try to ramp to
    public double frontRightTargetPower = 0;
    ElapsedTime patentTime = new ElapsedTime(0);
    public double frontLeftTargetPower = 0;
    public double backRightTargetPower = 0;
    public double backLeftTargetPower = 0;
    //the amount the power increases in each cycle
    private final double POWER_ACCELERATION = 0.08;
    //this variable is the positive acceleration of
    //the robot
    public double rampUp = 1;
    //the deceleration of the robot
    public double rampDown = -1.05;
    //the power the motors should be set to
    public double motorPower = 0;
    //the position the motors are trying to run to
    public double targetPos = 0;
    //after the motors are within this many
    //ticks of the destination, start ramping
    //down the speeds
    private final double RAMP_DOWN_RANGE = 1400;
    //the time it takes for each acceleration cycle
    private final double REFRESH_TIME = 0.050;
    //keeps track of the initial time of the cycle
    private double resetTime = 0;
    //tells the robot when it should accelerate
    private boolean speedReset = false;
    //the range of the ticks the robot will shoot for
    public double errorMarginTicks = 15;
    //the number of ticks before the target position the
    //robot will go to before stopping
    private final double STOP_DISTANCE = 350;
    //the highest speed the robot went to during the acceleration
    public double maxSpeed = 0;
    //step for the moving functions
    public int moveStep = 0;
    public boolean doneMoving = false;
    public boolean turnDone = false;
    public boolean turnToDone = false;

    public double lastHeading = 2000.0;

    double calculatedFrontDirection = 0;
    double desiredFrontDirection = 0;

    // auto acceleration variables
    boolean accelDone = false;
    int accelStep = 0;
    double distanceGone;
    double minPower;
    double maxPower;
    double currentPower;
    double distanceMin;
    double distanceToGo;
    int accelLoop = 0;
    int accelTurnLoop = 0;
    double accelGear;
    double accelTicksPerInch;

    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;

    public double headingError;

    // auto acceleration constants
    double DISTANCE_TO_ACCELERATE = 30;
    double DISTANCE_TO_CORRECT = 5;
    double ACCELERATION_CONSTANT;
    double DECELERATION_CONSTANT;

    double DELTA_POWER_ACCEL = 0.05;
    double DELTA_POWER_DECEL = 0.025;
    double DECEL_DISTANCE = 72; // in inches
    double ACCEL_K = DELTA_POWER_DECEL * DECEL_DISTANCE;

    double maxError = 3.5;
    public String whatLineCorrectionRegionAmI = "not set";
    public double absoluteDistanceToMove = 0;

    OdometerNavTracking odometerNavTracking = new OdometerNavTracking();

    final ElapsedTime period = new ElapsedTime();

    // Initialize standard Hardware interfaces
    public void init(HardwareMap hwMap) {

        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotorStatus = 3;
        // This allows the motor sides to be reversed depending on robot motor directions
        if (reverseLeftDriveMotors) {
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            driveMotorStatus = 1;
        } else {
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            driveMotorStatus = 2;
        }
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // This allows hardware class to reset which side should be reversed and must be called before init()

    public void setReverseDriveMotorSideLeft(boolean left) {
        reverseLeftDriveMotors = left;
    }


    //-----------------------------------------------------------------------

    //----------------- MOVING, SUCH AS TURNING AND DRIVING AND TURNING DRIVING -----------------


    public boolean combinedDrive(double desiredHdg, double desiredX, double desiredY, double speed, double error) {

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();
                // First time through - no distance moved, so no math done for error - so go straight to step 2
                step = 2;
                break;
            case 1:
                updateNavFromEncodersAfterInitial();
                // TCalculate heading error!
                // NOTE - "break;" not included on purpose - flow directly into "case 2"

            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;
                //value

                if (Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) {
                    step = 3;
                    break;
                }
                // Using similar triangle rules to get the speeds under 1
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);

                turnErrorDirection = headingError / Math.abs(headingError);
                if (Math.abs(headingError) > 1) {
                    if (Math.abs(headingError) < 10) {
                        twistSpeed = turnErrorDirection * -0.05;
                    } else {
                        twistSpeed = turnErrorDirection * -speed * .5;
                    }
                } else {
                    twistSpeed = 0;
                }

                // set speed
                mecanumMovementFieldOrientedControl(deltaYNorm * speed, -deltaXNorm * speed, twistSpeed);
                // since not done - ensure return to step 1 for next nav update
                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                step = 0; // Reset step to 0 so it will work next time.
                setDrivePower(0);
                break;
        }

        return combinedDriveDone;
    }

    public boolean combinedDrive(double desiredHdg, double desiredX, double desiredY, double speed, double error, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();
                step = 2;
                break;
            case 1:
                updateNavFromEncodersAfterInitial();
                // Omitting step and break intentionally -->  Goes straight to setting power without needing another loop...

            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                //value
                // These values are never used...could be deleted / commented out...
                xDistance = (distanceRightToMoveX * Math.cos(heading)) + (distanceLeftToMoveY * Math.sin(heading));
                yDistance = (-distanceRightToMoveX * Math.sin(heading)) + (distanceLeftToMoveY * Math.cos(heading));

                if (DoYouWantTheTurnToFinish) {
                    // If turn completion required - check both turn and distance error
                    if ((Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) && Math.abs(headingError) <= turnError) {
                        // If both distance and angles are done - go to step 3
                        step = 3;
                        break;
                    }
                } else {
                    // Ignore turn position!
                    if (Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) {
                        //  This is OK...
                        step = 3;
                        break;
                    }
                }

                // Using similar triangle rules to get the speeds under 1
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                turnErrorDirection = headingError / Math.abs(headingError);

                // For small errors <10 deg -> ignore turn speed inputs and go slow...
                    if (Math.abs(headingError) < 10) {
                        twistSpeed = turnErrorDirection * -0.05;
                    } else if (Math.abs(headingError) > turnError) {
                        /** This is where this version is different than "combinedDrive"
                         */
                        twistSpeed = turnErrorDirection * -turnSpeed * 0.5;
                    } else {

                        twistSpeed =  0;
                }
                // kind of like we do for small turn errors (
                if ((Math.abs(distanceRightToMoveX)  <= 10 &&  Math.abs(distanceLeftToMoveY) <= 10)) {
                    modifiedSpeed = 0.2;
                } else {
                    modifiedSpeed = speed;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(deltaYNorm * modifiedSpeed, -deltaXNorm * modifiedSpeed, twistSpeed);

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
        }

        return combinedDriveDone;
    }
    public boolean combinedDriveWithXYError(double desiredHdg, double desiredX, double desiredY, double speed, double yError, double xError, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();

                step = 2;
                break;
            case 1:
                // This case only updates nav (calculated from previous positions based on motor ticks)
                updateNavFromEncodersAfterInitial();

            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                if (DoYouWantTheTurnToFinish) {
                    // If turn completion required - check both turn and distance error
                    if (Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError && Math.abs(headingError) <= turnError) {
                        // If both distance and angles are done - go to step 3
                        step = 3;
                        break;
                    }
                } else {
                    // Ignore turn position!
                    if (Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError) {
                        //  This is OK...
                        step = 3;
                        break;
                    }
                }

                // Using similar triangle rules to get the speeds under 1
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                // turnErrorDirection returens either 1 or -1 to drive turn direction properly
                turnErrorDirection = -headingError / Math.abs(headingError);

                //Gradual Decceleration (turn)
                if (Math.abs(headingError) < turnError) {
                    twistSpeed = 0;
                } else if (Math.abs(headingError) < 2) {
                    twistSpeed =  turnErrorDirection*0.04;
                } else if (Math.abs(headingError) < 3) {
                    twistSpeed =  turnErrorDirection*0.05;
                } else if (Math.abs(headingError) < 4) {
                    twistSpeed =  turnErrorDirection*0.06;
                } else if (Math.abs(headingError) < 6) {
                    twistSpeed = turnErrorDirection * 0.07;
                } else if (Math.abs(headingError) < 8) {
                    twistSpeed = turnErrorDirection * 0.08;
                } else if (Math.abs(headingError) < 10) {
                    twistSpeed =  turnErrorDirection*0.1;
                } else if (Math.abs(headingError) < 20) {
                    twistSpeed =  turnErrorDirection*0.12;
                } else {
                 twistSpeed = turnErrorDirection*turnSpeed;
                }

                if (Math.abs(distanceLeftToMoveY) <= 2) {
                    yModifiedSpeed = 0.15;
                } else if (Math.abs(distanceLeftToMoveY) <= 2.5) {
                    yModifiedSpeed = 0.175;
                } else if (Math.abs(distanceLeftToMoveY) <= 3) {
                    yModifiedSpeed = 0.2;
                } else if (Math.abs(distanceLeftToMoveY) <= 4) {
                    yModifiedSpeed = 0.225;
                } else if (Math.abs(distanceLeftToMoveY) <= 6) {
                    yModifiedSpeed = 0.25;
                } else if (Math.abs(distanceLeftToMoveY) <= 8) {
                    yModifiedSpeed = 0.3;
                } else  if (Math.abs(distanceLeftToMoveY) <= 12) {
                    yModifiedSpeed = 0.4;
                } else  if (Math.abs(distanceLeftToMoveY) <= 18) {
                    yModifiedSpeed = 0.5;
                } else  if (Math.abs(distanceLeftToMoveY) <= 24) {
                    yModifiedSpeed = 0.7;
                } else  {
                    yModifiedSpeed = 1;
                }
                if (Math.abs(distanceRightToMoveX)  <= 2) {
                    xModifiedSpeed = 0.15;
                } else if (Math.abs(distanceRightToMoveX)  <= 2.5) {
                    xModifiedSpeed = 0.175;
                } else if (Math.abs(distanceRightToMoveX)  <= 3) {
                    xModifiedSpeed = 0.2;
                } else if (Math.abs(distanceRightToMoveX)  <= 4) {
                    xModifiedSpeed = 0.225;
                } else if (Math.abs(distanceRightToMoveX)  <= 6) {
                    xModifiedSpeed = 0.25;
                } else if (Math.abs(distanceRightToMoveX)  <= 8) {
                    xModifiedSpeed = 0.3;
                } else if (Math.abs(distanceRightToMoveX)  <= 12) {
                    xModifiedSpeed = 0.4;
                } else if (Math.abs(distanceRightToMoveX)  <= 18) {
                    xModifiedSpeed = 0.5;
                } else if (Math.abs(distanceRightToMoveX)  <= 24) {
                    xModifiedSpeed = 0.7;
                } else {
                    xModifiedSpeed = 1;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(deltaYNorm * yModifiedSpeed, -deltaXNorm * xModifiedSpeed, twistSpeed*.5);

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
        }

        return combinedDriveDone;
    }

    public boolean combinedDriveWithXYErrorFastCurve(double desiredHdg, double desiredX, double desiredY, double speed, double yError, double xError, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();

                step = 2;
                break;
            case 1:
                // This case only updates nav (calculated from previous positions based on motor ticks)
                updateNavFromEncodersAfterInitial();

            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                if (DoYouWantTheTurnToFinish) {
                    // If turn completion required - check both turn and distance error
                    if ((Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError) && Math.abs(headingError) <= turnError) {
                        // If both distance and angles are done - go to step 3
                        step = 3;
                        break;
                    }
                } else {
                    // Ignore turn position!
                    if (Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError) {
                        //  This is OK...
                        step = 3;
                        break;
                    }
                }

                // Using similar triangle rules to get the speeds under 1
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                turnErrorDirection = -headingError / Math.abs(headingError);

                //Gradual Decceleration (turn)
                if (Math.abs(headingError) < turnError) {
                    twistSpeed = 0;
                } else if (Math.abs(headingError) < 2) {
                    twistSpeed =  turnErrorDirection* .025;
                } else if (Math.abs(headingError) < 3) {
                    twistSpeed =  turnErrorDirection*0.03;
                } else if (Math.abs(headingError) < 4) {
                    twistSpeed =  turnErrorDirection*0.04;
                } else if (Math.abs(headingError) < 6) {
                    twistSpeed = turnErrorDirection * 0.05;
                } else if (Math.abs(headingError) < 8) {
                    twistSpeed = turnErrorDirection * 0.07;
                } else if (Math.abs(headingError) < 10) {
                    twistSpeed =  turnErrorDirection*0.09;
                } else if (Math.abs(headingError) < 20) {
                    twistSpeed =  turnErrorDirection*0.12;
                } else {
                    twistSpeed = turnErrorDirection*turnSpeed;
                }

                if (Math.abs(distanceLeftToMoveY) <= 2) {
                    yModifiedSpeed = 0.2;
                } else if (Math.abs(distanceLeftToMoveY) <= 2.5) {
                    yModifiedSpeed = 0.225;
                } else if (Math.abs(distanceLeftToMoveY) <= 3) {
                    yModifiedSpeed = 0.25;
                } else if (Math.abs(distanceLeftToMoveY) <= 4) {
                    yModifiedSpeed = 0.275;
                } else if (Math.abs(distanceLeftToMoveY) <= 6) {
                    yModifiedSpeed = 0.3;
                } else if (Math.abs(distanceLeftToMoveY) <= 8) {
                    yModifiedSpeed = 0.35;
                } else  if (Math.abs(distanceLeftToMoveY) <= 12) {
                    yModifiedSpeed = 0.4;
                } else  if (Math.abs(distanceLeftToMoveY) <= 18) {
                    yModifiedSpeed = 0.5;
                } else  if (Math.abs(distanceLeftToMoveY) <= 24) {
                    yModifiedSpeed = 0.7;
                } else  {
                    yModifiedSpeed = 1;
                }
                if (Math.abs(distanceRightToMoveX)  <= 2) {
                    xModifiedSpeed = 0.2;
                } else if (Math.abs(distanceRightToMoveX)  <= 2.5) {
                    xModifiedSpeed = 0.225;
                } else if (Math.abs(distanceRightToMoveX)  <= 3) {
                    xModifiedSpeed = 0.25;
                } else if (Math.abs(distanceRightToMoveX)  <= 4) {
                    xModifiedSpeed = 0.275;
                } else if (Math.abs(distanceRightToMoveX)  <= 6) {
                    xModifiedSpeed = 0.3;
                } else if (Math.abs(distanceRightToMoveX)  <= 8) {
                    xModifiedSpeed = 0.35;
                } else if (Math.abs(distanceRightToMoveX)  <= 12) {
                    xModifiedSpeed = 0.4;
                } else if (Math.abs(distanceRightToMoveX)  <= 18) {
                    xModifiedSpeed = 0.5;
                } else if (Math.abs(distanceRightToMoveX)  <= 24) {
                    xModifiedSpeed = 0.7;
                } else {
                    xModifiedSpeed = 1;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(deltaYNorm * yModifiedSpeed, -deltaXNorm * xModifiedSpeed, twistSpeed*.5);

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
        }

        return combinedDriveDone;
    }

    public boolean combinedDriveStartToPole(double desiredHdg, double desiredX, double desiredY, double speed, double yError, double xError, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish, double poleErrorFromMiddle) {

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                poleTurnDone = false;
                usePoleHeading = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();
                step = 2;
                break;
            case 1:
                // This case only updates nav (calculated from previous positions based on motor ticks)
                updateNavFromEncodersAfterInitial();
                if(!usePoleHeading) {
                    usePoleHeading = Math.abs(headingError) < 8 && Math.abs(distanceRightToMoveX) <= 5 && Math.abs(distanceLeftToMoveY) <= 5;
                }
            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;
                // This section added to determine direction:
                xDirection = distanceRightToMoveX/Math.abs(distanceRightToMoveX);
                yDirection = distanceLeftToMoveY/Math.abs(distanceLeftToMoveY);

                if (DoYouWantTheTurnToFinish) {
                    // If turn completion required - check both turn and distance error
                    if ((Math.abs(distanceRightToMoveX) <= xError && Math.abs(distanceLeftToMoveY) <= yError) && Math.abs(poleErrorFromMiddle) <2) {
                        // If both distance and angles are done - go to step 3
                        // IF done - set all power to zero & move on!
                        xModifiedSpeed = 0;
                        yModifiedSpeed = 0;
                        captureThePoleError = poleErrorFromMiddle;
                        captureTheXError = distanceRightToMoveX;
                        captureTheYError = distanceLeftToMoveY;
                        step = 3;
                        break;
                    }
                } else {
                    // Ignore turn position!
                    if (Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError) {
                        xModifiedSpeed = 0;
                        yModifiedSpeed = 0;
                        step = 3;
                        break;
                    }
                }
                double turnErrorDirection = -headingError / Math.abs(headingError);

                if (usePoleHeading){
                    twistSpeed = centerOnPoleUpdate(poleErrorFromMiddle);
                    if (twistSpeed == 0 ) {
                        poleTurnDone = true;
                    } else {
                        poleTurnDone = false;
                    }
                }else {
                    //Gradual Decceleration (turn)
                    if (Math.abs(headingError) < turnError) {
                        twistSpeed = 0;
                    } else if (Math.abs(headingError) < 2) {
                        twistSpeed = turnErrorDirection * .015;
                    } else if (Math.abs(headingError) < 3) {
                        twistSpeed = turnErrorDirection * 0.017;
                    } else if (Math.abs(headingError) < 4) {
                        twistSpeed = turnErrorDirection * 0.019;
                    } else if (Math.abs(headingError) < 6) {
                        twistSpeed = turnErrorDirection * 0.02;
                    } else if (Math.abs(headingError) < 10) {
                        twistSpeed = turnErrorDirection * 0.035;//.25 I think it was supposed to be .025 but whatever
                    } else if (Math.abs(headingError) < 20) {
                        twistSpeed = turnErrorDirection * 0.05;//.0375
                    } else {
                        twistSpeed = turnErrorDirection * turnSpeed;
                    }
                }

                if (Math.abs(distanceLeftToMoveY) <= yError) {
                    yModifiedSpeed = 0.0;
                } else if (Math.abs(distanceLeftToMoveY) <= 2){
                    yModifiedSpeed = 0.1;
                } else if (Math.abs(distanceLeftToMoveY) <= 4) {
                    yModifiedSpeed = 0.2;
                } else  if (Math.abs(distanceLeftToMoveY) <= 12) {
                    yModifiedSpeed = 0.3;
                } else  if (Math.abs(distanceLeftToMoveY) <= 18) {
                    yModifiedSpeed = 0.5;
                } else  if (Math.abs(distanceLeftToMoveY) <= 24) {
                    yModifiedSpeed = 0.7;
                } else  {
                    yModifiedSpeed = 1;
                }

                if (Math.abs(distanceRightToMoveX)  <= 0.5) {
                    xModifiedSpeed = 0;
                } else if (Math.abs(distanceRightToMoveX)  <= 1){
                    xModifiedSpeed = 0.1;
                } else if (Math.abs(distanceRightToMoveX)  <= 2) {
                    xModifiedSpeed = 0.4;
                } else if (Math.abs(distanceRightToMoveX)  <= 3) {
                    xModifiedSpeed = 0.6;
                } else if (Math.abs(distanceRightToMoveX)  <= 4) {
                    xModifiedSpeed = 0.8;
                } else if (Math.abs(distanceRightToMoveX)  <= 6) {
                    xModifiedSpeed = 0.9;
                } else {
                    xModifiedSpeed = 1;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(yModifiedSpeed * yDirection, -xModifiedSpeed * xDirection, twistSpeed);

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
        }

        return combinedDriveDone;
    }

    public boolean combinedDriveStackToPole(double desiredHdg, double desiredX, double desiredY, double speed, double yError, double xError, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish, double poleErrorFromMiddle) {

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                poleTurnDone = false;
                usePoleHeading = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();

                step = 2;
                break;
            case 1:
                // This case only updates nav (calculated from previous positions based on motor ticks)
                updateNavFromEncodersAfterInitial();
                if(!usePoleHeading) {
                    if (Math.abs(headingError) < 10 && (Math.abs(distanceRightToMoveX)  <= 5 && Math.abs(distanceLeftToMoveY) <= 5)) {
                        usePoleHeading = true;
                    } else {
                        usePoleHeading = false;
                    }
                }
            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                // This section added to determine direction:
                xDirection = distanceRightToMoveX/Math.abs(distanceRightToMoveX);
                yDirection = distanceLeftToMoveY/Math.abs(distanceLeftToMoveY);

                double turnErrorDirection = -headingError / Math.abs(headingError);

                if (usePoleHeading){
                    twistSpeed = centerOnPoleUpdate(poleErrorFromMiddle);
                    if (twistSpeed == 0 ) {
                        poleTurnDone = true;
                    } else {
                        poleTurnDone = false;
                    }
                }else {
                    //Gradual Decceleration (turn)
                    if (Math.abs(headingError) < turnError) {
                        twistSpeed = 0;
                    } else if (Math.abs(headingError) < 2) {
                        twistSpeed = turnErrorDirection * .015;
                    } else if (Math.abs(headingError) < 3) {
                        twistSpeed = turnErrorDirection * 0.02;
                    } else if (Math.abs(headingError) < 4) {
                        twistSpeed = turnErrorDirection * 0.025;
                    } else if (Math.abs(headingError) < 6) {
                        twistSpeed = turnErrorDirection * 0.03;
                    } else if (Math.abs(headingError) < 10) {
                        twistSpeed = turnErrorDirection * 0.05;
                    } else if (Math.abs(headingError) < 20) {
                        twistSpeed = turnErrorDirection * 0.075;
                    } else {
                        twistSpeed = turnErrorDirection * turnSpeed * .5;
                    }
                }
                if (DoYouWantTheTurnToFinish) {
                    // If turn completion required - check both turn and distance error
                    if ((Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError) && Math.abs(poleErrorFromMiddle) <2 ) {
                        // If both distance and angles are done - go to step 3
                        step = 3;
                        break;
                    }
                } else {
                    // Ignore turn position!
                    if (Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError) {
                        //  This is OK...
                        step = 3;
                        break;
                    }
                }
                // Proportional correction for Y error
                if (Math.abs(distanceLeftToMoveY) <= 1) {
                    yModifiedSpeed = 0.2;
                } else if (Math.abs(distanceLeftToMoveY) <= 2) {
                    yModifiedSpeed = 0.4;
                } else  if (Math.abs(distanceLeftToMoveY) <= 3) {
                    yModifiedSpeed = 0.6;
                } else  if (Math.abs(distanceLeftToMoveY) <= 4) {
                    yModifiedSpeed = 0.8;
                } else  if (Math.abs(distanceLeftToMoveY) <= 6) {
                    yModifiedSpeed = 0.9;
                } else  {
                    yModifiedSpeed = 1;
                }

                // Proportional correction for X error
                if (Math.abs(distanceRightToMoveX)  <= 2) {
                    xModifiedSpeed = 0.1;
                } else if (Math.abs(distanceRightToMoveX)  <= 4) {
                    xModifiedSpeed = 0.2;
                } else if (Math.abs(distanceRightToMoveX)  <= 12) {
                    xModifiedSpeed = 0.3;
                } else if (Math.abs(distanceRightToMoveX)  <= 16) {
                    xModifiedSpeed = 0.4;
                } else if (Math.abs(distanceRightToMoveX)  <= 20) {
                    xModifiedSpeed = 0.7;
                } else {
                    xModifiedSpeed = 1;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(yModifiedSpeed * yDirection, -xModifiedSpeed * xDirection, twistSpeed);

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
        }

        return combinedDriveDone;
    }
    public boolean combinedDrivePoleToStack(double desiredHdg, double desiredX, double desiredY, double speed, double yError, double xError, double colorError, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish, double redTarget, double blueTarget, double redMeasured, double blueMeasured) {

        heading = nav.getCorrectedImuHdg();
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                colorLineUpDone = false;
                useColorSensor = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();

                step = 2;
                break;
            case 1:
                // This case only updates nav (calculated from previous positions based on motor ticks)

                updateNavFromEncodersAfterInitial();
                headingError = Reference.angleConversion(desiredHdg - heading);
                if(!useColorSensor) {
                    if (nav.getRobotX() < -54 && nav.getRobotY() < -12) {
                        useColorSensor = true;
                    } else {
                        useColorSensor = false;
                    }
                }
            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;
                // This section added to determine direction - when driven by x & y error...
                xDirection = distanceRightToMoveX/Math.abs(distanceRightToMoveX);
                yDirection = distanceLeftToMoveY/Math.abs(distanceLeftToMoveY);

                double turnErrorDirection = -headingError / Math.abs(headingError);

                //Gradual Decceleration (turn)
                if (Math.abs(headingError) < turnError) {
                        twistSpeed = 0;
                } else if (Math.abs(headingError) < 2) {
                        twistSpeed = turnErrorDirection * 0.018;
                } else if (Math.abs(headingError) < 3) {
                        twistSpeed = turnErrorDirection * 0.022;
                } else if (Math.abs(headingError) < 4) {
                        twistSpeed = turnErrorDirection * 0.027;
                } else if (Math.abs(headingError) < 6) {
                        twistSpeed = turnErrorDirection * 0.035;
                } else if (Math.abs(headingError) < 10) {
                    twistSpeed = turnErrorDirection * 0.07;
                } else if (Math.abs(headingError) < 20) {
                        twistSpeed = turnErrorDirection * 0.1;
                } else {
                        twistSpeed = turnErrorDirection * turnSpeed * .4;
                }

                colorRedError = redTarget - redMeasured;
                // this calculates the error of what the color sensor is reading vs what you want

                double colorRedErrorDirection = colorRedError / Math.abs(colorRedError);
                // this calculates which side of the line you are on by dividing by the absolute of the error so you get 1 or -1

                if (useColorSensor){
                    // checks to see if you are on color line, and moves depending on that
                    // when on gray color sensor reads 78. when fully on red color sensor reads 205
                    //Gradual Decceleration (on color line)
                    if (Math.abs(colorRedError) < colorError) {
                        colorLineUpDone = true;
                    } else {
                        colorLineUpDone = false;
                    }
                    // Proportional correction of Y error using color sensor
                    if (Math.abs(colorRedError) < 30) {
                        yModifiedSpeed = 0;
                    } else if (Math.abs(colorRedError) < 40) {
                        yModifiedSpeed = colorRedErrorDirection * 0.015;
                    } else if (Math.abs(colorRedError) < 60) {
                        yModifiedSpeed = colorRedErrorDirection * 0.03;
                    } else {
                        yModifiedSpeed = colorRedErrorDirection * .06;
                    }
                    yDirection = 1;

                }else {

                    // Proportional correction of Y error when not using color sensor
                    if (Math.abs(distanceLeftToMoveY) <= 1) {
                        yModifiedSpeed = 0.05;
                    } else if (Math.abs(distanceLeftToMoveY) <= 2) {
                        yModifiedSpeed = 0.07;
                    } else if (Math.abs(distanceLeftToMoveY) <= 3) {
                        yModifiedSpeed = 0.1;
                    } else if (Math.abs(distanceLeftToMoveY) <= 4) {
                        yModifiedSpeed = 0.2;
                    } else if (Math.abs(distanceLeftToMoveY) <= 6) {
                        yModifiedSpeed = 0.3;
                    } else {
                        yModifiedSpeed = 0.5;
                    }
                }
                // Proportional correction of X error
                if (Math.abs(distanceRightToMoveX)  <= 4) {
                    xModifiedSpeed = 0;
                } else if (Math.abs(distanceRightToMoveX)  <= 6) {
                    xModifiedSpeed = 0.25;
                } else if (Math.abs(distanceRightToMoveX)  <= 12) {
                    xModifiedSpeed = 0.3;
                } else if (Math.abs(distanceRightToMoveX)  <= 16) {
                    xModifiedSpeed = 0.4;
                } else if (Math.abs(distanceRightToMoveX)  <= 20) {
                    xModifiedSpeed = 0.7;
                } else {
                    xModifiedSpeed = 1;
                }

                if (DoYouWantTheTurnToFinish) {
                    // If turn completion required - check both turn and distance error
                    if (Math.abs(distanceRightToMoveX)  <= xError && colorLineUpDone && Math.abs(headingError) <= turnError) {
                        // If both distance and angles are done - go to step 3
                        step = 3;
                        break;
                    }
                } else {
                    // Ignore turn position!
                    if (Math.abs(distanceRightToMoveX)  <= xError && Math.abs(distanceLeftToMoveY) <= yError) {
                        //  This is OK...
                        step = 3;
                        break;
                    }
                }
                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(yModifiedSpeed * yDirection, -xModifiedSpeed * xDirection, twistSpeed);

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
        }

        return combinedDriveDone;
    }

    public double centerOnPole (double poleErrorFromMiddle){
        double twistSpeed = 0;
        double turnErrorDirection = poleErrorFromMiddle / Math.abs(poleErrorFromMiddle);
        if (Math.abs(poleErrorFromMiddle) < 2.2) {
            twistSpeed = 0;
        } else if (Math.abs(poleErrorFromMiddle) < 4) {
            twistSpeed =  turnErrorDirection*0.03;
        } else if (Math.abs(poleErrorFromMiddle) < 6) {
            twistSpeed =  turnErrorDirection*0.04;
        } else if (Math.abs(poleErrorFromMiddle) < 10) {
            twistSpeed =  turnErrorDirection*0.06;
        } else if (Math.abs(poleErrorFromMiddle) < 15) {
            twistSpeed =  turnErrorDirection*0.1;
        } else if (Math.abs(poleErrorFromMiddle) < 20) {
            twistSpeed =  turnErrorDirection * .2;
        } else {
            twistSpeed = turnErrorDirection *.4;
        }
        return twistSpeed;
    }

    public double centerOnPoleUpdate (double poleErrorFromMiddle){
        double twistSpeed = 0;
        double turnErrorDirection = poleErrorFromMiddle / Math.abs(poleErrorFromMiddle);
        if (Math.abs(poleErrorFromMiddle) < 2.2) {
            twistSpeed = 0;
        } else if (Math.abs(poleErrorFromMiddle) < 4) {
            twistSpeed =  turnErrorDirection*0.04;
        } else if (Math.abs(poleErrorFromMiddle) < 6) {
            twistSpeed =  turnErrorDirection*0.06;
        } else if (Math.abs(poleErrorFromMiddle) < 10) {
            twistSpeed =  turnErrorDirection*0.08;
        } else if (Math.abs(poleErrorFromMiddle) < 15) {
            twistSpeed =  turnErrorDirection*0.1;
        } else if (Math.abs(poleErrorFromMiddle) < 20) {
            twistSpeed =  turnErrorDirection * .2;
        } else {
            twistSpeed = turnErrorDirection *.4;
        }
        return twistSpeed;
    }


    public boolean combinedDriveWithColorLine(double desiredHdg, double redValue, double blueValue, double speed, double error, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();
                step = 2;
                break;

            case 1:
                updateNavFromEncodersAfterInitial();
                // Omitting step and break intentionally -->  Goes straight to setting power without needing another loop...

            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.

                // checks to see if you are on color line, and moves depending on that
                if (colorSensorsExt.colorSensor.red() > redValue || colorSensorsExt.colorSensor.blue() > blueValue){
                    // if true you are on line - start moving -y direction
                    // old version: setCustomMove(90,.1,0,0,0,0);
                    modifiedSpeed = -speed;

                }else{
                   // if not true you are not on the line
                    modifiedSpeed = speed;
                }

                if (DoYouWantTheTurnToFinish) {
                    // If turn completion required - check both turn and distance error

                    if ((redValue - error)<= colorSensorsExt.colorSensor.red() && (redValue + error)> colorSensorsExt.colorSensor.red() || (blueValue - error)<= colorSensorsExt.colorSensor.blue() && (blueValue + error)> colorSensorsExt.colorSensor.blue() && Math.abs(headingError) <= turnError) {
                        // If both distance and angles are done - go to step 3
                        step = 3;
                        break;
                    }
                } else {
                    // Ignore turn position!
                    if ((redValue - error)<= colorSensorsExt.colorSensor.red() && (redValue + error)> colorSensorsExt.colorSensor.red() || (blueValue - error)<= colorSensorsExt.colorSensor.blue() && (blueValue + error)> colorSensorsExt.colorSensor.blue()) {
                        //  This is OK...
                        step = 3;
                        break;
                    }
                }

                // TODO these lines never used...
                // Using similar triangle rules to get the speeds under 1
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                turnErrorDirection = headingError / Math.abs(headingError);

                // For small errors <10 deg -> ignore turn speed inputs and go slow...
                if (Math.abs(headingError) < 10) {
                    twistSpeed = turnErrorDirection * -0.05;
                } else if (Math.abs(headingError) > turnError) {
                    /** This is where this version is different than "combinedDrive"
                     */
                    twistSpeed = turnErrorDirection * -turnSpeed * 0.5;
                } else {

                    twistSpeed =  0;
                }


                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(modifiedSpeed, 0, twistSpeed);

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
            case 4:
                // This case checks if done and sets speed
                // Distance needed to move to finish
                //value
                xDistance = (distanceRightToMoveX * Math.cos(heading)) + (distanceLeftToMoveY * Math.sin(heading));
                yDistance = (-distanceRightToMoveX * Math.sin(heading)) + (distanceLeftToMoveY * Math.cos(heading)); //negative?

                // Original:  " if (Math.abs(desiredX - xDistance)  <= error && Math.abs(desiredY - yDistance) <= error)

                if (DoYouWantTheTurnToFinish) {
                    // Checks to see if distance error is within limits
                    if ((Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) && Math.abs(headingError) <= turnError) {
                        step = 3;
                        break;
                    } else {
                        step = 4;
                    }
                } else {
                    // Ignore turn position!
                    if (Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) {
                        step = 3;
                        break;
                    }
                }


                // Using similar triangle rules to get the speeds under 1
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);

                turnErrorDirection = headingError / Math.abs(headingError);

                // For small errors <10 deg -> ignore turn speed inputs and go slow...
                if (Math.abs(headingError) > turnError) {
                    if (Math.abs(headingError) < 5) {
                        twistSpeed = turnErrorDirection * -0.05;
                    } else {
                        /** This is where this version is different than "combinedDrive"
                         */
                        twistSpeed = turnErrorDirection * -turnSpeed * 0.5;
                    }
                } else {
                    twistSpeed = 0;
                }

                if ((Math.abs(distanceRightToMoveX)  <= 3 &&  Math.abs(distanceLeftToMoveY) <= 3)) {
                    modifiedSpeed = 0.1;
                } else {
                    modifiedSpeed = speed;
                }

                mecanumMovementFieldOrientedControl(0, 0, twistSpeed);

                step = 1;
                break;
        }

        return combinedDriveDone;
    }

    public void captureInitialEncoderPositions() {
        initialFR = frontRight.getCurrentPosition();
        initialFL = frontLeft.getCurrentPosition();
        initialBR = backRight.getCurrentPosition();
        initialBL = backLeft.getCurrentPosition();
    }

    public void updateNavFromEncodersAfterInitial() {
        // This case only updates nav (calculated from previous positions based on motor ticks)

        // Current (new) positions for each motor
        fRPosition = frontRight.getCurrentPosition();
        fLPosition = frontLeft.getCurrentPosition();
        bLPosition = backLeft.getCurrentPosition();
        bRPosition = backRight.getCurrentPosition();

        // Capture old position for each motor prior to subtraction
        prevFR = initialFR; prevFL = initialFL; prevBL = initialBL; prevBR = initialBR;

        // Calculate disatance each motor has moved since last time through loop
        distFR = fRPosition - initialFR;
        initialFR = fRPosition;
        distFL = fLPosition - initialFL;
        initialFL = fLPosition;
        distBR = bRPosition - initialBR;
        initialBR = bRPosition;
        distBL = bLPosition - initialBL;
        initialBL = bLPosition;

        // Calculate average drive and strafe distances
        averageDriveDistance = (distFL + distFR + distBL + distBR) / (4 * TICKS_PER_INCH);
        averageStrafeDistance = (-distFL + distFR + -distBR + distBL) / (4 * STRAFE_TICKS_PER_INCH);

        // Distance moved since last cycle
        calculatedXDistance = (averageDriveDistance * Math.cos(Math.toRadians(heading)) - (averageStrafeDistance * Math.sin(Math.toRadians(heading))));
        calculatedYDistance = (averageDriveDistance * Math.sin(Math.toRadians(heading)) + (averageStrafeDistance * Math.cos(Math.toRadians(heading)))); //negative?

        //Update Nav - this statement updates robot current nav position from previous
        nav.tellDeltaXYPositionAfterMove(calculatedXDistance, calculatedYDistance);
    }

    public boolean combinedDriveWithXLine(double desiredHdg, double desiredX, double desiredY, double speed, double error, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {

        double xDriveInput = 0;
        double yDriveInput = 0;

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();

                step = 2;
                break;
            case 1:
                // This case only updates nav (calculated from previous positions based on motor ticks)
                updateNavFromEncodersAfterInitial();
                // Omitting step and break intentionally -->  Goes straight to setting power without needing another loop...

            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                // Using similar triangle rules to get the speeds (xInput and yInput) under 1 with similar triangles
                // The "deltaYNorm" and "deltaXNorm" are turned into "joystick" inputs for field-oriented driving
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                // This lowers speed for small errors...
                if ((Math.abs(distanceRightToMoveX) <= 10 && Math.abs(distanceLeftToMoveY) <= 10)) {
                    modifiedSpeed = 0.1;
                } else {
                    modifiedSpeed = speed;
                }

                absoluteDistanceToMove = Reference.DISTANCE_FORMULA(0,0,distanceRightToMoveX,distanceLeftToMoveY);

                // DistanceRightToMoveX IS the X Error we want to control corrections using!
                if (Math.abs(distanceRightToMoveX) > error && Math.abs(distanceRightToMoveX) < maxError  && absoluteDistanceToMove > 12) {
                    // This case - more than minimum error, but less than max error - do proportional correction
                    // NOTE - for short distances to "y" target - just use pure pursuit (do not correct to line)
                    // do proportional correction to line = x input = xError / maxError * y input

                    // The following provides proportional nav back to the line (towards x error equals zero)
                    xDriveInput = (distanceRightToMoveX/maxError) * deltaYNorm * modifiedSpeed;
                    yDriveInput = deltaYNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "proportional correction";

                } else if (Math.abs(distanceRightToMoveX) > maxError && absoluteDistanceToMove > 12) {
                    // make x input = y input
                    // IF error is large - correct back to line using a 45 degree correction = max correction!

                    // This one command a 45 degree (max!) correction - by making X input equal to y input (negative because of conventions)
                    xDriveInput = deltaYNorm * modifiedSpeed * deltaXNorm / Math.abs(deltaXNorm); // Added "* deltaXNorm / Math.abs(deltaXNorm)" to correct sign for +/- quadrants
                    yDriveInput = deltaYNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "max correction";

                } else {
                    // do everything the way we did! - Pure pursuit to target

                    // First - See if I'm done!
                    if (DoYouWantTheTurnToFinish) {
                        // If turn completion required - check both turn and distance error
                        if ((Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) && Math.abs(headingError) <= turnError) {
                            // If both distance and angles are done - go to step 3
                            step = 3;
                            break;
                        }
                    } else {
                        // Ignore turn position!
                        if (Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) {
                            //  This is OK...
                            step = 3;
                            break;
                        }
                    }
                    // The following line takes the direct inputs for pure pursuit
                    xDriveInput = deltaXNorm * modifiedSpeed; // This was the problem - "double" entering the -1* for "pure pursuit"
                    yDriveInput = deltaYNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "pure pursuit";
                }

                turnErrorDirection = headingError / Math.abs(headingError);

                // For small errors <10 deg -> ignore turn speed inputs and go slow...
                if (Math.abs(headingError) < 10) {
                    twistSpeed = turnErrorDirection * -0.05;
                } else if (Math.abs(headingError) > turnError) {
                    /** This is where this version is different than "combinedDrive"
                     */
                    twistSpeed = turnErrorDirection * -turnSpeed * 0.5;
                } else {
                    twistSpeed =  0;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(yDriveInput, -xDriveInput, twistSpeed); // CORRECTED!  Removed "modified speed" multiplier from this section - made it "modify" twice!

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                break;
        }
        return combinedDriveDone;
    }

    public boolean combinedDriveWithYLine(double desiredHdg, double desiredX, double desiredY, double speed, double error, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {

        double xDriveInput = 0;
        double yDriveInput = 0;

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();

                step = 2;
                break;

            case 1:
                updateNavFromEncodersAfterInitial();
                // Omitting step and break intentionally -->  Goes straight to setting power without needing another loop...

            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                // Using similar triangle rules to get the speeds (xInput and yInput) under 1 with similar triangles
                // The "deltaYNorm" and "deltaXNorm" are turned into "joystick" inputs for field-oriented driving
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                // This lowers speed for small errors...
                if ((Math.abs(distanceRightToMoveX) <= 10 && Math.abs(distanceLeftToMoveY) <= 10)) {
                    modifiedSpeed = 0.10;
                } else {
                    modifiedSpeed = speed;
                }

                absoluteDistanceToMove = Reference.DISTANCE_FORMULA(0,0,distanceRightToMoveX,distanceLeftToMoveY);
                // three sections here
                // DistanceRightToMoveX IS the X Error we want to control corrections using!
                if (Math.abs(distanceLeftToMoveY) > error && Math.abs(distanceLeftToMoveY) < maxError && (absoluteDistanceToMove) > 5) {
                    // This case - more than minimum error, but less than max error - do proportional correction
                    // NOTE - for short distances to "y" target - just use pure pursuit (do not correct to line)
                    // do proportional correction to line = x input = xError / maxError * y input

                    // The following provides proportional nav back to the line (towards y error equals zero)
                    yDriveInput = (distanceLeftToMoveY/maxError) * deltaXNorm * modifiedSpeed; // This one works because deltaY is included (imparts sign)
                    xDriveInput = deltaXNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "proportional correction"; // fix when pure pursuit fixed...


                } else if (Math.abs(distanceLeftToMoveY) > maxError && (absoluteDistanceToMove) > 8) {
                    // make x input = y input
                    // IF error is large - correct back to line using a 45 degree correction = max correction!

                    // This one command a 45 degree (max!) correction - by making X input equal to y input (negative because of conventions)
                    // This needs to correct max!
                    yDriveInput = deltaXNorm * modifiedSpeed * -deltaYNorm / Math.abs(deltaYNorm);
                    xDriveInput = deltaXNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "max correction"; // fix when pure pursuit fixed...

                } else {
                    // do everything the way we did! - Pure pursuit to target

                    // First - See if I'm done!
                    if (DoYouWantTheTurnToFinish) {
                        // If turn completion required - check both turn and distance error
                        if ((Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) && Math.abs(headingError) <= turnError) {
                            // If both distance and angles are done - go to step 3
                            step = 3;
                            break;
                        }
                    } else {
                        // Ignore turn position!
                        if (Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) {
                            //  This is OK...
                            step = 3;
                            break;
                        }
                    }
                    xDriveInput = deltaXNorm * modifiedSpeed; // corrected - had modified speed multiplier in here twice (once here, once later!)
                    yDriveInput = deltaYNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "pure pursuit";
                }

                turnErrorDirection = headingError / Math.abs(headingError);

                // For small errors <10 deg -> ignore turn speed inputs and go slow...
                if (Math.abs(headingError) < 10) {
                    twistSpeed = turnErrorDirection * -0.05;
                } else if (Math.abs(headingError) > turnError) {
                    /** This is where this version is different than "combinedDrive"
                     */
                    twistSpeed = turnErrorDirection * -turnSpeed * 0.5;
                } else {

                    twistSpeed =  0;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(yDriveInput, -xDriveInput, twistSpeed); // FIXED!!  This had double-multiplied "modified speed" (made it .01...)
                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                whatLineCorrectionRegionAmI = "correction complete"; // fix when pure pursuit fixed...
                break;
        }
        return combinedDriveDone;
    }

    public boolean combinedDriveWithCustomPID(double desiredHdg, double desiredX, double desiredY, double speed, double error, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {
                // This was copied from CombinedDriveWithYline
        double xDriveInput = 0;
        double yDriveInput = 0;

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                initialFR = frontRight.getCurrentPosition();
                initialFL = frontLeft.getCurrentPosition();
                initialBR = backRight.getCurrentPosition();
                initialBL = backLeft.getCurrentPosition();

                step = 2;
                break;
            case 1:
                // This case only updates nav (calculated from previous positions based on motor ticks)

                // Current (new) positions for each motor
                fRPosition = frontRight.getCurrentPosition();
                fLPosition = frontLeft.getCurrentPosition();
                bLPosition = backLeft.getCurrentPosition();
                bRPosition = backRight.getCurrentPosition();

                // Capture old position for each motor prior to subtraction
                prevFR = initialFR; prevFL = initialFL; prevBL = initialBL; prevBR = initialBR;

                // Calculate disatance each motor has moved since last time through loop
                distFR = fRPosition - initialFR;
                initialFR = fRPosition;
                distFL = fLPosition - initialFL;
                initialFL = fLPosition;
                distBR = bRPosition - initialBR;
                initialBR = bRPosition;
                distBL = bLPosition - initialBL;
                initialBL = bLPosition;

                // Calculate average drive and strafe distances
                averageDriveDistance = (distFL + distFR + distBL + distBR) / (4 * TICKS_PER_INCH);
                averageStrafeDistance = (-distFL + distFR + -distBR + distBL) / (4 * STRAFE_TICKS_PER_INCH);

                // Distance moved since last cycle
                calculatedXDistance = (averageDriveDistance * Math.cos(Math.toRadians(heading)) - (averageStrafeDistance * Math.sin(Math.toRadians(heading))));
                calculatedYDistance = (averageDriveDistance * Math.sin(Math.toRadians(heading)) + (averageStrafeDistance * Math.cos(Math.toRadians(heading)))); //negative?

                //Update Nav - this statement updates robot current nav position from previous
                nav.tellDeltaXYPositionAfterMove(calculatedXDistance, calculatedYDistance);
                // Omitting step and break intentionally -->  Goes straight to setting power without needing another loop...


            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                // Using similar triangle rules to get the speeds (xInput and yInput) under 1 with similar triangles
                // The "deltaYNorm" and "deltaXNorm" are turned into "joystick" inputs for field-oriented driving
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                // This lowers speed for small errors...
                if ((Math.abs(distanceRightToMoveX) <= 10 && Math.abs(distanceLeftToMoveY) <= 10)) {
                    modifiedSpeed = 0.10;
                } else {
                    modifiedSpeed = speed;
                }

                absoluteDistanceToMove = Reference.DISTANCE_FORMULA(0,0,distanceRightToMoveX,distanceLeftToMoveY);
                // three sections here
                // DistanceRightToMoveX IS the X Error we want to control corrections using!
                if (Math.abs(distanceLeftToMoveY) > error && Math.abs(distanceLeftToMoveY) < maxError && (absoluteDistanceToMove) > 5) {
                    // This case - more than minimum error, but less than max error - do proportional correction
                    // NOTE - for short distances to "y" target - just use pure pursuit (do not correct to line)
                    // do proportional correction to line = x input = xError / maxError * y input

                    // The following provides proportional nav back to the line (towards y error equals zero)
                    yDriveInput = (distanceLeftToMoveY/maxError) * deltaXNorm * modifiedSpeed; // This one works because deltaY is included (imparts sign)
                    xDriveInput = deltaXNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "proportional correction"; // fix when pure pursuit fixed...


                } else if (Math.abs(distanceLeftToMoveY) > maxError && (absoluteDistanceToMove) > 8) {
                    // make x input = y input
                    // IF error is large - correct back to line using a 45 degree correction = max correction!

                    // This one command a 45 degree (max!) correction - by making X input equal to y input (negative because of conventions)
                    // This needs to correct max!
                    // TODO ******* - THIS ONLY CORRECTED IN ONE DIRECTION - needed to be updated to correct in either direction!
                    // TODO - Updated to muliply Y input times the value of the deltaYNorm / absolute value of deltaYnorm to get sign
                    // TODO - needed deltaXNorm to equal its formula - not deltaYNorm
                    // TODO - needed to update X-line version too!!!
                    yDriveInput = deltaXNorm * modifiedSpeed * -deltaYNorm / Math.abs(deltaYNorm);
                    xDriveInput = deltaXNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "max correction"; // fix when pure pursuit fixed...


                } else {
                    // do everything the way we did! - Pure pursuit to target

                    // First - See if I'm done!
                    if (DoYouWantTheTurnToFinish) {
                        // If turn completion required - check both turn and distance error
                        if ((Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) && Math.abs(headingError) <= turnError) {
                            // If both distance and angles are done - go to step 3
                            step = 3;
                            break;
                        }
                    } else {
                        // Ignore turn position!
                        if (Math.abs(distanceRightToMoveX)  <= error && Math.abs(distanceLeftToMoveY) <= error) {
                            //  This is OK...
                            step = 3;
                            break;
                        }
                        // The following line takes the direct inputs for pure pursuit

                    }
                    xDriveInput = deltaXNorm * modifiedSpeed; // corrected - had modified speed multiplier in here twice (once here, once later!)
                    yDriveInput = deltaYNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "pure pursuit";
                }


                turnErrorDirection = headingError / Math.abs(headingError);

                // For small errors <10 deg -> ignore turn speed inputs and go slow...
                if (Math.abs(headingError) < 10) {
                    twistSpeed = turnErrorDirection * -0.05;
                } else if (Math.abs(headingError) > turnError) {
                    /** This is where this version is different than "combinedDrive"
                     */
                    twistSpeed = turnErrorDirection * -turnSpeed * 0.5;
                } else {

                    twistSpeed =  0;
                }


                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(yDriveInput, -xDriveInput, twistSpeed); // FIXED!!  This had double-multiplied "modified speed" (made it .01...)

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                whatLineCorrectionRegionAmI = "correction complete"; // fix when pure pursuit fixed...
                break;
        }

        return combinedDriveDone;
    }



    //TODO see if this needs to be static cause now we can't put this in the hardware class
    public static void setDrivePower(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public boolean combinedDriveWithYLineEthan(double desiredHdg, double desiredX, double desiredY, double speed, double error, double turnSpeed, double turnError, boolean DoYouWantTheTurnToFinish) {

        double xDriveInput = 0;
        double yDriveInput = 0;

        heading = nav.getCorrectedImuHdg();
        headingError = Reference.angleConversion(desiredHdg - heading);
        switch (step) {
            case 0:
                // This step initializes the combined movement variables
                combinedDriveDone = false;
                // These capture wheel positions at the beginning of the movement
                captureInitialEncoderPositions();

                step = 2;
                break;
            case 1:
                updateNavFromEncodersAfterInitial();
                // Omitting step and break intentionally -->  Goes straight to setting power without needing another loop...


            case 2:
                // This case checks if done and sets speed
                // Distance needed to move to finish in each coordinate direction.  Goes directly into robot drive commands.
                distanceLeftToMoveY = desiredY - nav.robotY;
                distanceRightToMoveX = desiredX - nav.robotX;

                // Using similar triangle rules to get the speeds (xInput and yInput) under 1 with similar triangles
                // The "deltaYNorm" and "deltaXNorm" are turned into "joystick" inputs for field-oriented driving
                legs = Math.pow(distanceRightToMoveX, 2) + Math.pow(distanceLeftToMoveY, 2);
                deltaYNorm = distanceLeftToMoveY / Math.sqrt(legs);
                deltaXNorm = distanceRightToMoveX / Math.sqrt(legs);
                // These two numbers - deltaYNorm and deltaXNorm produce the X,Y inputs for "pure pursuit" guidance - direct from present position to target

                // This lowers speed for small errors...
                if ((Math.abs(distanceRightToMoveX) <= 5 && Math.abs(distanceLeftToMoveY) <= 5)) {
                    modifiedSpeed = 0.10;
                } else {
                    modifiedSpeed = speed;
                }

                absoluteDistanceToMove = Reference.DISTANCE_FORMULA(0,0,distanceRightToMoveX,distanceLeftToMoveY);
                // three sections here
                // DistanceRightToMoveX IS the X Error we want to control corrections using!
                if (Math.abs(distanceLeftToMoveY) > error && Math.abs(distanceLeftToMoveY) < maxError && (absoluteDistanceToMove) > 5) {
                    // This case - more than minimum error, but less than max error - do proportional correction
                    // NOTE - for short distances to "y" target - just use pure pursuit (do not correct to line)
                    // do proportional correction to line = x input = xError / maxError * y input

                    // The following provides proportional nav back to the line (towards y error equals zero)
                    yDriveInput = (distanceLeftToMoveY/maxError) * deltaXNorm * modifiedSpeed; // This one works because deltaY is included (imparts sign)
                    xDriveInput = deltaXNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "proportional correction"; // fix when pure pursuit fixed...

                } else if (Math.abs(distanceLeftToMoveY) > maxError && (absoluteDistanceToMove) > 8) {
                    // make x input = y input
                    // IF error is large - correct back to line using a 45 degree correction = max correction!

                    // This one command a 45 degree (max!) correction - by making X input equal to y input (negative because of conventions)
                    // This needs to correct max!
                    yDriveInput = deltaXNorm * modifiedSpeed * -deltaYNorm / Math.abs(deltaYNorm);
                    xDriveInput = deltaXNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "max correction"; // fix when pure pursuit fixed...

                } else {
                    // do everything the way we did! - Pure pursuit to target

                    // First - See if I'm done!
                    if (DoYouWantTheTurnToFinish) {
                        // If turn completion required - check both turn and distance error
                        if ((Math.abs(distanceRightToMoveX)  <= (3*error) && Math.abs(distanceLeftToMoveY) <= error) && Math.abs(headingError) <= turnError) {
                            // If both distance and angles are done - go to step 3
                            step = 3;
                            break;
                        }
                    } else {
                        // Ignore turn position!
                        if (Math.abs(distanceRightToMoveX)  <= (3*error) && Math.abs(distanceLeftToMoveY) <= error) {
                            //  This is OK...
                            step = 3;
                            break;
                        }
                    }
                    xDriveInput = deltaXNorm * modifiedSpeed; // corrected - had modified speed multiplier in here twice (once here, once later!)
                    yDriveInput = deltaYNorm * modifiedSpeed;

                    whatLineCorrectionRegionAmI = "pure pursuit";
                }

                turnErrorDirection = headingError / Math.abs(headingError);
                if (Math.abs(headingError) < turnError) {
                    twistSpeed = 0;
                } else if (Math.abs(headingError) < 2) {
                    twistSpeed = turnErrorDirection * .0075;
                } else if (Math.abs(headingError) < 3) {
                    twistSpeed = turnErrorDirection * 0.01;
                } else if (Math.abs(headingError) < 4) {
                    twistSpeed = turnErrorDirection * 0.0125;
                } else if (Math.abs(headingError) < 6) {
                    twistSpeed = turnErrorDirection * 0.015;
                } else if (Math.abs(headingError) < 10) {
                    twistSpeed = turnErrorDirection * 0.25;
                } else if (Math.abs(headingError) < 20) {
                    twistSpeed = turnErrorDirection * 0.0375;
                } else {
                    twistSpeed = turnErrorDirection * turnSpeed;
                }

                // This is the actual command to set the robot speeds - uses 2 coordinates - (strafe inputs, drive inputs)
                mecanumMovementFieldOrientedControl(yDriveInput, -xDriveInput, twistSpeed); // FIXED!!  This had double-multiplied "modified speed" (made it .01...)

                step = 1;
                break;
            case 3:
                combinedDriveDone = true;
                setDrivePower(0);
                step = 0; // MrE Added this so it will work next time!
                whatLineCorrectionRegionAmI = "correction complete"; // fix when pure pursuit fixed...
                break;
        }
        return combinedDriveDone;
    }

    //TODO see if this needs to be static cause now we can't put this in the hardware class
    public static void setDrivePower(double fr, double fl, double br, double bl) {

        frontRight.setPower(fr);
        frontLeft.setPower(fl);
        backRight.setPower(br);
        backLeft.setPower(bl);
    }

    // Method to be able to manually adjust the speed of each motor while strafing with customization
    public static void setCustomMove(double angle, double speed, double backRightSpeedCorrection, double backLeftSpeedCorrection,
                                     double frontRightSpeedCorrection, double frontLeftSpeedCorrection) {
        double speedRight = 0;
        double speedLeft = 0;

        if (angle == 0) {
            speedRight = speed;
            speedLeft = speed;
        }

        if (angle == 90) {
            speedRight = -speed;
            speedLeft = speed;
        }

        if (angle == 180) {
            speedRight = -speed;
            speedLeft = -speed;
        }

        if (angle == 270) {
            speedRight = speed;
            speedLeft = -speed;
        }

        setDrivePower(speedRight + frontRightSpeedCorrection, speedLeft + frontLeftSpeedCorrection,
                speedLeft + backRightSpeedCorrection, speedRight + backRightSpeedCorrection);
    }

    public void setTurnPower(double speed) {
        double speedRight = -speed;
        setDrivePower(speedRight, speed, speedRight, speed);
    }

    public void setRightTurnPower(double speed) {
        setDrivePower(0, speed, 0, speed);
    }
    public void setLeftTurnPower(double speed) {
        setDrivePower(speed, 0, speed, 0);
    }

    // Takes the shortest turn from where you are to where you want to go
    public static double shortTurn(double hdg1, double hdg2) {
        turn1 = hdg2 - hdg1;
        turn2 = hdg2 - hdg1 - (math.getSign(hdg2 - hdg1) * 360);
        if (Math.abs(turn1) < Math.abs(turn2)) {
            turn = turn1;
        } else {
            turn = turn2;
        }

        return turn;
    }

    //-----------------------------------------------------------------------------------------------------------------------

    //---------------------- GENERIC DRIVE FUNCTIONS, SUCH AS RESET DRIVE ENCODERS AND SET DRIVE POWER ----------------------

    public void resetDriveEncoders() {
        /* Reset the encoder values to zero(
         * i.e: frontRight.getCurrentPosition() will return 0)
         */
        if (frontLeft != null) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (frontRight != null) {
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backLeft != null) {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backRight != null) {
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        setDrivePower(0);
    }


    public void resetTicksBL() {
        currentTicks = backLeft.getCurrentPosition();
        referenceTicks = currentTicks;
    }

    public static double HIGHEST_TICKS() {
        double bestDistance = math.FIND_HIGH_NUMBER(backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        bestDistance = math.FIND_HIGH_NUMBER(bestDistance, frontRight.getCurrentPosition());
        bestDistance = math.FIND_HIGH_NUMBER(bestDistance, frontLeft.getCurrentPosition());
        return bestDistance;
    }

    public boolean strafeTo(Reference.Strafe direction, double linearSpeed, double distance) {
        boolean bBackwards = false;
        boolean bRight = false;
        boolean bLeft = false;

        switch (direction) {
            case REVERSE:
                bBackwards = true;
                break;
            case LEFT:
                bLeft = true;
                break;
            case RIGHT:
                bRight = true;
                break;
        }

        if (moveToPosition(bBackwards ? 180 : bLeft ? 270 : bRight ? 90 : 0, distance,
                linearSpeed, 0)) {
            //this assumes achievement of final position, not incremental updates
            //further consideration required if incremental updates are to be implemented
            nav.tellFinalPosition(direction, distance);
            return true;
        } else {
            // TODO incorporate intermediate nav update here based on ticks complete
            return false;
        }
    }

  // works
    public boolean accelStrafeTo(Reference.Strafe direction, double linearSpeed, double distance) {
        accelLoop++;
        minPower = 0.1;
        maxPower = linearSpeed;
        if (accelLoop == 1) {
            resetDriveEncoders();
            if(direction == Reference.Strafe.REVERSE){ // reverse = 2
                accelTicksPerInch = TICKS_PER_INCH;
                frontRightPower = -1;
                frontLeftPower = -1;
                backRightPower = -1;
                backLeftPower = -1;
            }else if (direction == Reference.Strafe.LEFT){ // left = 3
                accelTicksPerInch = STRAFE_TICKS_PER_INCH;
                frontRightPower = 1;
                frontLeftPower = -1;
                backRightPower = -1;
                backLeftPower = 1;
            }else if (direction == Reference.Strafe.RIGHT){ // right = 4
                accelTicksPerInch = STRAFE_TICKS_PER_INCH;
                frontRightPower = -1;
                frontLeftPower = 1;
                backRightPower = 1;
                backLeftPower = -1;
            }else { // forward
                accelTicksPerInch = TICKS_PER_INCH;
                frontRightPower = 1;
                frontLeftPower = 1;
                backRightPower = 1;
                backLeftPower = 1;
            }
            currentPower = 0;
            accelDone = false;
        } else {
            distanceGone = Math.abs(backRight.getCurrentPosition() / accelTicksPerInch);
            distanceToGo = distance - distanceGone;
            if (distanceToGo <= 0) {
                currentPower = 0;
                //setDrivePower(currentPower);
                accelLoop = 0;
                accelDone = true;
            } else {
                if (currentPower == 0) {
                    currentPower = minPower;
                    //setDrivePower(currentPower);
                } else {
                    if (distanceToGo <= (ACCEL_K * (Math.pow(currentPower, 2) / DELTA_POWER_DECEL) + DISTANCE_TO_CORRECT)) {
                        currentPower = currentPower - DELTA_POWER_DECEL;
                        // decelerating
                        if (currentPower <= minPower) {
                            currentPower = minPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    } else {
                        currentPower = currentPower + DELTA_POWER_ACCEL;
                        // accelerating
                        if (currentPower >= maxPower) {
                            currentPower = maxPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    }
                }
            }
        }
        setDrivePower(currentPower * frontRightPower, currentPower * frontLeftPower,
                        currentPower * backRightPower,currentPower * backLeftPower);
        return accelDone;
    }

    public boolean accelStrafeToCustomLeft(Reference.Strafe direction, double linearSpeed, double distance, double strafeValue) {
        accelLoop++;
        minPower = 0.1;
        maxPower = linearSpeed;
        if (accelLoop == 1) {
            resetDriveEncoders();
            if(direction == Reference.Strafe.REVERSE){
                accelTicksPerInch = TICKS_PER_INCH;
                frontRightPower = -1;
                frontLeftPower = -1;
                backRightPower = -1;
                backLeftPower = -1;
            }else if (direction == Reference.Strafe.LEFT){
                accelTicksPerInch = STRAFE_TICKS_PER_INCH;
                frontRightPower = 1;
                frontLeftPower = -1;
                backRightPower = -1;
                backLeftPower = 1;
            }else if (direction == Reference.Strafe.RIGHT){
                accelTicksPerInch = STRAFE_TICKS_PER_INCH;
                frontRightPower = -1;
                frontLeftPower = 1;
                backRightPower = 1;
                backLeftPower = -1;
            }else {
                accelTicksPerInch = TICKS_PER_INCH;
                frontRightPower = 1;
                frontLeftPower = 1;
                backRightPower = 1;
                backLeftPower = 1;
            }
            currentPower = 0;
            accelDone = false;
        } else {
            distanceGone = Math.abs(backRight.getCurrentPosition() / accelTicksPerInch);
            distanceToGo = distance - distanceGone;
            if (distanceToGo <= 0) {
                currentPower = 0;
                //setDrivePower(currentPower);
                accelLoop = 0;
                accelDone = true;
            } else {
                if (currentPower == 0) {
                    currentPower = minPower;
                    //setDrivePower(currentPower);
                } else {
                    if (distanceToGo <= (ACCEL_K * (Math.pow(currentPower, 2) / DELTA_POWER_DECEL) + DISTANCE_TO_CORRECT)) {
                        currentPower = currentPower - DELTA_POWER_DECEL;
                        // decelerating
                        if (currentPower <= minPower) {
                            currentPower = minPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    } else {
                        currentPower = currentPower + DELTA_POWER_ACCEL;
                        // accelerating
                        if (currentPower >= maxPower) {
                            currentPower = maxPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    }
                }
            }
        }

        setDrivePower(currentPower * (frontRightPower - strafeValue), currentPower * (frontLeftPower + strafeValue),
                currentPower * (backRightPower - strafeValue),currentPower * (backLeftPower + strafeValue));
        return accelDone;
    }


    public boolean turnAndGoTo(double linearSpeed, double xPosition, double yPosition) {
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        double finalHeading = Reference.ANGLE_FORMULA(currentX, currentY, xPosition, yPosition);
        double turnSpeed = 0.6;
        boolean bBackwards = false;
        boolean bRight = false;
        boolean bLeft = false;
        // was finalHeading
        if (pointToPointNavigation(currentX, currentY, xPosition, yPosition, nav.getHeading(), nav.getInitialHeading(),
                turnSpeed, linearSpeed, period.time(TimeUnit.SECONDS), bBackwards, bRight, bLeft)) {
            nav.tellFinalPosition(xPosition, yPosition); //this assumes achievement of final position, not incremental updates
            //further consideration required if incremental updates are to be implemented
            return true;
        } else {
            // TODO incorporate intermediate nav update here based on ticks complete
            return false;
        }
        // TODO final goal is 360 degree straight line movement with blended turning while traveling
        // TODO Ezra has enlisted Julia (mathematics major) to come help the team with the math for blended motion
    }

    //works
    public boolean turnAndGoTo(Reference.Strafe direction, double linearSpeed, double xPosition, double yPosition) {
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        double finalHeading = Reference.ANGLE_FORMULA(currentX, currentY, xPosition, yPosition);
        double turnSpeed = 0.6;
        double distance = Reference.DISTANCE_FORMULA(currentX, currentY, xPosition, yPosition);

        switch (goStep) {
            case 0:
                resetDriveEncoders();
                goDone = false;
                goStep++;
                break;
            case 1:
                if (imuTurnTo(direction, turnSpeed, xPosition, yPosition)) {
                    goStep++;
                }
                break;
            case 2:
                if (strafeTo(direction, linearSpeed, distance)) {
                    nav.tellFinalPosition(xPosition, yPosition);
                    goStep++;
                }
                break;
            case 3:
                resetDriveEncoders();
                goStep = 0;
                goDone = true;
                break;
        }
        return goDone;
    }

    //works
    public boolean turnAndAccelTo(Reference.Strafe direction, double linearSpeed, double xPosition, double yPosition) {
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        double finalHeading = Reference.ANGLE_FORMULA(currentX, currentY, xPosition, yPosition);
        double turnSpeed = 0.6;
        double minSpeed = 0.1;
        double distance = Reference.DISTANCE_FORMULA(currentX, currentY, xPosition, yPosition);

        switch (goStep) {
            case 0:
                resetDriveEncoders();
                goDone = false;
                goStep++;
                break;
            case 1:
                if (imuTurnTo(direction, turnSpeed, xPosition, yPosition)) {
                    goStep++;
                }
                break;
            case 2:
                if (accelPowerGoes(distance, linearSpeed, minSpeed)) {
                    nav.tellFinalPosition(xPosition, yPosition);
                    goStep++;
                }
                break;
            case 3:
                resetDriveEncoders();
                goStep = 0;
                goDone = true;
                break;
        }
        return goDone;
    }

    //this now works, yay!
    public boolean accelTurnAndAccelTo(Reference.Strafe direction, double linearSpeed, double xPosition, double yPosition) {
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        double finalHeading = Reference.ANGLE_FORMULA(currentX, currentY, xPosition, yPosition);
        double turnSpeed = 0.7;
        double minSpeed = 0.1;
        double distance = Reference.DISTANCE_FORMULA(currentX, currentY, xPosition, yPosition);

        switch (goStep) {
            case 0:
                resetDriveEncoders();
                goDone = false;
                goStep++;
                break;
            case 1:
                if (accelTurnTo(direction, turnSpeed, xPosition, yPosition)) {
                    goStep++;
                }
                break;
            case 2:
                if (accelPowerGoes(distance, linearSpeed, minSpeed)) {
                    nav.tellFinalPosition(xPosition, yPosition);
                    goStep++;
                }
                break;
            case 3:
                resetDriveEncoders();
                goStep = 0;
                goDone = true;
                break;
        }
        return goDone;
    }

    public boolean turnTo(double turnSpeed, double pointToX, double pointToY) {
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        // TODO - This calculation
        double finalHeading = Reference.ANGLE_FORMULA(currentX, currentY, pointToX, pointToY);
        boolean bBackwards = false;
        boolean bRight = false;
        boolean bLeft = false;
        // was finalHeading,
        if (pointToPointNavigationTurn(currentX, currentY, pointToX, pointToY, nav.getHeading(), nav.getInitialHeading(), turnSpeed)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean turnTo(Reference.Strafe direction, double turnSpeed, double pointToX, double pointToY) {
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        // the direction the front of the robot should point if it were told to turn to the given point
        // the front of the robot is the side with the intake system, not the camera (that's the rear)
        // switch on step state for turnTo
        switch (turnTonavStep) {
            case 0:
                resetDriveEncoders();
                hdg1 = nav.getHeading();
                calculatedFrontDirection = Reference.ANGLE_FORMULA(currentX, currentY, pointToX, pointToY);
                // set this to default to "front".
                desiredFrontDirection = calculatedFrontDirection;
                turnToDone = false;
                turnTonavStep++;
                break;
            case 1:
                // If Strafe direction other than FORWARD selected, then calculate the desired heading
                // for where the front of the robot should be pointed
                if (direction == Reference.Strafe.REVERSE) {
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 180);
                    turnTonavStep++;
                } // i don't know why, but left and right directions were switched, so i switched them (Nathaniel)
                // they currently work now
                else if (direction == Reference.Strafe.LEFT) {
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 90);
                    turnTonavStep++;
                } else if (direction == Reference.Strafe.RIGHT) {
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 270);
                    turnTonavStep++;
                }
                break;
            case 2:
// TODO - Copied from inside turn correction method...  Probably better to cut that section out as a separate function...
                if (desiredFrontDirection != 0) {
                    turnTonavStep = -1; // TODO put in
                } else {
                    turnTonavStep = 4; // TODO FIX
                }
                break;
            case -1:
                if (turnDistance(-Reference.turnAngle(hdg1, desiredFrontDirection), turnSpeed)) {//  removed negative o reference angle
                    turnTonavStep = 3;
                }
                break;
            case 3:
                double startingHeading = nav.getInitialHeading();
                if (imuTurnCorrection(startingHeading, desiredFrontDirection)) {
                    turnTonavStep = 4;
                }
                break;
            case 4:
                resetDriveEncoders();
                turnTonavStep = 0;
                turnToDone = true;
                break;

            default:
                // this enforces that turnTonavStep is only a value of 0, 1, 2, or 3
                throw new IllegalStateException("Unexpected value: " + turnTonavStep);
        }
        return turnToDone;
    }

    //made to get rid of the turns turning the wrong direction; THIS WORKS, YAY!!!!
    // TODO make faster and put where it needs to be
    public boolean imuTurnTo(Reference.Strafe direction, double turnSpeed, double pointToX, double pointToY){
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();

        switch (turnTonavStep) {
            case 0:
                resetDriveEncoders();
                hdg1 = nav.getHeading();
                calculatedFrontDirection = Reference.ANGLE_FORMULA(currentX, currentY, pointToX, pointToY);
                // set this to default to "front".
                desiredFrontDirection = calculatedFrontDirection;
                turnToDone = false;
                turnTonavStep++;
                break;
            case 1:
                // If Strafe direction other than FORWARD selected, then calculate the desired heading
                // for where the front of the robot should be pointed
                if (direction == Reference.Strafe.REVERSE) { // reverse
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 180);
                    turnTonavStep++;
                } // i don't know why, but left and right directions were switched, so i switched them (Nathaniel)
                // they currently work now
                else if (direction == Reference.Strafe.LEFT) { // left
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 90);
                    turnTonavStep++;
                }
                else if(direction == Reference.Strafe.RIGHT){ // right
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 270);
                    turnTonavStep++;
                }
                else if(direction == Reference.Strafe.FORWARD){ // forward
                    desiredFrontDirection = calculatedFrontDirection;
                    turnTonavStep++;
                }
                break;
            case 2:
                if (desiredFrontDirection != hdg1) {
                    turnTonavStep  = -1;
                } else {
                    turnTonavStep = 4;
                }
                break;
            case -1:
                //if (turnDistance(-Reference.turnAngle(hdg1, desiredFrontDirection), turnSpeed)) {//  removed negative o reference angle
                imuHdg = nav.getHeading();
                correctedImuHdg = imuHdg;
                convertedImuHdg = Reference.angleConversion(correctedImuHdg);
                turn = shortTurn(convertedImuHdg, desiredFrontDirection);
                turnTonavStep--;
                break;
            case -2:
                if(turn < 0){
                    setTurnPower(turnSpeed); //TODO eventually make this use accel
                    turnTonavStep--;
                }else {
                    setTurnPower(-turnSpeed); //TODO eventually make this use accel
                    turnTonavStep--;
                }

                break;
            case -3:
                if ((Math.abs(backRight.getCurrentPosition()) / TICKS_PER_DEGREE) >= Math.abs(turn)) {
                    setDrivePower(0);
                    turnTonavStep  = 3;
                }
            case 3:
                double startingHeading = nav.getInitialHeading();
                if (imuTurnCorrection(startingHeading, desiredFrontDirection)) {
                    turnTonavStep = 4;
                }
                break;
            case 4:
                resetDriveEncoders();
                turnTonavStep = 0;
                turnToDone = true;
                break;

            default:
                // this enforces that turnTonavStep is only a value of 0, 1, 2, or 3
                throw new IllegalStateException("Unexpected value: " + turnTonavStep);
        }
        return turnToDone;
    }

    //works
    public boolean accelTurnTo(Reference.Strafe direction, double turnSpeed, double pointToX, double pointToY){
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();

        switch (turnTonavStep) {
            case 0:
                resetDriveEncoders();
                hdg1 = nav.getHeading();
                calculatedFrontDirection = Reference.ANGLE_FORMULA(currentX, currentY, pointToX, pointToY);
                // set this to default to "front".
                desiredFrontDirection = calculatedFrontDirection;
                turnToDone = false;
                turnTonavStep++;
                break;
            case 1:
                // If Strafe direction other than FORWARD selected, then calculate the desired heading
                // for where the front of the robot should be pointed
                if (direction == Reference.Strafe.REVERSE) {
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 180);
                    turnTonavStep++;
                } // i don't know why, but left and right directions were switched, so i switched them (Nathaniel)
                // they currently work now
                else if (direction == Reference.Strafe.LEFT) {
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 90);
                    turnTonavStep++;
                }
                else if(direction == Reference.Strafe.RIGHT){
                    desiredFrontDirection = Reference.angleConversion(calculatedFrontDirection + 270);
                    turnTonavStep++;
                }
                else if(direction == Reference.Strafe.FORWARD){
                    desiredFrontDirection = calculatedFrontDirection;
                    turnTonavStep++;
                }
                break;
            case 2: //TODO test - was desiredFrontDirection != 0 (didn't work)
                if (desiredFrontDirection != hdg1) {
                    turnTonavStep  = -1;
                } else {
                    turnTonavStep = 4;
                }
                break;
            case -1:
                //if (turnDistance(-Reference.turnAngle(hdg1, desiredFrontDirection), turnSpeed)) {//  removed negative o reference angle
                imuHdg = nav.getHeading();
                correctedImuHdg = imuHdg;
                convertedImuHdg = Reference.angleConversion(correctedImuHdg);
                turn = shortTurn(convertedImuHdg, desiredFrontDirection);
                turnTonavStep--;
                break;
            case -2:
                accelGear = accelPower(turn * TICKS_PER_DEGREE / TICKS_PER_INCH, turnSpeed, 0.1);
                accelTurnLoop++;
                if(accelTurnLoop > 1){
                    if(turn < 0){
                        setTurnPower(turnSpeed * accelGear);
                        if (turnSpeed * accelGear != 0){
                            turnTonavStep = -2;
                        }else {
                            turnTonavStep--;
                        }
                    }else {
                        setTurnPower(-turnSpeed * accelGear);
                        if (-turnSpeed * accelGear != 0){
                            turnTonavStep = -2;
                        }else {
                            turnTonavStep--;
                        }
                    }
                }else {
                    setTurnPower(0.1);
                    turnTonavStep = -2;
                }
                break;
            case -3:
                if ((Math.abs(backRight.getCurrentPosition()) / TICKS_PER_DEGREE) >= Math.abs(turn)) {
                    setDrivePower(0);
                    turnTonavStep  = 3;
                }
            case 3:
                double startingHeading = nav.getInitialHeading();
                if (imuTurnCorrection(startingHeading, desiredFrontDirection)) {
                    turnTonavStep = 4;
                }
                break;
            case 4:
                resetDriveEncoders();
                turnTonavStep = 0;
                accelLoop = 0;
                accelTurnLoop = 0;
                turnToDone = true;
                break;

            default:
                // this enforces that turnTonavStep is only a value of defined cases
                throw new IllegalStateException("Unexpected value: " + turnTonavStep);
        }
        return turnToDone;
    }

    public boolean accelPowerGoes(double distance, double maxPower, double minPower) {

        accelLoop++;
        if (accelLoop == 1) {
            resetDriveEncoders();
            currentPower = 0;
            accelDone = false;
        } else {
            distanceGone = backRight.getCurrentPosition() / TICKS_PER_INCH;
            distanceToGo = distance - distanceGone;
            if (distanceToGo <= 0) {
                currentPower = 0;
                //setDrivePower(currentPower);
                accelLoop = 0;
                accelDone = true;
            } else {
                if (currentPower == 0) {
                    currentPower = minPower;
                    //setDrivePower(currentPower);
                } else {
                    if (distanceToGo <= (ACCEL_K * (Math.pow(currentPower, 2) / DELTA_POWER_DECEL) + DISTANCE_TO_CORRECT)) {
                        currentPower = currentPower - DELTA_POWER_DECEL;
                        // decelerating
                        if (currentPower <= minPower) {
                            currentPower = minPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    } else {
                        currentPower = currentPower + DELTA_POWER_ACCEL;
                        // accelerating
                        if (currentPower >= maxPower) {
                            currentPower = maxPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    }
                }
            }
        }

        setDrivePower(currentPower);
        return accelDone;
    }

    public double accelPower(double distance, double maxPower, double minPower) {

        accelLoop++;
        if (accelLoop == 1) {
            resetDriveEncoders();
            currentPower = 0;
            accelDone = false;
        } else {
            distanceGone = backRight.getCurrentPosition() / TICKS_PER_INCH;
            distanceToGo = distance - distanceGone;
            if (distanceToGo <= 0) {
                currentPower = 0;
                //setDrivePower(currentPower);
                accelLoop = 0;
                accelDone = true;
            } else {
                if (currentPower == 0) {
                    currentPower = minPower;
                    //setDrivePower(currentPower);
                } else {
                    if (distanceToGo <= (ACCEL_K * (Math.pow(currentPower, 2) / DELTA_POWER_DECEL) + DISTANCE_TO_CORRECT)) {
                        currentPower = currentPower - DELTA_POWER_DECEL;
                        // decelerating
                        if (currentPower <= minPower) {
                            currentPower = minPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    } else {
                        currentPower = currentPower + DELTA_POWER_ACCEL;
                        // accelerating
                        if (currentPower >= maxPower) {
                            currentPower = maxPower;
                            //setDrivePower(currentPower);
                        } else {
                            //setDrivePower(currentPower);
                        }
                    }
                }
            }
        }
        return currentPower;
    }

    //author: Nathaniel Cavallaro
    //This method moves the robot from a given set of coordinates to another set of coordinates on the field.
    //It gives the option to strafe sideways or move backwards instead of just forwards.
    //There are several versions of this method. This is the last one (or at least one of the later ones).
    private boolean pointToPointNavigation(double x1, double y1, double x2, double y2, double hdg1,
                                           double initialHdg1, double speedAngle, double speedStraight, double runTime,
                                           boolean backwards, boolean strafeRight, boolean strafeLeft) {

        switch (navStep) {

            case 0:
                resetDriveEncoders();
                //initialHdg = nav.getHeading(); was uncommented
                pointIsDone = false;
                navStep++;
                break;
            case 1:
                if (backwards) {
                    navStep = 3;
                } else if (strafeRight) {
                    navStep = 3;
                } else if (strafeLeft) {
                    navStep = 3;
                } else {
                    if (Reference.turnAngle(hdg1, Reference.ANGLE_FORMULA(x1, y1, x2, y2)) != 0) {
                        navStep = -1;
                    } else {
                        navStep = 3;
                    }
                }
                break;
            case -1:
                hdg2 = Reference.ANGLE_FORMULA(x1, y1, x2, y2);
                if (turnDistance(-Reference.turnAngle(hdg1, hdg2), speedAngle)) {
                    navStep = 2;
                }
                break;
            case 2:
                initHeading = initialHdg1;// these should not do anything
                expectedHdg = hdg2;// these should not do anything
                if (imuTurnCorrection(initialHdg1, hdg2)) { //was hdg2
                    navStep = 3;
                }
                break;
            case 3:
                navStep++;
                break;
            case 4:
                if (moveToPosition(backwards ? 180 : strafeLeft ? 270 : strafeRight ? 90 : 0, Reference.DISTANCE_FORMULA(x1, y1, x2, y2),
                        speedStraight, runTime)) {
                    navStep++;
                }
                break;
            case 5:
                resetDriveEncoders();
                navStep = 0;
                pointIsDone = true;
                break;
        }
        return pointIsDone;
    }
    //author: Nathaniel Cavallaro
    //This method performs the turn to a given set of coordinates on the field.
    private boolean pointToPointNavigationTurn(double x1, double y1, double x2, double y2, double hdg1, double initialHdg1, double speedAngle) {

        //point to point nav method variables

        switch (navStep) {

            case 0:
                resetDriveEncoders();
                pointIsDone = false;
                navStep++;
                break;
            case 1:
                if (Reference.turnAngle(hdg1, Reference.ANGLE_FORMULA(x1, y1, x2, y2)) != 0) {
                    navStep = -1;
                } else {
                    navStep = 3;
                }
                break;
            case -1:
                hdg2 = Reference.ANGLE_FORMULA(x1, y1, x2, y2);
                if (turnDistance(-Reference.turnAngle(hdg1, hdg2), speedAngle)) {
                    navStep = 2;
                }
                break;
            case 2:
                initHeading = initialHdg1; // these should not really do anything
                expectedHdg = hdg2; // these should not really do anything
                if (imuTurnCorrection(initialHdg1, hdg2)) { //was hdg2
                    navStep = 3;
                }
                break;
            case 3:
                resetDriveEncoders();
                navStep = 0;
                pointIsDone = true;
                break;
        }
        return pointIsDone;
    }

    private double encoderStart = 0;
    
    //author: Nathaniel Cavallaro
    //This method corrects each turn using the imu (inertial measurement unit) for greater accuracy.
    private boolean imuTurnCorrection(double initialHdg, double expectedHdg) {

        switch (correctionStep) {
            case 0:
//                resetDriveEncoders();
                encoderStart = encoderStart;
                encoderStart = backRight.getCurrentPosition();
                correctionDone = false;
                correctionStep++;
                break;

            case 1:
                imuHdg = nav.getHeading();
                correctedImuHdg = imuHdg + initialHdg;
                convertedImuHdg = Reference.angleConversion(correctedImuHdg);
                correctionTurn = shortTurn(convertedImuHdg, expectedHdg);
                correctionStep++;
                break;
            case 2:
                if (correctionTurn >= 0.7) {
                    switch (correctionStep2) {
                        case 0:
                            if (correctionTurn >= 15) {
                                setTurnPower(-0.25);//-0.3
                            } else {
                                setTurnPower(-0.09);// was -0.1 - Nathaniel changed because motors are now faster, needs to be tested
                            }
                            correctionStep2++;
                            break;
                        case 1://ticksSinceResetBL()
                            if ((Math.abs(backRight.getCurrentPosition() - encoderStart) / CORRECTION_TICKS_PER_DEGREE) >= Math.abs(correctionTurn)) {
                                correctionStep2++;
                            }
                            break;
                        case 2:
                            setDrivePower(0);
                            correctionStep = 0;
                            correctionStep2 = 0;
                            correctionStep3 = 0;
                            break;
                    }
                } else if (correctionTurn <= -0.7) {
                    switch (correctionStep3) {
                        case 0:
                            if (correctionTurn <= -15) {
                                setTurnPower(0.25);//0.3
                            } else {
                                setTurnPower(0.075);//0.1
                            }
                            correctionStep3++;
                            break;
                        case 1://ticksSinceResetBL()
                            if ((Math.abs(backRight.getCurrentPosition() - encoderStart) / CORRECTION_TICKS_PER_DEGREE) >= Math.abs(correctionTurn)) {
                                correctionStep3++;
                            }
                            break;
                        case 2:
                            setDrivePower(0);
                            correctionStep = 0;
                            correctionStep2 = 0;
                            correctionStep3 = 0;
                            break;
                    }
                } else {
                    correctionDone = true;
                    setDrivePower(0);
//                    resetDriveEncoders();
                    correctionStep = 0;
                    correctionStep2 = 0;
                    correctionStep3 = 0;
                }
                break;
        }
        return correctionDone;
    }

    public boolean turnDistance(double angle, double speed) {
        switch (goStep) {
            case 0:
                if (angle < 0) {
                    setTurnPower(-speed);
                } else {
                    setTurnPower(speed);
                }
                goStep++;
                break;
            case 1:
                if (turnDegrees(angle)) {
                    goStep++;
                }
                break;
            case 2:
                resetDriveEncoders();
                goStep = 0;
                goDone = true;
                break;

        }
        return goDone;
    }

    public boolean turn2WheelDistance(double angle, double speed) {
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        switch (goStep) {
            case 0:
                goDone = false;
                if (angle < 0) {
                    setLeftTurnPower(speed);
                } else {
                    setRightTurnPower(speed);
                }
                goStep++;
                break;
            case 1:
                if (turnDegrees(2*angle)) {
                    goStep++;
                }
                break;
            case 2:
                resetDriveEncoders();
                //TODO fix
                //this only states the positon before the turn
                nav.tellFinalPosition(currentX, currentY);
                goStep = 0;
                goDone = true;
                break;

        }
        return goDone;
    }
    public boolean fourWheelIMUTurn(double targetHeading, double speed, double error) {

        double currentHeading = nav.getCorrectedImuHdg();
        double turning = shortTurn(currentHeading, targetHeading);
        switch (goStep) {
            case 0:
                goDone = false;
                goStep++;
            case 1:
                if (Math.abs(shortTurn(currentHeading, targetHeading)) < error) {
                    setDrivePower(0, 0, 0, 0);
                    resetDriveEncoders();
                    goStep = 0;
//                    nav.tellFinalPosition(currentX, currentY);
                    goDone = true;
                    break;
                } else if (turning < 0) {
                    if (speed > 0) {
                        setTurnPower(speed);
                    } else {
                        setTurnPower(-speed);
                    }
                } else {
                    if (speed > 0) {
                        setTurnPower(-speed);
                    } else {
                        setTurnPower(speed);
                    }
                }
        }
        return goDone;
    }
    public boolean twoWheelIMUTurn(double targetHeading, double speed, double error) {

        double currentHeading = nav.getCorrectedImuHdg();
        double currentX = nav.getRobotX();
        double currentY = nav.getRobotY();
        double turning = shortTurn(currentHeading, targetHeading);
        switch (goStep) {
            case 0:
                goDone = false;

                goStep++;
            case 1:
                if (Math.abs(shortTurn(currentHeading, targetHeading)) < error) {
                    setDrivePower(0, 0, 0, 0);
                    resetDriveEncoders();
                    goStep = 0;
                    nav.tellFinalPosition(currentX, currentY);
                    goDone = true;
                    break;
                } else if (turning < 0) {
                    if (speed > 0) {
                        setRightTurnPower(speed);
                    } else setLeftTurnPower(speed);
                } else {
                    if (speed > 0) {
                        setLeftTurnPower(speed);
                    } else setRightTurnPower(speed);
                }
        }

        return goDone;
    }

    private static boolean turnDegrees(double angle) {

        //defines the distance the motors have traveled
        double dist = Math.abs(backLeft.getCurrentPosition());
        /*
        this section compares the motor encoder positions to find the
        highest value of the encoders
        */
        dist = Math.max(dist, Math.abs(backRight.getCurrentPosition()));
        dist = Math.max(dist, Math.abs(backRight.getCurrentPosition()));
        dist = Math.max(dist, Math.abs(backRight.getCurrentPosition()));
        /*
        return whether or not the ticks the encoder has divided
        by the ticks per degree (converting the encoder ticks to
        degrees) is greater than the angle the user has inputted.
        */
        return Math.abs(dist) / TICKS_PER_DEGREE > Math.abs(angle);
    }

    double speedRight = 0;
    double speedLeft = 0;

    private boolean moveToPosition(double angle, double distance, double speed, double runTime) {

        switch (moveStep) {
            case 0:
                //set target powers
                if (angle == 0) {
                    speedRight = speed;
                    speedLeft = speed;
                    targetPos = distance * NEW_TICKS_PER_INCH;
                }
                if (angle == 90) {
                    speedRight = -speed;
                    speedLeft = speed;
                    targetPos = distance * NEW_STRAFE_TICKS_PER_INCH;
                }
                if (angle == 180) {
                    speedRight = -speed;
                    speedLeft = -speed;
                    targetPos = distance * NEW_TICKS_PER_INCH;
                }
                if (angle == 270) {
                    speedRight = speed;
                    speedLeft = -speed;
                    targetPos = distance * NEW_STRAFE_TICKS_PER_INCH;
                }

                frontRightTargetPower = speedRight;
                frontLeftTargetPower = speedLeft;
                backRightTargetPower = speedLeft;
                backLeftTargetPower = speedRight;
                motorPower = 0;
                rampUp = 1;
                maxSpeed = 0;
                speedReset = true;
                moveStep++;
                break;
            case 1:


                if (!isDesiredTicksComplete(targetPos)) {
                    rampUp = rampDown;
                    motorPower += POWER_ACCELERATION * rampUp;
                    motorPower = Reference.clip(motorPower, 0.2, speed);
                    resetTime = runTime;
                    frontRight.setPower(motorPower * Reference.getSign(frontRightTargetPower));
                    frontLeft.setPower(motorPower * Reference.getSign(frontLeftTargetPower));
                    backRight.setPower(motorPower * Reference.getSign(backRightTargetPower));
                    backLeft.setPower(motorPower * Reference.getSign(backLeftTargetPower));
                } else {
                    setDrivePower(0);
                    moveStep++;
                }
                break;
            case 2:
                setDrivePower(0);
                doneMoving = true;
                moveStep = 0;
                break;
        }
        if (doneMoving) {
            doneMoving = false;
            return true;
        } else {
            return false;
        }

    }

    public boolean isDesiredTicksComplete(double ticks) {
        return Math.abs(backLeft.getCurrentPosition()) >= ticks;
    }

    //here is another method that was not placed by Nathaniel 11/12/21
    public void moveRobot(double controllerx, double controllery, double turnx) {

        double gear = FAST_GEAR;

        //the speed the robot will move along the x axis
        double x = 0;
        /*
        sign is an integer that stores whether the x speed is positive or negative
        (if x is positive, sign is 1, if x is negative, x is -1)
         */
        double sign = math.getSign(controllerx);
        //the minimum power the motors can go and have the robot still strafe
        double minSpeed = MIN_STRAFE_SPEED * sign / gear;
        /*
        if the controller is not moving in the x direction, the robot will not
        move in the x direction
         */
        if (Math.abs(controllerx) == 0) {
            x = 0;
        }
        /*
        the controlled portion of our control curve. This will strafe very slowly
        to give the user control over minor adjustments and is active when the
        stick is barely moved.
         */
        else if (Math.abs(controllerx) < 0.4) {
            x = minSpeed + controllerx * 0.3;
        }
        /*
        the moderate speed portion of our control curve. This will strafe relatively
        slowly to give the user control while being too fast for minor adjustments
        and is active when the stick is slightly moved.
         */
        else if (Math.abs(controllerx) < 1) {
            x = minSpeed + 0.12 / gear * math.getSign(controllerx) + controllerx * 0.4;
        }
        /*
        The high speed portion of the control curve. It only activates on the max
        speed because the control stick has a large portion of its total range
        of movement being the 1 value.
         */
        else {
            x = sign + minSpeed;
        }
        //the speed the robot will move along the y axis
        double y;
        //redefined the sign variable for the y speed
        sign = math.getSign(controllery);
        //sets the minimum speed for the y speed
        minSpeed = MIN_DRIVE_SPEED * sign / gear;
        //the stopped state in the y direction
        if (Math.abs(controllery) == 0) {
            y = 0;
        }
        //the controlled portion on the y section
        else if (Math.abs(controllery) < 0.7) {
            y = -minSpeed - 0.15 * sign + controllery * -0.3;
        }
        //the speed portion of the y section
        else {
            y = controllery * -1.2 - 0.2 * sign * -1;
        }
        /*
        if the robot is in the control mode, it will make the
        robot's movement in the x direction a bit faster
        (making the robot strafe faster relative to moving and
        turning)
         */
        x *= controlMode ? 1.4 : 1;
        /*
        sets the speed for the front right and back left motors
        for the moving / strafing
         */
        x *= gear;
        x /= x > 1 ? x : 1;
        y *= gear;
        y /= y > 1 ? y : 1;
        double speedRight = math.squarePreserveSign(y) - math.squarePreserveSign(x);
        /*
        sets the speed for the front left and back right motors
        for the moving / strafing
         */
        double speedLeft = math.squarePreserveSign(y) + math.squarePreserveSign(x);
        //pow = Math.max(speedRight, speedLeft);
        //the speed the robot will turn at
        double rx;
        //redefined the sign variable for the turning
        sign = math.getSign(turnx);
        //the stopped portion for turning
        if (Math.abs(turnx) == 0) {
            rx = 0;
        }
        //the controlled portion for turning
        else if (Math.abs(turnx) < 0.5) {
            rx = MIN_TURN_SPEED * sign / gear + turnx * 0.15;
        }
        //the moderate speed portion for turning
        else if (Math.abs(turnx) < 0.95) {
            rx = MIN_TURN_SPEED * sign / gear + 0.15 * sign + turnx * 0.25;
        }
        //the high speed portion for turning
        else {
            rx = turnx * 8 - 7 * sign;
        }
        /*
        in control mode, the robot will turn slightly
        slower relative to moving and strafing
         */
        rx *= controlMode ? 0.8 : 1;
        rx *= gear;
        rx /= rx > 1 ? rx : 1;
        /*
        the power of the right motors for turning (called
        speedyLeft because it is the power for a left turn)
         */
        double speedyLeft = -rx;
        /*
        the power of the left motors for turning (called
        speedyRight because it is the power for a right turn)
         */
        double speedyRight = rx;
        //the powers of the drive motors
        double FR;
        double FL;
        double BR;
        double BL;
        /*
        if the robot is attempting to move in both the
        x and y directions, it will not try to turn since
        the wheels could fight each other if it tried.
         */
        if (x != 0 && y != 0) {
            FR = speedRight;
            FL = speedLeft;
            BR = speedLeft;
            BL = speedRight;
        }
        /*
        attempt to both turn and move if the robot
        is not moving in both x and y directions
         */
        else {
            /*
            sets the power of the motors by combining the
            turning value with the moving value.
             */
            FR = speedRight + speedyLeft;
            FL = speedLeft + speedyRight;
            BR = speedLeft + speedyLeft;
            BL = speedRight + speedyRight;
        }
        /*
        //scales the powers by the gear
        FR = FR * gear;
        FL = FL * gear;
        BR = BR * gear;
        BL = BL * gear;
        */
            /*
            checks to see if any of the powers to set is
            greater than 2
             */
        double hv = math.FIND_HIGH_NUMBER(FL, FR);
        hv = math.FIND_HIGH_NUMBER(hv, BR);
        hv = math.FIND_HIGH_NUMBER(hv, BL);
            /*
            if one of the powers is greater than 2, then
            divide all of the motor powers by the number,
            making all the powers 1 or less.
             */
        if (Math.abs(hv) > 1) {

            FR /= hv;
            FL /= hv;
            BR /= hv;
            BL /= hv;

        }
        /*
        if is trying to drive and the motor powers are
        below the minimum speed, set the powers to the minimum drive speed
         */
        if (BR != 0 && BR == FR && FR == BL && FR == FL && Math.abs(FR) < MIN_DRIVE_SPEED) {
            FR = MIN_DRIVE_SPEED * math.getSign(FR);
            FL = MIN_DRIVE_SPEED * math.getSign(FR);
            BR = MIN_DRIVE_SPEED * math.getSign(FR);
            BL = MIN_DRIVE_SPEED * math.getSign(FR);
        }
        /*
        if the robot is trying to strafe and the motor power is lower than
        the minimum strafe power, then set the power to the minimum strafe power
         */
        if (BR != 0 && BR == FL && BL == FR && FR == FL * -1 && Math.abs(FR) < MIN_STRAFE_SPEED) {
            FR = MIN_STRAFE_SPEED * math.getSign(FR);
            FL = MIN_STRAFE_SPEED * math.getSign(FL);
            BR = MIN_STRAFE_SPEED * math.getSign(BR);
            BL = MIN_STRAFE_SPEED * math.getSign(BL);

        }
        /*
        if the robot is turning and the power is lower than the minimum
        turn power, then set the power to the minimum turn power.
         */
        if (BR != 0 && BR == FR && BR != FL && FL == BL && Math.abs(BR) < MIN_TURN_SPEED) {
            FR = MIN_TURN_SPEED * math.getSign(FR);
            FL = MIN_TURN_SPEED * math.getSign(FL);
            BR = MIN_TURN_SPEED * math.getSign(BR);
            BL = MIN_TURN_SPEED * math.getSign(BL);
        }
        /*
        checks to see if the motor powers are less than 1 before setting the power
        to make sure the program does not throw an error for a high power.
         */
        if (Math.abs(FR) <= 1 && Math.abs(FL) <= 1 && Math.abs(BR) <= 1 && Math.abs(BL) <= 1) {
            //sets the power for the motors
            setDrivePower(FR, FL, BR, BL);
        }
        //stops the motors if a power to set was above 1
        else {
            setDrivePower(0, 0, 0, 0);
        }
    }

    public void moveRobotHighSpeed(float controllerx, float controllery, float turnx, float precision) {

        //double gear = map.get(Reference.getGear());
        double gear = 0.4;  // TODO use Reference.Gear to create enumerated list to return numeric value of gear constants above
//        if (Reference.getGear() == Reference.Gear.SLOW)
//        {
//           gear = SLOW_GEAR;
//        }
//        if (Reference.getGear() == Reference.Gear.MID)
//        {
//            gear = MID_GEAR;
//        }
//        if (Reference.getGear() == Reference.Gear.FAST)
//        {
//            gear = FAST_GEAR;
//        }
        gear = FAST_GEAR - 0.1;
        //the speed the robot will move along the x axis
        double x;
        /*
        sign is an integer that stores whether the x speed is positive or negative
        (if x is positive, sign is 1, if x is negative, x is -1)
         */
        double sign = math.getSign(controllerx);
        /*
        if the controller is not moving in the x direction, the robot will not
        move in the x direction
         */
        if (Math.abs(controllerx) == 0) {
            x = 0;
        } else {
            if (Math.abs(controllerx) < 0.35) {
                x = 0.15 * sign * math.getSign(gear) + math.getSign(gear) * controllerx * 0.3;
            } else if (Math.abs(gear) < 0.7) {
                x = 0.27 * sign * math.getSign(gear) + controllerx * 0.5 * math.getSign(gear);
            } else {
                x = controllerx * gear;
            }
        }
        //the speed the robot will move along the y axis
        double y = 0;
        //redefined the sign variable for the y speed
        sign = math.getSign(controllery);
        //the stopped state in the y direction
        if (Math.abs(controllery) == 0) {
            y = 0;
        } else {
            if (Math.abs(controllery) < 0.35) {
                y = 0.12 * sign * math.getSign(gear) + math.getSign(gear) * controllery * 0.3;
            } else if (Math.abs(gear) < 0.7) {
                y = 0.27 * sign * math.getSign(gear) + controllery * 0.45 * math.getSign(gear);
            } else {
                y = controllery * gear;
            }
        }
        x /= x > 1 ? x : 1;
        y /= y > 1 ? y : 1;
        double speedRight = y - x;
        speedRight = speedRight * (1 - (precision * 0.8));
        /*
        sets the speed for the front left and back right motors
        for the moving / strafing
         */
        double speedLeft = y + x;
        speedLeft = speedLeft * (1 - (precision * 0.8));

        //pow = Math.max(speedRight, speedLeft);
        //the speed the robot will turn at
        double rx = 0;
        //redefined the sign variable for the turning
        sign = math.getSign(turnx);
        //the stopped portion for turning
        if (Math.abs(turnx) == 0) {
            rx = 0;
        } else {
            if (Math.abs(turnx) < 1) {
                rx = 0.09 * sign + turnx * 0.3;
            } else {
                rx = turnx * FAST_GEAR;
            }
        }
        /*
        in control mode, the robot will turn slightly
        slower relative to moving and strafing
         */
        rx /= rx > 1 ? rx : 1;
        /*
        the power of the right motors for turning (called
        speedyLeft because it is the power for a left turn)
         */
        double speedyLeft = -rx;
        /*
        the power of the left motors for turning (called
        speedyRight because it is the power for a right turn)
         */
        double speedyRight = rx;
        //the powers of the drive motors
        double FR;
        double FL;
        double BR;
        double BL;
        /*
        if the robot is attempting to move in both the
        x and y directions, it will not try to turn since
        the wheels could fight each other if it tried.
         */
        if (x != 0 && y != 0) {
            FR = speedRight;
            FL = speedLeft;
            BR = speedLeft;
            BL = speedRight;
        }
        /*
        attempt to both turn and move if the robot
        is not moving in both x and y directions
         */
        else {
            /*
            sets the power of the motors by combining the
            turning value with the moving value.
             */
            FR = speedRight + speedyLeft;
            FL = speedLeft + speedyRight;
            BR = speedLeft + speedyLeft;
            BL = speedRight + speedyRight;
        }

            /*
            checks to see if any of the powers to set is
            greater than 2
             */
        double hv = math.FIND_HIGH_NUMBER(FL, FR);
        hv = math.FIND_HIGH_NUMBER(hv, BR);
        hv = math.FIND_HIGH_NUMBER(hv, BL);
            /*
            if one of the powers is greater than 2, then
            divide all of the motor powers by the number,
            making all the powers 1 or less.
             */
        if (Math.abs(hv) > 1) {

            FR /= hv;
            FL /= hv;
            BR /= hv;
            BL /= hv;

        }
        /*
        checks to see if the motor powers are less than 1 before setting the power
        to make sure the program does not throw an error for a high power.
         */
        if (Math.abs(FR) <= 1 && Math.abs(FL) <= 1 && Math.abs(BR) <= 1 && Math.abs(BL) <= 1) {
            //sets the power for the motors
            setDrivePower(FR, FL, BR, BL);
        }
        //stops the motors if a power to set was above 1
        else {
            setDrivePower(0, 0, 0, 0);
        }


    }

    public void copiedMecanumMovementCodeForTest(double driveSpeed, double strafeSpeed, double twistSpeed) {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        // Sample inputs:
        // double drive  = gamepad1.left_stick_y;
        // double strafe = gamepad1.left_stick_x;
        // double twist  = gamepad1.right_stick_x;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (driveSpeed - strafeSpeed + twistSpeed),
                (driveSpeed + strafeSpeed - twistSpeed),
                (driveSpeed + strafeSpeed + twistSpeed),
                (driveSpeed - strafeSpeed - twistSpeed)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);
    }


    public void mecanumMovementFieldOrientedControl(double xDeflection, double yDeflection, double twistSpeed) {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        // Sample inputs:
        // double drive  = gamepad1.left_stick_y;
        // double strafe = gamepad1.left_stick_x;
        // double twist  = gamepad1.right_stick_x;

        // NOTE - these can be used as AUTO inputs as well - where X and Y inputs
        // need to be adjusted based on coordinate orientation vs. "stick" positions
        // For Power Play, +x direction corresponds to "gamepad1.left_stick_y"
        // and +y direction corresponds to "-gamepad1.left_stick_x"

        /*
         * if Field Oriented Control:
         *     get gyro heading
         *
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */
        double radianHeading = Math.toRadians(nav.getCorrectedImuHdg());
        double newDriveSpeed = -yDeflection * Math.cos(radianHeading) + xDeflection * Math.sin(radianHeading);
        double newStrafeSpeed = yDeflection * Math.sin(radianHeading) + xDeflection * Math.cos(radianHeading);


        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (newDriveSpeed - newStrafeSpeed + twistSpeed),
                (newDriveSpeed + newStrafeSpeed - twistSpeed),
                (newDriveSpeed + newStrafeSpeed + twistSpeed),
                (newDriveSpeed - newStrafeSpeed - twistSpeed)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);

        powerInsightFrontRight = speeds[1];
    }

}
