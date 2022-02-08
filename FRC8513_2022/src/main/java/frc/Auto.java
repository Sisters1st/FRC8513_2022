package frc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Auto {
    public Robot thisRobot;
    double goalMotorSpeed = 100000;
    double motorDelta = 100000;
    double controllerOutput = 100000;

    public Auto(Robot thisRobotParameter) {
        thisRobot = thisRobotParameter;
    }

    /** This function is run once at the beginning of each autonomous mode. */
    public void autoInit() {
        thisRobot.m_timer.reset();
        thisRobot.m_timer.start();
        thisRobot.autoStartingAngle = thisRobot.currentAngle;
        thisRobot.turnPID.reset();
        SmartDashboard.putNumber("autoStartingAngle", thisRobot.autoStartingAngle);
        thisRobot.leftEncoder.setPosition(0);
        thisRobot.rightEncoder.setPosition(0);
        thisRobot.straightPID.reset();
        thisRobot.distancePID.reset();
    }

    public void autoPeriodic() {
        switch ((int) thisRobot.autoStep) {
            case 0:
                thisRobot.autoAction = 1;
                resetSensors();
                thisRobot.autoGoalAngle = 90;
                thisRobot.autoStep++;
                break;
            case 1:
                // waiting for first turn to complete
                break;
            case 2:// drive straight
                thisRobot.autoAction = 2;
                resetSensors();
                thisRobot.autoGoalDistance = 10;
                thisRobot.autoStep++;
                break;
            case 3:
                // waiting for driving straight to complete
                break;
            case 4:
            thisRobot.autoAction = 1;
            resetSensors();
            thisRobot.autoGoalAngle = -90;
            thisRobot.autoStep++;
            break;
            case 5:
            //waiting
            break;
            case 6:
            thisRobot.autoAction = 0;
            default:
                thisRobot.autoAction = 0;
        }
        autoActions();
        if (thisRobot.autoActionIsDone == true) {
            thisRobot.autoStep++;
            thisRobot.autoActionIsDone = false;
        }

    }

    /** This function is called periodically during autonomous. */
    public void autoActions() {
        switch ((int) thisRobot.autoAction) {
            case 0: // default
                thisRobot.m_myRobot.stopMotor();
                break;
            case 1: // turn 90 degrees to the right with PID
                controllerOutput = thisRobot.turnPID.calculate(thisRobot.currentAngle, thisRobot.autoGoalAngle);
                goalMotorSpeed = MathUtil.clamp(controllerOutput, -1, 1);
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed, -goalMotorSpeed);
                if (Math.abs(thisRobot.autoGoalAngle - thisRobot.currentAngle) < thisRobot.autoAngleTHold) // if error
                                                                                                           // is less
                                                                                                           // than a set
                                                                                                           // value
                {
                    thisRobot.tHoldCounter++;
                    if (thisRobot.tHoldCounter > thisRobot.tHoldCounterTHold) {
                        thisRobot.autoActionIsDone = true;
                    }
                } else {
                    thisRobot.tHoldCounter = 0;
                }
                break;
            case 2: // drive straight to a distance with PID
                double distanceControllerOutput = thisRobot.distancePID.calculate(
                        thisRobot.currentPosition, thisRobot.autoGoalDistance);
                goalMotorSpeed = MathUtil.clamp(distanceControllerOutput, -.6, .6);
                controllerOutput = thisRobot.straightPID
                        .calculate(thisRobot.leftEncoderPosition - thisRobot.rightEncoderPosition, 0);
                motorDelta = MathUtil.clamp(controllerOutput, -.2, .2);
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed + motorDelta, goalMotorSpeed - motorDelta);
                if (Math.abs(thisRobot.autoGoalDistance - thisRobot.currentPosition) < thisRobot.autoDistanceTHold) {
                    thisRobot.tHoldCounter++;
                    if (thisRobot.tHoldCounter > thisRobot.tHoldCounterTHold) {
                        thisRobot.autoActionIsDone = true;

                    }
                } else {
                    thisRobot.tHoldCounter = 0;
                }
                break;
            default:
                thisRobot.m_myRobot.stopMotor();

        }
        SmartDashboard.putNumber("autoGoalError", thisRobot.autoGoalAngle - thisRobot.currentAngle);
        SmartDashboard.putNumber("controllerOutput", controllerOutput);
        SmartDashboard.putNumber("dirveStraightMotorDelta", motorDelta);
        SmartDashboard.putNumber("goalMotorSpeed", goalMotorSpeed);
    }

    public void resetSensors() {
        thisRobot.leftEncoder.setPosition(0);
        thisRobot.rightEncoder.setPosition(0);
        thisRobot.ahrs.reset();
    }
}