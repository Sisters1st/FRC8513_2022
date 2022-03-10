package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        thisRobot.autoStep = 0;
        thisRobot.autoStartTime = System.currentTimeMillis();
        thisRobot.autoDashboard = 6;
        thisRobot.autoWaitTime = Preferences.getInt("AutoWait", 0);

    }

    public void autoPeriodic() {
        switch (thisRobot.autoDashboard) {
            case 0:
                thisRobot.autoAction = 0;
                break;
            case 1:
                driveForward();
                break;
            case 2:
                driveBackwards();
                break;
            case 3:
                turn180();
                break;
            case 4:
                shootReverse();
                break;
            case 5:
                IntakeStraightTurnStraightShoot();
                break;
            case 6:
                if(System.currentTimeMillis()-thisRobot.autoStartTime>thisRobot.autoWaitTime)
                {
                    thisRobot.autoDashboard = Preferences.getInt("Auto", 0);
                }
                break;
            case 7:
                grabSecondScoreGrabThirdScore();
                break;
            default:
                thisRobot.autoDashboard = 0;
        }
        autoActions();
        if (thisRobot.autoActionIsDone) {
            thisRobot.autoStep++;
            thisRobot.autoActionIsDone = false;
            thisRobot.tHoldCounter=0;
        }
        thisRobot.intakeStateController.updateState();
        thisRobot.intakeStateController.updateMotorPower();
        SmartDashboard.putNumber("autoStep", thisRobot.autoStep);
    }

    /** This function is called periodically during autonomous. */
    public void autoActions() {
        switch ((int) thisRobot.autoAction) {
            case 0: // default
                thisRobot.m_myRobot.stopMotor();
                break;
            case 1: // turn a certain amount of degrees to the right with PID
                controllerOutput = thisRobot.turnPID.calculate(thisRobot.currentAngle, thisRobot.autoGoalAngle);
                goalMotorSpeed = MathUtil.clamp(controllerOutput, -1, 1);
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed, -goalMotorSpeed);
                // if error is less than a set value
                if (Math.abs(thisRobot.autoGoalAngle - thisRobot.currentAngle) < thisRobot.autoAngleTHold) {
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
               // motorDelta = MathUtil.clamp(controllerOutput, -.2, .2);
               motorDelta=0;
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
        SmartDashboard.putNumber("autoRotationError", thisRobot.autoGoalAngle - thisRobot.currentAngle);
        SmartDashboard.putNumber("controllerOutput", controllerOutput);
        SmartDashboard.putNumber("driveStraightMotorDelta", motorDelta);
        SmartDashboard.putNumber("goalMotorSpeed", goalMotorSpeed);
        SmartDashboard.putNumber("autoDistanceError", thisRobot.autoGoalDistance - thisRobot.currentPosition);
    }

    public void resetSensors() {
        thisRobot.leftEncoder.setPosition(0);
        thisRobot.rightEncoder.setPosition(0);
        thisRobot.ahrs.reset();
        thisRobot.turnPID.reset();
        thisRobot.straightPID.reset();
        thisRobot.distancePID.reset();
    }

    public void driveForward() {
        switch ((int) thisRobot.autoStep) {
            case 0: // drive forward
                thisRobot.autoAction = 2;
                resetSensors();
                thisRobot.autoGoalDistance = 5;
                thisRobot.autoStep++;
                break;
            case 1:
                // waiting for first turn to complete
                break;
            case 2: // stopping
                thisRobot.autoAction = 0;
            default:
                thisRobot.autoAction = 0;
        }
    }
    public void driveBackwards() {
        switch ((int) thisRobot.autoStep) {
            case 0: // drive backward
                thisRobot.autoAction = 2;
                resetSensors();
                thisRobot.autoGoalDistance = -5;
                thisRobot.autoStep++;
                break;
            case 1:
                // waiting for first turn to complete
                break;
            case 2: // stopping
                thisRobot.autoAction = 0;
            default:
                thisRobot.autoAction = 0;
        }
    }
    public void turn180() {
        switch ((int) thisRobot.autoStep) {
            case 0: // drive backward
                thisRobot.autoAction = 1;
                resetSensors();
                thisRobot.autoGoalAngle = 180;
                thisRobot.autoStep++;
                break;
            case 1:
                // waiting for first turn to complete
                break;
            case 2: // stopping
                thisRobot.autoAction = 0;
            default:
                thisRobot.autoAction = 0;
        }
    } 

public void shootReverse()
{
    switch ((int) thisRobot.autoStep) {
        case 0: //shoot
            thisRobot.intakeStateController.intakeState=8;
            thisRobot.autoStep++;
            break;
        case 1: //setting our current time
            thisRobot.autoStartTime = System.currentTimeMillis();
            thisRobot.autoStep++;
            break;
        case 2: //checking if our elapsed time is greater than out threshold, if it is we move on to the next case, otherwise, we wait
            if(System.currentTimeMillis()-thisRobot.autoStartTime>3000)
            {
                thisRobot.autoStep++;
            }
            break;
        case 3:
            thisRobot.intakeStateController.intakeState=1;
            thisRobot.autoStep++;
        case 4: //drive backward
            thisRobot.autoAction = 2;
            resetSensors();
            thisRobot.autoGoalDistance = -5;
            thisRobot.autoStep++;
            break;
        case 5:
        //wait for robot to finish driving backward
        break;
        case 6:
        thisRobot.autoAction = 0;
        break;
        }
}

public void IntakeStraightTurnStraightShoot()
{
    switch ((int) thisRobot.autoStep) {
        case 0: //turn intake on
            thisRobot.intakeStateController.intakeState=3;
            thisRobot.autoStep++;
            break;
        case 1: //drive forward
            thisRobot.autoAction = 2;
            resetSensors();
            thisRobot.autoGoalDistance = 2;
            thisRobot.autoStep++;
            break;
        case 2: //waiting
            break;
        case 3: //turn 180 degrees
            thisRobot.autoAction = 1;
            resetSensors();
            thisRobot.autoGoalAngle = 180;
            thisRobot.autoStep++;
            break;
        case 4://waiting
        SmartDashboard.putNumber("4", 4);
            break;
        case 5: //drive forward
            thisRobot.autoAction = 2;
            resetSensors();
            thisRobot.autoGoalDistance = 5;
            thisRobot.autoStep++;
            break;
        case 6: //waiting
            break;
        case 7://turn on shooter
            thisRobot.autoAction=0;
            thisRobot.intakeStateController.intakeState=8;
            thisRobot.autoStep++;
            break;
        case 8: //set timer
            thisRobot.autoStartTime = System.currentTimeMillis();
            thisRobot.autoStep++;
            break;
        case 9: //checking if our elapsed time is greater than out threshold, if it is we move on to the next case, otherwise, we wait
            /*System.out.println("current time" + System.currentTimeMillis()%10000);
            System.out.println("start time" + thisRobot.autoStartTime%10000);*/
            if(System.currentTimeMillis()-thisRobot.autoStartTime>3000)
            {
                thisRobot.autoStep++;
            }
            break;
        case 10: //stop
            thisRobot.intakeStateController.intakeState=1;
            thisRobot.autoStep++;
            break;
        case 11: //waiting
            break;
        case 12:
            thisRobot.autoAction = 0;
            break;
        }
}
public void grabSecondScoreGrabThirdScore()
{
    switch ((int) thisRobot.autoStep) {
        case 0: //turn intake on
            thisRobot.intakeStateController.intakeState=3;
            thisRobot.autoStep++;
            break;
        case 1: //drive forward
            thisRobot.autoAction = 2;
            resetSensors();
            thisRobot.autoGoalDistance = 2;
            thisRobot.autoStep++;
            break;
        case 2: //waiting
            break;
        case 3: //turn 180 degrees
            thisRobot.autoAction = 1;
            resetSensors();
            thisRobot.autoGoalAngle = 180;
            thisRobot.autoStep++;
            break;
        case 4://waiting
        SmartDashboard.putNumber("4", 4);
            break;
        case 5: //drive forward
            thisRobot.autoAction = 2;
            resetSensors();
            thisRobot.autoGoalDistance = 5;
            thisRobot.autoStep++;
            break;
        case 6: //waiting
            break;
        case 7://turn on shooter
            thisRobot.autoAction=0;
            thisRobot.intakeStateController.intakeState=8;
            thisRobot.autoStep++;
            break;
        case 8: //set timer
            thisRobot.autoStartTime = System.currentTimeMillis();
            thisRobot.autoStep++;
            break;
        case 9: //checking if our elapsed time is greater than out threshold, if it is we move on to the next case, otherwise, we wait
            /*System.out.println("current time" + System.currentTimeMillis()%10000);
            System.out.println("start time" + thisRobot.autoStartTime%10000);*/
            if(System.currentTimeMillis()-thisRobot.autoStartTime>3000)
            {
                thisRobot.autoStep++;
            }
            break;
        case 10: //turn 180 degrees
            thisRobot.autoAction = 1;
            resetSensors();
            thisRobot.autoGoalAngle = -160;
            thisRobot.autoStep++;
            break;
        case 11: //waiting 
        break;
        case 12: //turn intake on
            thisRobot.intakeStateController.intakeState=3;
            thisRobot.autoStep++;
            break;
        case 13: //drive forward
            thisRobot.autoAction = 2;
            resetSensors();
            thisRobot.autoGoalDistance = 2;
            thisRobot.autoStep++;
            break;
        case 14: //waiting
            break;
        case 15: //turn 180 degrees
            thisRobot.autoAction = 1;
            resetSensors();
            thisRobot.autoGoalAngle = 180;
            thisRobot.autoStep++;
            break;
        case 16://waiting
        SmartDashboard.putNumber("4", 4);
            break;
        case 17: //drive forward
            thisRobot.autoAction = 2;
            resetSensors();
            thisRobot.autoGoalDistance = 5;
            thisRobot.autoStep++;
            break;
        case 18: //waiting
            break;
        case 19:
        break;
        case 20: //turn 180 degrees
            thisRobot.autoAction = 1;
            resetSensors();
            thisRobot.autoGoalAngle = -160;
            thisRobot.autoStep++;
            break;
        case 21://turn on shooter
            thisRobot.autoAction=0;
            thisRobot.intakeStateController.intakeState=8;
            thisRobot.autoStep++;
            break;
        case 22: //set timer
            thisRobot.autoStartTime = System.currentTimeMillis();
            thisRobot.autoStep++;
            break;
        case 23: //checking if our elapsed time is greater than out threshold, if it is we move on to the next case, otherwise, we wait
            /*System.out.println("current time" + System.currentTimeMillis()%10000);
            System.out.println("start time" + thisRobot.autoStartTime%10000);*/
            if(System.currentTimeMillis()-thisRobot.autoStartTime>3000)
            {
                thisRobot.autoStep++;
            }
            break;
        case 24: //stop
            thisRobot.intakeStateController.intakeState=1;
            thisRobot.autoStep++;
            break;
        case 25: //waiting
            break;
        case 26:
            thisRobot.autoAction = 0;
            break;
        }
}
}