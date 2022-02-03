package frc;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Auto {
    public Robot thisRobot;

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

    /** This function is called periodically during autonomous. */
    public void autoPeriodic() {
        double goalMotorSpeed = 100000;
        double motorDelta = 100000;
        double controllerOutput = 100000;
        switch ((int) thisRobot.Auto) {
            case 0:
                // do nothing auto
                break;
            case 11: //turn 90 degrees to the right with PID
                controllerOutput = thisRobot.turnPID.calculate(thisRobot.currentAngle, thisRobot.autoGoalAngle);
                goalMotorSpeed = MathUtil.clamp(controllerOutput, -1, 1);
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed, -goalMotorSpeed);
                break;
            case 12: // drive straight with PID
                controllerOutput = thisRobot.straightPID.calculate(thisRobot.leftEncoderPosition - thisRobot.rightEncoderPosition, 0);
                motorDelta = MathUtil.clamp(controllerOutput, -.2, .2);
                goalMotorSpeed = 0.5;
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed + motorDelta, goalMotorSpeed - motorDelta);
                break;
            case 13: // drive straight backward with PID
                controllerOutput = thisRobot.straightPID.calculate(thisRobot.leftEncoderPosition - thisRobot.rightEncoderPosition, 0);
                motorDelta = MathUtil.clamp(controllerOutput, -.2, .2);
                goalMotorSpeed = -0.5;
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed + motorDelta, goalMotorSpeed - motorDelta);
                break;
            case 14: // drive straight to a distance with PID
                double distanceControllerOutput = thisRobot.distancePID.calculate(
                        (thisRobot.leftEncoderPosition + thisRobot.rightEncoderPosition) / 2, 38.216);
                goalMotorSpeed = MathUtil.clamp(distanceControllerOutput, -.6, .6);
                controllerOutput = thisRobot.straightPID.calculate(thisRobot.leftEncoderPosition - thisRobot.rightEncoderPosition, 0);
                motorDelta = MathUtil.clamp(controllerOutput, -.2, .2);
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed + motorDelta, goalMotorSpeed - motorDelta);
                break;
            case 15: // drive straight to a distance with PID
                distanceControllerOutput = thisRobot.distancePID.calculate((thisRobot.leftEncoderPosition + thisRobot.rightEncoderPosition) / 2,
                        -38.216);
                goalMotorSpeed = MathUtil.clamp(distanceControllerOutput, -.6, .6);
                controllerOutput = thisRobot.straightPID.calculate(thisRobot.leftEncoderPosition - thisRobot.rightEncoderPosition, 0);
                motorDelta = MathUtil.clamp(controllerOutput, -.2, .2);
                thisRobot.m_myRobot.tankDrive(goalMotorSpeed + motorDelta, goalMotorSpeed - motorDelta);
                break;
            default:
                // do nothing
        }
        SmartDashboard.putNumber("autoGoalError", thisRobot.autoGoalAngle - thisRobot.currentAngle);
        SmartDashboard.putNumber("controllerOutput", controllerOutput);
        SmartDashboard.putNumber("dirveStraightMotorDelta", motorDelta);
        SmartDashboard.putNumber("goalMotorSpeed", goalMotorSpeed);
    }

}
