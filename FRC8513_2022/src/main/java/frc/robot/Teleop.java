package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop {
  public Robot thisRobot;
  double leftPow = 0;
  double leftJoy;
  double rightPow = 0;
  double rightJoy;
  int arcadeDrive;

  public Teleop(Robot thisRobotParameter) {
    thisRobot = thisRobotParameter;
  }

  //
  public void teleInit() {
    thisRobot.intakeStateController.setState(1);
  }

  /** This function is called periodically during teleoperated mode. */
  public void telePeriodic() {
    int linearJoystick = Preferences.getInt("linearJoystick", 0);
    SmartDashboard.putNumber("linearJoystick", linearJoystick); // put the value of the autoDashboard on the Smart
    int arcadeDrive = Preferences.getInt("arcadeDrive", 0);
    leftJoy = -thisRobot.joystickBlue.getY();
    rightJoy = -thisRobot.joystickBlue.getRawAxis(5);
    switch (linearJoystick) {
      case 0:
        squaringController();
        break;
      case 1:
        linearController();
        break;
      default:
        squaringController();
    
      }
    switch (arcadeDrive) {
      case 0:
        thisRobot.m_myRobot.tankDrive(leftPow, rightPow); // tank drive
        break;
      case 1:
        thisRobot.m_myRobot.arcadeDrive(-thisRobot.joystickBlue.getY(), thisRobot.joystickBlue.getRawAxis(4));
        // arcade drive
        break;
      default:
        thisRobot.m_myRobot.tankDrive(leftPow, rightPow); // tank drive
    }
    thisRobot.intakeStateController.updateState();
    thisRobot.intakeStateController.updateMotorPower();
    }

  public void squaringController() {
    // squaring the axis to make it easier to drive
    if (leftJoy > 0) {
      leftPow = leftJoy * leftJoy;
    } else {
      leftPow = -leftJoy * leftJoy;
    }
    if (rightJoy > 0) {
      rightPow = rightJoy * rightJoy;
    } else {
      rightPow = -rightJoy * rightJoy;
    }
    
  }

  public void linearController() {
    // squaring the axis to make it easier to drive
    if (leftJoy > 0) {
      leftPow = leftJoy;
    } else {
      leftPow = leftJoy;
    }
    if (rightJoy > 0) {
      rightPow = rightJoy;
    } else {
      rightPow = rightJoy;
    }
    SmartDashboard.putNumber("Left Motor Power", leftPow);
    SmartDashboard.putNumber("Right Motor Power", rightPow);
  }
}
