package frc;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

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
  }

  /** This function is called periodically during teleoperated mode. */
  public void telePeriodic() {
    int linearJoystick = Preferences.getInt("linearJoystick", 0);
    SmartDashboard.putNumber("linearJoystick", linearJoystick); // put the value of the autoDashboard on the Smart
    int arcadeDrive = Preferences.getInt("arcadeDrive", 0);
    leftJoy = -thisRobot.joystick.getY();
    rightJoy = -thisRobot.joystick.getRawAxis(3);
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
        thisRobot.m_myRobot.arcadeDrive(-thisRobot.joystick.getY(), thisRobot.joystick.getRawAxis(2));
        // arcade drive
        break;
      default:
        thisRobot.m_myRobot.tankDrive(leftPow, rightPow); // tank drive
    }
    // coding for the buttons
    if (thisRobot.joystick.getRawButtonPressed(6)) {
      thisRobot.m_mechID1.set(.5);
    }
    if (thisRobot.joystick.getRawButtonPressed(8)) {
      thisRobot.m_mechID1.set(-.5);
    }
    if (thisRobot.joystick.getRawButtonPressed(7)) {
      thisRobot.m_mechID2.set(.5);
    }
    if (thisRobot.joystick.getRawButtonPressed(5)) {
      thisRobot.m_mechID2.set(-.5);
    }
    if (!thisRobot.joystick.getRawButton(6) && !thisRobot.joystick.getRawButton(8)) {
      thisRobot.m_mechID1.set(0);
    }
    if (!thisRobot.joystick.getRawButton(5) && !thisRobot.joystick.getRawButton(7)) {
      thisRobot.m_mechID2.set(0);
    }
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
    SmartDashboard.putNumber("Left Motor Power", leftPow);
    SmartDashboard.putNumber("Right Motor Power", rightPow);
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