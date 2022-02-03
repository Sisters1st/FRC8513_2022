package frc;
import frc.robot.Robot;

public class Teleop {
    public Robot thisRobot;
    public Teleop(Robot thisRobotParameter) {
        thisRobot = thisRobotParameter;
    }

//
    public void teleInit() {
    }

  /** This function is called periodically during teleoperated mode. */
  public void telePeriodic() {
    //squaring the axis to make it easier to drive
    double leftPow = 0;
    double leftJoy = thisRobot.joystick.getY();
    if (leftJoy > 0) {
      leftPow = -1 * leftJoy * leftJoy;
    } else {
      leftPow = leftJoy * leftJoy;
    }
    double rightPow = 0;
    double rightJoy = thisRobot.joystick.getRawAxis(3);
    if (rightJoy > 0) {
      rightPow = -1 * rightJoy * rightJoy;
    } else {
      rightPow = rightJoy * rightJoy;
    }
    //coding for the buttons
    thisRobot.m_myRobot.tankDrive(leftPow, rightPow);
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
}
