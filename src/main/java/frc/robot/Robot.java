// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.filter.SlewRateLimiter; // Hızlanmayı yumuşatmak için
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.controller.Driver;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private Drivetrain drivetrain;
  private Driver driver;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public enum RobotState {
    IDLE,
    INTAKING,
    SHOOTING,
    CLIMBING
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    drivetrain = Drivetrain.getInstance();
    driver = Driver.getInstance(Constants.kControllerPort);

    DataLogManager.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivetrain.periodic(); // Odometri güncellemesi için şart
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    drivetrain.stopModules();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double xSpeed = driver.getForwardSpeed();
    double ySpeed = driver.getStrafeSpeed();
    double rot = driver.getRotationSpeed();

    xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.1 ? rot : 0.0;

    double maxSpeed = SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    double maxRot = SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    
    if (driver.isSlowMode()) {
        maxSpeed /= 2;
        maxRot /= 2;
    }

    double finalX = xLimiter.calculate(xSpeed) * maxSpeed;
    double finalY = yLimiter.calculate(ySpeed) * maxSpeed;
    double finalRot = rotLimiter.calculate(rot) * maxRot;

    if (driver.isResetGyroButtonPressed()) {
        drivetrain.resetGyro();
    }

    drivetrain.drive(finalX, finalY, finalRot, true);
  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}