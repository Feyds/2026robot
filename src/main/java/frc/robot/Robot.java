// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter; // Hızlanmayı yumuşatmak için
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Loader.Loader;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.controller.Driver;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private Drivetrain drivetrain;
  private Intake intake;
  private Loader loader;
  private Shooter shooter;
  private Climber climber;

  private Driver driver;

  public enum RobotState {
    IDLE,
    INTAKING,
    LOADED,
    SHOOTING,
    CLIMBING
  }

  private RobotState state = RobotState.IDLE;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private Timer climbTimer = new Timer();
  private static final double maxclimbtime = 4.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    drivetrain = Drivetrain.getInstance();
    driver = Driver.getInstance(Constants.kControllerPort);

    intake = new Intake();
    loader = new Loader();
    shooter = new Shooter();
    climber = new Climber();

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
    drivetrain.periodic(); // Odometry
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

    if(driver.isEmergencyStopPressed()) {
      state = RobotState.IDLE;

      intake.disable();
      loader.stop();
      shooter.stop();
      climber.stop();

      SmartDashboard.putString("Robot/State", "EMERGENCY STOP");
      return;
    }

    if(driver.isClimbPressed()) {
      state = RobotState.CLIMBING;
    }

    switch(state) {

      case IDLE:
        climbTimer.stop();
        climbTimer.reset();
        intake.moveTo(IntakeConstants.stowAngle);
        intake.roller(false);

        loader.stop();
        shooter.stop();
        climber.stop();

        if(driver.isIntakePressed()) {
          state = RobotState.INTAKING;
        } else if(driver.isShootPressed()) {
          state = RobotState.SHOOTING;
        }
        break;
      
      case INTAKING:
        intake.moveTo(IntakeConstants.intakeAngle);
        intake.roller(true);

        loader.run(driver.isManualLoaderPressed());

        if(driver.isIntakeStowPressed()) {
          intake.roller(false);
          state = RobotState.LOADED;
        }
        break;

      case LOADED:
        intake.moveTo(IntakeConstants.stowAngle);
        intake.roller(false);

        loader.stop();
        shooter.stop();

        if(driver.isShootPressed()) {
          state = RobotState.SHOOTING;
        }

        if(driver.isIntakePressed()) {
          state = RobotState.INTAKING;
        }
        break;

      case SHOOTING:
        shooter.shoot();

        intake.moveTo(IntakeConstants.feedAngle);
        intake.roller(true);

        if(shooter.atSetpoint()) {
          loader.run(true);
        } else {
          loader.stop();
        }

        if(driver.isManualLoaderPressed()) {
          loader.run(true);
        }

        if(!driver.isShootPressed()) {
          loader.stop();
          shooter.stop();
          state = RobotState.IDLE;
        }
        break;

      case CLIMBING:
        intake.disable();
        loader.stop();
        shooter.stop();

        if(!climbTimer.isRunning()) {
          climbTimer.reset();
          climbTimer.start();
        }

        climber.climb();

        if(climbTimer.hasElapsed(maxclimbtime)) {
          climber.stop();
          climbTimer.stop();
          state = RobotState.IDLE;
        }

        if(!driver.isClimbPressed()) {
          climber.stop();
          state = RobotState.IDLE;
        }
        break;
    }

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

    SmartDashboard.putString("Robot/State", state.name());
    SmartDashboard.putNumber("Intake/Angle", intake.getAngle());
    SmartDashboard.putNumber("Shooter/RPM", shooter.getRPM());
    SmartDashboard.putNumber("Climber/Time", climbTimer.get());
    SmartDashboard.putBoolean("Shooter/Ready", shooter.atSetpoint());
    SmartDashboard.putBoolean("Climber/Active", state == RobotState.CLIMBING);
    SmartDashboard.putBoolean("Drive/SlowMode", driver.isSlowMode());
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