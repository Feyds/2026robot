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
import frc.robot.subsystems.Vision.Limelight;
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
  private Limelight limelight;

  public enum RobotState {
    IDLE,
    INTAKING,
    LOADED,
    AUTO_ALIGN,
    SHOOTING,
    CLIMBING
  }

  private RobotState state = RobotState.IDLE;
  private RobotState previousState = RobotState.IDLE;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private Timer climbTimer = new Timer();
  private static final double maxclimbtime = 4.0;

  private boolean emergencyStop = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    drivetrain = Drivetrain.getInstance();
    driver = Driver.getInstance(Constants.kControllerPort);
    limelight = Limelight.getInstance();

    intake = Intake.getInstance();
    loader = Loader.getInstance();
    shooter = Shooter.getInstance();
    climber = Climber.getInstance();

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
    emergencyStop = false;
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
  public void teleopInit() {
    climbTimer.stop();
    climbTimer.reset();
    emergencyStop = false;
    state = RobotState.IDLE;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (driver.isEmergencyStopPressed()) {
      emergencyStop = true;
    }

    if (emergencyStop) {
      drivetrain.stopModules();
      intake.disable();
      loader.stop();
      shooter.stop();
      climber.stop();
      SmartDashboard.putString("Robot/State", "EMERGENCY_STOP");
      return;
    }

    double x = Math.abs(driver.getForwardSpeed()) > 0.1 ? driver.getForwardSpeed() : 0;
    double y = Math.abs(driver.getStrafeSpeed()) > 0.1 ? driver.getStrafeSpeed() : 0;
    double rot = Math.abs(driver.getRotationSpeed()) > 0.1 ? driver.getRotationSpeed() : 0;

    double maxSpeed = SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    double maxRot = SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    if (driver.isSlowMode()) {
      maxSpeed *= 0.5;
      maxRot *= 0.5;
    }

    double finalX = xLimiter.calculate(x) * maxSpeed;
    double finalY = yLimiter.calculate(y) * maxSpeed;
    double finalRot = rotLimiter.calculate(rot) * maxRot;

    if (driver.isResetGyroButtonPressed()) {
      drivetrain.resetGyro();
    }

    if (state != RobotState.AUTO_ALIGN) {
      drivetrain.drive(finalX, finalY, finalRot, true);
    }

    if (driver.isClimbPressed()) {
      state = RobotState.CLIMBING;
    }

    switch (state) {

      case IDLE:
        intake.moveTo(IntakeConstants.stowAngle);
        intake.roller(false);
        loader.stop();
        shooter.stop();
        climber.stop();

        if (driver.isIntakePressed()) state = RobotState.INTAKING;
        else if (driver.isShootPressed()) state = RobotState.SHOOTING;
        else if (driver.isAutoAlignHeld() && state != RobotState.CLIMBING) enterAutoAlign();
        break;

      case INTAKING:
        intake.moveTo(IntakeConstants.intakeAngle);
        intake.roller(true);
        loader.run(driver.isManualLoaderPressed());

        if (driver.isIntakeStowPressed()) state = RobotState.LOADED;
        else if (driver.isAutoAlignHeld() && state != RobotState.CLIMBING) enterAutoAlign();
        break;

      case LOADED:
        intake.moveTo(IntakeConstants.stowAngle);
        intake.roller(false);
        loader.stop();
        shooter.stop();

        if (driver.isShootPressed()) state = RobotState.SHOOTING;
        else if (driver.isIntakePressed()) state = RobotState.INTAKING;
        else if (driver.isAutoAlignHeld() && state != RobotState.CLIMBING) enterAutoAlign();
        break;

      case SHOOTING:
        if (limelight.hasTarget()) {
          shooter.setTargetRPMFromDistance(limelight.getDistanceMeters());
        }

        shooter.shoot();
        intake.moveTo(IntakeConstants.feedAngle);
        intake.roller(true);

        if (shooter.atSetpoint() || driver.isManualLoaderPressed()) {
          loader.run(true);
        } else {
          loader.stop();
        }

        if (!driver.isShootPressed()) {
          loader.stop();
          shooter.stop();
          state = RobotState.IDLE;
        } else if (driver.isAutoAlignHeld() && state != RobotState.CLIMBING) enterAutoAlign();
        break;

      case CLIMBING:
        intake.disable();
        loader.stop();
        shooter.stop();

        if (!climbTimer.isRunning()) {
          climbTimer.reset();
          climbTimer.start();
        }

        climber.climb();

        if (climbTimer.hasElapsed(maxclimbtime) || !driver.isClimbPressed()) {
          climber.stop();
          climbTimer.stop();
          state = RobotState.IDLE;
        }
        break;

      case AUTO_ALIGN:
        double visionRot = 0;

        if (limelight.hasTarget()) {
          visionRot = Math.max(Math.min(limelight.getTx() * 0.02, 2.0), -2.0);
        }

        drivetrain.drive(finalX * 0.3, finalY * 0.3, visionRot, true);

        if (!driver.isAutoAlignHeld()) {
          state = previousState;
        }
        break;
    }

    SmartDashboard.putString("Robot/State", state.name());
    SmartDashboard.putNumber("Intake/Angle", intake.getAngle());
    SmartDashboard.putNumber("Shooter/RPM", shooter.getRPM());
    SmartDashboard.putNumber("Climber/Time", climbTimer.get());
    SmartDashboard.putBoolean("Shooter/Ready", shooter.atSetpoint());
    SmartDashboard.putBoolean("Climber/Active", state == RobotState.CLIMBING);
    SmartDashboard.putBoolean("Drive/SlowMode", driver.isSlowMode());
    SmartDashboard.putBoolean("Vision/HasTarget", limelight.hasTarget());
    SmartDashboard.putNumber("Vision/Tx", limelight.getTx());
    SmartDashboard.putNumber("Vision/Distance", limelight.getDistanceMeters());
    SmartDashboard.putBoolean("Vision/Aligned", limelight.isAligned());
    SmartDashboard.putNumber("Shooter/TargetRPM", shooter.getTargetRPM());
  }

  private void enterAutoAlign() {
    previousState = state;
    state = RobotState.AUTO_ALIGN;
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