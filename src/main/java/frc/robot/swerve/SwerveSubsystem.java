package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.CompBotTunerConstants.TunerSwerveDrivetrain;

public class SwerveSubsystem extends SubsystemBase {
  public static final double MaxSpeed = 4.75;
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          CompBotTunerConstants.DrivetrainConstants,
          CompBotTunerConstants.FrontLeft,
          CompBotTunerConstants.FrontRight,
          CompBotTunerConstants.BackLeft,
          CompBotTunerConstants.BackRight);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(0.07)
          .withRotationalDeadband(0.05);

  private final CommandXboxController driverController;
  private double lastSimTime;
  private Notifier simNotifier = null;
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();

  public SwerveSubsystem(CommandXboxController driverController) {
    this.driverController = driverController;
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public void zeroGyro() {
    drivetrain.seedFieldCentric();
  }

  public void driveTeleop(double x, double y, double theta) {
    var leftJoystickMagnitude = Math.hypot(x, y);
    var rightJoystickMagnitude = Math.abs(theta);

    var translation =
        new Translation2d(
            leftJoystickMagnitude, x == 0 && y == 0 ? Rotation2d.kZero : new Rotation2d(x, y));
    var rotation =
        new Translation2d(
            rightJoystickMagnitude, theta == 0 ? Rotation2d.kZero : new Rotation2d(theta, 0));

    var leftX = translation.getX();
    var leftY = -1 * translation.getY();
    var rightX = rotation.getX();

    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    teleopSpeeds =
        new ChassisSpeeds(
            -1.0 * leftY * MaxSpeed,
            leftX * MaxSpeed,
            rightX * TELEOP_MAX_ANGULAR_RATE.getRadians());

    sendSwerveRequest();
  }

  @Override
  public void periodic() {
    DogLog.log("Swerve/RobotPose", drivetrain.getState().Pose);

    if (DriverStation.isTeleopEnabled()) {
      driveTeleop(
          driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX());

      if (driverController.getHID().getBackButtonPressed()) {
        zeroGyro();
      }
    }
  }

  private void sendSwerveRequest() {
    drivetrain.setControl(
        drive
            .withVelocityX(teleopSpeeds.vxMetersPerSecond)
            .withVelocityY(teleopSpeeds.vyMetersPerSecond)
            .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
