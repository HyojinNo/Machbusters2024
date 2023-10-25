package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private double lastAngle;
  private double angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  private int encoderResetCounter = 0;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    // /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    // angleEncoder.setPosition(angleOffset);

  }

  private void configAngleMotor() {
    angleEncoder.configMagnetOffset(0);
    //Timer.delay(1);
    SmartDashboard.putNumber("CANCoder Initial Value " + moduleNumber, angleEncoder.getAbsolutePosition());
    angleEncoder.configMagnetOffset(angleOffset);
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    // TODO: Make this the CANCoder some day.
    // angleController.setFeedbackDevice(integratedAngleEncoder);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
    Timer.delay(1);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.angleKP);
    driveController.setI(Constants.Swerve.angleKI);
    driveController.setD(Constants.Swerve.angleKD);
    driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void resetToAbsolute() {
    this.resetToAbsolute(false);
  }

  private void resetToAbsolute(boolean resetQuickly) {
    // If you read the angle immediately, the magnetic offset won't be set yet. Wait
    // a second and it'll be there.
    if (!resetQuickly) {
      Timer.delay(1);
    }
    double canCoderDegrees = getCanCoderAbsolutePosition();
    // integratedAngleEncoder.setPosition((actualDegrees*(Constants.Swerve.angleConversionFactor))*Constants.Swerve.numberOfSensorCountsPerRevolution);
    DriverStation.reportWarning("Module: " + moduleNumber + " CanCoderDegrees:  " + canCoderDegrees
        + " AngleOffset: " + angleOffset, false);
    integratedAngleEncoder.setPosition(canCoderDegrees);

  }

  public void updateDashboardCancoders() {
    double canCoderDegrees = getCanCoderAbsolutePosition();
    double angleDegrees = angleOffset;
    double absolutePosition = canCoderDegrees - angleDegrees;
    SmartDashboard.putNumber("M1- CanDegrees: " + moduleNumber, canCoderDegrees);
    SmartDashboard.putNumber("M1- AngleOffsetDegrees: " + moduleNumber, angleDegrees);
    SmartDashboard.putNumber("M1- Setting angle to: " + moduleNumber, absolutePosition);
    SmartDashboard.putNumber("M1- Integrated Angle Motor Position: " + moduleNumber,
        integratedAngleEncoder.getPosition());
  }

  public void resetToAbsoluteNorth() {
    resetToAbsolute(true);
    double canCoderDegrees = getCanCoderAbsolutePosition();
    double angleDegrees = angleOffset;
    double absolutePosition = canCoderDegrees - angleDegrees;
    updateDashboardCancoders();
    angleController.setReference(0, ControlType.kPosition);
    lastAngle = 0;
    // this.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      if (moduleNumber == 1) {
        SmartDashboard.putNumber("wheel 1 speed", percentOutput);
      }
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    setAngle(desiredState, true);
  }

  private void setAngle(SwerveModuleState desiredState, boolean jitterCheck) {
    SmartDashboard.putNumber("setAngle A: " + moduleNumber, desiredState.angle.getDegrees());
    updateDashboardCancoders();
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    double angle = (jitterCheck && Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle.getDegrees();

    // double cancoderAngle = this.getCanCoderAbsolutePosition();
    // this.integratedAngleEncoder.setPosition(cancoderAngle);

    SmartDashboard.putNumber("Angle: " + moduleNumber, angle);
    SmartDashboard.putNumber("Last Angle: " + moduleNumber,
        lastAngle);
    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    setDesiredState(desiredState, isOpenLoop, true);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean jitterCheck) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    SmartDashboard.putNumber("setDesiredState: " + moduleNumber, desiredState.angle.getDegrees());
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    SmartDashboard.putNumber("setDesiredState B: " + moduleNumber, desiredState.angle.getDegrees());
    setAngle(desiredState, jitterCheck);
    setSpeed(desiredState, isOpenLoop);

  }

  public double getInternalAngle() {
    return integratedAngleEncoder.getPosition();
  }

  public double getCanCoderAbsolutePosition() {
    return angleEncoder.getAbsolutePosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getInternalAngle()));
  }

  public SwerveModulePosition getPosition() {
    SmartDashboard.putNumber("angleEncoder position " + moduleNumber, angleEncoder.getPosition());
    SmartDashboard.putNumber("angleOffset degrees " + moduleNumber, angleOffset);

    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromDegrees(angleEncoder.getPosition()));
  }

  

}