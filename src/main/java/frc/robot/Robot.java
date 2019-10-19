package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  public enum AutoMode {
    None, TeleDrive
  }

  public enum DriveState {
    Stop, Arcade
  }

  public enum ArmState {
    Stop, Manual, PID1, PID2, PID3, PID4
  }

  public enum RollerState {
    Stop, Manual, In, Out
  }

  public enum PistonState {
    Hatch, Ball
  }

  private boolean armIsReset = false;
  private DriveState driveState = DriveState.Arcade;
  private ArmState armState = ArmState.Manual;
  private RollerState rollerState = RollerState.Stop;
  private PistonState pistonState = PistonState.Ball;
  private AutoMode autoMode = AutoMode.None;
  SendableChooser<AutoMode> autoChooser = new SendableChooser<>();

  private TalonSRX leftMaster = new TalonSRX(3);
  private TalonSRX rightMaster = new TalonSRX(1);
  private VictorSPX leftSlave = new VictorSPX(1);
  private VictorSPX rightSlave = new VictorSPX(2);

  private TalonSRX armMotor = new TalonSRX(5);
  private VictorSPX armSlave = new VictorSPX(3);

  private TalonSRX leftRollerMotor = new TalonSRX(4);
  private TalonSRX rightRollerMotor = new TalonSRX(5);
  private TalonSRX topRollerMotor = new TalonSRX(7);

  private Compressor compressor = new Compressor();
  private DoubleSolenoid piston = new DoubleSolenoid(0, 1);
  private DoubleSolenoid frontClimb = new DoubleSolenoid(2, 3);
  private DoubleSolenoid backClimb = new DoubleSolenoid(4, 5);
  private DigitalInput armLimitSwitch = new DigitalInput(0);

  private Joystick driverJoystick = new Joystick(0);
  private Joystick operatorJoystick = new Joystick(1);

  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
  private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;
  private final double kDriveSecondsToMaxPower = 0.5;

  double prevTimestamp, prevDrivePower;

  @Override
  public void robotInit() {

    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    armSlave.follow(armMotor);
    armSlave.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(false);
    armMotor.setSensorPhase(true);
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);

    armMotor.configReverseSoftLimitThreshold(0, 10);
    armMotor.configForwardSoftLimitThreshold((int) (175 / kArmTick2Deg), 10);
    armMotor.configReverseSoftLimitEnable(true, 10);
    armMotor.configForwardSoftLimitEnable(true, 10);

    armMotor.config_kP(0, 0.6, 10);
    armMotor.config_kI(0, 0.002, 10);
    armMotor.config_kD(0, 0.008, 10);
    armMotor.config_kF(0, 0, 10);
    armMotor.config_IntegralZone(0, (int) (10 / kArmTick2Deg), 10);

    compressor.start();

    autoChooser.setDefaultOption("None", AutoMode.None);
    autoChooser.addOption("Tele-op Drive", AutoMode.TeleDrive);
  }

  @Override
  public void robotPeriodic() {
    displayStates();
    prevTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousInit() {
    activateMotors(true);
    autoMode = autoChooser.getSelected();
    switch (autoMode) {
    case TeleDrive:
      driveState = DriveState.Arcade;
      break;
    default:
    }
    System.out.println("Auto starting with mode: " + autoMode);
  }

  @Override
  public void autonomousPeriodic() {
    updateDrive();
    updateArm();
    updatePistons();
    updateRollers();
  }

  @Override
  public void teleopInit() {
    activateMotors(true);
  }

  @Override
  public void teleopPeriodic() {
    updateDrive();
    updateArm();
    updatePistons();
    updateRollers();
  }

  @Override
  public void disabledInit() {
    activateMotors(false);
    prevDrivePower = 0;
    driveState = DriveState.Stop;
    armState = ArmState.Stop;
    rollerState = RollerState.Stop;
  }

  @Override
  public void disabledPeriodic() {
    try2ResetArmEncoder();
  }

  public void try2ResetArmEncoder() {
    if (!armLimitSwitch.get()) {
      armMotor.setSelectedSensorPosition(0);
      armIsReset = true;
    }
  }

  public void updateDrive() {
    if (Math.abs(driverJoystick.getRawAxis(1)) > 0.1) {
      driveState = DriveState.Arcade;
    } else {
    }

    switch (driveState) {
    case Arcade:
      arcadeDrive();
      break;
    case Stop:
      drive(0, 0);
      break;
    default:
    }
  }

  public void updateArm() {
    if (Math.abs(operatorJoystick.getRawAxis(1)) > 0.1) {
      armState = ArmState.Manual;
    } else if (operatorJoystick.getRawButton(5)) {
      armState = ArmState.PID1;
    } else if (operatorJoystick.getRawButton(6)) {
      armState = ArmState.PID2;
    } else if (operatorJoystick.getRawButton(7)) {
      armState = ArmState.PID3;
    } else if (operatorJoystick.getRawButton(8)) {
      armState = ArmState.PID4;
    } else {
    }

    if (!armIsReset) {
      if (armState != ArmState.Stop) {
        armState = ArmState.Manual;
      }
    }

    switch (armState) {
    case PID1:
      armMotor.set(ControlMode.Position, 0);
      break;
    case PID2:
      armMotor.set(ControlMode.Position, 30);
      break;
    case PID3:
      armMotor.set(ControlMode.Position, 100);
      break;
    case PID4:
      armMotor.set(ControlMode.Position, 140);
      break;
    case Manual:
      armMotor.set(ControlMode.PercentOutput, operatorJoystick.getRawAxis(1));
      try2ResetArmEncoder();
      break;
    case Stop:
      armMotor.set(ControlMode.PercentOutput, 0);
      break;
    default:
    }

  }

  public void updatePistons() {
    if (operatorJoystick.getRawButton(1)) {
      pistonState = PistonState.Ball;
    } else if (operatorJoystick.getRawButton(2)) {
      pistonState = PistonState.Hatch;
    } else {
    }

    switch (pistonState) {
    case Ball:
      piston.set(Value.kForward);
      break;
    case Hatch:
      piston.set(Value.kReverse);
      break;
    default:
    }
  }

  public void updateRollers() {
    if (Math.abs(operatorJoystick.getRawAxis(4)) > 0.1) {
      rollerState = RollerState.Manual;
    } else if (operatorJoystick.getRawButton(3)) {
      rollerState = RollerState.In;
    } else if (operatorJoystick.getRawButton(4)) {
      rollerState = RollerState.Out;
    } else {
    }

    double power = 1;
    switch (rollerState) {
    case Stop:
      leftRollerMotor.set(ControlMode.PercentOutput, 0);
      rightRollerMotor.set(ControlMode.PercentOutput, 0);
      topRollerMotor.set(ControlMode.PercentOutput, 0);
      break;
    case Manual:
      leftRollerMotor.set(ControlMode.PercentOutput, operatorJoystick.getRawAxis(4));
      rightRollerMotor.set(ControlMode.PercentOutput, operatorJoystick.getRawAxis(4));
      topRollerMotor.set(ControlMode.PercentOutput, operatorJoystick.getRawAxis(4));
      break;
    case In:
      if (pistonState == PistonState.Ball) {
        power = -1;
        leftRollerMotor.set(ControlMode.PercentOutput, power);
        rightRollerMotor.set(ControlMode.PercentOutput, power);
        topRollerMotor.set(ControlMode.PercentOutput, power);
      } else {
        power = 1;
        leftRollerMotor.set(ControlMode.PercentOutput, power);
        rightRollerMotor.set(ControlMode.PercentOutput, power);
      }
    case Out:
      if (pistonState == PistonState.Ball) {
        power = -1;
        leftRollerMotor.set(ControlMode.PercentOutput, power);
        rightRollerMotor.set(ControlMode.PercentOutput, power);
        topRollerMotor.set(ControlMode.PercentOutput, power);
      } else {
        power = 1;
        leftRollerMotor.set(ControlMode.PercentOutput, power);
        rightRollerMotor.set(ControlMode.PercentOutput, power);
      }
      break;
    default:
    }
  }

  public void updateClimb() {
    if (operatorJoystick.getRawButton(9))
      frontClimb.set(Value.kReverse);
    else
      frontClimb.set(Value.kForward);

    if (operatorJoystick.getRawButton(10))
      backClimb.set(Value.kReverse);
    else
      backClimb.set(Value.kForward);
  }

  public void arcadeDrive() {
    double speed = -driverJoystick.getRawAxis(1) * 0.6;
    double turn = driverJoystick.getRawAxis(4) * 0.3;

    speed = applyDeadband(speed);
    turn = applyDeadband(turn);

    accCurve(speed);
    drive(speed + turn, speed - turn);
  }

  public void drive(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, left);
    rightMaster.set(ControlMode.PercentOutput, right);
  }

  private double applyDeadband(double value) {
    if (Math.abs(value) > 0.05)
      return value;
    return 0;
  }

  private double accCurve(double value) {
    double dt = Timer.getFPGATimestamp() - prevTimestamp;
    final double maxChange = 1.0 * dt / kDriveSecondsToMaxPower;
    if (value - prevDrivePower > maxChange) {
      value = prevDrivePower + maxChange;
    } else if (value - prevDrivePower < -maxChange) {
      value = prevDrivePower - maxChange;
    }
    prevDrivePower = value;
    return value;
  }

  public void displayStates() {
    SmartDashboard.putNumber("Timestamp", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Arm Encoder", armMotor.getSelectedSensorPosition(0) * kArmTick2Deg);
    SmartDashboard.putNumber("Drive Left Encoder", leftMaster.getSelectedSensorPosition(0) * kDriveTick2Feet);
    SmartDashboard.putNumber("Drive Right Encoder", rightMaster.getSelectedSensorPosition(0) * kDriveTick2Feet);

    SmartDashboard.putString("Arm State:", armState.toString());
    SmartDashboard.putString("Drive State:", driveState.toString());
    SmartDashboard.putString("Piston State:", pistonState.toString());
    SmartDashboard.putString("Rollers State:", rollerState.toString());
    SmartDashboard.putBoolean("Arm IsReset", armIsReset);
  }

  public void activateMotors(boolean on) {
    NeutralMode mode;
    if (on)
      mode = NeutralMode.Brake;
    else
      mode = NeutralMode.Coast;
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    armMotor.setNeutralMode(mode);
    armSlave.setNeutralMode(mode);
    rollerMotor.setNeutralMode(mode);
  }

  @Override
  public void testPeriodic() {

  }
}