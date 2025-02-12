package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    
    /*
     * https://docs.revrobotics.com/revlib/24-to-25
     * CANSparkMax is now SparkMax
     * CANSparkFlex is now SparkFlex
     * CANSparkLowLevel is now SparkLowLevel
     * SparkPIDController is now SparkClosedLoopController
     */
    private final SparkMax driveMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMax turnMotor;
    private final SparkMaxConfig turnConfig;

    //private final RelativeEncoder driveEncoder;
    //private final RelativeEncoder turnEncoder;

    private final PIDController turnPidController;
    //private final SparkClosedLoopController turnSparkPIDController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, 
                        int turnMotorId, 
                        boolean driveMotorReversed, 
                        boolean turnMotorReversed, 
                        int absoluteEncoderId, 
                        double absoluteEncoderOffset, 
                        boolean isAbsoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = isAbsoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        // Config Motors
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        // Set motor values
        driveConfig = new SparkMaxConfig();
        driveConfig.inverted(driveMotorReversed).idleMode(IdleMode.kCoast);
        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter).velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveConfig.smartCurrentLimit(40);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnConfig = new SparkMaxConfig();
        turnConfig.inverted(turnMotorReversed).idleMode(IdleMode.kCoast);
        turnConfig.encoder.positionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad).velocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);
        turnConfig.closedLoop.pid(turnMotorId, absoluteEncoderId, absoluteEncoderOffset);
        turnConfig.smartCurrentLimit(40);
        driveMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //RoboRIO PID controller, should switch to Spark max pid controller
        turnPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Never used in 2024 code
        //turnSparkPIDController = turnMotor.getClosedLoopController(); //turnMotor.getPIDController();

        //CANcoder.optimizeBusUtilizationForAll(absoluteEncoder);
        absoluteEncoder.optimizeBusUtilization();

        resetEncoders();
        
        // 2024 way of doing it
        //driveMotor.restoreFactoryDefaults();
        //driveMotor.setInverted(driveMotorReversed);
        //driveEncoder = driveMotor.getEncoder();
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //driveMotor.setSmartCurrentLimit(40);
        //driveMotor.setIdleMode(IdleMode.kCoast);
        //driveMotor.burnFlash();

        //turnMotor.restoreFactoryDefaults();
        //turnMotor.setInverted(turnMotorReversed);
        //turnEncoder = turnMotor.getEncoder();
        //turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        //turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);
        //turnMotor.setSmartCurrentLimit(40);
        //turnMotor.setIdleMode(IdleMode.kCoast);
        //turnMotor.burnFlash();
    }

    public double getDrivePosition() {
        return driveMotor.configAccessor.encoder.getPositionConversionFactor();
        //return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnMotor.configAccessor.encoder.getPositionConversionFactor();
        //return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        //SmartDashboard.putNumber("wheelSpeed",driveEncoder.getVelocity());
        return driveMotor.configAccessor.encoder.getVelocityConversionFactor();
        //return driveEncoder.getVelocity();

    }

    public double getTurningVelocity() {
        return turnMotor.configAccessor.encoder.getVelocityConversionFactor();
        //return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turnMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
        //driveEncoder.setPosition(0);
        //turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public void resetTurn(){
        turnMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
        //double position = getAbsoluteEncoderRad();
        //turnEncoder.setPosition(position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state.optimize(getState().angle);
        //state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void stop() {
        //driveMotor.setIdleMode(IdleMode.kBrake);
        //turnMotor.setIdleMode(IdleMode.kBrake);
        driveConfig.idleMode(IdleMode.kBrake);
        driveMotor.configure(driveConfig, null, null);
        turnConfig.idleMode(IdleMode.kBrake);
        turnMotor.configure(turnConfig, null, null);
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition(){
        //driveEncoder.getPosition()
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(),
            Rotation2d.fromDegrees(absoluteEncoder.getPosition().getValueAsDouble() - absoluteEncoderOffsetRad));
      }
}