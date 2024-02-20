// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    public double m_angleKP;
    public double m_angleKI;
    public double m_angleKD;
    public double m_angleKFF;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;

    private CANcoder angleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    // creates a feedforward for the swerve drive. feedforward does 90% of the work,
    // estimating stuff
    // PID fixes the error

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.m_angleKP = Constants.Swerve.angleKP;
        this.m_angleKI = Constants.Swerve.angleKI;
        this.m_angleKD = Constants.Swerve.angleKD;
        this.m_angleKFF = Constants.Swerve.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        // this.?
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public double getVoltage() {
        return driveMotor.getBusVoltage();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous
        // controller which
        // REV supports this now so dont have to worry with rev, but need some funky
        // configs i dont want to do
        // have to be sad with falcons but thats what you get for giving money to Tony
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            // when not taking feedback
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
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
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // the ? and : are a shorthand for an if-else loop
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition); // may need to change

    }

    public Rotation2d getCanCoder() {
        angleEncoder.getAbsolutePosition().refresh();
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue() * 360);
    }

    private void configAngleEncoder() {
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kSensorDataOnly);
        MagnetSensorConfigs f = new MagnetSensorConfigs();
        f.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        f.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        angleEncoder.getConfigurator().apply(f);

    }

    private void configAngleMotor() {
        // resets angle motor
        angleMotor.restoreFactoryDefaults();
        // limits can bus usage
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        // sets current limit
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        // sets inversion
        angleMotor.setInverted(Constants.Swerve.angleInvert);
        // sets brake mode or not
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        // sets a conversion factor for the encoder so it output correlates with the
        // rotation of the module
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        // oops pid loop time sets the pid
        angleController.setP(m_angleKP);
        angleController.setI(m_angleKI);
        angleController.setD(m_angleKD);
        angleController.setFF(m_angleKFF);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        // burns spark max
        angleMotor.burnFlash();

        Timer.delay(1.0);
        // resets to the cancoder
        resetToAbsolute();
    }

    private void configDriveMotor() {
        // factory resets the spark max
        driveMotor.restoreFactoryDefaults();
        // full utilisation on the can loop hell yea
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        // sets current limit
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        // sets inverted or not
        driveMotor.setInverted(Constants.Swerve.driveInvert);
        // sets brake mode or not
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        // sets encoder to read velocities as meters per second
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        // sets encoder to read positions as meters traveled
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        // pid setting fun
        driveController.setP(Constants.Swerve.driveKP);
        driveController.setI(Constants.Swerve.driveKI);
        driveController.setD(Constants.Swerve.driveKD);
        driveController.setFF(Constants.Swerve.driveKFF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        // burns to spark max
        driveMotor.burnFlash();
        // resets encoder position to 0
        driveEncoder.setPosition(0.0);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public void sysIdVolt(double in) {
        driveMotor.setVoltage(in);
    }

}