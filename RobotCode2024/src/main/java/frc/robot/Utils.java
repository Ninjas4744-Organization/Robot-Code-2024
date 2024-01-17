package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Utils {
    public static MotionMagicVoltage getMotionMagicConfig(){
        MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = Constants.kMotionMagicCruiseVelocity;
        mm.MotionMagicAcceleration = Constants.kMotionMagicAcceleration;
        mm.MotionMagicJerk = Constants.kMotionMagicJerk;
        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = Constants.kP;
        slot0.kI = Constants.kI;
        slot0.kD = Constants.kD;
        slot0.kV = Constants.kV;
        slot0.kS = Constants.kS;
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = Constants.kSensorToMechanismRatio;

        return m_mmReq;
    }

    public static SysIdRoutine getSysIdRoutine(TalonFX motor, Elevator elevator){
        MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
        MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
        MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

        return new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    motor.setVoltage(volts.in(Volts));
                },
                log -> {
                    log.motor("Motor")
                        .voltage(m_appliedVoltage.mut_replace(motor.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(motor.getPosition().getValue(), Meters))
                        .linearVelocity(m_velocity.mut_replace(motor.getVelocity().getValue(), MetersPerSecond));
                }, elevator));
    }
}