package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

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
}