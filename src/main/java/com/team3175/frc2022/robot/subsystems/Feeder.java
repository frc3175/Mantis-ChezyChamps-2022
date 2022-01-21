package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private TalonFX m_feederFalcon = new TalonFX(Constants.FEEDER_FALCON);

    public Feeder() {
        configFeederMotor();
    }

    public void feederRun(double rpm) {
        double falcon_units_100_ms = Conversions.RPMToFalcon(rpm, 1.0);
        m_feederFalcon.set(ControlMode.Velocity, falcon_units_100_ms);
    }

    public void resetEncoders() {
        m_feederFalcon.setSelectedSensorPosition(0);
    }

    public double getEncoderPos() {
        return m_feederFalcon.getSelectedSensorPosition();
    }

    private void configFeederMotor() {
        m_feederFalcon.configFactoryDefault();
        m_feederFalcon.configAllSettings(CTREConfigs.feederFXConfig);
        m_feederFalcon.setInverted(Constants.INVERT_FEEDER);
        m_feederFalcon.setNeutralMode(Constants.FEEDER_NEUTRAL_MODE);
    }

    
    
}