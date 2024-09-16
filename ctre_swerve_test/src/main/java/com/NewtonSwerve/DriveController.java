package com.NewtonSwerve;

import com.ctre.phoenix6.hardware.TalonFX;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    TalonFX getDriveFalcon();
}
