package com.NewtonSwerve;

import com.ctre.phoenix6.hardware.TalonFX;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    TalonFX getSteerMotor();

    double resetAbsoluteAngle();
}
