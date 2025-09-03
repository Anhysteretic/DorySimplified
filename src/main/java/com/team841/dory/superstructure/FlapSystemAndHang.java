package com.team841.dory.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.dory.constants.SC;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlapSystemAndHang extends SubsystemBase{

    public TalonFX intakeMotor = new TalonFX(SC.flapSystem.intakeMotor, "rio");
    public TalonFX flapMotor = new TalonFX(SC.flapSystem.flapMotor, "rio");
    public CANrange canrange = new CANrange(SC.flapSystem.intakeCanRangeId, "rio");
    public TalonFX hangMotor = new TalonFX(SC.flapSystem.hangMotor, "rio");
    public TalonFX hangMotor2 = new TalonFX(SC.flapSystem.hangMotor2, "rio");

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode latestStatusCode;

    public FlapSystemAndHang(){
        this.intakeMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        this.flapMotor.getConfigurator().apply(
                SC.flapSystem.configs.withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(5)
                                .withSupplyCurrentLimitEnable(true)));
        this.flapMotor.setNeutralMode(NeutralModeValue.Brake);

        this.canrange.getConfigurator().apply(SC.flapSystem.CanrangeConfigs);

        this.hangMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.hangMotor.setNeutralMode(NeutralModeValue.Brake);

        this.hangMotor2.getConfigurator().apply(SC.flapSystem.configs);
        this.hangMotor2.setNeutralMode(NeutralModeValue.Brake);

        this.hangMotor2.setControl(new Follower(SC.flapSystem.hangMotor, false));
    }

    public Command runIntake() {
        return new RunCommand(
                () -> {
                    this.setIntakeDutyCycle(0.2);
                }
        ).withName("runShooterIntakeCommand")
                .finallyDo(this::stopIntake);
    }

    public boolean flapHasCoral() {
        return this.canrange.getDistance().getValue().magnitude() < 0.05;
    }

    public void setIntakeDutyCycle(double output) {
        this.latestStatusCode = setControlIntake(dutyCycle.withOutput(output));
    }

    public void setFlapperDutyCycle(double output) {
        this.latestStatusCode = setControlFlapper(dutyCycle.withOutput(output));
    }

    public void setHangDutyCycle(double output) {
        this.latestStatusCode = setControlHang(dutyCycle.withOutput(output));
    }

    public StatusCode setControlIntake(DutyCycleOut control) {
        return this.intakeMotor.setControl(control);
    }

    public StatusCode setControlFlapper(DutyCycleOut control) {
        return this.flapMotor.setControl(control);
    }

    public StatusCode setControlHang(DutyCycleOut control) {
        return this.hangMotor.setControl(control);
    }

    public void stopIntake() {
        this.intakeMotor.stopMotor();
    }

    public void stopFlapper() {
        this.flapMotor.stopMotor();
    }

    public void stopHang() {
        this.hangMotor.stopMotor();
    }
}
