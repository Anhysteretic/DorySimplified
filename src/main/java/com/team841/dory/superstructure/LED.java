package com.team841.dory.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.sql.Driver;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

public class LED extends SubsystemBase{
    
    private Shooter shooter;
    private Timer timer;

    private final AddressableLED LED = new AddressableLED(0);
    private final AddressableLEDBuffer Buffer = new AddressableLEDBuffer(61);
    private final AddressableLEDBufferView BufferLeft = Buffer.createView(0, 28);
    private final AddressableLEDBufferView BufferRight = Buffer.createView(29, 60).reversed();
    
    Color yellow = new Color(255, 200, 0);
    Color red = new Color(255, 0, 0);
    Color green = new Color(0, 255, 0);
    Color blue = new Color(0, 0, 255);
    Distance ledSpacing = Meters.of(1 / 120.0);

    LEDPattern baseYellowIdle = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, yellow);
    LEDPattern patternYellowIdle = baseYellowIdle.scrollAtRelativeSpeed(Percent.per(Second).of(60));

    LEDPattern baseBlueEndgameFlash = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, blue);
    LEDPattern patternBlueEndgameFlash = baseBlueEndgameFlash.scrollAtRelativeSpeed(Percent.per(Second).of(100));

    LEDPattern patternYellowSolid = LEDPattern.solid(yellow);
    LEDPattern patternGreenSolid = LEDPattern.solid(green);
    LEDPattern patternBlueSolid = LEDPattern.solid(blue);

    LEDPattern baseRedBreathe = LEDPattern.solid(red);
    LEDPattern patternRedBreathe = baseRedBreathe.breathe(Second.of(0.25));

    public LED(Shooter shooter) {
        this.shooter = shooter;
        this.timer = new Timer();
        LED.setLength(Buffer.getLength());
        LED.setData(Buffer);
        LED.start();
    }

    public void periodic() {
        
        if (DriverStation.getMatchTime() < 21 && DriverStation.getMatchTime() > 20) {
            this.timer.reset();
            this.timer.start();
        }
        
        if (!this.timer.hasElapsed(3) && DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 0.01) {
            patternBlueEndgameFlash.applyTo(BufferLeft);
            patternBlueEndgameFlash.applyTo(BufferRight);
        } else if (!this.shooter.escalatorClear()) {
            patternRedBreathe.applyTo(BufferLeft);
            patternRedBreathe.applyTo(BufferRight);
        } else if (this.shooter.shooterHasCoral()) {
            patternGreenSolid.applyTo(BufferLeft);
            patternGreenSolid.applyTo(BufferRight);
        } else if (DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 0.01) {
            patternBlueSolid.applyTo(BufferLeft);
            patternBlueSolid.applyTo(BufferRight);
        } else if (DriverStation.isDisabled()) {
            patternYellowIdle.applyTo(BufferLeft);
            patternYellowIdle.applyTo(BufferRight);
        } else if (DriverStation.isTeleopEnabled()) {
            patternYellowSolid.applyTo(BufferLeft);
            patternYellowSolid.applyTo(BufferRight);
        } else if (DriverStation.isAutonomousEnabled()) {
            patternYellowIdle.applyTo(BufferLeft);
            patternYellowIdle.applyTo(BufferRight);
        }
        LED.setData(Buffer);
    }
}
