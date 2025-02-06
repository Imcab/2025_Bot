package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotState;

public class Leds{

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private static final int kport = 9;
    private static final int klength = 120; 
    private static final double kBlink = 1.0;
    private static final double kFastBlink = 0.5;
    private static final double kBreath = 3;

    public Leds(){

        ledStrip = new AddressableLED(kport);
        ledBuffer = new AddressableLEDBuffer(klength);
        ledStrip.setLength(klength);
        ledStrip.start();

    }

    public void periodic(){
        ledStrip.setData(ledBuffer);
    }

    //Sets an efect to the LED strip
    public void sendEffect(LEDPattern effect){
        effect.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
    }

    //LOW Brigthnes and Breathe effect in color oranges
    public void robotDisabled(){

        LEDPattern orange =  LEDPattern.solid(Color.kOrange).atBrightness(Percent.of(50));

        LEDPattern effect = orange.breathe(Seconds.of(kBreath));

        sendEffect(effect);
    }

    //Blink effect in color orange and red 
    public void elevatorRetract(){

        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kOrange, Color.kRed);
        LEDPattern pattern3 = base.blink(Seconds.of(kBlink));
        
        sendEffect(pattern3);
    }

    //Blink effect in color orange and yellow 
    public void elevatorL2(){

        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrange, Color.kYellow);
        LEDPattern pattern4 = base.blink(Seconds.of(kBlink));
        
        sendEffect(pattern4);
    }

    //Blink effect in color green and blue 
    public void elevatorL3(){

        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlue);
        LEDPattern pattern5 = base.blink(Seconds.of(kBlink));
        
        sendEffect(pattern5);
    }

    //Blink effect in color pink and violet pink 
    public void elevatorL4(){

        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kPink, Color.kPaleVioletRed);
        LEDPattern pattern6 = base.blink(Seconds.of(kBlink));
        
        sendEffect(pattern6);
    }

    //Elevador Intake
    public void elevatorIntake(){
    
        LEDPattern elevadorInBase = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kPurple, Color.kOrange);
        LEDPattern elevadorIn = elevadorInBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        
        sendEffect(elevadorIn);
    }

    //Activar Rueda de Intake
    public void coralIntake(){

        LEDPattern ruedaInBase = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        LEDPattern ruedaIn = ruedaInBase.blink(Seconds.of(kFastBlink));
        
        sendEffect(ruedaIn);
    }


    //NO PIECE (estado default) 
    public void idle(){

        LEDPattern base = RobotState.isRed() ? LEDPattern.solid(Color.kRed) : LEDPattern.solid(Color.kBlue);
        LEDPattern noPiece = base.breathe(Seconds.of(kBreath));

        sendEffect(noPiece);
    }

    //Coral Cargado
    public void coralCharged(){

        LEDPattern coralCargado = LEDPattern.solid(Color.kWhite);
        
        sendEffect(coralCargado);
    }

    //Expulsar Coral (Reversa Rueda del Intake)
    public void coralEjected(){

        LEDPattern expulsarCoralBase = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kPink);
        LEDPattern expulsarCoral = expulsarCoralBase.blink((Seconds).of(kFastBlink));
        
        sendEffect(expulsarCoral);
    }

    //Agarrar Pelota
    public void algaeIntaking(){

        LEDPattern agarrarPelotaBase = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kWhite, Color.kBlue);
        LEDPattern agarrarPelota = agarrarPelotaBase.blink((Seconds).of(kFastBlink));
        
        sendEffect(agarrarPelota);
    }

    //Expulsar Pelota
    public void algaeEjected(){
    
        LEDPattern expulsarPelotaBase = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kPink);
        LEDPattern expulsarPelotablink = expulsarPelotaBase.blink((Seconds).of(kFastBlink));
        LEDPattern expulsarPelota = expulsarPelotablink.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        
        sendEffect(expulsarPelota);
    }

    //Scrolling rainbow effect 
    public void climb(){

        LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        Distance spacing = Meters.of(1 / 120.0); 
        LEDPattern scrolling = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), spacing); 
        
        sendEffect(scrolling);

    }

    //Apaga leds
    public void off(){
        sendEffect(LEDPattern.kOff);
    }

}