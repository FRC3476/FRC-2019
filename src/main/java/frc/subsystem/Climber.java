package frc.subsystem;

public class Climber{
    private static Climber instance = new Climber();

    //Get instance of the Climber
    public static Climber getInstance(){
        return instance;
    }

}
