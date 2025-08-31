package frc.robot.subsystems.body;

public enum BodySetpoints {
    CORAL_LEVEL1(0.0, 0.0),
    CORAL_LEVEL2(0.0,0.0),
    CORAL_LEVEL3(0.0, 0.0),
    CORAL_LEVEL4(0.0,0.0),
    ALGAE_LEVEL3(0.0,0.0),
    ALGAE_LEVEL2(0.0,0.0),
    STOW_INTAKE(0.0,0.0),
    HIGH_NET(0.0,0.0),
    PROCESSOR(0.0,0.0);

    private final double elevTravel;
    private final double armDegrees;

    public double getArmDegrees(){
        return armDegrees;
    }
    public double getElevTravel(){
        return elevTravel;
    }


    private BodySetpoints(double elevTravel, double armDegrees){
        this.elevTravel = elevTravel;


        this.armDegrees = armDegrees;

    }
    




}


