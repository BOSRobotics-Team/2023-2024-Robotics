package frc.robot.testsystem;

public abstract interface TestInterface {

    public default Integer Test(String test){
        System.out.println("Not Implemented");
        return 1;
    }

}
