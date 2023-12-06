// Note : I will add Comments later
// I know it looks like a complete mess


package frc.robot.test;

import java.io.*;
import java.lang.reflect.InvocationTargetException;
import java.util.*;
import org.json.simple.*;
import org.json.simple.parser.*;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TestChecklist {

  private static TestChecklist instance;

  public RobotContainer m_container;
  private Subsystem[] m_subsystems;
  private Class<? extends Subsystem> GetSubsystem(String name){
    Subsystem result = null;
    for (Subsystem subsystem : m_subsystems) {
      if (subsystem.getClass().getName() == name)
        return subsystem.getClass();
    }
    return null;
  }

  private final String m_testDirectory = Robot.RESOURCES_PATH.resolve("tests").toString();

  private String m_currentTest;
  private boolean m_runTests = false;

  public TestChecklist(RobotContainer container, Subsystem... subsystems) {
    instance = this;
    m_subsystems = subsystems;
  }
  public static TestChecklist GetInstance() { return instance; }

  public String GetValueFromJSON(String test, String value) {
    try {

      JSONParser parser = new JSONParser();
      Object obj = parser.parse(m_testDirectory + "\\" + test + ".json");

      JSONObject jsonObject = (JSONObject)obj;
      return (String)jsonObject.get(value);

    } 
    catch(Exception e) {
      System.out.println(e.getMessage());
      return null;
    }
  }
  public void SetValueOnJSON(String test, String key, String value){
    try {

      JSONParser parser = new JSONParser();
      Object obj = parser.parse(m_testDirectory + "\\" + test + ".json");

      FileWriter writer = new FileWriter(m_testDirectory + "\\" + test + ".json");

      JSONObject jsonObject = (JSONObject)obj;
      jsonObject.put(key, value);

      writer.write(jsonObject.toJSONString());
      writer.close();

    } 
    catch(Exception e) { System.out.println(e.getMessage()); }
  }
  
  public void RunTest(String test) {

    m_currentTest = test;
    String subsystemName = GetValueFromJSON(test, "Subsystem");
    Class<? extends Subsystem> subsystem = GetSubsystem(subsystemName);

    try { 
      subsystem
        .getMethod(test, (Class<?>[])null)
        .invoke((Object)subsystem, (Object[])null); 
    } 
    catch (NoSuchMethodException e)     { System.out.println(e.getMessage()); } 
    catch (IllegalAccessException e)    { System.out.println(e.getMessage()); } 
    catch (InvocationTargetException e) { System.out.println(e.getMessage()); } 
    
  }
  public String[] GetTests() { 
    List<String> result = null;
    File[] listOfTests = new File(Robot.RESOURCES_PATH.resolve("tests").toString()).listFiles();
    for (File file : listOfTests)
      result.add(file.getName().replaceFirst("[.][^.]+$", ""));
    return (String[])result.toArray();
  }
  public String GetCurrentTest(){
    return m_runTests ? m_currentTest : "Disabled";
  }
  public void PassTest(String test, Boolean value) {
    SetValueOnJSON(test, "Passed", value.toString());
  }

  public void initialize() {}
  public void execute()    {}
  public void exit()       {}
  
}
