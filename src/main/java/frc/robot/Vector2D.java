package frc.robot;

public class Vector2D {
  private final double m_ahead; // positive is straight ahead
  private final double m_toRight; // positive is to right
  public enum ConstructorType {
    kAheadToRight,       // distance ahead and distance to right
    kRadiansCcwDistance, // radians counterclockwise from straight ahead (and distance)
    kDegreesCwDistance   // degrees clockwise from straight ahead (and distance)
  }
  public Vector2D(double arg1, double arg2, ConstructorType constructorType) {
    switch(constructorType) {
      case kAheadToRight:
        m_ahead = arg1;
        m_toRight = arg2;
        break;
      case kRadiansCcwDistance:
        m_ahead = arg2 * Math.cos(arg1);
        m_toRight = -arg2 * Math.sin(arg1);
        break;
      case kDegreesCwDistance:
        double radians = - arg1 / 180.0 * Math.PI;
        m_ahead = arg2 * Math.cos(radians);
        m_toRight = -arg2 * Math.sin(radians);
        break;
      default: // just to shut up javac: it complains m_ahead and m_toRight may not have been initialized
        m_ahead = 0;
        m_toRight = 0;
        break;
    }
  }
  public double getAhead() {
    return m_ahead;
  }
  public double getToRight() {
    return m_toRight;
  }
  public double getRadiansCcw() {
    // Java's atan2 returns number between -pi and pi
    return (m_ahead == 0.0 && m_toRight == 0.0)
             ? 0.0
             : -Math.atan2(m_toRight, m_ahead); // do we have to worry about 0,0 case?
  }
  public double getDegreesCw() {
    return -getRadiansCcw() / Math.PI * 180.0;
  }
  public double getDistance() {
    return Math.hypot(m_ahead, m_toRight);
  }
  public Vector2D vectorTo(Vector2D destination) {
    return new Vector2D(destination.getAhead() - getAhead(), destination.getToRight() - getToRight(), ConstructorType.kAheadToRight);
  }
  @Override
  public String toString() {
    return "" + getAhead() + " ahead and " + getToRight() + " to the right";
  }
}
