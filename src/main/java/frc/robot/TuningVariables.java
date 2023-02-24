// Author: Bill Dunlap <williamwdunlap@gmail.com>, FRC 4173 (Team IMVERT), January 2023.
// This software is free to use by anyone with no restrictions.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/** These things are like the variables in Constants,
 * but can be set via the Preferences class (e.g.,
 * via Shuffleboard) and so will be stored in [persistent]
 * flash memory on the roborio.  The TuningVariables class
 * also stores default values in Java (nonpersistent)
 * memory.
 * 
 * To add a new variable, say newVar, with default value newVarDefault,
 * to this scheme, add newVar(newVarDefault) to the comma-separated
 * list at the start of the enum below.
 * 
 * To get a tuning variable's value use the syntax
 *   TuningVariables.variableName.get()
 * 
 * A current limitation is that the turning variables
 * must be doubles.
 */
public enum TuningVariables {
    // To add a new value, just enter its name and default value to the following list
    defaultSpinRate_DegreesPerSecond (40.0),
    defaultTravelRate_FeetPerSecond (2.0),
    debugLevel(1.0); // scale of 0 to 10

    private double m_defaultValue;
    /** Users cannot call an enum constructor directly;
     * Java will call it for each variable listed above.
     * Note the constructor will not change pre-existing
     * values in flash memory.  Use setToDefaultValue or
     * setAllToDefaultValues for that.
     */
    private TuningVariables(double defaultValue){
        m_defaultValue = defaultValue;
        if (!Preferences.containsKey(name())) {
          Preferences.setDouble(name(), m_defaultValue);
        }
    }
    /** From flash memory, get the value of this variable */
    public double get() {
        return Preferences.getDouble(name(), m_defaultValue);
    }
    /** In flash memory, set this variable to a value */
    void set(double value) {
        Preferences.setDouble(name(), value);
    }
    /** In flash memory, set this variable to its default value */
    void setToDefaultValue() {
        set(m_defaultValue);
    }
    /** In flash memory, set all variables to their default values */
    static void setAllToDefaultValues() {
        for(TuningVariables tv: TuningVariables.values()) {
            tv.setToDefaultValue();
        }
    }
    /** Remove this variable from flash memory */
    void remove(){
        Preferences.remove(name());
    }
    /** Remove all variables from flash memory */
    static void removeAllKnown() {
        for(TuningVariables tv : TuningVariables.values()) {
            tv.remove();
        }
    }
}
