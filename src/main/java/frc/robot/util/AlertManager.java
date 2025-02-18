package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Utility for managing dynamic alerts
 */
public class AlertManager {

    private static final Map<String, Alert> ALERTS_BY_KEY = new HashMap<>();

    /**
     * Add an active alert
     * 
     * @param key Unique key identifying the alert - note the same key must be used to remove the alert
     * @param text Alert text / message to display
     * @param type Alert type (kInfo, kWarning, kError)
     */
    public static void addAlert(String key, String text, AlertType type) {
        addAlert(key, null, text, type);
    }

    /**
     * Add an active alert
     * 
     * @param key Unique key identifying the alert - note the same key must be used to remove the alert
     * @param group Group identifier, used as the entry name in NetworkTables
     * @param text Alert text / message to display
     * @param type Alert type (kInfo, kWarning, kError)
     */
    public static void addAlert(String key, String group, String text, AlertType type) {
        Alert alert = ALERTS_BY_KEY.get(key);
        if (alert == null) {
            if (group == null) {
                alert = new Alert(text, type);
            } else {
                alert = new Alert(group, text, type);
            }
            
            ALERTS_BY_KEY.put(key, alert);
        } else {
            alert.setText(text);
        }
        
        alert.set(true);
    }

    /**
     * Remove an alert
     * 
     * @param key Unique key identifying the alert to remove
     */
    public static void removeAlert(String key) {
        Alert alert = ALERTS_BY_KEY.get(key);
        if (alert != null) {
            alert.set(false);
            ALERTS_BY_KEY.remove(key);
        }
    }
}
