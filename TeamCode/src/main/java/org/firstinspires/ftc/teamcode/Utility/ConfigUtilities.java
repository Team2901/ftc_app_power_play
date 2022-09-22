package org.firstinspires.ftc.teamcode.Utility;

import android.app.Application;
import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.json.JSONException;
import org.json.JSONObject;

import java.lang.reflect.Method;

public class ConfigUtilities {

        private static Context getContext() {
            try {
                final Class<?> activityThreadClass = Class.forName("android.app.ActivityThread");
                final Method method = activityThreadClass.getMethod("currentApplication");
                return (Application) method.invoke(null, (Object[]) null);
            } catch (final java.lang.Throwable e) {
                throw new IllegalArgumentException("No context could be retrieved!");
            }
        }
        public static String getRobotConfigurationName() {
            Context context = getContext();
            SharedPreferences preferences= PreferenceManager.getDefaultSharedPreferences(context);
            String objSerialized = preferences.getString("pref_hardware_config_filename", null);
            String configName = null;
            try {
                JSONObject jObject = new JSONObject(objSerialized);
                configName = jObject.getString("name");
            } catch (JSONException e) {
                e.printStackTrace();
            }
            return configName;
        }
}
