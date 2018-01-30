package org.noname;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;

public class Demo extends  DemoDevice
{
    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        LoopBackDemoChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
        {
            @MAV_TYPE int  type = pack.type_GET();
            @MAV_AUTOPILOT int  autopilot = pack.autopilot_GET();
            char  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
            @MAV_STATE int  system_status = pack.system_status_GET();
            char  mavlink_version = pack.mavlink_version_GET();
        });
        HEARTBEAT p0 = LoopBackDemoChannel.instance.new_HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_OCTOROTOR);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
        p0.base_mode_SET((char)240);
        p0.custom_mode_SET(1344512591L);
        p0.system_status_SET(MAV_STATE.MAV_STATE_EMERGENCY);
        p0.mavlink_version_SET((char)191);
        LoopBackDemoChannel.instance.send(p0); //===============================
    }
}
