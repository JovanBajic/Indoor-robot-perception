"""Archived Isaac recovery script that deactivates the old TF graph and optical helper.

It is not intended for the current working camera setup.
"""

import omni.timeline
import omni.usd


def recover():
    """Deactivate only the two additions; report the known IMU workaround."""
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError('Stop playback first.')
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError('Open your working scene first.')
    mount = (
        '/World/rbkairos_franka_isaac/Geometry/summit_xl_base_footprint/'
        'summit_xl_base_link/rsd455/RSD455'
    )
    for path in (
        '/Graph/D455_OpticalTF',
        mount + '/Camera_OmniVision_OV9782_Color/d455_color_optical_frame',
    ):
        prim = stage.GetPrimAtPath(path)
        if prim:
            prim.SetActive(False)
            print('Deactivated: ' + path)
        else:
            print('Absent: ' + path)
    imu = stage.GetPrimAtPath(mount + '/Imu_Sensor')
    if imu:
        print('Imu_Sensor active: ' + str(imu.IsActive()))
        if imu.IsActive():
            print('Keep playback stopped: deactivate Imu_Sensor before testing.')
    else:
        print('Imu_Sensor not found at the expected path; inspect the D455 hierarchy.')
    print('Scene not saved automatically. No drive, LiDAR or camera transforms changed.')


recover()
