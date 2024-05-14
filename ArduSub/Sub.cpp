/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Sub.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_SCRIPTING_ENABLED
// set target position and velocity (for use by scripting)
bool Sub::set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel)
{
    if (control_mode != Mode::Number::GUIDED) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);

    return mode_guided.guided_set_destination_posvel(pos_neu_cm, vel_neu_cms);
}
#endif // AP_SCRIPTING_ENABLED

/*
  constructor for main Sub class
 */
Sub::Sub()
    :
          control_mode(Mode::Number::MANUAL),
          motors(MAIN_LOOP_RATE),
          auto_mode(Auto_WP),
          guided_mode(Guided_WP),
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
          inertial_nav(ahrs),
          ahrs_view(ahrs, ROTATION_NONE),
          attitude_control(ahrs_view, aparm, motors),
          pos_control(ahrs_view, inertial_nav, motors, attitude_control),
          wp_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          loiter_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          circle_nav(inertial_nav, ahrs_view, pos_control),
          param_loader(var_info),
          flightmode(&mode_manual)
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    failsafe.pilot_input = true;
#endif
    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one Sub");
    }
    _singleton = this;
}

Sub *Sub::_singleton = nullptr;

Sub sub;
AP_Vehicle& vehicle = sub;
