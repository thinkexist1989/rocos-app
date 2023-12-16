//
// Created by think on 2023/12/15.
//

#ifndef ROCOS_APP_FT_SENSOR_H
#define ROCOS_APP_FT_SENSOR_H

#include <rocos_app/ft_sensor_interface.h>

namespace rocos {

    class FtSensor : FtSensorInterface {
        FtSensor();

        ~FtSensor() override;

        float getFx() const override;

        float getFy() const override;

        float getFz() const override;

        float getMx() const override;

        float getMy() const override;

        float getMz() const override;

    };

} // rocos

#endif //ROCOS_APP_FT_SENSOR_H
