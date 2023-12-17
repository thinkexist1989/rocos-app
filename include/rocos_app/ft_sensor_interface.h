//
// Created by think on 2023/12/15.
//

#ifndef ROCOS_APP_FT_SENSOR_INTERFACE_H
#define ROCOS_APP_FT_SENSOR_INTERFACE_H

#include <string>

namespace rocos {

    class FtSensorInterface {

    public:
        virtual ~FtSensorInterface();

        virtual float getFx() const;

        virtual float getFy() const;

        virtual float getFz() const;

        virtual float getMx() const;

        virtual float getMy() const;

        virtual float getMz() const;


    };

} // rocos

#endif //ROCOS_APP_FT_SENSOR_INTERFACE_H
