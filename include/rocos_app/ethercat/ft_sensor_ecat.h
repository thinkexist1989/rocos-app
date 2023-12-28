//
// Created by think on 2023/12/15.
//

#ifndef ROCOS_APP_FT_SENSOR_ECAT_H
#define ROCOS_APP_FT_SENSOR_ECAT_H

#include <rocos_app/ft_sensor_interface.h>
#include <rocos_ecm/ecat_config.h>


namespace rocos {

    class FtSensorEcat : FtSensorInterface {
    public:
        explicit FtSensorEcat(const std::string &urdf_file_path = "robot.urdf");

        ~FtSensorEcat() override;

        float getFx() const override;

        float getFy() const override;

        float getFz() const override;

        float getMx() const override;

        float getMy() const override;

        float getMz() const override;

    private:

        void parseParamFormUrdf(const std::string &urdf_file_path);

        EcatConfig* ecPtr_ {nullptr};

        float* pFx_ {nullptr};
        float* pFy_ {nullptr};
        float* pFz_ {nullptr};
        float* pMx_ {nullptr};
        float* pMy_ {nullptr};
        float* pMz_ {nullptr};

    };

} // rocos

#endif //ROCOS_APP_FT_SENSOR_ECAT_H
