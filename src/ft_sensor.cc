//
// Created by think on 2023/12/15.
//

#include <rocos_app/ethercat/ft_sensor.h>
#include <tinyxml2.h>

#include <iostream>

namespace rocos {

    FtSensor::FtSensor(const std::string &urdf_file_path) {
        ecPtr_ = EcatConfig::getInstance();

        parseParamFormUrdf(urdf_file_path);
    }

    FtSensor::~FtSensor() = default;

    float FtSensor::getFx() const {
        return pFx_ ? *pFx_ : 0.0f;
    }

    float FtSensor::getFy() const {
        return pFy_ ? *pFy_ : 0.0f;
    }

    float FtSensor::getFz() const {
        return pFz_ ? *pFz_ : 0.0f;
    }

    float FtSensor::getMx() const {
        return pMx_ ? *pMx_ : 0.0f;
    }

    float FtSensor::getMy() const {
        return pMy_ ? *pMy_ : 0.0f;
    }

    float FtSensor::getMz() const {
        return pMz_ ? *pMz_ : 0.0f;
    }

    void FtSensor::parseParamFormUrdf(const std::string &urdf_file_path) {
        tinyxml2::XMLDocument doc;
        doc.LoadFile(urdf_file_path.c_str());

        auto robot = doc.FirstChildElement("robot");
        for (auto jnt = robot->FirstChildElement("joint"); jnt; jnt = jnt->NextSiblingElement("joint")) {

            auto hw = jnt->FirstChildElement("hardware");

            auto id = hw->IntAttribute("id", -1); // 对应的硬件ID，若没指定默认为-1

            if(!hw->Attribute("type", "ft-sensor")) { // 如果type为ft-sensor
                continue;
            }

            std::cout << "joint < " << jnt->Attribute("name") << " >'s assigned slave <id: " << id << " > is ft-sensor." << std::endl;

            auto inputs = hw->FirstChildElement("inputs");
            if(inputs) {
                auto fx = inputs->FirstChildElement("fx");
                if(fx) {
                    pFx_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, fx->GetText());
                } else {
                    pFx_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Fx");
                }

                auto fy = inputs->FirstChildElement("fy");
                if(fy) {
                    pFy_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, fy->GetText());
                } else {
                    pFy_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Fy");
                }

                auto fz = inputs->FirstChildElement("fz");
                if(fz) {
                    pFz_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, fz->GetText());
                } else {
                    pFz_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Fz");
                }

                auto mx = inputs->FirstChildElement("mx");
                if(mx) {
                    pMx_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, mx->GetText());
                } else {
                    pMx_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Mx");
                }

                auto my = inputs->FirstChildElement("my");
                if(my) {
                    pMy_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, my->GetText());
                } else {
                    pMy_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "My");
                }

                auto mz = inputs->FirstChildElement("mz");
                if(mz) {
                    pMz_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, mz->GetText());
                } else {
                    pMz_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Mz");
                }

            } else {
                pFx_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Fx");
                pFy_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Fy");
                pFz_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Fz");
                pMx_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Mx");
                pMy_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "My");
                pMz_ = ecPtr_->findSlaveInputVarPtrByName<float>(id, "Mz");
            }



        }
    }


} // rocos