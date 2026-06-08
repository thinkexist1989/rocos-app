//
// Created by think on 2023/12/11.
//

#ifndef ECAT_CONFIG_H_INCLUDED
#define ECAT_CONFIG_H_INCLUDED

#include "ecat_type.h"
#include <thread>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/format.hpp>
#include <map>

namespace rocos {
    class EcatConfig {
    private:
        EcatConfig(int id = 0);
        ~EcatConfig();

    public:
        static EcatConfig* getInstance(int id = 0);

        void wait();

        double getBusMinCycleTime() const;

        double getBusMaxCycleTime() const;

        double getBusAvgCycleTime() const;

        double getBusCurrentCycleTime() const;

        bool isAuthorized() const;

        long getTimestamp() const;

        int getSlaveNum() const;

        void resetCycleTime();

        void setBusRequestState(int state);
        int  getBusCurrentState() const;
        void waitForSignal(int id = 0); // compact code, not recommended use. use wait() instead


        std::string getSlaveName(int slaveId);

        Slave getSlave(int slaveId);

        Slave findSlaveByName(const std::string &slaveName);

        int findSlaveIdByName(const std::string &slaveName);

        std::string getInputVarName(int slaveId, int varId) const;

        std::string getOutputVarName(int slaveId, int varId) const;

        PdVar getSlaveInputVar(int slaveId, int varId);

        PdVar getSlaveOutputVar(int slaveId, int varId);

        PdVar findSlaveInputVarByName(int slaveId, const std::string &varName);

        int findSlaveInputVarIdByName(int slaveId, const std::string &varName);

        template<typename T>
        T getSlaveInputVarValue(int slaveId, int varId) {
            if (sizeof(T) != ecatBus->slaves[slaveId].input_vars[varId].size) {
                print_message("Size of Var is not equal", MessageLevel::WARNING);
            }
            return *(T *) ((char *) pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset);
        }

        template<typename T>
        void setSlaveInputVarValue(int slaveId, int varId, T value) {
            if (sizeof(T) != ecatBus->slaves[slaveId].input_vars[varId].size) {
                print_message("Size of Var is not equal", MessageLevel::WARNING);
            }
            *(T *) ((char *) pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset) = value;
        }

        template<typename T>
        T getSlaveOutputVarValue(int slaveId, int varId) {
            if (sizeof(T) != ecatBus->slaves[slaveId].output_vars[varId].size) {
                print_message("Size of Var is not equal", MessageLevel::WARNING);
            }
            return *(T *) ((char *) pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset);
        }

        template<typename T>
        void setSlaveOutputVarValue(int slaveId, int varId, T value) {
            if (sizeof(T) != ecatBus->slaves[slaveId].output_vars[varId].size) {
                print_message("Size of Var is not equal", MessageLevel::WARNING);
            }
            *(T *) ((char *) pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset) = value;
        }

        template<typename T>
        T getSlaveInputVarValueByName(int slaveId, const std::string &varName) {
            for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i) {
                if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, varName.c_str()) == 0) {
                    if (sizeof(T) != ecatBus->slaves[slaveId].input_vars[i].size) {
                        print_message("Size of Var is not equal", MessageLevel::WARNING);
                    }
                    return *(T *) ((char *) pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset);
                }
            }
            return std::numeric_limits<T>::max();
        }

        template<typename T>
        void setSlaveInputVarValueByName(int slaveId, const std::string &varName, T value) {
            for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i) {
                if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, varName.c_str()) == 0) {
                    if (sizeof(T) != ecatBus->slaves[slaveId].input_vars[i].size) {
                        print_message("Size of Var is not equal", MessageLevel::WARNING);
                    }
                    *(T *) ((char *) pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset) = value;
                }
            }
        }

        template<typename T>
        T getSlaveOutputVarValueByName(int slaveId, const std::string &varName) {
            for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i) {
                if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, varName.c_str()) == 0) {
                    if (sizeof(T) != ecatBus->slaves[slaveId].output_vars[i].size) {
                        print_message("Size of Var is not equal", MessageLevel::WARNING);
                    }
                    return *(T *) ((char *) pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset);
                }
            }
            return std::numeric_limits<T>::max();
        }

        template<typename T>
        void setSlaveOutputVarValueByName(int slaveId, const std::string &varName, T value) {
            for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i) {
                if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, varName.c_str()) == 0) {
                    if (sizeof(T) != ecatBus->slaves[slaveId].output_vars[i].size) {
                        print_message("Size of Var is not equal", MessageLevel::WARNING);
                    }
                    *(T *) ((char *) pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset) = value;
                }
            }
        }

        template<typename T>
        T* getSlaveInputVarPtr(int slaveId, int varId) {
            if (sizeof(T) != ecatBus->slaves[slaveId].input_vars[varId].size) {
                print_message("Size of Var is not equal", MessageLevel::WARNING);
            }
            return (T *) ((char *) pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset);
        }

        template<typename T>
        T* getSlaveOutputVarPtr(int slaveId, int varId) {
            if (sizeof(T) != ecatBus->slaves[slaveId].output_vars[varId].size) {
                print_message("Size of Var is not equal", MessageLevel::WARNING);
            }
            return (T *) ((char *) pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset);
        }

        template<typename T>
        T* findSlaveInputVarPtrByName(int slaveId, const std::string &varName) {
            for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i) {
                if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, varName.c_str()) == 0) {
                    if (sizeof(T) != ecatBus->slaves[slaveId].input_vars[i].size) {
                        print_message("Size of Var is not equal", MessageLevel::WARNING);
                    }
                    return (T *) ((char *) pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset);
                }
            }
            return nullptr;
        }

        template<typename T>
        T* findSlaveOutputVarPtrByName(int slaveId, const std::string &varName) {
            for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i) {
                if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, varName.c_str()) == 0) {
                    if (sizeof(T) != ecatBus->slaves[slaveId].output_vars[i].size) {
                        print_message("Size of Var is not equal", MessageLevel::WARNING);
                    }
                    return (T *) ((char *) pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset);
                }
            }
            return nullptr;
        }


    private:
        static std::map<int, EcatConfig*> instances;

        void init();

        bool getSharedMemory();

        bool getPdDataMemoryProvider();


        std::vector<std::thread::id> threadId;

        std::string ecmName {EC_SHM};
        std::string mutexName {EC_SEM_MUTEX};
        std::string pdInputName {"pd_input"};
        std::string pdOutputName {"pd_output"};

        boost::interprocess::managed_shared_memory *managedSharedMemory = nullptr;

        // PD Input and Output memory
        boost::interprocess::shared_memory_object *pdInputShm = nullptr;
        boost::interprocess::shared_memory_object *pdOutputShm = nullptr;
        boost::interprocess::mapped_region *pdInputRegion = nullptr;
        boost::interprocess::mapped_region *pdOutputRegion = nullptr;

        void *pdInputPtr = nullptr;
        void *pdOutputPtr = nullptr;

        EcatBus *ecatBus = nullptr;


        sem_t *sem_mutex[EC_SEM_NUM];

        //////////// OUTPUT FORMAT SETTINGS ////////////////////
        //Terminal Color Show
        enum Color {
            BLACK = 0,
            RED = 1,
            GREEN = 2,
            YELLOW = 3,
            BLUE = 4,
            MAGENTA = 5,
            CYAN = 6,
            WHITE = 7,
            DEFAULT = 9
        };

        enum MessageLevel {
            NORMAL = 0,
            WARNING = 1,
            ERROR = 2
        };


        boost::format _f{"\033[1;3%1%m "};       //设置前景色
        boost::format _b{"\033[1;4%1%m "};       //设置背景色
        boost::format _fb{"\033[1;3%1%;4%2%m "}; //前景背景都设置
        boost::format _def{"\033[0m "};          //恢复默认

        void print_message(const std::string &msg, MessageLevel msgLvl = MessageLevel::NORMAL);

    };
}


#endif //ROCOS_ECM_ECAT_CONFIG_H
