/*
Copyright 2021, Yang Luo"
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

@Author
Yang Luo, PHD
Shenyang Institute of Automation, Chinese Academy of Sciences.
 email: luoyang@sia.cn

@Created on: 2021.11.29
*/


/*-----------------------------------------------------------------------------
 * ecat_config.hpp
 * Description              EtherCAT Configurations
 *
 *---------------------------------------------------------------------------*/

#ifndef ECAT_CONFIG_HPP_INCLUDED
#define ECAT_CONFIG_HPP_INCLUDED

#define ROCOS_APP_ENABLED
//#define ROCOS_ECM_ENABLED

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sstream>

#include <boost/format.hpp>
#include <boost/timer/timer.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/chrono.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>

#include <string>
#include <cstring>
#include <cstdlib>
#include <unordered_map>

#include <sys/mman.h> //shm_open() mmap()
#include <unistd.h>   // ftruncate()
#include <fcntl.h>
#include <semaphore.h> //sem
#include <sys/stat.h>  //umask

#include <math.h>

#define ROBOT_CONFIG_FILE "ecat_config.yaml"

/*-SHARED MEMORY ------------------------------------------------------------*/
#define EC_SHM "ecm"
#define EC_SHM_MAX_SIZE 65536

//#define EC_SEM_SYNC "sync"
#define EC_SEM_MUTEX "sync"

#define MAX_NAME_LEN 20
#define MAX_JOINT_NUM 10

#define MAX_SLAVE_NUM 20


enum INPUTS //
{
    STATUS_WORD,
    POSITION_ACTUAL_VALUE,
    VELOCITY_ACTUAL_VALUE,
    TORQUE_ACTUAL_VALUE,
    LOAD_TORQUE_VALUE
};
enum OUTPUTS {
    MODE_OF_OPERATION,
    CONTROL_WORD,
    TARGET_POSITION,
    TARGET_VELOCITY,
    TARGET_TORQUE
};

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


typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> CharAlloc;
typedef boost::interprocess::basic_string<char, std::char_traits<char>, CharAlloc> EcString;
typedef boost::interprocess::allocator<EcString, boost::interprocess::managed_shared_memory::segment_manager> StringAlloc;
typedef boost::interprocess::vector<EcString, StringAlloc> EcStringVec;

struct  EcatSlaveInfo;
typedef boost::interprocess::allocator<EcatSlaveInfo,boost::interprocess::managed_shared_memory::segment_manager> EcSlaveAlloc;
typedef boost::interprocess::vector<EcatSlaveInfo,EcSlaveAlloc>  EcSlaveVec;

struct EcatSlaveInfo {

    uint32_t slave_id {0};

    /** Struct store the EtherCAT process data input  **/
    struct PDInput {
        uint16_t status_word{0};          //  Size: 2.0 unsigned
        int32_t position_actual_value{0}; // Size: 4.0 signed
        int32_t velocity_actual_value{0}; // Size: 4.0 signed
        int16_t torque_actual_value{0};   // Size: 2.0 signed
        int16_t load_torque_value{0};     // Size: 2.0 signed
    };

    PDInput inputs;

    /** Struct store the EtherCAT process data output  **/
    struct PDOutput {
        int8_t mode_of_operation{8}; // Size: 1.0 signed
        uint16_t control_word{0};    // Size: 2.0 unsigned
        int32_t target_position{0};  // Size: 4.0 signed
        int32_t target_velocity{0};  // Size: 4.0 signed
        int16_t target_torque{0};    // Size: 2.0 signed
    };

    PDOutput outputs;
};

/////////////////////   STRUCT DEFINITION   /////////////////////////////////
struct EcatInfo {

    boost::chrono::time_point<boost::chrono::system_clock> timestamp;  // Timestamp

    enum EcatState {
        UNKNOWN = 0,
        INIT = 1,
        PREOP = 2,
        SAFEOP = 4,
        OP = 8,

        BOOTSTRAP = 3
    };

    double minCyclcTime {0.0}; // minimum cycling time   /* usec */
    double maxCycleTime {0.0}; // maximum cycling time  /* usec */
    double avgCycleTime {0.0}; // average cycling time  /* usec */
    double currCycleTime {0.0}; // current cycling time /* usec */

    EcatState ecatState {UNKNOWN};    // State of Ec-Master

    int32_t slave_number {0};

//    EcVec slaves; // all the slaves data
//    std::vector<EcatSlaveInfo> slaves; // all the slaves data
//     EcatSlaveInfo slaves[MAX_SLAVE_NUM];

};



/** Class SlaveConfig contains all configurations of one joint
 * 
 */
class SlaveConfig {
public:
    SlaveConfig() {
        // EtherCAT Process Data Input default Name Mapping
        ecInpMap[STATUS_WORD] = "Status word";
        ecInpMap[POSITION_ACTUAL_VALUE] = "Position actual value";
        ecInpMap[VELOCITY_ACTUAL_VALUE] = "Velocity actual value";
        ecInpMap[TORQUE_ACTUAL_VALUE] = "Torque actual value";
        ecInpMap[LOAD_TORQUE_VALUE] = "Analog Input 1";

        ecInpOffsets[STATUS_WORD] = 0;
        ecInpOffsets[POSITION_ACTUAL_VALUE] = 0;
        ecInpOffsets[VELOCITY_ACTUAL_VALUE] = 0;
        ecInpOffsets[TORQUE_ACTUAL_VALUE] = 0;
        ecInpOffsets[LOAD_TORQUE_VALUE] = 0;

        // EtherCAT Process Data Output default Name Mapping
        ecOutpMap[MODE_OF_OPERATION] = "Mode of operation";
        ecOutpMap[CONTROL_WORD] = "Control word";
        ecOutpMap[TARGET_POSITION] = "Target Position";
        ecOutpMap[TARGET_VELOCITY] = "Target Velocity";
        ecOutpMap[TARGET_TORQUE] = "Target Torque";

        ecOutpOffsets[MODE_OF_OPERATION] = 0;
        ecOutpOffsets[CONTROL_WORD] = 0;
        ecOutpOffsets[TARGET_POSITION] = 0;
        ecOutpOffsets[TARGET_VELOCITY] = 0;
        ecOutpOffsets[TARGET_TORQUE] = 0;

    }

    ~SlaveConfig() {

    }

    int id{0};

//    std::string jntName;

    std::string name{"Slave_1001 [Elmo Drive ]"};
    std::map<INPUTS, std::string> ecInpMap;
    std::map<OUTPUTS, std::string> ecOutpMap;

    std::map<INPUTS, int> ecInpOffsets;
    std::map<OUTPUTS, int> ecOutpOffsets;

//    T_JOINT_EC_INPUT *jntEcInpPtr = nullptr;
//    T_JOINT_EC_OUTPUT *jntEcOutpPtr = nullptr;
};


/** Class RobotConfig contains all configurations of the robot
 * 
 */
class EcatConfig {
public:
    explicit EcatConfig(std::string configFile = ROBOT_CONFIG_FILE) : configFileName(configFile) {

    }

    ~EcatConfig() {

    }

    std::string configFileName{};

    std::string name{"default_robot"};

    uint32_t loop_hz{1000};

    int slave_number {0};

    std::vector<SlaveConfig> slaveCfg;


    int ec_in_fd;
    int ec_out_fd;

    sem_t *sem_mutex;
    sem_t *sem_sync;

    EcatInfo* ecatInfo = nullptr;
    EcSlaveVec* ecatSlaveVec = nullptr;
    EcStringVec* ecatSlaveNameVec = nullptr;
    boost::interprocess::managed_shared_memory* managedSharedMemory;

public:
    bool parserYamlFile(const std::string &configFile) {
        if (!boost::filesystem::exists(configFile)) {
            print_message("[YAML] Can not find the config file.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node config = YAML::LoadFile(configFile);

        if (!config["robot"]) {
            print_message("[YAML] Can not find the robot label.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node robot = config["robot"];

        name = robot["name"].as<std::string>();
        print_message("[YAML] Robot name is: " + name, MessageLevel::NORMAL);

        loop_hz = robot["loop_hz"].as<uint32_t>();

        slave_number = robot["slave_number"].as<int>();
        if (slave_number == robot["slaves"].size()) {
            print_message((boost::format("[YAML] Robot has %d slaves") % slave_number).str(), MessageLevel::NORMAL);
        } else {
            print_message("[YAML] Robot has bad number of slaves.", MessageLevel::ERROR);
            return false;
        }

        YAML::Node slaves = robot["slaves"];
        slaveCfg.resize(slave_number);

        std::set<int> isJntOK;
        for (int i = 0; i < slave_number; i++) {

            /// slaves.id
            int id = slaves[i]["id"].as<int>();
            if (id >= 0) {
                print_message((boost::format("[YAML] -- Joint ID: %d. ") % id).str(), MessageLevel::NORMAL);
            } else {
                print_message("[YAML] Bad slaves ID!! ", MessageLevel::ERROR);
                return false;
            }
            auto res = isJntOK.insert(id);
            if (!res.second) { //Found duplicate elements
                print_message("[YAML] Bad slaves ID is DUPLICATE!! ", MessageLevel::ERROR);
                return false;
            }

//            //name
//            slaveCfg[id].jntName = slaves[i]["name"].as<std::string>();
//            print_message((boost::format("[YAML] -- Joint name: %s. ") % slaveCfg[id].jntName).str(),
//                          MessageLevel::NORMAL);

            /// slaves[].name
            slaveCfg[id].name = slaves[i]["name"].as<std::string>();
            print_message((boost::format("[YAML] -- Slave name: %s .") % slaveCfg[id].name).str(),
                          MessageLevel::NORMAL);

            /// Process Data Input Mapping
            if (slaves[i]["inputs"]["status_word"])
                slaveCfg[id].ecInpMap[STATUS_WORD] = slaves[i]["inputs"]["status_word"].as<std::string>();
            if (slaves[i]["inputs"]["position_actual_value"])
                slaveCfg[id].ecInpMap[POSITION_ACTUAL_VALUE] = slaves[i]["inputs"]["position_actual_value"].as<std::string>();
            if (slaves[i]["inputs"]["velocity_actual_value"])
                slaveCfg[id].ecInpMap[VELOCITY_ACTUAL_VALUE] = slaves[i]["inputs"]["velocity_actual_value"].as<std::string>();
            if (slaves[i]["inputs"]["torque_actual_value"])
                slaveCfg[id].ecInpMap[TORQUE_ACTUAL_VALUE] = slaves[i]["inputs"]["torque_actual_value"].as<std::string>();
            if (slaves[i]["inputs"]["load_torque_value"])
                slaveCfg[id].ecInpMap[LOAD_TORQUE_VALUE] = slaves[i]["inputs"]["load_torque_value"].as<std::string>();

            /// Process Data Out Mapping
            if (slaves[i]["outputs"]["mode_of_operation"])
                slaveCfg[id].ecOutpMap[MODE_OF_OPERATION] = slaves[i]["outputs"]["mode_of_operation"].as<std::string>();
            if (slaves[i]["outputs"]["control_word"])
                slaveCfg[id].ecOutpMap[CONTROL_WORD] = slaves[i]["outputs"]["control_word"].as<std::string>();
            if (slaves[i]["outputs"]["target_position"])
                slaveCfg[id].ecOutpMap[TARGET_POSITION] = slaves[i]["outputs"]["target_position"].as<std::string>();
            if (slaves[i]["outputs"]["target_velocity"])
                slaveCfg[id].ecOutpMap[TARGET_VELOCITY] = slaves[i]["outputs"]["target_velocity"].as<std::string>();
            if (slaves[i]["outputs"]["target_torque"])
                slaveCfg[id].ecOutpMap[TARGET_TORQUE] = slaves[i]["outputs"]["target_torque"].as<std::string>();

        }

        if (isJntOK.size() != slave_number) {
            print_message("[YAML] Bad joint IDs. Please check the ID of each joint. ", MessageLevel::ERROR);
            return false;
        }

        return true;
    }

    inline bool parserYamlFile() { return parserYamlFile(configFileName); }

#ifdef ROCOS_ECM_ENABLED
    std::string getEcInpVarName(int jntId, INPUTS enumEcInp) {
        return slaveCfg[jntId].name + ".Inputs." + slaveCfg[jntId].ecInpMap[enumEcInp];
    }

    std::string getEcOutpVarName(int jntId, OUTPUTS enumEcOutp) {
        return slaveCfg[jntId].name + ".Outputs." + slaveCfg[jntId].ecOutpMap[enumEcOutp];
    }

    bool createSharedMemory() {
        mode_t mask = umask(0); // 取消屏蔽的权限位

        sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT | O_RDWR, 0777, 1);
        if (sem_mutex == SEM_FAILED) {
            print_message("[SHM] Can not open or create semaphore mutex.", MessageLevel::ERROR);
            return false;
        }

        int val = 0;
        sem_getvalue(sem_mutex, &val);
        std::cout << "value of sem_mutex is: " << val << std::endl;
        if (val != 1) {
            sem_destroy(sem_mutex);
            sem_unlink(EC_SEM_MUTEX);
            sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT | O_RDWR, 0777, 1);
        }

        sem_getvalue(sem_mutex, &val);
        if (val != 1) {
            print_message("[SHM] Can not set semaphore mutex to 1.", MessageLevel::ERROR);
            return false;
        }

        using namespace boost::interprocess;
        shared_memory_object::remove(EC_SHM);

        managedSharedMemory = new managed_shared_memory {open_or_create, EC_SHM, EC_SHM_MAX_SIZE};

        ecatInfo = managedSharedMemory->find_or_construct<EcatInfo>("ecat")();
        ecatInfo->slave_number = slave_number;

        EcSlaveAlloc alloc_inst(managedSharedMemory->get_segment_manager());
        ecatSlaveVec = managedSharedMemory->find_or_construct<EcSlaveVec>("slaves")(slave_number, alloc_inst);

        CharAlloc   char_alloc_inst(managedSharedMemory->get_segment_manager());
        StringAlloc string_alloc_inst(managedSharedMemory->get_segment_manager());
        ecatSlaveNameVec = managedSharedMemory->find_or_construct<EcStringVec>("slave_names")(slave_number, EcString(char_alloc_inst) ,string_alloc_inst);

//        ecatInfo->slaves.resize(1);
        print_message("OK!!!!.", MessageLevel::NORMAL);

        umask(mask); // 恢复umask的值

        return true;
    }

#endif

#ifdef ROCOS_APP_ENABLED
    /// \brief
    /// \return
    bool getSharedMemory() {
        mode_t mask = umask(0); // 取消屏蔽的权限位

        sem_mutex = sem_open(EC_SEM_MUTEX, O_CREAT, 0777, 1);
        if (sem_mutex == SEM_FAILED) {
            print_message("[SHM] Can not open or create semaphore mutex.", MessageLevel::ERROR);
            return false;
        }

        using namespace boost::interprocess;
        managedSharedMemory = new managed_shared_memory {open_or_create, EC_SHM, EC_SHM_MAX_SIZE};
        EcSlaveAlloc alloc_inst(managedSharedMemory->get_segment_manager());
        CharAlloc   char_alloc_inst(managedSharedMemory->get_segment_manager());
        StringAlloc string_alloc_inst(managedSharedMemory->get_segment_manager());

        std::pair<EcatInfo *, std::size_t> p1 =  managedSharedMemory->find<EcatInfo>("ecat");
        if(p1.first) {
            ecatInfo = p1.first;
        }
        else {
            print_message("[SHM] Ec-Master is not running.", MessageLevel::WARNING);
            ecatInfo = managedSharedMemory->construct<EcatInfo>("ecat")();
        }

        auto p2 =  managedSharedMemory->find<EcSlaveVec>("slaves");
        if(p2.first) {
            ecatSlaveVec = p2.first;
        }
        else {
            print_message("[SHM] Ec-Master is not running.", MessageLevel::WARNING);
            ecatSlaveVec = managedSharedMemory->construct<EcSlaveVec>("slaves")(alloc_inst);
        }

        auto p3 =  managedSharedMemory->find<EcStringVec>("slave_names");
        if(p3.first) {
            ecatSlaveNameVec = p3.first;
        }
        else {
            print_message("[SHM] Ec-Master is not running.", MessageLevel::WARNING);
            ecatSlaveNameVec = managedSharedMemory->construct<EcStringVec>("slave_names")(string_alloc_inst);
        }

        umask(mask); // 恢复umask的值

        return true;
    }

    ////////////// Get joints info for Ec Input /////////////////////

    inline int32_t getActualPositionEC(int id) const { return ecatSlaveVec->at(id).inputs.position_actual_value; }

    inline int32_t getActualVelocityEC(int id) const { return ecatSlaveVec->at(id).inputs.velocity_actual_value; }

    inline int16_t getActualTorqueEC(int id) const { return ecatSlaveVec->at(id).inputs.torque_actual_value; }

    inline int32_t getLoadTorqueEC(int id) const { return ecatSlaveVec->at(id).inputs.load_torque_value; }

    inline uint16_t getStatusWord(int id) const { return ecatSlaveVec->at(id).inputs.status_word; }

    ////////////// Get joints info for Ec Input /////////////////////

    inline void setTargetPositionEC(int id, int32_t pos) { ecatSlaveVec->at(id).outputs.target_position = pos; }

    inline void setTargetVelocityEC(int id, int32_t vel) { ecatSlaveVec->at(id).outputs.target_velocity = vel; }

    inline void setTargetTorqueEC(int id, int32_t tor) { ecatSlaveVec->at(id).outputs.target_torque = tor; }

    inline void setJointMode(int id, int32_t mode) { ecatSlaveVec->at(id).outputs.mode_of_operation = mode; }

    inline void
    setControlWord(int id, int32_t ctrlword) { ecatSlaveVec->at(id).outputs.control_word = ctrlword; }

    inline void waitForSignal() { sem_wait(sem_mutex); }

//    inline void unlock() { sem_post(sem_mutex); }

#endif

    ///////////// Format robot info /////////////////
    std::string to_string() {
        std::stringstream ss;
        for (int i = 0; i < slave_number; i++) {
            ss << "[Joint " << i << "] "
               << " stat: " << ecatSlaveVec->at(i).inputs.status_word << "; pos: " << ecatSlaveVec->at(i).inputs.position_actual_value
               << "; vel: " << ecatSlaveVec->at(i).inputs.velocity_actual_value << "; tor: " << ecatSlaveVec->at(i).inputs.torque_actual_value << ";";
        }
        return ss.str();
    }

private:
    //////////// OUTPUT FORMAT SETTINGS ////////////////////
    boost::format _f{"\033[1;3%1%m "};       //设置前景色
    boost::format _b{"\033[1;4%1%m "};       //设置背景色
    boost::format _fb{"\033[1;3%1%;4%2%m "}; //前景背景都设置
    boost::format _def{"\033[0m "};          //恢复默认

    void print_message(const std::string &msg, MessageLevel msgLvl = MessageLevel::NORMAL) {
        switch (msgLvl) {
            case MessageLevel::NORMAL:
                std::cout << _f % Color::GREEN << "[INFO]";
                break;
            case MessageLevel::WARNING:
                std::cout << _f % Color::YELLOW << "[WARNING]";
                break;
            case MessageLevel::ERROR:
                std::cout << _f % Color::RED << "[ERROR]";
                break;
            default:
                break;
        }

        std::cout << msg << _def << std::endl;
    }
};

#endif //ifndef ECAT_CONFIG_HPP_INCLUDED
