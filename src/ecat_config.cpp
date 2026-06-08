//
// Created by think on 2023/12/11.
//

#include <rocos_app/ethercat/ecat_config.h>
#include <algorithm>
#include <iostream>


using namespace rocos;

EcatConfig::EcatConfig(int id) {
    ecmName = EC_SHM + std::to_string(id);
    mutexName = EC_SEM_MUTEX + std::to_string(id) + "_";
    pdInputName = "pd_input" + std::to_string(id);
    pdOutputName = "pd_output" + std::to_string(id);

    init();
}

EcatConfig::~EcatConfig() {

}

bool EcatConfig::getSharedMemory() {
    mode_t mask = umask(0); // 取消屏蔽的权限位


    getPdDataMemoryProvider();


    using namespace boost::interprocess;
    managedSharedMemory = new managed_shared_memory{open_or_create, ecmName.c_str(), EC_SHM_MAX_SIZE};
//    managedSharedMemory = new managed_shared_memory{open_only, EC_SHM};

    std::pair<EcatBus *, std::size_t> p1 = managedSharedMemory->find<EcatBus>("ecat");
    if (p1.first) {
        ecatBus = p1.first;
    } else {
        print_message("[SHM] Ec-Master is not running.", MessageLevel::WARNING);
        ecatBus = managedSharedMemory->construct<EcatBus>("ecat")();
    }

    for (int i = 0; i < EC_SEM_NUM; i++) {
        sem_mutex[i] = sem_open((mutexName + std::to_string(i)).c_str(), O_CREAT, 0777, 1);
        if (sem_mutex[i] == SEM_FAILED) {
            print_message("[SHM] Can not open or create semaphore mutex " + mutexName + std::to_string(i) + ".",
                          MessageLevel::ERROR);
            return false;
        }
    }

    umask(mask); // 恢复umask的值

    return true;
}

bool EcatConfig::getPdDataMemoryProvider() {
    using namespace boost::interprocess;

    pdInputShm = new shared_memory_object(open_or_create, pdInputName.c_str(), read_write);
    pdOutputShm = new shared_memory_object(open_or_create, pdOutputName.c_str(), read_write);

    pdInputRegion = new mapped_region(*pdInputShm, read_write);
    pdOutputRegion = new mapped_region(*pdOutputShm, read_write);


    pdInputPtr = static_cast<char *>(pdInputRegion->get_address());
    pdOutputPtr = static_cast<char *>(pdOutputRegion->get_address());

    return true;
}

void EcatConfig::waitForSignal(int id) {
    sem_wait(sem_mutex[id]);
}

void EcatConfig::wait() {
    auto id = std::this_thread::get_id();
    auto it = std::find(threadId.begin(), threadId.end(), id);
    if(it != threadId.end()) { // thread is already in the list
        waitForSignal(std::distance(threadId.begin(), it));
    }
    else {
        if(threadId.size() >= EC_SEM_NUM) {
            print_message("[SHM] Too many threads.", MessageLevel::ERROR);
            return;
        }
        threadId.push_back(id);
        waitForSignal(threadId.size() - 1);
    }
}

void EcatConfig::init() {
    if (!getSharedMemory()) {
        print_message("[INIT] Can not get shared memory.", MessageLevel::ERROR);
        exit(1);
    }

    getPdDataMemoryProvider();
}

void EcatConfig::print_message(const std::string &msg, EcatConfig::MessageLevel msgLvl) {
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


double EcatConfig::getBusMinCycleTime() const {
    return ecatBus->min_cycle_time;
}

double EcatConfig::getBusMaxCycleTime() const {
    return ecatBus->max_cycle_time;
}

double EcatConfig::getBusAvgCycleTime() const {
    return ecatBus->avg_cycle_time;
}

double EcatConfig::getBusCurrentCycleTime() const {
    return ecatBus->current_cycle_time;
}

bool EcatConfig::isAuthorized() const {
    return ecatBus->is_authorized;
}

long EcatConfig::getTimestamp() const {
    return ecatBus->timestamp;
}

int EcatConfig::getSlaveNum() const {
    return ecatBus->slave_num;
}

std::string EcatConfig::getSlaveName(int slaveId) {
    return ecatBus->slaves[slaveId].name;
}

Slave EcatConfig::getSlave(int slaveId) {
    return ecatBus->slaves[slaveId];
}

Slave EcatConfig::findSlaveByName(const std::string &slaveName) {
    for (int i = 0; i < ecatBus->slave_num; ++i) {
        if(ecatBus->slaves[i].name == slaveName.c_str()) {
            return ecatBus->slaves[i];
        }
    }

    return {};
}

int EcatConfig::findSlaveIdByName(const std::string &slaveName) {
    for (int i = 0; i < ecatBus->slave_num; ++i) {
        if(ecatBus->slaves[i].name == slaveName.c_str()) {
            return i;
        }
    }

    return -1;
}

std::string EcatConfig::getInputVarName(int slaveId, int varId) const {
    return ecatBus->slaves[slaveId].input_vars[varId].name;
}

std::string EcatConfig::getOutputVarName(int slaveId, int varId) const {
    return ecatBus->slaves[slaveId].output_vars[varId].name;
}

PdVar EcatConfig::getSlaveOutputVar(int slaveId, int varId) {
    return ecatBus->slaves[slaveId].output_vars[varId];
}


PdVar EcatConfig::getSlaveInputVar(int slaveId, int varId) {
    return ecatBus->slaves[slaveId].input_vars[varId];
}

PdVar EcatConfig::findSlaveInputVarByName(int slaveId, const std::string &varName) {
    for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i) {
        if(ecatBus->slaves[slaveId].input_vars[i].name == varName.c_str()) {
            return ecatBus->slaves[slaveId].input_vars[i];
        }
    }

    return {};
}

int EcatConfig::findSlaveInputVarIdByName(int slaveId, const std::string &varName) {
    for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i) {
        if(ecatBus->slaves[slaveId].input_vars[i].name == varName.c_str()) {
            return i;
        }
    }

    return -1;
}

void EcatConfig::resetCycleTime() {
    ecatBus->resetCycleTime = true;
}

void EcatConfig::setBusRequestState(int state) {
    ecatBus->request_state = state;
}

int EcatConfig::getBusCurrentState() const {
    return ecatBus->current_state;
}

EcatConfig *EcatConfig::getInstance(int id) {
    if(instances.find(id) == instances.end()) {
        std::cout << "Create New Ecat Config Instance: " << id << std::endl;
        instances[id] = new EcatConfig(id);
    }

    return instances[id];
}


std::map<int, EcatConfig*> EcatConfig::instances;

