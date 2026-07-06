//
// Created by think on 2023/12/11.
//

#include "ecat_config.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace rocos;

namespace {
std::string toPosixSharedMemoryName(const std::string &name) {
    if (!name.empty() && name.front() == '/') {
        return name;
    }

    return "/" + name;
}
}  // namespace

EcatConfig::EcatConfig(int id) {
    ecmName = EC_SHM + std::to_string(id);
    mutexName = EC_SEM_MUTEX + std::to_string(id) + "_";
    pdInputName = "pd_input" + std::to_string(id);
    pdOutputName = "pd_output" + std::to_string(id);

    init();
}

EcatConfig::~EcatConfig() {
    if (ecatBus != nullptr) {
        if (munmap(ecatBus, ecm_size_) != 0) {
            print_message("[SHM] Can not unmap shared memory " + ecmName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        ecatBus = nullptr;
    }
    if (pdInputPtr != nullptr) {
        if (munmap(pdInputPtr, pd_input_size_) != 0) {
            print_message("[SHM] Can not unmap shared memory " + pdInputName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        pdInputPtr = nullptr;
    }
    if (pdOutputPtr != nullptr) {
        if (munmap(pdOutputPtr, pd_output_size_) != 0) {
            print_message("[SHM] Can not unmap shared memory " + pdOutputName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        pdOutputPtr = nullptr;
    }

    if (ecm_fd_ >= 0) {
        if (close(ecm_fd_) != 0) {
            print_message("[SHM] Can not close shared memory " + ecmName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        ecm_fd_ = -1;
    }
    if (pd_input_fd_ >= 0) {
        if (close(pd_input_fd_) != 0) {
            print_message("[SHM] Can not close shared memory " + pdInputName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        pd_input_fd_ = -1;
    }
    if (pd_output_fd_ >= 0) {
        if (close(pd_output_fd_) != 0) {
            print_message("[SHM] Can not close shared memory " + pdOutputName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        pd_output_fd_ = -1;
    }

    for (auto &mutex : sem_mutex) {
        if (mutex != nullptr && mutex != SEM_FAILED) {
            if (sem_close(mutex) != 0) {
                print_message("[SHM] Can not close semaphore mutex: " +
                                  std::string(std::strerror(errno)),
                              MessageLevel::ERROR);
            }
            mutex = nullptr;
        }
    }
}

bool EcatConfig::getSharedMemory() {
    mode_t mask = umask(0); // 取消屏蔽的权限位

    bool created = false;
    void *ecm_ptr = nullptr;
    if (!mapSharedMemory(ecmName, ecm_size_, &ecm_fd_, &ecm_ptr, &created)) {
        umask(mask);
        return false;
    }

    ecatBus = static_cast<EcatBus *>(ecm_ptr);
    if (created) {
        print_message("[SHM] Ec-Master is not running.", MessageLevel::WARNING);
        *ecatBus = EcatBus{};
    }

    for (int i = 0; i < EC_SEM_NUM; i++) {
        sem_mutex[i] = sem_open((mutexName + std::to_string(i)).c_str(), O_CREAT, 0777, 1);
        if (sem_mutex[i] == SEM_FAILED) {
            print_message("[SHM] Can not open or create semaphore mutex " + mutexName + std::to_string(i) + ".",
                          MessageLevel::ERROR);
            umask(mask);
            return false;
        }
    }

    umask(mask); // 恢复umask的值

    return true;
}

bool EcatConfig::getPdDataMemoryProvider() {
    bool created = false;
    if (!mapSharedMemory(pdInputName, pd_input_size_, &pd_input_fd_,
                         &pdInputPtr, &created)) {
        return false;
    }

    if (!mapSharedMemory(pdOutputName, pd_output_size_, &pd_output_fd_,
                         &pdOutputPtr, &created)) {
        if (munmap(pdInputPtr, pd_input_size_) != 0) {
            print_message("[SHM] Can not unmap shared memory " + pdInputName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        pdInputPtr = nullptr;
        if (close(pd_input_fd_) != 0) {
            print_message("[SHM] Can not close shared memory " + pdInputName +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        pd_input_fd_ = -1;
        return false;
    }

    return true;
}

bool EcatConfig::mapSharedMemory(const std::string &name, std::size_t size,
                                 int *fd, void **ptr, bool *created) {
    if (fd == nullptr || ptr == nullptr || created == nullptr) {
        print_message("[SHM] Invalid shared memory output parameter.",
                      MessageLevel::ERROR);
        return false;
    }

    const std::string shm_name = toPosixSharedMemoryName(name);
    const int shm_fd = shm_open(shm_name.c_str(), O_RDWR | O_CREAT, 0666);
    if (shm_fd < 0) {
        print_message("[SHM] Can not open shared memory " + shm_name + ": " +
                          std::strerror(errno),
                      MessageLevel::ERROR);
        return false;
    }

    struct stat shm_stat {};
    if (fstat(shm_fd, &shm_stat) != 0) {
        print_message("[SHM] Can not stat shared memory " + shm_name + ": " +
                          std::strerror(errno),
                      MessageLevel::ERROR);
        if (close(shm_fd) != 0) {
            print_message("[SHM] Can not close shared memory " + shm_name +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        return false;
    }

    *created = shm_stat.st_size == 0;
    if (shm_stat.st_size < static_cast<off_t>(size)) {
        if (ftruncate(shm_fd, static_cast<off_t>(size)) != 0) {
            print_message("[SHM] Can not resize shared memory " + shm_name +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
            if (close(shm_fd) != 0) {
                print_message("[SHM] Can not close shared memory " + shm_name +
                                  ": " + std::strerror(errno),
                              MessageLevel::ERROR);
            }
            return false;
        }
    }

    void *address = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED,
                         shm_fd, 0);
    if (address == MAP_FAILED) {
        print_message("[SHM] Can not map shared memory " + shm_name + ": " +
                          std::strerror(errno),
                      MessageLevel::ERROR);
        if (close(shm_fd) != 0) {
            print_message("[SHM] Can not close shared memory " + shm_name +
                              ": " + std::strerror(errno),
                          MessageLevel::ERROR);
        }
        return false;
    }

    *fd = shm_fd;
    *ptr = address;
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
