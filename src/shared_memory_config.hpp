// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// shared_memory_config.hpp
// Single header-only class that covers both the EtherCAT master side
// (create/own shared memory) and the client side (open/read shared memory).
//
// Backward-compatible aliases are provided at the bottom:
//   rocos::EcatConfigMaster  ->  rocos::SharedMemoryConfig
//   rocos::EcatConfig        ->  rocos::SharedMemoryConfig

#pragma once

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <semaphore.h>
#include <sstream>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

// --------------------------------------------------------------------------
// EtherCAT shared-memory constants
// --------------------------------------------------------------------------
#ifndef EC_SHM
#  define EC_SHM          "ecm"
#endif
#ifndef EC_SEM_MUTEX
#  define EC_SEM_MUTEX    "sync"
#endif
#ifndef EC_SEM_NUM
#  define EC_SEM_NUM      10
#endif
#ifndef EC_SHM_MAX_SIZE
#  define EC_SHM_MAX_SIZE 5242880  // 5 MB
#endif

#ifndef MAX_SLAVE_NUM
#  define MAX_SLAVE_NUM      50
#endif
#ifndef MAX_PDINPUT_NUM
#  define MAX_PDINPUT_NUM    25
#endif
#ifndef MAX_PDOUTPUT_NUM
#  define MAX_PDOUTPUT_NUM   25
#endif
#ifndef MAX_PD_NAME_LEN
#  define MAX_PD_NAME_LEN    72
#endif
#ifndef MAX_SLAVE_NAME_LEN
#  define MAX_SLAVE_NAME_LEN 80
#endif

#ifndef ECAT_STATE_INIT
#  define ECAT_STATE_INIT      1
#  define ECAT_STATE_PREOP     2
#  define ECAT_STATE_SAFEOP    4
#  define ECAT_STATE_OP        8
#  define ECAT_STATE_BOOTSTRAP 3
#endif

namespace rocos {

// --------------------------------------------------------------------------
// Data structures shared between master and client
// --------------------------------------------------------------------------
struct PdVar {
    char     name[MAX_PD_NAME_LEN]{'\0'};
    int      offset{-1};
    int      size{-1};
    uint16_t index{0};
    uint8_t  sub_index{0};
};

struct Slave {
    int  id{-1};
    char name[MAX_SLAVE_NAME_LEN]{'\0'};

    int input_var_num{0};
    int output_var_num{0};

    PdVar input_vars[MAX_PDINPUT_NUM];
    PdVar output_vars[MAX_PDOUTPUT_NUM];
};

struct EcatBus {
    long   timestamp{0};

    double min_cycle_time{0.0};
    double max_cycle_time{0.0};
    double avg_cycle_time{0.0};
    double current_cycle_time{0.0};

    /// Control cycle period written by the master (mujoco / real ECM), unit: microseconds.
    /// Example: 1 kHz -> dt = 1000.
    uint32_t dt{0};

    bool   resetCycleTime{false};

    int    current_state{ECAT_STATE_INIT};
    int    request_state{ECAT_STATE_OP};
    int    next_expected_state{};   // internal use

    bool   is_authorized{false};

    int    slave_num{0};
    Slave  slaves[MAX_SLAVE_NUM];
};

// --------------------------------------------------------------------------
// SharedMemoryConfig
//
// Master usage (direct instantiation):
//   SharedMemoryConfig master(id);
//   master.createSharedMemory();
//   master.createPdDataMemoryProvider(inSz, outSz);
//
// Client usage (singleton, auto-connects on first call):
//   SharedMemoryConfig *cfg = SharedMemoryConfig::getInstance(id);
// --------------------------------------------------------------------------
class SharedMemoryConfig {
public:
    // -----------------------------------------------------------------------
    // Construction / Destruction
    // -----------------------------------------------------------------------
    explicit SharedMemoryConfig(int id = 0) {
        ecmName     = EC_SHM      + std::to_string(id);
        mutexName   = EC_SEM_MUTEX + std::to_string(id) + "_";
        pdInputName  = "pd_input"  + std::to_string(id);
        pdOutputName = "pd_output" + std::to_string(id);
    }

    ~SharedMemoryConfig() {
        if (ecatBus != nullptr) {
            if (munmap(ecatBus, ecm_size_) != 0)
                print_message("[SHM] Cannot unmap " + ecmName + ": " + std::strerror(errno), MessageLevel::ERROR);
            ecatBus = nullptr;
        }
        if (pdInputPtr != nullptr) {
            if (munmap(pdInputPtr, pd_input_size_) != 0)
                print_message("[SHM] Cannot unmap " + pdInputName + ": " + std::strerror(errno), MessageLevel::ERROR);
            pdInputPtr = nullptr;
        }
        if (pdOutputPtr != nullptr) {
            if (munmap(pdOutputPtr, pd_output_size_) != 0)
                print_message("[SHM] Cannot unmap " + pdOutputName + ": " + std::strerror(errno), MessageLevel::ERROR);
            pdOutputPtr = nullptr;
        }
        if (ecm_fd_       >= 0) { close(ecm_fd_);       ecm_fd_       = -1; }
        if (pd_input_fd_  >= 0) { close(pd_input_fd_);  pd_input_fd_  = -1; }
        if (pd_output_fd_ >= 0) { close(pd_output_fd_); pd_output_fd_ = -1; }
        for (auto &mutex : sem_mutex) {
            if (mutex != nullptr && mutex != SEM_FAILED) {
                if (sem_close(mutex) != 0)
                    print_message("[SHM] Cannot close semaphore: " + std::string(std::strerror(errno)), MessageLevel::ERROR);
                mutex = nullptr;
            }
        }
    }

    // Non-copyable, non-movable (owns OS resources)
    SharedMemoryConfig(const SharedMemoryConfig &)            = delete;
    SharedMemoryConfig &operator=(const SharedMemoryConfig &) = delete;

    // -----------------------------------------------------------------------
    // Client singleton factory
    // Automatically calls init() (getSharedMemory + getPdDataMemoryProvider).
    // -----------------------------------------------------------------------
    static SharedMemoryConfig *getInstance(int id = 0) {
        static std::unordered_map<int, SharedMemoryConfig *> instances;
        if (instances.find(id) == instances.end()) {
            std::cout << "[SHM] Create new SharedMemoryConfig instance: " << id << std::endl;
            instances[id] = new SharedMemoryConfig(id);
            instances[id]->init();
        }
        return instances[id];
    }

    // -----------------------------------------------------------------------
    // Master-side: create (and own) shared memory objects
    // -----------------------------------------------------------------------
    bool createSharedMemory() {
        mode_t mask = umask(0);

        const std::string shm_name = toPosixName(ecmName);
        shm_unlink(shm_name.c_str());

        ecm_fd_ = shm_open(shm_name.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (ecm_fd_ < 0) {
            print_message("[SHM] Cannot create " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            umask(mask); return false;
        }
        if (ftruncate(ecm_fd_, static_cast<off_t>(ecm_size_)) != 0) {
            print_message("[SHM] Cannot resize " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            close(ecm_fd_); ecm_fd_ = -1; umask(mask); return false;
        }
        void *addr = mmap(nullptr, ecm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, ecm_fd_, 0);
        if (addr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            close(ecm_fd_); ecm_fd_ = -1; umask(mask); return false;
        }
        ecatBus  = static_cast<EcatBus *>(addr);
        *ecatBus = EcatBus{};  // zero-initialise

        for (int i = 0; i < EC_SEM_NUM; i++) {
            std::string semName = mutexName + std::to_string(i);
            sem_mutex[i] = sem_open(semName.c_str(), O_CREAT | O_RDWR, 0777, 1);
            if (sem_mutex[i] == SEM_FAILED) {
                print_message("[SHM] Cannot create semaphore " + semName, MessageLevel::ERROR);
                umask(mask); return false;
            }
            int val = 0;
            sem_getvalue(sem_mutex[i], &val);
            if (val != 1) {
                sem_destroy(sem_mutex[i]);
                sem_unlink(semName.c_str());
                sem_mutex[i] = sem_open(semName.c_str(), O_CREAT | O_RDWR, 0777, 1);
            }
            sem_getvalue(sem_mutex[i], &val);
            if (val != 1) {
                print_message("[SHM] Cannot set semaphore " + semName + " to 1", MessageLevel::ERROR);
                umask(mask); return false;
            }
        }

        umask(mask);
        return true;
    }

    bool createPdDataMemoryProvider(int pdInputSize, int pdOutputSize) {
        const std::string pd_in  = toPosixName(pdInputName);
        const std::string pd_out = toPosixName(pdOutputName);
        shm_unlink(pd_in.c_str());
        shm_unlink(pd_out.c_str());

        pd_input_fd_ = shm_open(pd_in.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (pd_input_fd_ < 0) {
            print_message("[SHM] Cannot create " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            return false;
        }
        if (ftruncate(pd_input_fd_, static_cast<off_t>(pdInputSize)) != 0) {
            print_message("[SHM] Cannot resize " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            close(pd_input_fd_); pd_input_fd_ = -1; return false;
        }
        pdInputPtr = mmap(nullptr, static_cast<std::size_t>(pdInputSize), PROT_READ | PROT_WRITE, MAP_SHARED, pd_input_fd_, 0);
        if (pdInputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            close(pd_input_fd_); pd_input_fd_ = -1; pdInputPtr = nullptr; return false;
        }
        pd_input_size_ = static_cast<std::size_t>(pdInputSize);

        pd_output_fd_ = shm_open(pd_out.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (pd_output_fd_ < 0) {
            print_message("[SHM] Cannot create " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            munmap(pdInputPtr, pd_input_size_); pdInputPtr = nullptr;
            close(pd_input_fd_); pd_input_fd_ = -1; return false;
        }
        if (ftruncate(pd_output_fd_, static_cast<off_t>(pdOutputSize)) != 0) {
            print_message("[SHM] Cannot resize " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            munmap(pdInputPtr, pd_input_size_); pdInputPtr = nullptr;
            close(pd_input_fd_); pd_input_fd_ = -1;
            close(pd_output_fd_); pd_output_fd_ = -1; return false;
        }
        pdOutputPtr = mmap(nullptr, static_cast<std::size_t>(pdOutputSize), PROT_READ | PROT_WRITE, MAP_SHARED, pd_output_fd_, 0);
        if (pdOutputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            munmap(pdInputPtr, pd_input_size_); pdInputPtr = nullptr;
            close(pd_input_fd_); pd_input_fd_ = -1;
            close(pd_output_fd_); pd_output_fd_ = -1; pdOutputPtr = nullptr; return false;
        }
        pd_output_size_ = static_cast<std::size_t>(pdOutputSize);
        return true;
    }

    // Notify all waiting threads/processes that a new cycle is ready
    void updateSempahore() {
        for (auto &sem : sem_mutex) {
            int val = 0;
            sem_getvalue(sem, &val);
            if (val < 1) sem_post(sem);
        }
    }

    // -----------------------------------------------------------------------
    // Common: open existing shared memory (used by both sides)
    // -----------------------------------------------------------------------
    bool getSharedMemory() {
        mode_t mask = umask(0);

        for (int i = 0; i < EC_SEM_NUM; i++) {
            sem_mutex[i] = sem_open((mutexName + std::to_string(i)).c_str(), O_CREAT, 0777, 1);
            if (sem_mutex[i] == SEM_FAILED) {
                print_message("[SHM] Cannot open semaphore " + mutexName + std::to_string(i), MessageLevel::ERROR);
                umask(mask); return false;
            }
        }

        const std::string shm_name = toPosixName(ecmName);
        ecm_fd_ = shm_open(shm_name.c_str(), O_RDWR | O_CREAT, 0666);
        if (ecm_fd_ < 0) {
            print_message("[SHM] Cannot open " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            umask(mask); return false;
        }

        struct stat st{};
        if (fstat(ecm_fd_, &st) != 0) {
            print_message("[SHM] Cannot stat " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            close(ecm_fd_); ecm_fd_ = -1; umask(mask); return false;
        }

        if (st.st_size < static_cast<off_t>(ecm_size_)) {
            print_message("[SHM] Ec-Master is not running.", MessageLevel::WARNING);
            if (ftruncate(ecm_fd_, static_cast<off_t>(ecm_size_)) != 0) {
                print_message("[SHM] Cannot resize " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
                close(ecm_fd_); ecm_fd_ = -1; umask(mask); return false;
            }
        }

        void *addr = mmap(nullptr, ecm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, ecm_fd_, 0);
        if (addr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            close(ecm_fd_); ecm_fd_ = -1; umask(mask); return false;
        }
        ecatBus = static_cast<EcatBus *>(addr);

        umask(mask);
        return true;
    }

    bool getPdDataMemoryProvider() {
        const std::string pd_in  = toPosixName(pdInputName);
        const std::string pd_out = toPosixName(pdOutputName);

        pd_input_fd_ = shm_open(pd_in.c_str(), O_RDWR | O_CREAT, 0666);
        if (pd_input_fd_ < 0) {
            print_message("[SHM] Cannot open " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            return false;
        }
        struct stat st_in{};
        fstat(pd_input_fd_, &st_in);
        if (st_in.st_size == 0) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
            ftruncate(pd_input_fd_, static_cast<off_t>(pd_input_size_));
#pragma GCC diagnostic pop
        } else
            pd_input_size_ = static_cast<std::size_t>(st_in.st_size);

        pdInputPtr = mmap(nullptr, pd_input_size_, PROT_READ | PROT_WRITE, MAP_SHARED, pd_input_fd_, 0);
        if (pdInputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            close(pd_input_fd_); pd_input_fd_ = -1; pdInputPtr = nullptr; return false;
        }

        pd_output_fd_ = shm_open(pd_out.c_str(), O_RDWR | O_CREAT, 0666);
        if (pd_output_fd_ < 0) {
            print_message("[SHM] Cannot open " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            munmap(pdInputPtr, pd_input_size_); pdInputPtr = nullptr;
            close(pd_input_fd_); pd_input_fd_ = -1; return false;
        }
        struct stat st_out{};
        fstat(pd_output_fd_, &st_out);
        if (st_out.st_size == 0) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
            ftruncate(pd_output_fd_, static_cast<off_t>(pd_output_size_));
#pragma GCC diagnostic pop
        } else
            pd_output_size_ = static_cast<std::size_t>(st_out.st_size);

        pdOutputPtr = mmap(nullptr, pd_output_size_, PROT_READ | PROT_WRITE, MAP_SHARED, pd_output_fd_, 0);
        if (pdOutputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            munmap(pdInputPtr, pd_input_size_); pdInputPtr = nullptr;
            close(pd_input_fd_); pd_input_fd_ = -1;
            close(pd_output_fd_); pd_output_fd_ = -1; pdOutputPtr = nullptr; return false;
        }
        return true;
    }

    // Called by getInstance(); opens shm and PD memory for client use.
    void init() {
        if (!getSharedMemory()) {
            print_message("[INIT] Cannot get shared memory.", MessageLevel::ERROR);
            exit(1);
        }
        getPdDataMemoryProvider();
        print_message("[SHM] Shared memory ready.", MessageLevel::NORMAL);
    }

    // -----------------------------------------------------------------------
    // Synchronisation
    // -----------------------------------------------------------------------
    void waitForSignal(int id = 0) {
        sem_wait(sem_mutex[id]);
    }

    void wait() {
        auto id = std::this_thread::get_id();
        auto it = std::find(threadId_.begin(), threadId_.end(), id);
        if (it != threadId_.end()) {
            waitForSignal(static_cast<int>(std::distance(threadId_.begin(), it)));
        } else {
            if (threadId_.size() >= EC_SEM_NUM) {
                print_message("[SHM] Too many threads.", MessageLevel::ERROR);
                return;
            }
            threadId_.push_back(id);
            waitForSignal(static_cast<int>(threadId_.size() - 1));
        }
    }

    // -----------------------------------------------------------------------
    // EcatBus accessors (client helpers)
    // -----------------------------------------------------------------------
    double getBusMinCycleTime()     const { return ecatBus->min_cycle_time;     }
    double getBusMaxCycleTime()     const { return ecatBus->max_cycle_time;     }
    double getBusAvgCycleTime()     const { return ecatBus->avg_cycle_time;     }
    double getBusCurrentCycleTime() const { return ecatBus->current_cycle_time; }
    /// @brief Control cycle period [us] from shared memory (master writes, app reads)
    uint32_t getDt()                const { return ecatBus->dt;                 }
    bool   isAuthorized()           const { return ecatBus->is_authorized;      }
    long   getTimestamp()           const { return ecatBus->timestamp;          }
    int    getSlaveNum()            const { return ecatBus->slave_num;          }
    int    getBusCurrentState()     const { return ecatBus->current_state;      }

    void resetCycleTime()                { ecatBus->resetCycleTime = true;   }
    void setBusRequestState(int state)   { ecatBus->request_state  = state;  }

    std::string getSlaveName(int slaveId) {
        return ecatBus->slaves[slaveId].name;
    }
    Slave getSlave(int slaveId) {
        return ecatBus->slaves[slaveId];
    }
    Slave findSlaveByName(const std::string &name) {
        for (int i = 0; i < ecatBus->slave_num; ++i)
            if (std::string(ecatBus->slaves[i].name) == name)
                return ecatBus->slaves[i];
        return {};
    }
    int findSlaveIdByName(const std::string &name) {
        for (int i = 0; i < ecatBus->slave_num; ++i)
            if (std::string(ecatBus->slaves[i].name) == name)
                return i;
        return -1;
    }
    std::string getInputVarName(int slaveId, int varId)  const { return ecatBus->slaves[slaveId].input_vars[varId].name;  }
    std::string getOutputVarName(int slaveId, int varId) const { return ecatBus->slaves[slaveId].output_vars[varId].name; }
    PdVar getSlaveInputVar(int slaveId, int varId)  { return ecatBus->slaves[slaveId].input_vars[varId];  }
    PdVar getSlaveOutputVar(int slaveId, int varId) { return ecatBus->slaves[slaveId].output_vars[varId]; }
    PdVar findSlaveInputVarByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (std::string(ecatBus->slaves[slaveId].input_vars[i].name) == name)
                return ecatBus->slaves[slaveId].input_vars[i];
        return {};
    }
    int findSlaveInputVarIdByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (std::string(ecatBus->slaves[slaveId].input_vars[i].name) == name)
                return i;
        return -1;
    }

    // -----------------------------------------------------------------------
    // Template PD access methods (by index)
    // -----------------------------------------------------------------------
    template<typename T> T getSlaveInputVarValue(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset);
    }
    template<typename T> void setSlaveInputVarValue(int slaveId, int varId, T value) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset) = value;
    }
    template<typename T> T getSlaveOutputVarValue(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset);
    }
    template<typename T> void setSlaveOutputVarValue(int slaveId, int varId, T value) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset) = value;
    }

    // Template PD access methods (by name)
    template<typename T> T getSlaveInputVarValueByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset);
            }
        return std::numeric_limits<T>::max();
    }
    template<typename T> void setSlaveInputVarValueByName(int slaveId, const std::string &name, T value) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset) = value;
            }
    }
    template<typename T> T getSlaveOutputVarValueByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset);
            }
        return std::numeric_limits<T>::max();
    }
    template<typename T> void setSlaveOutputVarValueByName(int slaveId, const std::string &name, T value) {
        for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset) = value;
            }
    }

    // Template PD pointer methods
    template<typename T> T *getSlaveInputVarPtr(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return (T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset);
    }
    template<typename T> T *getSlaveOutputVarPtr(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return (T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset);
    }
    template<typename T> T *findSlaveInputVarPtrByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return (T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset);
            }
        return nullptr;
    }
    template<typename T> T *findSlaveOutputVarPtrByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return (T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset);
            }
        return nullptr;
    }

    std::string to_string() { return std::string{}; }

    // -----------------------------------------------------------------------
    // Public data (accessible by both sides for direct struct manipulation)
    // -----------------------------------------------------------------------
    EcatBus *ecatBus    = nullptr;
    void    *pdInputPtr  = nullptr;
    void    *pdOutputPtr = nullptr;
    sem_t   *sem_mutex[EC_SEM_NUM]{};

    int         ecm_fd_{-1};
    int         pd_input_fd_{-1};
    int         pd_output_fd_{-1};
    std::size_t ecm_size_{EC_SHM_MAX_SIZE};
    std::size_t pd_input_size_{EC_SHM_MAX_SIZE};
    std::size_t pd_output_size_{EC_SHM_MAX_SIZE};

private:
    std::string ecmName{EC_SHM};
    std::string mutexName{EC_SEM_MUTEX};
    std::string pdInputName{"pd_input"};
    std::string pdOutputName{"pd_output"};

    std::vector<std::thread::id> threadId_;

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    static std::string toPosixName(const std::string &name) {
        return (name.front() == '/') ? name : "/" + name;
    }

    enum class MessageLevel { NORMAL, WARNING, ERROR };

    void print_message(const std::string &msg, MessageLevel lvl) const {
        switch (lvl) {
            case MessageLevel::NORMAL:  std::cout << "\033[1;32m [INFO]";    break;
            case MessageLevel::WARNING: std::cout << "\033[1;33m [WARNING]"; break;
            case MessageLevel::ERROR:   std::cout << "\033[1;31m [ERROR]";   break;
        }
        std::cout << msg << "\033[0m" << std::endl;
    }
};

// --------------------------------------------------------------------------
// Backward-compatible type aliases
// --------------------------------------------------------------------------
using EcatConfigMaster = SharedMemoryConfig;
using EcatConfig       = SharedMemoryConfig;

}  // namespace rocos
